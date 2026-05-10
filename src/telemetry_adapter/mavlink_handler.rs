use core::fmt;

use heapless::{Deque, Vec, spsc::Queue};
use mavlink::{
    MavConnection, MavHeader, connect,
    dialects::common::{
        GLOBAL_POSITION_INT_DATA, GPS_RAW_INT_DATA, GpsFixType, HIGHRES_IMU_DATA, MavMessage,
    },
    error::{MessageReadError, MessageWriteError},
};
use nalgebra::Vector3;

use crate::{
    ekf_core::state::{EskfState, ImuSample},
    statistical_monitor::observation::GpsObservation,
    telemetry_adapter::conversions::{
        ConversionError, GeodeticPosition, HomePosition,
        centimetres_per_second_to_metres_per_second, geodetic_to_ned, microseconds_to_seconds,
        millimetres_to_metres, milliseconds_to_seconds, scaled_degrees_e7_to_radians,
    },
};

pub const DEFAULT_MAVLINK_UDP_ADDRESS: &str = "udpin:127.0.0.1:14550";
pub const MAX_MAVLINK_FRAME_BYTES: usize = 280;
const GPS_QUEUE_CAPACITY: usize = 32;
const STATE_HISTORY_CAPACITY: usize = 64;
const EPOCH_TIME_THRESHOLD_USEC: u64 = 1_000_000_000_000;

pub type MavlinkFrameBuffer = Vec<u8, MAX_MAVLINK_FRAME_BYTES>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsNoiseModel {
    pub user_equivalent_range_error_m: f32,
    pub min_horizontal_position_std_m: f32,
    pub min_vertical_position_std_m: f32,
    pub min_horizontal_velocity_std_mps: f32,
    pub min_vertical_velocity_std_mps: f32,
    pub velocity_std_from_position_scale_hz: f32,
    pub fallback_hdop: f32,
    pub fallback_vdop: f32,
}

impl Default for GpsNoiseModel {
    fn default() -> Self {
        Self {
            user_equivalent_range_error_m: 5.0,
            min_horizontal_position_std_m: 1.5,
            min_vertical_position_std_m: 2.5,
            min_horizontal_velocity_std_mps: 0.35,
            min_vertical_velocity_std_mps: 0.50,
            velocity_std_from_position_scale_hz: 0.25,
            fallback_hdop: 3.0,
            fallback_vdop: 4.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MavlinkSubscriberConfig {
    pub gps_noise_model: GpsNoiseModel,
}

impl Default for MavlinkSubscriberConfig {
    fn default() -> Self {
        Self {
            gps_noise_model: GpsNoiseModel::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum TelemetryUpdate {
    Imu {
        sample: ImuSample,
        raw_frame: MavlinkFrameBuffer,
    },
    GpsObservationQueued {
        timestamp_s: f64,
        queue_depth: usize,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub struct SynchronizedGpsSample {
    pub timestamp_ns: u64,
    pub gps_observation: GpsObservation,
    pub aligned_predicted_state: EskfState,
    pub raw_frame: MavlinkFrameBuffer,
}

#[derive(Clone, Debug, PartialEq)]
struct PendingGpsObservation {
    timestamp_ns: u64,
    gps_observation: GpsObservation,
    raw_frame: MavlinkFrameBuffer,
}

pub struct MavlinkSubscriber {
    connection: mavlink::Connection<MavMessage>,
    config: MavlinkSubscriberConfig,
    time_normalizer: TimestampNormalizer,
    home_position: Option<HomePosition>,
    latest_gps_quality: Option<GpsQualityMetrics>,
    pending_gps_observations: Queue<PendingGpsObservation, GPS_QUEUE_CAPACITY>,
    state_history: Deque<EskfState, STATE_HISTORY_CAPACITY>,
}

impl MavlinkSubscriber {
    pub fn bind_default() -> Result<Self, TelemetryError> {
        Self::bind(DEFAULT_MAVLINK_UDP_ADDRESS)
    }

    pub fn bind(address: &str) -> Result<Self, TelemetryError> {
        Self::with_config(address, MavlinkSubscriberConfig::default())
    }

    pub fn with_config(
        address: &str,
        config: MavlinkSubscriberConfig,
    ) -> Result<Self, TelemetryError> {
        let connection = connect::<MavMessage>(address).map_err(TelemetryError::ConnectionError)?;

        Ok(Self {
            connection,
            config,
            time_normalizer: TimestampNormalizer::default(),
            home_position: None,
            latest_gps_quality: None,
            pending_gps_observations: Queue::new(),
            state_history: Deque::new(),
        })
    }

    pub fn home_position(&self) -> Option<HomePosition> {
        self.home_position
    }

    pub fn record_predicted_state(&mut self, state: &EskfState) {
        if self.state_history.is_full() {
            let _ = self.state_history.pop_front();
        }

        self.state_history
            .push_back(state.clone())
            .expect("state history push must succeed after making room");
    }

    pub fn recv_next(&mut self) -> Result<TelemetryUpdate, TelemetryError> {
        loop {
            let (header, message) = self
                .connection
                .recv()
                .map_err(TelemetryError::MavlinkReadError)?;

            match message {
                MavMessage::HIGHRES_IMU(data) => {
                    let raw_frame = encode_mavlink_frame(
                        header,
                        &MavMessage::HIGHRES_IMU(data.clone()),
                    )
                        .map_err(TelemetryError::FrameEncodingError)?;
                    let imu_sample = self.parse_highres_imu(data);
                    return Ok(TelemetryUpdate::Imu {
                        sample: imu_sample,
                        raw_frame,
                    });
                }
                MavMessage::GPS_RAW_INT(data) => {
                    let mut raw_frame = encode_mavlink_frame(
                        header,
                        &MavMessage::GPS_RAW_INT(data.clone()),
                    )
                    .map_err(TelemetryError::FrameEncodingError)?;
                    self.update_gps_quality(data);
                    purge_frame_buffer(&mut raw_frame);
                }
                MavMessage::GLOBAL_POSITION_INT(data) => {
                    let raw_frame = encode_mavlink_frame(
                        header,
                        &MavMessage::GLOBAL_POSITION_INT(data.clone()),
                    )
                    .map_err(TelemetryError::FrameEncodingError)?;
                    if let Some(pending_observation) = self.try_build_gps_observation(data, raw_frame)? {
                        let timestamp_s = pending_observation.gps_observation.timestamp_s;
                        self.pending_gps_observations
                            .enqueue(pending_observation)
                            .map_err(|_| TelemetryError::BufferOverflow)?;
                        return Ok(TelemetryUpdate::GpsObservationQueued {
                            timestamp_s,
                            queue_depth: self.pending_gps_observations.len(),
                        });
                    }
                }
                _ => {}
            }
        }
    }

    pub fn try_dequeue_synchronized_gps(
        &mut self,
    ) -> Result<Option<SynchronizedGpsSample>, TelemetryError> {
        let Some(oldest_observation) = self.pending_gps_observations.peek().cloned() else {
            return Ok(None);
        };

        let aligned_state = match self.interpolate_state_at(oldest_observation.gps_observation.timestamp_s)? {
            Some(aligned_state) => aligned_state,
            None => return Ok(None),
        };

        let pending_gps_observation = self
            .pending_gps_observations
            .dequeue()
            .expect("pending observation must still be available after peek");

        Ok(Some(SynchronizedGpsSample {
            timestamp_ns: pending_gps_observation.timestamp_ns,
            gps_observation: pending_gps_observation.gps_observation,
            aligned_predicted_state: aligned_state,
            raw_frame: pending_gps_observation.raw_frame,
        }))
    }

    fn parse_highres_imu(&mut self, message: HIGHRES_IMU_DATA) -> ImuSample {
        let timestamp_s = self.time_normalizer.normalize_time_usec(message.time_usec);
        // PX4 SITL publishes HIGHRES_IMU in the FRD body convention, which matches the
        // EKF's body axes for a NED navigation frame.
        let accel_body_mps2 = Vector3::new(message.xacc, message.yacc, message.zacc);
        let gyro_body_rps = Vector3::new(message.xgyro, message.ygyro, message.zgyro);

        ImuSample::new(timestamp_s, accel_body_mps2, gyro_body_rps)
    }

    fn update_gps_quality(&mut self, message: GPS_RAW_INT_DATA) {
        let time_seconds = self.time_normalizer.peek_time_usec(message.time_usec);
        self.latest_gps_quality = Some(GpsQualityMetrics::from_message(message, time_seconds));
    }

    fn try_build_gps_observation(
        &mut self,
        message: GLOBAL_POSITION_INT_DATA,
        raw_frame: MavlinkFrameBuffer,
    ) -> Result<Option<PendingGpsObservation>, TelemetryError> {
        let gps_quality = match self.latest_gps_quality {
            Some(gps_quality) if gps_quality.has_valid_position_fix() => gps_quality,
            _ => return Ok(None),
        };

        let position_geodetic = GeodeticPosition::new(
            scaled_degrees_e7_to_radians(message.lat)
                .map_err(TelemetryError::InvalidUnitScaling)?,
            scaled_degrees_e7_to_radians(message.lon)
                .map_err(TelemetryError::InvalidUnitScaling)?,
            f64::from(
                millimetres_to_metres(message.alt).map_err(TelemetryError::InvalidUnitScaling)?,
            ),
        )
        .map_err(TelemetryError::InvalidUnitScaling)?;

        let timestamp_s = milliseconds_to_seconds(message.time_boot_ms);
        self.time_normalizer
            .update_boot_epoch_offset(gps_quality.timestamp_s, timestamp_s);

        if self.home_position.is_none() {
            self.home_position = Some(HomePosition::new(position_geodetic));
        }

        let home_position = self
            .home_position
            .ok_or(TelemetryError::HomePositionUnavailable)?;
        let position_ned_m = geodetic_to_ned(home_position, position_geodetic)
            .map_err(TelemetryError::InvalidUnitScaling)?;

        let velocity_ned_mps = Vector3::new(
            centimetres_per_second_to_metres_per_second(message.vx),
            centimetres_per_second_to_metres_per_second(message.vy),
            centimetres_per_second_to_metres_per_second(message.vz),
        );

        let (horizontal_position_std_m, vertical_position_std_m) =
            gps_quality.position_standard_deviations(self.config.gps_noise_model);
        let (horizontal_velocity_std_mps, vertical_velocity_std_mps) = gps_quality
            .velocity_standard_deviations(
                self.config.gps_noise_model,
                horizontal_position_std_m,
                vertical_position_std_m,
            );

        let gps_observation = GpsObservation::from_accuracy_metrics(
            timestamp_s,
            position_ned_m,
            velocity_ned_mps,
            horizontal_position_std_m,
            vertical_position_std_m,
            horizontal_velocity_std_mps,
            vertical_velocity_std_mps,
        );
        let timestamp_ns = gps_quality
            .absolute_timestamp_ns
            .unwrap_or_else(|| u64::from(message.time_boot_ms) * 1_000_000);

        Ok(Some(PendingGpsObservation {
            timestamp_ns,
            gps_observation,
            raw_frame,
        }))
    }

    fn interpolate_state_at(
        &mut self,
        target_timestamp_s: f64,
    ) -> Result<Option<EskfState>, TelemetryError> {
        if self.state_history.is_empty() {
            return Ok(None);
        }

        let oldest_timestamp_s = self
            .state_history
            .front()
            .map(|state| state.nominal.timestamp_s)
            .expect("non-empty state history must have a front element");
        if target_timestamp_s < oldest_timestamp_s {
            let _ = self.pending_gps_observations.dequeue();
            return Err(TelemetryError::StateHistoryUnderflow {
                target_timestamp_s,
                oldest_timestamp_s,
            });
        }

        let latest_timestamp_s = self
            .state_history
            .back()
            .map(|state| state.nominal.timestamp_s)
            .expect("non-empty state history must have a back element");
        if target_timestamp_s > latest_timestamp_s {
            return Ok(None);
        }

        let mut previous_state: Option<&EskfState> = None;
        for state in self.state_history.iter() {
            if state.nominal.timestamp_s >= target_timestamp_s {
                return match previous_state {
                    None => Ok(Some(state.clone())),
                    Some(previous_state) => {
                        if (state.nominal.timestamp_s - target_timestamp_s).abs() <= f64::EPSILON {
                            Ok(Some(state.clone()))
                        } else if (target_timestamp_s - previous_state.nominal.timestamp_s).abs()
                            <= f64::EPSILON
                        {
                            Ok(Some(previous_state.clone()))
                        } else {
                            Ok(Some(interpolate_eskf_state(
                                previous_state,
                                state,
                                target_timestamp_s,
                            )))
                        }
                    }
                };
            }

            previous_state = Some(state);
        }

        Ok(None)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GpsQualityMetrics {
    timestamp_s: f64,
    absolute_timestamp_ns: Option<u64>,
    fix_type: GpsFixType,
    hdop: f32,
    vdop: f32,
    satellites_visible: u8,
}

impl GpsQualityMetrics {
    fn from_message(message: GPS_RAW_INT_DATA, timestamp_s: f64) -> Self {
        Self {
            timestamp_s,
            absolute_timestamp_ns: if message.time_usec >= EPOCH_TIME_THRESHOLD_USEC {
                message.time_usec.checked_mul(1_000)
            } else {
                None
            },
            fix_type: message.fix_type,
            hdop: if message.eph == u16::MAX {
                GpsNoiseModel::default().fallback_hdop
            } else {
                f32::from(message.eph) * 0.01
            },
            vdop: if message.epv == u16::MAX {
                GpsNoiseModel::default().fallback_vdop
            } else {
                f32::from(message.epv) * 0.01
            },
            satellites_visible: message.satellites_visible,
        }
    }

    fn has_valid_position_fix(&self) -> bool {
        matches!(
            self.fix_type,
            GpsFixType::GPS_FIX_TYPE_3D_FIX
                | GpsFixType::GPS_FIX_TYPE_DGPS
                | GpsFixType::GPS_FIX_TYPE_RTK_FLOAT
                | GpsFixType::GPS_FIX_TYPE_RTK_FIXED
                | GpsFixType::GPS_FIX_TYPE_STATIC
                | GpsFixType::GPS_FIX_TYPE_PPP
        )
    }

    fn position_standard_deviations(&self, noise_model: GpsNoiseModel) -> (f32, f32) {
        let fix_scale = self.fix_type_scale();
        let horizontal_position_std_m =
            (self.hdop * noise_model.user_equivalent_range_error_m * fix_scale)
                .max(noise_model.min_horizontal_position_std_m);
        let vertical_position_std_m =
            (self.vdop * noise_model.user_equivalent_range_error_m * fix_scale)
                .max(noise_model.min_vertical_position_std_m);

        (horizontal_position_std_m, vertical_position_std_m)
    }

    fn velocity_standard_deviations(
        &self,
        noise_model: GpsNoiseModel,
        horizontal_position_std_m: f32,
        vertical_position_std_m: f32,
    ) -> (f32, f32) {
        let satellite_scale = if self.satellites_visible >= 10 {
            1.0
        } else if self.satellites_visible >= 7 {
            1.15
        } else {
            1.35
        };

        let horizontal_velocity_std_mps = (horizontal_position_std_m
            * noise_model.velocity_std_from_position_scale_hz
            * satellite_scale)
            .max(noise_model.min_horizontal_velocity_std_mps);
        let vertical_velocity_std_mps = (vertical_position_std_m
            * noise_model.velocity_std_from_position_scale_hz
            * satellite_scale)
            .max(noise_model.min_vertical_velocity_std_mps);

        (horizontal_velocity_std_mps, vertical_velocity_std_mps)
    }

    fn fix_type_scale(&self) -> f32 {
        match self.fix_type {
            GpsFixType::GPS_FIX_TYPE_RTK_FIXED => 0.20,
            GpsFixType::GPS_FIX_TYPE_RTK_FLOAT => 0.35,
            GpsFixType::GPS_FIX_TYPE_DGPS | GpsFixType::GPS_FIX_TYPE_PPP => 0.60,
            GpsFixType::GPS_FIX_TYPE_STATIC => 0.25,
            GpsFixType::GPS_FIX_TYPE_3D_FIX => 1.00,
            GpsFixType::GPS_FIX_TYPE_2D_FIX => 1.50,
            GpsFixType::GPS_FIX_TYPE_NO_FIX | GpsFixType::GPS_FIX_TYPE_NO_GPS => 10.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct TimestampNormalizer {
    epoch_minus_boot_s: Option<f64>,
    first_epoch_timestamp_s: Option<f64>,
}

impl TimestampNormalizer {
    fn normalize_time_usec(&mut self, time_usec: u64) -> f64 {
        let time_s = microseconds_to_seconds(time_usec);
        if time_usec >= EPOCH_TIME_THRESHOLD_USEC {
            if let Some(epoch_minus_boot_s) = self.epoch_minus_boot_s {
                return time_s - epoch_minus_boot_s;
            }

            let origin = self.first_epoch_timestamp_s.get_or_insert(time_s);
            time_s - *origin
        } else {
            time_s
        }
    }

    fn peek_time_usec(&self, time_usec: u64) -> f64 {
        let time_s = microseconds_to_seconds(time_usec);
        if time_usec >= EPOCH_TIME_THRESHOLD_USEC {
            if let Some(epoch_minus_boot_s) = self.epoch_minus_boot_s {
                return time_s - epoch_minus_boot_s;
            }

            if let Some(origin) = self.first_epoch_timestamp_s {
                return time_s - origin;
            }
        }

        time_s
    }

    fn update_boot_epoch_offset(&mut self, gps_raw_timestamp_s: f64, gps_boot_timestamp_s: f64) {
        if gps_raw_timestamp_s > 60.0 * 60.0 * 24.0 * 365.0 {
            self.epoch_minus_boot_s = Some(gps_raw_timestamp_s - gps_boot_timestamp_s);
        }
    }
}

#[derive(Debug)]
pub enum TelemetryError {
    ConnectionError(std::io::Error),
    MavlinkReadError(MessageReadError),
    FrameEncodingError(MessageWriteError),
    InvalidUnitScaling(ConversionError),
    BufferOverflow,
    HomePositionUnavailable,
    StateHistoryUnderflow {
        target_timestamp_s: f64,
        oldest_timestamp_s: f64,
    },
}

impl fmt::Display for TelemetryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ConnectionError(error) => write!(f, "failed to open MAVLink connection: {error}"),
            Self::MavlinkReadError(error) => write!(f, "failed to read MAVLink frame: {error}"),
            Self::FrameEncodingError(error) => {
                write!(f, "failed to encode MAVLink frame bytes for evidence hashing: {error}")
            }
            Self::InvalidUnitScaling(error) => {
                write!(
                    f,
                    "invalid telemetry unit scaling or coordinate projection: {error:?}"
                )
            }
            Self::BufferOverflow => {
                write!(f, "GPS observation queue overflowed its 32-slot capacity")
            }
            Self::HomePositionUnavailable => {
                write!(
                    f,
                    "home position was required before projecting GPS into local NED"
                )
            }
            Self::StateHistoryUnderflow {
                target_timestamp_s,
                oldest_timestamp_s,
            } => write!(
                f,
                "state history underflow while aligning GPS at {target_timestamp_s:.6}s; oldest EKF snapshot is {oldest_timestamp_s:.6}s"
            ),
        }
    }
}

pub fn purge_frame_buffer(frame: &mut MavlinkFrameBuffer) {
    frame.as_mut_slice().fill(0);
    frame.clear();
}

fn encode_mavlink_frame(
    header: MavHeader,
    message: &MavMessage,
) -> Result<MavlinkFrameBuffer, MessageWriteError> {
    let mut encoded_bytes = [0_u8; MAX_MAVLINK_FRAME_BYTES];
    let mut cursor = std::io::Cursor::new(encoded_bytes.as_mut_slice());
    let frame_length = mavlink::write_v2_msg(&mut cursor, header, message)?;
    let mut frame = MavlinkFrameBuffer::new();
    frame.extend_from_slice(&encoded_bytes[..frame_length])
        .expect("MAVLink frame length must fit within MAX_MAVLINK_FRAME_BYTES");
    Ok(frame)
}

fn interpolate_eskf_state(
    previous_state: &EskfState,
    next_state: &EskfState,
    target_timestamp_s: f64,
) -> EskfState {
    let interval_s = next_state.nominal.timestamp_s - previous_state.nominal.timestamp_s;
    let interpolation_factor = ((target_timestamp_s - previous_state.nominal.timestamp_s)
        / interval_s)
        .clamp(0.0, 1.0) as f32;

    let nominal = crate::ekf_core::state::NominalState {
        timestamp_s: target_timestamp_s,
        position_ned_m: lerp_vector3(
            previous_state.nominal.position_ned_m,
            next_state.nominal.position_ned_m,
            interpolation_factor,
        ),
        velocity_ned_mps: lerp_vector3(
            previous_state.nominal.velocity_ned_mps,
            next_state.nominal.velocity_ned_mps,
            interpolation_factor,
        ),
        attitude_body_to_ned: previous_state.nominal.attitude_body_to_ned.slerp(
            &next_state.nominal.attitude_body_to_ned,
            interpolation_factor,
        ),
        accel_bias_mps2: lerp_vector3(
            previous_state.nominal.accel_bias_mps2,
            next_state.nominal.accel_bias_mps2,
            interpolation_factor,
        ),
        gyro_bias_rps: lerp_vector3(
            previous_state.nominal.gyro_bias_rps,
            next_state.nominal.gyro_bias_rps,
            interpolation_factor,
        ),
        geodetic_reference: previous_state.nominal.geodetic_reference,
    };
    let covariance = previous_state.covariance * (1.0 - interpolation_factor)
        + next_state.covariance * interpolation_factor;

    EskfState::new(nominal, covariance)
}

fn lerp_vector3(start: Vector3<f32>, end: Vector3<f32>, interpolation_factor: f32) -> Vector3<f32> {
    start * (1.0 - interpolation_factor) + end * interpolation_factor
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::{GpsNoiseModel, GpsQualityMetrics, interpolate_eskf_state};
    use crate::ekf_core::state::{EskfState, NominalState, StateCovariance};
    use mavlink::dialects::common::GpsFixType;

    #[test]
    fn interpolation_matches_midpoint_of_bracketing_states() {
        let previous = EskfState::new(
            NominalState {
                timestamp_s: 10.0,
                position_ned_m: Vector3::new(0.0, 0.0, 0.0),
                velocity_ned_mps: Vector3::new(0.0, 0.0, 0.0),
                attitude_body_to_ned: UnitQuaternion::identity(),
                accel_bias_mps2: Vector3::zeros(),
                gyro_bias_rps: Vector3::zeros(),
                geodetic_reference: None,
            },
            StateCovariance::identity(),
        );
        let next = EskfState::new(
            NominalState {
                timestamp_s: 12.0,
                position_ned_m: Vector3::new(20.0, -10.0, 4.0),
                velocity_ned_mps: Vector3::new(4.0, -2.0, 0.5),
                attitude_body_to_ned: UnitQuaternion::from_euler_angles(0.0, 0.0, 0.4),
                accel_bias_mps2: Vector3::new(0.2, 0.1, -0.1),
                gyro_bias_rps: Vector3::new(0.02, 0.01, -0.01),
                geodetic_reference: None,
            },
            StateCovariance::identity() * 3.0,
        );

        let interpolated = interpolate_eskf_state(&previous, &next, 11.0);

        assert!(
            (interpolated.nominal.position_ned_m - Vector3::new(10.0, -5.0, 2.0)).norm() < 1.0e-5
        );
        assert!(
            (interpolated.nominal.velocity_ned_mps - Vector3::new(2.0, -1.0, 0.25)).norm() < 1.0e-5
        );
        assert!((interpolated.covariance.trace() - 30.0).abs() < 1.0e-5);
    }

    #[test]
    fn dop_metrics_expand_covariance_for_weaker_fix_quality() {
        let metrics = GpsQualityMetrics {
            timestamp_s: 1.0,
            absolute_timestamp_ns: Some(1_000_000_000),
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            hdop: 2.0,
            vdop: 3.0,
            satellites_visible: 6,
        };

        let (hpos, vpos) = metrics.position_standard_deviations(GpsNoiseModel::default());
        let (hvel, vvel) =
            metrics.velocity_standard_deviations(GpsNoiseModel::default(), hpos, vpos);

        assert!(hpos >= 10.0);
        assert!(vpos >= 15.0);
        assert!(hvel > 2.0);
        assert!(vvel > 3.0);
    }
}
