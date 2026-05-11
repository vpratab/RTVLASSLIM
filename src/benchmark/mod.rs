use core::fmt;
use std::{
    fs::File,
    io::{Read, Write},
    path::Path,
    time::Instant,
};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::{
    ekf_core::state::{ATT_IDX, EskfState, NominalState, POS_IDX, StateCovariance, VEL_IDX},
    statistical_monitor::{
        monitor::{EwmaRiskAccumulator, MonitorError, StatisticalMonitor},
        observation::{
            BarometerObservation, ChiSquareThresholdConfig, GpsObservation, HeadingObservation,
            TrustLevel,
        },
    },
    telemetry_adapter::SynchronizedGpsSample,
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MonitorDatasetRow {
    pub timestamp_s: f64,
    pub state_px_ned_m: f32,
    pub state_py_ned_m: f32,
    pub state_pz_ned_m: f32,
    pub state_vx_ned_mps: f32,
    pub state_vy_ned_mps: f32,
    pub state_vz_ned_mps: f32,
    pub state_qw: f32,
    pub state_qx: f32,
    pub state_qy: f32,
    pub state_qz: f32,
    pub cov_pxx: f32,
    pub cov_pxy: f32,
    pub cov_pxz: f32,
    pub cov_pxvx: f32,
    pub cov_pxvy: f32,
    pub cov_pxvz: f32,
    pub cov_pyy: f32,
    pub cov_pyz: f32,
    pub cov_pyvx: f32,
    pub cov_pyvy: f32,
    pub cov_pyvz: f32,
    pub cov_pzz: f32,
    pub cov_pzvx: f32,
    pub cov_pzvy: f32,
    pub cov_pzvz: f32,
    pub cov_vxvx: f32,
    pub cov_vxvy: f32,
    pub cov_vxvz: f32,
    pub cov_vyvy: f32,
    pub cov_vyvz: f32,
    pub cov_vzvz: f32,
    pub cov_yaw_yaw: f32,
    pub gps_px_ned_m: f32,
    pub gps_py_ned_m: f32,
    pub gps_pz_ned_m: f32,
    pub gps_vx_ned_mps: f32,
    pub gps_vy_ned_mps: f32,
    pub gps_vz_ned_mps: f32,
    pub gps_horizontal_position_std_m: f32,
    pub gps_vertical_position_std_m: f32,
    pub gps_horizontal_velocity_std_mps: f32,
    pub gps_vertical_velocity_std_mps: f32,
    pub barometer_altitude_ned_down_m: Option<f32>,
    pub barometer_std_m: Option<f32>,
    pub heading_rad: Option<f32>,
    pub heading_std_rad: Option<f32>,
    #[serde(default)]
    pub label_spoofed: bool,
}

impl MonitorDatasetRow {
    pub fn from_synchronized_sample(sample: &SynchronizedGpsSample) -> Self {
        let state = &sample.aligned_predicted_state;
        let quaternion = state.nominal.attitude_body_to_ned.quaternion();
        let gps_noise = &sample.gps_observation.observation_noise;

        Self {
            timestamp_s: sample.gps_observation.timestamp_s,
            state_px_ned_m: state.nominal.position_ned_m.x,
            state_py_ned_m: state.nominal.position_ned_m.y,
            state_pz_ned_m: state.nominal.position_ned_m.z,
            state_vx_ned_mps: state.nominal.velocity_ned_mps.x,
            state_vy_ned_mps: state.nominal.velocity_ned_mps.y,
            state_vz_ned_mps: state.nominal.velocity_ned_mps.z,
            state_qw: quaternion.w,
            state_qx: quaternion.i,
            state_qy: quaternion.j,
            state_qz: quaternion.k,
            cov_pxx: state.covariance[(POS_IDX, POS_IDX)],
            cov_pxy: state.covariance[(POS_IDX, POS_IDX + 1)],
            cov_pxz: state.covariance[(POS_IDX, POS_IDX + 2)],
            cov_pxvx: state.covariance[(POS_IDX, VEL_IDX)],
            cov_pxvy: state.covariance[(POS_IDX, VEL_IDX + 1)],
            cov_pxvz: state.covariance[(POS_IDX, VEL_IDX + 2)],
            cov_pyy: state.covariance[(POS_IDX + 1, POS_IDX + 1)],
            cov_pyz: state.covariance[(POS_IDX + 1, POS_IDX + 2)],
            cov_pyvx: state.covariance[(POS_IDX + 1, VEL_IDX)],
            cov_pyvy: state.covariance[(POS_IDX + 1, VEL_IDX + 1)],
            cov_pyvz: state.covariance[(POS_IDX + 1, VEL_IDX + 2)],
            cov_pzz: state.covariance[(POS_IDX + 2, POS_IDX + 2)],
            cov_pzvx: state.covariance[(POS_IDX + 2, VEL_IDX)],
            cov_pzvy: state.covariance[(POS_IDX + 2, VEL_IDX + 1)],
            cov_pzvz: state.covariance[(POS_IDX + 2, VEL_IDX + 2)],
            cov_vxvx: state.covariance[(VEL_IDX, VEL_IDX)],
            cov_vxvy: state.covariance[(VEL_IDX, VEL_IDX + 1)],
            cov_vxvz: state.covariance[(VEL_IDX, VEL_IDX + 2)],
            cov_vyvy: state.covariance[(VEL_IDX + 1, VEL_IDX + 1)],
            cov_vyvz: state.covariance[(VEL_IDX + 1, VEL_IDX + 2)],
            cov_vzvz: state.covariance[(VEL_IDX + 2, VEL_IDX + 2)],
            cov_yaw_yaw: state.covariance[(ATT_IDX + 2, ATT_IDX + 2)],
            gps_px_ned_m: sample.gps_observation.position_ned_m.x,
            gps_py_ned_m: sample.gps_observation.position_ned_m.y,
            gps_pz_ned_m: sample.gps_observation.position_ned_m.z,
            gps_vx_ned_mps: sample.gps_observation.velocity_ned_mps.x,
            gps_vy_ned_mps: sample.gps_observation.velocity_ned_mps.y,
            gps_vz_ned_mps: sample.gps_observation.velocity_ned_mps.z,
            gps_horizontal_position_std_m: gps_noise[(0, 0)].sqrt(),
            gps_vertical_position_std_m: gps_noise[(2, 2)].sqrt(),
            gps_horizontal_velocity_std_mps: gps_noise[(3, 3)].sqrt(),
            gps_vertical_velocity_std_mps: gps_noise[(5, 5)].sqrt(),
            barometer_altitude_ned_down_m: sample
                .barometer_observation
                .as_ref()
                .map(|observation| observation.altitude_ned_down_m),
            barometer_std_m: sample
                .barometer_observation
                .as_ref()
                .map(|observation| observation.altitude_std_m),
            heading_rad: sample
                .heading_observation
                .as_ref()
                .map(|observation| observation.heading_rad),
            heading_std_rad: sample
                .heading_observation
                .as_ref()
                .map(|observation| observation.heading_std_rad),
            label_spoofed: false,
        }
    }

    pub fn gps_observation(&self) -> GpsObservation {
        GpsObservation::from_accuracy_metrics(
            self.timestamp_s,
            Vector3::new(self.gps_px_ned_m, self.gps_py_ned_m, self.gps_pz_ned_m),
            Vector3::new(
                self.gps_vx_ned_mps,
                self.gps_vy_ned_mps,
                self.gps_vz_ned_mps,
            ),
            self.gps_horizontal_position_std_m,
            self.gps_vertical_position_std_m,
            self.gps_horizontal_velocity_std_mps,
            self.gps_vertical_velocity_std_mps,
        )
    }

    pub fn barometer_observation(&self) -> Option<BarometerObservation> {
        match (self.barometer_altitude_ned_down_m, self.barometer_std_m) {
            (Some(altitude_ned_down_m), Some(altitude_std_m)) => Some(BarometerObservation::new(
                self.timestamp_s,
                altitude_ned_down_m,
                altitude_std_m,
            )),
            _ => None,
        }
    }

    pub fn heading_observation(&self) -> Option<HeadingObservation> {
        match (self.heading_rad, self.heading_std_rad) {
            (Some(heading_rad), Some(heading_std_rad)) => Some(HeadingObservation::new(
                self.timestamp_s,
                heading_rad,
                heading_std_rad,
            )),
            _ => None,
        }
    }

    pub fn reconstruct_state(&self) -> EskfState {
        let mut covariance = StateCovariance::zeros();

        covariance[(POS_IDX, POS_IDX)] = self.cov_pxx;
        covariance[(POS_IDX, POS_IDX + 1)] = self.cov_pxy;
        covariance[(POS_IDX + 1, POS_IDX)] = self.cov_pxy;
        covariance[(POS_IDX, POS_IDX + 2)] = self.cov_pxz;
        covariance[(POS_IDX + 2, POS_IDX)] = self.cov_pxz;

        covariance[(POS_IDX, VEL_IDX)] = self.cov_pxvx;
        covariance[(VEL_IDX, POS_IDX)] = self.cov_pxvx;
        covariance[(POS_IDX, VEL_IDX + 1)] = self.cov_pxvy;
        covariance[(VEL_IDX + 1, POS_IDX)] = self.cov_pxvy;
        covariance[(POS_IDX, VEL_IDX + 2)] = self.cov_pxvz;
        covariance[(VEL_IDX + 2, POS_IDX)] = self.cov_pxvz;

        covariance[(POS_IDX + 1, POS_IDX + 1)] = self.cov_pyy;
        covariance[(POS_IDX + 1, POS_IDX + 2)] = self.cov_pyz;
        covariance[(POS_IDX + 2, POS_IDX + 1)] = self.cov_pyz;
        covariance[(POS_IDX + 1, VEL_IDX)] = self.cov_pyvx;
        covariance[(VEL_IDX, POS_IDX + 1)] = self.cov_pyvx;
        covariance[(POS_IDX + 1, VEL_IDX + 1)] = self.cov_pyvy;
        covariance[(VEL_IDX + 1, POS_IDX + 1)] = self.cov_pyvy;
        covariance[(POS_IDX + 1, VEL_IDX + 2)] = self.cov_pyvz;
        covariance[(VEL_IDX + 2, POS_IDX + 1)] = self.cov_pyvz;

        covariance[(POS_IDX + 2, POS_IDX + 2)] = self.cov_pzz;
        covariance[(POS_IDX + 2, VEL_IDX)] = self.cov_pzvx;
        covariance[(VEL_IDX, POS_IDX + 2)] = self.cov_pzvx;
        covariance[(POS_IDX + 2, VEL_IDX + 1)] = self.cov_pzvy;
        covariance[(VEL_IDX + 1, POS_IDX + 2)] = self.cov_pzvy;
        covariance[(POS_IDX + 2, VEL_IDX + 2)] = self.cov_pzvz;
        covariance[(VEL_IDX + 2, POS_IDX + 2)] = self.cov_pzvz;

        covariance[(VEL_IDX, VEL_IDX)] = self.cov_vxvx;
        covariance[(VEL_IDX, VEL_IDX + 1)] = self.cov_vxvy;
        covariance[(VEL_IDX + 1, VEL_IDX)] = self.cov_vxvy;
        covariance[(VEL_IDX, VEL_IDX + 2)] = self.cov_vxvz;
        covariance[(VEL_IDX + 2, VEL_IDX)] = self.cov_vxvz;
        covariance[(VEL_IDX + 1, VEL_IDX + 1)] = self.cov_vyvy;
        covariance[(VEL_IDX + 1, VEL_IDX + 2)] = self.cov_vyvz;
        covariance[(VEL_IDX + 2, VEL_IDX + 1)] = self.cov_vyvz;
        covariance[(VEL_IDX + 2, VEL_IDX + 2)] = self.cov_vzvz;
        covariance[(ATT_IDX + 2, ATT_IDX + 2)] = self.cov_yaw_yaw;

        EskfState::new(
            NominalState {
                timestamp_s: self.timestamp_s,
                position_ned_m: Vector3::new(
                    self.state_px_ned_m,
                    self.state_py_ned_m,
                    self.state_pz_ned_m,
                ),
                velocity_ned_mps: Vector3::new(
                    self.state_vx_ned_mps,
                    self.state_vy_ned_mps,
                    self.state_vz_ned_mps,
                ),
                attitude_body_to_ned: UnitQuaternion::from_quaternion(Quaternion::new(
                    self.state_qw,
                    self.state_qx,
                    self.state_qy,
                    self.state_qz,
                )),
                accel_bias_mps2: Vector3::zeros(),
                gyro_bias_rps: Vector3::zeros(),
                geodetic_reference: None,
            },
            covariance,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SpoofInjectionConfig {
    pub onset_time_s: f64,
    pub ramp_duration_s: f64,
    pub position_offset_ned_m: Vector3<f32>,
    pub velocity_offset_ned_mps: Vector3<f32>,
}

impl SpoofInjectionConfig {
    pub const fn new(
        onset_time_s: f64,
        ramp_duration_s: f64,
        position_offset_ned_m: Vector3<f32>,
        velocity_offset_ned_mps: Vector3<f32>,
    ) -> Self {
        Self {
            onset_time_s,
            ramp_duration_s,
            position_offset_ned_m,
            velocity_offset_ned_mps,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct MonitorDatasetReport {
    pub total_samples: u64,
    pub spoof_labeled_samples: u64,
    pub clean_labeled_samples: u64,
    pub trusted_verdicts: u64,
    pub flagged_verdicts: u64,
    pub rejected_verdicts: u64,
    pub anomaly_true_positives: u64,
    pub anomaly_false_positives: u64,
    pub rejected_true_positives: u64,
    pub rejected_false_positives: u64,
    pub mean_evaluation_latency_us: f64,
    pub p95_evaluation_latency_us: f64,
    pub max_evaluation_latency_us: f64,
}

impl MonitorDatasetReport {
    pub fn anomaly_true_positive_rate(&self) -> f64 {
        ratio(self.anomaly_true_positives, self.spoof_labeled_samples)
    }

    pub fn anomaly_false_positive_rate(&self) -> f64 {
        ratio(self.anomaly_false_positives, self.clean_labeled_samples)
    }

    pub fn rejected_true_positive_rate(&self) -> f64 {
        ratio(self.rejected_true_positives, self.spoof_labeled_samples)
    }

    pub fn rejected_false_positive_rate(&self) -> f64 {
        ratio(self.rejected_false_positives, self.clean_labeled_samples)
    }
}

#[derive(Debug)]
pub enum MonitorDatasetError {
    Io(std::io::Error),
    Csv(csv::Error),
    Monitor(MonitorError),
}

impl fmt::Display for MonitorDatasetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "dataset I/O error: {error}"),
            Self::Csv(error) => write!(f, "dataset CSV error: {error}"),
            Self::Monitor(error) => write!(f, "dataset monitor error: {error}"),
        }
    }
}

pub fn run_monitor_dataset_file<P: AsRef<Path>>(
    path: P,
    thresholds: ChiSquareThresholdConfig,
    ewma_alpha: f32,
) -> Result<MonitorDatasetReport, MonitorDatasetError> {
    let file = File::open(path).map_err(MonitorDatasetError::Io)?;
    run_monitor_dataset_reader(file, thresholds, ewma_alpha)
}

pub fn run_monitor_dataset_reader<R: Read>(
    reader: R,
    thresholds: ChiSquareThresholdConfig,
    ewma_alpha: f32,
) -> Result<MonitorDatasetReport, MonitorDatasetError> {
    let mut csv_reader = csv::Reader::from_reader(reader);
    let mut monitor = StatisticalMonitor::new(thresholds, EwmaRiskAccumulator::new(ewma_alpha));
    let mut report = MonitorDatasetReport::default();
    let mut evaluation_latencies_us = Vec::new();

    for row in csv_reader.deserialize::<MonitorDatasetRow>() {
        let row = row.map_err(MonitorDatasetError::Csv)?;
        report.total_samples += 1;

        let evaluation_started = Instant::now();
        let verdict = monitor
            .evaluate_observations(
                &row.reconstruct_state(),
                &row.gps_observation(),
                row.barometer_observation().as_ref(),
                row.heading_observation().as_ref(),
            )
            .map_err(MonitorDatasetError::Monitor)?;
        evaluation_latencies_us.push(evaluation_started.elapsed().as_secs_f64() * 1_000_000.0);

        match verdict.trust_level {
            TrustLevel::Trusted => report.trusted_verdicts += 1,
            TrustLevel::Flagged => report.flagged_verdicts += 1,
            TrustLevel::Rejected => report.rejected_verdicts += 1,
        }

        if row.label_spoofed {
            report.spoof_labeled_samples += 1;
            if matches!(
                verdict.trust_level,
                TrustLevel::Flagged | TrustLevel::Rejected
            ) {
                report.anomaly_true_positives += 1;
            }
            if matches!(verdict.trust_level, TrustLevel::Rejected) {
                report.rejected_true_positives += 1;
            }
        } else {
            report.clean_labeled_samples += 1;
            if matches!(
                verdict.trust_level,
                TrustLevel::Flagged | TrustLevel::Rejected
            ) {
                report.anomaly_false_positives += 1;
            }
            if matches!(verdict.trust_level, TrustLevel::Rejected) {
                report.rejected_false_positives += 1;
            }
        }
    }

    if !evaluation_latencies_us.is_empty() {
        let total_latency_us: f64 = evaluation_latencies_us.iter().sum();
        report.mean_evaluation_latency_us = total_latency_us / evaluation_latencies_us.len() as f64;

        evaluation_latencies_us.sort_by(|left, right| left.total_cmp(right));
        let p95_index = ((evaluation_latencies_us.len() as f64 * 0.95).ceil() as usize)
            .saturating_sub(1)
            .min(evaluation_latencies_us.len() - 1);
        report.p95_evaluation_latency_us = evaluation_latencies_us[p95_index];
        report.max_evaluation_latency_us = *evaluation_latencies_us.last().unwrap_or(&0.0);
    }

    Ok(report)
}

pub fn write_spoofed_monitor_dataset_file<P: AsRef<Path>, Q: AsRef<Path>>(
    input_path: P,
    output_path: Q,
    config: SpoofInjectionConfig,
) -> Result<(), MonitorDatasetError> {
    let input = File::open(input_path).map_err(MonitorDatasetError::Io)?;
    let output = File::create(output_path).map_err(MonitorDatasetError::Io)?;
    write_spoofed_monitor_dataset_reader_writer(input, output, config)
}

pub fn write_spoofed_monitor_dataset_reader_writer<R: Read, W: Write>(
    reader: R,
    writer: W,
    config: SpoofInjectionConfig,
) -> Result<(), MonitorDatasetError> {
    let mut csv_reader = csv::Reader::from_reader(reader);
    let mut csv_writer = csv::Writer::from_writer(writer);

    for row in csv_reader.deserialize::<MonitorDatasetRow>() {
        let mut row = row.map_err(MonitorDatasetError::Csv)?;
        let scale = spoof_scale(row.timestamp_s, config);
        if scale > 0.0 {
            row.gps_px_ned_m += config.position_offset_ned_m.x * scale;
            row.gps_py_ned_m += config.position_offset_ned_m.y * scale;
            row.gps_pz_ned_m += config.position_offset_ned_m.z * scale;
            row.gps_vx_ned_mps += config.velocity_offset_ned_mps.x * scale;
            row.gps_vy_ned_mps += config.velocity_offset_ned_mps.y * scale;
            row.gps_vz_ned_mps += config.velocity_offset_ned_mps.z * scale;
            row.label_spoofed = true;
        } else {
            row.label_spoofed = false;
        }

        csv_writer
            .serialize(row)
            .map_err(MonitorDatasetError::Csv)?;
    }

    csv_writer.flush().map_err(MonitorDatasetError::Io)
}

fn spoof_scale(timestamp_s: f64, config: SpoofInjectionConfig) -> f32 {
    if timestamp_s < config.onset_time_s {
        return 0.0;
    }

    if config.ramp_duration_s <= 0.0 {
        return 1.0;
    }

    (((timestamp_s - config.onset_time_s) / config.ramp_duration_s).clamp(0.0, 1.0)) as f32
}

fn ratio(numerator: u64, denominator: u64) -> f64 {
    if denominator == 0 {
        0.0
    } else {
        numerator as f64 / denominator as f64
    }
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use nalgebra::{UnitQuaternion, Vector3};

    use super::{
        MonitorDatasetRow, SpoofInjectionConfig, run_monitor_dataset_reader,
        write_spoofed_monitor_dataset_reader_writer,
    };
    use crate::{
        ekf_core::state::{EskfState, NominalState, StateCovariance},
        statistical_monitor::observation::ChiSquareThresholdConfig,
        telemetry_adapter::SynchronizedGpsSample,
    };

    #[test]
    fn spoof_writer_labels_rows_after_onset() {
        let mut input = Vec::new();
        {
            let mut writer = csv::Writer::from_writer(&mut input);
            writer
                .serialize(MonitorDatasetRow {
                    timestamp_s: 0.5,
                    state_px_ned_m: 0.0,
                    state_py_ned_m: 0.0,
                    state_pz_ned_m: 0.0,
                    state_vx_ned_mps: 0.0,
                    state_vy_ned_mps: 0.0,
                    state_vz_ned_mps: 0.0,
                    state_qw: 1.0,
                    state_qx: 0.0,
                    state_qy: 0.0,
                    state_qz: 0.0,
                    cov_pxx: 0.01,
                    cov_pxy: 0.0,
                    cov_pxz: 0.0,
                    cov_pxvx: 0.0,
                    cov_pxvy: 0.0,
                    cov_pxvz: 0.0,
                    cov_pyy: 0.01,
                    cov_pyz: 0.0,
                    cov_pyvx: 0.0,
                    cov_pyvy: 0.0,
                    cov_pyvz: 0.0,
                    cov_pzz: 0.01,
                    cov_pzvx: 0.0,
                    cov_pzvy: 0.0,
                    cov_pzvz: 0.0,
                    cov_vxvx: 0.01,
                    cov_vxvy: 0.0,
                    cov_vxvz: 0.0,
                    cov_vyvy: 0.01,
                    cov_vyvz: 0.0,
                    cov_vzvz: 0.01,
                    cov_yaw_yaw: 0.01,
                    gps_px_ned_m: 0.0,
                    gps_py_ned_m: 0.0,
                    gps_pz_ned_m: 0.0,
                    gps_vx_ned_mps: 0.0,
                    gps_vy_ned_mps: 0.0,
                    gps_vz_ned_mps: 0.0,
                    gps_horizontal_position_std_m: 1.5,
                    gps_vertical_position_std_m: 2.0,
                    gps_horizontal_velocity_std_mps: 0.3,
                    gps_vertical_velocity_std_mps: 0.5,
                    barometer_altitude_ned_down_m: None,
                    barometer_std_m: None,
                    heading_rad: None,
                    heading_std_rad: None,
                    label_spoofed: false,
                })
                .unwrap();
            writer
                .serialize(MonitorDatasetRow {
                    timestamp_s: 1.5,
                    state_px_ned_m: 0.0,
                    state_py_ned_m: 0.0,
                    state_pz_ned_m: 0.0,
                    state_vx_ned_mps: 0.0,
                    state_vy_ned_mps: 0.0,
                    state_vz_ned_mps: 0.0,
                    state_qw: 1.0,
                    state_qx: 0.0,
                    state_qy: 0.0,
                    state_qz: 0.0,
                    cov_pxx: 0.01,
                    cov_pxy: 0.0,
                    cov_pxz: 0.0,
                    cov_pxvx: 0.0,
                    cov_pxvy: 0.0,
                    cov_pxvz: 0.0,
                    cov_pyy: 0.01,
                    cov_pyz: 0.0,
                    cov_pyvx: 0.0,
                    cov_pyvy: 0.0,
                    cov_pyvz: 0.0,
                    cov_pzz: 0.01,
                    cov_pzvx: 0.0,
                    cov_pzvy: 0.0,
                    cov_pzvz: 0.0,
                    cov_vxvx: 0.01,
                    cov_vxvy: 0.0,
                    cov_vxvz: 0.0,
                    cov_vyvy: 0.01,
                    cov_vyvz: 0.0,
                    cov_vzvz: 0.01,
                    cov_yaw_yaw: 0.01,
                    gps_px_ned_m: 0.0,
                    gps_py_ned_m: 0.0,
                    gps_pz_ned_m: 0.0,
                    gps_vx_ned_mps: 0.0,
                    gps_vy_ned_mps: 0.0,
                    gps_vz_ned_mps: 0.0,
                    gps_horizontal_position_std_m: 1.5,
                    gps_vertical_position_std_m: 2.0,
                    gps_horizontal_velocity_std_mps: 0.3,
                    gps_vertical_velocity_std_mps: 0.5,
                    barometer_altitude_ned_down_m: None,
                    barometer_std_m: None,
                    heading_rad: None,
                    heading_std_rad: None,
                    label_spoofed: false,
                })
                .unwrap();
            writer.flush().unwrap();
        }
        let mut output = Vec::new();

        write_spoofed_monitor_dataset_reader_writer(
            Cursor::new(input),
            &mut output,
            SpoofInjectionConfig::new(
                1.0,
                0.5,
                Vector3::new(10.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
            ),
        )
        .unwrap();

        let mut reader = csv::Reader::from_reader(Cursor::new(output));
        let rows: Vec<MonitorDatasetRow> = reader.deserialize().map(Result::unwrap).collect();

        assert_eq!(rows.len(), 2);
        assert!(!rows[0].label_spoofed);
        assert!(rows[1].label_spoofed);
        assert!((rows[1].gps_px_ned_m - 10.0).abs() < 1.0e-6);
        assert!((rows[1].gps_vx_ned_mps - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn monitor_dataset_reports_true_positive_rate() {
        let nominal_state = EskfState::new(
            NominalState {
                timestamp_s: 1.0,
                position_ned_m: Vector3::zeros(),
                velocity_ned_mps: Vector3::zeros(),
                attitude_body_to_ned: UnitQuaternion::identity(),
                accel_bias_mps2: Vector3::zeros(),
                gyro_bias_rps: Vector3::zeros(),
                geodetic_reference: None,
            },
            StateCovariance::identity() * 1.0e-3,
        );
        let row = MonitorDatasetRow::from_synchronized_sample(&SynchronizedGpsSample {
            timestamp_ns: 1_000_000_000,
            gps_observation:
                crate::statistical_monitor::observation::GpsObservation::from_accuracy_metrics(
                    1.0,
                    Vector3::new(120.0, -60.0, 15.0),
                    Vector3::new(12.0, -6.0, 1.5),
                    1.5,
                    2.0,
                    0.3,
                    0.5,
                ),
            barometer_observation: None,
            heading_observation: None,
            aligned_predicted_state: nominal_state,
            raw_frame: heapless::Vec::new(),
        });
        let mut bytes = Vec::new();
        let mut writer = csv::Writer::from_writer(&mut bytes);
        let mut spoofed_row = row.clone();
        spoofed_row.label_spoofed = true;
        writer.serialize(spoofed_row).unwrap();
        writer.flush().unwrap();
        drop(writer);

        let report = run_monitor_dataset_reader(
            Cursor::new(bytes),
            ChiSquareThresholdConfig::new(12.592, 22.458),
            1.0,
        )
        .unwrap();

        assert_eq!(report.total_samples, 1);
        assert_eq!(report.rejected_true_positive_rate(), 1.0);
    }
}
