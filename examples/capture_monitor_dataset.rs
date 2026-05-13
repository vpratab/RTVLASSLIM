use std::{
    io::Write,
    path::PathBuf,
    process::ExitCode,
    thread,
    time::{Duration, Instant},
};

use nalgebra::{UnitQuaternion, Vector3};

use rtvlas::{
    benchmark::MonitorDatasetRow,
    ekf_core::{
        predict::predict_in_place,
        state::{EskfState, ImuNoiseModel, NominalState, PredictConfig, StateCovariance},
    },
    telemetry_adapter::{MavlinkSubscriber, TelemetryUpdate, purge_frame_buffer},
};

const SETPOINT_INTERVAL: Duration = Duration::from_millis(100);
const ARM_AFTER: Duration = Duration::from_secs(1);
const OFFBOARD_AFTER: Duration = Duration::from_secs(2);
const COMMAND_RETRY_INTERVAL: Duration = Duration::from_millis(750);

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("dataset capture failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let connection =
        argument_value("--connection").unwrap_or_else(|| "udpout:127.0.0.1:18570".to_owned());
    let samples = argument_value("--samples")
        .map(|value| value.parse::<usize>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(60);
    let output_path = argument_value("--output")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_monitor_dataset.csv"));
    let mission_profile = MissionProfile::parse(
        argument_value("--mission-profile")
            .as_deref()
            .unwrap_or("hover"),
    )?;

    if let Some(parent_directory) = output_path.parent() {
        std::fs::create_dir_all(parent_directory).map_err(|error| error.to_string())?;
    }

    let subscriber = MavlinkSubscriber::bind(&connection).map_err(|error| error.to_string())?;
    for _ in 0..3 {
        subscriber
            .announce_ground_station()
            .map_err(|error| error.to_string())?;
        subscriber
            .request_standard_message_streams()
            .map_err(|error| error.to_string())?;
    }

    let mut state = initial_eskf_state();
    let predict_config = predict_config();
    let file = std::fs::File::create(&output_path).map_err(|error| error.to_string())?;
    let mut writer = csv::Writer::from_writer(file);
    let mut subscriber = subscriber;
    let mut captured_samples = 0_usize;
    let mut mission_controller = MissionController::new(mission_profile);
    let mut capture_summary = CaptureSummary::default();

    println!("Capturing {samples} synchronized samples from {connection}");
    println!("Mission profile: {}", mission_profile.label());
    println!("Writing dataset to {}", output_path.display());
    let _ = std::io::stdout().flush();

    while captured_samples < samples {
        mission_controller
            .tick(&subscriber)
            .map_err(|error| error.to_string())?;

        match subscriber
            .try_recv_next()
            .map_err(|error| error.to_string())?
        {
            Some(TelemetryUpdate::Imu {
                sample,
                mut raw_frame,
            }) => {
                predict_in_place(&mut state, &predict_config, &sample)
                    .map_err(|error| error.to_string())?;
                subscriber.record_predicted_state(&state);
                purge_frame_buffer(&mut raw_frame);
            }
            Some(TelemetryUpdate::GpsObservationQueued { .. }) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }

        while let Some(mut synchronized_sample) = subscriber
            .try_dequeue_synchronized_gps()
            .map_err(|error| error.to_string())?
        {
            let row = MonitorDatasetRow::from_synchronized_sample(&synchronized_sample);
            capture_summary.observe(&row);
            writer.serialize(row).map_err(|error| error.to_string())?;
            writer.flush().map_err(|error| error.to_string())?;
            purge_frame_buffer(&mut synchronized_sample.raw_frame);

            captured_samples += 1;
            println!(
                "captured sample #{:03} at t={:.3}s",
                captured_samples, synchronized_sample.gps_observation.timestamp_s
            );
            let _ = std::io::stdout().flush();

            if captured_samples >= samples {
                break;
            }
        }
    }

    println!(
        "capture summary (GPS NED): x=[{:.2}, {:.2}] m y=[{:.2}, {:.2}] m z=[{:.2}, {:.2}] m max_speed={:.2} m/s",
        capture_summary.min_position_ned_m.x,
        capture_summary.max_position_ned_m.x,
        capture_summary.min_position_ned_m.y,
        capture_summary.max_position_ned_m.y,
        capture_summary.min_position_ned_m.z,
        capture_summary.max_position_ned_m.z,
        capture_summary.max_speed_mps,
    );
    let _ = std::io::stdout().flush();

    Ok(())
}

#[derive(Clone, Copy, Debug)]
enum MissionProfile {
    Hover,
    Forward,
    Turn,
    Climb,
}

impl MissionProfile {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "hover" => Ok(Self::Hover),
            "forward" => Ok(Self::Forward),
            "turn" => Ok(Self::Turn),
            "climb" => Ok(Self::Climb),
            other => Err(format!(
                "unsupported mission profile '{other}', expected hover|forward|turn|climb"
            )),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Hover => "hover",
            Self::Forward => "forward",
            Self::Turn => "turn",
            Self::Climb => "climb",
        }
    }

    fn target(self, elapsed_s: f32) -> MissionTarget {
        match self {
            Self::Hover => MissionTarget::new(Vector3::new(0.0, 0.0, -6.0), 0.0),
            Self::Forward => {
                let north_m = ramp(elapsed_s, 4.0, 12.0, 0.0, 30.0);
                MissionTarget::new(Vector3::new(north_m, 0.0, -6.0), 0.0)
            }
            Self::Turn => {
                let radius_m = 12.0;
                let angular_rate_rps = 0.35;
                let phase_rad = ((elapsed_s - 4.0).max(0.0)) * angular_rate_rps;
                let north_m = radius_m * phase_rad.sin();
                let east_m = radius_m * (1.0 - phase_rad.cos());
                MissionTarget::new(
                    Vector3::new(north_m, east_m, -6.0),
                    phase_rad + core::f32::consts::FRAC_PI_2,
                )
            }
            Self::Climb => {
                let north_m = ramp(elapsed_s, 6.0, 14.0, 0.0, 10.0);
                let down_m = ramp(elapsed_s, 4.0, 14.0, -4.0, -20.0);
                MissionTarget::new(Vector3::new(north_m, 0.0, down_m), 0.0)
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct MissionTarget {
    position_ned_m: Vector3<f32>,
    yaw_rad: f32,
}

impl MissionTarget {
    const fn new(position_ned_m: Vector3<f32>, yaw_rad: f32) -> Self {
        Self {
            position_ned_m,
            yaw_rad,
        }
    }
}

struct MissionController {
    profile: MissionProfile,
    started_at: Instant,
    last_setpoint_sent_at: Option<Instant>,
    last_arm_command_at: Option<Instant>,
    last_offboard_command_at: Option<Instant>,
}

impl MissionController {
    fn new(profile: MissionProfile) -> Self {
        Self {
            profile,
            started_at: Instant::now(),
            last_setpoint_sent_at: None,
            last_arm_command_at: None,
            last_offboard_command_at: None,
        }
    }

    fn tick(&mut self, subscriber: &MavlinkSubscriber) -> Result<(), String> {
        let elapsed = self.started_at.elapsed();
        let send_setpoint = self
            .last_setpoint_sent_at
            .map(|last_sent_at| last_sent_at.elapsed() >= SETPOINT_INTERVAL)
            .unwrap_or(true);

        if send_setpoint {
            let elapsed_s = elapsed.as_secs_f32();
            let target = self.profile.target(elapsed_s);
            subscriber
                .send_local_position_setpoint(
                    (elapsed_s * 1_000.0).round() as u32,
                    target.position_ned_m,
                    target.yaw_rad,
                )
                .map_err(|error| error.to_string())?;
            self.last_setpoint_sent_at = Some(Instant::now());
        }

        let send_arm = elapsed >= ARM_AFTER
            && self
                .last_arm_command_at
                .map(|last_sent_at| last_sent_at.elapsed() >= COMMAND_RETRY_INTERVAL)
                .unwrap_or(true);
        if send_arm {
            subscriber
                .arm_vehicle()
                .map_err(|error| error.to_string())?;
            self.last_arm_command_at = Some(Instant::now());
        }

        let send_offboard = elapsed >= OFFBOARD_AFTER
            && self
                .last_offboard_command_at
                .map(|last_sent_at| last_sent_at.elapsed() >= COMMAND_RETRY_INTERVAL)
                .unwrap_or(true);
        if send_offboard {
            subscriber
                .set_offboard_mode()
                .map_err(|error| error.to_string())?;
            self.last_offboard_command_at = Some(Instant::now());
        }

        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct CaptureSummary {
    min_position_ned_m: Vector3<f32>,
    max_position_ned_m: Vector3<f32>,
    max_speed_mps: f32,
}

impl Default for CaptureSummary {
    fn default() -> Self {
        Self {
            min_position_ned_m: Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max_position_ned_m: Vector3::new(
                f32::NEG_INFINITY,
                f32::NEG_INFINITY,
                f32::NEG_INFINITY,
            ),
            max_speed_mps: 0.0,
        }
    }
}

impl CaptureSummary {
    fn observe(&mut self, row: &MonitorDatasetRow) {
        self.min_position_ned_m.x = self.min_position_ned_m.x.min(row.gps_px_ned_m);
        self.min_position_ned_m.y = self.min_position_ned_m.y.min(row.gps_py_ned_m);
        self.min_position_ned_m.z = self.min_position_ned_m.z.min(row.gps_pz_ned_m);
        self.max_position_ned_m.x = self.max_position_ned_m.x.max(row.gps_px_ned_m);
        self.max_position_ned_m.y = self.max_position_ned_m.y.max(row.gps_py_ned_m);
        self.max_position_ned_m.z = self.max_position_ned_m.z.max(row.gps_pz_ned_m);

        let speed_mps =
            Vector3::new(row.gps_vx_ned_mps, row.gps_vy_ned_mps, row.gps_vz_ned_mps).norm();
        self.max_speed_mps = self.max_speed_mps.max(speed_mps);
    }
}

fn ramp(elapsed_s: f32, start_s: f32, end_s: f32, start_value: f32, end_value: f32) -> f32 {
    if elapsed_s <= start_s {
        return start_value;
    }
    if elapsed_s >= end_s {
        return end_value;
    }
    let alpha = (elapsed_s - start_s) / (end_s - start_s);
    start_value + alpha * (end_value - start_value)
}

fn initial_eskf_state() -> EskfState {
    EskfState::new(
        NominalState {
            timestamp_s: 0.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        },
        StateCovariance::identity() * 1.0e-3,
    )
}

fn predict_config() -> PredictConfig {
    PredictConfig::new(
        Vector3::new(0.0, 0.0, 9.80665),
        0.2,
        ImuNoiseModel::new(
            Vector3::new(0.05, 0.05, 0.05),
            Vector3::new(0.002, 0.002, 0.002),
            Vector3::new(0.0002, 0.0002, 0.0002),
            Vector3::new(0.00002, 0.00002, 0.00002),
        ),
    )
}

fn argument_value(flag: &str) -> Option<String> {
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == flag {
            return arguments.next();
        }
    }
    None
}
