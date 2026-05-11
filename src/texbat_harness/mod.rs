use core::fmt;
use std::{
    fs::File,
    path::{Path, PathBuf},
    time::Instant,
};

use libm::{atan2, cos, sin, sqrt};
use matfile::{MatFile, NumericData};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::Serialize;

use crate::{
    ekf_core::state::{ATT_IDX, EskfState, NominalState, StateCovariance},
    statistical_monitor::{
        monitor::{EwmaRiskAccumulator, MonitorError, StatisticalMonitor},
        observation::{
            ChiSquareThresholdConfig, ClockBiasObservation, GpsObservation, TrustLevel,
        },
    },
};

const WGS84_SEMI_MAJOR_AXIS_M: f64 = 6_378_137.0;
const WGS84_FLATTENING: f64 = 1.0 / 298.257_223_563;
const WGS84_SEMI_MINOR_AXIS_M: f64 = WGS84_SEMI_MAJOR_AXIS_M * (1.0 - WGS84_FLATTENING);
const WGS84_ECCENTRICITY_SQUARED: f64 = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING);
const WGS84_SECOND_ECCENTRICITY_SQUARED: f64 = (WGS84_SEMI_MAJOR_AXIS_M * WGS84_SEMI_MAJOR_AXIS_M
    - WGS84_SEMI_MINOR_AXIS_M * WGS84_SEMI_MINOR_AXIS_M)
    / (WGS84_SEMI_MINOR_AXIS_M * WGS84_SEMI_MINOR_AXIS_M);

pub const TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S: f64 = 2.996_917_52;
pub const TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S: f64 = 2.203_320_84;
pub const TEXBAT_SCENARIO_7_OFFSET_FROM_CLEAN_S: f64 = 0.0;

pub const TEXBAT_SCENARIO_2_SPOOF_ONSET_AFTER_SCENARIO_2_START_S: f64 = 110.1;
pub const TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S: f64 = 118.9;
pub const TEXBAT_SCENARIO_7_SPOOF_ONSET_AFTER_SCENARIO_2_START_S: f64 = 110.0;

#[derive(Clone, Debug)]
pub struct TexbatScenarioConfig {
    pub scenario_name: String,
    pub clean_navsol_path: PathBuf,
    pub observed_navsol_path: PathBuf,
    pub scenario_offset_from_clean_s: f64,
    pub spoof_onset_in_observed_file_s: Option<f64>,
    pub position_state_std_m: f32,
    pub velocity_state_std_mps: f32,
    pub gps_horizontal_position_std_m: f32,
    pub gps_vertical_position_std_m: f32,
    pub gps_horizontal_velocity_std_mps: f32,
    pub gps_vertical_velocity_std_mps: f32,
    pub clock_bias_std_m: f32,
    pub yaw_state_std_rad: f32,
}

impl TexbatScenarioConfig {
    pub fn new(
        scenario_name: impl Into<String>,
        clean_navsol_path: impl Into<PathBuf>,
        observed_navsol_path: impl Into<PathBuf>,
        scenario_offset_from_clean_s: f64,
        spoof_onset_in_observed_file_s: Option<f64>,
    ) -> Self {
        Self {
            scenario_name: scenario_name.into(),
            clean_navsol_path: clean_navsol_path.into(),
            observed_navsol_path: observed_navsol_path.into(),
            scenario_offset_from_clean_s,
            spoof_onset_in_observed_file_s,
            position_state_std_m: 2.0,
            velocity_state_std_mps: 0.2,
            gps_horizontal_position_std_m: 1.5,
            gps_vertical_position_std_m: 2.0,
            gps_horizontal_velocity_std_mps: 0.3,
            gps_vertical_velocity_std_mps: 0.5,
            clock_bias_std_m: 5.0,
            yaw_state_std_rad: 0.5,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct TexbatScenarioReport {
    pub scenario_name: String,
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
    pub position_bias_calibration_ned_m: Vector3<f32>,
    pub velocity_bias_calibration_ned_mps: Vector3<f32>,
    pub clock_bias_calibration_m: f32,
}

impl TexbatScenarioReport {
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

#[derive(Clone, Debug, Serialize)]
pub struct TexbatReplayRow {
    pub timestamp_s: f64,
    pub reference_px_ned_m: f32,
    pub reference_py_ned_m: f32,
    pub reference_pz_ned_m: f32,
    pub observed_px_ned_m: f32,
    pub observed_py_ned_m: f32,
    pub observed_pz_ned_m: f32,
    pub reference_vx_ned_mps: f32,
    pub reference_vy_ned_mps: f32,
    pub reference_vz_ned_mps: f32,
    pub observed_vx_ned_mps: f32,
    pub observed_vy_ned_mps: f32,
    pub observed_vz_ned_mps: f32,
    pub reference_clock_bias_m: f32,
    pub observed_clock_bias_m: f32,
    pub label_spoofed: bool,
}

#[derive(Debug)]
pub enum TexbatError {
    Io(std::io::Error),
    Csv(csv::Error),
    Matfile(matfile::Error),
    MissingArray {
        path: PathBuf,
        array_name: &'static str,
    },
    InvalidArrayShape {
        path: PathBuf,
        array_name: &'static str,
        expected_rows_at_least: usize,
        actual_shape: Vec<usize>,
    },
    UnsupportedNumericType {
        path: PathBuf,
        array_name: &'static str,
    },
    EmptyScenario(String),
    Monitor(MonitorError),
}

impl fmt::Display for TexbatError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "TEXBAT I/O error: {error}"),
            Self::Csv(error) => write!(f, "TEXBAT CSV error: {error}"),
            Self::Matfile(error) => write!(f, "TEXBAT MAT-file error: {error}"),
            Self::MissingArray { path, array_name } => write!(
                f,
                "MAT-file {} did not contain required array {array_name}",
                path.display()
            ),
            Self::InvalidArrayShape {
                path,
                array_name,
                expected_rows_at_least,
                actual_shape,
            } => write!(
                f,
                "MAT-file {} array {array_name} had shape {:?}, expected at least {} rows",
                path.display(),
                actual_shape,
                expected_rows_at_least
            ),
            Self::UnsupportedNumericType { path, array_name } => write!(
                f,
                "MAT-file {} array {array_name} was not stored as supported numeric doubles",
                path.display()
            ),
            Self::EmptyScenario(scenario_name) => {
                write!(f, "TEXBAT scenario {scenario_name} did not yield any aligned samples")
            }
            Self::Monitor(error) => write!(f, "TEXBAT monitor error: {error}"),
        }
    }
}

#[derive(Clone, Debug)]
struct NavSolutionSeries {
    timestamps_s: Vec<f64>,
    ecef_positions_m: Vec<Vector3<f64>>,
    ecef_velocities_mps: Vec<Vector3<f64>>,
    clock_bias_m: Vec<f64>,
}

#[derive(Clone, Debug)]
struct AlignedReplaySample {
    timestamp_s: f64,
    reference_position_ned_m: Vector3<f32>,
    observed_position_ned_m: Vector3<f32>,
    reference_velocity_ned_mps: Vector3<f32>,
    observed_velocity_ned_mps: Vector3<f32>,
    reference_clock_bias_m: f32,
    observed_clock_bias_m: f32,
    label_spoofed: bool,
}

#[derive(Clone, Debug)]
struct AlignedScenarioData {
    samples: Vec<AlignedReplaySample>,
    position_bias_calibration_ned_m: Vector3<f32>,
    velocity_bias_calibration_ned_mps: Vector3<f32>,
    clock_bias_calibration_m: f32,
}

pub fn scenario_onset_in_file_seconds(
    scenario_offset_from_clean_s: f64,
    spoof_onset_after_scenario_2_start_s: f64,
) -> f64 {
    spoof_onset_after_scenario_2_start_s
        + TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S
        - scenario_offset_from_clean_s
}

pub fn run_texbat_scenario(
    config: &TexbatScenarioConfig,
    thresholds: ChiSquareThresholdConfig,
    ewma_alpha: f32,
) -> Result<TexbatScenarioReport, TexbatError> {
    let aligned_data = build_aligned_samples(config)?;
    evaluate_aligned_samples(config, aligned_data, thresholds, ewma_alpha)
}

pub fn write_texbat_replay_csv<P: AsRef<Path>>(
    config: &TexbatScenarioConfig,
    output_path: P,
) -> Result<(), TexbatError> {
    let aligned_data = build_aligned_samples(config)?;
    let file = File::create(output_path).map_err(TexbatError::Io)?;
    let mut writer = csv::Writer::from_writer(file);

    for sample in aligned_data.samples {
        writer
            .serialize(TexbatReplayRow {
                timestamp_s: sample.timestamp_s,
                reference_px_ned_m: sample.reference_position_ned_m.x,
                reference_py_ned_m: sample.reference_position_ned_m.y,
                reference_pz_ned_m: sample.reference_position_ned_m.z,
                observed_px_ned_m: sample.observed_position_ned_m.x,
                observed_py_ned_m: sample.observed_position_ned_m.y,
                observed_pz_ned_m: sample.observed_position_ned_m.z,
                reference_vx_ned_mps: sample.reference_velocity_ned_mps.x,
                reference_vy_ned_mps: sample.reference_velocity_ned_mps.y,
                reference_vz_ned_mps: sample.reference_velocity_ned_mps.z,
                observed_vx_ned_mps: sample.observed_velocity_ned_mps.x,
                observed_vy_ned_mps: sample.observed_velocity_ned_mps.y,
                observed_vz_ned_mps: sample.observed_velocity_ned_mps.z,
                reference_clock_bias_m: sample.reference_clock_bias_m,
                observed_clock_bias_m: sample.observed_clock_bias_m,
                label_spoofed: sample.label_spoofed,
            })
            .map_err(TexbatError::Csv)?;
    }

    writer.flush().map_err(TexbatError::Io)
}

fn build_aligned_samples(
    config: &TexbatScenarioConfig,
) -> Result<AlignedScenarioData, TexbatError> {
    let clean = load_navsol_file(&config.clean_navsol_path)?;
    let observed = load_navsol_file(&config.observed_navsol_path)?;

    let home_ecef = *clean
        .ecef_positions_m
        .first()
        .ok_or_else(|| TexbatError::EmptyScenario(config.scenario_name.clone()))?;
    let home_lat_lon = ecef_to_lat_lon(home_ecef);
    let ecef_to_ned = ecef_to_ned_rotation(home_lat_lon.0, home_lat_lon.1);

    let mut clean_index = 0_usize;
    let mut aligned_samples = Vec::with_capacity(observed.timestamps_s.len());

    for (observed_index, observed_timestamp_s) in observed.timestamps_s.iter().enumerate() {
        let aligned_timestamp_s = *observed_timestamp_s - config.scenario_offset_from_clean_s;
        clean_index = nearest_clean_index(&clean.timestamps_s, aligned_timestamp_s, clean_index);

        let reference_position_ned_m = ecef_to_local_ned(
            &ecef_to_ned,
            home_ecef,
            clean.ecef_positions_m[clean_index],
        );
        let observed_position_ned_m = ecef_to_local_ned(
            &ecef_to_ned,
            home_ecef,
            observed.ecef_positions_m[observed_index],
        );
        let reference_velocity_ned_mps =
            ecef_vector_to_ned(&ecef_to_ned, clean.ecef_velocities_mps[clean_index]);
        let observed_velocity_ned_mps =
            ecef_vector_to_ned(&ecef_to_ned, observed.ecef_velocities_mps[observed_index]);
        let file_relative_time_s = *observed_timestamp_s - observed.timestamps_s[0];
        let label_spoofed = config
            .spoof_onset_in_observed_file_s
            .is_some_and(|onset_s| file_relative_time_s >= onset_s);

        aligned_samples.push(AlignedReplaySample {
            timestamp_s: aligned_timestamp_s,
            reference_position_ned_m,
            observed_position_ned_m,
            reference_velocity_ned_mps,
            observed_velocity_ned_mps,
            reference_clock_bias_m: clean.clock_bias_m[clean_index] as f32,
            observed_clock_bias_m: observed.clock_bias_m[observed_index] as f32,
            label_spoofed,
        });
    }

    if aligned_samples.is_empty() {
        return Err(TexbatError::EmptyScenario(config.scenario_name.clone()));
    }

    let position_bias_calibration_ned_m = mean_position_bias(&aligned_samples);
    let velocity_bias_calibration_ned_mps = mean_velocity_bias(&aligned_samples);
    let clock_bias_calibration_m = mean_clock_bias(&aligned_samples);

    for sample in &mut aligned_samples {
        sample.observed_position_ned_m -= position_bias_calibration_ned_m;
        sample.observed_velocity_ned_mps -= velocity_bias_calibration_ned_mps;
        sample.observed_clock_bias_m -= clock_bias_calibration_m;
    }

    Ok(AlignedScenarioData {
        samples: aligned_samples,
        position_bias_calibration_ned_m,
        velocity_bias_calibration_ned_mps,
        clock_bias_calibration_m,
    })
}

fn evaluate_aligned_samples(
    config: &TexbatScenarioConfig,
    aligned_data: AlignedScenarioData,
    thresholds: ChiSquareThresholdConfig,
    ewma_alpha: f32,
) -> Result<TexbatScenarioReport, TexbatError> {
    let mut monitor = StatisticalMonitor::new(thresholds, EwmaRiskAccumulator::new(ewma_alpha));
    let mut report = TexbatScenarioReport {
        scenario_name: config.scenario_name.clone(),
        position_bias_calibration_ned_m: aligned_data.position_bias_calibration_ned_m,
        velocity_bias_calibration_ned_mps: aligned_data.velocity_bias_calibration_ned_mps,
        clock_bias_calibration_m: aligned_data.clock_bias_calibration_m,
        ..TexbatScenarioReport::default()
    };
    let mut latencies_us = Vec::with_capacity(aligned_data.samples.len());

    for sample in aligned_data.samples {
        report.total_samples += 1;

        if sample.label_spoofed {
            report.spoof_labeled_samples += 1;
        } else {
            report.clean_labeled_samples += 1;
        }

        let predicted_state = predicted_state_from_reference(config, &sample);
        let gps_observation = GpsObservation::from_accuracy_metrics(
            sample.timestamp_s,
            sample.observed_position_ned_m,
            sample.observed_velocity_ned_mps,
            config.gps_horizontal_position_std_m,
            config.gps_vertical_position_std_m,
            config.gps_horizontal_velocity_std_mps,
            config.gps_vertical_velocity_std_mps,
        );
        let clock_bias_observation = ClockBiasObservation::new(
            sample.timestamp_s,
            sample.reference_clock_bias_m,
            sample.observed_clock_bias_m,
            config.clock_bias_std_m,
        );

        let started = Instant::now();
        let verdict = monitor
            .evaluate_observations_with_clock(
                &predicted_state,
                &gps_observation,
                None,
                None,
                Some(&clock_bias_observation),
            )
            .map_err(TexbatError::Monitor)?;
        latencies_us.push(started.elapsed().as_secs_f64() * 1_000_000.0);

        match verdict.trust_level {
            TrustLevel::Trusted => report.trusted_verdicts += 1,
            TrustLevel::Flagged => report.flagged_verdicts += 1,
            TrustLevel::Rejected => report.rejected_verdicts += 1,
        }

        if sample.label_spoofed {
            if matches!(verdict.trust_level, TrustLevel::Flagged | TrustLevel::Rejected) {
                report.anomaly_true_positives += 1;
            }
            if matches!(verdict.trust_level, TrustLevel::Rejected) {
                report.rejected_true_positives += 1;
            }
        } else {
            if matches!(verdict.trust_level, TrustLevel::Flagged | TrustLevel::Rejected) {
                report.anomaly_false_positives += 1;
            }
            if matches!(verdict.trust_level, TrustLevel::Rejected) {
                report.rejected_false_positives += 1;
            }
        }
    }

    if !latencies_us.is_empty() {
        let total_latency_us: f64 = latencies_us.iter().sum();
        report.mean_evaluation_latency_us = total_latency_us / latencies_us.len() as f64;
        latencies_us.sort_by(|left, right| left.total_cmp(right));
        let p95_index = ((latencies_us.len() as f64 * 0.95).ceil() as usize)
            .saturating_sub(1)
            .min(latencies_us.len() - 1);
        report.p95_evaluation_latency_us = latencies_us[p95_index];
        report.max_evaluation_latency_us = *latencies_us.last().unwrap_or(&0.0);
    }

    Ok(report)
}

fn predicted_state_from_reference(
    config: &TexbatScenarioConfig,
    sample: &AlignedReplaySample,
) -> EskfState {
    let nominal = NominalState {
        timestamp_s: sample.timestamp_s,
        position_ned_m: sample.reference_position_ned_m,
        velocity_ned_mps: sample.reference_velocity_ned_mps,
        attitude_body_to_ned: UnitQuaternion::identity(),
        accel_bias_mps2: Vector3::zeros(),
        gyro_bias_rps: Vector3::zeros(),
        geodetic_reference: None,
    };
    let mut covariance = StateCovariance::zeros();
    let position_variance = config.position_state_std_m * config.position_state_std_m;
    let velocity_variance = config.velocity_state_std_mps * config.velocity_state_std_mps;
    let yaw_variance = config.yaw_state_std_rad * config.yaw_state_std_rad;

    covariance[(0, 0)] = position_variance;
    covariance[(1, 1)] = position_variance;
    covariance[(2, 2)] = position_variance;
    covariance[(3, 3)] = velocity_variance;
    covariance[(4, 4)] = velocity_variance;
    covariance[(5, 5)] = velocity_variance;
    covariance[(ATT_IDX + 2, ATT_IDX + 2)] = yaw_variance;

    EskfState::new(nominal, covariance)
}

fn load_navsol_file(path: &Path) -> Result<NavSolutionSeries, TexbatError> {
    let file = File::open(path).map_err(TexbatError::Io)?;
    let mat_file = MatFile::parse(file).map_err(TexbatError::Matfile)?;
    let array = mat_file
        .find_by_name("navsol")
        .ok_or_else(|| TexbatError::MissingArray {
            path: path.to_path_buf(),
            array_name: "navsol",
        })?;
    let shape = array.size().clone();
    if shape.len() < 2 || shape[0] < 12 {
        return Err(TexbatError::InvalidArrayShape {
            path: path.to_path_buf(),
            array_name: "navsol",
            expected_rows_at_least: 12,
            actual_shape: shape,
        });
    }

    let data = match array.data() {
        NumericData::Double { real, imag: None } => real,
        _ => {
            return Err(TexbatError::UnsupportedNumericType {
                path: path.to_path_buf(),
                array_name: "navsol",
            });
        }
    };

    let rows = shape[0];
    let cols = shape[1];
    let mut timestamps_s = Vec::with_capacity(cols);
    let mut ecef_positions_m = Vec::with_capacity(cols);
    let mut ecef_velocities_mps = Vec::with_capacity(cols);
    let mut clock_bias_m = Vec::with_capacity(cols);

    for column in 0..cols {
        let week_seconds = value_at(data, rows, 1, column);
        let fractional_seconds = value_at(data, rows, 2, column);
        timestamps_s.push(week_seconds + fractional_seconds);
        ecef_positions_m.push(Vector3::new(
            value_at(data, rows, 3, column),
            value_at(data, rows, 4, column),
            value_at(data, rows, 5, column),
        ));
        clock_bias_m.push(value_at(data, rows, 6, column));
        ecef_velocities_mps.push(Vector3::new(
            value_at(data, rows, 7, column),
            value_at(data, rows, 8, column),
            value_at(data, rows, 9, column),
        ));
    }

    Ok(NavSolutionSeries {
        timestamps_s,
        ecef_positions_m,
        ecef_velocities_mps,
        clock_bias_m,
    })
}

fn value_at(data: &[f64], rows: usize, row: usize, column: usize) -> f64 {
    data[row + rows * column]
}

fn nearest_clean_index(clean_timestamps_s: &[f64], target_timestamp_s: f64, start_index: usize) -> usize {
    let mut index = start_index.min(clean_timestamps_s.len().saturating_sub(1));
    while index + 1 < clean_timestamps_s.len()
        && clean_timestamps_s[index + 1] <= target_timestamp_s
    {
        index += 1;
    }

    if index + 1 < clean_timestamps_s.len() {
        let current_error = (clean_timestamps_s[index] - target_timestamp_s).abs();
        let next_error = (clean_timestamps_s[index + 1] - target_timestamp_s).abs();
        if next_error < current_error {
            return index + 1;
        }
    }

    index
}

fn mean_position_bias(samples: &[AlignedReplaySample]) -> Vector3<f32> {
    let calibration_samples: Vec<&AlignedReplaySample> = samples
        .iter()
        .filter(|sample| !sample.label_spoofed)
        .collect();
    let source = if calibration_samples.is_empty() {
        samples.iter().collect::<Vec<_>>()
    } else {
        calibration_samples
    };

    let mut sum = Vector3::zeros();
    for sample in source {
        sum += sample.observed_position_ned_m - sample.reference_position_ned_m;
    }
    sum / samples_for_mean(samples).max(1) as f32
}

fn mean_velocity_bias(samples: &[AlignedReplaySample]) -> Vector3<f32> {
    let calibration_samples: Vec<&AlignedReplaySample> = samples
        .iter()
        .filter(|sample| !sample.label_spoofed)
        .collect();
    let source = if calibration_samples.is_empty() {
        samples.iter().collect::<Vec<_>>()
    } else {
        calibration_samples
    };

    let mut sum = Vector3::zeros();
    for sample in source {
        sum += sample.observed_velocity_ned_mps - sample.reference_velocity_ned_mps;
    }
    sum / samples_for_mean(samples).max(1) as f32
}

fn mean_clock_bias(samples: &[AlignedReplaySample]) -> f32 {
    let calibration_samples: Vec<&AlignedReplaySample> = samples
        .iter()
        .filter(|sample| !sample.label_spoofed)
        .collect();
    let source = if calibration_samples.is_empty() {
        samples.iter().collect::<Vec<_>>()
    } else {
        calibration_samples
    };

    let mut sum = 0.0_f32;
    for sample in source {
        sum += sample.observed_clock_bias_m - sample.reference_clock_bias_m;
    }
    sum / samples_for_mean(samples).max(1) as f32
}

fn samples_for_mean(samples: &[AlignedReplaySample]) -> usize {
    let clean_count = samples.iter().filter(|sample| !sample.label_spoofed).count();
    if clean_count == 0 {
        samples.len()
    } else {
        clean_count
    }
}

fn ecef_to_lat_lon(ecef_m: Vector3<f64>) -> (f64, f64) {
    let p = sqrt(ecef_m.x * ecef_m.x + ecef_m.y * ecef_m.y);
    let theta = atan2(
        ecef_m.z * WGS84_SEMI_MAJOR_AXIS_M,
        p * WGS84_SEMI_MINOR_AXIS_M,
    );
    let sin_theta = sin(theta);
    let cos_theta = cos(theta);
    let latitude_rad = atan2(
        ecef_m.z
            + WGS84_SECOND_ECCENTRICITY_SQUARED
                * WGS84_SEMI_MINOR_AXIS_M
                * sin_theta
                * sin_theta
                * sin_theta,
        p - WGS84_ECCENTRICITY_SQUARED * WGS84_SEMI_MAJOR_AXIS_M * cos_theta * cos_theta * cos_theta,
    );
    let longitude_rad = atan2(ecef_m.y, ecef_m.x);
    (latitude_rad, longitude_rad)
}

fn ecef_to_ned_rotation(latitude_rad: f64, longitude_rad: f64) -> Matrix3<f64> {
    let sin_lat = sin(latitude_rad);
    let cos_lat = cos(latitude_rad);
    let sin_lon = sin(longitude_rad);
    let cos_lon = cos(longitude_rad);

    Matrix3::new(
        -sin_lat * cos_lon,
        -sin_lat * sin_lon,
        cos_lat,
        -sin_lon,
        cos_lon,
        0.0,
        -cos_lat * cos_lon,
        -cos_lat * sin_lon,
        -sin_lat,
    )
}

fn ecef_to_local_ned(
    ecef_to_ned: &Matrix3<f64>,
    home_ecef_m: Vector3<f64>,
    position_ecef_m: Vector3<f64>,
) -> Vector3<f32> {
    let ned_m = ecef_to_ned * (position_ecef_m - home_ecef_m);
    Vector3::new(ned_m.x as f32, ned_m.y as f32, ned_m.z as f32)
}

fn ecef_vector_to_ned(ecef_to_ned: &Matrix3<f64>, velocity_ecef_mps: Vector3<f64>) -> Vector3<f32> {
    let ned_mps = ecef_to_ned * velocity_ecef_mps;
    Vector3::new(ned_mps.x as f32, ned_mps.y as f32, ned_mps.z as f32)
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
    use super::{
        TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        scenario_onset_in_file_seconds,
    };

    #[test]
    fn scenario_onset_converts_from_common_reference_to_file_time() {
        let onset_in_file_s = scenario_onset_in_file_seconds(
            TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
            TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        );

        assert!((onset_in_file_s - 119.693_596_68).abs() < 1.0e-9);
    }
}
