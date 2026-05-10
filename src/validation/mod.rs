use core::fmt;
use std::{fs::File, io::Read, path::Path};

use nalgebra::Vector3;
use serde::Deserialize;

use crate::{
    ekf_core::{
        predict::{PredictError, predict_in_place},
        state::{EskfState, ImuSample, PredictConfig},
    },
    statistical_monitor::{
        monitor::{EwmaRiskAccumulator, MonitorError, StatisticalMonitor},
        observation::{
            BarometerObservation, ChiSquareThresholdConfig, GpsObservation, HeadingObservation,
            TrustLevel,
        },
    },
};

#[derive(Clone, Debug)]
pub struct ValidationHarnessConfig {
    pub initial_state: EskfState,
    pub predict_config: PredictConfig,
    pub thresholds: ChiSquareThresholdConfig,
    pub ewma_alpha: f32,
}

impl ValidationHarnessConfig {
    pub const fn new(
        initial_state: EskfState,
        predict_config: PredictConfig,
        thresholds: ChiSquareThresholdConfig,
        ewma_alpha: f32,
    ) -> Self {
        Self {
            initial_state,
            predict_config,
            thresholds,
            ewma_alpha,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ValidationReport {
    pub total_rows: u64,
    pub gps_updates_processed: u64,
    pub spoof_labeled_updates: u64,
    pub clean_labeled_updates: u64,
    pub trusted_verdicts: u64,
    pub flagged_verdicts: u64,
    pub rejected_verdicts: u64,
    pub anomaly_true_positives: u64,
    pub anomaly_false_positives: u64,
    pub rejected_true_positives: u64,
    pub rejected_false_positives: u64,
}

impl ValidationReport {
    pub fn anomaly_true_positive_rate(&self) -> f64 {
        ratio(self.anomaly_true_positives, self.spoof_labeled_updates)
    }

    pub fn anomaly_false_positive_rate(&self) -> f64 {
        ratio(self.anomaly_false_positives, self.clean_labeled_updates)
    }

    pub fn rejected_true_positive_rate(&self) -> f64 {
        ratio(self.rejected_true_positives, self.spoof_labeled_updates)
    }

    pub fn rejected_false_positive_rate(&self) -> f64 {
        ratio(self.rejected_false_positives, self.clean_labeled_updates)
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct ValidationRow {
    pub timestamp_s: f64,
    pub imu_ax_mps2: f32,
    pub imu_ay_mps2: f32,
    pub imu_az_mps2: f32,
    pub imu_gx_rps: f32,
    pub imu_gy_rps: f32,
    pub imu_gz_rps: f32,
    #[serde(default)]
    pub gps_px_ned_m: Option<f32>,
    #[serde(default)]
    pub gps_py_ned_m: Option<f32>,
    #[serde(default)]
    pub gps_pz_ned_m: Option<f32>,
    #[serde(default)]
    pub gps_vx_ned_mps: Option<f32>,
    #[serde(default)]
    pub gps_vy_ned_mps: Option<f32>,
    #[serde(default)]
    pub gps_vz_ned_mps: Option<f32>,
    #[serde(default)]
    pub gps_horizontal_position_std_m: Option<f32>,
    #[serde(default)]
    pub gps_vertical_position_std_m: Option<f32>,
    #[serde(default)]
    pub gps_horizontal_velocity_std_mps: Option<f32>,
    #[serde(default)]
    pub gps_vertical_velocity_std_mps: Option<f32>,
    #[serde(default)]
    pub barometer_altitude_ned_down_m: Option<f32>,
    #[serde(default)]
    pub barometer_std_m: Option<f32>,
    #[serde(default)]
    pub heading_rad: Option<f32>,
    #[serde(default)]
    pub heading_std_rad: Option<f32>,
    #[serde(default)]
    pub label_spoofed: bool,
}

impl ValidationRow {
    fn imu_sample(&self) -> ImuSample {
        ImuSample::new(
            self.timestamp_s,
            Vector3::new(self.imu_ax_mps2, self.imu_ay_mps2, self.imu_az_mps2),
            Vector3::new(self.imu_gx_rps, self.imu_gy_rps, self.imu_gz_rps),
        )
    }

    fn gps_observation(&self) -> Result<Option<GpsObservation>, ValidationError> {
        let gps_position = match (self.gps_px_ned_m, self.gps_py_ned_m, self.gps_pz_ned_m) {
            (Some(px), Some(py), Some(pz)) => Vector3::new(px, py, pz),
            (None, None, None) => return Ok(None),
            _ => return Err(ValidationError::IncompleteGpsObservation),
        };
        let gps_velocity = match (self.gps_vx_ned_mps, self.gps_vy_ned_mps, self.gps_vz_ned_mps) {
            (Some(vx), Some(vy), Some(vz)) => Vector3::new(vx, vy, vz),
            _ => return Err(ValidationError::IncompleteGpsObservation),
        };
        let (
            Some(horizontal_position_std_m),
            Some(vertical_position_std_m),
            Some(horizontal_velocity_std_mps),
            Some(vertical_velocity_std_mps),
        ) = (
            self.gps_horizontal_position_std_m,
            self.gps_vertical_position_std_m,
            self.gps_horizontal_velocity_std_mps,
            self.gps_vertical_velocity_std_mps,
        )
        else {
            return Err(ValidationError::IncompleteGpsObservation);
        };

        Ok(Some(GpsObservation::from_accuracy_metrics(
            self.timestamp_s,
            gps_position,
            gps_velocity,
            horizontal_position_std_m,
            vertical_position_std_m,
            horizontal_velocity_std_mps,
            vertical_velocity_std_mps,
        )))
    }

    fn barometer_observation(&self) -> Option<BarometerObservation> {
        match (self.barometer_altitude_ned_down_m, self.barometer_std_m) {
            (Some(altitude_ned_down_m), Some(altitude_std_m)) => Some(BarometerObservation::new(
                self.timestamp_s,
                altitude_ned_down_m,
                altitude_std_m,
            )),
            _ => None,
        }
    }

    fn heading_observation(&self) -> Option<HeadingObservation> {
        match (self.heading_rad, self.heading_std_rad) {
            (Some(heading_rad), Some(heading_std_rad)) => Some(HeadingObservation::new(
                self.timestamp_s,
                heading_rad,
                heading_std_rad,
            )),
            _ => None,
        }
    }
}

#[derive(Debug)]
pub enum ValidationError {
    Io(std::io::Error),
    Csv(csv::Error),
    Predict(PredictError),
    Monitor(MonitorError),
    IncompleteGpsObservation,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "validation I/O error: {error}"),
            Self::Csv(error) => write!(f, "validation CSV error: {error}"),
            Self::Predict(error) => write!(f, "validation predict error: {error}"),
            Self::Monitor(error) => write!(f, "validation monitor error: {error}"),
            Self::IncompleteGpsObservation => {
                write!(f, "validation row had a partial GPS observation")
            }
        }
    }
}

pub fn run_csv_validation_file<P: AsRef<Path>>(
    path: P,
    config: &ValidationHarnessConfig,
) -> Result<ValidationReport, ValidationError> {
    let file = File::open(path).map_err(ValidationError::Io)?;
    run_csv_validation_reader(file, config)
}

pub fn run_csv_validation_reader<R: Read>(
    reader: R,
    config: &ValidationHarnessConfig,
) -> Result<ValidationReport, ValidationError> {
    let mut csv_reader = csv::Reader::from_reader(reader);
    let mut state = config.initial_state.clone();
    let mut monitor =
        StatisticalMonitor::new(config.thresholds, EwmaRiskAccumulator::new(config.ewma_alpha));
    let mut report = ValidationReport::default();

    for row in csv_reader.deserialize::<ValidationRow>() {
        let row = row.map_err(ValidationError::Csv)?;
        report.total_rows += 1;

        predict_in_place(&mut state, &config.predict_config, &row.imu_sample())
            .map_err(ValidationError::Predict)?;

        let Some(gps_observation) = row.gps_observation()? else {
            continue;
        };
        report.gps_updates_processed += 1;

        let verdict = monitor
            .evaluate_observations(
                &state,
                &gps_observation,
                row.barometer_observation().as_ref(),
                row.heading_observation().as_ref(),
            )
            .map_err(ValidationError::Monitor)?;

        match verdict.trust_level {
            TrustLevel::Trusted => report.trusted_verdicts += 1,
            TrustLevel::Flagged => report.flagged_verdicts += 1,
            TrustLevel::Rejected => report.rejected_verdicts += 1,
        }

        if row.label_spoofed {
            report.spoof_labeled_updates += 1;
            if matches!(verdict.trust_level, TrustLevel::Flagged | TrustLevel::Rejected) {
                report.anomaly_true_positives += 1;
            }
            if matches!(verdict.trust_level, TrustLevel::Rejected) {
                report.rejected_true_positives += 1;
            }
        } else {
            report.clean_labeled_updates += 1;
            if matches!(verdict.trust_level, TrustLevel::Flagged | TrustLevel::Rejected) {
                report.anomaly_false_positives += 1;
            }
            if matches!(verdict.trust_level, TrustLevel::Rejected) {
                report.rejected_false_positives += 1;
            }
        }
    }

    Ok(report)
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

    use super::{ValidationHarnessConfig, run_csv_validation_reader};
    use crate::{
        ekf_core::state::{EskfState, ImuNoiseModel, NominalState, PredictConfig, StateCovariance},
        statistical_monitor::observation::ChiSquareThresholdConfig,
    };

    #[test]
    fn csv_validation_reports_true_and_false_positive_rates() {
        let csv = "\
timestamp_s,imu_ax_mps2,imu_ay_mps2,imu_az_mps2,imu_gx_rps,imu_gy_rps,imu_gz_rps,gps_px_ned_m,gps_py_ned_m,gps_pz_ned_m,gps_vx_ned_mps,gps_vy_ned_mps,gps_vz_ned_mps,gps_horizontal_position_std_m,gps_vertical_position_std_m,gps_horizontal_velocity_std_mps,gps_vertical_velocity_std_mps,label_spoofed\n\
0.01,0.0,0.0,-9.80665,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.5,2.0,0.3,0.5,false\n\
0.02,0.0,0.0,-9.80665,0.0,0.0,0.0,150.0,-80.0,25.0,18.0,-9.0,3.0,1.5,2.0,0.3,0.5,true\n";
        let config = ValidationHarnessConfig::new(
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
            ),
            PredictConfig::new(
                Vector3::new(0.0, 0.0, 9.80665),
                0.02,
                ImuNoiseModel::new(
                    Vector3::new(0.05, 0.05, 0.05),
                    Vector3::new(0.002, 0.002, 0.002),
                    Vector3::new(0.0002, 0.0002, 0.0002),
                    Vector3::new(0.00002, 0.00002, 0.00002),
                ),
            ),
            ChiSquareThresholdConfig::new(12.592, 22.458),
            1.0,
        );

        let report = run_csv_validation_reader(Cursor::new(csv.as_bytes()), &config).unwrap();

        assert_eq!(report.gps_updates_processed, 2);
        assert_eq!(report.clean_labeled_updates, 1);
        assert_eq!(report.spoof_labeled_updates, 1);
        assert_eq!(report.anomaly_true_positive_rate(), 1.0);
        assert_eq!(report.anomaly_false_positive_rate(), 0.0);
        assert_eq!(report.rejected_true_positive_rate(), 1.0);
    }
}
