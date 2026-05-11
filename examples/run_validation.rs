use std::path::PathBuf;

use nalgebra::{UnitQuaternion, Vector3};

use rtvlas::{
    ekf_core::state::{EskfState, ImuNoiseModel, NominalState, PredictConfig, StateCovariance},
    statistical_monitor::observation::ChiSquareThresholdConfig,
    validation::{ValidationHarnessConfig, run_csv_validation_file},
};

fn main() {
    let path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("examples/synthetic_validation.csv"));
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

    let report = run_csv_validation_file(&path, &config).expect("validation run");
    println!("Validation file: {}", path.display());
    println!("Rows: {}", report.total_rows);
    println!("GPS updates: {}", report.gps_updates_processed);
    println!(
        "Trusted/Flagged/Rejected: {}/{}/{}",
        report.trusted_verdicts, report.flagged_verdicts, report.rejected_verdicts
    );
    println!(
        "Anomaly TPR/FPR: {:.3}/{:.3}",
        report.anomaly_true_positive_rate(),
        report.anomaly_false_positive_rate()
    );
    println!(
        "Rejected TPR/FPR: {:.3}/{:.3}",
        report.rejected_true_positive_rate(),
        report.rejected_false_positive_rate()
    );
}
