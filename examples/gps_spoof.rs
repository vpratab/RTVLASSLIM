use nalgebra::{UnitQuaternion, Vector3};

use rtvlas::{
    ekf_core::{
        predict::predict_in_place,
        state::{EskfState, ImuNoiseModel, ImuSample, NominalState, PredictConfig, StateCovariance},
    },
    statistical_monitor::{
        monitor::{EwmaRiskAccumulator, StatisticalMonitor},
        observation::{
            BarometerObservation, ChiSquareThresholdConfig, GpsObservation, HeadingObservation,
        },
    },
};

fn main() {
    let mut state = EskfState::new(
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
    );
    let predict_config = PredictConfig::new(
        Vector3::new(0.0, 0.0, 9.80665),
        0.02,
        ImuNoiseModel::new(
            Vector3::new(0.05, 0.05, 0.05),
            Vector3::new(0.002, 0.002, 0.002),
            Vector3::new(0.0002, 0.0002, 0.0002),
            Vector3::new(0.00002, 0.00002, 0.00002),
        ),
    );
    let mut monitor = StatisticalMonitor::new(
        ChiSquareThresholdConfig::new(12.592, 22.458),
        EwmaRiskAccumulator::new(0.6),
    );

    println!("Synthetic spoofing run with GPS, barometer, and heading checks");

    for step in 1..=20 {
        let timestamp_s = f64::from(step) * 0.01;
        let imu_sample = ImuSample::new(
            timestamp_s,
            Vector3::new(0.0, 0.0, -9.80665),
            Vector3::zeros(),
        );
        predict_in_place(&mut state, &predict_config, &imu_sample).expect("predict step");

        let spoofed = step >= 12;
        let gps_position = if spoofed {
            Vector3::new(90.0, -40.0, 12.0)
        } else {
            Vector3::zeros()
        };
        let heading_rad = if spoofed { 1.2 } else { 0.0 };
        let gps_observation = GpsObservation::from_accuracy_metrics(
            timestamp_s,
            gps_position,
            Vector3::zeros(),
            1.5,
            2.0,
            0.3,
            0.5,
        );
        let barometer_observation = BarometerObservation::new(timestamp_s, 0.0, 0.8);
        let heading_observation = HeadingObservation::new(timestamp_s, heading_rad, 0.08);

        let verdict = monitor
            .evaluate_observations(
                &state,
                &gps_observation,
                Some(&barometer_observation),
                Some(&heading_observation),
            )
            .expect("monitor evaluation");

        println!(
            "t={timestamp_s:>4.2}s spoofed={spoofed:<5} verdict={:?} total_d2={:>8.2} gps_d2={:>8.2} baro_d2={:>8.2?} heading_d2={:>8.2?}",
            verdict.trust_level,
            verdict.squared_mahalanobis_distance,
            verdict.gps_squared_mahalanobis_distance,
            verdict.barometer_squared_mahalanobis_distance,
            verdict.heading_squared_mahalanobis_distance,
        );
    }
}
