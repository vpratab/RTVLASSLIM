use nalgebra::{SMatrix, SVector, Vector3};

use crate::ekf_core::state::{ATT_IDX, ERROR_STATE_DIM, POS_IDX, VEL_IDX};

pub const GPS_OBSERVATION_DIM: usize = 6;
pub const BAROMETER_OBSERVATION_DIM: usize = 1;
pub const HEADING_OBSERVATION_DIM: usize = 1;
pub const CLOCK_BIAS_OBSERVATION_DIM: usize = 1;

pub type ObservationVector = SVector<f32, GPS_OBSERVATION_DIM>;
pub type ObservationNoiseMatrix = SMatrix<f32, GPS_OBSERVATION_DIM, GPS_OBSERVATION_DIM>;
pub type ObservationJacobian = SMatrix<f32, GPS_OBSERVATION_DIM, ERROR_STATE_DIM>;
pub type InnovationVector = ObservationVector;
pub type InnovationCovariance = ObservationNoiseMatrix;
pub type ScalarObservationNoiseMatrix = SMatrix<f32, 1, 1>;
pub type ScalarObservationJacobian = SMatrix<f32, 1, ERROR_STATE_DIM>;

#[derive(Clone, Debug, PartialEq)]
pub struct GpsObservation {
    pub timestamp_s: f64,
    pub position_ned_m: Vector3<f32>,
    pub velocity_ned_mps: Vector3<f32>,
    pub observation_noise: ObservationNoiseMatrix,
}

impl GpsObservation {
    pub fn new(
        timestamp_s: f64,
        position_ned_m: Vector3<f32>,
        velocity_ned_mps: Vector3<f32>,
        observation_noise: ObservationNoiseMatrix,
    ) -> Self {
        Self {
            timestamp_s,
            position_ned_m,
            velocity_ned_mps,
            observation_noise,
        }
    }

    pub fn from_accuracy_metrics(
        timestamp_s: f64,
        position_ned_m: Vector3<f32>,
        velocity_ned_mps: Vector3<f32>,
        horizontal_position_std_m: f32,
        vertical_position_std_m: f32,
        horizontal_velocity_std_mps: f32,
        vertical_velocity_std_mps: f32,
    ) -> Self {
        let position_std = Vector3::new(
            horizontal_position_std_m,
            horizontal_position_std_m,
            vertical_position_std_m,
        );
        let velocity_std = Vector3::new(
            horizontal_velocity_std_mps,
            horizontal_velocity_std_mps,
            vertical_velocity_std_mps,
        );

        let mut observation_noise = ObservationNoiseMatrix::zeros();
        observation_noise.fixed_view_mut::<3, 3>(0, 0).copy_from(
            &SMatrix::<f32, 3, 3>::from_diagonal(&position_std.component_mul(&position_std)),
        );
        observation_noise.fixed_view_mut::<3, 3>(3, 3).copy_from(
            &SMatrix::<f32, 3, 3>::from_diagonal(&velocity_std.component_mul(&velocity_std)),
        );

        Self::new(
            timestamp_s,
            position_ned_m,
            velocity_ned_mps,
            observation_noise,
        )
    }

    pub fn measurement_vector(&self) -> ObservationVector {
        let mut measurement = ObservationVector::zeros();
        measurement
            .fixed_rows_mut::<3>(0)
            .copy_from(&self.position_ned_m);
        measurement
            .fixed_rows_mut::<3>(3)
            .copy_from(&self.velocity_ned_mps);
        measurement
    }

    pub fn observation_matrix() -> ObservationJacobian {
        let mut observation_matrix = ObservationJacobian::zeros();
        observation_matrix
            .fixed_view_mut::<3, 3>(0, POS_IDX)
            .copy_from(&SMatrix::<f32, 3, 3>::identity());
        observation_matrix
            .fixed_view_mut::<3, 3>(3, VEL_IDX)
            .copy_from(&SMatrix::<f32, 3, 3>::identity());
        observation_matrix
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BarometerObservation {
    pub timestamp_s: f64,
    pub altitude_ned_down_m: f32,
    pub altitude_std_m: f32,
}

impl BarometerObservation {
    pub const fn new(timestamp_s: f64, altitude_ned_down_m: f32, altitude_std_m: f32) -> Self {
        Self {
            timestamp_s,
            altitude_ned_down_m,
            altitude_std_m,
        }
    }

    pub fn observation_noise(&self) -> ScalarObservationNoiseMatrix {
        ScalarObservationNoiseMatrix::from_element(self.altitude_std_m * self.altitude_std_m)
    }

    pub fn observation_matrix() -> ScalarObservationJacobian {
        let mut observation_matrix = ScalarObservationJacobian::zeros();
        observation_matrix[(0, POS_IDX + 2)] = 1.0;
        observation_matrix
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadingObservation {
    pub timestamp_s: f64,
    pub heading_rad: f32,
    pub heading_std_rad: f32,
}

impl HeadingObservation {
    pub const fn new(timestamp_s: f64, heading_rad: f32, heading_std_rad: f32) -> Self {
        Self {
            timestamp_s,
            heading_rad,
            heading_std_rad,
        }
    }

    pub fn observation_noise(&self) -> ScalarObservationNoiseMatrix {
        ScalarObservationNoiseMatrix::from_element(self.heading_std_rad * self.heading_std_rad)
    }

    pub fn observation_matrix() -> ScalarObservationJacobian {
        let mut observation_matrix = ScalarObservationJacobian::zeros();
        observation_matrix[(0, ATT_IDX + 2)] = 1.0;
        observation_matrix
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockBiasObservation {
    pub timestamp_s: f64,
    pub reference_clock_bias_m: f32,
    pub observed_clock_bias_m: f32,
    pub clock_bias_std_m: f32,
}

impl ClockBiasObservation {
    pub const fn new(
        timestamp_s: f64,
        reference_clock_bias_m: f32,
        observed_clock_bias_m: f32,
        clock_bias_std_m: f32,
    ) -> Self {
        Self {
            timestamp_s,
            reference_clock_bias_m,
            observed_clock_bias_m,
            clock_bias_std_m,
        }
    }

    pub fn observation_noise(&self) -> ScalarObservationNoiseMatrix {
        ScalarObservationNoiseMatrix::from_element(self.clock_bias_std_m * self.clock_bias_std_m)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChiSquareThresholdConfig {
    pub flagged_risk_threshold: f32,
    pub rejected_risk_threshold: f32,
}

impl ChiSquareThresholdConfig {
    pub const fn new(flagged_risk_threshold: f32, rejected_risk_threshold: f32) -> Self {
        Self {
            flagged_risk_threshold,
            rejected_risk_threshold,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TrustLevel {
    Trusted,
    Flagged,
    Rejected,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MonitorVerdict {
    pub squared_mahalanobis_distance: f32,
    pub gps_squared_mahalanobis_distance: f32,
    pub barometer_squared_mahalanobis_distance: Option<f32>,
    pub heading_squared_mahalanobis_distance: Option<f32>,
    pub clock_bias_squared_mahalanobis_distance: Option<f32>,
    pub clock_bias_persistent_score: Option<f32>,
    pub horizontal_residual_persistent_score: Option<f32>,
    pub velocity_residual_persistent_score: Option<f32>,
    pub stale_gps_persistent_score: Option<f32>,
    pub accumulated_risk: f32,
    pub innovation: InnovationVector,
    pub barometer_residual_m: Option<f32>,
    pub heading_residual_rad: Option<f32>,
    pub clock_bias_residual_m: Option<f32>,
    pub trust_level: TrustLevel,
}
