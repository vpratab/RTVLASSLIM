use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector3};

pub const ERROR_STATE_DIM: usize = 15;
pub const PROCESS_NOISE_DIM: usize = 12;

pub const POS_IDX: usize = 0;
pub const VEL_IDX: usize = 3;
pub const ATT_IDX: usize = 6;
pub const ACCEL_BIAS_IDX: usize = 9;
pub const GYRO_BIAS_IDX: usize = 12;

pub type ErrorStateVector = SVector<f32, ERROR_STATE_DIM>;
pub type StateCovariance = SMatrix<f32, ERROR_STATE_DIM, ERROR_STATE_DIM>;
pub type StateTransitionMatrix = SMatrix<f32, ERROR_STATE_DIM, ERROR_STATE_DIM>;
pub type ProcessNoiseCovariance = SMatrix<f32, PROCESS_NOISE_DIM, PROCESS_NOISE_DIM>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticReference {
    pub latitude_rad: f64,
    pub longitude_rad: f64,
    pub altitude_m: f64,
}

impl GeodeticReference {
    pub const fn new(latitude_rad: f64, longitude_rad: f64, altitude_m: f64) -> Self {
        Self {
            latitude_rad,
            longitude_rad,
            altitude_m,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub timestamp_s: f64,
    pub accel_body_mps2: Vector3<f32>,
    pub gyro_body_rps: Vector3<f32>,
}

impl ImuSample {
    pub const fn new(
        timestamp_s: f64,
        accel_body_mps2: Vector3<f32>,
        gyro_body_rps: Vector3<f32>,
    ) -> Self {
        Self {
            timestamp_s,
            accel_body_mps2,
            gyro_body_rps,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuNoiseModel {
    pub accel_noise_density_mps2_per_sqrt_hz: Vector3<f32>,
    pub gyro_noise_density_rps_per_sqrt_hz: Vector3<f32>,
    pub accel_bias_random_walk_mps2_per_sqrt_s: Vector3<f32>,
    pub gyro_bias_random_walk_rps_per_sqrt_s: Vector3<f32>,
}

impl ImuNoiseModel {
    pub const fn new(
        accel_noise_density_mps2_per_sqrt_hz: Vector3<f32>,
        gyro_noise_density_rps_per_sqrt_hz: Vector3<f32>,
        accel_bias_random_walk_mps2_per_sqrt_s: Vector3<f32>,
        gyro_bias_random_walk_rps_per_sqrt_s: Vector3<f32>,
    ) -> Self {
        Self {
            accel_noise_density_mps2_per_sqrt_hz,
            gyro_noise_density_rps_per_sqrt_hz,
            accel_bias_random_walk_mps2_per_sqrt_s,
            gyro_bias_random_walk_rps_per_sqrt_s,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PredictConfig {
    pub gravity_ned_mps2: Vector3<f32>,
    pub max_propagation_dt_s: f32,
    pub imu_noise: ImuNoiseModel,
}

impl PredictConfig {
    pub const fn new(
        gravity_ned_mps2: Vector3<f32>,
        max_propagation_dt_s: f32,
        imu_noise: ImuNoiseModel,
    ) -> Self {
        Self {
            gravity_ned_mps2,
            max_propagation_dt_s,
            imu_noise,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct NominalState {
    pub timestamp_s: f64,
    pub position_ned_m: Vector3<f32>,
    pub velocity_ned_mps: Vector3<f32>,
    pub attitude_body_to_ned: UnitQuaternion<f32>,
    pub accel_bias_mps2: Vector3<f32>,
    pub gyro_bias_rps: Vector3<f32>,
    pub geodetic_reference: Option<GeodeticReference>,
}

impl Default for NominalState {
    fn default() -> Self {
        Self {
            timestamp_s: 0.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        }
    }
}

impl NominalState {
    pub fn with_geodetic_reference(mut self, geodetic_reference: GeodeticReference) -> Self {
        self.geodetic_reference = Some(geodetic_reference);
        self
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct EskfState {
    pub nominal: NominalState,
    pub covariance: StateCovariance,
}

impl Default for EskfState {
    fn default() -> Self {
        Self {
            nominal: NominalState::default(),
            covariance: StateCovariance::identity() * 1.0e-3,
        }
    }
}

impl EskfState {
    pub fn new(nominal: NominalState, covariance: StateCovariance) -> Self {
        Self {
            nominal,
            covariance,
        }
    }

    pub fn from_diagonal_std(
        nominal: NominalState,
        position_std_m: Vector3<f32>,
        velocity_std_mps: Vector3<f32>,
        attitude_std_rad: Vector3<f32>,
        accel_bias_std_mps2: Vector3<f32>,
        gyro_bias_std_rps: Vector3<f32>,
    ) -> Self {
        let mut covariance = StateCovariance::zeros();
        covariance
            .fixed_view_mut::<3, 3>(POS_IDX, POS_IDX)
            .copy_from(&diagonal_from_std(position_std_m));
        covariance
            .fixed_view_mut::<3, 3>(VEL_IDX, VEL_IDX)
            .copy_from(&diagonal_from_std(velocity_std_mps));
        covariance
            .fixed_view_mut::<3, 3>(ATT_IDX, ATT_IDX)
            .copy_from(&diagonal_from_std(attitude_std_rad));
        covariance
            .fixed_view_mut::<3, 3>(ACCEL_BIAS_IDX, ACCEL_BIAS_IDX)
            .copy_from(&diagonal_from_std(accel_bias_std_mps2));
        covariance
            .fixed_view_mut::<3, 3>(GYRO_BIAS_IDX, GYRO_BIAS_IDX)
            .copy_from(&diagonal_from_std(gyro_bias_std_rps));

        Self {
            nominal,
            covariance,
        }
    }
}

fn diagonal_from_std(std: Vector3<f32>) -> SMatrix<f32, 3, 3> {
    let variances = std.component_mul(&std);
    SMatrix::<f32, 3, 3>::from_diagonal(&variances)
}
