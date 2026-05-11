use core::fmt;

use nalgebra::{Matrix3, UnitQuaternion, Vector3};

use super::state::{
    ACCEL_BIAS_IDX, ATT_IDX, EskfState, GYRO_BIAS_IDX, ImuSample, POS_IDX, PredictConfig,
    StateCovariance, StateTransitionMatrix, VEL_IDX,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PredictError {
    NonMonotonicTimestamp {
        previous_timestamp_s: f64,
        sample_timestamp_s: f64,
    },
    PropagationStepTooLarge {
        dt_s: f32,
        max_dt_s: f32,
    },
}

impl fmt::Display for PredictError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonMonotonicTimestamp {
                previous_timestamp_s,
                sample_timestamp_s,
            } => write!(
                f,
                "non-monotonic IMU timestamp: previous={previous_timestamp_s:.9}, sample={sample_timestamp_s:.9}"
            ),
            Self::PropagationStepTooLarge { dt_s, max_dt_s } => write!(
                f,
                "IMU propagation step {dt_s:.6}s exceeds configured maximum {max_dt_s:.6}s"
            ),
        }
    }
}

pub fn predict_in_place(
    state: &mut EskfState,
    config: &PredictConfig,
    imu_sample: &ImuSample,
) -> Result<(), PredictError> {
    if state.nominal.timestamp_s <= f64::EPSILON && imu_sample.timestamp_s > f64::EPSILON {
        state.nominal.timestamp_s = imu_sample.timestamp_s;
        return Ok(());
    }

    let dt_s = compute_dt_seconds(state.nominal.timestamp_s, imu_sample.timestamp_s)?;
    if dt_s > config.max_propagation_dt_s {
        return Err(PredictError::PropagationStepTooLarge {
            dt_s,
            max_dt_s: config.max_propagation_dt_s,
        });
    }

    if dt_s <= f32::EPSILON {
        state.nominal.timestamp_s = imu_sample.timestamp_s;
        return Ok(());
    }

    let unbiased_accel = imu_sample.accel_body_mps2 - state.nominal.accel_bias_mps2;
    let unbiased_gyro = imu_sample.gyro_body_rps - state.nominal.gyro_bias_rps;

    let previous_attitude = state.nominal.attitude_body_to_ned;
    let half_step_delta = UnitQuaternion::from_scaled_axis(unbiased_gyro * (0.5 * dt_s));
    let mid_attitude = previous_attitude * half_step_delta;
    let delta_attitude = UnitQuaternion::from_scaled_axis(unbiased_gyro * dt_s);
    let next_attitude = previous_attitude * delta_attitude;

    let accel_ned_mps2 = mid_attitude.transform_vector(&unbiased_accel) + config.gravity_ned_mps2;
    let previous_velocity = state.nominal.velocity_ned_mps;

    state.nominal.position_ned_m += previous_velocity * dt_s + accel_ned_mps2 * (0.5 * dt_s * dt_s);
    state.nominal.velocity_ned_mps += accel_ned_mps2 * dt_s;
    state.nominal.attitude_body_to_ned = next_attitude;
    state.nominal.timestamp_s = imu_sample.timestamp_s;

    let rotation_mid = mid_attitude.to_rotation_matrix().into_inner();
    let transition = discrete_state_transition(&rotation_mid, unbiased_accel, unbiased_gyro, dt_s);
    let process_noise = discrete_process_noise(config, &rotation_mid, dt_s);
    let propagated_covariance =
        transition * state.covariance * transition.transpose() + process_noise;
    state.covariance = symmetrize(propagated_covariance);

    Ok(())
}

fn compute_dt_seconds(
    previous_timestamp_s: f64,
    sample_timestamp_s: f64,
) -> Result<f32, PredictError> {
    let dt_s = sample_timestamp_s - previous_timestamp_s;
    if dt_s < 0.0 {
        return Err(PredictError::NonMonotonicTimestamp {
            previous_timestamp_s,
            sample_timestamp_s,
        });
    }

    Ok(dt_s as f32)
}

fn discrete_state_transition(
    rotation_body_to_ned: &Matrix3<f32>,
    unbiased_accel_body_mps2: Vector3<f32>,
    unbiased_gyro_body_rps: Vector3<f32>,
    dt_s: f32,
) -> StateTransitionMatrix {
    let mut continuous = StateTransitionMatrix::zeros();
    continuous
        .fixed_view_mut::<3, 3>(POS_IDX, VEL_IDX)
        .copy_from(&Matrix3::identity());
    continuous
        .fixed_view_mut::<3, 3>(VEL_IDX, ATT_IDX)
        .copy_from(&(-rotation_body_to_ned * skew_symmetric(unbiased_accel_body_mps2)));
    continuous
        .fixed_view_mut::<3, 3>(VEL_IDX, ACCEL_BIAS_IDX)
        .copy_from(&(-rotation_body_to_ned));
    continuous
        .fixed_view_mut::<3, 3>(ATT_IDX, ATT_IDX)
        .copy_from(&(-skew_symmetric(unbiased_gyro_body_rps)));
    continuous
        .fixed_view_mut::<3, 3>(ATT_IDX, GYRO_BIAS_IDX)
        .copy_from(&(-Matrix3::identity()));

    let identity = StateTransitionMatrix::identity();
    let continuous_squared = continuous * continuous;
    identity + continuous * dt_s + continuous_squared * (0.5 * dt_s * dt_s)
}

fn discrete_process_noise(
    config: &PredictConfig,
    rotation_body_to_ned: &Matrix3<f32>,
    dt_s: f32,
) -> StateCovariance {
    let accel_spectral_density_body =
        diagonal_square(config.imu_noise.accel_noise_density_mps2_per_sqrt_hz);
    let accel_spectral_density_ned =
        rotation_body_to_ned * accel_spectral_density_body * rotation_body_to_ned.transpose();
    let gyro_spectral_density =
        diagonal_square(config.imu_noise.gyro_noise_density_rps_per_sqrt_hz);
    let accel_bias_spectral_density =
        diagonal_square(config.imu_noise.accel_bias_random_walk_mps2_per_sqrt_s);
    let gyro_bias_spectral_density =
        diagonal_square(config.imu_noise.gyro_bias_random_walk_rps_per_sqrt_s);

    let dt2 = dt_s * dt_s;
    let dt3 = dt2 * dt_s;

    let mut qd = StateCovariance::zeros();
    qd.fixed_view_mut::<3, 3>(POS_IDX, POS_IDX)
        .copy_from(&(accel_spectral_density_ned * (dt3 / 3.0)));
    qd.fixed_view_mut::<3, 3>(POS_IDX, VEL_IDX)
        .copy_from(&(accel_spectral_density_ned * (dt2 * 0.5)));
    qd.fixed_view_mut::<3, 3>(VEL_IDX, POS_IDX)
        .copy_from(&(accel_spectral_density_ned * (dt2 * 0.5)));
    qd.fixed_view_mut::<3, 3>(VEL_IDX, VEL_IDX)
        .copy_from(&(accel_spectral_density_ned * dt_s));
    qd.fixed_view_mut::<3, 3>(ATT_IDX, ATT_IDX)
        .copy_from(&(gyro_spectral_density * dt_s));
    qd.fixed_view_mut::<3, 3>(ACCEL_BIAS_IDX, ACCEL_BIAS_IDX)
        .copy_from(&(accel_bias_spectral_density * dt_s));
    qd.fixed_view_mut::<3, 3>(GYRO_BIAS_IDX, GYRO_BIAS_IDX)
        .copy_from(&(gyro_bias_spectral_density * dt_s));

    qd
}

fn diagonal_square(components: Vector3<f32>) -> Matrix3<f32> {
    Matrix3::from_diagonal(&components.component_mul(&components))
}

fn skew_symmetric(v: Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

fn symmetrize(covariance: StateCovariance) -> StateCovariance {
    (covariance + covariance.transpose()) * 0.5
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::predict_in_place;
    use crate::ekf_core::state::{
        EskfState, ImuNoiseModel, ImuSample, NominalState, PredictConfig,
    };

    #[test]
    fn stationary_specific_force_keeps_state_at_rest() {
        let imu_noise = ImuNoiseModel::new(
            Vector3::new(0.05, 0.05, 0.05),
            Vector3::new(0.002, 0.002, 0.002),
            Vector3::new(0.0002, 0.0002, 0.0002),
            Vector3::new(0.00002, 0.00002, 0.00002),
        );
        let config = PredictConfig::new(Vector3::new(0.0, 0.0, 9.80665), 0.02, imu_noise);
        let nominal = NominalState {
            timestamp_s: 0.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        };
        let mut state = EskfState::new(nominal, super::StateCovariance::identity() * 1.0e-4);

        for step in 1..=100 {
            let timestamp_s = f64::from(step) * 0.01;
            let sample = ImuSample::new(
                timestamp_s,
                Vector3::new(0.0, 0.0, -9.80665),
                Vector3::zeros(),
            );
            predict_in_place(&mut state, &config, &sample).unwrap();
        }

        assert!(state.nominal.position_ned_m.norm() < 1.0e-5);
        assert!(state.nominal.velocity_ned_mps.norm() < 1.0e-5);
        assert!(state.nominal.attitude_body_to_ned.angle() < 1.0e-6);
        assert!(state.covariance.trace() > 0.0);
    }

    #[test]
    fn first_nonzero_timestamp_bootstraps_without_large_dt_error() {
        let imu_noise = ImuNoiseModel::new(
            Vector3::new(0.05, 0.05, 0.05),
            Vector3::new(0.002, 0.002, 0.002),
            Vector3::new(0.0002, 0.0002, 0.0002),
            Vector3::new(0.00002, 0.00002, 0.00002),
        );
        let config = PredictConfig::new(Vector3::new(0.0, 0.0, 9.80665), 0.02, imu_noise);
        let nominal = NominalState {
            timestamp_s: 0.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        };
        let mut state = EskfState::new(nominal, super::StateCovariance::identity() * 1.0e-4);
        let sample = ImuSample::new(3.18, Vector3::new(0.0, 0.0, -9.80665), Vector3::zeros());

        predict_in_place(&mut state, &config, &sample).unwrap();

        assert!((state.nominal.timestamp_s - 3.18).abs() < 1.0e-9);
        assert!(state.nominal.position_ned_m.norm() < 1.0e-6);
        assert!(state.nominal.velocity_ned_mps.norm() < 1.0e-6);
    }
}
