use core::fmt;

use nalgebra::linalg::Cholesky;

use crate::ekf_core::state::EskfState;

use super::observation::{
    ChiSquareThresholdConfig, GpsObservation, InnovationCovariance, InnovationVector,
    MonitorVerdict, ObservationJacobian, TrustLevel,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EwmaRiskAccumulator {
    alpha: f32,
    risk: f32,
}

impl EwmaRiskAccumulator {
    pub const fn new(alpha: f32) -> Self {
        Self { alpha, risk: 0.0 }
    }

    pub const fn alpha(&self) -> f32 {
        self.alpha
    }

    pub const fn risk(&self) -> f32 {
        self.risk
    }

    pub fn reset(&mut self) {
        self.risk = 0.0;
    }

    pub fn update(&mut self, squared_mahalanobis_distance: f32) -> f32 {
        self.risk = self.alpha * squared_mahalanobis_distance + (1.0 - self.alpha) * self.risk;
        self.risk
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StatisticalMonitor {
    thresholds: ChiSquareThresholdConfig,
    risk_accumulator: EwmaRiskAccumulator,
}

impl StatisticalMonitor {
    pub const fn new(
        thresholds: ChiSquareThresholdConfig,
        risk_accumulator: EwmaRiskAccumulator,
    ) -> Self {
        Self {
            thresholds,
            risk_accumulator,
        }
    }

    pub const fn thresholds(&self) -> ChiSquareThresholdConfig {
        self.thresholds
    }

    pub const fn risk_accumulator(&self) -> EwmaRiskAccumulator {
        self.risk_accumulator
    }

    pub fn evaluate_gps_observation(
        &mut self,
        predicted_state: &EskfState,
        gps_observation: &GpsObservation,
    ) -> Result<MonitorVerdict, MonitorError> {
        validate_thresholds(self.thresholds)?;
        validate_ewma_alpha(self.risk_accumulator.alpha())?;

        let predicted_measurement = predicted_measurement(predicted_state);
        let innovation = gps_observation.measurement_vector() - predicted_measurement;
        let observation_matrix = GpsObservation::observation_matrix();
        let innovation_covariance = innovation_covariance(
            &observation_matrix,
            &predicted_state.covariance,
            &gps_observation.observation_noise,
        );

        let squared_mahalanobis_distance =
            squared_mahalanobis_distance(innovation, innovation_covariance)?;
        let accumulated_risk = self.risk_accumulator.update(squared_mahalanobis_distance);
        let trust_level = classify_trust_level(accumulated_risk, self.thresholds);

        Ok(MonitorVerdict {
            squared_mahalanobis_distance,
            accumulated_risk,
            innovation,
            trust_level,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MonitorError {
    InvalidEwmaAlpha {
        alpha: f32,
    },
    InvalidThresholdConfig {
        flagged_risk_threshold: f32,
        rejected_risk_threshold: f32,
    },
    SingularInnovationCovariance,
}

impl fmt::Display for MonitorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidEwmaAlpha { alpha } => write!(
                f,
                "EWMA alpha must be in the closed interval [0, 1], got {alpha:.6}"
            ),
            Self::InvalidThresholdConfig {
                flagged_risk_threshold,
                rejected_risk_threshold,
            } => write!(
                f,
                "invalid chi-square thresholds: flagged={flagged_risk_threshold:.6}, rejected={rejected_risk_threshold:.6}"
            ),
            Self::SingularInnovationCovariance => write!(
                f,
                "innovation covariance is not positive definite; Cholesky factorization failed"
            ),
        }
    }
}

fn validate_thresholds(thresholds: ChiSquareThresholdConfig) -> Result<(), MonitorError> {
    if thresholds.flagged_risk_threshold <= 0.0
        || thresholds.rejected_risk_threshold <= thresholds.flagged_risk_threshold
    {
        return Err(MonitorError::InvalidThresholdConfig {
            flagged_risk_threshold: thresholds.flagged_risk_threshold,
            rejected_risk_threshold: thresholds.rejected_risk_threshold,
        });
    }

    Ok(())
}

fn validate_ewma_alpha(alpha: f32) -> Result<(), MonitorError> {
    if !(0.0..=1.0).contains(&alpha) {
        return Err(MonitorError::InvalidEwmaAlpha { alpha });
    }

    Ok(())
}

fn predicted_measurement(predicted_state: &EskfState) -> InnovationVector {
    let mut predicted = InnovationVector::zeros();
    predicted
        .fixed_rows_mut::<3>(0)
        .copy_from(&predicted_state.nominal.position_ned_m);
    predicted
        .fixed_rows_mut::<3>(3)
        .copy_from(&predicted_state.nominal.velocity_ned_mps);
    predicted
}

fn innovation_covariance(
    observation_matrix: &ObservationJacobian,
    state_covariance: &crate::ekf_core::state::StateCovariance,
    observation_noise: &InnovationCovariance,
) -> InnovationCovariance {
    observation_matrix * state_covariance * observation_matrix.transpose() + observation_noise
}

fn squared_mahalanobis_distance(
    innovation: InnovationVector,
    innovation_covariance: InnovationCovariance,
) -> Result<f32, MonitorError> {
    let Some(cholesky) = Cholesky::new(innovation_covariance) else {
        return Err(MonitorError::SingularInnovationCovariance);
    };

    let innovation_whitened = cholesky.solve(&innovation);
    Ok(innovation.dot(&innovation_whitened))
}

fn classify_trust_level(accumulated_risk: f32, thresholds: ChiSquareThresholdConfig) -> TrustLevel {
    if accumulated_risk >= thresholds.rejected_risk_threshold {
        TrustLevel::Rejected
    } else if accumulated_risk >= thresholds.flagged_risk_threshold {
        TrustLevel::Flagged
    } else {
        TrustLevel::Trusted
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::{EwmaRiskAccumulator, StatisticalMonitor};
    use crate::{
        ekf_core::state::{EskfState, NominalState, StateCovariance},
        statistical_monitor::observation::{ChiSquareThresholdConfig, GpsObservation, TrustLevel},
    };

    #[test]
    fn drifting_gps_observation_triggers_rejected_verdict() {
        let nominal = NominalState {
            timestamp_s: 10.0,
            position_ned_m: Vector3::new(0.0, 0.0, 0.0),
            velocity_ned_mps: Vector3::new(0.0, 0.0, 0.0),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        };
        let predicted_state = EskfState::new(nominal, StateCovariance::identity() * 1.0e-3);
        let gps_observation = GpsObservation::from_accuracy_metrics(
            10.1,
            Vector3::new(150.0, -80.0, 25.0),
            Vector3::new(18.0, -9.0, 3.0),
            1.5,
            2.0,
            0.3,
            0.5,
        );

        let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
        let risk_accumulator = EwmaRiskAccumulator::new(0.5);
        let mut monitor = StatisticalMonitor::new(thresholds, risk_accumulator);

        let verdict = monitor
            .evaluate_gps_observation(&predicted_state, &gps_observation)
            .unwrap();

        assert!(verdict.squared_mahalanobis_distance > thresholds.rejected_risk_threshold);
        assert!(verdict.accumulated_risk > thresholds.rejected_risk_threshold);
        assert_eq!(verdict.trust_level, TrustLevel::Rejected);
        assert!(verdict.innovation.fixed_rows::<3>(0).norm() > 100.0);
    }
}
