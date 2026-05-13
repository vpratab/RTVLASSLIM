use core::fmt;

use libm::sqrtf;
use nalgebra::linalg::Cholesky;

use crate::ekf_core::state::EskfState;

use super::observation::{
    BarometerObservation, ChiSquareThresholdConfig, ClockBiasObservation, GpsObservation,
    HeadingObservation, InnovationCovariance, InnovationVector, MonitorVerdict,
    ObservationJacobian, TrustLevel,
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
pub struct ClockBiasPersistenceConfig {
    pub slack_sigma: f32,
    pub rejection_score_threshold: f32,
}

impl ClockBiasPersistenceConfig {
    pub const fn new(slack_sigma: f32, rejection_score_threshold: f32) -> Self {
        Self {
            slack_sigma,
            rejection_score_threshold,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HorizontalResidualPersistenceConfig {
    pub slack_sigma: f32,
    pub rejection_score_threshold: f32,
}

impl HorizontalResidualPersistenceConfig {
    pub const fn new(slack_sigma: f32, rejection_score_threshold: f32) -> Self {
        Self {
            slack_sigma,
            rejection_score_threshold,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImmediateTriggerConfig {
    pub gps_flag_squared_mahalanobis_threshold: Option<f32>,
    pub gps_reject_squared_mahalanobis_threshold: Option<f32>,
    pub total_flag_squared_mahalanobis_threshold: Option<f32>,
    pub total_reject_squared_mahalanobis_threshold: Option<f32>,
    pub position_residual_flag_threshold_m: Option<f32>,
    pub position_residual_reject_threshold_m: Option<f32>,
}

impl ImmediateTriggerConfig {
    pub const fn new(
        gps_flag_squared_mahalanobis_threshold: Option<f32>,
        gps_reject_squared_mahalanobis_threshold: Option<f32>,
        total_flag_squared_mahalanobis_threshold: Option<f32>,
        total_reject_squared_mahalanobis_threshold: Option<f32>,
    ) -> Self {
        Self {
            gps_flag_squared_mahalanobis_threshold,
            gps_reject_squared_mahalanobis_threshold,
            total_flag_squared_mahalanobis_threshold,
            total_reject_squared_mahalanobis_threshold,
            position_residual_flag_threshold_m: None,
            position_residual_reject_threshold_m: None,
        }
    }

    pub const fn gps_only(
        gps_flag_squared_mahalanobis_threshold: Option<f32>,
        gps_reject_squared_mahalanobis_threshold: Option<f32>,
    ) -> Self {
        Self::new(
            gps_flag_squared_mahalanobis_threshold,
            gps_reject_squared_mahalanobis_threshold,
            None,
            None,
        )
    }

    pub const fn with_position_residual_thresholds(
        mut self,
        position_residual_flag_threshold_m: Option<f32>,
        position_residual_reject_threshold_m: Option<f32>,
    ) -> Self {
        self.position_residual_flag_threshold_m = position_residual_flag_threshold_m;
        self.position_residual_reject_threshold_m = position_residual_reject_threshold_m;
        self
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ClockBiasPersistenceState {
    config: ClockBiasPersistenceConfig,
    score: f32,
}

impl ClockBiasPersistenceState {
    const fn new(config: ClockBiasPersistenceConfig) -> Self {
        Self { config, score: 0.0 }
    }

    fn update(&mut self, normalized_absolute_residual: f32) -> f32 {
        self.score = (self.score + normalized_absolute_residual - self.config.slack_sigma).max(0.0);
        self.score
    }

    fn reset(&mut self) {
        self.score = 0.0;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct HorizontalResidualPersistenceState {
    config: HorizontalResidualPersistenceConfig,
    score: f32,
}

impl HorizontalResidualPersistenceState {
    const fn new(config: HorizontalResidualPersistenceConfig) -> Self {
        Self { config, score: 0.0 }
    }

    fn update(&mut self, normalized_horizontal_residual: f32) -> f32 {
        self.score =
            (self.score + normalized_horizontal_residual - self.config.slack_sigma).max(0.0);
        self.score
    }

    fn reset(&mut self) {
        self.score = 0.0;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StatisticalMonitor {
    thresholds: ChiSquareThresholdConfig,
    risk_accumulator: EwmaRiskAccumulator,
    clock_bias_persistence: Option<ClockBiasPersistenceState>,
    horizontal_residual_persistence: Option<HorizontalResidualPersistenceState>,
    horizontal_residual_normalization_std_override_m: Option<f32>,
    immediate_triggers: Option<ImmediateTriggerConfig>,
}

impl StatisticalMonitor {
    pub const fn new(
        thresholds: ChiSquareThresholdConfig,
        risk_accumulator: EwmaRiskAccumulator,
    ) -> Self {
        Self {
            thresholds,
            risk_accumulator,
            clock_bias_persistence: None,
            horizontal_residual_persistence: None,
            horizontal_residual_normalization_std_override_m: None,
            immediate_triggers: None,
        }
    }

    pub fn with_clock_bias_persistence(mut self, config: ClockBiasPersistenceConfig) -> Self {
        self.clock_bias_persistence = Some(ClockBiasPersistenceState::new(config));
        self
    }

    pub fn with_horizontal_residual_persistence(
        mut self,
        config: HorizontalResidualPersistenceConfig,
    ) -> Self {
        self.horizontal_residual_persistence =
            Some(HorizontalResidualPersistenceState::new(config));
        self
    }

    pub fn set_horizontal_residual_persistence(
        &mut self,
        config: Option<HorizontalResidualPersistenceConfig>,
    ) {
        self.horizontal_residual_persistence = config.map(HorizontalResidualPersistenceState::new);
    }

    pub fn set_horizontal_residual_normalization_std_override_m(
        &mut self,
        horizontal_residual_normalization_std_override_m: Option<f32>,
    ) {
        self.horizontal_residual_normalization_std_override_m =
            horizontal_residual_normalization_std_override_m.map(|value| value.max(1.0e-6));
    }

    pub fn with_immediate_triggers(mut self, config: ImmediateTriggerConfig) -> Self {
        self.immediate_triggers = Some(config);
        self
    }

    pub const fn thresholds(&self) -> ChiSquareThresholdConfig {
        self.thresholds
    }

    pub const fn risk_accumulator(&self) -> EwmaRiskAccumulator {
        self.risk_accumulator
    }

    pub fn reset_runtime_state(&mut self) {
        self.risk_accumulator.reset();
        if let Some(clock_bias_persistence) = &mut self.clock_bias_persistence {
            clock_bias_persistence.reset();
        }
        if let Some(horizontal_residual_persistence) = &mut self.horizontal_residual_persistence {
            horizontal_residual_persistence.reset();
        }
    }

    pub fn evaluate_gps_observation(
        &mut self,
        predicted_state: &EskfState,
        gps_observation: &GpsObservation,
    ) -> Result<MonitorVerdict, MonitorError> {
        self.evaluate_observations(predicted_state, gps_observation, None, None)
    }

    pub fn evaluate_observations(
        &mut self,
        predicted_state: &EskfState,
        gps_observation: &GpsObservation,
        barometer_observation: Option<&BarometerObservation>,
        heading_observation: Option<&HeadingObservation>,
    ) -> Result<MonitorVerdict, MonitorError> {
        self.evaluate_observations_with_clock(
            predicted_state,
            gps_observation,
            barometer_observation,
            heading_observation,
            None,
        )
    }

    pub fn evaluate_observations_with_clock(
        &mut self,
        predicted_state: &EskfState,
        gps_observation: &GpsObservation,
        barometer_observation: Option<&BarometerObservation>,
        heading_observation: Option<&HeadingObservation>,
        clock_bias_observation: Option<&ClockBiasObservation>,
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

        let gps_squared_mahalanobis_distance =
            squared_mahalanobis_distance(innovation, innovation_covariance)?;
        let (barometer_squared_mahalanobis_distance, barometer_residual_m) =
            match barometer_observation {
                Some(barometer_observation) => {
                    let barometer_residual_m = barometer_observation.altitude_ned_down_m
                        - predicted_state.nominal.position_ned_m.z;
                    let innovation_variance = scalar_innovation_variance(
                        &BarometerObservation::observation_matrix(),
                        &predicted_state.covariance,
                        barometer_observation.observation_noise()[(0, 0)],
                    );
                    let squared_mahalanobis_distance = scalar_squared_mahalanobis_distance(
                        barometer_residual_m,
                        innovation_variance,
                        "barometer",
                    )?;
                    (
                        Some(squared_mahalanobis_distance),
                        Some(barometer_residual_m),
                    )
                }
                None => (None, None),
            };
        let (heading_squared_mahalanobis_distance, heading_residual_rad) = match heading_observation
        {
            Some(heading_observation) => {
                let predicted_heading = predicted_heading_rad(predicted_state);
                let heading_residual_rad =
                    wrap_angle_pi(heading_observation.heading_rad - predicted_heading);
                let innovation_variance = scalar_innovation_variance(
                    &HeadingObservation::observation_matrix(),
                    &predicted_state.covariance,
                    heading_observation.observation_noise()[(0, 0)],
                );
                let squared_mahalanobis_distance = scalar_squared_mahalanobis_distance(
                    heading_residual_rad,
                    innovation_variance,
                    "heading",
                )?;
                (
                    Some(squared_mahalanobis_distance),
                    Some(heading_residual_rad),
                )
            }
            None => (None, None),
        };
        let (clock_bias_squared_mahalanobis_distance, clock_bias_residual_m) =
            match clock_bias_observation {
                Some(clock_bias_observation) => {
                    let clock_bias_residual_m = clock_bias_observation.observed_clock_bias_m
                        - clock_bias_observation.reference_clock_bias_m;
                    let squared_mahalanobis_distance = scalar_squared_mahalanobis_distance(
                        clock_bias_residual_m,
                        clock_bias_observation.observation_noise()[(0, 0)],
                        "clock bias",
                    )?;
                    (
                        Some(squared_mahalanobis_distance),
                        Some(clock_bias_residual_m),
                    )
                }
                None => (None, None),
            };
        let clock_bias_persistent_score = match (
            &mut self.clock_bias_persistence,
            clock_bias_observation,
            clock_bias_residual_m,
        ) {
            (
                Some(persistence_state),
                Some(clock_bias_observation),
                Some(clock_bias_residual_m),
            ) => {
                let normalized_absolute_residual = clock_bias_residual_m.abs()
                    / clock_bias_observation.clock_bias_std_m.max(1.0e-6);
                Some(persistence_state.update(normalized_absolute_residual))
            }
            (Some(persistence_state), _, _) => {
                persistence_state.reset();
                Some(0.0)
            }
            (None, _, _) => None,
        };

        let squared_mahalanobis_distance = gps_squared_mahalanobis_distance
            + barometer_squared_mahalanobis_distance.unwrap_or(0.0)
            + heading_squared_mahalanobis_distance.unwrap_or(0.0)
            + clock_bias_squared_mahalanobis_distance.unwrap_or(0.0);
        let accumulated_risk = self.risk_accumulator.update(squared_mahalanobis_distance);
        let (immediate_flag_triggered, immediate_reject_triggered) = match self.immediate_triggers {
            Some(config) => evaluate_immediate_triggers(
                config,
                gps_squared_mahalanobis_distance,
                squared_mahalanobis_distance,
                innovation.fixed_rows::<3>(0).norm(),
            ),
            None => (false, false),
        };
        let horizontal_residual_persistent_score = match &mut self.horizontal_residual_persistence {
            Some(persistence_state) => {
                let horizontal_residual_norm_m = innovation.fixed_rows::<2>(0).norm();
                let horizontal_position_std_m =
                    match self.horizontal_residual_normalization_std_override_m {
                        Some(override_std_m) => override_std_m,
                        None => {
                            let innovation_position_covariance =
                                innovation_covariance.fixed_view::<2, 2>(0, 0).into_owned();
                            let horizontal_position_variance_m2 =
                                innovation_position_covariance.diagonal().amax();
                            sqrtf(horizontal_position_variance_m2).max(1.0e-6)
                        }
                    };
                Some(
                    persistence_state
                        .update(horizontal_residual_norm_m / horizontal_position_std_m),
                )
            }
            None => None,
        };
        let persistent_clock_reject =
            match (self.clock_bias_persistence, clock_bias_persistent_score) {
                (Some(persistence_state), Some(score)) => {
                    score >= persistence_state.config.rejection_score_threshold
                }
                _ => false,
            };
        let persistent_horizontal_reject = match (
            self.horizontal_residual_persistence,
            horizontal_residual_persistent_score,
        ) {
            (Some(persistence_state), Some(score)) => {
                score >= persistence_state.config.rejection_score_threshold
            }
            _ => false,
        };
        let trust_level = classify_trust_level(
            accumulated_risk,
            self.thresholds,
            persistent_clock_reject || persistent_horizontal_reject,
            immediate_flag_triggered,
            immediate_reject_triggered,
        );

        Ok(MonitorVerdict {
            squared_mahalanobis_distance,
            gps_squared_mahalanobis_distance,
            barometer_squared_mahalanobis_distance,
            heading_squared_mahalanobis_distance,
            clock_bias_squared_mahalanobis_distance,
            clock_bias_persistent_score,
            horizontal_residual_persistent_score,
            accumulated_risk,
            innovation,
            barometer_residual_m,
            heading_residual_rad,
            clock_bias_residual_m,
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
    NonPositiveScalarInnovationVariance {
        sensor: &'static str,
        variance: f32,
    },
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
            Self::NonPositiveScalarInnovationVariance { sensor, variance } => write!(
                f,
                "{sensor} innovation variance must be positive, got {variance:.6}"
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

fn classify_trust_level(
    accumulated_risk: f32,
    thresholds: ChiSquareThresholdConfig,
    persistent_clock_reject: bool,
    immediate_flag_triggered: bool,
    immediate_reject_triggered: bool,
) -> TrustLevel {
    if persistent_clock_reject
        || immediate_reject_triggered
        || accumulated_risk >= thresholds.rejected_risk_threshold
    {
        TrustLevel::Rejected
    } else if immediate_flag_triggered || accumulated_risk >= thresholds.flagged_risk_threshold {
        TrustLevel::Flagged
    } else {
        TrustLevel::Trusted
    }
}

fn evaluate_immediate_triggers(
    config: ImmediateTriggerConfig,
    gps_squared_mahalanobis_distance: f32,
    total_squared_mahalanobis_distance: f32,
    position_residual_norm_m: f32,
) -> (bool, bool) {
    let immediate_flag_triggered = config
        .gps_flag_squared_mahalanobis_threshold
        .is_some_and(|threshold| gps_squared_mahalanobis_distance >= threshold)
        || config
            .total_flag_squared_mahalanobis_threshold
            .is_some_and(|threshold| total_squared_mahalanobis_distance >= threshold)
        || config
            .position_residual_flag_threshold_m
            .is_some_and(|threshold| position_residual_norm_m >= threshold);
    let immediate_reject_triggered = config
        .gps_reject_squared_mahalanobis_threshold
        .is_some_and(|threshold| gps_squared_mahalanobis_distance >= threshold)
        || config
            .total_reject_squared_mahalanobis_threshold
            .is_some_and(|threshold| total_squared_mahalanobis_distance >= threshold)
        || config
            .position_residual_reject_threshold_m
            .is_some_and(|threshold| position_residual_norm_m >= threshold);

    (immediate_flag_triggered, immediate_reject_triggered)
}

fn scalar_innovation_variance(
    observation_matrix: &nalgebra::SMatrix<f32, 1, { crate::ekf_core::state::ERROR_STATE_DIM }>,
    state_covariance: &crate::ekf_core::state::StateCovariance,
    observation_noise_variance: f32,
) -> f32 {
    (observation_matrix * state_covariance * observation_matrix.transpose())[(0, 0)]
        + observation_noise_variance
}

fn scalar_squared_mahalanobis_distance(
    residual: f32,
    innovation_variance: f32,
    sensor: &'static str,
) -> Result<f32, MonitorError> {
    if innovation_variance <= 0.0 {
        return Err(MonitorError::NonPositiveScalarInnovationVariance {
            sensor,
            variance: innovation_variance,
        });
    }

    Ok((residual * residual) / innovation_variance)
}

fn predicted_heading_rad(predicted_state: &EskfState) -> f32 {
    predicted_state
        .nominal
        .attitude_body_to_ned
        .euler_angles()
        .2
}

fn wrap_angle_pi(mut angle_rad: f32) -> f32 {
    const TWO_PI: f32 = core::f32::consts::PI * 2.0;
    while angle_rad > core::f32::consts::PI {
        angle_rad -= TWO_PI;
    }
    while angle_rad < -core::f32::consts::PI {
        angle_rad += TWO_PI;
    }
    angle_rad
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use super::{
        ClockBiasPersistenceConfig, EwmaRiskAccumulator, HorizontalResidualPersistenceConfig,
        ImmediateTriggerConfig, StatisticalMonitor,
    };
    use crate::{
        ekf_core::state::{EskfState, NominalState, StateCovariance},
        statistical_monitor::observation::{
            BarometerObservation, ChiSquareThresholdConfig, ClockBiasObservation, GpsObservation,
            HeadingObservation, TrustLevel,
        },
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

    #[test]
    fn barometer_and_heading_anomalies_raise_total_risk() {
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
            Vector3::new(0.5, -0.3, 0.1),
            Vector3::new(0.05, -0.02, 0.01),
            1.5,
            2.0,
            0.3,
            0.5,
        );
        let barometer_observation = BarometerObservation::new(10.1, 12.0, 0.5);
        let heading_observation = HeadingObservation::new(10.1, core::f32::consts::FRAC_PI_2, 0.08);

        let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
        let risk_accumulator = EwmaRiskAccumulator::new(1.0);
        let mut monitor = StatisticalMonitor::new(thresholds, risk_accumulator);

        let verdict = monitor
            .evaluate_observations(
                &predicted_state,
                &gps_observation,
                Some(&barometer_observation),
                Some(&heading_observation),
            )
            .unwrap();

        assert!(verdict.gps_squared_mahalanobis_distance < thresholds.flagged_risk_threshold);
        assert!(verdict.barometer_squared_mahalanobis_distance.unwrap() > 100.0);
        assert!(verdict.heading_squared_mahalanobis_distance.unwrap() > 100.0);
        assert_eq!(verdict.trust_level, TrustLevel::Rejected);
    }

    #[test]
    fn clock_bias_anomaly_raises_total_risk() {
        let nominal = NominalState {
            timestamp_s: 20.0,
            position_ned_m: Vector3::new(1.0, -1.0, 0.5),
            velocity_ned_mps: Vector3::new(0.1, -0.1, 0.0),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        };
        let predicted_state = EskfState::new(nominal, StateCovariance::identity() * 1.0e-3);
        let gps_observation = GpsObservation::from_accuracy_metrics(
            20.1,
            Vector3::new(1.2, -0.8, 0.4),
            Vector3::new(0.1, -0.1, 0.0),
            1.5,
            2.0,
            0.3,
            0.5,
        );
        let clock_bias_observation = ClockBiasObservation::new(20.1, 0.0, 120.0, 5.0);

        let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
        let risk_accumulator = EwmaRiskAccumulator::new(1.0);
        let mut monitor = StatisticalMonitor::new(thresholds, risk_accumulator);

        let verdict = monitor
            .evaluate_observations_with_clock(
                &predicted_state,
                &gps_observation,
                None,
                None,
                Some(&clock_bias_observation),
            )
            .unwrap();

        assert!(verdict.gps_squared_mahalanobis_distance < thresholds.flagged_risk_threshold);
        assert!(verdict.clock_bias_squared_mahalanobis_distance.unwrap() > 500.0);
        assert_eq!(verdict.trust_level, TrustLevel::Rejected);
    }

    #[test]
    fn persistent_clock_bias_rejects_sustained_moderate_drift() {
        let nominal = NominalState {
            timestamp_s: 30.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        };
        let predicted_state = EskfState::new(nominal, StateCovariance::identity() * 1.0e-3);
        let gps_observation = GpsObservation::from_accuracy_metrics(
            30.0,
            Vector3::zeros(),
            Vector3::zeros(),
            1.5,
            2.0,
            0.3,
            0.5,
        );
        let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
        let mut monitor = StatisticalMonitor::new(thresholds, EwmaRiskAccumulator::new(0.6))
            .with_clock_bias_persistence(ClockBiasPersistenceConfig::new(1.0, 20.0));

        let mut last_trust_level = TrustLevel::Trusted;
        for step in 0..40 {
            let clock_bias_observation =
                ClockBiasObservation::new(30.0 + step as f64, 0.0, 8.0, 5.0);
            let verdict = monitor
                .evaluate_observations_with_clock(
                    &predicted_state,
                    &gps_observation,
                    None,
                    None,
                    Some(&clock_bias_observation),
                )
                .unwrap();
            last_trust_level = verdict.trust_level;
        }

        assert_eq!(last_trust_level, TrustLevel::Rejected);
    }

    #[test]
    fn persistent_horizontal_residual_rejects_sustained_moderate_drift() {
        let nominal = NominalState {
            timestamp_s: 40.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        };
        let predicted_state = EskfState::new(nominal, StateCovariance::identity() * 1.0e-3);
        let gps_observation = GpsObservation::from_accuracy_metrics(
            40.0,
            Vector3::new(7.0, -4.0, 0.2),
            Vector3::zeros(),
            5.0,
            6.0,
            1.0,
            1.5,
        );
        let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
        let mut monitor = StatisticalMonitor::new(thresholds, EwmaRiskAccumulator::new(0.6))
            .with_horizontal_residual_persistence(HorizontalResidualPersistenceConfig::new(
                0.2, 8.0,
            ));

        let mut last_verdict = TrustLevel::Trusted;
        for _ in 0..6 {
            let verdict = monitor
                .evaluate_gps_observation(&predicted_state, &gps_observation)
                .unwrap();
            last_verdict = verdict.trust_level;
        }

        assert_eq!(last_verdict, TrustLevel::Rejected);
    }

    #[test]
    fn immediate_gps_trigger_rejects_large_jump_before_ewma_accumulates() {
        let nominal = NominalState {
            timestamp_s: 5.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        };
        let predicted_state = EskfState::new(nominal, StateCovariance::identity() * 1.0e-3);
        let gps_observation = GpsObservation::from_accuracy_metrics(
            5.1,
            Vector3::new(50.0, -25.0, 6.0),
            Vector3::new(6.0, -3.0, 0.5),
            1.5,
            2.0,
            0.3,
            0.5,
        );
        let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
        let risk_accumulator = EwmaRiskAccumulator::new(0.005);
        let mut monitor = StatisticalMonitor::new(thresholds, risk_accumulator)
            .with_immediate_triggers(ImmediateTriggerConfig::gps_only(Some(64.0), Some(144.0)));

        let verdict = monitor
            .evaluate_gps_observation(&predicted_state, &gps_observation)
            .unwrap();

        assert!(verdict.accumulated_risk < thresholds.rejected_risk_threshold);
        assert!(verdict.gps_squared_mahalanobis_distance >= 144.0);
        assert_eq!(verdict.trust_level, TrustLevel::Rejected);
    }
}
