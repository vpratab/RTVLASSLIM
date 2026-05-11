use nalgebra::SMatrix;

use super::{
    AlignedReplaySample, EmpiricalNoiseCalibration, TexbatError, TexbatScenarioConfig,
    build_aligned_samples, predicted_state_from_reference,
};
use crate::ekf_core::state::POS_IDX;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NaiveDistanceBaselineConfig {
    pub distance_threshold_m: f32,
}

impl NaiveDistanceBaselineConfig {
    pub const fn new(distance_threshold_m: f32) -> Self {
        Self {
            distance_threshold_m,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InnovationSigmaBaselineConfig {
    pub nsigma_threshold: f32,
}

impl InnovationSigmaBaselineConfig {
    pub const fn new(nsigma_threshold: f32) -> Self {
        Self { nsigma_threshold }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct TexbatBaselineReport {
    pub scenario_name: String,
    pub baseline_name: String,
    pub threshold_value: f32,
    pub total_samples: u64,
    pub spoof_labeled_samples: u64,
    pub clean_labeled_samples: u64,
    pub true_positives: u64,
    pub false_positives: u64,
}

impl TexbatBaselineReport {
    pub fn true_positive_rate(&self) -> f64 {
        ratio(self.true_positives, self.spoof_labeled_samples)
    }

    pub fn false_positive_rate(&self) -> f64 {
        ratio(self.false_positives, self.clean_labeled_samples)
    }
}

pub fn run_naive_distance_baseline(
    config: &TexbatScenarioConfig,
    baseline_config: NaiveDistanceBaselineConfig,
) -> Result<TexbatBaselineReport, TexbatError> {
    let aligned_data = build_aligned_samples(config)?;
    Ok(evaluate_baseline(
        config,
        "naive_distance_m",
        baseline_config.distance_threshold_m,
        &aligned_data.samples,
        |sample| naive_distance_is_spoofed(sample, baseline_config),
    ))
}

pub fn run_innovation_sigma_baseline(
    config: &TexbatScenarioConfig,
    baseline_config: InnovationSigmaBaselineConfig,
) -> Result<TexbatBaselineReport, TexbatError> {
    let aligned_data = build_aligned_samples(config)?;
    let empirical_noise_calibration = aligned_data.empirical_noise_calibration;
    Ok(evaluate_baseline(
        config,
        "innovation_nsigma",
        baseline_config.nsigma_threshold,
        &aligned_data.samples,
        |sample| {
            innovation_sigma_is_spoofed(
                config,
                sample,
                empirical_noise_calibration,
                baseline_config,
            )
        },
    ))
}

fn evaluate_baseline(
    config: &TexbatScenarioConfig,
    baseline_name: &str,
    threshold_value: f32,
    samples: &[AlignedReplaySample],
    is_spoofed: impl Fn(&AlignedReplaySample) -> bool,
) -> TexbatBaselineReport {
    let mut report = TexbatBaselineReport {
        scenario_name: config.scenario_name.clone(),
        baseline_name: baseline_name.to_owned(),
        threshold_value,
        ..TexbatBaselineReport::default()
    };

    for sample in samples {
        report.total_samples += 1;
        let spoofed = is_spoofed(sample);
        if sample.label_spoofed {
            report.spoof_labeled_samples += 1;
            if spoofed {
                report.true_positives += 1;
            }
        } else {
            report.clean_labeled_samples += 1;
            if spoofed {
                report.false_positives += 1;
            }
        }
    }

    report
}

fn naive_distance_is_spoofed(
    sample: &AlignedReplaySample,
    baseline_config: NaiveDistanceBaselineConfig,
) -> bool {
    let position_error_m = sample.observed_position_ned_m - sample.reference_position_ned_m;
    position_error_m.norm() > baseline_config.distance_threshold_m
}

fn innovation_sigma_is_spoofed(
    config: &TexbatScenarioConfig,
    sample: &AlignedReplaySample,
    empirical_noise_calibration: EmpiricalNoiseCalibration,
    baseline_config: InnovationSigmaBaselineConfig,
) -> bool {
    let predicted_state = predicted_state_from_reference(config, sample);
    let innovation = sample.observed_position_ned_m - sample.reference_position_ned_m;
    let state_position_covariance = predicted_state
        .covariance
        .fixed_view::<3, 3>(POS_IDX, POS_IDX)
        .into_owned();
    let observation_position_covariance = SMatrix::<f32, 3, 3>::from_diagonal(
        &empirical_noise_calibration
            .position_std_ned_m
            .component_mul(&empirical_noise_calibration.position_std_ned_m),
    );
    let innovation_covariance = state_position_covariance + observation_position_covariance;
    let diagonal = innovation_covariance
        .diagonal()
        .map(|variance| variance.sqrt());
    let max_axis_sigma = [
        innovation.x.abs() / diagonal.x.max(1.0e-6),
        innovation.y.abs() / diagonal.y.max(1.0e-6),
        innovation.z.abs() / diagonal.z.max(1.0e-6),
    ]
    .into_iter()
    .fold(0.0_f32, f32::max);
    max_axis_sigma > baseline_config.nsigma_threshold
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
    use nalgebra::Vector3;

    use super::{NaiveDistanceBaselineConfig, naive_distance_is_spoofed};
    use crate::texbat_harness::AlignedReplaySample;

    #[test]
    fn naive_distance_baseline_trips_when_position_error_exceeds_threshold() {
        let sample = AlignedReplaySample {
            timestamp_s: 1.0,
            reference_position_ned_m: Vector3::zeros(),
            observed_position_ned_m: Vector3::new(12.0, 0.0, 0.0),
            reference_velocity_ned_mps: Vector3::zeros(),
            observed_velocity_ned_mps: Vector3::zeros(),
            reference_clock_bias_m: 0.0,
            observed_clock_bias_m: 0.0,
            label_spoofed: true,
        };

        assert!(naive_distance_is_spoofed(
            &sample,
            NaiveDistanceBaselineConfig::new(10.0)
        ));
        assert!(!naive_distance_is_spoofed(
            &sample,
            NaiveDistanceBaselineConfig::new(20.0)
        ));
    }
}
