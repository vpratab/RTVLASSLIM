use std::{fs::File, io::Write, path::Path};

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::{
    MonitorDatasetError, MonitorDatasetReport, MonitorDatasetRow, SpoofInjectionConfig,
    run_monitor_dataset_rows, spoof_monitor_dataset_rows,
};
use crate::statistical_monitor::observation::ChiSquareThresholdConfig;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SweepCase {
    pub label: String,
    pub onset_time_s: f64,
    pub ramp_duration_s: f64,
    pub direction_label: String,
    pub offset_mode_label: String,
    pub position_offset_ned_m: [f32; 3],
    pub velocity_offset_ned_mps: [f32; 3],
}

impl SweepCase {
    pub fn new(
        label: impl Into<String>,
        onset_time_s: f64,
        ramp_duration_s: f64,
        direction_label: impl Into<String>,
        offset_mode_label: impl Into<String>,
        position_offset_ned_m: Vector3<f32>,
        velocity_offset_ned_mps: Vector3<f32>,
    ) -> Self {
        Self {
            label: label.into(),
            onset_time_s,
            ramp_duration_s,
            direction_label: direction_label.into(),
            offset_mode_label: offset_mode_label.into(),
            position_offset_ned_m: [
                position_offset_ned_m.x,
                position_offset_ned_m.y,
                position_offset_ned_m.z,
            ],
            velocity_offset_ned_mps: [
                velocity_offset_ned_mps.x,
                velocity_offset_ned_mps.y,
                velocity_offset_ned_mps.z,
            ],
        }
    }

    pub fn spoof_config(&self) -> SpoofInjectionConfig {
        SpoofInjectionConfig::new(
            self.onset_time_s,
            self.ramp_duration_s,
            Vector3::new(
                self.position_offset_ned_m[0],
                self.position_offset_ned_m[1],
                self.position_offset_ned_m[2],
            ),
            Vector3::new(
                self.velocity_offset_ned_mps[0],
                self.velocity_offset_ned_mps[1],
                self.velocity_offset_ned_mps[2],
            ),
        )
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SweepResultRow {
    pub dataset_label: String,
    pub scenario_label: String,
    pub onset_time_s: f64,
    pub ramp_duration_s: f64,
    pub direction_label: String,
    pub offset_mode_label: String,
    pub position_offset_north_m: f32,
    pub position_offset_east_m: f32,
    pub position_offset_down_m: f32,
    pub velocity_offset_north_mps: f32,
    pub velocity_offset_east_mps: f32,
    pub velocity_offset_down_mps: f32,
    pub total_samples: u64,
    pub spoof_labeled_samples: u64,
    pub clean_labeled_samples: u64,
    pub trusted_verdicts: u64,
    pub flagged_verdicts: u64,
    pub rejected_verdicts: u64,
    pub anomaly_tpr: f64,
    pub anomaly_fpr: f64,
    pub rejected_tpr: f64,
    pub rejected_fpr: f64,
    pub first_spoof_labeled_sample_index: Option<u64>,
    pub first_anomaly_sample_index: Option<u64>,
    pub first_rejected_sample_index: Option<u64>,
    pub samples_from_onset_to_first_anomaly: Option<u64>,
    pub samples_from_onset_to_first_rejection: Option<u64>,
    pub mean_evaluation_latency_us: f64,
    pub p95_evaluation_latency_us: f64,
    pub max_evaluation_latency_us: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SweepRunReport {
    pub dataset_label: String,
    pub thresholds_flagged: f32,
    pub thresholds_rejected: f32,
    pub ewma_alpha: f32,
    pub nominal_report: SweepResultRow,
    pub results: Vec<SweepResultRow>,
}

pub fn build_default_sweep_cases(
    onset_times_s: &[f64],
    ramp_durations_s: &[f64],
) -> Vec<SweepCase> {
    let directions = [
        ("north", Vector3::new(30.0_f32, 0.0, 0.0)),
        ("east", Vector3::new(0.0_f32, 30.0, 0.0)),
        ("northeast", Vector3::new(21.213_203_f32, 21.213_203, 0.0)),
        ("northwest", Vector3::new(21.213_203_f32, -21.213_203, 0.0)),
    ];
    let velocity_scale = 0.3_f32;
    let mut cases = Vec::new();

    for &onset_time_s in onset_times_s {
        for &ramp_duration_s in ramp_durations_s {
            for (direction_label, position_offset_ned_m) in directions {
                let position_only_label = format!(
                    "{direction_label}_onset_{onset_time_s:.1}_ramp_{ramp_duration_s:.1}_pos"
                );
                cases.push(SweepCase::new(
                    position_only_label,
                    onset_time_s,
                    ramp_duration_s,
                    direction_label,
                    "position_only",
                    position_offset_ned_m,
                    Vector3::zeros(),
                ));

                let velocity_offset_ned_mps = if ramp_duration_s <= f64::EPSILON {
                    position_offset_ned_m.normalize() * 8.0
                } else {
                    position_offset_ned_m * velocity_scale
                };
                let position_velocity_label = format!(
                    "{direction_label}_onset_{onset_time_s:.1}_ramp_{ramp_duration_s:.1}_posvel"
                );
                cases.push(SweepCase::new(
                    position_velocity_label,
                    onset_time_s,
                    ramp_duration_s,
                    direction_label,
                    "position_plus_velocity",
                    position_offset_ned_m,
                    velocity_offset_ned_mps,
                ));
            }
        }
    }

    cases
}

pub fn build_extended_sweep_cases(
    onset_times_s: &[f64],
    ramp_durations_s: &[f64],
) -> Vec<SweepCase> {
    let directions = [
        ("north", Vector3::new(1.0_f32, 0.0, 0.0)),
        ("east", Vector3::new(0.0_f32, 1.0, 0.0)),
        ("south", Vector3::new(-1.0_f32, 0.0, 0.0)),
        ("west", Vector3::new(0.0_f32, -1.0, 0.0)),
        (
            "northeast",
            Vector3::new(0.707_106_77_f32, 0.707_106_77, 0.0),
        ),
        (
            "northwest",
            Vector3::new(0.707_106_77_f32, -0.707_106_77, 0.0),
        ),
        ("up", Vector3::new(0.0_f32, 0.0, -1.0)),
        ("down", Vector3::new(0.0_f32, 0.0, 1.0)),
    ];
    let magnitudes_m = [10.0_f32, 30.0, 60.0];
    let mut cases = Vec::new();

    for &onset_time_s in onset_times_s {
        for &ramp_duration_s in ramp_durations_s {
            for &magnitude_m in &magnitudes_m {
                for (direction_label, unit_direction) in directions {
                    let position_offset_ned_m = unit_direction * magnitude_m;
                    let magnitude_label = format_magnitude_label(magnitude_m);
                    let position_label = format!(
                        "{direction_label}_{magnitude_label}m_onset_{onset_time_s:.1}_ramp_{ramp_duration_s:.1}_pos"
                    );
                    cases.push(SweepCase::new(
                        position_label,
                        onset_time_s,
                        ramp_duration_s,
                        direction_label,
                        "position_only",
                        position_offset_ned_m,
                        Vector3::zeros(),
                    ));

                    let velocity_offset_ned_mps = if ramp_duration_s <= f64::EPSILON {
                        unit_direction * 8.0
                    } else {
                        position_offset_ned_m / ramp_duration_s as f32
                    };
                    let position_velocity_label = format!(
                        "{direction_label}_{magnitude_label}m_onset_{onset_time_s:.1}_ramp_{ramp_duration_s:.1}_posvel"
                    );
                    cases.push(SweepCase::new(
                        position_velocity_label,
                        onset_time_s,
                        ramp_duration_s,
                        direction_label,
                        "position_plus_velocity",
                        position_offset_ned_m,
                        velocity_offset_ned_mps,
                    ));
                }
            }
        }
    }

    cases
}

pub fn run_adversarial_sweep(
    rows: &[MonitorDatasetRow],
    dataset_label: &str,
    cases: &[SweepCase],
    thresholds: ChiSquareThresholdConfig,
    ewma_alpha: f32,
) -> Result<SweepRunReport, MonitorDatasetError> {
    let nominal_report = run_monitor_dataset_rows(rows.to_vec(), thresholds, ewma_alpha)?;
    let nominal_row = result_row_from_report(
        dataset_label,
        "nominal",
        0.0,
        0.0,
        "none",
        "none",
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        &nominal_report,
    );

    let mut results = Vec::with_capacity(cases.len());
    for case in cases {
        let spoofed_rows = spoof_monitor_dataset_rows(rows, case.spoof_config());
        let report = run_monitor_dataset_rows(spoofed_rows, thresholds, ewma_alpha)?;
        results.push(result_row_from_report(
            dataset_label,
            &case.label,
            case.onset_time_s,
            case.ramp_duration_s,
            &case.direction_label,
            &case.offset_mode_label,
            case.position_offset_ned_m,
            case.velocity_offset_ned_mps,
            &report,
        ));
    }

    Ok(SweepRunReport {
        dataset_label: dataset_label.to_owned(),
        thresholds_flagged: thresholds.flagged_risk_threshold,
        thresholds_rejected: thresholds.rejected_risk_threshold,
        ewma_alpha,
        nominal_report: nominal_row,
        results,
    })
}

fn format_magnitude_label(magnitude_m: f32) -> String {
    if (magnitude_m.fract()).abs() <= f32::EPSILON {
        format!("{magnitude_m:.0}")
    } else {
        format!("{magnitude_m:.1}")
    }
}

pub fn write_sweep_csv<P: AsRef<Path>>(
    path: P,
    report: &SweepRunReport,
) -> Result<(), MonitorDatasetError> {
    let file = File::create(path).map_err(MonitorDatasetError::Io)?;
    let mut writer = csv::Writer::from_writer(file);
    writer
        .serialize(&report.nominal_report)
        .map_err(MonitorDatasetError::Csv)?;
    for row in &report.results {
        writer.serialize(row).map_err(MonitorDatasetError::Csv)?;
    }
    writer.flush().map_err(MonitorDatasetError::Io)
}

pub fn write_sweep_json<P: AsRef<Path>>(
    path: P,
    report: &SweepRunReport,
) -> Result<(), MonitorDatasetError> {
    let file = File::create(path).map_err(MonitorDatasetError::Io)?;
    serde_json::to_writer_pretty(file, report)
        .map_err(|error| MonitorDatasetError::Io(std::io::Error::other(error)))
}

pub fn write_worst_case_summary<W: Write>(
    mut writer: W,
    report: &SweepRunReport,
) -> Result<(), MonitorDatasetError> {
    if let Some(worst_case) = report
        .results
        .iter()
        .min_by(|left, right| left.rejected_tpr.total_cmp(&right.rejected_tpr))
    {
        writeln!(
            writer,
            "Worst-case rejected TPR: {:.3} for {} (direction={}, mode={}, onset={:.1}s, ramp={:.1}s, onset->reject={:?})",
            worst_case.rejected_tpr,
            worst_case.scenario_label,
            worst_case.direction_label,
            worst_case.offset_mode_label,
            worst_case.onset_time_s,
            worst_case.ramp_duration_s,
            worst_case.samples_from_onset_to_first_rejection,
        )
        .map_err(MonitorDatasetError::Io)?;
    }

    Ok(())
}

fn result_row_from_report(
    dataset_label: &str,
    scenario_label: &str,
    onset_time_s: f64,
    ramp_duration_s: f64,
    direction_label: &str,
    offset_mode_label: &str,
    position_offset_ned_m: [f32; 3],
    velocity_offset_ned_mps: [f32; 3],
    report: &MonitorDatasetReport,
) -> SweepResultRow {
    SweepResultRow {
        dataset_label: dataset_label.to_owned(),
        scenario_label: scenario_label.to_owned(),
        onset_time_s,
        ramp_duration_s,
        direction_label: direction_label.to_owned(),
        offset_mode_label: offset_mode_label.to_owned(),
        position_offset_north_m: position_offset_ned_m[0],
        position_offset_east_m: position_offset_ned_m[1],
        position_offset_down_m: position_offset_ned_m[2],
        velocity_offset_north_mps: velocity_offset_ned_mps[0],
        velocity_offset_east_mps: velocity_offset_ned_mps[1],
        velocity_offset_down_mps: velocity_offset_ned_mps[2],
        total_samples: report.total_samples,
        spoof_labeled_samples: report.spoof_labeled_samples,
        clean_labeled_samples: report.clean_labeled_samples,
        trusted_verdicts: report.trusted_verdicts,
        flagged_verdicts: report.flagged_verdicts,
        rejected_verdicts: report.rejected_verdicts,
        anomaly_tpr: report.anomaly_true_positive_rate(),
        anomaly_fpr: report.anomaly_false_positive_rate(),
        rejected_tpr: report.rejected_true_positive_rate(),
        rejected_fpr: report.rejected_false_positive_rate(),
        first_spoof_labeled_sample_index: report.first_spoof_labeled_sample_index,
        first_anomaly_sample_index: report.first_anomaly_sample_index,
        first_rejected_sample_index: report.first_rejected_sample_index,
        samples_from_onset_to_first_anomaly: report.samples_from_onset_to_first_anomaly,
        samples_from_onset_to_first_rejection: report.samples_from_onset_to_first_rejection,
        mean_evaluation_latency_us: report.mean_evaluation_latency_us,
        p95_evaluation_latency_us: report.p95_evaluation_latency_us,
        max_evaluation_latency_us: report.max_evaluation_latency_us,
    }
}

#[cfg(test)]
mod tests {
    use super::build_extended_sweep_cases;

    #[test]
    fn extended_sweep_covers_axes_magnitudes_and_offset_modes() {
        let cases = build_extended_sweep_cases(&[2.0, 4.0], &[0.0, 40.0]);

        assert_eq!(cases.len(), 2 * 2 * 3 * 8 * 2);
        assert!(cases.iter().any(|case| case.direction_label == "up"));
        assert!(cases.iter().any(|case| case.direction_label == "down"));
        assert!(cases.iter().any(|case| case.direction_label == "south"));
        assert!(
            cases
                .iter()
                .any(|case| case.offset_mode_label == "position_plus_velocity")
        );
        assert!(
            cases
                .iter()
                .any(|case| case.label == "north_60m_onset_4.0_ramp_40.0_posvel")
        );
    }
}
