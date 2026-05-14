use std::{fs, path::PathBuf, process::ExitCode};

use serde::Serialize;

use rtvlas::{
    benchmark::{
        load_monitor_dataset_rows_file,
        realistic_spoof::{
            RealisticSpoofCase, apply_realistic_spoof_case, built_in_realistic_spoof_cases,
        },
        run_monitor_dataset_rows,
    },
    statistical_monitor::observation::ChiSquareThresholdConfig,
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("realistic spoof suite failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args = CliArgs::parse()?;
    fs::create_dir_all(&args.output_dir).map_err(|error| error.to_string())?;

    let rows = load_monitor_dataset_rows_file(&args.dataset_path)
        .map_err(|error| format!("failed to load {}: {error}", args.dataset_path.display()))?;
    if rows.is_empty() {
        return Err(format!(
            "dataset {} contained no rows",
            args.dataset_path.display()
        ));
    }

    let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
    let ewma_alpha = 0.6;
    let nominal_report = run_monitor_dataset_rows(rows.clone(), thresholds, ewma_alpha)
        .map_err(|error| error.to_string())?;

    let mut results = Vec::new();
    for case in built_in_realistic_spoof_cases() {
        let spoofed_rows = apply_realistic_spoof_case(&rows, &case);
        let report = run_monitor_dataset_rows(spoofed_rows, thresholds, ewma_alpha)
            .map_err(|error| error.to_string())?;
        results.push(SpoofSuiteResultRow::from_case(
            &args.dataset_label,
            case,
            report,
        ));
    }

    let csv_path = args
        .output_dir
        .join(format!("{}_realistic_spoof_suite.csv", args.dataset_label));
    let json_path = args
        .output_dir
        .join(format!("{}_realistic_spoof_suite.json", args.dataset_label));
    write_results_csv(&csv_path, &results)?;
    write_results_json(&json_path, &results)?;

    println!("Dataset: {}", args.dataset_path.display());
    println!("Dataset label: {}", args.dataset_label);
    println!("Cases evaluated: {}", results.len());
    println!(
        "Nominal trusted/flagged/rejected: {}/{}/{}",
        nominal_report.trusted_verdicts,
        nominal_report.flagged_verdicts,
        nominal_report.rejected_verdicts
    );
    println!(
        "Nominal anomaly FPR: {:.3}",
        nominal_report.anomaly_false_positive_rate()
    );
    println!();
    println!("profile,anomaly_tpr,rejected_tpr,first_reject");
    for row in &results {
        println!(
            "{},{:.3},{:.3},{}",
            row.profile_label,
            row.anomaly_tpr,
            row.rejected_tpr,
            row.samples_from_onset_to_first_rejection
                .map(|value| value.to_string())
                .unwrap_or_else(|| "none".to_owned())
        );
    }
    println!("CSV export: {}", csv_path.display());
    println!("JSON export: {}", json_path.display());

    Ok(())
}

#[derive(Clone, Debug)]
struct CliArgs {
    dataset_path: PathBuf,
    dataset_label: String,
    output_dir: PathBuf,
}

impl CliArgs {
    fn parse() -> Result<Self, String> {
        let mut dataset_path = None;
        let mut dataset_label = None;
        let mut output_dir = PathBuf::from("artifacts/spoof_suites");
        let mut arguments = std::env::args().skip(1);

        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--dataset-label" => {
                    dataset_label = Some(
                        arguments
                            .next()
                            .ok_or_else(|| "--dataset-label requires a value".to_string())?,
                    );
                }
                "--output-dir" => {
                    output_dir = PathBuf::from(
                        arguments
                            .next()
                            .ok_or_else(|| "--output-dir requires a value".to_string())?,
                    );
                }
                "--help" | "-h" => {
                    return Err(
                        "usage: run_realistic_spoof_suite [dataset.csv] [--dataset-label LABEL] [--output-dir DIR]"
                            .to_string(),
                    );
                }
                _ if argument.starts_with("--") => {
                    return Err(format!("unknown option {argument}"));
                }
                _ => {
                    if dataset_path.is_some() {
                        return Err(format!("unexpected extra positional argument {argument}"));
                    }
                    dataset_path = Some(PathBuf::from(argument));
                }
            }
        }

        let dataset_path =
            dataset_path.unwrap_or_else(|| PathBuf::from("artifacts/px4_hover_dataset.csv"));
        let dataset_label = dataset_label.unwrap_or_else(|| {
            dataset_path
                .file_stem()
                .and_then(|stem| stem.to_str())
                .unwrap_or("monitor_dataset")
                .to_owned()
        });

        Ok(Self {
            dataset_path,
            dataset_label,
            output_dir,
        })
    }
}

#[derive(Debug, Serialize)]
struct SpoofSuiteResultRow {
    dataset_label: String,
    profile_label: String,
    family: String,
    source_basis: String,
    description: String,
    total_samples: u64,
    spoof_labeled_samples: u64,
    trusted_verdicts: u64,
    flagged_verdicts: u64,
    rejected_verdicts: u64,
    anomaly_tpr: f64,
    anomaly_fpr: f64,
    rejected_tpr: f64,
    rejected_fpr: f64,
    first_anomaly_sample_index: Option<u64>,
    first_rejected_sample_index: Option<u64>,
    samples_from_onset_to_first_anomaly: Option<u64>,
    samples_from_onset_to_first_rejection: Option<u64>,
    mean_evaluation_latency_us: f64,
    p95_evaluation_latency_us: f64,
    max_evaluation_latency_us: f64,
    max_gps_squared_mahalanobis_distance: f32,
    max_accumulated_risk: f32,
    max_horizontal_position_residual_m: f32,
    max_horizontal_velocity_residual_mps: f32,
    max_abs_clock_bias_residual_m: f32,
    max_clock_bias_persistent_score: f32,
    max_horizontal_residual_persistent_score: f32,
    max_velocity_residual_persistent_score: f32,
    max_stale_gps_persistent_score: f32,
    dominant_signal: String,
}

impl SpoofSuiteResultRow {
    fn from_case(
        dataset_label: &str,
        case: RealisticSpoofCase,
        report: rtvlas::benchmark::MonitorDatasetReport,
    ) -> Self {
        Self {
            dataset_label: dataset_label.to_owned(),
            profile_label: case.label,
            family: case.family,
            source_basis: case.source_basis,
            description: case.description,
            total_samples: report.total_samples,
            spoof_labeled_samples: report.spoof_labeled_samples,
            trusted_verdicts: report.trusted_verdicts,
            flagged_verdicts: report.flagged_verdicts,
            rejected_verdicts: report.rejected_verdicts,
            anomaly_tpr: report.anomaly_true_positive_rate(),
            anomaly_fpr: report.anomaly_false_positive_rate(),
            rejected_tpr: report.rejected_true_positive_rate(),
            rejected_fpr: report.rejected_false_positive_rate(),
            first_anomaly_sample_index: report.first_anomaly_sample_index,
            first_rejected_sample_index: report.first_rejected_sample_index,
            samples_from_onset_to_first_anomaly: report.samples_from_onset_to_first_anomaly,
            samples_from_onset_to_first_rejection: report.samples_from_onset_to_first_rejection,
            mean_evaluation_latency_us: report.mean_evaluation_latency_us,
            p95_evaluation_latency_us: report.p95_evaluation_latency_us,
            max_evaluation_latency_us: report.max_evaluation_latency_us,
            max_gps_squared_mahalanobis_distance: report.max_gps_squared_mahalanobis_distance,
            max_accumulated_risk: report.max_accumulated_risk,
            max_horizontal_position_residual_m: report.max_horizontal_position_residual_m,
            max_horizontal_velocity_residual_mps: report.max_horizontal_velocity_residual_mps,
            max_abs_clock_bias_residual_m: report.max_abs_clock_bias_residual_m,
            max_clock_bias_persistent_score: report.max_clock_bias_persistent_score,
            max_horizontal_residual_persistent_score: report
                .max_horizontal_residual_persistent_score,
            max_velocity_residual_persistent_score: report.max_velocity_residual_persistent_score,
            max_stale_gps_persistent_score: report.max_stale_gps_persistent_score,
            dominant_signal: dominant_signal(&report),
        }
    }
}

fn dominant_signal(report: &rtvlas::benchmark::MonitorDatasetReport) -> String {
    let signals = [
        (
            "clock_bias_persistence",
            report.max_clock_bias_persistent_score / 20.0,
        ),
        (
            "horizontal_position_persistence",
            report.max_horizontal_residual_persistent_score / 30.0,
        ),
        (
            "horizontal_velocity_persistence",
            report.max_velocity_residual_persistent_score / 25.0,
        ),
        (
            "stale_gps_persistence",
            report.max_stale_gps_persistent_score / 2.0,
        ),
        ("ewma_risk", report.max_accumulated_risk / 22.458_f32),
    ];

    signals
        .into_iter()
        .max_by(|left, right| left.1.total_cmp(&right.1))
        .map(|(label, _)| label.to_owned())
        .unwrap_or_else(|| "none".to_owned())
}

fn write_results_csv(path: &PathBuf, rows: &[SpoofSuiteResultRow]) -> Result<(), String> {
    let mut writer = csv::Writer::from_path(path).map_err(|error| error.to_string())?;
    for row in rows {
        writer.serialize(row).map_err(|error| error.to_string())?;
    }
    writer.flush().map_err(|error| error.to_string())
}

fn write_results_json(path: &PathBuf, rows: &[SpoofSuiteResultRow]) -> Result<(), String> {
    let file = std::fs::File::create(path).map_err(|error| error.to_string())?;
    serde_json::to_writer_pretty(file, rows).map_err(|error| error.to_string())
}
