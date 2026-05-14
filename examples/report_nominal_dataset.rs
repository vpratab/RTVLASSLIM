use std::{path::PathBuf, process::ExitCode};

use rtvlas::{
    benchmark::{MonitorDatasetRow, load_monitor_dataset_rows_file, run_monitor_dataset_rows},
    statistical_monitor::observation::ChiSquareThresholdConfig,
};
use serde::Serialize;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("nominal report failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args = CliArgs::parse()?;
    let rows =
        load_monitor_dataset_rows_file(&args.dataset_path).map_err(|error| error.to_string())?;
    if rows.is_empty() {
        return Err(format!("{} contained no rows", args.dataset_path.display()));
    }

    let report = run_monitor_dataset_rows(
        rows.clone(),
        ChiSquareThresholdConfig::new(12.592, 22.458),
        0.6,
    )
    .map_err(|error| error.to_string())?;

    let horizontal_residuals_m = horizontal_residuals_m(&rows);
    let horizontal_velocity_residuals_mps = horizontal_velocity_residuals_mps(&rows);
    let duration_s = rows
        .last()
        .zip(rows.first())
        .map(|(last, first)| (last.timestamp_s - first.timestamp_s).max(0.0))
        .unwrap_or(0.0);
    let spoof_labeled_rows = rows.iter().filter(|row| row.label_spoofed).count();

    let summary = NominalDatasetSummary {
        dataset_path: args.dataset_path.display().to_string(),
        samples: rows.len(),
        duration_s,
        spoof_labeled_rows,
        trusted_verdicts: report.trusted_verdicts,
        flagged_verdicts: report.flagged_verdicts,
        rejected_verdicts: report.rejected_verdicts,
        anomaly_false_positive_rate: report.anomaly_false_positive_rate(),
        rejected_false_positive_rate: report.rejected_false_positive_rate(),
        mean_evaluation_latency_us: report.mean_evaluation_latency_us,
        p95_evaluation_latency_us: report.p95_evaluation_latency_us,
        max_evaluation_latency_us: report.max_evaluation_latency_us,
        horizontal_residual_m: ResidualSummary::from_values(horizontal_residuals_m),
        horizontal_velocity_residual_mps: ResidualSummary::from_values(
            horizontal_velocity_residuals_mps,
        ),
        acceptance_threshold_fpr: args.acceptance_fpr,
        accepted: spoof_labeled_rows == 0
            && report.anomaly_false_positive_rate() <= args.acceptance_fpr
            && report.rejected_false_positive_rate() <= args.acceptance_fpr,
    };

    println!("Nominal dataset: {}", summary.dataset_path);
    println!("  samples: {}", summary.samples);
    println!("  duration: {:.3} s", summary.duration_s);
    println!("  spoof-labeled rows: {}", summary.spoof_labeled_rows);
    println!(
        "  trusted/flagged/rejected: {}/{}/{}",
        summary.trusted_verdicts, summary.flagged_verdicts, summary.rejected_verdicts
    );
    println!(
        "  anomaly/rejected FPR: {:.3}/{:.3}",
        summary.anomaly_false_positive_rate, summary.rejected_false_positive_rate
    );
    println!(
        "  horizontal residual mean/p95/max: {:.3}/{:.3}/{:.3} m",
        summary.horizontal_residual_m.mean,
        summary.horizontal_residual_m.p95,
        summary.horizontal_residual_m.max
    );
    println!(
        "  horizontal velocity residual mean/p95/max: {:.3}/{:.3}/{:.3} m/s",
        summary.horizontal_velocity_residual_mps.mean,
        summary.horizontal_velocity_residual_mps.p95,
        summary.horizontal_velocity_residual_mps.max
    );
    println!(
        "  eval latency mean/p95/max: {:.2}/{:.2}/{:.2} us",
        summary.mean_evaluation_latency_us,
        summary.p95_evaluation_latency_us,
        summary.max_evaluation_latency_us
    );
    println!("  accepted: {}", summary.accepted);

    if let Some(path) = args.json_output {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|error| error.to_string())?;
        }
        let json = serde_json::to_string_pretty(&summary).map_err(|error| error.to_string())?;
        std::fs::write(&path, json).map_err(|error| error.to_string())?;
        println!("JSON report: {}", path.display());
    }

    if summary.accepted {
        Ok(())
    } else {
        Err("nominal dataset did not meet acceptance criteria".to_string())
    }
}

#[derive(Clone, Debug)]
struct CliArgs {
    dataset_path: PathBuf,
    json_output: Option<PathBuf>,
    acceptance_fpr: f64,
}

impl CliArgs {
    fn parse() -> Result<Self, String> {
        let mut dataset_path = None;
        let mut json_output = None;
        let mut acceptance_fpr = 0.01_f64;
        let mut arguments = std::env::args().skip(1);

        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--json-output" => {
                    json_output =
                        Some(PathBuf::from(arguments.next().ok_or_else(|| {
                            "--json-output requires a path".to_string()
                        })?));
                }
                "--acceptance-fpr" => {
                    acceptance_fpr = arguments
                        .next()
                        .ok_or_else(|| "--acceptance-fpr requires a value".to_string())?
                        .parse::<f64>()
                        .map_err(|error| error.to_string())?;
                }
                "--help" | "-h" => {
                    return Err(
                        "usage: report_nominal_dataset <dataset.csv> [--json-output PATH] [--acceptance-fpr FPR]"
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

        Ok(Self {
            dataset_path: dataset_path
                .unwrap_or_else(|| PathBuf::from("artifacts/px4_monitor_dataset.csv")),
            json_output,
            acceptance_fpr,
        })
    }
}

#[derive(Clone, Debug, Serialize)]
struct NominalDatasetSummary {
    dataset_path: String,
    samples: usize,
    duration_s: f64,
    spoof_labeled_rows: usize,
    trusted_verdicts: u64,
    flagged_verdicts: u64,
    rejected_verdicts: u64,
    anomaly_false_positive_rate: f64,
    rejected_false_positive_rate: f64,
    mean_evaluation_latency_us: f64,
    p95_evaluation_latency_us: f64,
    max_evaluation_latency_us: f64,
    horizontal_residual_m: ResidualSummary,
    horizontal_velocity_residual_mps: ResidualSummary,
    acceptance_threshold_fpr: f64,
    accepted: bool,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct ResidualSummary {
    mean: f32,
    p95: f32,
    max: f32,
}

impl ResidualSummary {
    fn from_values(mut values: Vec<f32>) -> Self {
        if values.is_empty() {
            return Self {
                mean: 0.0,
                p95: 0.0,
                max: 0.0,
            };
        }

        values.sort_by(|left, right| left.total_cmp(right));
        let mean = values.iter().sum::<f32>() / values.len() as f32;
        let p95_index = ((values.len() as f32 * 0.95).ceil() as usize)
            .saturating_sub(1)
            .min(values.len() - 1);
        let max = *values.last().unwrap_or(&0.0);

        Self {
            mean,
            p95: values[p95_index],
            max,
        }
    }
}

fn horizontal_residuals_m(rows: &[MonitorDatasetRow]) -> Vec<f32> {
    rows.iter()
        .map(|row| {
            let north_m = row.gps_px_ned_m - row.state_px_ned_m;
            let east_m = row.gps_py_ned_m - row.state_py_ned_m;
            (north_m * north_m + east_m * east_m).sqrt()
        })
        .collect()
}

fn horizontal_velocity_residuals_mps(rows: &[MonitorDatasetRow]) -> Vec<f32> {
    rows.iter()
        .map(|row| {
            let north_mps = row.gps_vx_ned_mps - row.state_vx_ned_mps;
            let east_mps = row.gps_vy_ned_mps - row.state_vy_ned_mps;
            (north_mps * north_mps + east_mps * east_mps).sqrt()
        })
        .collect()
}
