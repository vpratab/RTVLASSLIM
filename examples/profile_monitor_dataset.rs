use std::{mem::size_of, path::PathBuf, process::ExitCode, time::Instant};

use rtvlas::{
    attestation::{EvidencePacket, SignedEvidencePacket},
    benchmark::{MonitorDatasetRow, load_monitor_dataset_rows_file, run_monitor_dataset_rows},
    ekf_core::state::{EskfState, StateCovariance},
    statistical_monitor::{
        monitor::{EwmaRiskAccumulator, StatisticalMonitor},
        observation::{ChiSquareThresholdConfig, InnovationCovariance, InnovationVector},
    },
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("profiling run failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let args = CliArgs::parse()?;
    let dataset_path = args
        .dataset_path
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_monitor_dataset.csv"));
    let iterations = args.iterations.max(1);
    let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
    let ewma_alpha = 0.6;

    let rows = load_monitor_dataset_rows_file(&dataset_path).map_err(|error| error.to_string())?;
    if rows.is_empty() {
        return Err(format!(
            "dataset {} contained no rows",
            dataset_path.display()
        ));
    }

    let started = Instant::now();
    let mut total_samples = 0_u64;
    let mut max_latency_us = 0.0_f64;
    let mut p95_latency_sum_us = 0.0_f64;
    let mut mean_latency_sum_us = 0.0_f64;
    let mut final_trusted = 0_u64;
    let mut final_flagged = 0_u64;
    let mut final_rejected = 0_u64;

    for _ in 0..iterations {
        let report =
            run_monitor_dataset_rows(rows.clone(), thresholds, ewma_alpha).map_err(|error| {
                format!(
                    "monitor evaluation failed for {}: {error}",
                    dataset_path.display()
                )
            })?;
        total_samples += report.total_samples;
        max_latency_us = max_latency_us.max(report.max_evaluation_latency_us);
        p95_latency_sum_us += report.p95_evaluation_latency_us;
        mean_latency_sum_us += report.mean_evaluation_latency_us;
        final_trusted = report.trusted_verdicts;
        final_flagged = report.flagged_verdicts;
        final_rejected = report.rejected_verdicts;
    }

    let elapsed_s = started.elapsed().as_secs_f64();
    let samples_per_second = total_samples as f64 / elapsed_s.max(f64::EPSILON);

    println!("Dataset: {}", dataset_path.display());
    println!("Rows per iteration: {}", rows.len());
    println!("Iterations: {iterations}");
    println!("Total monitor evaluations: {total_samples}");
    println!("Wall time: {elapsed_s:.6} s");
    println!("Throughput: {samples_per_second:.1} evaluations/s");
    println!(
        "Latency mean/p95/max per iteration (us): {:.2}/{:.2}/{:.2}",
        mean_latency_sum_us / iterations as f64,
        p95_latency_sum_us / iterations as f64,
        max_latency_us
    );
    println!(
        "Final verdict counts: {}/{}/{} trusted/flagged/rejected",
        final_trusted, final_flagged, final_rejected
    );
    println!("Type size snapshot (bytes):");
    println!("  MonitorDatasetRow: {}", size_of::<MonitorDatasetRow>());
    println!("  EskfState: {}", size_of::<EskfState>());
    println!("  StateCovariance: {}", size_of::<StateCovariance>());
    println!(
        "  InnovationVector / InnovationCovariance: {} / {}",
        size_of::<InnovationVector>(),
        size_of::<InnovationCovariance>()
    );
    println!("  StatisticalMonitor: {}", size_of::<StatisticalMonitor>());
    println!(
        "  EwmaRiskAccumulator: {}",
        size_of::<EwmaRiskAccumulator>()
    );
    println!("  EvidencePacket: {}", size_of::<EvidencePacket>());
    println!(
        "  SignedEvidencePacket: {}",
        size_of::<SignedEvidencePacket>()
    );
    println!("Note: this is host profiling, not target flight-hardware profiling.");

    Ok(())
}

struct CliArgs {
    dataset_path: Option<PathBuf>,
    iterations: usize,
}

impl CliArgs {
    fn parse() -> Result<Self, String> {
        let mut dataset_path = None;
        let mut iterations = 100_usize;
        let mut arguments = std::env::args().skip(1);

        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--iterations" => {
                    let value = arguments
                        .next()
                        .ok_or_else(|| "--iterations requires a value".to_string())?;
                    iterations = value.parse::<usize>().map_err(|error| error.to_string())?;
                }
                "--help" | "-h" => {
                    return Err(
                        "usage: profile_monitor_dataset [dataset.csv] [--iterations N]".to_string(),
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
            dataset_path,
            iterations,
        })
    }
}
