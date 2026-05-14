use std::{mem::size_of, path::PathBuf, process::ExitCode, time::Instant};

use serde::Serialize;

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
    let mean_latency_us = mean_latency_sum_us / iterations as f64;
    let p95_latency_us = p95_latency_sum_us / iterations as f64;
    let accepted = p95_latency_us <= args.acceptance_p95_us
        && max_latency_us <= args.acceptance_max_us
        && final_flagged == 0
        && final_rejected == 0;

    let summary = ProfileSummary {
        dataset_path: dataset_path.display().to_string(),
        host_os: std::env::consts::OS.to_owned(),
        host_arch: std::env::consts::ARCH.to_owned(),
        rows_per_iteration: rows.len(),
        iterations,
        total_monitor_evaluations: total_samples,
        wall_time_s: elapsed_s,
        throughput_evaluations_per_s: samples_per_second,
        latency_mean_us: mean_latency_us,
        latency_p95_us: p95_latency_us,
        latency_max_us: max_latency_us,
        final_trusted_verdicts: final_trusted,
        final_flagged_verdicts: final_flagged,
        final_rejected_verdicts: final_rejected,
        type_sizes_bytes: TypeSizeSummary::current(),
        acceptance_p95_us: args.acceptance_p95_us,
        acceptance_max_us: args.acceptance_max_us,
        accepted,
        note: "host or target replay profiling only; not a flight-scheduler WCET proof".to_owned(),
    };

    println!("Dataset: {}", dataset_path.display());
    println!("Rows per iteration: {}", rows.len());
    println!("Iterations: {iterations}");
    println!("Total monitor evaluations: {total_samples}");
    println!("Wall time: {elapsed_s:.6} s");
    println!("Throughput: {samples_per_second:.1} evaluations/s");
    println!(
        "Latency mean/p95/max per iteration (us): {:.2}/{:.2}/{:.2}",
        summary.latency_mean_us, summary.latency_p95_us, summary.latency_max_us
    );
    println!(
        "Final verdict counts: {}/{}/{} trusted/flagged/rejected",
        final_trusted, final_flagged, final_rejected
    );
    println!("Type size snapshot (bytes):");
    println!(
        "  MonitorDatasetRow: {}",
        summary.type_sizes_bytes.monitor_dataset_row
    );
    println!("  EskfState: {}", summary.type_sizes_bytes.eskf_state);
    println!(
        "  StateCovariance: {}",
        summary.type_sizes_bytes.state_covariance
    );
    println!(
        "  InnovationVector / InnovationCovariance: {} / {}",
        summary.type_sizes_bytes.innovation_vector, summary.type_sizes_bytes.innovation_covariance
    );
    println!(
        "  StatisticalMonitor: {}",
        summary.type_sizes_bytes.statistical_monitor
    );
    println!(
        "  EwmaRiskAccumulator: {}",
        summary.type_sizes_bytes.ewma_risk_accumulator
    );
    println!(
        "  EvidencePacket: {}",
        summary.type_sizes_bytes.evidence_packet
    );
    println!(
        "  SignedEvidencePacket: {}",
        summary.type_sizes_bytes.signed_evidence_packet
    );
    println!(
        "Acceptance p95/max thresholds (us): {:.2}/{:.2}",
        summary.acceptance_p95_us, summary.acceptance_max_us
    );
    println!("Accepted: {}", summary.accepted);
    println!("Note: this is host profiling, not target flight-hardware profiling.");

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
        Err("profile run did not meet acceptance criteria".to_string())
    }
}

struct CliArgs {
    dataset_path: Option<PathBuf>,
    iterations: usize,
    json_output: Option<PathBuf>,
    acceptance_p95_us: f64,
    acceptance_max_us: f64,
}

impl CliArgs {
    fn parse() -> Result<Self, String> {
        let mut dataset_path = None;
        let mut iterations = 100_usize;
        let mut json_output = None;
        let mut acceptance_p95_us = 10_000.0_f64;
        let mut acceptance_max_us = 50_000.0_f64;
        let mut arguments = std::env::args().skip(1);

        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--iterations" => {
                    let value = arguments
                        .next()
                        .ok_or_else(|| "--iterations requires a value".to_string())?;
                    iterations = value.parse::<usize>().map_err(|error| error.to_string())?;
                }
                "--json-output" => {
                    json_output =
                        Some(PathBuf::from(arguments.next().ok_or_else(|| {
                            "--json-output requires a path".to_string()
                        })?));
                }
                "--acceptance-p95-us" => {
                    acceptance_p95_us = arguments
                        .next()
                        .ok_or_else(|| "--acceptance-p95-us requires a value".to_string())?
                        .parse::<f64>()
                        .map_err(|error| error.to_string())?;
                }
                "--acceptance-max-us" => {
                    acceptance_max_us = arguments
                        .next()
                        .ok_or_else(|| "--acceptance-max-us requires a value".to_string())?
                        .parse::<f64>()
                        .map_err(|error| error.to_string())?;
                }
                "--help" | "-h" => {
                    return Err(
                        "usage: profile_monitor_dataset [dataset.csv] [--iterations N] [--json-output PATH] [--acceptance-p95-us US] [--acceptance-max-us US]".to_string(),
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
            json_output,
            acceptance_p95_us,
            acceptance_max_us,
        })
    }
}

#[derive(Clone, Debug, Serialize)]
struct ProfileSummary {
    dataset_path: String,
    host_os: String,
    host_arch: String,
    rows_per_iteration: usize,
    iterations: usize,
    total_monitor_evaluations: u64,
    wall_time_s: f64,
    throughput_evaluations_per_s: f64,
    latency_mean_us: f64,
    latency_p95_us: f64,
    latency_max_us: f64,
    final_trusted_verdicts: u64,
    final_flagged_verdicts: u64,
    final_rejected_verdicts: u64,
    type_sizes_bytes: TypeSizeSummary,
    acceptance_p95_us: f64,
    acceptance_max_us: f64,
    accepted: bool,
    note: String,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct TypeSizeSummary {
    monitor_dataset_row: usize,
    eskf_state: usize,
    state_covariance: usize,
    innovation_vector: usize,
    innovation_covariance: usize,
    statistical_monitor: usize,
    ewma_risk_accumulator: usize,
    evidence_packet: usize,
    signed_evidence_packet: usize,
}

impl TypeSizeSummary {
    const fn current() -> Self {
        Self {
            monitor_dataset_row: size_of::<MonitorDatasetRow>(),
            eskf_state: size_of::<EskfState>(),
            state_covariance: size_of::<StateCovariance>(),
            innovation_vector: size_of::<InnovationVector>(),
            innovation_covariance: size_of::<InnovationCovariance>(),
            statistical_monitor: size_of::<StatisticalMonitor>(),
            ewma_risk_accumulator: size_of::<EwmaRiskAccumulator>(),
            evidence_packet: size_of::<EvidencePacket>(),
            signed_evidence_packet: size_of::<SignedEvidencePacket>(),
        }
    }
}
