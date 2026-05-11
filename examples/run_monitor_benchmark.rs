use std::{path::PathBuf, process::ExitCode};

use nalgebra::Vector3;

use rtvlas::{
    benchmark::{
        SpoofInjectionConfig, run_monitor_dataset_file, write_spoofed_monitor_dataset_file,
    },
    statistical_monitor::observation::ChiSquareThresholdConfig,
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("benchmark run failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let input_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_monitor_dataset.csv"));
    let spoofed_path = std::env::args()
        .nth(2)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_monitor_dataset_spoofed.csv"));
    let onset_time_s = argument_value("--onset")
        .map(|value| value.parse::<f64>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(3.0);
    let ramp_duration_s = argument_value("--ramp")
        .map(|value| value.parse::<f64>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(1.0);
    let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
    let ewma_alpha = 0.6;

    write_spoofed_monitor_dataset_file(
        &input_path,
        &spoofed_path,
        SpoofInjectionConfig::new(
            onset_time_s,
            ramp_duration_s,
            Vector3::new(75.0, -40.0, 12.0),
            Vector3::new(8.0, -4.0, 1.0),
        ),
    )
    .map_err(|error| error.to_string())?;

    let nominal_report = run_monitor_dataset_file(&input_path, thresholds, ewma_alpha)
        .map_err(|error| error.to_string())?;
    let spoofed_report = run_monitor_dataset_file(&spoofed_path, thresholds, ewma_alpha)
        .map_err(|error| error.to_string())?;

    println!("Nominal dataset: {}", input_path.display());
    println!("  samples: {}", nominal_report.total_samples);
    println!(
        "  trusted/flagged/rejected: {}/{}/{}",
        nominal_report.trusted_verdicts,
        nominal_report.flagged_verdicts,
        nominal_report.rejected_verdicts
    );
    println!(
        "  anomaly FPR: {:.3}",
        nominal_report.anomaly_false_positive_rate()
    );
    println!(
        "  rejected FPR: {:.3}",
        nominal_report.rejected_false_positive_rate()
    );
    println!(
        "  eval latency mean/p95/max (us): {:.2}/{:.2}/{:.2}",
        nominal_report.mean_evaluation_latency_us,
        nominal_report.p95_evaluation_latency_us,
        nominal_report.max_evaluation_latency_us
    );

    println!("Spoofed dataset: {}", spoofed_path.display());
    println!("  samples: {}", spoofed_report.total_samples);
    println!(
        "  trusted/flagged/rejected: {}/{}/{}",
        spoofed_report.trusted_verdicts,
        spoofed_report.flagged_verdicts,
        spoofed_report.rejected_verdicts
    );
    println!(
        "  anomaly TPR/FPR: {:.3}/{:.3}",
        spoofed_report.anomaly_true_positive_rate(),
        spoofed_report.anomaly_false_positive_rate()
    );
    println!(
        "  rejected TPR/FPR: {:.3}/{:.3}",
        spoofed_report.rejected_true_positive_rate(),
        spoofed_report.rejected_false_positive_rate()
    );
    println!(
        "  eval latency mean/p95/max (us): {:.2}/{:.2}/{:.2}",
        spoofed_report.mean_evaluation_latency_us,
        spoofed_report.p95_evaluation_latency_us,
        spoofed_report.max_evaluation_latency_us
    );

    Ok(())
}

fn argument_value(flag: &str) -> Option<String> {
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == flag {
            return arguments.next();
        }
    }
    None
}
