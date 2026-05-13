use std::{fs, path::PathBuf, process::ExitCode};

use rtvlas::{
    benchmark::{
        load_monitor_dataset_rows_file,
        sweep::{
            build_default_sweep_cases, run_adversarial_sweep, write_sweep_csv, write_sweep_json,
            write_worst_case_summary,
        },
    },
    statistical_monitor::observation::ChiSquareThresholdConfig,
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("adversarial sweep failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let dataset_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_monitor_dataset.csv"));
    let dataset_label = argument_value("--dataset-label").unwrap_or_else(|| {
        dataset_path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or("px4_monitor_dataset")
            .to_owned()
    });
    let output_dir = argument_value("--output-dir")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts"));
    let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
    let ewma_alpha = 0.6;
    let onset_times_s = parse_f64_list("--onsets").unwrap_or_else(|| vec![2.0, 4.0, 6.0]);
    let ramp_durations_s =
        parse_f64_list("--ramps").unwrap_or_else(|| vec![0.0, 1.0, 2.5, 5.0, 10.0, 20.0]);

    fs::create_dir_all(&output_dir).map_err(|error| error.to_string())?;
    let rows = load_monitor_dataset_rows_file(&dataset_path).map_err(|error| error.to_string())?;
    let cases = build_default_sweep_cases(&onset_times_s, &ramp_durations_s);
    let report = run_adversarial_sweep(&rows, &dataset_label, &cases, thresholds, ewma_alpha)
        .map_err(|error| error.to_string())?;

    let csv_path = output_dir.join(format!("{dataset_label}_adversarial_sweep.csv"));
    let json_path = output_dir.join(format!("{dataset_label}_adversarial_sweep.json"));
    write_sweep_csv(&csv_path, &report).map_err(|error| error.to_string())?;
    write_sweep_json(&json_path, &report).map_err(|error| error.to_string())?;

    println!("Dataset: {}", dataset_path.display());
    println!("Dataset label: {dataset_label}");
    println!("Cases evaluated: {}", report.results.len());
    println!(
        "Nominal trusted/flagged/rejected: {}/{}/{}",
        report.nominal_report.trusted_verdicts,
        report.nominal_report.flagged_verdicts,
        report.nominal_report.rejected_verdicts
    );
    println!(
        "Nominal anomaly FPR: {:.3}",
        report.nominal_report.anomaly_fpr
    );
    write_worst_case_summary(std::io::stdout(), &report).map_err(|error| error.to_string())?;
    println!("CSV export: {}", csv_path.display());
    println!("JSON export: {}", json_path.display());

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

fn parse_f64_list(flag: &str) -> Option<Vec<f64>> {
    argument_value(flag).map(|value| {
        value
            .split(',')
            .filter_map(|part| {
                let trimmed = part.trim();
                if trimmed.is_empty() {
                    None
                } else {
                    trimmed.parse::<f64>().ok()
                }
            })
            .collect()
    })
}
