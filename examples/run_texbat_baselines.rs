use std::{path::PathBuf, process::ExitCode};

use rtvlas::{
    statistical_monitor::observation::ChiSquareThresholdConfig,
    texbat_harness::baselines::{
        InnovationSigmaBaselineConfig, NaiveDistanceBaselineConfig, run_innovation_sigma_baseline,
        run_naive_distance_baseline,
    },
    texbat_harness::{
        TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_2_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        TEXBAT_SCENARIO_7_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_7_SPOOF_ONSET_AFTER_SCENARIO_2_START_S, TexbatScenarioConfig,
        run_texbat_scenario, scenario_onset_in_file_seconds,
    },
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("TEXBAT baseline benchmark failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let texbat_dir = path_argument("--texbat-dir")
        .or_else(first_positional_argument)
        .unwrap_or_else(|| PathBuf::from("artifacts/texbat"));
    let distance_threshold_m = parse_f32_argument("--distance-threshold-m")?.unwrap_or(5.0);
    let innovation_nsigma = parse_f32_argument("--innovation-nsigma")?.unwrap_or(3.0);
    let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
    let ewma_alpha = 0.6;
    let clean_navsol = texbat_dir.join("cleanStatic_navsol.mat");
    let scenarios = vec![
        configure_processed_navsol_proxy(TexbatScenarioConfig::new(
            "cleanStatic",
            clean_navsol.clone(),
            clean_navsol.clone(),
            0.0,
            None,
        )),
        scenario_config(
            "ds2",
            clean_navsol.clone(),
            texbat_dir.join("ds2_navsol.mat"),
            TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S,
            TEXBAT_SCENARIO_2_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        ),
        scenario_config(
            "ds3",
            clean_navsol.clone(),
            texbat_dir.join("ds3_navsol.mat"),
            TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
            TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        ),
        {
            let mut config = configure_processed_navsol_proxy(TexbatScenarioConfig::new(
                "ds7",
                clean_navsol,
                texbat_dir.join("ds7_navsol.mat"),
                TEXBAT_SCENARIO_7_OFFSET_FROM_CLEAN_S,
                Some(scenario_onset_in_file_seconds(
                    TEXBAT_SCENARIO_7_OFFSET_FROM_CLEAN_S,
                    TEXBAT_SCENARIO_7_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
                )),
            ));
            config.alignment_search_window_s = 1.0;
            config.alignment_scale_search_window = 0.0;
            config
        },
    ];

    println!(
        "Thresholds: naive distance = {:.1} m, innovation = {:.1} sigma",
        distance_threshold_m, innovation_nsigma
    );
    println!(
        "{:<12} {:>9} {:>9} {:>12} {:>12} {:>14} {:>14}",
        "Scenario", "FullTPR", "FullFPR", "NaiveTPR", "NaiveFPR", "InnovTPR", "InnovFPR"
    );
    println!("{}", "-".repeat(90));

    for scenario in &scenarios {
        let full_report =
            run_texbat_scenario(scenario, thresholds, ewma_alpha).map_err(|e| e.to_string())?;
        let naive_report = run_naive_distance_baseline(
            scenario,
            NaiveDistanceBaselineConfig::new(distance_threshold_m),
        )
        .map_err(|e| e.to_string())?;
        let innovation_report = run_innovation_sigma_baseline(
            scenario,
            InnovationSigmaBaselineConfig::new(innovation_nsigma),
        )
        .map_err(|e| e.to_string())?;

        println!(
            "{:<12} {:>9.3} {:>9.3} {:>12.3} {:>12.3} {:>14.3} {:>14.3}",
            scenario.scenario_name,
            full_report.anomaly_true_positive_rate(),
            full_report.anomaly_false_positive_rate(),
            naive_report.true_positive_rate(),
            naive_report.false_positive_rate(),
            innovation_report.true_positive_rate(),
            innovation_report.false_positive_rate(),
        );
    }

    println!();
    println!("Per-scenario thresholds used:");
    for scenario in &scenarios {
        println!(
            "  {}: naive distance {:.1} m, innovation {:.1} sigma",
            scenario.scenario_name, distance_threshold_m, innovation_nsigma
        );
    }

    Ok(())
}

fn scenario_config(
    scenario_name: &str,
    clean_navsol_path: PathBuf,
    observed_navsol_path: PathBuf,
    scenario_offset_from_clean_s: f64,
    spoof_onset_after_scenario_start_s: f64,
) -> TexbatScenarioConfig {
    let mut config = configure_processed_navsol_proxy(TexbatScenarioConfig::new(
        scenario_name,
        clean_navsol_path,
        observed_navsol_path,
        scenario_offset_from_clean_s,
        Some(scenario_onset_in_file_seconds(
            scenario_offset_from_clean_s,
            spoof_onset_after_scenario_start_s,
        )),
    ));
    config.alignment_search_window_s = 20.0;
    config.alignment_scale_search_window = 0.0015;
    config
}

fn configure_processed_navsol_proxy(mut config: TexbatScenarioConfig) -> TexbatScenarioConfig {
    config.position_state_std_m = 5.0;
    config.velocity_state_std_mps = 1.0;
    config.gps_horizontal_position_std_m = 5.0;
    config.gps_vertical_position_std_m = 6.0;
    config.gps_horizontal_velocity_std_mps = 1.0;
    config.gps_vertical_velocity_std_mps = 1.5;
    config
}

fn parse_f32_argument(flag: &str) -> Result<Option<f32>, String> {
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == flag {
            return arguments
                .next()
                .ok_or_else(|| format!("missing value for {flag}"))?
                .parse::<f32>()
                .map(Some)
                .map_err(|error| error.to_string());
        }
    }
    Ok(None)
}

fn path_argument(flag: &str) -> Option<PathBuf> {
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == flag {
            return arguments.next().map(PathBuf::from);
        }
    }
    None
}

fn first_positional_argument() -> Option<PathBuf> {
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument.starts_with("--") {
            let _ = arguments.next();
            continue;
        }
        return Some(PathBuf::from(argument));
    }
    None
}
