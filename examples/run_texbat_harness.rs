use std::{path::PathBuf, process::ExitCode};

use rtvlas::{
    statistical_monitor::observation::ChiSquareThresholdConfig,
    texbat_harness::{
        TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_2_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        TEXBAT_SCENARIO_7_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_7_SPOOF_ONSET_AFTER_SCENARIO_2_START_S, TexbatScenarioConfig,
        run_texbat_scenario, scenario_onset_in_file_seconds, write_texbat_replay_csv,
    },
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("TEXBAT harness failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let texbat_dir = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/texbat"));

    let clean_navsol = texbat_dir.join("cleanStatic_navsol.mat");
    let scenario_configs = vec![
        configure_processed_navsol_proxy(TexbatScenarioConfig::new(
            "cleanStatic-baseline",
            clean_navsol.clone(),
            clean_navsol.clone(),
            0.0,
            None,
        )),
        {
            let mut config = configure_processed_navsol_proxy(TexbatScenarioConfig::new(
                "ds2",
                clean_navsol.clone(),
                texbat_dir.join("ds2_navsol.mat"),
                TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S,
                Some(scenario_onset_in_file_seconds(
                    TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S,
                    TEXBAT_SCENARIO_2_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
                )),
            ));
            config.alignment_search_window_s = 20.0;
            config.alignment_scale_search_window = 0.0015;
            config
        },
        {
            let mut config = configure_processed_navsol_proxy(TexbatScenarioConfig::new(
                "ds3",
                clean_navsol.clone(),
                texbat_dir.join("ds3_navsol.mat"),
                TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
                Some(scenario_onset_in_file_seconds(
                    TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
                    TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
                )),
            ));
            config.alignment_search_window_s = 20.0;
            config.alignment_scale_search_window = 0.0015;
            config
        },
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
    let thresholds = ChiSquareThresholdConfig::new(12.592, 22.458);
    let ewma_alpha = 0.6;

    for config in scenario_configs {
        let csv_path = texbat_dir.join(format!("{}_replay.csv", config.scenario_name));
        write_texbat_replay_csv(&config, &csv_path).map_err(|error| error.to_string())?;
        let report = run_texbat_scenario(&config, thresholds, ewma_alpha)
            .map_err(|error| error.to_string())?;

        println!("Scenario: {}", report.scenario_name);
        println!("  replay CSV: {}", csv_path.display());
        println!("  samples: {}", report.total_samples);
        println!(
            "  trusted/flagged/rejected: {}/{}/{}",
            report.trusted_verdicts, report.flagged_verdicts, report.rejected_verdicts
        );
        println!(
            "  anomaly TPR/FPR: {:.3}/{:.3}",
            report.anomaly_true_positive_rate(),
            report.anomaly_false_positive_rate()
        );
        println!(
            "  rejected TPR/FPR: {:.3}/{:.3}",
            report.rejected_true_positive_rate(),
            report.rejected_false_positive_rate()
        );
        println!(
            "  eval latency mean/p95/max (us): {:.2}/{:.2}/{:.2}",
            report.mean_evaluation_latency_us,
            report.p95_evaluation_latency_us,
            report.max_evaluation_latency_us
        );
        println!(
            "  calibrated clean alignment offset (s): {:.6}",
            report.calibrated_alignment_offset_s
        );
        println!(
            "  calibrated clean alignment scale: {:.9}",
            report.calibrated_alignment_scale
        );
        println!(
            "  calibration position bias NED (m): {:.2}, {:.2}, {:.2}",
            report.position_bias_calibration_ned_m.x,
            report.position_bias_calibration_ned_m.y,
            report.position_bias_calibration_ned_m.z
        );
        println!(
            "  calibration clock bias (m): {:.2}",
            report.clock_bias_calibration_m
        );
    }

    Ok(())
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
