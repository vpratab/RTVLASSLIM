use std::{path::PathBuf, process::ExitCode};

use rtvlas::{
    statistical_monitor::{
        monitor::{
            ClockBiasPersistenceConfig, HorizontalResidualPersistenceConfig, ImmediateTriggerConfig,
        },
        observation::ChiSquareThresholdConfig,
    },
    texbat_harness::{
        TEXBAT_SCENARIO_2_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_2_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        TEXBAT_SCENARIO_3_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_3_SPOOF_ONSET_AFTER_SCENARIO_2_START_S,
        TEXBAT_SCENARIO_7_OFFSET_FROM_CLEAN_S,
        TEXBAT_SCENARIO_7_SPOOF_ONSET_AFTER_SCENARIO_2_START_S, TexbatMonitorProfile,
        TexbatScenarioConfig, run_texbat_scenario_with_profile, scenario_onset_in_file_seconds,
    },
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("TEXBAT ablation failed: {error}");
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
    let profiles = vec![
        TexbatMonitorProfile::full(0.6).named("full"),
        TexbatMonitorProfile {
            profile_name: "no_horiz_cusum".to_owned(),
            ewma_alpha: 0.6,
            use_clock_bias_observation: true,
            clock_bias_persistence: Some(ClockBiasPersistenceConfig::new(0.9, 92.0)),
            horizontal_residual_persistence: None,
            immediate_triggers: None,
        },
        TexbatMonitorProfile {
            profile_name: "full_hybrid".to_owned(),
            ewma_alpha: 0.6,
            use_clock_bias_observation: true,
            clock_bias_persistence: Some(ClockBiasPersistenceConfig::new(0.9, 92.0)),
            horizontal_residual_persistence: Some(HorizontalResidualPersistenceConfig::new(
                0.2, 65.0,
            )),
            immediate_triggers: Some(ImmediateTriggerConfig::gps_only(Some(64.0), Some(196.0))),
        },
        TexbatMonitorProfile {
            profile_name: "no_persistence".to_owned(),
            ewma_alpha: 0.6,
            use_clock_bias_observation: true,
            clock_bias_persistence: None,
            horizontal_residual_persistence: None,
            immediate_triggers: None,
        },
        TexbatMonitorProfile {
            profile_name: "single_epoch_gps_clock".to_owned(),
            ewma_alpha: 1.0,
            use_clock_bias_observation: true,
            clock_bias_persistence: None,
            horizontal_residual_persistence: None,
            immediate_triggers: None,
        },
        TexbatMonitorProfile {
            profile_name: "single_epoch_gps_only".to_owned(),
            ewma_alpha: 1.0,
            use_clock_bias_observation: false,
            clock_bias_persistence: None,
            horizontal_residual_persistence: None,
            immediate_triggers: None,
        },
    ];

    println!(
        "{:<8} {:<22} {:>8} {:>8} {:>8} {:>8}",
        "Scenario", "Profile", "AnomTPR", "AnomFPR", "RejTPR", "RejFPR"
    );
    println!("{}", "-".repeat(72));

    for config in &scenario_configs {
        for profile in &profiles {
            let report = run_texbat_scenario_with_profile(config, thresholds, profile)
                .map_err(|error| error.to_string())?;
            println!(
                "{:<8} {:<22} {:>8.3} {:>8.3} {:>8.3} {:>8.3}",
                report.scenario_name,
                report.profile_name,
                report.anomaly_true_positive_rate(),
                report.anomaly_false_positive_rate(),
                report.rejected_true_positive_rate(),
                report.rejected_false_positive_rate(),
            );
        }
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
