use std::{io::Write, path::PathBuf, process::ExitCode, time::Instant};

use nalgebra::{UnitQuaternion, Vector3};

use rtvlas::{
    attestation::{Ed25519AttestationProvider, MockSecureElement},
    ekf_core::state::{EskfState, ImuNoiseModel, NominalState, PredictConfig, StateCovariance},
    orchestrator::{FileSink, Orchestrator},
    statistical_monitor::{
        monitor::{
            EwmaRiskAccumulator, HorizontalResidualPersistenceConfig, ImmediateTriggerConfig,
            StatisticalMonitor,
        },
        observation::ChiSquareThresholdConfig,
    },
    telemetry_adapter::MavlinkSubscriber,
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("live PX4 run failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let connection =
        argument_value("--connection").unwrap_or_else(|| "udpout:127.0.0.1:18570".to_owned());
    let skip_handshake = flag_present("--skip-handshake");
    let verdict_limit = argument_value("--verdict-limit")
        .map(|value| value.parse::<u64>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(3);
    let evidence_path = argument_value("--evidence")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_sitl_evidence.bin"));
    let immediate_gps_flag_threshold = optional_f32_argument("--immediate-gps-flag")?;
    let immediate_gps_reject_threshold = optional_f32_argument("--immediate-gps-reject")?;
    let immediate_position_flag_threshold = optional_f32_argument("--immediate-position-flag")?;
    let immediate_position_reject_threshold = optional_f32_argument("--immediate-position-reject")?;
    let disable_horizontal_persistence = flag_present("--disable-horizontal-persistence");
    let horizontal_persistence_slack = optional_f32_argument("--horizontal-persistence-slack")?;
    let horizontal_persistence_reject = optional_f32_argument("--horizontal-persistence-reject")?;

    if let Some(parent_directory) = evidence_path.parent() {
        std::fs::create_dir_all(parent_directory).map_err(|error| error.to_string())?;
    }

    eprintln!("binding live subscriber on {connection}");
    let subscriber = MavlinkSubscriber::bind(&connection).map_err(|error| error.to_string())?;
    if !skip_handshake {
        for _ in 0..3 {
            subscriber
                .announce_ground_station()
                .map_err(|error| error.to_string())?;
            subscriber
                .request_standard_message_streams()
                .map_err(|error| error.to_string())?;
        }
    }
    eprintln!("subscriber ready");

    let secure_element = match std::env::var("RTVLAS_SECRET_KEY") {
        Ok(_) => {
            MockSecureElement::from_env("RTVLAS_SECRET_KEY").map_err(|error| error.to_string())?
        }
        Err(_) => MockSecureElement::from_secret_key_bytes([11_u8; 32]),
    };
    let attestation_provider = Ed25519AttestationProvider::new(secure_element);
    let evidence_sink = FileSink::create(&evidence_path).map_err(|error| error.to_string())?;

    let mut orchestrator = Orchestrator::new(
        subscriber,
        initial_eskf_state(),
        predict_config(),
        monitor(
            immediate_gps_flag_threshold,
            immediate_gps_reject_threshold,
            immediate_position_flag_threshold,
            immediate_position_reject_threshold,
            disable_horizontal_persistence,
            horizontal_persistence_slack,
            horizontal_persistence_reject,
        ),
        attestation_provider,
        evidence_sink,
    );

    println!("Starting live PX4 SITL capture on {connection}");
    println!("Evidence output: {}", evidence_path.display());
    println!("Stopping after {verdict_limit} verdicts");
    let _ = std::io::stdout().flush();

    let started_at = Instant::now();
    let mut last_progress_report = Instant::now();
    while orchestrator.mission_report().verdicts_emitted < verdict_limit {
        let outcome = orchestrator.step().map_err(|error| error.to_string())?;

        if last_progress_report.elapsed().as_secs_f32() >= 2.0 {
            let report = orchestrator.mission_report();
            println!(
                "progress after {:.2?}: total={} imu={} gps={} verdicts={}",
                started_at.elapsed(),
                report.total_packets_processed,
                report.imu_packets_processed,
                report.gps_packets_processed,
                report.verdicts_emitted,
            );
            let _ = std::io::stdout().flush();
            last_progress_report = Instant::now();
        }

        if let Some(trust_level) = outcome.trust_level {
            let report = orchestrator.mission_report();
            println!(
                "verdict #{:02} {:?} after {:.2?} (trusted={}, flagged={}, rejected={})",
                report.verdicts_emitted,
                trust_level,
                started_at.elapsed(),
                report.trusted_verdicts,
                report.flagged_verdicts,
                report.rejected_verdicts,
            );
            let _ = std::io::stdout().flush();
        }
    }

    let report = orchestrator.mission_report();
    println!("Mission report:");
    println!(
        "  total packets processed: {}",
        report.total_packets_processed
    );
    println!("  imu packets processed: {}", report.imu_packets_processed);
    println!("  gps packets processed: {}", report.gps_packets_processed);
    println!("  verdicts emitted: {}", report.verdicts_emitted);
    println!(
        "  trusted/flagged/rejected: {}/{}/{}",
        report.trusted_verdicts, report.flagged_verdicts, report.rejected_verdicts
    );
    println!(
        "  rejected percentage: {:.2}%",
        report.rejected_percentage()
    );
    let _ = std::io::stdout().flush();

    Ok(())
}

fn initial_eskf_state() -> EskfState {
    EskfState::new(
        NominalState {
            timestamp_s: 0.0,
            position_ned_m: Vector3::zeros(),
            velocity_ned_mps: Vector3::zeros(),
            attitude_body_to_ned: UnitQuaternion::identity(),
            accel_bias_mps2: Vector3::zeros(),
            gyro_bias_rps: Vector3::zeros(),
            geodetic_reference: None,
        },
        StateCovariance::identity() * 1.0e-3,
    )
}

fn predict_config() -> PredictConfig {
    PredictConfig::new(
        Vector3::new(0.0, 0.0, 9.80665),
        0.02,
        ImuNoiseModel::new(
            Vector3::new(0.05, 0.05, 0.05),
            Vector3::new(0.002, 0.002, 0.002),
            Vector3::new(0.0002, 0.0002, 0.0002),
            Vector3::new(0.00002, 0.00002, 0.00002),
        ),
    )
}

fn monitor(
    immediate_gps_flag_threshold: Option<f32>,
    immediate_gps_reject_threshold: Option<f32>,
    immediate_position_flag_threshold: Option<f32>,
    immediate_position_reject_threshold: Option<f32>,
    disable_horizontal_persistence: bool,
    horizontal_persistence_slack: Option<f32>,
    horizontal_persistence_reject: Option<f32>,
) -> StatisticalMonitor {
    let mut monitor = StatisticalMonitor::new(
        ChiSquareThresholdConfig::new(12.592, 22.458),
        EwmaRiskAccumulator::new(0.6),
    );

    if !disable_horizontal_persistence {
        monitor =
            monitor.with_horizontal_residual_persistence(HorizontalResidualPersistenceConfig::new(
                horizontal_persistence_slack.unwrap_or(0.2),
                horizontal_persistence_reject.unwrap_or(65.0),
            ));
    }

    if immediate_gps_flag_threshold.is_some()
        || immediate_gps_reject_threshold.is_some()
        || immediate_position_flag_threshold.is_some()
        || immediate_position_reject_threshold.is_some()
    {
        monitor = monitor.with_immediate_triggers(
            ImmediateTriggerConfig::gps_only(
                immediate_gps_flag_threshold,
                immediate_gps_reject_threshold,
            )
            .with_position_residual_thresholds(
                immediate_position_flag_threshold,
                immediate_position_reject_threshold,
            ),
        );
    }

    monitor
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

fn flag_present(flag: &str) -> bool {
    std::env::args().skip(1).any(|argument| argument == flag)
}

fn optional_f32_argument(flag: &str) -> Result<Option<f32>, String> {
    argument_value(flag)
        .map(|value| value.parse::<f32>().map_err(|error| error.to_string()))
        .transpose()
}
