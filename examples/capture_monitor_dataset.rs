use std::{io::Write, path::PathBuf, process::ExitCode};

use nalgebra::{UnitQuaternion, Vector3};

use rtvlas::{
    benchmark::MonitorDatasetRow,
    ekf_core::{
        predict::predict_in_place,
        state::{EskfState, ImuNoiseModel, NominalState, PredictConfig, StateCovariance},
    },
    telemetry_adapter::{MavlinkSubscriber, TelemetryUpdate, purge_frame_buffer},
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("dataset capture failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let connection =
        argument_value("--connection").unwrap_or_else(|| "udpout:127.0.0.1:18570".to_owned());
    let samples = argument_value("--samples")
        .map(|value| value.parse::<usize>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(60);
    let output_path = argument_value("--output")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_monitor_dataset.csv"));

    if let Some(parent_directory) = output_path.parent() {
        std::fs::create_dir_all(parent_directory).map_err(|error| error.to_string())?;
    }

    let subscriber = MavlinkSubscriber::bind(&connection).map_err(|error| error.to_string())?;
    for _ in 0..3 {
        subscriber
            .announce_ground_station()
            .map_err(|error| error.to_string())?;
        subscriber
            .request_standard_message_streams()
            .map_err(|error| error.to_string())?;
    }

    let mut state = initial_eskf_state();
    let predict_config = predict_config();
    let file = std::fs::File::create(&output_path).map_err(|error| error.to_string())?;
    let mut writer = csv::Writer::from_writer(file);
    let mut subscriber = subscriber;
    let mut captured_samples = 0_usize;

    println!("Capturing {samples} synchronized samples from {connection}");
    println!("Writing dataset to {}", output_path.display());
    let _ = std::io::stdout().flush();

    while captured_samples < samples {
        match subscriber.recv_next().map_err(|error| error.to_string())? {
            TelemetryUpdate::Imu {
                sample,
                mut raw_frame,
            } => {
                predict_in_place(&mut state, &predict_config, &sample)
                    .map_err(|error| error.to_string())?;
                subscriber.record_predicted_state(&state);
                purge_frame_buffer(&mut raw_frame);
            }
            TelemetryUpdate::GpsObservationQueued { .. } => {}
        }

        while let Some(mut synchronized_sample) = subscriber
            .try_dequeue_synchronized_gps()
            .map_err(|error| error.to_string())?
        {
            writer
                .serialize(MonitorDatasetRow::from_synchronized_sample(
                    &synchronized_sample,
                ))
                .map_err(|error| error.to_string())?;
            writer.flush().map_err(|error| error.to_string())?;
            purge_frame_buffer(&mut synchronized_sample.raw_frame);

            captured_samples += 1;
            println!(
                "captured sample #{:03} at t={:.3}s",
                captured_samples, synchronized_sample.gps_observation.timestamp_s
            );
            let _ = std::io::stdout().flush();

            if captured_samples >= samples {
                break;
            }
        }
    }

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

fn argument_value(flag: &str) -> Option<String> {
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == flag {
            return arguments.next();
        }
    }
    None
}
