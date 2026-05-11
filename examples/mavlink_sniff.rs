use std::{io::Write, process::ExitCode};

use rtvlas::telemetry_adapter::{MavlinkSubscriber, TelemetryUpdate};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("mavlink sniff failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let connection =
        argument_value("--connection").unwrap_or_else(|| "udpout:127.0.0.1:14550".to_owned());
    let event_limit = argument_value("--event-limit")
        .map(|value| value.parse::<usize>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(12);
    let gps_limit = argument_value("--gps-limit")
        .map(|value| value.parse::<usize>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(0);
    let suppress_imu = has_flag("--suppress-imu");

    let mut subscriber = MavlinkSubscriber::bind(&connection).map_err(|error| error.to_string())?;
    for _ in 0..3 {
        subscriber
            .announce_ground_station()
            .map_err(|error| error.to_string())?;
        subscriber
            .request_standard_message_streams()
            .map_err(|error| error.to_string())?;
    }

    println!("Listening for PX4-targeted telemetry on {connection}");
    println!("Expecting {event_limit} matching events");
    if gps_limit > 0 {
        println!("Will exit after {gps_limit} GPS observations");
    }
    let _ = std::io::stdout().flush();

    let mut received_events = 0_usize;
    let mut gps_events = 0_usize;
    while received_events < event_limit && (gps_limit == 0 || gps_events < gps_limit) {
        match subscriber.recv_next().map_err(|error| error.to_string())? {
            TelemetryUpdate::Imu { sample, raw_frame } => {
                received_events += 1;
                if !suppress_imu {
                    println!(
                        "[{received_events:02}] IMU t={:.3}s accel=({:.3}, {:.3}, {:.3}) gyro=({:.3}, {:.3}, {:.3}) bytes={}",
                        sample.timestamp_s,
                        sample.accel_body_mps2.x,
                        sample.accel_body_mps2.y,
                        sample.accel_body_mps2.z,
                        sample.gyro_body_rps.x,
                        sample.gyro_body_rps.y,
                        sample.gyro_body_rps.z,
                        raw_frame.len(),
                    );
                    let _ = std::io::stdout().flush();
                }
            }
            TelemetryUpdate::GpsObservationQueued {
                timestamp_s,
                queue_depth,
            } => {
                received_events += 1;
                gps_events += 1;
                println!(
                    "[{received_events:02}] GPS queued t={timestamp_s:.3}s queue_depth={queue_depth}"
                );
                let _ = std::io::stdout().flush();
            }
        }
    }

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

fn has_flag(flag: &str) -> bool {
    std::env::args().skip(1).any(|argument| argument == flag)
}
