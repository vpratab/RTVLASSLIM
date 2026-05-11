use std::{io::Write, process::ExitCode, time::Instant};

use mavlink::{
    MavConnection, MavHeader, MessageData, connect,
    dialects::common::{
        COMMAND_LONG_DATA, GLOBAL_POSITION_INT_DATA, GPS_RAW_INT_DATA, HEARTBEAT_DATA,
        HIGHRES_IMU_DATA, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavState, MavType,
    },
};
use nalgebra::Vector3;

use rtvlas::telemetry_adapter::{
    GeodeticPosition, offset_geodetic_position_ned, scaled_degrees_e7_to_radians,
};

const GCS_HEADER: MavHeader = MavHeader {
    sequence: 0,
    system_id: 255,
    component_id: 190,
};
const MILLIMETRES_PER_METRE: f64 = 1_000.0;
const CENTIMETRES_PER_SECOND_PER_METRE_PER_SECOND: f32 = 100.0;
const RADIANS_TO_DEGREES_E7: f64 = 180.0 * 10_000_000.0 / std::f64::consts::PI;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("px4 spoof proxy failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let upstream = argument_value("--upstream").unwrap_or_else(|| "udpout:127.0.0.1:18570".to_owned());
    let downstream =
        argument_value("--downstream").unwrap_or_else(|| "udpout:127.0.0.1:18571".to_owned());
    let spoof_onset_s = argument_value("--spoof-onset-s")
        .map(|value| value.parse::<f64>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(1.5);
    let spoof_config = SpoofConfig {
        position_offset_ned_m: Vector3::new(
            parse_f64_argument("--north-offset-m", 90.0)?,
            parse_f64_argument("--east-offset-m", -50.0)?,
            parse_f64_argument("--down-offset-m", 8.0)?,
        ),
        velocity_offset_ned_mps: Vector3::new(
            parse_f32_argument("--north-velocity-offset-mps", 10.0)?,
            parse_f32_argument("--east-velocity-offset-mps", -5.0)?,
            parse_f32_argument("--down-velocity-offset-mps", 1.0)?,
        ),
    };

    eprintln!("proxy upstream: {upstream}");
    eprintln!("proxy downstream: {downstream}");
    eprintln!(
        "spoof onset: {spoof_onset_s:.2}s, position offset NED=({:.1}, {:.1}, {:.1}) m, velocity offset NED=({:.1}, {:.1}, {:.1}) m/s",
        spoof_config.position_offset_ned_m.x,
        spoof_config.position_offset_ned_m.y,
        spoof_config.position_offset_ned_m.z,
        spoof_config.velocity_offset_ned_mps.x,
        spoof_config.velocity_offset_ned_mps.y,
        spoof_config.velocity_offset_ned_mps.z,
    );

    let upstream_connection = connect::<MavMessage>(&upstream).map_err(|error| error.to_string())?;
    let downstream_connection =
        connect::<MavMessage>(&downstream).map_err(|error| error.to_string())?;

    for _ in 0..3 {
        announce_ground_station(&upstream_connection)?;
        request_streams(&upstream_connection)?;
    }

    let started_at = Instant::now();
    let mut gps_frames_forwarded = 0_u64;
    let mut spoofed_gps_frames = 0_u64;
    let mut spoof_announced = false;

    loop {
        let (header, message) = upstream_connection.recv().map_err(|error| error.to_string())?;
        let elapsed_s = started_at.elapsed().as_secs_f64();
        let spoof_enabled = elapsed_s >= spoof_onset_s;

        let outbound_message = match message {
            MavMessage::GLOBAL_POSITION_INT(data) => {
                gps_frames_forwarded += 1;
                let outbound = if spoof_enabled {
                    if !spoof_announced {
                        eprintln!("enabling live GPS spoof at t={elapsed_s:.2}s");
                        spoof_announced = true;
                    }
                    spoofed_gps_frames += 1;
                    MavMessage::GLOBAL_POSITION_INT(spoof_global_position(data, spoof_config)?)
                } else {
                    MavMessage::GLOBAL_POSITION_INT(data)
                };

                if gps_frames_forwarded % 10 == 0 {
                    eprintln!(
                        "proxy progress: gps_forwarded={gps_frames_forwarded} spoofed={spoofed_gps_frames}"
                    );
                }

                outbound
            }
            MavMessage::GPS_RAW_INT(data) => MavMessage::GPS_RAW_INT(data),
            MavMessage::HIGHRES_IMU(data) => MavMessage::HIGHRES_IMU(data),
            other => other,
        };

        downstream_connection
            .send(&header, &outbound_message)
            .map_err(|error| error.to_string())?;
        let _ = std::io::stderr().flush();
    }
}

#[derive(Clone, Copy, Debug)]
struct SpoofConfig {
    position_offset_ned_m: Vector3<f64>,
    velocity_offset_ned_mps: Vector3<f32>,
}

fn announce_ground_station(connection: &mavlink::Connection<MavMessage>) -> Result<(), String> {
    let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GCS,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });
    connection
        .send(&GCS_HEADER, &heartbeat)
        .map_err(|error| error.to_string())?;
    Ok(())
}

fn request_streams(connection: &mavlink::Connection<MavMessage>) -> Result<(), String> {
    for (message_id, interval_us) in [
        (HIGHRES_IMU_DATA::ID, 10_000_i32),
        (GLOBAL_POSITION_INT_DATA::ID, 100_000_i32),
        (GPS_RAW_INT_DATA::ID, 200_000_i32),
    ] {
        let command = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            confirmation: 0,
            param1: message_id as f32,
            param2: interval_us as f32,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });
        connection
            .send(&GCS_HEADER, &command)
            .map_err(|error| error.to_string())?;
    }

    Ok(())
}

fn spoof_global_position(
    mut data: GLOBAL_POSITION_INT_DATA,
    spoof_config: SpoofConfig,
) -> Result<GLOBAL_POSITION_INT_DATA, String> {
    let geodetic_position = GeodeticPosition::new(
        scaled_degrees_e7_to_radians(data.lat).map_err(|error| format!("{error:?}"))?,
        scaled_degrees_e7_to_radians(data.lon).map_err(|error| format!("{error:?}"))?,
        f64::from(data.alt) / MILLIMETRES_PER_METRE,
    )
    .map_err(|error| format!("{error:?}"))?;
    let spoofed_geodetic_position =
        offset_geodetic_position_ned(geodetic_position, spoof_config.position_offset_ned_m)
            .map_err(|error| format!("{error:?}"))?;

    data.lat = radians_to_scaled_degrees_e7(spoofed_geodetic_position.latitude_rad)?;
    data.lon = radians_to_scaled_degrees_e7(spoofed_geodetic_position.longitude_rad)?;
    data.alt = metres_to_millimetres_i32(spoofed_geodetic_position.altitude_m)?;
    data.relative_alt =
        saturating_add_i32(data.relative_alt, metres_to_millimetres_i32(-spoof_config.position_offset_ned_m.z)?);
    data.vx = saturating_add_i16(
        data.vx,
        metres_per_second_to_centimetres_per_second_i16(spoof_config.velocity_offset_ned_mps.x)?,
    );
    data.vy = saturating_add_i16(
        data.vy,
        metres_per_second_to_centimetres_per_second_i16(spoof_config.velocity_offset_ned_mps.y)?,
    );
    data.vz = saturating_add_i16(
        data.vz,
        metres_per_second_to_centimetres_per_second_i16(spoof_config.velocity_offset_ned_mps.z)?,
    );

    Ok(data)
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

fn parse_f64_argument(flag: &str, default_value: f64) -> Result<f64, String> {
    argument_value(flag)
        .map(|value| value.parse::<f64>().map_err(|error| error.to_string()))
        .transpose()
        .map(|value| value.unwrap_or(default_value))
}

fn parse_f32_argument(flag: &str, default_value: f32) -> Result<f32, String> {
    argument_value(flag)
        .map(|value| value.parse::<f32>().map_err(|error| error.to_string()))
        .transpose()
        .map(|value| value.unwrap_or(default_value))
}

fn radians_to_scaled_degrees_e7(value: f64) -> Result<i32, String> {
    let scaled = value * RADIANS_TO_DEGREES_E7;
    if !scaled.is_finite() || scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
        return Err("geodetic coordinate exceeded GLOBAL_POSITION_INT integer range".to_owned());
    }

    Ok(scaled.round() as i32)
}

fn metres_to_millimetres_i32(value: f64) -> Result<i32, String> {
    let scaled = value * MILLIMETRES_PER_METRE;
    if !scaled.is_finite() || scaled < f64::from(i32::MIN) || scaled > f64::from(i32::MAX) {
        return Err("altitude exceeded GLOBAL_POSITION_INT millimetre range".to_owned());
    }

    Ok(scaled.round() as i32)
}

fn metres_per_second_to_centimetres_per_second_i16(value: f32) -> Result<i16, String> {
    let scaled = value * CENTIMETRES_PER_SECOND_PER_METRE_PER_SECOND;
    if !scaled.is_finite() || scaled < f32::from(i16::MIN) || scaled > f32::from(i16::MAX) {
        return Err("velocity exceeded GLOBAL_POSITION_INT centimetres-per-second range".to_owned());
    }

    Ok(scaled.round() as i16)
}

fn saturating_add_i32(base: i32, delta: i32) -> i32 {
    base.saturating_add(delta)
}

fn saturating_add_i16(base: i16, delta: i16) -> i16 {
    base.saturating_add(delta)
}
