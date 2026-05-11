use std::{collections::BTreeMap, io::Write, process::ExitCode};

use mavlink::{
    MavConnection, MavHeader, Message, MessageData, connect,
    dialects::common::{
        COMMAND_LONG_DATA, GLOBAL_POSITION_INT_DATA, GPS_RAW_INT_DATA, HEARTBEAT_DATA,
        HIGHRES_IMU_DATA, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavState, MavType,
    },
};

const GCS_HEADER: MavHeader = MavHeader {
    sequence: 0,
    system_id: 255,
    component_id: 190,
};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("mavlink dump failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let connection =
        argument_value("--connection").unwrap_or_else(|| "udpout:127.0.0.1:18570".to_owned());
    let frame_limit = argument_value("--frame-limit")
        .map(|value| value.parse::<usize>().map_err(|error| error.to_string()))
        .transpose()?
        .unwrap_or(40);

    let connection = connect::<MavMessage>(&connection).map_err(|error| error.to_string())?;
    announce_ground_station(&connection)?;
    request_streams(&connection)?;

    let mut counts = BTreeMap::<u32, usize>::new();

    for index in 1..=frame_limit {
        let (_header, message) = connection.recv().map_err(|error| error.to_string())?;
        *counts.entry(message.message_id()).or_default() += 1;

        let label = match &message {
            MavMessage::HEARTBEAT(_) => "HEARTBEAT",
            MavMessage::HIGHRES_IMU(_) => "HIGHRES_IMU",
            MavMessage::GLOBAL_POSITION_INT(_) => "GLOBAL_POSITION_INT",
            MavMessage::GPS_RAW_INT(_) => "GPS_RAW_INT",
            _ => "OTHER",
        };

        println!(
            "[{index:02}] id={} type={label} payload={:?}",
            message.message_id(),
            truncated_debug(&message),
        );
        let _ = std::io::stdout().flush();
    }

    println!("Message id counts:");
    for (message_id, count) in counts {
        println!("  {message_id}: {count}");
    }
    let _ = std::io::stdout().flush();

    Ok(())
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

fn argument_value(flag: &str) -> Option<String> {
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        if argument == flag {
            return arguments.next();
        }
    }
    None
}

fn truncated_debug(message: &MavMessage) -> String {
    let rendered = format!("{message:?}");
    const MAX_LEN: usize = 180;
    if rendered.len() <= MAX_LEN {
        rendered
    } else {
        format!("{}...", &rendered[..MAX_LEN])
    }
}
