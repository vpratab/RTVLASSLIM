use std::{path::PathBuf, process::ExitCode};

use rtvlas::attestation::verify_framed_evidence_bytes;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("evidence verification failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let evidence_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("artifacts/px4_sitl_evidence.bin"));
    let bytes = std::fs::read(&evidence_path).map_err(|error| error.to_string())?;
    let summary = verify_framed_evidence_bytes(&bytes).map_err(|error| error.to_string())?;

    println!("Evidence file: {}", evidence_path.display());
    println!("  packets verified: {}", summary.total_packets);
    println!("  trusted verdicts: {}", summary.trusted_verdicts);
    println!(
        "  flagged/rejected verdicts: {}",
        summary.flagged_or_rejected_verdicts
    );
    println!("  diagnostic packets: {}", summary.diagnostic_packets);
    if summary.diagnostic_packets > 0 {
        println!(
            "  max GPS D_M^2: {:.3}",
            summary.max_gps_squared_mahalanobis_distance
        );
        println!("  max risk: {:.3}", summary.max_accumulated_risk);
        println!(
            "  max horizontal residual (m): {:.3}",
            summary.max_horizontal_position_residual_m
        );
        println!(
            "  max horizontal velocity residual (m/s): {:.3}",
            summary.max_horizontal_velocity_residual_mps
        );
    }
    if let Some(first_timestamp_ns) = summary.first_timestamp_ns {
        println!("  first timestamp (ns): {first_timestamp_ns}");
    }
    if let Some(last_timestamp_ns) = summary.last_timestamp_ns {
        println!("  last timestamp (ns): {last_timestamp_ns}");
    }
    println!(
        "  evidence chain root: {}",
        hex_encode(&summary.evidence_chain_root)
    );

    Ok(())
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}
