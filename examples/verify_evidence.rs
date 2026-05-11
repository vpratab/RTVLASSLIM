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
    if let Some(first_timestamp_ns) = summary.first_timestamp_ns {
        println!("  first timestamp (ns): {first_timestamp_ns}");
    }
    if let Some(last_timestamp_ns) = summary.last_timestamp_ns {
        println!("  last timestamp (ns): {last_timestamp_ns}");
    }

    Ok(())
}
