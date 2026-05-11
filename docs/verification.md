# Verification Guide

These are the verification commands that have been used locally on this repository.

## Core Rust Checks

```powershell
cargo check --no-default-features --lib
cargo check --all-targets
cargo test --lib
cargo check --examples
```

## Quick Examples

```powershell
cargo run --example gps_spoof
cargo run --example run_validation
cargo run --example run_texbat_harness
```

## WSL PX4 Paths

These paths assume PX4 SIH has already been built inside the ignored local `external/PX4-Autopilot` checkout.

```bash
bash scripts/wsl_inline_sniff.sh --connection udpout:127.0.0.1:18570 --event-limit 500 --gps-limit 1 --suppress-imu
bash scripts/wsl_inline_live.sh
bash scripts/wsl_px4_benchmark.sh 60
bash scripts/wsl_px4_live_spoof.sh
```

## External Data

Processed TEXBAT helper downloads:

```powershell
.\scripts\download_texbat_processed.ps1
```

or:

```bash
bash scripts/download_texbat_processed.sh
```

The repository README remains the source of truth for the exact verification results that have already been observed.
