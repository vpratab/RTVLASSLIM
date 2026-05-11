# Repository Layout

This repository is intentionally split between a small Rust crate core and optional validation or simulator-facing tooling.

## Top Level

- `src/`
  - the crate itself
- `examples/`
  - runnable demonstrations, replay drivers, and live PX4 utilities
- `scripts/`
  - shell and PowerShell helpers for verified local workflows
- `docs/`
  - repository maps and verification notes
- `artifacts/`
  - ignored local outputs such as evidence bundles, benchmark logs, and capture files
- `external/`
  - ignored local dependencies such as the PX4 checkout used for WSL verification

## Crate Modules

- `src/ekf_core/`
  - ESKF nominal state, covariance, and IMU predict step
- `src/statistical_monitor/`
  - GPS, barometer, heading, and clock-bias residual evaluation
- `src/telemetry_adapter/`
  - MAVLink ingestion plus geodetic and timing conversion utilities
- `src/attestation/`
  - signed evidence packet generation and verification
- `src/orchestrator/`
  - process -> sign -> purge mission loop
- `src/validation/`
  - generic CSV replay harness
- `src/benchmark/`
  - PX4 capture/replay benchmark support
- `src/texbat_harness/`
  - processed-TEXBAT replay tooling

## Examples

See [examples/README.md](../examples/README.md) for a categorized map of the executable examples.

## Scripts

See [scripts/README.md](../scripts/README.md) for the verified shell and PowerShell entry points.
