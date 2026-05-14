# Examples

The `examples/` directory is grouped by purpose rather than by verification status.

## Fast Local Smoke Tests

- `gps_spoof.rs`
  - synthetic nominal vs spoofed GPS monitor behavior
- `run_validation.rs`
  - replay the included `synthetic_validation.csv` sample through the generic validation harness

## MAVLink / PX4 Utilities

- `mavlink_dump.rs`
  - print and classify live MAVLink traffic
- `mavlink_sniff.rs`
  - queue and inspect monitor-relevant MAVLink observations
- `px4_sitl_live.rs`
  - run the full orchestrator against live PX4 MAVLink telemetry
- `px4_spoof_proxy.rs`
  - act as a MAVLink man-in-the-middle and mutate `GLOBAL_POSITION_INT` live

## Benchmark / Capture Drivers

- `capture_monitor_dataset.rs`
  - record synchronized monitor inputs from MAVLink telemetry, either passively (`--mission-profile passive`) or while driving a simple PX4 SIH offboard mission profile (`hover`, `forward`, `turn`, `climb`)
- `run_monitor_benchmark.rs`
  - replay a recorded dataset and summarize nominal vs spoofed behavior
- `report_nominal_dataset.rs`
  - turn any captured nominal monitor dataset into a reviewer-friendly false-positive and residual report, with optional JSON export
- `run_adversarial_sweep.rs`
  - evaluate a recorded dataset against default or extended grids of replayed spoof profiles and export JSON / CSV results
- `run_realistic_spoof_suite.rs`
  - evaluate a recorded dataset against built-in TEXBAT-like, UAV-spoofer-like, navigation-deception, and intermittent spoof profiles
- `profile_monitor_dataset.rs`
  - replay a monitor dataset repeatedly to report host-side throughput, latency, and key Rust type sizes

## External Replay Drivers

- `run_texbat_harness.rs`
  - load processed TEXBAT `navsol.mat` files and summarize scenario results
- `run_texbat_baselines.rs`
  - compare the full detector against a naive GPS/DR distance threshold and a standard innovation `N_sigma` baseline on the same processed TEXBAT scenarios

## Local Fixture Data

- `synthetic_validation.csv`
  - small checked-in CSV fixture for the generic replay harness
