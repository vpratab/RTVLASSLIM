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
  - record synchronized monitor inputs from PX4 telemetry, optionally while driving a simple offboard mission profile (`hover`, `forward`, `turn`, `climb`)
- `run_monitor_benchmark.rs`
  - replay a recorded dataset and summarize nominal vs spoofed behavior
- `run_adversarial_sweep.rs`
  - evaluate a recorded dataset against a structured grid of replayed spoof profiles and export JSON / CSV results

## External Replay Drivers

- `run_texbat_harness.rs`
  - load processed TEXBAT `navsol.mat` files and summarize scenario results
- `run_texbat_baselines.rs`
  - compare the full detector against a naive GPS/DR distance threshold and a standard innovation `N_sigma` baseline on the same processed TEXBAT scenarios

## Local Fixture Data

- `synthetic_validation.csv`
  - small checked-in CSV fixture for the generic replay harness
