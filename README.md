# RTVLAS

`rtvlas` is a Rust prototype for GPS spoofing detection in autonomy telemetry.

It is not a finished product, not a validated deployment claim, and not a research breakthrough on its own. It is a structured prototype that combines:

- IMU-driven ESKF-style state propagation in a local NED frame.
- GPS innovation checking with Mahalanobis distance and EWMA risk accumulation.
- Barometer altitude and magnetometer-derived heading consistency checks.
- MAVLink ingestion for `HIGHRES_IMU`, `GPS_RAW_INT`, and `GLOBAL_POSITION_INT`.
- Signed evidence packets using SHA-256 and Ed25519.
- A process -> sign -> purge orchestrator that explicitly wipes raw MAVLink frame buffers after attestation.
- An offline CSV validation harness for replaying logged traces and summarizing detection outcomes.
- A PX4 SIH capture-and-benchmark path that records synchronized monitor inputs and measures simulator-only detection outcomes plus per-sample evaluation latency.
- A processed-TEXBAT replay harness that aligns clean and spoofed navigation-solution traces and measures scenario-level detection outcomes, including optional clock-bias checks.

## What Is Implemented

- `ekf_core`
  - Nominal state for position, velocity, attitude, accel bias, and gyro bias.
  - IMU predict step with gravity compensation and covariance propagation.
- `statistical_monitor`
  - Position/velocity innovation residuals against GPS observations.
  - Optional barometer and heading residual checks.
  - Cholesky-based Mahalanobis distance evaluation.
  - EWMA risk accumulation and `Trusted` / `Flagged` / `Rejected` verdicts.
- `telemetry_adapter`
  - MAVLink UDP listener for PX4-style telemetry.
  - Geodetic-to-NED conversion and home-position establishment.
  - GPS/IMU time alignment against EKF history.
  - Auxiliary barometer and heading observations derived from `HIGHRES_IMU`.
  - Canonical MAVLink frame capture for evidence hashing.
- `attestation`
  - Compact evidence packet with timestamp, telemetry hash, pass/fail verdict, and state snapshot.
  - `postcard` serialization.
  - Ed25519 signing and verification.
  - Std-only mock secure element backed by env/file-loaded secret material.
- `orchestrator`
  - End-to-end mission loop that processes telemetry, evaluates GPS consistency, signs evidence, writes to a sink, and purges raw frame bytes from memory.
- `validation`
  - CSV-driven offline replay harness with anomaly/rejection TPR/FPR summaries.
- `benchmark`
  - Capture format for synchronized PX4 monitor inputs.
  - Spoofed replay generation by perturbing captured GPS position/velocity.
  - Simulator-only nominal/spoofed report generation with latency statistics.
- `texbat_harness`
  - MAT-file loader for processed TEXBAT `navsol.mat` artifacts.
  - Clean/spoofed scenario alignment using published TEXBAT timing offsets.
  - Optional clock-bias residual checks for time-push spoofing scenarios.
  - Scenario replay CSV export and per-scenario TPR/FPR reporting.

## What Is Not Implemented

- No raw-TEXBAT IF replay or IMU-paired TEXBAT benchmark exists yet.
- No real-world or hardware-flight detection-rate, false-positive, or latency benchmark is claimed here.
- No hardware-backed secure element, HSM, TPM, enclave, or flight-controller integration is present.
- No GPS update is fused back into the filter state yet; GPS is monitored, not used as a measurement update.
- No production persistence or distributed-ledger sink exists; only local file/log sinks are provided.
- The TEXBAT path uses processed `navsol.mat` reference trajectories, not raw IF samples and not paired IMU from TEXBAT.

## Verification Performed

The code in this repository has been locally verified with:

```powershell
cargo test --lib
cargo check --all-targets
```

and, inside WSL2 Ubuntu with PX4 SIH built from the local `external/PX4-Autopilot` clone:

```bash
bash scripts/wsl_inline_sniff.sh --connection udpout:127.0.0.1:18570 --event-limit 500 --gps-limit 1 --suppress-imu
bash scripts/wsl_inline_live.sh
bash scripts/wsl_px4_benchmark.sh 60
```

and, on the local processed TEXBAT artifacts downloaded from the UT Radionavigation Laboratory:

```powershell
.\scripts\download_texbat_processed.ps1
cargo run --example run_texbat_harness
```

At the time of writing, the crate passes 19 library tests covering:

- IMU propagation staying stable at rest.
- GPS innovation rejection on a large spoof-like offset.
- Combined barometer and heading anomaly rejection.
- Geodetic projection and state interpolation.
- Live UDP/MAVLink loopback ingestion into the telemetry adapter.
- Ed25519 evidence signing and tamper detection.
- An orchestrator integration path that emits a rejected verdict and persists signed evidence.
- Offline CSV validation report generation.
- First-sample EKF timestamp bootstrapping for live PX4 startup.

The live PX4 verification that has actually been run is narrow and specific:

- PX4 SIH was built and launched inside WSL2.
- The local PX4 startup script was patched to stream `GPS_RAW_INT` and `HIGHRES_IMU` on the GCS MAVLink port (`18570`).
- `scripts/wsl_inline_sniff.sh` confirmed `HIGHRES_IMU` and queued GPS observations on `udpout:127.0.0.1:18570`.
- `scripts/wsl_inline_live.sh` completed an end-to-end orchestrator run and emitted 3 signed `Trusted` verdicts over 72 processed packets (`69` IMU, `3` GPS), producing a non-empty evidence file at `artifacts/wsl_px4_sitl_evidence.bin`.
- `scripts/wsl_px4_benchmark.sh 60` captured 60 synchronized PX4 SIH samples on `2026-05-10`, replayed them as a nominal dataset and as an injected-spoof dataset, and produced the following simulator-only results:
  - nominal dataset: `60/0/0` trusted/flagged/rejected, anomaly FPR `0.000`, rejected FPR `0.000`
  - spoofed replay dataset: `0/0/60` trusted/flagged/rejected, anomaly TPR/FPR `1.000/0.000`, rejected TPR/FPR `1.000/0.000`
  - nominal evaluation latency mean/p95/max: `333.17 / 377.31 / 935.51 us`
  - spoofed evaluation latency mean/p95/max: `312.32 / 334.31 / 382.50 us`

Those PX4 numbers are narrow and should be read narrowly: they come from PX4 SIH telemetry captured in WSL2 and a synthetic GPS spoof injected into the captured dataset by this repository's own benchmark tooling. They are not a live adversarial spoofing test and not a claim about general real-world performance.

- `cargo run --example run_texbat_harness` was run on `2026-05-10` against the downloaded processed TEXBAT `cleanStatic`, `ds2`, `ds3`, and `ds7` `navsol.mat` files. The harness uses the timing offsets and spoof-onset timings published in the 2016 TEXBAT analysis paper to align the clean and spoofed traces, calibrates constant pre-spoof receiver bias, and then replays the aligned solutions through the monitor with an added clock-bias residual. The observed results were:
  - `cleanStatic-baseline`: `2115/0/0` trusted/flagged/rejected, anomaly FPR `0.000`, rejected FPR `0.000`, latency mean/p95/max `191.77 / 248.90 / 648.20 us`, calibrated clean-alignment offset `0.000000 s`
  - `ds2`: `513/21/1566` trusted/flagged/rejected, anomaly TPR/FPR `0.988/0.103`, rejected TPR/FPR `0.978/0.093`, latency mean/p95/max `179.26 / 185.00 / 305.20 us`, calibrated clean-alignment offset `-2.003082 s`
  - `ds3`: `2030/16/50` trusted/flagged/rejected, anomaly TPR/FPR `0.000/0.110`, rejected TPR/FPR `0.000/0.083`, latency mean/p95/max `178.04 / 186.10 / 248.30 us`, calibrated clean-alignment offset `-1.096679 s`
  - `ds7`: `1106/77/992` trusted/flagged/rejected, anomaly TPR/FPR `0.664/0.000`, rejected TPR/FPR `0.616/0.000`, latency mean/p95/max `180.42 / 197.50 / 329.20 us`, calibrated clean-alignment offset `-0.050000 s`

The TEXBAT numbers are also narrow and should be read narrowly: they come from the UT processed `navsol.mat` products, not the 40+ GB raw IF captures, and this repository does not have paired IMU data for TEXBAT. The harness therefore uses the clean TEXBAT navigation solution as a reference trajectory proxy instead of replaying a true IMU-driven dead-reckoning path. That makes the TEXBAT results useful as an external sanity check, but not a full end-to-end claim for the live MAVLink product path.

The automatic clean-alignment calibration slightly improves scenario replay consistency, but it does not change the main qualitative outcome: this monitor performs strongly on `ds2`, partially on `ds7`, and still poorly on `ds3` when limited to processed `navsol.mat` trajectories.

## How To Run

```powershell
cargo test --lib
cargo run --example gps_spoof
cargo run --example run_validation
cargo run --example px4_sitl_live -- --connection udpout:127.0.0.1:18570
cargo run --example run_monitor_benchmark -- artifacts/px4_monitor_dataset.csv artifacts/px4_monitor_dataset_spoofed.csv
cargo run --example run_texbat_harness
```

`run_validation` replays the included `examples/synthetic_validation.csv` file and prints TPR/FPR-style summary metrics.

For the live PX4 path that has been verified here, run it from WSL2 so PX4 and the Rust client share the same localhost network stack:

```bash
bash scripts/wsl_inline_live.sh
bash scripts/wsl_px4_benchmark.sh 60
bash scripts/download_texbat_processed.sh
```

There is still no raw-TEXBAT IF processing path, no paired-IMU TEXBAT replay, no live adversarial spoofed PX4 mission, and no hardware validation. The strongest exercised paths today are the library test suite, the synthetic spoofing example, the offline validation example, the UDP/MAVLink loopback test, the WSL-local PX4 SIH nominal run, the WSL-local PX4 SIH capture/replay benchmark, and the processed-TEXBAT replay harness.

## Honest Status

This repository should be understood as an early-stage systems prototype:

- stronger than a toy class assignment in architecture and engineering discipline
- not yet a validated defense-grade detector
- not yet novel research by itself
- now verified on a narrow PX4 SIH simulator capture/replay benchmark with clean nominal behavior and full rejection of one injected spoof profile
- now also verified on a narrow processed-TEXBAT replay path, where it performs strongly on `ds2`, partially on `ds7`, and poorly on `ds3`
- potentially useful as a foundation for a real PX4/TEXBAT validation effort
