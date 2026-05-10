# RTVLAS

`rtvlas` is a Rust prototype for GPS spoofing detection in autonomy telemetry.

It is not a finished product, not a validated benchmark result, and not a research breakthrough on its own. It is a structured prototype that combines:

- IMU-driven ESKF-style state propagation in a local NED frame.
- GPS innovation checking with Mahalanobis distance and EWMA risk accumulation.
- Barometer altitude and magnetometer-derived heading consistency checks.
- MAVLink ingestion for `HIGHRES_IMU`, `GPS_RAW_INT`, and `GLOBAL_POSITION_INT`.
- Signed evidence packets using SHA-256 and Ed25519.
- A process -> sign -> purge orchestrator that explicitly wipes raw MAVLink frame buffers after attestation.
- An offline CSV validation harness for replaying logged traces and summarizing detection outcomes.

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
  - MAVLink UDP listener for PX4-style telemetry on `127.0.0.1:14550`.
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

## What Is Not Implemented

- No PX4 SITL or hardware-in-the-loop run has been completed from this repository yet.
- No TEXBAT dataset replay or TEXBAT benchmark result exists yet.
- No real-world detection-rate, false-positive, or latency benchmark is claimed here.
- No hardware-backed secure element, HSM, TPM, enclave, or flight-controller integration is present.
- No GPS update is fused back into the filter state yet; GPS is monitored, not used as a measurement update.
- No production persistence or distributed-ledger sink exists; only local file/log sinks are provided.

## Verification Performed

The code in this repository has been locally verified with:

```powershell
cargo test --lib
cargo check --features telemetry,attestation --lib
```

At the time of writing, the crate passes 14 library tests covering:

- IMU propagation staying stable at rest.
- GPS innovation rejection on a large spoof-like offset.
- Combined barometer and heading anomaly rejection.
- Geodetic projection and state interpolation.
- Live UDP/MAVLink loopback ingestion into the telemetry adapter.
- Ed25519 evidence signing and tamper detection.
- An orchestrator integration path that emits a rejected verdict and persists signed evidence.
- Offline CSV validation report generation.

## How To Run

```powershell
cargo test --lib
cargo run --example gps_spoof
cargo run --example run_validation
```

`run_validation` replays the included `examples/synthetic_validation.csv` file and prints TPR/FPR-style summary metrics.

There is still no complete live PX4 SITL demo binary or TEXBAT benchmark result in this repo yet. The strongest exercised paths today are the library test suite, the synthetic spoofing example, the offline validation example, and the UDP/MAVLink loopback test.

## Honest Status

This repository should be understood as an early-stage systems prototype:

- stronger than a toy class assignment in architecture and engineering discipline
- not yet a validated defense-grade detector
- not yet novel research by itself
- potentially useful as a foundation for a real PX4/TEXBAT validation effort
