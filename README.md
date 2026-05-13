# RTVLAS

`rtvlas` is a Rust prototype for GPS spoofing detection in autonomy telemetry.

It compares an IMU-driven predicted state against GPS-reported position and velocity, assigns `Trusted` / `Flagged` / `Rejected` verdicts, and emits signed evidence records. The repository includes simulator-facing MAVLink ingestion, an orchestrator, replay harnesses, and measured results from a narrow set of exercised paths.

> This repository is an early-stage technical artifact. It is not a deployment claim, not a hardware-qualified system, and not a validated field-performance claim.

## Objective

The current repository is aimed at one narrow use case:

- detect GPS inconsistency in small-UAS-style telemetry
- keep the core monitor in Rust
- support both live MAVLink evaluation and offline replay
- produce signed evidence for each verdict

## Current Scope

Implemented today:

- IMU-driven ESKF-style propagation in local NED coordinates
- GPS innovation residuals with Mahalanobis distance and EWMA risk accumulation
- optional immediate trigger gates for large single-epoch GPS residuals
- auxiliary barometer altitude checks and opt-in heading consistency checks
- MAVLink UDP ingestion for `HIGHRES_IMU`, `GPS_RAW_INT`, and `GLOBAL_POSITION_INT`
- signed evidence packets using SHA-256 and Ed25519
- a process -> sign -> purge orchestrator
- replay tooling for CSV traces, PX4 SIH captures, and processed TEXBAT navigation solutions

Not implemented today:

- raw IF TEXBAT replay
- paired IMU + TEXBAT replay
- GPS measurement fusion back into the filter
- hardware secure elements or flight-controller deployment
- RF-level spoofing tests
- hardware-flight validation

## Status Summary

The repository is in the "prototype with measured simulator and processed-data results" stage.

- The Rust crate builds cleanly.
- The core crate still checks with `--no-default-features`.
- The library test suite currently passes `32/32`.
- PX4 SIH paths, a live MAVLink spoof proxy, and a processed TEXBAT harness have been exercised locally.

## Benchmark Snapshot

The table below is the shortest honest summary of what has actually been run.

| Evaluation path | Input | Result | What it means |
| --- | --- | --- | --- |
| PX4 SIH replay, nominal | 60 captured synchronized samples | anomaly FPR `0.000`, rejected FPR `0.000` | clean behavior on one narrow simulator capture |
| PX4 SIH replay, injected spoof | same capture with software-injected GPS offset | anomaly TPR `1.000`, rejected TPR `1.000` | full rejection on one replayed spoof profile |
| PX4 SIH adversarial replay sweep, structured export | 144 spoof profiles per captured mission dataset, exported as JSON/CSV | hover/forward/turn/climb still contain zero-rejection slow-ramp cases | the repository now maps evasion floors systematically instead of relying only on a few hand-picked ramps |
| PX4 SIH multi-mission nominal runs | hover, forward, turn, climb offboard profiles in SIH | all four measured regimes produced `0.000` anomaly FPR | the previous turn false-positive blocker was removed in this SIH path without loosening rejection thresholds |
| PX4 SIH live MAVLink spoof proxy, abrupt offset | live PX4 SIH stream, spoof onset at `1.5 s` inside the proxy after a `1 s` startup delay | `13/2/15` trusted/flagged/rejected | the state machine surfaced two `Flagged` verdicts before confirming `Rejected` |
| PX4 SIH live MAVLink spoof proxy, gradual carry-off | live PX4 SIH stream, `30 m` north ramp over `2.5 s` after the same onset timing | `25/6/14` trusted/flagged/rejected | the measured run stayed clean before spoof onset, then showed a visible `Trusted -> Flagged -> Rejected` progression |
| PX4 SIH live MAVLink spoof proxy, calibrated gradual sweep (opt-in) | live PX4 SIH stream, same proxy path with `--calibrate-live` and a conservative `1.0 m` sigma floor | all five tested ramp durations from `30 m / 2.5 s` through `30 m / 40 s` reached `Rejected` | in the measured software-MITM path, the opt-in calibrated mode lowered the live detection floor while preserving `60/0/0` on one clean nominal live run |
| TEXBAT `ds2` processed replay | UT processed `navsol.mat` | anomaly TPR/FPR `0.978/0.034` | strong result with lower clean false positives than the earlier fixed-noise proxy |
| TEXBAT `ds3` processed replay | UT processed `navsol.mat` | anomaly TPR/FPR `0.953/0.032` | improved gradual-drift sensitivity after calibrating horizontal residual CUSUM from the clean segment |
| TEXBAT `ds7` processed replay | UT processed `navsol.mat` | anomaly TPR/FPR `0.999/0.000` | near-complete detection on this processed-dataset scenario after the same calibration |

These are narrow results. They should not be generalized beyond the exact simulator and processed-data paths described in this repository.

## Quantitative Results

### Current Change Acceptance Summary

| Change | Acceptance criterion | Measured result | Decision |
| --- | --- | --- | --- |
| turn-regime false-positive fix | turn nominal anomaly FPR below `0.10`; hover/forward/climb stay `0.000` | hover/forward/turn/climb all measured `0.000` anomaly FPR | kept |
| velocity residual persistence | reduce zero-rejection sweep cases from the stated hover/forward/climb baseline `56/54/56` without regressing nominal FPR or TEXBAT | hover `47`, forward `53`, climb `50`; TEXBAT remained `ds2 0.978/0.034`, `ds3 0.953/0.032`, `ds7 0.999/0.000` | kept |
| flag-early / confirm-to-reject state machine | abrupt live spoof shows warning before rejection; clean nominal runs keep `0` flags | abrupt live run `13/2/15`, first flag verdict `#14`, first rejection verdict `#16`; nominal replay `60/0/0` | kept |

The implementation note is important: the turn false-positive fix was not shipped as a broad maneuver-aware gating claim. The measured culprit was uncalibrated auxiliary/warning behavior in the PX4 SIH path. Heading observations are now opt-in for that path, while persistence warning flags are opt-in for live operator output.

### PX4 SIH Capture / Replay

Observed on `2026-05-10` using `scripts/wsl_px4_benchmark.sh 60`:

| Dataset | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| nominal replay | `60 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `286.77 us` | `332.60 us` | `2441.92 us` |
| spoofed replay | `0 / 0 / 60` | `1.000` | `0.000` | `1.000` | `0.000` | `253.53 us` | `340.80 us` | `421.40 us` |

Method notes:

- source telemetry came from PX4 SIH in WSL2
- the spoof was injected by this repository's own replay tooling
- this is a simulator replay benchmark, not a live adversarial RF spoof test

### PX4 SIH Multi-Mission Nominal + Adversarial Replay Sweep

Observed on `2026-05-13` using `scripts/wsl_px4_multi_mission_benchmark.sh 120` and per-mission `artifacts/sweeps/px4_*_adversarial_sweep.{csv,json}` exports:

| Mission | GPS path summary | Nominal verdicts | Nominal anomaly FPR | Standard replayed spoof TPR / rejected TPR | Worst-case rejected TPR in 144-case sweep | Zero-rejection sweep cases |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `hover` | x `[-0.26, 0.13]`, y `[-0.36, 0.23]`, z `[-6.04, 0.27]`, max speed `2.59 m/s` | `120 / 0 / 0` | `0.000` | `0.951 / 0.676` | `0.000` | `47 / 144` |
| `forward` | x `[-0.02, 30.94]`, y `[-0.35, 0.27]`, z `[-6.17, 0.15]`, max speed `4.15 m/s` | `120 / 0 / 0` | `0.000` | `0.660 / 0.540` | `0.000` | `53 / 144` |
| `turn` | x `[-13.00, 11.94]`, y `[-0.06, 24.66]`, z `[-6.10, 0.13]`, max speed `4.79 m/s` | `120 / 0 / 0` | `0.000` | `0.802 / 0.475` | `0.000` | `46 / 144` |
| `climb` | x `[-0.02, 10.38]`, y `[-0.30, 0.26]`, z `[-19.97, 0.15]`, max speed `3.09 m/s` | `120 / 0 / 0` | `0.000` | `0.554 / 0.455` | `0.000` | `50 / 144` |

Method notes:

- the capture path now supports offboard mission-driving directly in `examples/capture_monitor_dataset.rs`
- the sweep export is machine-readable: one CSV and one JSON report per mission
- the current detector is clean on the measured hover, forward, turn, and climb nominal profiles
- the turn-regime acceptance target was anomaly FPR below `0.10`; the current measured result is `0.000` on this SIH profile
- the actual turn fix was narrower than the original maneuver-gating hypothesis: heading observations remain implemented but are not enabled by default on the PX4 SIH path, and persistence warning flags are opt-in for live operator output
- the worst evasion cases remain gradual slow-ramp carry-off profiles; velocity persistence reduced zero-rejection cases, but did not eliminate them

### PX4 SIH Live MAVLink Spoof Proxy

Observed on `2026-05-12` using `scripts/wsl_px4_live_spoof.sh`:

| Configuration | Value |
| --- | --- |
| proxy startup delay before spoof begins | `1.0 s` |
| proxy spoof onset after proxy start | `1.5 s` |
| injected position offset | `+90 m north, -50 m east, +8 m down` |
| injected velocity offset | `+10, -5, +1 m/s` in NED |
| total packets processed | `339` |
| IMU packets | `309` |
| GPS packets | `30` |
| verdicts | `13 trusted / 2 flagged / 15 rejected` |
| first flagged verdict | verdict `#14` |
| first rejection | verdict `#16` |
| evidence output | `artifacts/wsl_px4_live_spoof_evidence.bin` |
| observed evidence size | `6090 bytes` |

Method notes:

- PX4 SIH ran locally inside WSL2
- `examples/px4_spoof_proxy.rs` acted as a MAVLink man-in-the-middle
- only `GLOBAL_POSITION_INT` was modified
- the earlier "first rejection at verdict `#14` / `1.5 s` lag" wording was misleading because verdict numbering started before the proxy began spoofing
- the current state machine intentionally emits `Flagged` before `Rejected`, so abrupt-spoof confirmation now takes two GPS verdicts after the first flag
- this is a live software-level MAVLink spoof path, not an RF-level spoof or receiver compromise

Observed on `2026-05-13` using `scripts/wsl_px4_gradual_spoof.sh`:

| Configuration | Value |
| --- | --- |
| proxy startup delay before spoof begins | `1.0 s` |
| proxy spoof onset after proxy start | `1.5 s` |
| injected position offset | north ramp from `0 m` to `30 m` over `2.5 s` |
| injected velocity offset | `0 m/s` |
| total packets processed | `494` |
| IMU packets | `449` |
| GPS packets | `45` |
| verdicts | `25 trusted / 6 flagged / 14 rejected` |
| first clearly spoof-affected GPS packet in the measured run | verdict `#16` |
| first flagged verdict | verdict `#26` |
| first rejection | verdict `#32` |
| evidence output | `artifacts/wsl_px4_gradual_spoof_evidence.bin` |

Observed on `2026-05-13` using the same live proxy path with opt-in live warm-up calibration:

| Configuration | Value |
| --- | --- |
| detector mode | `--calibrate-live --live-warmup-verdicts 12 --live-calibration-min-sigma-m 1.0 --live-calibration-min-slack-sigma 0.2 --live-calibration-min-threshold 3.0` |
| fixed-threshold default changed? | no; this remains opt-in |
| live nominal check | `60 / 0 / 0` trusted / flagged / rejected |
| replay nominal check | `60 / 0 / 0` trusted / flagged / rejected |

Measured calibrated gradual sweep:

| Ramp profile | Approximate ramp rate | Trusted / Flagged / Rejected | First rejection | Verdicts from onset to rejection |
| --- | ---: | --- | --- | ---: |
| `30 m / 2.5 s` | `~12 m/s` | `15 / 0 / 105` | verdict `#16` | `3` |
| `30 m / 5 s` | `~6 m/s` | `17 / 0 / 103` | verdict `#18` | `4` |
| `30 m / 10 s` | `~3 m/s` | `18 / 0 / 102` | verdict `#19` | `5` |
| `30 m / 20 s` | `~1.5 m/s` | `21 / 0 / 99` | verdict `#22` | `8` |
| `30 m / 40 s` | `~0.75 m/s` | `24 / 0 / 96` | verdict `#25` | `11` |

Method notes:

- the live warm-up path now floors horizontal innovation sigma at `1.0 m` and floors horizontal CUSUM slack / threshold at `0.2 / 3.0`
- this was added because unconstrained live calibration overfit the hover-noise segment and became too sensitive
- the calibrated mode improved the measured slow-ramp floor on this live software-MITM path, but it is still not an RF-layer or hardware validation result

### Processed TEXBAT Replay

Observed on `2026-05-12` using `cargo run --example run_texbat_harness` after downloading processed TEXBAT artifacts:

| Scenario | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `cleanStatic-baseline` | `2115 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `182.27 us` | `200.50 us` | `286.50 us` |
| `ds2` | `566 / 13 / 1521` | `0.978` | `0.034` | `0.975` | `0.020` | `178.57 us` | `186.50 us` | `249.80 us` |
| `ds3` | `651 / 4 / 1441` | `0.953` | `0.032` | `0.953` | `0.025` | `181.04 us` | `191.70 us` | `308.40 us` |
| `ds7` | `567 / 0 / 1608` | `0.999` | `0.000` | `0.999` | `0.000` | `180.42 us` | `189.10 us` | `339.80 us` |

Method notes:

- these results use UT processed `navsol.mat` products, not raw IF captures
- the harness uses the clean navigation solution as a reference trajectory proxy
- there is no paired IMU stream from TEXBAT in this repository
- the current harness now calibrates replay noise from the pre-spoof clean segment, including per-axis position spread and clock-bias spread
- `ds3` was originally weak because the processed attack looked more like a sustained moderate horizontal drift than a large one-shot jump
- the current full profile calibrates a horizontal residual CUSUM directly from the pre-spoof clean segment and runs it in parallel with the older clock-bias persistence
- on the current processed-TEXBAT path, that raises `ds3` anomaly TPR from `0.907` to `0.953` at the same measured anomaly FPR of `0.032`

## Interpretation

The current evidence supports these narrower statements:

- the monitor path works end to end on live PX4 SIH telemetry
- the current residual checks flagged the measured abrupt live MAVLink spoof first, then confirmed rejection two verdicts later
- the current gradual carry-off live profile stayed clean before spoof onset, then reached rejection within `15` verdicts of spoof onset without introducing false positives on the nominal replay benchmark
- the new opt-in live calibration mode lowered the measured live carry-off floor from the earlier `~3-6 m/s` band to at least the tested `~0.75 m/s` profile on the same software-MITM PX4 path while staying clean on one `60`-verdict live nominal run and one `60`-sample replay nominal run
- the current processed-TEXBAT harness performs strongly on `ds2` and `ds7`, and reaches a materially improved but still narrow result on `ds3`
- the new TEXBAT ablation runs show that the clock-bias path and persistence logic are carrying most of the detection burden on `ds3`
- the new structured PX4 replay sweep shows that the current detector still has broad slow-ramp evasion space even after the measured turn-regime false positives were removed

The current evidence does not support these broader statements:

- field-ready GPS spoofing performance
- robustness to RF-level spoofing
- robustness across arbitrary platforms or missions
- flight-qualified latency or resource claims

## Baseline Comparison

The repository now includes a direct ablation runner:

```powershell
cargo run --example run_texbat_ablation
```

Observed on `2026-05-12`:

| Scenario | Profile | Anomaly TPR | Anomaly FPR | What it shows |
| --- | --- | ---: | ---: | --- |
| `ds2` | `full` | `0.978` | `0.034` | current full monitor operating point |
| `ds2` | `no_horiz_cusum` | `0.978` | `0.034` | new horizontal CUSUM does not materially change this easier case |
| `ds2` | `single_epoch_gps_clock` | `0.979` | `0.033` | persistence matters less on this easier processed case |
| `ds2` | `single_epoch_gps_only` | `0.000` | `0.016` | GPS-only residuals do not carry the detection here |
| `ds3` | `full` | `0.953` | `0.032` | current best processed result on the hardest case here |
| `ds3` | `no_horiz_cusum` | `0.749` | `0.032` | isolating the new horizontal CUSUM shows its direct contribution |
| `ds3` | `no_persistence` | `0.000` | `0.032` | removing persistence collapses detection on `ds3` |
| `ds3` | `single_epoch_gps_clock` | `0.000` | `0.030` | single-epoch clock checks alone are also insufficient on `ds3` |
| `ds7` | `full` | `0.999` | `0.000` | current best processed result on this scenario |
| `ds7` | `no_horiz_cusum` | `0.705` | `0.000` | the new horizontal CUSUM is carrying most of the gain here |
| `ds7` | `no_persistence` | `0.662` | `0.000` | persistence still helps beyond one-shot checks |
| `ds7` | `single_epoch_gps_only` | `0.000` | `0.000` | the GPS-only baseline again fails completely here |

This does not prove the architecture is globally optimal, but it does answer one basic question: the harder processed-replay detections are coming from sequential logic, not from plain one-shot GPS residual thresholds. On `ds3`, single-epoch checks still fail completely, while the full persistence path reaches `0.953` anomaly TPR.

## Evidence Verification

The repository now includes a standalone verifier for the framed `FileSink` output:

```powershell
cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin
```

Observed on `2026-05-13` against the live PX4 spoof evidence file:

- packets verified: `30`
- trusted verdicts: `13`
- flagged/rejected verdicts: `17`

This verifies the signed evidence stream outside the main orchestrator loop using a separate program.

## Verification

Core checks used locally:

```powershell
cargo fmt --all
cargo check --no-default-features
cargo check --all-targets
cargo test --lib
cargo check --examples
cargo run --example run_adversarial_sweep -- artifacts/px4_hover_dataset.csv --dataset-label hover --output-dir artifacts/sweeps
cargo run --example run_texbat_harness
cargo run --example run_texbat_ablation
cargo run --example run_texbat_baselines
cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin
```

WSL2 PX4 paths used locally:

```bash
bash scripts/wsl_inline_sniff.sh --connection udpout:127.0.0.1:18570 --event-limit 500 --gps-limit 1 --suppress-imu
bash scripts/wsl_inline_live.sh
bash scripts/wsl_px4_benchmark.sh 60
bash scripts/wsl_px4_multi_mission_benchmark.sh 120
bash scripts/wsl_px4_live_spoof.sh
bash scripts/wsl_px4_gradual_spoof.sh
```

Processed TEXBAT path used locally:

```powershell
.\scripts\download_texbat_processed.ps1
cargo run --example run_texbat_harness
cargo run --example run_texbat_ablation
```

More detail:

- [docs/verification.md](docs/verification.md)
- [docs/benchmark-summary.md](docs/benchmark-summary.md)

## Repository Structure

- `src/`
  - crate code
- `examples/`
  - smoke tests, live utilities, and replay drivers
- `scripts/`
  - verified shell and PowerShell helpers
- `docs/`
  - benchmark and verification notes

Useful maps:

- [docs/repository-layout.md](docs/repository-layout.md)
- [examples/README.md](examples/README.md)
- [scripts/README.md](scripts/README.md)
- [CONTRIBUTING.md](CONTRIBUTING.md)

## Next Technical Gaps

The most important remaining work is external validation, not more framing:

1. raw IF TEXBAT processing
2. paired IMU + GNSS replay
3. stronger spoof scenario coverage
4. hardware or flight-controller deployment path
5. hardware or field validation

## Startup Coverage

The library tests now explicitly cover:

- first nonzero IMU timestamp bootstrap
- out-of-order IMU rejection
- GPS arriving before the first recorded IMU state in the orchestrator

## License

This repository is dual-licensed under MIT or Apache-2.0, at your option.

- [LICENSE-MIT](LICENSE-MIT)
- [LICENSE-APACHE](LICENSE-APACHE)
