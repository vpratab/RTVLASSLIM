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
- auxiliary barometer altitude and heading consistency checks
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
- The library test suite currently passes `21/21`.
- PX4 SIH paths, a live MAVLink spoof proxy, and a processed TEXBAT harness have been exercised locally.

## Benchmark Snapshot

The table below is the shortest honest summary of what has actually been run.

| Evaluation path | Input | Result | What it means |
| --- | --- | --- | --- |
| PX4 SIH replay, nominal | 60 captured synchronized samples | anomaly FPR `0.000`, rejected FPR `0.000` | clean behavior on one narrow simulator capture |
| PX4 SIH replay, injected spoof | same capture with software-injected GPS offset | anomaly TPR `1.000`, rejected TPR `1.000` | full rejection on one replayed spoof profile |
| PX4 SIH live MAVLink spoof proxy | live PX4 SIH stream, spoof onset at `1.5 s` | `13/0/17` trusted/flagged/rejected | verdicts stayed trusted before spoof onset and then flipped to sustained rejection |
| TEXBAT `ds2` processed replay | UT processed `navsol.mat` | anomaly TPR/FPR `0.978/0.034` | strong result with lower clean false positives than the earlier fixed-noise proxy |
| TEXBAT `ds3` processed replay | UT processed `navsol.mat` | anomaly TPR/FPR `0.749/0.032` | lower clean false positives, with reduced sensitivity, on the hardest processed scenario here |
| TEXBAT `ds7` processed replay | UT processed `navsol.mat` | anomaly TPR/FPR `0.705/0.000` | partial detection on a harder processed-dataset scenario |

These are narrow results. They should not be generalized beyond the exact simulator and processed-data paths described in this repository.

## Quantitative Results

### PX4 SIH Capture / Replay

Observed on `2026-05-10` using `scripts/wsl_px4_benchmark.sh 60`:

| Dataset | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| nominal replay | `60 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `333.17 us` | `377.31 us` | `935.51 us` |
| spoofed replay | `0 / 0 / 60` | `1.000` | `0.000` | `1.000` | `0.000` | `312.32 us` | `334.31 us` | `382.50 us` |

Method notes:

- source telemetry came from PX4 SIH in WSL2
- the spoof was injected by this repository's own replay tooling
- this is a simulator replay benchmark, not a live adversarial RF spoof test

### PX4 SIH Live MAVLink Spoof Proxy

Observed on `2026-05-11` using `scripts/wsl_px4_live_spoof.sh`:

| Configuration | Value |
| --- | --- |
| spoof onset | `1.5 s` |
| injected position offset | `+90 m north, -50 m east, +8 m down` |
| injected velocity offset | `+10, -5, +1 m/s` in NED |
| total packets processed | `341` |
| IMU packets | `311` |
| GPS packets | `30` |
| verdicts | `13 trusted / 0 flagged / 17 rejected` |
| first rejection | verdict `#14` |
| evidence output | `artifacts/wsl_px4_live_spoof_evidence.bin` |
| observed evidence size | `6090 bytes` |

Method notes:

- PX4 SIH ran locally inside WSL2
- `examples/px4_spoof_proxy.rs` acted as a MAVLink man-in-the-middle
- only `GLOBAL_POSITION_INT` was modified
- this is a live software-level MAVLink spoof path, not an RF-level spoof or receiver compromise

### Processed TEXBAT Replay

Observed on `2026-05-11` using `cargo run --example run_texbat_harness` after downloading processed TEXBAT artifacts:

| Scenario | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `cleanStatic-baseline` | `2115 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `176.50 us` | `183.10 us` | `295.10 us` |
| `ds2` | `566 / 13 / 1521` | `0.978` | `0.034` | `0.975` | `0.020` | `178.35 us` | `188.50 us` | `286.80 us` |
| `ds3` | `956 / 4 / 1136` | `0.749` | `0.032` | `0.749` | `0.025` | `182.10 us` | `222.30 us` | `622.00 us` |
| `ds7` | `1040 / 0 / 1135` | `0.705` | `0.000` | `0.705` | `0.000` | `177.97 us` | `186.00 us` | `422.90 us` |

Method notes:

- these results use UT processed `navsol.mat` products, not raw IF captures
- the harness uses the clean navigation solution as a reference trajectory proxy
- there is no paired IMU stream from TEXBAT in this repository
- the current harness now calibrates replay noise from the pre-spoof clean segment, including per-axis position spread and clock-bias spread
- `ds3` now has materially lower pre-spoof false positives than the earlier fixed-noise proxy, but that reduction comes with lower spoof sensitivity than the earlier over-tight setting

## Interpretation

The current evidence supports these narrower statements:

- the monitor path works end to end on live PX4 SIH telemetry
- the current residual checks can reject at least one live software-injected MAVLink spoof profile after onset
- the current processed-TEXBAT harness performs strongly on `ds2`, reduces clean false positives on `ds3`, and remains partial on `ds7`

The current evidence does not support these broader statements:

- field-ready GPS spoofing performance
- robustness to RF-level spoofing
- robustness across arbitrary platforms or missions
- flight-qualified latency or resource claims

## Verification

Core checks used locally:

```powershell
cargo fmt --all
cargo check --no-default-features --lib
cargo check --all-targets
cargo test --lib
cargo check --examples
```

WSL2 PX4 paths used locally:

```bash
bash scripts/wsl_inline_sniff.sh --connection udpout:127.0.0.1:18570 --event-limit 500 --gps-limit 1 --suppress-imu
bash scripts/wsl_inline_live.sh
bash scripts/wsl_px4_benchmark.sh 60
bash scripts/wsl_px4_live_spoof.sh
```

Processed TEXBAT path used locally:

```powershell
.\scripts\download_texbat_processed.ps1
cargo run --example run_texbat_harness
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

## License

This repository is dual-licensed under MIT or Apache-2.0, at your option.

- [LICENSE-MIT](LICENSE-MIT)
- [LICENSE-APACHE](LICENSE-APACHE)
