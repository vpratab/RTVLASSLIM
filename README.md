# RTVLAS-Slim

RTVLAS-Slim is a Rust prototype for GPS spoofing detection in small-UAS telemetry. It operates downstream of the GPS receiver on processed MAVLink navigation messages, compares GPS-reported motion against an IMU-propagated state estimate, and emits `Trusted`, `Flagged`, or `Rejected` verdicts with signed evidence records.

GPS spoofing matters because small autonomous aircraft often trust processed GPS position and velocity even when an attacker is manipulating the navigation solution. RTVLAS-Slim fills a specific software-layer gap: it can be added to a PX4-style MAVLink telemetry path without replacing the GPS receiver, accessing raw intermediate-frequency samples, or modifying RF hardware.

> Current scope: this is a simulator and processed-dataset prototype. It is not field validated, not RF-layer detection, not a hardware-qualified flight system, and not a claim of generalized platform robustness.

## Readiness Snapshot

| Dimension | Current status |
| --- | --- |
| Best TRL description | TRL 3, with TRL 4-style evidence only for controlled replay and PX4 SIH paths |
| Measured validation | processed TEXBAT replay, PX4 SIH replay, PX4 SIH software MAVLink spoof proxy |
| Not measured | outdoor receiver logs, real flight, raw IF replay, RF spoofing, target flight hardware CPU/memory |
| Primary technical risk | processed-navigation monitoring cannot see RF-layer attacks that remain internally consistent through the receiver |
| New pre-hardware tooling | signed evidence chain-root verification, host profiling, extended adversarial sweeps, realistic spoof-profile suite, stale-GPS replay detector, passive outdoor nominal-report workflow |
| Next evidence needed | target-hardware profiling and outdoor GNSS/IMU nominal data |

See [docs/PRE_PHASE1_ASSESSMENT.md](docs/PRE_PHASE1_ASSESSMENT.md) for the risk table and recommended pre-Phase 1 work plan.

## Key Results

### Processed TEXBAT Replay

Command:

```powershell
cargo run --example run_texbat_harness
```

Measured on `2026-05-13` using processed TEXBAT `navsol.mat` artifacts under `artifacts/texbat`:

| Scenario | Attack shape represented here | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `cleanStatic-baseline` | clean static receiver | `2115 / 0 / 0` | n/a | `0.000` | n/a | `0.000` |
| `ds2` | abrupt carry-off | `566 / 13 / 1521` | `0.978` | `0.034` | `0.975` | `0.020` |
| `ds3` | gradual low-magnitude drift | `651 / 4 / 1441` | `0.953` | `0.032` | `0.953` | `0.025` |
| `ds7` | subtle time-push / phase-aligned case | `567 / 0 / 1608` | `0.999` | `0.000` | `0.999` | `0.000` |

These are processed navigation-solution replays, not raw RF or raw IF receiver tests. See [docs/THREAT_MODEL.md](docs/THREAT_MODEL.md) for the architectural boundary.

### PX4 SIH Multi-Mission Nominal FPR

Command:

```bash
bash scripts/wsl_px4_multi_mission_benchmark.sh 120
```

Measured on `2026-05-13` using PX4 Software-In-The-Loop with SIH dynamics:

| Mission | Nominal verdicts | Nominal anomaly FPR | Nominal rejected FPR | Standard injected-spoof anomaly / rejected TPR | Zero-rejection sweep cases |
| --- | ---: | ---: | ---: | ---: | ---: |
| `hover` | `120 / 0 / 0` | `0.000` | `0.000` | `0.961 / 0.961` | `0 / 144` |
| `forward` | `120 / 0 / 0` | `0.000` | `0.000` | `0.960 / 0.960` | `0 / 144` |
| `turn` | `120 / 0 / 0` | `0.000` | `0.000` | `0.960 / 0.960` | `0 / 144` |
| `climb` | `120 / 0 / 0` | `0.000` | `0.000` | `0.960 / 0.960` | `0 / 144` |

The previous turn-regime false-positive blocker was `0.717` anomaly FPR. The current measured SIH result is `0.000`, against an acceptance target of below `0.10`. This fix should not be generalized to hardware or high-dynamics flight until those paths are tested.

### Realistic Spoof-Profile Suite

Command:

```powershell
cargo run --example run_realistic_spoof_suite -- artifacts/px4_hover_dataset.csv --dataset-label px4_hover
```

Measured on `2026-05-13` across the four PX4 SIH mission datasets:

| Profile family | Representative profile | Anomaly TPR range | Rejected TPR range | Current interpretation |
| --- | --- | ---: | ---: | --- |
| abrupt takeover | `texbat_ds1_static_takeover` | `1.000` | `1.000` | caught immediately |
| overpowered time-push | `texbat_ds2_overpowered_time_push` | `0.894-0.905` | `0.875-0.876` | strong |
| slow matched-power carry-off | `texbat_ds3_matched_power_slow_carryoff` | `0.779-0.827` | `0.779-0.827` | partially caught, still a hard case |
| subtle phase-aligned time-push | `texbat_ds7_phase_aligned_time_push` | `0.692-0.762` | `0.692-0.762` | improved, but still weaker than processed TEXBAT |
| SDR-style UAV takeover | `uav_sdr_takeover_30m_10s` | `0.894-0.914` | `0.894-0.914` | strong |
| hold-last-fix / frozen GPS | `uav_freeze_or_hold_last_fix` | `0.705-0.788` | `0.705-0.788` | now partially caught in generated replay |
| wrong-turn cross-track spoof | `nav_wrong_turn_cross_track` | `0.904-0.905` | `0.904-0.905` | strong |
| along-track route overshoot | `nav_overshoot_along_track` | `0.933-0.943` | `0.933-0.943` | strong |
| intermittent carry-off | `intermittent_pulsed_carryoff` | `0.736-0.782` | `0.736-0.782` | partially caught |

This suite is generated from measured SIH mission logs. It is not a substitute for real RF or flight data, but it makes the pre-hardware adversarial coverage broader and more reproducible.

### Baseline Comparison

Command:

```powershell
cargo run --example run_texbat_baselines
```

Measured with a `5.0 m` naive distance threshold and a `3.0 sigma` innovation baseline:

| Scenario | RTVLAS full TPR/FPR | Naive distance TPR/FPR | Innovation `N_sigma` TPR/FPR |
| --- | ---: | ---: | ---: |
| `cleanStatic` | `0.000 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |
| `ds2` | `0.978 / 0.034` | `0.445 / 0.102` | `0.000 / 0.018` |
| `ds3` | `0.953 / 0.032` | `0.631 / 0.125` | `0.000 / 0.025` |
| `ds7` | `0.999 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |

The baseline result is the main evidence that the sequential detection logic is doing real work beyond a single residual threshold. See [docs/BASELINES.md](docs/BASELINES.md).

## Architecture

RTVLAS-Slim uses an IMU-driven ESKF prediction as the local physics reference, then evaluates GPS position and velocity claims against that reference. The detector combines Mahalanobis-normalized innovation scoring, EWMA risk accumulation, clock-bias persistence, horizontal position-residual CUSUM, horizontal velocity-residual CUSUM, stale-GPS persistence for held/frozen fixes, and an opt-in flag-then-confirm state machine for live operator output.

```mermaid
flowchart LR
    imu["HIGHRES_IMU"] --> eskf["ESKF predict"]
    gps["GPS_RAW_INT / GLOBAL_POSITION_INT"] --> sync["time alignment"]
    eskf --> innov["GPS innovation residual"]
    sync --> innov
    innov --> maha["Mahalanobis + EWMA"]
    innov --> cusum["clock / position / velocity / stale-fix persistence"]
    maha --> verdict["Trusted / Flagged / Rejected"]
    cusum --> verdict
    verdict --> evidence["SHA-256 + Ed25519 signed evidence"]
    evidence --> file["length-framed evidence stream"]
    file --> root["verifier-computed chain root"]
```

The current evidence stream is length-framed and each packet is individually signed. The verifier now computes a deterministic SHA-256 chain root over the signed packet sequence for external anchoring. This is a hash-chain audit root, not a Merkle tree.

## Reproduce

Core Rust checks:

```powershell
cargo fmt --all --check
cargo check --no-default-features
cargo check --all-targets
cargo clippy --all-targets --all-features -- -D warnings
cargo test --lib
```

Processed TEXBAT results:

```powershell
.\scripts\download_texbat_processed.ps1
cargo run --example run_texbat_harness
cargo run --example run_texbat_ablation
cargo run --example run_texbat_baselines
```

PX4 SIH replay and live-proxy paths:

```bash
bash scripts/wsl_px4_benchmark.sh 60
bash scripts/wsl_px4_multi_mission_benchmark.sh 120
bash scripts/wsl_px4_live_spoof.sh
bash scripts/wsl_px4_gradual_spoof.sh
```

Pre-hardware characterization utilities:

```powershell
cargo run --example profile_monitor_dataset -- artifacts/px4_monitor_dataset.csv --iterations 50
cargo run --example report_nominal_dataset -- artifacts/px4_monitor_dataset.csv --json-output artifacts/px4_monitor_nominal_report.json
cargo run --example run_adversarial_sweep -- artifacts/px4_hover_dataset.csv --dataset-label hover_extended --output-dir artifacts/sweeps --extended --onsets 2.0,4.0 --ramps 0.0,5.0,20.0,40.0
cargo run --example run_realistic_spoof_suite -- artifacts/px4_hover_dataset.csv --dataset-label px4_hover
```

Evidence verification:

```powershell
cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin
```

Expected measured output for the current evidence artifact:

```text
Evidence file: artifacts/wsl_px4_live_spoof_evidence.bin
  packets verified: 30
  trusted verdicts: 13
  flagged/rejected verdicts: 17
  first timestamp (ns): 3796000000
  last timestamp (ns): 6700000000
  evidence chain root: aee3dce6be23e5ed8ff0674decc34769cab1579e06db539ac265257eb341db36
```

For a step-by-step reproduction guide, see [docs/REPRODUCE.md](docs/REPRODUCE.md).

## Repository Map

| Path | Purpose |
| --- | --- |
| [src/ekf_core](src/ekf_core) | IMU propagation, ESKF state, covariance propagation |
| [src/statistical_monitor](src/statistical_monitor) | Mahalanobis scoring, EWMA, CUSUM persistence, trust verdicts |
| [src/telemetry_adapter](src/telemetry_adapter) | MAVLink ingestion, geodetic-to-NED conversion, GPS/IMU synchronization |
| [src/attestation](src/attestation) | evidence packet serialization, SHA-256 hashing, Ed25519 signing and verification |
| [src/benchmark](src/benchmark) | replay dataset runner and measurement summaries |
| [src/texbat_harness](src/texbat_harness) | processed TEXBAT replay harness and baselines |
| [examples](examples) | runnable benchmark, live, verification, and replay binaries |
| [scripts](scripts) | WSL2/PX4 helper scripts and dataset generation commands |
| [artifacts](artifacts) | measured datasets, sweep exports, logs, and evidence files |
| [docs/ALGORITHM.md](docs/ALGORITHM.md) | technical detector math and ablation interpretation |
| [docs/FORENSICS.md](docs/FORENSICS.md) | signed evidence format and verification chain |
| [docs/THREAT_MODEL.md](docs/THREAT_MODEL.md) | what the detector does and does not cover |
| [docs/BASELINES.md](docs/BASELINES.md) | baseline and ablation tables |
| [docs/REPRODUCE.md](docs/REPRODUCE.md) | full reproduction guide |
| [docs/SPOOF_DATASETS.md](docs/SPOOF_DATASETS.md) | public spoof datasets and current integration status |
| [docs/PRE_PHASE1_ASSESSMENT.md](docs/PRE_PHASE1_ASSESSMENT.md) | TRL, risks, and pre-Phase 1 work plan |
| [docs/benchmark-summary.md](docs/benchmark-summary.md) | compact measured-result record |
| [docs/verification.md](docs/verification.md) | additional verification notes |

## Known Limitations

- RTVLAS-Slim operates on processed navigation solutions, not raw RF or raw intermediate-frequency GPS samples.
- A receiver-level RF attack that remains internally consistent through the GPS receiver tracking loops is outside the current architecture.
- The repository does not contain paired TEXBAT IMU data, raw IF TEXBAT replay, outdoor receiver logs, hardware flight tests, or flight-controller deployment measurements.
- Current PX4 results are SIH simulator measurements over localhost/WSL2 paths.
- Current live-spoof results are software MAVLink man-in-the-middle tests, not RF spoofing tests.
- MAVLink compatibility is implemented around common PX4/ArduPilot-style messages, but the measured platform path is PX4 SIH.

## License

This repository is dual-licensed under MIT or Apache-2.0, at your option.

- [LICENSE-MIT](LICENSE-MIT)
- [LICENSE-APACHE](LICENSE-APACHE)
