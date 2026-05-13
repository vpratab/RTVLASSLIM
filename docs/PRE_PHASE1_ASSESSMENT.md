# Pre-Phase 1 Assessment

This document translates the current RTVLAS-Slim artifact into a pre-Phase 1 readiness assessment. It is intentionally conservative: every measured claim is tied to an existing command or artifact, and unmeasured claims are listed as risks or next-step work.

## Executive Summary

RTVLAS-Slim is a Rust prototype for GPS spoofing detection in small-UAS telemetry. It monitors processed MAVLink navigation solutions, compares GPS position and velocity against an IMU-propagated ESKF prediction, applies statistical residual and persistence checks, and writes signed evidence packets for later verification.

The prototype has encouraging simulator and processed-dataset results, especially on processed TEXBAT `ds2`, `ds3`, and `ds7`, but it remains low maturity. It has not been tested with outdoor hardware, raw GPS intermediate-frequency data, paired TEXBAT IMU, real RF spoofing, real jamming, or flight-controller deployment. The best current readiness description is TRL 3, with a narrow argument for TRL 4 only inside controlled replay and PX4 SIH bench conditions.

## Corrections To Avoid Overclaiming

| Assessment topic | Precise repository status |
| --- | --- |
| RF-layer coverage | Out of scope. RTVLAS-Slim sees processed navigation output after the receiver. |
| `GPS_RAW_INT` | MAVLink GPS status/accuracy/position metadata, not raw IF or raw pseudorange processing. |
| GPS fusion | The current monitor compares GPS against the propagated ESKF state; it does not fuse accepted GPS back into the ESKF state as a flight-control navigation filter. |
| Target hardware latency | Not measured. Host/SITL benchmark latency is measured, but CPU load on NuttX, PX4 hardware, or representative ARM boards is not. |
| Evidence audit root | Implemented as a verifier-computed SHA-256 chain root over signed framed packets; not a Merkle tree or external timestamp anchor. |
| Platform support | Code targets common MAVLink messages; measured platform path is PX4 SIH only. |
| Production readiness | Not production ready, not certified, and not field validated. |

## Current Measured Evidence

| Evidence path | Command | Current result |
| --- | --- | --- |
| Processed TEXBAT replay | `cargo run --example run_texbat_harness` | `ds2 0.978/0.034`, `ds3 0.953/0.032`, `ds7 0.999/0.000` anomaly TPR/FPR |
| TEXBAT ablation | `cargo run --example run_texbat_ablation` | Persistence paths materially improve `ds3` and `ds7` |
| Simple baselines | `cargo run --example run_texbat_baselines` | Full detector outperforms naive distance and innovation-only baselines on measured scenarios |
| PX4 SIH replay | `bash scripts/wsl_px4_benchmark.sh 60` | nominal `60/0/0`, spoofed `0/0/60` |
| PX4 SIH multi-mission | `bash scripts/wsl_px4_multi_mission_benchmark.sh 120` | hover/forward/turn/climb nominal anomaly FPR `0.000` |
| Live software MAVLink abrupt spoof | `bash scripts/wsl_px4_live_spoof.sh` | `13/2/15` trusted/flagged/rejected |
| Live software MAVLink gradual spoof | `bash scripts/wsl_px4_gradual_spoof.sh` | `25/6/14` trusted/flagged/rejected |
| Evidence verification | `cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin` | 30 packets verified, 13 trusted, 17 flagged/rejected, chain root `aee3dce6be23e5ed8ff0674decc34769cab1579e06db539ac265257eb341db36` |
| Host monitor profiling | `cargo run --example profile_monitor_dataset -- artifacts/px4_monitor_dataset.csv --iterations 50` | 3000 monitor evaluations, `3844.8 eval/s`, mean/p95/max `259.27/271.32/397.80 us` on the local host |
| Realistic spoof-profile suite | `cargo run --example run_realistic_spoof_suite -- artifacts/px4_hover_dataset.csv --dataset-label px4_hover` | abrupt takeover `1.000`, SDR-style 30 m / 10 s `0.894-0.914`, frozen GPS `0.705-0.788`, generated `ds7` `0.692-0.762` rejected TPR |

## Technical Maturity

| Dimension | Current status | Readiness interpretation |
| --- | --- | --- |
| Algorithm implementation | ESKF propagation, Mahalanobis innovation monitor, EWMA, clock/position/velocity CUSUM, live flag-confirm state machine | implemented prototype |
| Replay validation | processed TEXBAT and CSV/PX4 replay paths | controlled benchmark evidence |
| Simulator validation | PX4 SIH hover/forward/turn/climb and live software spoof proxy | controlled bench evidence |
| Hardware validation | none in repository | not TRL 5 |
| RF validation | none in repository | not RF anti-spoofing |
| Embedded deployment | `no-default-features` check passes, but target hardware CPU/memory not measured | embedded feasibility only partially supported |
| Certification posture | no DO-178C/DO-254 evidence package | research prototype only |

Best current TRL statement:

```text
RTVLAS-Slim is TRL 3, with TRL 4-style evidence for the narrow simulator/replay paths documented in this repository.
```

## Compute And Latency

Measured host/SITL latencies are documented in [benchmark-summary.md](benchmark-summary.md). The processed TEXBAT harness currently reports mean evaluation latency around `180 us` on the local host run, and PX4 replay reports sub-millisecond mean evaluation latency with occasional host-load outliers.

That does not establish target-hardware performance. Before claiming embedded real-time readiness, the project needs:

- CPU load on representative flight hardware.
- Memory use under `no_std` or constrained `std` deployment.
- Worst-case execution time at expected GPS update rates.
- Signing overhead with production key storage.
- Scheduling behavior under concurrent autopilot workload.

Until those tests exist, target-hardware latency should be stated as unmeasured.

## Comparative Positioning

RTVLAS-Slim should be compared against method categories, not uncited external headline numbers:

| Category | Strength | Weakness | RTVLAS-Slim relationship |
| --- | --- | --- | --- |
| RAIM / receiver integrity checks | mature, lightweight, receiver-side | not spoof-specific and often limited against coordinated spoofing | RTVLAS-Slim is heavier but uses inertial consistency and persistence |
| Raw RF / IF spoof detectors | can see receiver-layer attacks before navigation output | receiver-specific, data-heavy, often needs RF access | RTVLAS-Slim does not cover this layer |
| ML / autoencoder approaches | can learn subtle feature patterns | training data, generalization, and explainability risks | RTVLAS-Slim is classical, explainable, and lower complexity |
| INS/GNSS residual monitors | physically interpretable and low-compute | threshold tuning and maneuver sensitivity | RTVLAS-Slim is in this family, with added CUSUM persistence and signed evidence |

Any table with external paper TPR/FPR numbers should include exact citations and reproduction conditions before being published in a proposal.

## Main Risks

| Risk | Probability | Impact | Mitigation |
| --- | --- | --- | --- |
| No real flight validation | high | critical | collect outdoor GNSS/IMU logs, then fly controlled hardware tests |
| No raw IF / RF-layer coverage | high | high | pair RTVLAS-Slim with receiver/RF-layer monitoring; do not sell it as RF detection |
| Target hardware performance unknown | medium | high | benchmark on representative ARM/PX4 companion hardware; current host profiling is not a substitute |
| Threshold transfer across vehicles unknown | medium | high | calibrate per mission class; add adaptive threshold studies |
| Slow/subtle generated profiles remain partial | medium | high | keep expanding adversarial sweep, add real external datasets, and validate stale-solution detection on real receiver logs before claiming field coverage |
| False alarms under real vibration/multipath unknown | medium | high | collect outdoor nominal logs across environments |
| Signing key storage is mock/software-backed | medium | medium | define TPM/HSM/secure-element path before field demo |
| Evidence chain root is not externally anchored | medium | medium | record chain roots outside the evidence file and add timestamping or Merkle anchoring if audit integrity becomes a primary claim |
| Certification evidence absent | high | medium | keep safety-critical claims out of Phase 1; document test methodology now |

## Pre-Phase 1 Work Plan

The highest-value next experiments are:

1. Target-hardware profiling: run the detector and evidence signing on a representative ARM/Linux companion computer or PX4-class hardware; report CPU, memory, mean latency, p95 latency, and worst-case latency.
2. Outdoor nominal data: collect at least 30 minutes of GPS/IMU/barometer logs from a real receiver without spoofing; measure false positives.
3. Hardware-in-the-loop replay: replay measured telemetry through the MAVLink adapter at real rates and compare against CSV replay results.
4. Dynamic trajectory expansion: add aggressive turn, climb/descent, wind, vibration, and weak-GPS simulator profiles before claiming general flight-regime robustness.
5. Raw IF feasibility study: process raw TEXBAT or equivalent IF data through GNSS-SDR to understand what RTVLAS-Slim misses at the receiver layer.
6. Adversarial sweep expansion: include intermittent spoofing, stepped drift, low-rate carry-off below current detection floor, vertical axes, diagonal axes, and position-plus-velocity profiles.
7. Evidence hardening: archive raw telemetry next to signed verdicts, externally anchor the chain root, and move signing keys into a hardware-backed provider.

## Phase 1 Framing

The honest Phase 1 claim is:

```text
RTVLAS-Slim has demonstrated a software-layer GPS spoofing monitor on processed navigation telemetry in PX4 SIH and processed TEXBAT replay. Phase 1 will validate transfer to representative hardware, collect real receiver data, characterize false positives under outdoor dynamics, and define how this software-layer monitor should be paired with receiver/RF-layer defenses.
```

The claim to avoid:

```text
RTVLAS-Slim is a field-ready universal GPS anti-spoofing system.
```

The second statement is not supported by the current artifact.
