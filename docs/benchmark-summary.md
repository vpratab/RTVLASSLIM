# Benchmark Summary

This document is a compact record of the benchmark and replay paths that have actually been exercised in this repository.

## Purpose

The goal is to separate:

- what has been measured
- what has only been implemented
- what still requires external validation

## Evaluation Matrix

| Path | Input source | Attack model | Status |
| --- | --- | --- | --- |
| PX4 SIH nominal live run | live PX4 SIH MAVLink | none | exercised |
| PX4 SIH capture/replay benchmark | recorded PX4 SIH monitor dataset | software-injected GPS offset in replay | exercised |
| PX4 SIH multi-mission replay sweep | mission-specific PX4 SIH monitor datasets | systematic replay sweep over onset, ramp, direction, and position+velocity mode | exercised |
| PX4 SIH live spoof proxy | live PX4 SIH MAVLink | software MITM on `GLOBAL_POSITION_INT` | exercised |
| TEXBAT processed replay | processed `navsol.mat` solutions | scenario-dependent processed replay | exercised |
| raw TEXBAT IF replay | raw IF captures | receiver-level spoofing path | not implemented |
| paired IMU + TEXBAT replay | external paired sensor data | integrated inertial replay | not implemented |
| hardware flight test | live vehicle telemetry | real platform conditions | not implemented |
| host monitor profiling | recorded monitor dataset | no attack; compute characterization only | exercised |

## Measured Results

### Current Change Acceptance Summary

| Change | Acceptance criterion | Measured result | Decision |
| --- | --- | --- | --- |
| turn-regime false-positive fix | turn nominal anomaly FPR below `0.10`; hover/forward/climb stay `0.000` | hover/forward/turn/climb all measured `0.000` anomaly FPR | kept |
| velocity residual persistence | reduce zero-rejection sweep cases from the stated hover/forward/climb baseline `56/54/56` without regressing nominal FPR or TEXBAT | initial pass reduced gaps without nominal regression; later replay hardening now measures `0 / 144` zero-rejection cases on all four default SIH sweeps | kept |
| stale-GPS replay detector and spoof-onset fix | improve generated hold-last-fix and subtle time-push profiles without raising nominal SIH FPR | frozen GPS moved from `0.000` to `0.705-0.788` rejected TPR; generated `ds7` moved from `0.417-0.425` to `0.692-0.762`; nominal remains `0.000` FPR | kept |
| flag-early / confirm-to-reject state machine | abrupt live spoof shows warning before rejection; clean nominal runs keep `0` flags | abrupt live run `13/2/15`, first flag verdict `#14`, first rejection verdict `#16`; nominal replay `60/0/0` | kept |

The implementation note is important: the turn false-positive fix was not shipped as a broad maneuver-aware gating claim. The measured culprit was uncalibrated auxiliary/warning behavior in the PX4 SIH path. Heading observations are now opt-in for that path, while persistence warning flags are opt-in for live operator output.

### PX4 SIH Capture / Replay

| Dataset | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| nominal replay | `60 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `263.92 us` | `274.80 us` | `474.80 us` |
| spoofed replay | `0 / 0 / 60` | `1.000` | `0.000` | `1.000` | `0.000` | `264.80 us` | `287.60 us` | `305.20 us` |

### PX4 SIH Multi-Mission Nominal + Replay Sweep

Observed on `2026-05-13` using `scripts/wsl_px4_multi_mission_benchmark.sh 120`:

| Mission | GPS path summary | Nominal verdicts | Nominal anomaly FPR | Nominal rejected FPR | Standard replayed spoof anomaly / rejected TPR | Worst-case rejected TPR in 144-case sweep | Zero-rejection sweep cases |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `hover` | x `[-0.26, 0.13]`, y `[-0.36, 0.23]`, z `[-6.04, 0.27]`, max speed `2.59 m/s` | `120 / 0 / 0` | `0.000` | `0.000` | `0.961 / 0.961` | `0.758` | `0 / 144` |
| `forward` | x `[-0.02, 30.94]`, y `[-0.35, 0.27]`, z `[-6.17, 0.15]`, max speed `4.15 m/s` | `120 / 0 / 0` | `0.000` | `0.000` | `0.960 / 0.960` | `0.700` | `0 / 144` |
| `turn` | x `[-13.00, 11.94]`, y `[-0.06, 24.66]`, z `[-6.10, 0.13]`, max speed `4.79 m/s` | `120 / 0 / 0` | `0.000` | `0.000` | `0.960 / 0.960` | `0.725` | `0 / 144` |
| `climb` | x `[-0.02, 10.38]`, y `[-0.30, 0.26]`, z `[-19.97, 0.15]`, max speed `3.09 m/s` | `120 / 0 / 0` | `0.000` | `0.000` | `0.960 / 0.960` | `0.733` | `0 / 144` |

Structured exports now available per mission:

- `artifacts/sweeps/px4_hover_adversarial_sweep.csv`
- `artifacts/sweeps/px4_hover_adversarial_sweep.json`
- `artifacts/sweeps/px4_forward_adversarial_sweep.csv`
- `artifacts/sweeps/px4_forward_adversarial_sweep.json`
- `artifacts/sweeps/px4_turn_adversarial_sweep.csv`
- `artifacts/sweeps/px4_turn_adversarial_sweep.json`
- `artifacts/sweeps/px4_climb_adversarial_sweep.csv`
- `artifacts/sweeps/px4_climb_adversarial_sweep.json`

The previous turn-regime blocker is now removed on this measured SIH profile: anomaly FPR went from `0.717` to `0.000`, against an acceptance target of below `0.10`. The fix was narrower than the original maneuver-gating hypothesis: heading observations remain implemented, but the PX4 SIH path no longer enables uncalibrated heading checks by default, and persistence warning flags are opt-in for live operator output. The current default structured sweep no longer has zero-rejection cases, but worst-case rejected TPR remains `0.700-0.758`, so this is still a simulator characterization rather than a general robustness claim.

An optional extended adversarial sweep mode is now available through `--extended`. A local smoke run over hover data with two onset times and four ramp durations evaluated `384` cases, preserved nominal `120/0/0` verdicts, and exported CSV/JSON results. This mode adds diagonal, vertical, larger-magnitude, and slower-ramp cases for pre-hardware characterization; it is not part of the published four-mission acceptance table until all mission profiles are rerun in that mode.

### Realistic Spoof-Profile Suite

Observed on `2026-05-13` using `examples/run_realistic_spoof_suite.rs` against the four PX4 SIH mission datasets:

| Profile | Hover rejected TPR | Forward rejected TPR | Turn rejected TPR | Climb rejected TPR | Note |
| --- | ---: | ---: | ---: | ---: | --- |
| `texbat_ds1_static_takeover` | `1.000` | `1.000` | `1.000` | `1.000` | caught immediately |
| `texbat_ds2_overpowered_time_push` | `0.876` | `0.875` | `0.876` | `0.875` | strong generated time-push result |
| `texbat_ds3_matched_power_slow_carryoff` | `0.819` | `0.827` | `0.800` | `0.779` | partially caught; still difficult |
| `texbat_ds7_phase_aligned_time_push` | `0.743` | `0.740` | `0.762` | `0.692` | improved but still weaker than processed TEXBAT |
| `uav_sdr_takeover_30m_10s` | `0.905` | `0.894` | `0.914` | `0.913` | strong software-defined-spoofer-style result |
| `uav_freeze_or_hold_last_fix` | `0.764` | `0.788` | `0.705` | `0.743` | partially caught by stale-GPS persistence in generated replay |
| `nav_wrong_turn_cross_track` | `0.905` | `0.904` | `0.905` | `0.904` | strong wrong-turn profile result |
| `nav_overshoot_along_track` | `0.943` | `0.933` | `0.933` | `0.942` | strong route-overshoot result |
| `intermittent_pulsed_carryoff` | `0.782` | `0.736` | `0.768` | `0.741` | partially caught |

All four nominal mission datasets stayed at `120 / 0 / 0` trusted/flagged/rejected under the same runner. The suite is generated from measured SIH logs and public attack categories; it is not a replacement for running the actual public UAV/RF datasets.

### PX4 SIH Live Spoof Proxy

Abrupt offset profile observed on `2026-05-12` with `scripts/wsl_px4_live_spoof.sh`:

| Metric | Value |
| --- | --- |
| proxy startup delay before spoof begins | `1.0 s` |
| proxy spoof onset after proxy start | `1.5 s` |
| injected position offset | `+90 m north, -50 m east, +8 m down` |
| injected velocity offset | `+10, -5, +1 m/s` |
| verdicts | `13 trusted / 2 flagged / 15 rejected` |
| first flagged verdict | verdict `#14` |
| first rejection | verdict `#16` |
| total packets processed | `339` |
| IMU packets | `309` |
| GPS packets | `30` |

The earlier wording that described this as "first rejection at verdict `#14` / `1.5 s` lag" was misleading. Verdict numbering started before the proxy began spoofing. The current live path intentionally emits `Flagged` before `Rejected`, so this run surfaced warning at verdict `#14` and confirmed rejection at verdict `#16`.

Gradual carry-off profile observed on `2026-05-13` with `scripts/wsl_px4_gradual_spoof.sh`:

| Metric | Value |
| --- | --- |
| proxy startup delay before spoof begins | `1.0 s` |
| proxy spoof onset after proxy start | `1.5 s` |
| injected position offset | north ramp from `0 m` to `30 m` over `2.5 s` |
| injected velocity offset | `0 m/s` |
| verdicts | `25 trusted / 6 flagged / 14 rejected` |
| first clearly spoof-affected GPS packet in the measured run | verdict `#16` |
| first flagged verdict | verdict `#26` |
| first rejection | verdict `#32` |
| total packets processed | `494` |
| IMU packets | `449` |
| GPS packets | `45` |

A separate one-off verbose diagnostic run of the same profile showed the horizontal residual rising from about `1.285 m` at verdict `#16` to `18.191 m` at verdict `#30`, while horizontal CUSUM rose from `0.164` to `35.813`. That diagnostic run was used to confirm that the live path was accumulating residual persistence rather than tracking the spoof away.

Optional calibrated live mode observed on `2026-05-13` using the same PX4 SIH live proxy path:

| Metric | Value |
| --- | --- |
| detector mode | `--calibrate-live --live-warmup-verdicts 12 --live-calibration-min-sigma-m 1.0 --live-calibration-min-slack-sigma 0.2 --live-calibration-min-threshold 3.0` |
| fixed default replaced? | no |
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

This mode exists because a naive live calibration against hover-noise alone was too sensitive. The current opt-in configuration uses a conservative `1.0 m` minimum horizontal sigma and `0.2 / 3.0` minimum slack / threshold floors. On the measured PX4 software-MITM path, that materially improves the slow carry-off floor without introducing false positives on the measured clean live or clean replay runs.

### Processed TEXBAT Replay

| Scenario | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `cleanStatic-baseline` | `2115 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `192.39 us` | `250.80 us` | `680.70 us` |
| `ds2` | `566 / 13 / 1521` | `0.978` | `0.034` | `0.975` | `0.020` | `182.78 us` | `190.30 us` | `283.80 us` |
| `ds3` | `651 / 4 / 1441` | `0.953` | `0.032` | `0.953` | `0.025` | `182.36 us` | `189.70 us` | `280.30 us` |
| `ds7` | `567 / 0 / 1608` | `0.999` | `0.000` | `0.999` | `0.000` | `186.63 us` | `206.60 us` | `341.20 us` |

### TEXBAT Ablation Snapshot

| Scenario | Profile | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR |
| --- | --- | ---: | ---: | ---: | ---: |
| `ds2` | `full` | `0.978` | `0.034` | `0.975` | `0.020` |
| `ds2` | `no_horiz_cusum` | `0.978` | `0.034` | `0.975` | `0.020` |
| `ds2` | `single_epoch_gps_clock` | `0.979` | `0.033` | `0.969` | `0.018` |
| `ds2` | `single_epoch_gps_only` | `0.000` | `0.016` | `0.000` | `0.007` |
| `ds3` | `full` | `0.953` | `0.032` | `0.953` | `0.025` |
| `ds3` | `no_horiz_cusum` | `0.749` | `0.032` | `0.749` | `0.025` |
| `ds3` | `no_persistence` | `0.000` | `0.032` | `0.000` | `0.025` |
| `ds3` | `single_epoch_gps_clock` | `0.000` | `0.030` | `0.000` | `0.025` |
| `ds7` | `full` | `0.999` | `0.000` | `0.999` | `0.000` |
| `ds7` | `no_horiz_cusum` | `0.705` | `0.000` | `0.705` | `0.000` |
| `ds7` | `no_persistence` | `0.662` | `0.000` | `0.615` | `0.000` |
| `ds7` | `single_epoch_gps_only` | `0.000` | `0.000` | `0.000` | `0.000` |

The strongest takeaway from this table is that the harder processed-TEXBAT detections are not coming from plain GPS residual thresholds alone. On `ds3`, removing only the new horizontal CUSUM drops anomaly TPR from `0.953` to `0.749` at the same false-positive rate, and removing persistence entirely drops it to `0.000`.

### Simple Baseline Comparison

The repository now also includes `examples/run_texbat_baselines.rs`, which compares:

- the current full detector
- a naive GPS vs. dead-reckoning position-distance threshold
- a standard position-innovation `N_sigma` threshold with no EWMA, persistence, or clock-bias logic

Observed on `2026-05-12` with defaults of `5.0 m` for the naive distance threshold and `3.0 sigma` for the innovation threshold:

| Scenario | Full TPR/FPR | Naive distance TPR/FPR | Innovation `N_sigma` TPR/FPR |
| --- | --- | --- | --- |
| `cleanStatic` | `0.000 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |
| `ds2` | `0.978 / 0.034` | `0.445 / 0.102` | `0.000 / 0.018` |
| `ds3` | `0.953 / 0.032` | `0.631 / 0.125` | `0.000 / 0.025` |
| `ds7` | `0.999 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |

These defaults are intentionally simple, not tuned for best possible baseline performance. The main use is to quantify what the persistence-heavy full detector is buying over simpler alternatives.

### Evidence Verification

The framed evidence file emitted by the live PX4 spoof path was rechecked with:

```powershell
cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin
```

Observed on `2026-05-13`:

- packets verified: `30`
- trusted verdicts: `13`
- flagged/rejected verdicts: `17`
- evidence chain root: `aee3dce6be23e5ed8ff0674decc34769cab1579e06db539ac265257eb341db36`

The verifier computes this root over the signed framed packet sequence. It is a hash-chain audit root suitable for external recording, not a Merkle tree and not a hardware-backed timestamp.

### Host Monitor Profiling

The replay monitor can be profiled without PX4 using:

```powershell
cargo run --example profile_monitor_dataset -- artifacts/px4_monitor_dataset.csv --iterations 50
```

Observed on `2026-05-13`:

| Dataset | Rows x iterations | Throughput | Mean / p95 / max per iteration | Verdicts |
| --- | ---: | ---: | ---: | ---: |
| `artifacts/px4_monitor_dataset.csv` | `60 x 50` | `3844.8 evaluations/s` | `259.27 / 271.32 / 397.80 us` | `60 / 0 / 0` |

The reported type-size snapshot includes `MonitorDatasetRow` at `240` bytes, `EskfState` at `1008` bytes, `StateCovariance` at `900` bytes, `StatisticalMonitor` at `192` bytes, and `SignedEvidencePacket` at `208` bytes. This is host profiling only; target flight hardware remains unmeasured.

## Limitations

### PX4 SIH Paths

- simulator-only
- localhost-only networking inside WSL2
- spoof profiles were generated by local tooling
- no RF interference or receiver attack path
- the new multi-mission capture path is still PX4 SIH offboard control, not hardware
- the gradual live carry-off result is still a software MITM over processed MAVLink packets, not a receiver- or RF-layer carry-off
- the calibrated live mode is still a software MITM over processed MAVLink packets, not a receiver- or RF-layer carry-off

### Processed TEXBAT Path

- uses processed navigation solutions, not raw IF
- no paired IMU stream is available in this repository
- clean trajectory is used as a reference proxy
- replay noise is now calibrated from the pre-spoof clean segment rather than relying only on fixed observation-noise assumptions
- `ds3` and `ds7` improved materially after calibrating the horizontal residual CUSUM from the pre-spoof clean segment, but they are still processed-replay results rather than hardware or raw-IF results
- the optional immediate trigger hooks are implemented, but they did not materially move the first-rejection point in the current live PX4 spoof profile when trialed locally
- the default structured replay sweep now avoids zero-rejection cases on the four measured SIH datasets, but the worst-case rejected TPR is still only `0.700-0.758`

## Failure Analysis Note

The main `ds3` weakness was not a large instantaneous innovation. In the processed replay used here, `ds3` behaves more like a sustained moderate horizontal carry-off with only partial clock-bias separation from the clean segment. That shape punishes one-shot thresholds and innovation-only baselines.

The current full profile addresses that by combining:

- calibrated observation noise from the clean pre-spoof segment
- clock-bias persistence
- horizontal residual CUSUM with slack and threshold calibrated from the same clean segment
- horizontal velocity persistence and stale-GPS persistence for generated replay profiles

Those changes are why the current processed TEXBAT `ds3` full-profile result moved from `0.907 / 0.032` to `0.953 / 0.032` in anomaly TPR/FPR, while processed TEXBAT `ds7` moved from `0.705 / 0.000` to `0.999 / 0.000`. The generated PX4 SIH realistic spoof-profile suite remains weaker than processed TEXBAT on subtle `ds7`-style time-push.

## Reviewer Guidance

The strongest honest takeaway today is:

- the monitor is a working Rust prototype with measured simulator and processed-dataset behavior

The repository does **not** yet justify claims of:

- fielded performance
- generalized platform robustness
- hardware-qualified latency
- end-to-end RF spoof resilience

For the pre-Phase 1 risk table and recommended next experiments, see [PRE_PHASE1_ASSESSMENT.md](PRE_PHASE1_ASSESSMENT.md).
