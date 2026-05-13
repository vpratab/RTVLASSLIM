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

## Measured Results

### PX4 SIH Capture / Replay

| Dataset | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| nominal replay | `60 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `333.17 us` | `377.31 us` | `935.51 us` |
| spoofed replay | `0 / 0 / 60` | `1.000` | `0.000` | `1.000` | `0.000` | `312.32 us` | `334.31 us` | `382.50 us` |

### PX4 SIH Multi-Mission Nominal + Replay Sweep

Observed on `2026-05-13` using `scripts/wsl_px4_multi_mission_benchmark.sh 120`:

| Mission | GPS path summary | Nominal verdicts | Nominal anomaly FPR | Nominal rejected FPR | Standard replayed spoof anomaly / rejected TPR | Worst-case rejected TPR in 144-case sweep | Zero-rejection sweep cases |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `hover` | x `[-0.10, 0.23]`, y `[-0.31, 0.17]`, z `[-6.05, 0.03]`, max speed `2.63 m/s` | `120 / 0 / 0` | `0.000` | `0.000` | `0.670 / 0.550` | `0.000` | `56 / 144` |
| `forward` | x `[0.00, 30.78]`, y `[-0.61, 0.51]`, z `[-6.11, 0.00]`, max speed `4.75 m/s` | `120 / 0 / 0` | `0.000` | `0.000` | `0.684 / 0.510` | `0.000` | `54 / 144` |
| `turn` | x `[-12.52, 11.72]`, y `[-0.26, 24.61]`, z `[-6.00, 0.10]`, max speed `4.84 m/s` | `34 / 30 / 56` | `0.717` | `0.467` | `0.980 / 0.970` | `0.425` | `0 / 144` |
| `climb` | x `[-0.03, 10.41]`, y `[-0.55, 0.40]`, z `[-20.01, 0.07]`, max speed `3.11 m/s` | `120 / 0 / 0` | `0.000` | `0.000` | `0.676 / 0.520` | `0.000` | `56 / 144` |

Structured exports now available per mission:

- `artifacts/sweeps/px4_hover_adversarial_sweep.csv`
- `artifacts/sweeps/px4_hover_adversarial_sweep.json`
- `artifacts/sweeps/px4_forward_adversarial_sweep.csv`
- `artifacts/sweeps/px4_forward_adversarial_sweep.json`
- `artifacts/sweeps/px4_turn_adversarial_sweep.csv`
- `artifacts/sweeps/px4_turn_adversarial_sweep.json`
- `artifacts/sweeps/px4_climb_adversarial_sweep.csv`
- `artifacts/sweeps/px4_climb_adversarial_sweep.json`

The most important operational takeaway from this matrix is not flattering: the current detector is clean on the measured hover, forward, and climb profiles, but the measured turn profile drives nominal anomaly FPR to `0.717`. The structured sweep also shows broad zero-rejection slow-ramp space in the calmer regimes. This means the repository now has a more credible characterization of detector limits, not just better headline numbers.

### PX4 SIH Live Spoof Proxy

Abrupt offset profile observed on `2026-05-12` with `scripts/wsl_px4_live_spoof.sh`:

| Metric | Value |
| --- | --- |
| proxy startup delay before spoof begins | `1.0 s` |
| proxy spoof onset after proxy start | `1.5 s` |
| injected position offset | `+90 m north, -50 m east, +8 m down` |
| injected velocity offset | `+10, -5, +1 m/s` |
| verdicts | `13 trusted / 0 flagged / 17 rejected` |
| first spoofed GPS packet in the measured run | verdict `#15` |
| first rejection | verdict `#15` |
| total packets processed | `339` |
| IMU packets | `309` |
| GPS packets | `30` |

The earlier wording that described this as "first rejection at verdict `#14` / `1.5 s` lag" was misleading. Verdict numbering started before the proxy began spoofing. In the measured run, verdicts `#1` through `#14` were clean pre-spoof packets and the detector rejected on the first spoofed GPS packet.

Gradual carry-off profile observed on `2026-05-13` with `scripts/wsl_px4_gradual_spoof.sh`:

| Metric | Value |
| --- | --- |
| proxy startup delay before spoof begins | `1.0 s` |
| proxy spoof onset after proxy start | `1.5 s` |
| injected position offset | north ramp from `0 m` to `30 m` over `2.5 s` |
| injected velocity offset | `0 m/s` |
| verdicts | `25 trusted / 4 flagged / 1 rejected` |
| first clearly spoof-affected GPS packet in the measured run | verdict `#16` |
| first flagged verdict | verdict `#26` |
| first rejection | verdict `#30` |
| total packets processed | `328` |
| IMU packets | `298` |
| GPS packets | `30` |

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
| `cleanStatic-baseline` | `2115 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `185.23 us` | `267.60 us` | `309.00 us` |
| `ds2` | `566 / 13 / 1521` | `0.978` | `0.034` | `0.975` | `0.020` | `184.16 us` | `212.60 us` | `340.60 us` |
| `ds3` | `651 / 4 / 1441` | `0.953` | `0.032` | `0.953` | `0.025` | `184.00 us` | `266.10 us` | `371.80 us` |
| `ds7` | `567 / 0 / 1608` | `0.999` | `0.000` | `0.999` | `0.000` | `181.67 us` | `193.00 us` | `340.00 us` |

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

Observed on `2026-05-11`:

- packets verified: `30`
- trusted verdicts: `13`
- flagged/rejected verdicts: `17`

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
- the new structured replay sweep shows that many slow position-plus-velocity carry-off cases still evade rejection in calm PX4 captures, and that the current turn profile causes large nominal false positives

## Failure Analysis Note

The main `ds3` weakness was not a large instantaneous innovation. In the processed replay used here, `ds3` behaves more like a sustained moderate horizontal carry-off with only partial clock-bias separation from the clean segment. That shape punishes one-shot thresholds and innovation-only baselines.

The current full profile addresses that by combining:

- calibrated observation noise from the clean pre-spoof segment
- clock-bias persistence
- horizontal residual CUSUM with slack and threshold calibrated from the same clean segment

That change is why the current `ds3` full-profile result moved from `0.907 / 0.032` to `0.953 / 0.032` in anomaly TPR/FPR, while `ds7` moved from `0.705 / 0.000` to `0.999 / 0.000`.

## Reviewer Guidance

The strongest honest takeaway today is:

- the monitor is a working Rust prototype with measured simulator and processed-dataset behavior

The repository does **not** yet justify claims of:

- fielded performance
- generalized platform robustness
- hardware-qualified latency
- end-to-end RF spoof resilience
