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

### PX4 SIH Live Spoof Proxy

| Metric | Value |
| --- | --- |
| spoof onset | `1.5 s` |
| injected position offset | `+90 m north, -50 m east, +8 m down` |
| injected velocity offset | `+10, -5, +1 m/s` |
| verdicts | `13 trusted / 0 flagged / 17 rejected` |
| first rejection | verdict `#14` |
| total packets processed | `341` |
| IMU packets | `311` |
| GPS packets | `30` |

### Processed TEXBAT Replay

| Scenario | Trusted / Flagged / Rejected | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR | Mean latency | P95 latency | Max latency |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `cleanStatic-baseline` | `2115 / 0 / 0` | n/a | `0.000` | n/a | `0.000` | `176.50 us` | `183.10 us` | `295.10 us` |
| `ds2` | `566 / 13 / 1521` | `0.978` | `0.034` | `0.975` | `0.020` | `178.35 us` | `188.50 us` | `286.80 us` |
| `ds3` | `956 / 4 / 1136` | `0.749` | `0.032` | `0.749` | `0.025` | `182.10 us` | `222.30 us` | `622.00 us` |
| `ds7` | `1040 / 0 / 1135` | `0.705` | `0.000` | `0.705` | `0.000` | `177.97 us` | `186.00 us` | `422.90 us` |

### TEXBAT Ablation Snapshot

| Scenario | Profile | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR |
| --- | --- | ---: | ---: | ---: | ---: |
| `ds2` | `full` | `0.978` | `0.034` | `0.975` | `0.020` |
| `ds2` | `single_epoch_gps_clock` | `0.979` | `0.033` | `0.969` | `0.018` |
| `ds2` | `single_epoch_gps_only` | `0.000` | `0.016` | `0.000` | `0.007` |
| `ds3` | `full` | `0.749` | `0.032` | `0.749` | `0.025` |
| `ds3` | `no_persistence` | `0.000` | `0.032` | `0.000` | `0.025` |
| `ds3` | `single_epoch_gps_clock` | `0.000` | `0.030` | `0.000` | `0.025` |
| `ds7` | `full` | `0.705` | `0.000` | `0.705` | `0.000` |
| `ds7` | `no_persistence` | `0.662` | `0.000` | `0.615` | `0.000` |
| `ds7` | `single_epoch_gps_only` | `0.000` | `0.000` | `0.000` | `0.000` |

The strongest takeaway from this table is that the harder processed-TEXBAT detections are not coming from plain GPS residual thresholds alone. On `ds3`, removing clock-bias persistence drops anomaly TPR from `0.749` to `0.000` at roughly the same false-positive rate.

### Simple Baseline Comparison

The repository now also includes `examples/run_texbat_baselines.rs`, which compares:

- the current full detector
- a naive GPS vs. dead-reckoning position-distance threshold
- a standard position-innovation `N_sigma` threshold with no EWMA, persistence, or clock-bias logic

Observed on `2026-05-11` with defaults of `5.0 m` for the naive distance threshold and `3.0 sigma` for the innovation threshold:

| Scenario | Full TPR/FPR | Naive distance TPR/FPR | Innovation `N_sigma` TPR/FPR |
| --- | --- | --- | --- |
| `cleanStatic` | `0.000 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |
| `ds2` | `0.978 / 0.034` | `0.445 / 0.102` | `0.000 / 0.018` |
| `ds3` | `0.749 / 0.032` | `0.631 / 0.125` | `0.000 / 0.025` |
| `ds7` | `0.705 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |

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

### Processed TEXBAT Path

- uses processed navigation solutions, not raw IF
- no paired IMU stream is available in this repository
- clean trajectory is used as a reference proxy
- replay noise is now calibrated from the pre-spoof clean segment rather than relying only on fixed observation-noise assumptions
- `ds3` still remains a partial-detection case, even though its clean false positives are substantially lower than in the earlier fixed-noise proxy
- the optional immediate trigger hooks are implemented, but they did not materially move the first-rejection point in the current live PX4 spoof profile when trialed locally

## Reviewer Guidance

The strongest honest takeaway today is:

- the monitor is a working Rust prototype with measured simulator and processed-dataset behavior

The repository does **not** yet justify claims of:

- fielded performance
- generalized platform robustness
- hardware-qualified latency
- end-to-end RF spoof resilience
