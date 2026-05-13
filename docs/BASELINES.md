# Baselines And Ablations

This document collects the baseline and ablation results in one place. Every table here is produced by a runnable command.

## Why These Baselines

The baselines were chosen to answer a reviewer question: is RTVLAS-Slim doing something beyond a simple GPS consistency threshold?

The comparisons are:

- Full RTVLAS-Slim detector.
- Naive GPS-vs-dead-reckoning Euclidean position-distance threshold.
- Standard innovation `N_sigma` threshold with no EWMA, CUSUM, clock-bias persistence, or warning state machine.

These represent the simple alternatives an evaluator would reasonably ask about before accepting the added complexity.

## Three-Way TEXBAT Baseline

Command:

```powershell
cargo run --example run_texbat_baselines
```

Measured with:

- naive distance threshold: `5.0 m`
- innovation threshold: `3.0 sigma`

| Scenario | RTVLAS full TPR/FPR | Naive distance TPR/FPR | Innovation `N_sigma` TPR/FPR |
| --- | ---: | ---: | ---: |
| `cleanStatic` | `0.000 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |
| `ds2` | `0.978 / 0.034` | `0.445 / 0.102` | `0.000 / 0.018` |
| `ds3` | `0.953 / 0.032` | `0.631 / 0.125` | `0.000 / 0.025` |
| `ds7` | `0.999 / 0.000` | `0.000 / 0.000` | `0.000 / 0.000` |

## Ablation Table

Command:

```powershell
cargo run --example run_texbat_ablation
```

| Scenario | Profile | Anomaly TPR | Anomaly FPR | Rejected TPR | Rejected FPR |
| --- | --- | ---: | ---: | ---: | ---: |
| `ds2` | `full` | `0.978` | `0.034` | `0.975` | `0.020` |
| `ds2` | `no_horiz_cusum` | `0.978` | `0.034` | `0.975` | `0.020` |
| `ds2` | `full_hybrid` | `0.978` | `0.034` | `0.975` | `0.020` |
| `ds2` | `no_persistence` | `0.978` | `0.034` | `0.968` | `0.020` |
| `ds2` | `single_epoch_gps_clock` | `0.979` | `0.033` | `0.969` | `0.018` |
| `ds2` | `single_epoch_gps_only` | `0.000` | `0.016` | `0.000` | `0.007` |
| `ds3` | `full` | `0.953` | `0.032` | `0.953` | `0.025` |
| `ds3` | `no_horiz_cusum` | `0.749` | `0.032` | `0.749` | `0.025` |
| `ds3` | `full_hybrid` | `0.953` | `0.032` | `0.953` | `0.025` |
| `ds3` | `no_persistence` | `0.000` | `0.032` | `0.000` | `0.025` |
| `ds3` | `single_epoch_gps_clock` | `0.000` | `0.030` | `0.000` | `0.025` |
| `ds3` | `single_epoch_gps_only` | `0.000` | `0.022` | `0.000` | `0.012` |
| `ds7` | `full` | `0.999` | `0.000` | `0.999` | `0.000` |
| `ds7` | `no_horiz_cusum` | `0.705` | `0.000` | `0.705` | `0.000` |
| `ds7` | `full_hybrid` | `0.999` | `0.000` | `0.999` | `0.000` |
| `ds7` | `no_persistence` | `0.662` | `0.000` | `0.615` | `0.000` |
| `ds7` | `single_epoch_gps_clock` | `0.663` | `0.000` | `0.615` | `0.000` |
| `ds7` | `single_epoch_gps_only` | `0.000` | `0.000` | `0.000` | `0.000` |

## Scenario Interpretation

`cleanStatic` is the nominal processed-navigation baseline. The full system, naive distance threshold, and innovation baseline all produce `0.000` FPR here.

`ds2` is easier for many detectors because abrupt carry-off creates stronger instantaneous evidence. The full detector reaches `0.978 / 0.034`, but some simpler clock-aware paths also perform strongly.

`ds3` is the most important stress case in the current repository. The full detector reaches `0.953 / 0.032`, while `single_epoch_gps_clock` and `single_epoch_gps_only` both produce `0.000` TPR. Removing horizontal CUSUM drops anomaly TPR to `0.749`. This is the strongest evidence that sequential persistence is materially improving detection.

`ds7` also shows the value of persistence. The full detector reaches `0.999 / 0.000`; the GPS-only baseline remains `0.000 / 0.000`, and removing horizontal CUSUM drops anomaly TPR to `0.705`.

## What The Baselines Do Not Prove

These baselines do not prove global optimality. The naive thresholds were intentionally simple defaults, not a fully tuned competing product.

They do prove a narrower point: the measured TEXBAT outcomes are not explained by a plain Euclidean threshold or a standard single-epoch innovation threshold.
