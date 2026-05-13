# Algorithm

This document explains the current RTVLAS-Slim detector without source code. It describes what the algorithm computes, which measurements drive each decision path, and what the ablation results prove.

For measured benchmark tables, see [benchmark-summary.md](benchmark-summary.md). For baseline comparisons, see [BASELINES.md](BASELINES.md).

## Detector Summary

RTVLAS-Slim compares a predicted navigation state from IMU propagation against GPS position and velocity claims. A GPS update is not trusted just because it is well-formed MAVLink. It must be physically consistent with the predicted local NED state, the innovation covariance, and the persistence scores accumulated across recent samples.

The current decision paths are:

- GPS innovation Mahalanobis distance.
- EWMA risk accumulation over total squared innovation evidence.
- Optional clock-bias CUSUM when a clock-bias observation is available.
- Horizontal position-residual CUSUM.
- Horizontal velocity-residual CUSUM.
- Stale-GPS persistence for held or frozen position fixes in replay.
- Optional `Flagged` warning before confirmed `Rejected` output in the live path.

## ESKF Propagation

The ESKF nominal state contains:

| Component | Meaning |
| --- | --- |
| `p` | position in local NED meters |
| `v` | velocity in local NED meters per second |
| `q` | body-to-NED attitude quaternion |
| `b_a` | accelerometer bias |
| `b_g` | gyro bias |

The 15-dimensional error state tracks position, velocity, attitude error, accelerometer bias error, and gyro bias error. The state covariance propagates with the IMU noise model.

At each `HIGHRES_IMU` sample:

```text
a_unbiased = a_measured - b_a
w_unbiased = w_measured - b_g
q_next = q * exp(w_unbiased * dt)
a_ned = R(q_mid) * a_unbiased + gravity_ned
p_next = p + v * dt + 0.5 * a_ned * dt^2
v_next = v + a_ned * dt
P_next = F * P * F^T + Q
```

The implementation uses quaternion integration, NED gravity, and a discrete covariance propagation. The propagation path is tested by `cargo test --lib`.

## GPS Innovation

The GPS observation vector is six-dimensional:

```text
z = [p_n, p_e, p_d, v_n, v_e, v_d]^T
```

The predicted observation is extracted directly from the nominal ESKF state:

```text
z_hat = [p_hat_n, p_hat_e, p_hat_d, v_hat_n, v_hat_e, v_hat_d]^T
```

The innovation is:

```text
y = z - z_hat
```

The observation Jacobian maps directly into the position and velocity blocks of the 15-state error vector:

```text
H = [ I_3  0    0    0    0
      0    I_3  0    0    0 ]
```

The innovation covariance is:

```text
S = H * P * H^T + R
```

`R` comes from GPS accuracy assumptions or replay calibration. The monitor computes squared Mahalanobis distance using Cholesky factorization, not a naive unchecked matrix inverse:

```text
D_M^2 = y^T * S^-1 * y
```

If `S` is not positive definite, evaluation fails instead of silently producing an invalid score.

## EWMA Risk

The EWMA path accumulates total squared evidence across GPS, barometer, heading when enabled, and clock-bias terms:

```text
risk_k = alpha * D_total,k^2 + (1 - alpha) * risk_k-1
```

In the benchmark harness, the current TEXBAT replay uses `alpha = 0.6`. The EWMA path is useful for smoothing repeated evidence, but the strongest TEXBAT results come from the persistence paths below.

## Clock-Bias CUSUM

Clock bias is used when the harness provides an observed receiver clock-bias term and a clean-segment reference. The residual is:

```text
e_clock = clock_bias_observed - clock_bias_reference
```

The normalized residual is:

```text
n_clock = abs(e_clock) / sigma_clock
```

The CUSUM score is:

```text
score_clock = max(0, score_clock + n_clock - slack_clock)
```

If `score_clock` crosses its rejection threshold, the monitor can reject even if a single GPS position sample is not individually extreme. This matters because some spoofing profiles accumulate evidence gradually.

## Horizontal-Residual CUSUM

The horizontal position residual is:

```text
h = norm([y_position_n, y_position_e])
```

The normalized residual is:

```text
n_h = h / sigma_h
```

The CUSUM score is:

```text
score_h = max(0, score_h + n_h - slack_h)
```

The TEXBAT harness calibrates `sigma_h`, `slack_h`, and the rejection threshold from the clean pre-spoof segment. This is the main path that improved gradual processed-replay performance on `ds3` and `ds7`.

## Velocity-Residual CUSUM

The horizontal velocity residual is:

```text
v_h = norm([y_velocity_n, y_velocity_e])
```

The normalized residual uses the horizontal velocity innovation covariance with a floor:

```text
n_v = v_h / max(sigma_v, 0.5 m/s)
```

The CUSUM score is:

```text
score_v = max(0, score_v + n_v - slack_v)
```

This path was added to catch profiles where spoofed position and velocity move together. Later replay hardening, including pre-spoof residual calibration and stale-GPS persistence, reduced zero-rejection cases in the default four-mission sweep to `0 / 144` for hover, forward, turn, and climb while keeping nominal FPR at `0.000`.

## Stale-GPS Persistence

A frozen or hold-last-fix GPS stream may not produce a large instantaneous GPS residual if the vehicle is initially slow or near-stationary. The stale-GPS path compares the change in predicted ESKF position against the change in observed GPS position between successive GPS epochs:

```text
delta_pred = norm(p_hat,k - p_hat,k-1)
delta_gps  = norm(p_gps,k - p_gps,k-1)
```

When the predicted state is confident enough, the predicted displacement is above a minimum motion threshold, and the observed GPS displacement is near-static, the score accumulates:

```text
score_stale = max(0, score_stale + delta_pred - delta_gps - slack_stale)
```

This path is deliberately gated by predicted position uncertainty so that it does not punish normal hover noise or early convergence. It improved the generated hold-last-fix profile from `0.000` rejected TPR to `0.705-0.788` across the four PX4 SIH replay datasets. That is generated-replay evidence only, not proof against real receiver stale-output behavior.

## Flag-Then-Confirm State Machine

The live path can surface early operator warnings before committing to rejection:

```text
if raw_reject:
    emit Flagged for N confirming epochs
    then emit Rejected
else if raw_flag:
    emit Flagged
else:
    emit Trusted
```

The measured abrupt live spoof run produced `13 / 2 / 15` trusted/flagged/rejected verdicts. The first `Flagged` verdict was `#14`; the first `Rejected` verdict was `#16`.

This state machine is operational output shaping. It does not loosen the TEXBAT rejection thresholds.

## TEXBAT Scenario Stress

| Scenario | What it stresses in this repository |
| --- | --- |
| `cleanStatic` | clean processed-navigation baseline and false-positive control |
| `ds2` | abrupt carry-off, where large inconsistencies should appear quickly |
| `ds3` | gradual low-magnitude drift, which punishes one-shot thresholds |
| `ds7` | subtle time-push / phase-aligned behavior, where persistence matters |

## Ablation Results

Command:

```powershell
cargo run --example run_texbat_ablation
```

Measured on `2026-05-13`:

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

## What The Ablation Proves

The `ds3` result is the clearest evidence. Removing horizontal CUSUM drops anomaly TPR from `0.953` to `0.749`, and removing persistence entirely drops detection to `0.000`. That means the gradual-drift result is not explained by a single GPS residual threshold.

The `ds7` result shows the same pattern. The full detector reaches `0.999` anomaly TPR, while `single_epoch_gps_only` remains at `0.000`.

The `ds2` result is easier. Several profiles perform similarly because abrupt carry-off produces stronger instantaneous evidence.

## Current Novelty Claim

The defensible technical claim is narrow:

RTVLAS-Slim combines ESKF-based GPS innovation monitoring with parallel calibrated persistence paths for clock bias, horizontal position residual, and horizontal velocity residual, then emits signed per-verdict evidence. The measured value over simpler baselines appears in the processed TEXBAT `ds3` and `ds7` ablations, where persistence changes the result materially.

This is not a claim that RTVLAS-Slim detects RF-layer spoofing or replaces receiver-internal spoofing monitors.
