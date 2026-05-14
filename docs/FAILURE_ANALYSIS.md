# Failure Analysis

This document explains how to interpret weak or missed replay cases without guessing. The goal is not to make every result look good; it is to show which detector path produced evidence and which path did not.

## Diagnostic Exports

The adversarial sweep and realistic spoof-profile suite now export diagnostic maxima in their CSV and JSON artifacts:

```powershell
cargo run --example run_adversarial_sweep -- artifacts/px4_hover_dataset.csv --dataset-label px4_hover --output-dir artifacts/sweeps
cargo run --example run_realistic_spoof_suite -- artifacts/px4_hover_dataset.csv --dataset-label px4_hover
```

Important fields:

| Field | Meaning |
| --- | --- |
| `max_gps_squared_mahalanobis_distance` | largest single-epoch GPS innovation score |
| `max_accumulated_risk` | largest EWMA risk value |
| `max_horizontal_position_residual_m` | largest horizontal GPS-vs-ESKF position residual |
| `max_horizontal_velocity_residual_mps` | largest horizontal GPS-vs-ESKF velocity residual |
| `max_abs_clock_bias_residual_m` | largest absolute clock-bias residual when clock observations exist |
| `max_clock_bias_persistent_score` | strongest clock-bias CUSUM score |
| `max_horizontal_residual_persistent_score` | strongest horizontal-position CUSUM score |
| `max_velocity_residual_persistent_score` | strongest horizontal-velocity CUSUM score |
| `max_stale_gps_persistent_score` | strongest stale/frozen-GPS score |
| `dominant_signal` | strongest normalized persistence or risk path in the realistic spoof suite |

These fields are diagnostic aids, not independent validation. They explain the internal replay behavior for the current detector configuration.

## Current Weakest Generated Profiles

Measured on the current four PX4 SIH datasets:

| Mission | Weakest realistic profile | Rejected TPR | Dominant measured signal |
| --- | --- | ---: | --- |
| `hover` | `texbat_ds7_phase_aligned_time_push` | `0.743` | horizontal position persistence |
| `forward` | `intermittent_pulsed_carryoff` | `0.736` | horizontal position persistence |
| `turn` | `uav_freeze_or_hold_last_fix` | `0.705` | horizontal position persistence |
| `climb` | `texbat_ds7_phase_aligned_time_push` | `0.692` | horizontal position persistence |

The honest interpretation is that generated SIH replay coverage is broad but not complete. The weakest generated profiles are still being caught by position-residual persistence, not by a separate RF or receiver-layer cue.

## Current Worst-Case Sweep Profiles

The default 144-case sweep currently has zero zero-rejection cases, but worst-case rejected TPR is still partial:

| Mission | Worst-case sweep profile | Rejected TPR | First rejection from onset | Main diagnostic |
| --- | --- | ---: | ---: | --- |
| `hover` | `north_onset_2.0_ramp_20.0_pos` | `0.758` | `30` samples | horizontal score `56.33` |
| `forward` | `northeast_onset_2.0_ramp_20.0_pos` | `0.700` | `32` samples | horizontal score `36.10` |
| `turn` | `northwest_onset_2.0_ramp_20.0_pos` | `0.725` | `34` samples | horizontal score `63.52` |
| `climb` | `northwest_onset_2.0_ramp_20.0_pos` | `0.733` | `33` samples | horizontal score `37.69` |

The weakest sweep cases are slow 20-second position-only ramps with early onset. They eventually reject, but not immediately. This should be described as a remaining slow-carry-off sensitivity limit in generated simulation, not as field-proven robustness.

## What To Improve Next

The diagnostics point to three practical next steps:

- Run the same diagnostic exports on outdoor nominal logs to see whether horizontal residual persistence stays quiet in real multipath and vibration.
- Add external UAV spoof datasets normalized into `MonitorDatasetRow` so weak generated profiles can be compared against non-self-generated data.
- Evaluate whether stale/frozen receiver behavior from real logs produces a stale-fix score; the current generated frozen-GPS replay is dominated by horizontal residual persistence instead.
