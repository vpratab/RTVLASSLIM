# Spoof Datasets And Integration Status

This document tracks realistic spoof sources that can strengthen RTVLAS-Slim before hardware flight testing. It separates what is integrated now from what still requires dataset access, receiver processing, or normalization into the repository monitor-dataset format.

## Integrated Now

The repository now includes a generated realistic spoof-profile suite:

```powershell
cargo run --example run_realistic_spoof_suite -- artifacts/px4_hover_dataset.csv --dataset-label px4_hover
```

The suite applies replayable attack profiles to captured PX4 SIH monitor datasets:

- TEXBAT-like abrupt takeover, overpowered time-push, matched-power slow carry-off, and subtle phase-aligned time-push.
- UAV-spoofer-like 30 m / 10 s SDR takeover.
- Hold-last-fix / frozen GPS.
- Navigation-deception profiles: wrong-turn cross-track and along-track route overshoot.
- Intermittent pulsed carry-off.

These are generated profiles over measured SIH logs. They are useful for pre-hardware characterization, but they are not real RF captures and not real flight tests.

## Public Sources To Add Next

| Source | Best use in RTVLAS-Slim | Current integration status |
| --- | --- | --- |
| [TEXBAT, University of Texas Radionavigation Lab](https://radionavlab.ae.utexas.edu/texbat/) | Expand from processed `navsol.mat` replay to raw IF receiver-processing experiments and additional scenarios. | Processed `ds2/ds3/ds7` path is implemented; raw IF path is not. |
| [UAV Attack Dataset, IEEE DataPort DOI 10.21227/00dg-0d12](https://doi.org/10.21227/00dg-0d12) | Closest public UAV-oriented source for GPS spoofing/jamming-style validation. Normalize its flight/log fields into `MonitorDatasetRow` CSV before running the monitor. | Not downloaded or measured in this repository. |
| [OAKBAT GPS, Oak Ridge Spoofing and Interference Test Battery](https://impact.ornl.gov/en/datasets/oak-ridge-spoofing-and-interference-test-battery-oakbat-gps/) | RF/IQ spoofing and interference source for receiver-processing studies. | Not directly consumable by RTVLAS; requires GNSS-SDR or equivalent to produce navigation solutions. |
| [Tuni2025 GNSS spoofing datasets, Zenodo](https://zenodo.org/records/15624648) | Raw I/Q lab captures for spoofing, multipath, and delayed all-PRN injection studies. | Not directly consumable by RTVLAS; requires receiver processing first. |
| [Mendeley UAS GPS spoofing dataset](https://data.mendeley.com/datasets/z7dj3yyzt8/3) | UAS-oriented authentic/simulated GPS spoofing features for comparison against navigation-solution-level monitors. | Not normalized or measured in this repository. |

## Normalization Target

All external datasets should be converted into the existing monitor dataset CSV schema before they are used for claims:

```text
timestamp_s
state_px_ned_m,state_py_ned_m,state_pz_ned_m
state_vx_ned_mps,state_vy_ned_mps,state_vz_ned_mps
gps_px_ned_m,gps_py_ned_m,gps_pz_ned_m
gps_vx_ned_mps,gps_vy_ned_mps,gps_vz_ned_mps
gps_*_std_*
label_spoofed
optional: reference_clock_bias_m,observed_clock_bias_m,clock_bias_std_m
```

The important rule is that `state_*` must come from an independent inertial or trusted reference path, not from the same spoofed GPS solution being evaluated. If the external dataset does not include an independent reference, it can still be used for parser development, but not for a strong detection claim.

## Current Honest Gaps

- The generated hold-last-fix / frozen GPS profile is not caught in the current replay setup.
- The generated subtle phase-aligned time-push profile is only partially caught in PX4 SIH replay, even though processed TEXBAT `ds7` remains strong.
- Raw RF/IQ datasets are not integrated until a receiver-processing step produces navigation solutions.
- None of these sources replace outdoor receiver logs or real flight testing.
