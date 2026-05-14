# Artifacts

This directory contains generated datasets, logs, benchmark outputs, sweep exports, and evidence files. These files are measurement artifacts from the current prototype, not source-of-truth code.

## Primary Artifacts

| File or directory | Produced by | Meaning |
| --- | --- | --- |
| `artifacts/texbat/` | `.\scripts\download_texbat_processed.ps1` | processed TEXBAT `navsol.mat` inputs and replay CSV outputs |
| `artifacts/px4_monitor_dataset.csv` | `bash scripts/wsl_px4_benchmark.sh 60` | 60-sample nominal PX4 SIH replay dataset |
| `artifacts/px4_monitor_dataset_spoofed.csv` | `bash scripts/wsl_px4_benchmark.sh 60` | software-injected spoof replay dataset |
| `artifacts/px4_monitor_nominal_report.json` | `cargo run --example report_nominal_dataset -- artifacts/px4_monitor_dataset.csv --json-output artifacts/px4_monitor_nominal_report.json` | JSON false-positive and residual report for the nominal PX4 SIH replay dataset |
| `artifacts/px4_monitor_profile_report.json` | `cargo run --example profile_monitor_dataset -- artifacts/px4_monitor_dataset.csv --iterations 50 --json-output artifacts/px4_monitor_profile_report.json --acceptance-p95-us 10000 --acceptance-max-us 50000` | JSON host replay profiling report with timing acceptance fields |
| `artifacts/px4_hover_dataset.csv` | `bash scripts/wsl_px4_multi_mission_benchmark.sh 120` | hover mission nominal dataset |
| `artifacts/px4_forward_dataset.csv` | `bash scripts/wsl_px4_multi_mission_benchmark.sh 120` | forward mission nominal dataset |
| `artifacts/px4_turn_dataset.csv` | `bash scripts/wsl_px4_multi_mission_benchmark.sh 120` | turn mission nominal dataset |
| `artifacts/px4_climb_dataset.csv` | `bash scripts/wsl_px4_multi_mission_benchmark.sh 120` | climb mission nominal dataset |
| `artifacts/px4_multi_mission_summary.csv` | `bash scripts/wsl_px4_multi_mission_benchmark.sh 120` | compact multi-mission summary |
| `artifacts/wsl_px4_live_spoof_evidence.bin` | `bash scripts/wsl_px4_live_spoof.sh` | signed evidence stream from abrupt live software spoof |
| `artifacts/wsl_px4_gradual_spoof_evidence.bin` | `bash scripts/wsl_px4_gradual_spoof.sh` | signed evidence stream from gradual live software spoof |
| `artifacts/spoof_suites/` | `cargo run --example run_realistic_spoof_suite -- <dataset>` | generated realistic spoof-profile CSV/JSON summaries, including diagnostic maxima and dominant-signal fields |

## Sweep Artifacts

The current measured four-mission sweep exports are:

| File | Meaning |
| --- | --- |
| `artifacts/sweeps/px4_hover_adversarial_sweep.csv` | hover mission sweep, tabular |
| `artifacts/sweeps/px4_hover_adversarial_sweep.json` | hover mission sweep, structured JSON |
| `artifacts/sweeps/px4_forward_adversarial_sweep.csv` | forward mission sweep, tabular |
| `artifacts/sweeps/px4_forward_adversarial_sweep.json` | forward mission sweep, structured JSON |
| `artifacts/sweeps/px4_turn_adversarial_sweep.csv` | turn mission sweep, tabular |
| `artifacts/sweeps/px4_turn_adversarial_sweep.json` | turn mission sweep, structured JSON |
| `artifacts/sweeps/px4_climb_adversarial_sweep.csv` | climb mission sweep, tabular |
| `artifacts/sweeps/px4_climb_adversarial_sweep.json` | climb mission sweep, structured JSON |

Some older unprefixed sweep files are also present from earlier runs. The `px4_*` files above are the current four-mission set referenced by the README and benchmark summary.

The current sweep CSV/JSON files include diagnostic `max_*` fields for residuals, EWMA risk, and persistence scores. These fields are intended for failure analysis, not as independent validation.

## Evidence Verification

Verify the abrupt live-spoof evidence file:

```powershell
cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin
```

Current expected output:

```text
Evidence file: artifacts/wsl_px4_live_spoof_evidence.bin
  packets verified: 30
  trusted verdicts: 13
  flagged/rejected verdicts: 17
  diagnostic packets: 0
  first timestamp (ns): 3796000000
  last timestamp (ns): 6700000000
  evidence chain root: aee3dce6be23e5ed8ff0674decc34769cab1579e06db539ac265257eb341db36
```

The chain root is computed by the verifier over the signed framed packet sequence. It is useful for external anchoring or comparison between evidence files, but it is not a Merkle tree and it is not externally anchored by this repository.

## Notes

- These artifacts are reproducibility aids.
- The live-spoof evidence files come from software MAVLink proxy tests, not RF-layer spoofing.
- The processed TEXBAT artifacts are navigation-solution products, not raw IF captures.
- Regenerating artifacts may change latency values slightly because timing depends on host load.
