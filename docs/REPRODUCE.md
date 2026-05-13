# Reproduce Results

This guide lists the commands used to reproduce the benchmark tables in the README and [benchmark-summary.md](benchmark-summary.md).

## Prerequisites

Required:

- Rust toolchain with `cargo`.
- Windows PowerShell for local Rust and TEXBAT commands.
- WSL2 for PX4 SIH scripts.
- PX4-Autopilot SIH build available at `external/PX4-Autopilot/build/px4_sitl_sih`.
- Processed TEXBAT artifacts downloaded into `artifacts/texbat`.

The current scripts assume the workspace path used by this checkout. If the repository is moved, update the `workspace` variable in the WSL helper scripts before running PX4 commands.

## Rust Checks

Run:

```powershell
cargo fmt --all --check
cargo check --no-default-features
cargo check --all-targets
cargo test --lib
```

Successful current result:

```text
test result: ok. 32 passed; 0 failed
```

## TEXBAT Processed Replay

Download processed TEXBAT artifacts:

```powershell
.\scripts\download_texbat_processed.ps1
```

Run the harness:

```powershell
cargo run --example run_texbat_harness
```

Expected current result range:

| Scenario | Expected anomaly TPR/FPR |
| --- | ---: |
| `cleanStatic-baseline` | `0.000 / 0.000` |
| `ds2` | about `0.978 / 0.034` |
| `ds3` | about `0.953 / 0.032` |
| `ds7` | about `0.999 / 0.000` |

Run ablations:

```powershell
cargo run --example run_texbat_ablation
```

Run simple baselines:

```powershell
cargo run --example run_texbat_baselines
```

Successful output includes the full detector, naive distance threshold, and innovation `N_sigma` comparison table.

## PX4 SIH Replay Benchmark

Run:

```bash
bash scripts/wsl_px4_benchmark.sh 60
```

Expected current result:

```text
Nominal dataset: artifacts/px4_monitor_dataset.csv
  trusted/flagged/rejected: 60/0/0
  anomaly FPR: 0.000
  rejected FPR: 0.000
Spoofed dataset: artifacts/px4_monitor_dataset_spoofed.csv
  trusted/flagged/rejected: 0/0/60
  anomaly TPR/FPR: 1.000/0.000
  rejected TPR/FPR: 1.000/0.000
```

If PX4 fails to start, check that `external/PX4-Autopilot/build/px4_sitl_sih/bin/px4` exists and that no old PX4 process is still bound to the expected UDP ports.

## PX4 SIH Multi-Mission Benchmark

Run:

```bash
bash scripts/wsl_px4_multi_mission_benchmark.sh 120
```

Expected current nominal result:

| Mission | Expected nominal verdicts | Expected nominal anomaly FPR |
| --- | ---: | ---: |
| `hover` | `120/0/0` | `0.000` |
| `forward` | `120/0/0` | `0.000` |
| `turn` | `120/0/0` | `0.000` |
| `climb` | `120/0/0` | `0.000` |

This command also writes per-mission datasets, spoofed datasets, logs, and adversarial sweep exports under `artifacts`.

## Adversarial Sweep

Run one sweep directly:

```powershell
cargo run --example run_adversarial_sweep -- artifacts/px4_hover_dataset.csv --dataset-label hover --output-dir artifacts/sweeps
```

Expected output:

```text
Cases evaluated: 144
Nominal trusted/flagged/rejected: 120/0/0
Nominal anomaly FPR: 0.000
CSV export: artifacts/sweeps\hover_adversarial_sweep.csv
JSON export: artifacts/sweeps\hover_adversarial_sweep.json
```

How to read the CSV:

- `scenario_label` names direction, onset, ramp duration, and offset mode.
- `anomaly_tpr` counts `Flagged` or `Rejected` during spoof-labeled samples.
- `rejected_tpr` counts only `Rejected` during spoof-labeled samples.
- `samples_from_onset_to_first_rejection` is empty when a case never reaches rejection.

## Live PX4 Software Spoof Proxy

Abrupt profile:

```bash
bash scripts/wsl_px4_live_spoof.sh
```

Expected current result:

```text
trusted/flagged/rejected: 13/2/15
```

Gradual profile:

```bash
bash scripts/wsl_px4_gradual_spoof.sh
```

Expected current result:

```text
trusted/flagged/rejected: 25/6/14
```

These are software MAVLink man-in-the-middle tests. They are not RF spoofing tests.

## Evidence Verification

Run:

```powershell
cargo run --example verify_evidence artifacts/wsl_px4_live_spoof_evidence.bin
```

Expected current output:

```text
Evidence file: artifacts/wsl_px4_live_spoof_evidence.bin
  packets verified: 30
  trusted verdicts: 13
  flagged/rejected verdicts: 17
  first timestamp (ns): 3796000000
  last timestamp (ns): 6700000000
```

If verification fails, the evidence file may be missing, truncated, or modified.

## Successful Run Versus Failed Run

A successful run produces:

- A zero exit code.
- The expected verdict counts or close matching TPR/FPR values.
- Generated artifacts in `artifacts` or `artifacts/sweeps`.
- No `evidence verification failed` message.

A failed run usually shows:

- Missing TEXBAT files under `artifacts/texbat`.
- Missing PX4 binary under `external/PX4-Autopilot/build/px4_sitl_sih`.
- UDP port conflicts from a stale PX4 process.
- Evidence truncation or signature verification failure.

## Related Documents

- [benchmark-summary.md](benchmark-summary.md)
- [verification.md](verification.md)
- [ALGORITHM.md](ALGORITHM.md)
- [BASELINES.md](BASELINES.md)
- [THREAT_MODEL.md](THREAT_MODEL.md)
- [FORENSICS.md](FORENSICS.md)
