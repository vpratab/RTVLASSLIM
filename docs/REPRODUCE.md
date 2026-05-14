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
cargo clippy --all-targets --all-features -- -D warnings
cargo test --lib
```

Successful current result:

```text
test result: ok. 38 passed; 0 failed
```

These checks prove the crate builds and the library tests pass on the local development host. They do not measure CPU load, memory use, or scheduling behavior on representative flight hardware.

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

## Outdoor Nominal Logging Workflow

The repository now includes a passive capture path for real receiver or outdoor MAVLink logs. It does not arm a vehicle, does not request offboard mode, and does not send mission setpoints when `--mission-profile passive` is used:

```powershell
cargo run --example capture_monitor_dataset -- --connection udpout:127.0.0.1:14550 --mission-profile passive --samples 1800 --output artifacts/outdoor_nominal_dataset.csv
```

After capture, generate a nominal false-positive report:

```powershell
cargo run --example report_nominal_dataset -- artifacts/outdoor_nominal_dataset.csv --json-output artifacts/outdoor_nominal_report.json --acceptance-fpr 0.01
```

The report prints trusted/flagged/rejected counts, anomaly and rejected FPR, horizontal residual statistics, velocity residual statistics, and monitor latency. It exits with failure if the dataset contains spoof-labeled rows or if FPR exceeds the configured threshold. This workflow is intended for outdoor nominal evidence; it is not a spoofing or RF-layer validation by itself.

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

Run the broader pre-hardware characterization grid:

```powershell
cargo run --example run_adversarial_sweep -- artifacts/px4_hover_dataset.csv --dataset-label hover_extended --output-dir artifacts/sweeps --extended --onsets 2.0,4.0 --ramps 0.0,5.0,20.0,40.0
```

Expected smoke-test output from the current local run:

```text
Sweep profile: extended
Cases evaluated: 384
Nominal trusted/flagged/rejected: 120/0/0
Nominal anomaly FPR: 0.000
```

How to read the CSV:

- `scenario_label` names direction, onset, ramp duration, and offset mode.
- `anomaly_tpr` counts `Flagged` or `Rejected` during spoof-labeled samples.
- `rejected_tpr` counts only `Rejected` during spoof-labeled samples.
- `samples_from_onset_to_first_rejection` is empty when a case never reaches rejection.

The default sweep is the measured four-mission table in the README. The `--extended` sweep adds diagonal, vertical, larger-magnitude, and slower-ramp cases for pre-hardware adversarial characterization. It is deliberately harsher and should not be read as a field result.

## Realistic Spoof-Profile Suite

Run:

```powershell
cargo run --example run_realistic_spoof_suite -- artifacts/px4_hover_dataset.csv --dataset-label px4_hover
cargo run --example run_realistic_spoof_suite -- artifacts/px4_forward_dataset.csv --dataset-label px4_forward
cargo run --example run_realistic_spoof_suite -- artifacts/px4_turn_dataset.csv --dataset-label px4_turn
cargo run --example run_realistic_spoof_suite -- artifacts/px4_climb_dataset.csv --dataset-label px4_climb
```

Expected current behavior:

- Nominal verdicts stay `120/0/0` on each mission dataset.
- Abrupt takeover reaches `1.000` rejected TPR.
- SDR-style 30 m / 10 s takeover reaches about `0.894-0.914` rejected TPR.
- Hold-last-fix / frozen GPS reaches about `0.705-0.788` rejected TPR in this generated replay setup. Treat this as partial generated-replay coverage, not proof against real stale receiver outputs.
- Subtle generated phase-aligned time-push reaches about `0.692-0.762` rejected TPR, weaker than processed TEXBAT `ds7`.

The command writes CSV and JSON under `artifacts/spoof_suites`.

## Host Monitor Profiling

Run:

```powershell
cargo run --example profile_monitor_dataset -- artifacts/px4_monitor_dataset.csv --iterations 50 --json-output artifacts/px4_monitor_profile_report.json --acceptance-p95-us 10000 --acceptance-max-us 50000
```

Observed local output:

```text
Total monitor evaluations: 3000
Throughput: 3850.1 evaluations/s
Latency mean/p95/max per iteration (us): 258.98/269.86/1003.50
Final verdict counts: 60/0/0 trusted/flagged/rejected
Accepted: true
```

This profiles the Rust replay path on the development host and writes a structured JSON report. Running the same command on a representative companion computer or flight-controller-class Linux target is the intended next step. This still does not establish certified worst-case execution time under an autopilot scheduler.

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
  evidence chain root: aee3dce6be23e5ed8ff0674decc34769cab1579e06db539ac265257eb341db36
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
- [PRE_PHASE1_ASSESSMENT.md](PRE_PHASE1_ASSESSMENT.md)
