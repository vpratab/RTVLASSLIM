# Scripts

These scripts are convenience wrappers around workflows that have already been exercised locally.

## Data Download

- `download_texbat_processed.ps1`
- `download_texbat_processed.sh`

Download the processed TEXBAT files used by the current replay harness.

## WSL PX4 Inspection

- `wsl_inline_sniff.sh`

Starts a local client against PX4 SIH traffic and prints monitor-relevant observations.

## WSL PX4 Nominal Run

- `wsl_inline_live.sh`

Runs the orchestrator live against PX4 SIH in WSL2 without spoof injection.

## WSL PX4 Replay Benchmark

- `wsl_px4_benchmark.sh`

Captures synchronized PX4 monitor samples, replays them nominally, then replays a software-injected spoof profile.

## WSL PX4 Live Spoof Path

- `wsl_px4_live_spoof.sh`

Starts PX4 SIH, launches the live MAVLink spoof proxy, runs the orchestrator downstream of that proxy, and prints the resulting mission report.

## Windows Convenience

- `run_px4_sitl_live.ps1`

Thin Windows-side convenience wrapper for the live PX4 example.
