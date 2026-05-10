#!/usr/bin/env bash
set -u

workspace="/mnt/c/Users/saanvi/Documents/Codex/2026-05-10-build-what-this-thing-is-suggesting"
px4_build_dir="$workspace/external/PX4-Autopilot/build/px4_sitl_sih"
samples="${1:-60}"

mkdir -p "$workspace/artifacts"
rm -f "$workspace/artifacts/px4_monitor_dataset.csv" \
      "$workspace/artifacts/px4_monitor_dataset_spoofed.csv" \
      "$workspace/artifacts/wsl_px4_benchmark.log"

pkill -f "/bin/px4" >/dev/null 2>&1 || true

cd "$workspace" || exit 1
. "$HOME/.cargo/env"
cargo build --example capture_monitor_dataset --example run_monitor_benchmark >/tmp/rtvlas_wsl_benchmark_build.log 2>&1 || {
  cat /tmp/rtvlas_wsl_benchmark_build.log
  exit 1
}

cd "$px4_build_dir" || exit 1
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ./bin/px4 >/tmp/px4_benchmark.log 2>&1 &
px4pid=$!

sleep 6

cd "$workspace" || exit 1
target/debug/examples/capture_monitor_dataset \
  --connection udpout:127.0.0.1:18570 \
  --samples "$samples" \
  --output artifacts/px4_monitor_dataset.csv \
  > artifacts/wsl_px4_capture_stdout.log \
  2> artifacts/wsl_px4_capture_stderr.log
capture_status=$?

kill "$px4pid" >/dev/null 2>&1 || true

if [ "$capture_status" -ne 0 ]; then
  echo "---CAPTURE STDOUT---"
  cat artifacts/wsl_px4_capture_stdout.log 2>/dev/null || true
  echo "---CAPTURE STDERR---"
  cat artifacts/wsl_px4_capture_stderr.log 2>/dev/null || true
  exit "$capture_status"
fi

target/debug/examples/run_monitor_benchmark \
  artifacts/px4_monitor_dataset.csv \
  artifacts/px4_monitor_dataset_spoofed.csv \
  --onset 3.0 \
  --ramp 1.0 \
  > artifacts/wsl_px4_benchmark.log \
  2>&1

echo "---CAPTURE STDOUT---"
cat artifacts/wsl_px4_capture_stdout.log 2>/dev/null || true
echo "---BENCHMARK---"
cat artifacts/wsl_px4_benchmark.log 2>/dev/null || true
echo "---FILES---"
ls -l artifacts/px4_monitor_dataset.csv artifacts/px4_monitor_dataset_spoofed.csv 2>/dev/null || true
