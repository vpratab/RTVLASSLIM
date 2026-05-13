#!/usr/bin/env bash
set -u

workspace="/mnt/c/Users/saanvi/Documents/Codex/2026-05-10-build-what-this-thing-is-suggesting"
px4_build_dir="$workspace/external/PX4-Autopilot/build/px4_sitl_sih"
artifacts_dir="$workspace/artifacts"
px4_log="/tmp/px4_gradual_spoof.log"

cleanup() {
  if [ -n "${livepid:-}" ]; then
    kill "$livepid" >/dev/null 2>&1 || true
  fi
  if [ -n "${proxypid:-}" ]; then
    kill "$proxypid" >/dev/null 2>&1 || true
  fi
  if [ -n "${px4pid:-}" ]; then
    kill "$px4pid" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

pkill -f "/bin/px4" >/dev/null 2>&1 || true
mkdir -p "$artifacts_dir"
rm -f \
  "$artifacts_dir/wsl_px4_gradual_spoof_evidence.bin" \
  "$artifacts_dir/wsl_px4_gradual_spoof_stdout.log" \
  "$artifacts_dir/wsl_px4_gradual_spoof_stderr.log" \
  "$artifacts_dir/wsl_px4_gradual_spoof_proxy.log" \
  "$artifacts_dir/wsl_px4_gradual_spoof_build.log"

cd "$workspace" || exit 1
. "$HOME/.cargo/env"

cargo build --examples > "$artifacts_dir/wsl_px4_gradual_spoof_build.log" 2>&1 || exit 1

cd "$px4_build_dir" || exit 1
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ./bin/px4 > "$px4_log" 2>&1 &
px4pid=$!

sleep 6

cd "$workspace" || exit 1

timeout 25s target/debug/examples/px4_sitl_live \
  --connection udpin:127.0.0.1:18571 \
  --skip-handshake \
  --verdict-limit 45 \
  --evidence artifacts/wsl_px4_gradual_spoof_evidence.bin \
  > artifacts/wsl_px4_gradual_spoof_stdout.log \
  2> artifacts/wsl_px4_gradual_spoof_stderr.log &
livepid=$!

sleep 1

timeout 25s target/debug/examples/px4_spoof_proxy \
  --upstream udpout:127.0.0.1:18570 \
  --downstream udpout:127.0.0.1:18571 \
  --spoof-onset-s 1.5 \
  --position-ramp-duration-s 2.5 \
  --north-offset-m 30 \
  --east-offset-m 0 \
  --down-offset-m 0 \
  --north-velocity-offset-mps 0 \
  --east-velocity-offset-mps 0 \
  --down-velocity-offset-mps 0 \
  > artifacts/wsl_px4_gradual_spoof_proxy.log \
  2>&1 &
proxypid=$!

wait "$livepid" || true

echo "---GRADUAL SPOOF STDOUT---"
cat artifacts/wsl_px4_gradual_spoof_stdout.log 2>/dev/null || true
echo "---GRADUAL SPOOF STDERR---"
cat artifacts/wsl_px4_gradual_spoof_stderr.log 2>/dev/null || true
echo "---PROXY LOG---"
cat artifacts/wsl_px4_gradual_spoof_proxy.log 2>/dev/null || true
echo "---EVIDENCE---"
ls -l artifacts/wsl_px4_gradual_spoof_evidence.bin 2>/dev/null || true
echo "---PX4 LOG---"
strings "$px4_log" 2>/dev/null | sed -n '1,40p' || true
