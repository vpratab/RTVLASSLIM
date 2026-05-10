#!/usr/bin/env bash
set -u

workspace="/mnt/c/Users/saanvi/Documents/Codex/2026-05-10-build-what-this-thing-is-suggesting"
px4_build_dir="$workspace/external/PX4-Autopilot/build/px4_sitl_sih"

pkill -f "/bin/px4" >/dev/null 2>&1 || true

cd "$px4_build_dir" || exit 1
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ./bin/px4 >/tmp/px4_inline_live.log 2>&1 &
px4pid=$!

sleep 6

cd "$workspace" || exit 1
. "$HOME/.cargo/env"

timeout 25s target/debug/examples/px4_sitl_live \
  --connection udpout:127.0.0.1:18570 \
  --verdict-limit 3 \
  --evidence artifacts/wsl_px4_sitl_evidence.bin \
  > artifacts/wsl_px4_live_stdout.log \
  2> artifacts/wsl_px4_live_stderr.log || true

kill "$px4pid" >/dev/null 2>&1 || true

echo "---LIVE STDOUT---"
cat artifacts/wsl_px4_live_stdout.log 2>/dev/null || true
echo "---LIVE STDERR---"
cat artifacts/wsl_px4_live_stderr.log 2>/dev/null || true
echo "---EVIDENCE---"
ls -l artifacts/wsl_px4_sitl_evidence.bin 2>/dev/null || true
echo "---PX4 LOG---"
sed -n '1,80p' /tmp/px4_inline_live.log 2>/dev/null || true
