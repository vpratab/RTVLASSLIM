#!/usr/bin/env bash
set -u

workspace="/mnt/c/Users/saanvi/Documents/Codex/2026-05-10-build-what-this-thing-is-suggesting"
px4_build_dir="$workspace/external/PX4-Autopilot/build/px4_sitl_sih"
artifacts_dir="$workspace/artifacts"
px4_log="/tmp/px4_live_spoof.log"
immediate_gps_flag="${RTVLAS_IMMEDIATE_GPS_FLAG:-}"
immediate_gps_reject="${RTVLAS_IMMEDIATE_GPS_REJECT:-}"
immediate_position_flag="${RTVLAS_IMMEDIATE_POSITION_FLAG:-}"
immediate_position_reject="${RTVLAS_IMMEDIATE_POSITION_REJECT:-}"
horizontal_persistence_slack="${RTVLAS_HORIZONTAL_PERSISTENCE_SLACK:-}"
horizontal_persistence_reject="${RTVLAS_HORIZONTAL_PERSISTENCE_REJECT:-}"
disable_horizontal_persistence="${RTVLAS_DISABLE_HORIZONTAL_PERSISTENCE:-}"

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
  "$artifacts_dir/wsl_px4_live_spoof_evidence.bin" \
  "$artifacts_dir/wsl_px4_live_spoof_stdout.log" \
  "$artifacts_dir/wsl_px4_live_spoof_stderr.log" \
  "$artifacts_dir/wsl_px4_live_spoof_proxy.log" \
  "$artifacts_dir/wsl_px4_live_spoof_build.log"

cd "$workspace" || exit 1
. "$HOME/.cargo/env"

cargo build --examples > "$artifacts_dir/wsl_px4_live_spoof_build.log" 2>&1 || exit 1

cd "$px4_build_dir" || exit 1
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ./bin/px4 > "$px4_log" 2>&1 &
px4pid=$!

sleep 6

cd "$workspace" || exit 1

live_args=(
  --connection udpin:127.0.0.1:18571
  --skip-handshake
  --verdict-limit 30
  --evidence artifacts/wsl_px4_live_spoof_evidence.bin
)

if [ -n "$immediate_gps_flag" ]; then
  live_args+=(--immediate-gps-flag "$immediate_gps_flag")
fi
if [ -n "$immediate_gps_reject" ]; then
  live_args+=(--immediate-gps-reject "$immediate_gps_reject")
fi
if [ -n "$immediate_position_flag" ]; then
  live_args+=(--immediate-position-flag "$immediate_position_flag")
fi
if [ -n "$immediate_position_reject" ]; then
  live_args+=(--immediate-position-reject "$immediate_position_reject")
fi
if [ -n "$horizontal_persistence_slack" ]; then
  live_args+=(--horizontal-persistence-slack "$horizontal_persistence_slack")
fi
if [ -n "$horizontal_persistence_reject" ]; then
  live_args+=(--horizontal-persistence-reject "$horizontal_persistence_reject")
fi
if [ -n "$disable_horizontal_persistence" ]; then
  live_args+=(--disable-horizontal-persistence)
fi

timeout 25s target/debug/examples/px4_sitl_live \
  "${live_args[@]}" \
  > artifacts/wsl_px4_live_spoof_stdout.log \
  2> artifacts/wsl_px4_live_spoof_stderr.log &
livepid=$!

sleep 1

timeout 25s target/debug/examples/px4_spoof_proxy \
  --upstream udpout:127.0.0.1:18570 \
  --downstream udpout:127.0.0.1:18571 \
  --spoof-onset-s 1.5 \
  --north-offset-m 90 \
  --east-offset-m -50 \
  --down-offset-m 8 \
  --north-velocity-offset-mps 10 \
  --east-velocity-offset-mps -5 \
  --down-velocity-offset-mps 1 \
  > artifacts/wsl_px4_live_spoof_proxy.log \
  2>&1 &
proxypid=$!

wait "$livepid" || true

echo "---LIVE SPOOF STDOUT---"
cat artifacts/wsl_px4_live_spoof_stdout.log 2>/dev/null || true
echo "---LIVE SPOOF STDERR---"
cat artifacts/wsl_px4_live_spoof_stderr.log 2>/dev/null || true
echo "---PROXY LOG---"
cat artifacts/wsl_px4_live_spoof_proxy.log 2>/dev/null || true
echo "---EVIDENCE---"
ls -l artifacts/wsl_px4_live_spoof_evidence.bin 2>/dev/null || true
echo "---PX4 LOG---"
strings "$px4_log" 2>/dev/null | sed -n '1,40p' || true
