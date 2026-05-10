#!/usr/bin/env bash
set -u

workspace="/mnt/c/Users/saanvi/Documents/Codex/2026-05-10-build-what-this-thing-is-suggesting"
px4_build_dir="$workspace/external/PX4-Autopilot/build/px4_sitl_sih"

pkill -f "/bin/px4" >/dev/null 2>&1 || true

cd "$px4_build_dir" || exit 1
env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ./bin/px4 >/tmp/px4_inline.log 2>&1 &
px4pid=$!

sleep 6

cd "$workspace" || exit 1
. "$HOME/.cargo/env"

if [ "$#" -eq 0 ]; then
  set -- --connection udpout:127.0.0.1:18570 --event-limit 5
fi

timeout 15s target/debug/examples/mavlink_sniff \
  "$@" \
  > artifacts/wsl_inline_sniff_stdout.log \
  2> artifacts/wsl_inline_sniff_stderr.log || true

kill "$px4pid" >/dev/null 2>&1 || true

echo "---SNIFF STDOUT---"
cat artifacts/wsl_inline_sniff_stdout.log 2>/dev/null || true
echo "---SNIFF STDERR---"
cat artifacts/wsl_inline_sniff_stderr.log 2>/dev/null || true
echo "---PX4 LOG---"
sed -n '1,80p' /tmp/px4_inline.log 2>/dev/null || true
