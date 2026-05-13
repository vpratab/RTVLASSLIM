#!/usr/bin/env bash
set -u

workspace="/mnt/c/Users/saanvi/Documents/Codex/2026-05-10-build-what-this-thing-is-suggesting"
px4_build_dir="$workspace/external/PX4-Autopilot/build/px4_sitl_sih"
artifacts_dir="$workspace/artifacts"
samples="${1:-240}"
profiles=(hover forward turn climb)

cleanup() {
  if [ -n "${px4pid:-}" ]; then
    kill "$px4pid" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

mkdir -p "$artifacts_dir/sweeps"
rm -f "$artifacts_dir/px4_multi_mission_summary.csv"

cd "$workspace" || exit 1
. "$HOME/.cargo/env"

cargo build --example capture_monitor_dataset --example run_monitor_benchmark --example run_adversarial_sweep >/tmp/rtvlas_multi_mission_build.log 2>&1 || {
  cat /tmp/rtvlas_multi_mission_build.log
  exit 1
}

echo "mission,nominal_verdicts,nominal_anomaly_fpr,nominal_rejected_fpr,standard_spoof_verdicts,standard_spoof_anomaly_tpr,standard_spoof_rejected_tpr,sweep_csv,sweep_json" > "$artifacts_dir/px4_multi_mission_summary.csv"

for profile in "${profiles[@]}"; do
  dataset_path="$artifacts_dir/px4_${profile}_dataset.csv"
  spoofed_path="$artifacts_dir/px4_${profile}_dataset_spoofed.csv"
  capture_log="$artifacts_dir/px4_${profile}_capture.log"
  benchmark_log="$artifacts_dir/px4_${profile}_benchmark.log"
  sweep_log="$artifacts_dir/px4_${profile}_sweep.log"
  sweep_csv="$artifacts_dir/sweeps/px4_${profile}_adversarial_sweep.csv"
  sweep_json="$artifacts_dir/sweeps/px4_${profile}_adversarial_sweep.json"
  px4_log="/tmp/px4_${profile}_mission.log"

  rm -f "$dataset_path" "$spoofed_path" "$capture_log" "$benchmark_log" "$sweep_log" "$sweep_csv" "$sweep_json"
  pkill -f "/bin/px4" >/dev/null 2>&1 || true

  cd "$px4_build_dir" || exit 1
  env PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ./bin/px4 > "$px4_log" 2>&1 &
  px4pid=$!
  sleep 6

  cd "$workspace" || exit 1
  target/debug/examples/capture_monitor_dataset \
    --connection udpout:127.0.0.1:18570 \
    --samples "$samples" \
    --mission-profile "$profile" \
    --output "$dataset_path" \
    > "$capture_log" 2>&1
  capture_status=$?

  kill "$px4pid" >/dev/null 2>&1 || true
  unset px4pid

  if [ "$capture_status" -ne 0 ]; then
    echo "capture failed for $profile"
    cat "$capture_log" 2>/dev/null || true
    exit "$capture_status"
  fi

  target/debug/examples/run_monitor_benchmark \
    "$dataset_path" \
    "$spoofed_path" \
    --onset 6.0 \
    --ramp 5.0 \
    > "$benchmark_log" 2>&1

  target/debug/examples/run_adversarial_sweep \
    "$dataset_path" \
    --dataset-label "px4_${profile}" \
    --output-dir "$artifacts_dir/sweeps" \
    > "$sweep_log" 2>&1

  nominal_verdicts=$(awk '/Nominal dataset:/{flag=1;next}/Spoofed dataset:/{flag=0} flag&&/trusted\/flagged\/rejected:/{sub(/.*: /,""); print; exit}' "$benchmark_log")
  nominal_anomaly_fpr=$(awk '/Nominal dataset:/{flag=1;next}/Spoofed dataset:/{flag=0} flag&&/anomaly FPR:/{print $3; exit}' "$benchmark_log")
  nominal_rejected_fpr=$(awk '/Nominal dataset:/{flag=1;next}/Spoofed dataset:/{flag=0} flag&&/rejected FPR:/{print $3; exit}' "$benchmark_log")
  standard_spoof_verdicts=$(awk '/Spoofed dataset:/{flag=1;next} flag&&/trusted\/flagged\/rejected:/{sub(/.*: /,""); print; exit}' "$benchmark_log")
  standard_spoof_anomaly_tpr=$(awk '/Spoofed dataset:/{flag=1;next} flag&&/anomaly TPR\/FPR:/{split($3,parts,"/"); print parts[1]; exit}' "$benchmark_log")
  standard_spoof_rejected_tpr=$(awk '/Spoofed dataset:/{flag=1;next} flag&&/rejected TPR\/FPR:/{split($3,parts,"/"); print parts[1]; exit}' "$benchmark_log")

  echo "$profile,$nominal_verdicts,$nominal_anomaly_fpr,$nominal_rejected_fpr,$standard_spoof_verdicts,$standard_spoof_anomaly_tpr,$standard_spoof_rejected_tpr,$sweep_csv,$sweep_json" >> "$artifacts_dir/px4_multi_mission_summary.csv"

  echo "---MISSION $profile CAPTURE---"
  cat "$capture_log" 2>/dev/null || true
  echo "---MISSION $profile BENCHMARK---"
  cat "$benchmark_log" 2>/dev/null || true
  echo "---MISSION $profile SWEEP---"
  cat "$sweep_log" 2>/dev/null || true
done

echo "---MULTI-MISSION SUMMARY---"
cat "$artifacts_dir/px4_multi_mission_summary.csv"
