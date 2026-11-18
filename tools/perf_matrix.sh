#!/usr/bin/env bash
# Headless performance matrix of the project desk task (robot standing, no hub): RTF, control rate, GPU memory, CPU.
# Writes logs/checks/perf_<label>.json + a summary table to stdout. Stops each simulator it starts.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
WS=$PWD
E="env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONPATH=$WS/src $WS/.conda/envs/g1_inspire_teleop/bin/python"
run() {
  local label=$1; shift
  echo "=== $label: scripts/run_sim.sh --headless $*"
  scripts/run_sim.sh --headless "$@" > "logs/checks/perf_${label}_console.txt" 2>&1 &
  sleep 3
  local F; F=$(ls -t logs/sim/sim_*.log | head -1)
  local t0=$SECONDS
  until grep -qE "start controller success|Failed to" "$F" 2>/dev/null; do
    sleep 1; (( SECONDS - t0 > 300 )) && { echo "  timeout"; scripts/stop_all.sh; return; }
  done
  local startup=$((SECONDS - t0))
  sleep 12
  $E -m g1iv_teleop.status_log --seconds 20 --out "logs/checks/perf_${label}_status.json" 2>&1 | grep -v multicast
  local pid; pid=$(cat logs/run/sim.pid)
  local cpu; cpu=$(ps -o %cpu= -p "$pid" | tr -d ' ')
  local gpu; gpu=$(nvidia-smi --query-gpu=name,memory.used,utilization.gpu --format=csv,noheader | tr '\n' ';')
  python3 - "$label" "$startup" "$cpu" "$gpu" <<'EOF'
import json, sys, numpy as np
label, startup, cpu, gpu = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
rows = json.load(open(f"logs/checks/perf_{label}_status.json"))
st = np.array([r["sim_time"] for r in rows]); wt = np.array([r["recv_wall"] for r in rows])
rtf = float((st[-1] - st[0]) / (wt[-1] - wt[0])) if len(rows) > 1 else float("nan")
out = {"label": label, "startup_s_to_controller": startup, "rtf": round(rtf, 3), "control_hz_wall": round(rtf * 50, 1),
       "fallen": any(r["fallen"] for r in rows), "sim_process_cpu_percent": cpu, "gpus": gpu}
json.dump(out, open(f"logs/checks/perf_{label}.json", "w"), indent=1)
print("  ", out)
EOF
  scripts/stop_all.sh > /dev/null
  sleep 3
}
[[ "${1:-}" == only_cuda ]] || run 4060_cams3
[[ "${1:-}" == only_cuda ]] || run 5060ti_cams3 --gpu 5060ti
[[ "${1:-}" == only_cuda ]] || run 4060_cams0 -- --camera_exclude front_camera,left_wrist_camera,right_wrist_camera,world_camera
[[ "${1:-}" == only_cuda ]] || run 4060_norender -- --no_render
run 4060_cams3_gpu_physics -- --device cuda:0   # experiment: GPU PhysX instead of the official --device cpu
echo "=== done"
