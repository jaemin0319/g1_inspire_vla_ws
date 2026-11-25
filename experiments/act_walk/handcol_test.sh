#!/usr/bin/env bash
# Hand <-> body collision + walking arm pose test (2026-11-24 user request: the hands passed through the thighs while
# walking). Runs the scripted walk -> grasp expert on the RTX 4060 with recording + observer video, in two configs:
#   new      hand_body_collision=true + reset_arm_q = walking pose (shoulders 0.17 rad wider)
#   control  hand_body_collision=true + official arm pose (shows the hands now hit the thighs instead of passing through)
# Same seed / starts in both. Needs the 4060 free: pause the v3 eval loop first (kill -STOP), resume it after.
#   handcol_test.sh [n_new=4] [n_control=2]
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
ST=/home/jammin/g1iv_store
OUT=$ST/handcol_test
TPY=$WS/.conda/envs/g1_inspire_teleop/bin/python
POSE="[0.35,0.35,0,0.87,0,0,0,0.35,-0.35,0,0.87,0,0,0]"
N_NEW=${1:-4} N_CTL=${2:-2}
# front view from above the desk (looking back at the walking robot): both hands and thighs visible while walking
CAM=(--scene-set "cameras.observer.pos=[1.60,0.30,1.20]" --scene-set "cameras.observer.look_at=[-0.60,-0.12,0.50]"
     --scene-set cameras.observer.width=1280 --scene-set cameras.observer.height=960)
cd "$WS"
export PYTHONNOUSERSITE=1
log() { echo "=== $(date '+%F %T') $*"; }
sim_alive() { [[ -f logs/run/sim.pid ]] && kill -0 "$(cat logs/run/sim.pid)" 2>/dev/null; }

run_cfg() {   # label n scene-sets...
  local lab=$1 n=$2; shift 2
  mkdir -p "$OUT/$lab/raw" "$OUT/$lab/video"
  scripts/stop_all.sh > /dev/null 2>&1; sleep 3
  log "$lab: simulator ($*)"
  # own memory-capped scope: a runaway process here cannot take down the training / other jobs (2026-11-24 OOM)
  G1IV_RECORD_OVERRIDES="[\"smoke_root=$OUT/$lab/raw\",\"raw_root=$OUT/$lab/raw\"]" G1IV_OBSERVER_VIDEO="$OUT/$lab/video" \
    setsid nohup systemd-run --user --scope --quiet -p MemoryMax=24G -p MemorySwapMax=0 \
    scripts/run_sim.sh --gpu 4060 --headless --scene-set desk.top_surface_height=0.65 \
    --scene-set extra_objects.enabled=false "${CAM[@]}" "$@" > "$OUT/$lab/sim.out" 2>&1 < /dev/null &
  sleep 8
  local ok=0
  for _ in $(seq 1 300); do
    grep -q "DDS project topics up" "$OUT/$lab/sim.out" 2>/dev/null && { ok=1; break; }
    grep -qE "Traceback|Error:" "$OUT/$lab/sim.out" 2>/dev/null && break
    sleep 2
  done
  grep -E "hand <-> body collision|reset arm pose" "$OUT/$lab/sim.out" | cut -c1-600
  (( ok )) || { log "$lab: simulator did not start"; tail -20 "$OUT/$lab/sim.out"; return 1; }
  sleep 15
  # demo episode: straight start 1.0 m behind, facing the desk (the video for the user)
  log "$lab: demo episode"
  "$TPY" tools/batch_walk_grasp.py -n 1 --record --desk 0.65 --plan configs/scripted_walk_wide_grasp_desk065.yaml \
    --seed 20261124 --robot-x -1.0 -1.0 --robot-y 0.0 0.0 --robot-yaw 0 0 --object-x 1.02 1.02 --object-y -0.20 -0.20 \
    --instruction "walk to the desk and pick up the red cylinder" --task-name walk_grasp_cylinder \
    --out "$OUT/$lab/batch_demo.jsonl" > "$OUT/$lab/batch_demo.log" 2>&1 < /dev/null
  log "$lab: demo exit $?"
  (( n > 0 )) || { scripts/stop_all.sh > /dev/null 2>&1; return 0; }
  log "$lab: $n scripted episodes"
  "$TPY" tools/batch_walk_grasp.py -n "$n" --record --desk 0.65 --plan configs/scripted_walk_wide_grasp_desk065.yaml \
    --seed 20261124 --robot-x -1.0 -0.4 --robot-y -1.0 1.0 --object-y -0.45 0.45 \
    --instruction "walk to the desk and pick up the red cylinder" --task-name walk_grasp_cylinder \
    --out "$OUT/$lab/batch.jsonl" > "$OUT/$lab/batch.log" 2>&1 < /dev/null
  log "$lab: batch exit $?"
  scripts/stop_all.sh > /dev/null 2>&1
}

run_cfg new "$N_NEW" --scene-set robot.hand_body_collision=true --scene-set "robot.reset_arm_q=$POSE"
(( N_CTL >= 0 )) && run_cfg control "$N_CTL" --scene-set robot.hand_body_collision=true
scripts/stop_all.sh > /dev/null 2>&1
log "done"
