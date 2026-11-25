#!/usr/bin/env bash
# Front-camera video of the v4 100k ACT checkpoint (2026-11-25 user request "10만스텝 모델 front카메라 시점 영상도 하나").
# Same camera as docs/ACT_EVAL.md section 6 (observer world_camera behind the desk, never a policy input):
#   pos [2.6,-0.55,1.5] look_at [0.25,-0.08,0.78], 1280x720, focal 16 mm, one mp4 per episode (G1IV_OBSERVER_VIDEO).
# Starts: the first 3 of the 18 evaluation starts (seed 20261124), n_action_steps 25, 4060.
# --hold-frames 50: success = lift >= 5 cm held for 2 s, so the video runs on 2 s after the lift.
# Pauses the v4 evaluation loop (SIGSTOP) while it runs and resumes it at the end (SIGCONT).
#   front_v4_100k.sh EVAL_LOOP_PID
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
ST=/home/jammin/g1iv_store
CK=$ST/train/act_g1_walk_wide_v4/checkpoints/100000/pretrained_model
VDIR=$ST/demo/front_video_v4_100k
LOOP=${1:?eval loop pid}
cd "$WS"
export PYTHONNOUSERSITE=1
log() { echo "=== $(date '+%F %T') $*"; }
resume() { scripts/run_act_eval.sh stop-server > /dev/null 2>&1; scripts/stop_all.sh > /dev/null 2>&1
           kill -CONT "$LOOP" 2>/dev/null && log "eval loop $LOOP resumed"; }
trap resume EXIT
kill -STOP "$LOOP" && log "eval loop $LOOP paused"
scripts/run_act_eval.sh stop-server > /dev/null 2>&1; scripts/stop_all.sh > /dev/null 2>&1; sleep 5
mkdir -p "$VDIR"
G1IV_GPU=4060 G1IV_OBSERVER_VIDEO=$VDIR G1IV_POLICY_EVAL_ROOT=$ST/policy_eval \
  systemd-run --user --scope --quiet -p MemoryMax=24G -p MemorySwapMax=0 \
  scripts/run_act_eval.sh sim --headless --scene-set "cameras.observer.pos=[2.6,-0.55,1.5]" \
  --scene-set "cameras.observer.look_at=[0.25,-0.08,0.78]" --scene-set cameras.observer.width=1280 \
  --scene-set cameras.observer.height=720 --scene-set cameras.observer.focal_length=16.0 > /dev/null
sleep 5
ok=0
for _ in $(seq 1 450); do
  grep -q "DDS project topics up" logs/run/act_eval_sim.out 2>/dev/null && { ok=1; break; }
  [[ -f logs/run/sim.pid ]] && kill -0 "$(cat logs/run/sim.pid)" 2>/dev/null || break
  sleep 2
done
(( ok )) || { log "simulator did not start"; tail -20 logs/run/act_eval_sim.out; exit 1; }
grep -E "hand <-> body collision|reset arm pose|observer" logs/run/act_eval_sim.out | cut -c1-300
sleep 15
G1IV_POLICY_GPU=4060 systemd-run --user --scope --quiet -p MemoryMax=8G -p MemorySwapMax=0 \
  scripts/run_act_eval.sh server "$CK" --n-action-steps 25
log "run: 3 episodes"
scripts/run_act_eval.sh run --mode far -n 3 --seed 20261124 --robot-y -1.0 1.0 --object-y -0.45 0.45 \
  --max-frames 1000 --hold-frames 50 --label front_v4_100k < /dev/null
log "runner exit $?"
ls -la "$VDIR"
