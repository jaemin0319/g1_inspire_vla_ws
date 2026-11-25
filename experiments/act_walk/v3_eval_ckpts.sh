#!/usr/bin/env bash
# v3 checkpoint evaluation WHILE the 5060 Ti keeps training to 200k (2026-11-23 user request):
#   every 20k checkpoint -> 18 closed-loop episodes on the RTX 4060 (simulator + policy server), n_action_steps 25,
#   the same 18 random starts for every checkpoint (seed EVAL_SEED; robot x -1.0..-0.4, y -1..+1, yaw +/-15 deg;
#   cylinder x 0.98..1.08, y -0.45..+0.45), max EVAL_FRAMES frames
#   -> video ~/다운로드/g1_act_v3_eval/g1_act_v3_<k>k_eval18.mp4 (two 3x3 head-camera grids, O/X marks, success rates)
#   -> data /home/jammin/g1iv_store/eval_v3/ (CSV/JSON/PNG, updated after every checkpoint)
# usage: setsid nohup v3_eval_ckpts.sh >> /home/jammin/g1iv_store/logs/v3_eval_ckpts.log 2>&1 < /dev/null &
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
ST=/home/jammin/g1iv_store
OUT=$ST/train/act_g1_walk_wide_v3
TPY=$WS/.conda/envs/g1_inspire_teleop/bin/python
VID=/home/jammin/다운로드/g1_act_v3_eval
N=18
EVAL_SEED=${EVAL_SEED:-20261124}
EVAL_FRAMES=${EVAL_FRAMES:-1000}
STEPS="020000 040000 060000 080000 100000 120000 140000 160000 180000 200000"
cd "$WS"
export PYTHONNOUSERSITE=1
unset G1IV_OBSERVER_VIDEO
log() { echo "=== $(date '+%F %T') $*"; }
sim_alive() { [[ -f logs/run/sim.pid ]] && kill -0 "$(cat logs/run/sim.pid)" 2>/dev/null; }
start_sim() {
  scripts/run_act_eval.sh stop-server > /dev/null 2>&1; scripts/stop_all.sh > /dev/null 2>&1; sleep 5
  G1IV_GPU=4060 G1IV_POLICY_EVAL_ROOT=$ST/policy_eval scripts/run_act_eval.sh sim --headless > /dev/null
  sleep 5
  for _ in $(seq 1 450); do
    grep -q "DDS project topics up" logs/run/act_eval_sim.out 2>/dev/null && { sleep 15; return 0; }
    sim_alive || return 1
    sleep 2
  done
  return 1
}
folder() { ls -d logs/policy_eval/*_v3_${1}_nas25_far 2>/dev/null | grep -v _incomplete_ | tail -1; }
complete() { local d; d=$(folder "$1"); [[ -n $d && -f $d/summary.jsonl ]] && (( $(wc -l < "$d/summary.jsonl") >= N )); }

mkdir -p "$VID"
for s in $STEPS; do
  C=$OUT/checkpoints/$s
  if ! complete "$s"; then
    log "waiting for checkpoint $s"
    until [[ -f $C/training_state/training_step.json && -f $C/pretrained_model/model.safetensors ]]; do sleep 60; done
    sleep 90                                            # let the trainer finish writing and switch 'last'
    tries=0
    until complete "$s" || (( tries >= 3 )); do
      tries=$((tries + 1))
      sim_alive || start_sim || { log "simulator did not start (try $tries)"; continue; }
      d=$(folder "$s"); [[ -n $d ]] && mv "$d" "${d}_incomplete_$(date +%s)"
      scripts/run_act_eval.sh stop-server > /dev/null
      G1IV_POLICY_GPU=4060 scripts/run_act_eval.sh server "$C/pretrained_model" --n-action-steps 25 > /dev/null
      log "eval $s (try $tries): $N episodes on the 4060, seed $EVAL_SEED, max $EVAL_FRAMES frames"
      scripts/run_act_eval.sh run --mode far -n "$N" --seed "$EVAL_SEED" --robot-y -1.0 1.0 --object-y -0.45 0.45 \
        --max-frames "$EVAL_FRAMES" --label "v3_${s}_nas25" > "$ST/logs/eval_v3_$s.log" 2>&1 < /dev/null
      log "eval $s: runner exit $?"
    done
    scripts/run_act_eval.sh stop-server > /dev/null
  fi
  d=$(folder "$s")
  if complete "$s"; then
    k=$((10#$s / 1000))
    log "eval $s: $(grep -c '"outcome": "success"' "$d/summary.jsonl") / $N successes"
    "$TPY" "$ST/eval_grid18.py" "$WS/$d" "$ST/policy_eval" "$VID/g1_act_v3_${k}k_eval18.mp4" 2>&1 | tail -1
    "$TPY" "$ST/eval_v3_summary.py" "$WS/logs/policy_eval" "$ST/eval_v3" "$N" 2>&1 | tail -2
    cp -f "$ST/eval_v3/checkpoints.csv" "$VID/success_rate_by_checkpoint.csv" 2>/dev/null
    cp -f "$ST/eval_v3/success_rate.png" "$VID/success_rate_by_checkpoint.png" 2>/dev/null
  else
    log "eval $s: incomplete after $tries tries"
  fi
done
scripts/run_act_eval.sh stop-server > /dev/null 2>&1; scripts/stop_all.sh > /dev/null 2>&1
log "all checkpoints done"
