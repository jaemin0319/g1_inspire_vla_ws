#!/usr/bin/env bash
# v3 pipeline (2026-11-23 user request): randomised starts against overfitting.
#   robot start x -1.0..-0.4, y -1.0..+1.0 (current start +/- 1 m sideways), yaw +/-15 deg;
#   cylinder x 0.98..1.08, y -0.45..+0.45 (20-80 % of the 1.5 m desk width centred at y 0)
#   -> scripted expert (configs/scripted_walk_wide_grasp_desk065.yaml, lateral_mode pursuit) with recording
#   -> stability filter (same limits as v2), first 100 passing episodes
#   -> LeRobot v2.1 -> ACT v3.0 dataset (loco loss weight 3, as v2)
#   -> ACT training with the v2 settings (batch 12, 100k, save every 10k, chunk 50, lr 1.5e-5, AMP, same augmentation)
#   -> closed-loop evaluation of every checkpoint: 30 episodes from the same 30 random starts (seed EVAL_SEED),
#      n_action_steps 25 (the setting adopted for v2) -> eval_v3/ summary (CSV/JSON/PNG)
# usage: v3_pipeline.sh collect|convert|train|eval|summary|all     (each stage skips work that is already done)
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
ST=/home/jammin/g1iv_store
RAW=$ST/raw/walk_wide_v3
SEL=$ST/raw/walk_wide_v3_sel
NAME=walk_wide_grasp_desk065_scripted_v3
V21=$ST/processed/${NAME}_lerobot_v21
V30=$ST/processed/${NAME}_act29_lw3_lerobot_v30
JOB=act_g1_walk_wide_v3
OUT=$ST/train/$JOB
LOGD=$ST/logs
TPY=$WS/.conda/envs/g1_inspire_teleop/bin/python
DPY=$WS/.conda/envs/g1_inspire_data/bin/python
LPY=/mnt/isaac/so101_ws/.venv-train/bin/python
N_KEEP=${N_KEEP:-100}
EVAL_N=${EVAL_N:-30}
EVAL_SEED=${EVAL_SEED:-20261124}
EVAL_FRAMES=${EVAL_FRAMES:-1000}
STEPS="010000 020000 030000 040000 050000 060000 070000 080000 090000 100000"
TRAIN_GPU=GPU-64746556-b38e-683b-cf44-fb33e351c65f       # RTX 5060 Ti
cd "$WS"
export PYTHONNOUSERSITE=1
unset G1IV_OBSERVER_VIDEO
log() { echo "=== $(date '+%F %T') $*"; }
sim_alive() { [[ -f logs/run/sim.pid ]] && kill -0 "$(cat logs/run/sim.pid)" 2>/dev/null; }
stop_sims() { scripts/run_act_eval.sh stop-server > /dev/null 2>&1; scripts/stop_all.sh > /dev/null 2>&1; sleep 5; }
wait_ready() {   # $1 = simulator stdout file
  for _ in $(seq 1 450); do
    grep -q "DDS project topics up" "$1" 2>/dev/null && { sleep 15; return 0; }
    sim_alive || return 1
    sleep 2
  done
  return 1
}
n_pass() {
  [[ -f $SEL/report.json ]] || { echo 0; return; }
  python3 -c "import json;r=json.load(open('$SEL/report.json'));print(sum(1 for e in r['episodes'] if not e['excluded']))"
}
start_collect_sim() {
  stop_sims
  G1IV_RECORD_OVERRIDES="[\"smoke_root=$RAW\"]" setsid nohup scripts/run_sim.sh --gpu 4060 --headless \
    --scene-set desk.top_surface_height=0.65 --scene-set extra_objects.enabled=false \
    > logs/run/collect_sim.out 2>&1 < /dev/null &
  sleep 5
  wait_ready logs/run/collect_sim.out
}

collect() {
  mkdir -p "$RAW" logs/checks/walk_wide
  local k=0 np need
  while :; do
    np=$(n_pass)
    log "collect: $np / $N_KEEP episodes pass the filter"
    (( np >= N_KEEP )) && break
    (( k >= 10 )) && { log "collect: giving up after $k rounds"; return 1; }
    sim_alive || start_collect_sim || { log "collect: simulator did not start"; k=$((k + 1)); continue; }
    need=$((N_KEEP - np))
    log "collect round $k: need $need more"
    "$TPY" tools/batch_walk_grasp.py -n $((need * 2 + 5)) --until-success $((need + need / 4 + 2)) --record --desk 0.65 \
      --plan configs/scripted_walk_wide_grasp_desk065.yaml --seed $((1123100 + k)) \
      --robot-x -1.0 -0.4 --robot-y -1.0 1.0 --object-y -0.45 0.45 \
      --instruction "walk to the desk and pick up the red cylinder" --task-name walk_grasp_cylinder \
      --out "logs/checks/walk_wide/batch_v3_$k.jsonl" >> "$LOGD/collect_walk_wide_v3.log" 2>&1 < /dev/null
    log "collect round $k: batch exit $?"
    "$DPY" tools/filter_walk_episodes.py "$RAW/*" --out-dir "$SEL" --limit "$N_KEEP" > "$LOGD/filter_walk_wide_v3.log" 2>&1
    tail -1 "$LOGD/filter_walk_wide_v3.log"
    k=$((k + 1))
  done
  stop_sims
}

convert() {
  if [[ ! -f $V21/meta/info.json ]]; then
    local paths
    paths=$(python3 -c "import json;r=json.load(open('$SEL/report.json'));print(' '.join(e['path'] for e in r['episodes'] if not e['excluded']))")
    log "convert: $(echo $paths | wc -w) episodes -> $V21"
    scripts/convert_dataset.sh --name "$NAME" --out-root "$ST/processed" --include-outcomes success --allow-smoke $paths \
      > "$LOGD/convert_walk_wide_v3.log" 2>&1 < /dev/null || { log "convert failed"; return 1; }
  fi
  if [[ ! -f $V30/meta/info.json ]]; then
    log "make_act_dataset -> $V30"
    HF_HUB_OFFLINE=1 "$LPY" tools/make_act_dataset.py "$V21" --loco-loss-weight 3 --out "$V30" \
      > "$LOGD/make_act_v3.log" 2>&1 < /dev/null || { log "make_act_dataset failed"; return 1; }
  fi
  log "convert done: $(python3 -c "import json;i=json.load(open('$V30/meta/info.json'));print(i['total_episodes'],'episodes',i['total_frames'],'frames')")"
}

train() {
  if [[ -d $OUT/checkpoints/100000/training_state ]]; then log "train: already finished"; return 0; fi
  [[ -d $OUT ]] && { log "train: $OUT exists but is unfinished; not touching it"; return 1; }
  stop_sims
  log "train: $JOB start"
  local TFS; TFS=$(tr -d '\n' < "$ST/aug_tfs.json")
  (cd /mnt/isaac/so101_ws && \
   SO101_DATASET=local/$(basename "$V30") SO101_JOB=$JOB SO101_WANDB=1 SO101_WANDB_PROJECT=g1_inspire_act \
   SO101_TRAIN_GPU=$TRAIN_GPU HF_HUB_OFFLINE=1 PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
   scripts/train_act.sh --dataset.root="$V30" --batch_size=12 --steps=100000 --save_freq=10000 \
     --policy.chunk_size=50 --policy.n_action_steps=50 --policy.optimizer_lr=1.5e-5 \
     --policy.optimizer_lr_backbone=1.5e-5 --policy.use_amp=true --dataset.image_transforms.enable=true \
     --dataset.image_transforms.max_num_transforms=3 "--dataset.image_transforms.tfs=$TFS" --output_dir="$OUT" \
     > "$ST/train/$JOB.log" 2>&1 < /dev/null)
  log "train: exit $?"
  [[ -d $OUT/checkpoints/100000/training_state ]]
}

eval_done() {   # $1 = step -> 0 if a folder with EVAL_N finished episodes exists
  local d
  for d in logs/policy_eval/*_v3_${1}_nas25_far; do
    [[ -f $d/summary.jsonl ]] && (( $(wc -l < "$d/summary.jsonl") >= EVAL_N )) && return 0
  done
  return 1
}
start_eval_sim() {
  stop_sims
  G1IV_POLICY_EVAL_ROOT=$ST/policy_eval scripts/run_act_eval.sh sim --headless > /dev/null
  sleep 5
  wait_ready logs/run/act_eval_sim.out
}
evaluate() {
  local s tries
  for s in $STEPS; do
    [[ -d $OUT/checkpoints/$s/pretrained_model ]] || { log "eval: no checkpoint $s"; continue; }
    tries=0
    until eval_done "$s" || (( tries >= 3 )); do
      tries=$((tries + 1))
      sim_alive || start_eval_sim || { log "eval: simulator did not start"; continue; }
      for d in logs/policy_eval/*_v3_${s}_nas25_far; do [[ -d $d ]] && mv "$d" "${d}_incomplete_$(date +%s)"; done
      scripts/run_act_eval.sh stop-server > /dev/null
      G1IV_POLICY_GPU=5060ti scripts/run_act_eval.sh server "$OUT/checkpoints/$s/pretrained_model" --n-action-steps 25 > /dev/null
      log "eval $s (try $tries): $EVAL_N episodes, seed $EVAL_SEED, max $EVAL_FRAMES frames"
      scripts/run_act_eval.sh run --mode far -n "$EVAL_N" --seed "$EVAL_SEED" --robot-y -1.0 1.0 --object-y -0.45 0.45 \
        --max-frames "$EVAL_FRAMES" --label "v3_${s}_nas25" > "$LOGD/eval_v3_$s.log" 2>&1 < /dev/null
      log "eval $s: runner exit $?, $(grep -c '"outcome": "success"' logs/policy_eval/*_v3_${s}_nas25_far/summary.jsonl 2>/dev/null | tail -1) successes"
    done
  done
  stop_sims
}

summary() {
  "$TPY" "$ST/eval_v3_summary.py" "$WS/logs/policy_eval" "$ST/eval_v3" "$EVAL_N"
}

case "${1:-all}" in
  collect) collect ;;
  convert) convert ;;
  train) train ;;
  eval) evaluate ;;
  summary) summary ;;
  all) collect && convert && train && evaluate && summary ;;
  *) sed -n 2,12p "$0"; exit 2 ;;
esac
log "pipeline ${1:-all} finished (exit $?)"
