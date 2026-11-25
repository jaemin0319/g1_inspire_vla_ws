#!/usr/bin/env bash
# v4 pipeline (2026-11-24 user request): the v3 recipe again, now with the hand <-> body collision on and the walking
# arm pose (configs/scene_desk.yaml robot.hand_body_collision / reset_arm_q defaults since 2026-11-24; contact
# friction static/dynamic 1.0, checked from PhysX at start-up: "[g1iv] friction" line of the simulator log).
#   robot start x -1.0..-0.4, y -1.0..+1.0, yaw +/-15 deg; cylinder x 0.98..1.08, y -0.45..+0.45 (same seeds as v3)
#   -> scripted expert (configs/scripted_walk_wide_grasp_desk065.yaml) with recording, on the RTX 5060 Ti
#   -> stability filter (same limits as v2/v3), first 100 passing episodes
#   -> LeRobot v2.1 -> ACT v3.0 dataset (loco loss weight 3)
#   -> v4_hf_upload.sh in the background (public dataset like v1-v3) and, at the same time,
#      ACT training on the 5060 Ti (v3 settings: batch 12, chunk 50, lr 1.5e-5, AMP, augmentation; 200k, save every 20k)
#   evaluation of every checkpoint: v4_eval_ckpts.sh (4060, 18 episodes, videos), started separately.
# usage: v4_pipeline.sh sim|collect|convert|train|all     (each stage skips work that is already done)
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
ST=/home/jammin/g1iv_store
RAW=$ST/raw/walk_wide_v4
SEL=$ST/raw/walk_wide_v4_sel
NAME=walk_wide_grasp_desk065_scripted_v4
V21=$ST/processed/${NAME}_lerobot_v21
V30=$ST/processed/${NAME}_act29_lw3_lerobot_v30
JOB=act_g1_walk_wide_v4
OUT=$ST/train/$JOB
LOGD=$ST/logs
TPY=$WS/.conda/envs/g1_inspire_teleop/bin/python
DPY=$WS/.conda/envs/g1_inspire_data/bin/python
LPY=/mnt/isaac/so101_ws/.venv-train/bin/python
N_KEEP=${N_KEEP:-100}
COLLECT_GPU=${COLLECT_GPU:-5060ti}
TRAIN_STEPS=${TRAIN_STEPS:-200000}
SAVE_FREQ=${SAVE_FREQ:-20000}
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
start_collect_sim() {   # own memory-capped scope (2026-11-24 OOM): a runaway simulator cannot take other jobs down
  stop_sims
  G1IV_RECORD_OVERRIDES="[\"smoke_root=$RAW\"]" setsid nohup systemd-run --user --scope --quiet -p MemoryMax=24G \
    -p MemorySwapMax=0 scripts/run_sim.sh --gpu "$COLLECT_GPU" --headless \
    --scene-set desk.top_surface_height=0.65 --scene-set extra_objects.enabled=false \
    > logs/run/collect_sim.out 2>&1 < /dev/null &
  sleep 5
  wait_ready logs/run/collect_sim.out || return 1
  grep -E "hand <-> body collision on|reset arm pose|\[g1iv\] friction" logs/run/collect_sim.out | cut -c1-400
  grep -q "hand <-> body collision: True" logs/run/collect_sim.out || { log "hand collision not on; stopping"; stop_sims; return 1; }
}

collect() {
  mkdir -p "$RAW" logs/checks/walk_wide_v4
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
      --out "logs/checks/walk_wide_v4/batch_v4_$k.jsonl" >> "$LOGD/collect_walk_wide_v4.log" 2>&1 < /dev/null
    log "collect round $k: batch exit $?"
    "$DPY" tools/filter_walk_episodes.py "$RAW/*" --out-dir "$SEL" --limit "$N_KEEP" > "$LOGD/filter_walk_wide_v4.log" 2>&1
    tail -1 "$LOGD/filter_walk_wide_v4.log"
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
      > "$LOGD/convert_walk_wide_v4.log" 2>&1 < /dev/null || { log "convert failed"; return 1; }
  fi
  if [[ ! -f $V30/meta/info.json ]]; then
    log "make_act_dataset -> $V30"
    HF_HUB_OFFLINE=1 "$LPY" tools/make_act_dataset.py "$V21" --loco-loss-weight 3 --out "$V30" \
      > "$LOGD/make_act_v4.log" 2>&1 < /dev/null || { log "make_act_dataset failed"; return 1; }
  fi
  log "convert done: $(python3 -c "import json;i=json.load(open('$V30/meta/info.json'));print(i['total_episodes'],'episodes',i['total_frames'],'frames')")"
}

upload_bg() {
  log "upload: v4_hf_upload.sh in the background (log $LOGD/v4_hf_upload.log)"
  setsid nohup systemd-run --user --scope --quiet -p MemoryMax=12G -p MemorySwapMax=0 \
    bash "$ST/v4_hf_upload.sh" >> "$LOGD/v4_hf_upload.log" 2>&1 < /dev/null &
}

train() {
  if [[ -d $OUT/checkpoints/$TRAIN_STEPS/training_state ]]; then log "train: already finished"; return 0; fi
  [[ -d $OUT ]] && { log "train: $OUT exists but is unfinished; not touching it"; return 1; }
  stop_sims
  log "train: $JOB start ($TRAIN_STEPS steps, save every $SAVE_FREQ)"
  local TFS; TFS=$(tr -d '\n' < "$ST/aug_tfs.json")
  # own scope too: an OOM elsewhere (VS Code scope) no longer takes the training down
  (cd /mnt/isaac/so101_ws && \
   SO101_DATASET=local/$(basename "$V30") SO101_JOB=$JOB SO101_WANDB=1 SO101_WANDB_PROJECT=g1_inspire_act \
   SO101_WANDB_ENTITY=positive0mind- SO101_TRAIN_GPU=$TRAIN_GPU HF_HUB_OFFLINE=1 \
   PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
   systemd-run --user --scope --quiet -p MemoryMax=36G \
   scripts/train_act.sh --dataset.root="$V30" --batch_size=12 --steps="$TRAIN_STEPS" --save_freq="$SAVE_FREQ" \
     --policy.chunk_size=50 --policy.n_action_steps=50 --policy.optimizer_lr=1.5e-5 \
     --policy.optimizer_lr_backbone=1.5e-5 --policy.use_amp=true --dataset.image_transforms.enable=true \
     --dataset.image_transforms.max_num_transforms=3 "--dataset.image_transforms.tfs=$TFS" --output_dir="$OUT" \
     > "$ST/train/$JOB.log" 2>&1 < /dev/null)
  log "train: exit $?"
  [[ -d $OUT/checkpoints/$TRAIN_STEPS/training_state ]]
}

case "${1:-all}" in
  sim) sim_alive || start_collect_sim ;;
  collect) collect ;;
  convert) convert ;;
  train) train ;;
  all) collect && convert && { upload_bg; train; } ;;
  *) sed -n 2,12p "$0"; exit 2 ;;
esac
log "pipeline ${1:-all} finished (exit $?)"
