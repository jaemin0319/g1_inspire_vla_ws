#!/usr/bin/env bash
# Overnight closed-loop evaluation of act_g1_walk_far_v2 checkpoints as training writes them (4060 sim + policy server).
#   setsid nohup /home/jammin/g1iv_store/eval_ckpts.sh 020000 030000 ... > /home/jammin/g1iv_store/logs/eval_ckpts.log 2>&1 &
# Per checkpoint: wait until its training_state is written, restart the policy server on it (temporal ensemble 0.01),
# run N far-start episodes (same seed = same start poses for every checkpoint), then summarise with eval_summary.py.
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
RUN=/home/jammin/g1iv_store/train/act_g1_walk_far_v2
N="${EVAL_N:-5}"
cd "$WS"
for s in "$@"; do
  C="$RUN/checkpoints/$s"
  until [[ -d "$C/training_state" ]]; do sleep 60; done
  sleep 60
  # never overlap with another runner (e.g. a manual run)
  while ps -eo cmd | grep -q "[t]ools/run_act_policy.py"; do sleep 30; done
  echo "=== $(date '+%F %T') checkpoint $s"
  scripts/run_act_eval.sh stop-server
  scripts/run_act_eval.sh server "$C/pretrained_model" --temporal-ensemble 0.01
  scripts/run_act_eval.sh run --mode far -n "$N" --label "v2_$s" --max-frames 900 \
    > "/home/jammin/g1iv_store/logs/eval_v2_$s.log" 2>&1 < /dev/null
  D=$(ls -d logs/policy_eval/*_v2_${s}_far | tail -1)
  .conda/envs/g1_inspire_data/bin/python /home/jammin/g1iv_store/eval_summary.py "$D"
done
echo "=== $(date '+%F %T') done"
