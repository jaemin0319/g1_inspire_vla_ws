#!/usr/bin/env bash
# Closed-loop evaluation of act_g1_walk_far_v2 checkpoints (4060 sim + policy server), chunked execution.
#   setsid nohup /home/jammin/g1iv_store/eval_ckpts2.sh 050000 040000@50 060000 ... >> /home/jammin/g1iv_store/logs/eval_ckpts.log 2>&1 &
# Item = <step>[@<n_action_steps>|@te]; default n_action_steps $EVAL_NAS (25 = replan every 1 s of the 2 s chunk).
# 2026-11-20: 40k with temporal ensemble 0.01 = 3/5, |dy| 5.4 cm; with n_action_steps 25 = 4/5, |dy| 2.4 cm.
# Per item: wait until the checkpoint's training_state exists, restart the policy server, run N far-start episodes
# (default seed = same start poses for every checkpoint), summarise with eval_summary.py.
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
RUN=/home/jammin/g1iv_store/train/act_g1_walk_far_v2
N="${EVAL_N:-5}"
NAS="${EVAL_NAS:-25}"
cd "$WS"
for item in "$@"; do
  s="${item%@*}"; m="$NAS"; [[ "$item" == *@* ]] && m="${item#*@}"
  C="$RUN/checkpoints/$s"
  until [[ -d "$C/training_state" ]]; do sleep 60; done
  sleep 60
  while ps -eo cmd | grep -q "[t]ools/run_act_policy.py"; do sleep 30; done
  if [[ "$m" == te ]]; then opt=(--temporal-ensemble 0.01); lab="v2_${s}_te"; else opt=(--n-action-steps "$m"); lab="v2_${s}_nas$m"; fi
  echo "=== $(date '+%F %T') checkpoint $s ${opt[*]}"
  scripts/run_act_eval.sh stop-server
  scripts/run_act_eval.sh server "$C/pretrained_model" "${opt[@]}" > /dev/null
  grep -o "'n_action_steps': [0-9]*, 'temporal_ensemble_coeff': [^,]*" logs/run/act_policy_server.log
  scripts/run_act_eval.sh run --mode far -n "$N" --label "$lab" --max-frames 900 \
    > "/home/jammin/g1iv_store/logs/eval_$lab.log" 2>&1 < /dev/null
  D=$(ls -d logs/policy_eval/*_${lab}_far | tail -1)
  .conda/envs/g1_inspire_data/bin/python /home/jammin/g1iv_store/eval_summary.py "$D"
done
echo "=== $(date '+%F %T') done"
