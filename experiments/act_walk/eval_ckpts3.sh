#!/usr/bin/env bash
# Sequential closed-loop evaluation queue for act_g1_walk_far_v2 checkpoints (4060 sim + policy server). One instance only.
#   setsid nohup /home/jammin/g1iv_store/eval_ckpts3.sh 070000 030000,25,20,20261120 ... >> logs/eval_ckpts.log 2>&1 &
# Item = <step>[,<exec>[,<episodes>[,<seed>]]]
#   exec: 25 (= --n-action-steps 25, default), any N, "te" (temporal ensemble 0.01), "25s50" (25 walking / 50 still)
#   episodes default 5, seed default 20261118 (the runner default: same 5 start poses as all earlier evaluations)
# Waits until the checkpoint's training_state exists, restarts the policy server, runs far-start episodes,
# then prints eval_summary.py and fail_types.py for the folder.
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
RUN=/home/jammin/g1iv_store/train/act_g1_walk_far_v2
PY=$WS/.conda/envs/g1_inspire_data/bin/python
cd "$WS"
for item in "$@"; do
  IFS=, read -r s m n seed <<< "$item"
  m="${m:-25}"; n="${n:-5}"; seed="${seed:-20261118}"
  C="$RUN/checkpoints/$s"
  until [[ -d "$C/training_state" ]]; do sleep 60; done
  sleep 60
  while ps -eo cmd | grep -q "[t]ools/run_act_policy.py"; do sleep 30; done
  case "$m" in
    te) opt=(--temporal-ensemble 0.01) ;;
    *s*) opt=(--n-action-steps "${m%s*}" --n-action-steps-still "${m#*s}") ;;
    *) opt=(--n-action-steps "$m") ;;
  esac
  lab="v2_${s}_nas${m}"; [[ "$seed" != 20261118 ]] && lab="${lab}_seed${seed}"
  echo "=== $(date '+%F %T') checkpoint $s ${opt[*]} n=$n seed=$seed"
  scripts/run_act_eval.sh stop-server > /dev/null
  scripts/run_act_eval.sh server "$C/pretrained_model" "${opt[@]}" > /dev/null
  grep -o "'n_action_steps': [0-9]*, 'n_action_steps_still': [^,]*" logs/run/act_policy_server.log
  scripts/run_act_eval.sh run --mode far -n "$n" --seed "$seed" --label "$lab" --max-frames 900 \
    > "/home/jammin/g1iv_store/logs/eval_$lab.log" 2>&1 < /dev/null
  D=$(ls -d logs/policy_eval/*_${lab}_far | tail -1)
  "$PY" /home/jammin/g1iv_store/eval_summary.py "$D"
  "$PY" /home/jammin/g1iv_store/fail_types.py "$D"
done
echo "=== $(date '+%F %T') done"
