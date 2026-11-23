#!/usr/bin/env bash
# Overfitting check (2026-11-23, user request): adopted v2 30k (n_action_steps 25) with the cylinder moved across the
# desk width 20-80 % (y -0.45..+0.45 m; v2 training data had y -0.30..-0.10), robot start = v2 distribution (--mode far).
# One episode per cylinder y (10 values), observer front video per episode (G1IV_OBSERVER_VIDEO set on the sim).
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
cd "$WS"
i=0
for y in -0.45 -0.35 -0.25 -0.15 -0.05 0.05 0.15 0.25 0.35 0.45; do
  seed=$((20261123 + i))
  echo "=== $(date '+%F %T') trial $i object_y $y seed $seed"
  scripts/run_act_eval.sh run --mode far -n 1 --seed "$seed" --object-y "$y" "$y" --max-frames 900 --hold-frames 50 \
    --label "overfit_t${i}_y${y}" < /dev/null
  i=$((i + 1))
done
echo "=== $(date '+%F %T') done"
