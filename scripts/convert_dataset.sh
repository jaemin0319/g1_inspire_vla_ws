#!/usr/bin/env bash
# Raw episodes -> LeRobot v2.1 (+ GR00T modality.json/stats.json), then load one batch with the pinned loader.
#   scripts/convert_dataset.sh --name g1iv_pick_v1 data/raw/20261111 [--include-outcomes success] [--allow-smoke]
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros
export PYTHONPATH="$G1IV_WS/src" CUDA_VISIBLE_DEVICES=""
cd "$G1IV_WS"
PY=$(g1iv_py g1_inspire_data)
[[ -x "$PY" ]] || { echo "env g1_inspire_data missing: scripts/setup_envs.sh data" >&2; exit 1; }
"$PY" -m g1iv_data.convert_lerobot "$@"
NAME=$(sed -n 's/.*--name[ =]\([^ ]*\).*/\1/p' <<<"$*")
[[ -n "$NAME" ]] && "$PY" -m g1iv_data.check_loader "data/processed/${NAME}_lerobot_v21" --report "logs/validate/loader_${NAME}.json"
