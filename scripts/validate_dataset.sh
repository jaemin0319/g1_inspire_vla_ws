#!/usr/bin/env bash
# Validate raw episodes (default: everything under data/raw and data/smoke) or a converted dataset.
#   scripts/validate_dataset.sh [paths...]
#   scripts/validate_dataset.sh --lerobot data/processed/<name>_lerobot_v21
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros
export PYTHONPATH="$G1IV_WS/src"
cd "$G1IV_WS"
mkdir -p logs/validate
if [[ "${1:-}" == "--lerobot" ]]; then
  exec "$(g1iv_py g1_inspire_data)" -m g1iv_data.check_loader "$2" --report "logs/validate/loader_$(basename "$2")_$(date +%H%M%S).json"
fi
PY=$(g1iv_py g1_inspire_data); [[ -x "$PY" ]] || PY=$(g1iv_py g1_inspire_sim)
P=("$@"); [[ ${#P[@]} -eq 0 ]] && P=(data/raw data/smoke)
exec "$PY" -m g1iv_data.validate "${P[@]}" --report "logs/validate/raw_$(date +%Y%m%d_%H%M%S).json"
