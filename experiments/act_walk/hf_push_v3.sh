#!/usr/bin/env bash
# 2026-11-24 user request "로컬 모델 업로드 확인 후 삭제": v3 checkpoints (not on the Hub yet) -> private repo, like v2/ep15
# with the so101 hub sync tool (official push_checkpoint_to_hub layout checkpoints/<step>/ + step tags, remote hash check).
set -uo pipefail
WS=/mnt/isaac/so101_ws
cd "$WS"
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH VIRTUAL_ENV CONDA_PREFIX || true
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1 TMPDIR="$WS/.tmp"
export HF_HUB_CACHE="$WS/.cache/huggingface/hub" HF_DATASETS_CACHE="$WS/.cache/huggingface/datasets"
export HF_LEROBOT_HOME="$WS/.cache/huggingface/lerobot" HF_XET_CACHE="$WS/.cache/huggingface/xet"
export HF_HUB_DISABLE_PROGRESS_BARS=1
mkdir -p "$TMPDIR"
for pair in "/home/jammin/g1iv_store/train/act_g1_walk_wide_v3:jammin0319/act_g1_walk_wide_v3"; do
  out="${pair%%:*}"; repo="${pair##*:}"
  echo "=== $(date '+%F %T') $out -> $repo"
  "$WS/.venv-train/bin/python" "$WS/tools/hub_sync_checkpoints.py" --output-dir "$out" --repo-id "$repo" --private --once
  echo "=== exit $?"
done
echo "=== $(date '+%F %T') all done"
