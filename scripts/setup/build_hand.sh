#!/usr/bin/env bash
# Build the Inspire RH56E2 hand assets (docs/HAND_RH56E2.md) on this machine:
#   1) env g1_inspire_cad : Inspire URDF/STL (data/reference/inspire_ftp) -> data/assets/inspire_rh56e2/rh56e2_hands.npz
#   2) env g1_inspire_sim : official G1 + Inspire USD (sub-layer) + hand overrides -> configs/hand_rh56e2.yaml `usd`
# Needs scripts/setup/fetch_hand_reference.py, scripts/setup/fetch_assets.py, scripts/setup_envs.sh sim cad and the
# accepted NVIDIA EULA (step 2 starts Isaac Sim headless). Existing outputs are kept unless --force.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../_env_common.sh"
g1iv_strip_ros
cd "$G1IV_WS"
FORCE=0; [[ "${1:-}" == --force ]] && FORCE=1
REF=data/reference/inspire_ftp
NPZ=$(sed -n 's/^hands_npz: *//p' configs/hand_rh56e2.yaml)
USD=$(sed -n 's/^usd: *//p' configs/hand_rh56e2.yaml)
mkdir -p logs/setup "$(dirname "$NPZ")"
CAD=$(g1iv_py g1_inspire_cad); SIM=$(g1iv_py g1_inspire_sim)
[[ -x "$CAD" ]] || { echo "env g1_inspire_cad missing: scripts/setup_envs.sh cad" >&2; exit 1; }
[[ -x "$SIM" ]] || { echo "env g1_inspire_sim missing: scripts/setup_envs.sh sim" >&2; exit 1; }
[[ -d "$REF/rh56e2/R_2026_1_5" ]] || { echo "hand references missing: python3 scripts/setup/fetch_hand_reference.py" >&2; exit 1; }
if [[ $FORCE == 1 || ! -f "$NPZ" ]]; then
  CUDA_VISIBLE_DEVICES="" "$CAD" tools/cad/make_inspire_e2_hand.py "$REF/rh56e2/R_2026_1_5" "$NPZ" \
      --left-ref "$REF/rh56e2/L_2025_9_10" --unitree-ftp "$REF/unitree_ros" --preview logs/setup/rh56e2_preview.png
fi
echo "[hand] $NPZ $(sha256sum "$NPZ" | cut -c1-16)"
if [[ $FORCE == 1 || ! -f "$USD" ]]; then
  g1iv_eula
  g1iv_select_gpu
  "$SIM" tools/build_inspire_e2_usd.py
fi
echo "[hand] $USD $(sha256sum "$USD" | cut -c1-16)"
