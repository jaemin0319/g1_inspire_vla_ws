#!/usr/bin/env bash
# Mock XR controllers -> official televuer conversion -> IK -> sim. NOT a WebXR/PICO connection test.
#   scripts/run_mock_teleop.sh [dry_run|dropout|recenter|walk_xr|walk_seq|walk_arm|hold] [hub args]
#   scripts/run_mock_teleop.sh scripted [--record]     scripted right-hand grasp (ground-truth plan, smoke data; 0.70 m desk)
#   scripts/run_mock_teleop.sh scripted_walk [--desk 0.65] [--record]  walk to the desk, grasp + lift (ground truth);
#       plans: configs/scripted_walk_grasp_desk065.yaml / _desk070.yaml work, _desk055.yaml (0.55 m) does not yet
set -euo pipefail
SC=${1:-dry_run}; shift || true
H="$(dirname "${BASH_SOURCE[0]}")/_hub_launch.sh"
if [[ "$SC" == "scripted_walk" ]]; then   # walk -> stand -> grasp -> lift; the XR sticks drive the walking
  # plan = configs/scripted_walk_grasp_desk<cm>.yaml for --desk H (default: desk.top_surface_height of
  # configs/scene_desk.yaml); an explicit --plan wins. The plan checks the real desk height before the arm moves.
  DESK=""; ARGS=()
  while [[ $# -gt 0 ]]; do
    case "$1" in --desk) DESK=$2; shift 2 ;; *) ARGS+=("$1"); shift ;; esac
  done
  if [[ -z "$DESK" ]]; then
    DESK=$(sed -n '/^desk:/,/^[a-z]/s/^ *top_surface_height: *\([0-9.]*\).*/\1/p' "$(dirname "${BASH_SOURCE[0]}")/../configs/scene_desk.yaml" | head -1)
  fi
  PLAN=$(printf "configs/scripted_walk_grasp_desk%03d.yaml" "$(awk -v d="$DESK" 'BEGIN{printf "%d", d*100+0.5}')")
  exec "$H" scripted_walk --arm-source scripted --loco-source xr --plan "$PLAN" --exit-when-done ${ARGS[@]+"${ARGS[@]}"}
fi
if [[ "$SC" == "scripted" ]]; then
  exec "$H" scripted --arm-source scripted --loco-source none --exit-when-done "$@"
fi
LOCO=none; [[ "$SC" == walk_xr || "$SC" == walk_seq || "$SC" == walk_arm ]] && LOCO=xr
exec "$H" "mock_$SC" --arm-source mock --scenario "$SC" --loco-source "$LOCO" --exit-when-done "$@"
