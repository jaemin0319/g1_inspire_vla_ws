#!/usr/bin/env bash
# Episode control without the hub window (same messages the hub sends).
#   scripts/record_episode.sh start --instruction "pick up the red cylinder" --source keyboard
#   scripts/record_episode.sh stop --outcome success|failure|undetermined
#   scripts/record_episode.sh abort | reset [--robot-pose X Y YAW] [--object-xy X Y] | mark --label L | status
#   scripts/record_episode.sh preview [episode_dir]   (contact sheet of the latest / given episode)
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros
if [[ "${1:-}" == "preview" ]]; then
  shift
  export PYTHONPATH="$G1IV_WS/src"
  exec "$(g1iv_py g1_inspire_sim)" -m g1iv_data.preview "$@"
fi
export PYTHONPATH="$G1IV_WS/src:$G1IV_XR_REPO"
cd "$G1IV_WS"
exec "$(g1iv_py g1_inspire_teleop)" -m g1iv_teleop.ctl "$@"
