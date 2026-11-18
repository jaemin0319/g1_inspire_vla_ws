#!/usr/bin/env bash
# internal: start the teleop hub in g1_inspire_teleop with a PID file. Usage: _hub_launch.sh <pidname> <hub args...>
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros
NAME=$1; shift
if g1iv_pid_alive hub; then echo "a hub is already running (pid $(cat "$G1IV_WS/logs/run/hub.pid")): one owner per command topic" >&2; exit 1; fi
PY=$(g1iv_py g1_inspire_teleop)
[[ -x "$PY" ]] || { echo "env g1_inspire_teleop missing: scripts/setup_envs.sh teleop" >&2; exit 1; }
g1iv_certs
export PYTHONPATH="$G1IV_WS/src:$G1IV_XR_REPO:$G1IV_XR_REPO/teleop/televuer/src:$G1IV_XR_REPO/teleop/teleimager/src"
export CUDA_VISIBLE_DEVICES=""              # the hub needs no GPU
mkdir -p "$G1IV_WS/logs/hub"
LOG="$G1IV_WS/logs/hub/${NAME}_$(date +%Y%m%d_%H%M%S).log"
cd "$G1IV_WS"
"$PY" -m g1iv_teleop.hub "$@" > >(tee "$LOG") 2>&1 &
PID=$!
g1iv_pid_write hub "$PID"
trap 'kill -INT $PID 2>/dev/null' INT TERM
RC=0
while true; do                            # wait again if a trapped signal interrupted "wait"
  RC=0; wait $PID || RC=$?            # (plain "wait; RC=$?" would trip set -e)
  kill -0 $PID 2>/dev/null || break
done
rm -f "$G1IV_WS/logs/run/hub.pid"
echo "[hub] exited with code $RC (log: $LOG)"
exit $RC
