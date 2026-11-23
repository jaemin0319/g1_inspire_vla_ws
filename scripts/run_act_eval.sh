#!/usr/bin/env bash
# Learned ACT policy rollouts in the simulator (lockstep policy bridge; tools/run_act_policy.py header has the details).
#
#   scripts/run_act_eval.sh sim [run_sim.sh args ...]        simulator with the policy bridge, detached; defaults:
#        GPU 4060, desk 0.65 m, single red cylinder (the scene of walk_grasp_desk065_scripted_v1), recordings to
#        $G1IV_POLICY_EVAL_ROOT (default data/policy_eval/; never data/raw or data/smoke), grasp observer view.
#        Stop: scripts/stop_all.sh
#        G1IV_OBSERVER_VIDEO=<dir>: also an mp4 of the observer camera per episode (docs/ACT_EVAL.md, front view)
#   scripts/run_act_eval.sh server <checkpoint/pretrained_model> [act_policy_server.py args ...]
#        policy server on $G1IV_POLICY_GPU (default 4060), detached; log logs/run/act_policy_server.log
#   scripts/run_act_eval.sh run [run_act_policy.py args ...]  episodes (foreground; --mode walk|far|near, -n N, --label L)
#   scripts/run_act_eval.sh stop-server
# No teleop hub may run at the same time (the runner owns rt/lowcmd, rt/inspire/cmd, rt/run_command/cmd).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros
BRIDGE="$G1IV_WS/logs/run/policy_bridge.sock"
POLSOCK="$G1IV_WS/logs/run/act_policy.sock"
LEROBOT_PY="${G1IV_LEROBOT_PY:-/mnt/isaac/so101_ws/.venv-train/bin/python}"
EVAL_ROOT="${G1IV_POLICY_EVAL_ROOT:-data/policy_eval}"   # evaluation recordings (~0.16 MB/frame); may be absolute
cmd="${1:-}"; shift || true
case "$cmd" in
  sim)
    if g1iv_pid_alive hub; then echo "teleop hub is running: scripts/stop_all.sh first" >&2; exit 1; fi
    export G1IV_POLICY_BRIDGE="$BRIDGE"
    mkdir -p "$G1IV_WS/logs/run"
    setsid nohup "$G1IV_WS/scripts/run_sim.sh" --gpu "${G1IV_GPU:-4060}" \
      --scene-set desk.top_surface_height=0.65 --scene-set extra_objects.enabled=false \
      --scene-set "cameras.observer.pos=[1.75,-1.05,1.30]" --scene-set "cameras.observer.look_at=[0.90,-0.12,0.85]" \
      --record-set "raw_root=$EVAL_ROOT" --record-set "smoke_root=$EVAL_ROOT" "$@" \
      > "$G1IV_WS/logs/run/act_eval_sim.out" 2>&1 < /dev/null &
    echo "simulator starting (log: logs/run/act_eval_sim.out, then logs/sim/sim_*.log); bridge $BRIDGE"
    ;;
  server)
    ck="${1:?checkpoint .../pretrained_model}"; shift
    if g1iv_pid_alive act_policy_server; then
      echo "policy server already running (pid $(cat "$G1IV_WS/logs/run/act_policy_server.pid")):"
      grep -o "'checkpoint': '[^']*'\|'temporal_ensemble_coeff': [^,]*" "$G1IV_WS/logs/run/act_policy_server.log" | head -2
      echo "to change checkpoint/options: scripts/run_act_eval.sh stop-server, then server again"
      exit 0
    fi
    [[ -d "$ck" ]] || { echo "checkpoint folder not found: $ck" >&2; exit 1; }
    [[ -x "$LEROBOT_PY" ]] || { echo "LeRobot python missing: $LEROBOT_PY (set G1IV_LEROBOT_PY)" >&2; exit 1; }
    g1iv_select_gpu "${G1IV_POLICY_GPU:-4060}"
    export PYTHONNOUSERSITE=1 HF_HUB_OFFLINE=1 TORCH_HOME="${TORCH_HOME:-/mnt/isaac/so101_ws/.cache/torch}"
    setsid nohup "$LEROBOT_PY" "$G1IV_WS/tools/act_policy_server.py" --checkpoint "$ck" --socket "$POLSOCK" "$@" \
      > "$G1IV_WS/logs/run/act_policy_server.log" 2>&1 < /dev/null &
    echo $! > "$G1IV_WS/logs/run/act_policy_server.pid"
    for _ in $(seq 1 120); do grep -q "listening on" "$G1IV_WS/logs/run/act_policy_server.log" 2>/dev/null && break; sleep 1; done
    grep -E "act_server|Error" "$G1IV_WS/logs/run/act_policy_server.log" | tail -3
    ;;
  stop-server)
    f="$G1IV_WS/logs/run/act_policy_server.pid"
    if g1iv_pid_alive act_policy_server; then
      p=$(cat "$f"); kill "$p"
      for _ in $(seq 1 50); do kill -0 "$p" 2>/dev/null || break; sleep 0.2; done
      echo "policy server stopped"
    else
      echo "no policy server"
    fi
    rm -f "$f" "$POLSOCK"
    ;;
  run)
    if g1iv_pid_alive hub; then echo "teleop hub is running: scripts/stop_all.sh first" >&2; exit 1; fi
    g1iv_pid_alive sim || { echo "simulator is not running: scripts/run_act_eval.sh sim first" >&2; exit 1; }
    g1iv_pid_alive act_policy_server || { echo "policy server is not running: scripts/run_act_eval.sh server <checkpoint> first" >&2; exit 1; }
    # the bridge socket is (re)created by the simulator once its extension is up (~1 min after start)
    for i in $(seq 1 300); do
      [[ -S "$BRIDGE" && "$BRIDGE" -nt "$G1IV_WS/logs/run/sim.pid" ]] && break
      (( i == 1 )) && echo "waiting for the simulator to finish loading (policy bridge) ..."
      sleep 1
    done
    [[ -S "$BRIDGE" && "$BRIDGE" -nt "$G1IV_WS/logs/run/sim.pid" ]] \
      || { echo "simulator is up but has no policy bridge: start it with scripts/run_act_eval.sh sim (not run_sim.sh)" >&2; exit 1; }
    export PYTHONPATH="$G1IV_WS/src:$G1IV_XR_REPO"
    cd "$G1IV_WS"
    exec "$(g1iv_py g1_inspire_teleop)" tools/run_act_policy.py --bridge "$BRIDGE" --policy "$POLSOCK" "$@"
    ;;
  *) sed -n 2,12p "$0"; exit 1 ;;
esac
