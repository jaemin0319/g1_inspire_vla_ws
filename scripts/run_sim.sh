#!/usr/bin/env bash
# Start the simulator: official unitree_sim_isaaclab sim_main.py (IsaacSim5.1.0 branch + project patch)
# with the project task, the wholebody locomotion policy and the sim-side recorder.
#
#   scripts/run_sim.sh                      GUI on the first GPU (or $G1IV_GPU), project desk task
#   scripts/run_sim.sh --headless           no window (offscreen rendering for cameras)
#   scripts/run_sim.sh --gpu 5060ti         render/CUDA on another GPU (4060 | 5060ti | part of the name | GPU-uuid)
#   scripts/run_sim.sh --official-task      unmodified official Isaac-Move-Cylinder-G129-Inspire-Wholebody scene
#   scripts/run_sim.sh --scene-set robot.init_pos=[0.55,0,0.8]   (repeatable YAML overrides)
#   scripts/run_sim.sh --rig-set use_rig=false   camera rig overrides (configs/camera_rig.yaml, docs/CAMERA_RIG.md)
#   scripts/run_sim.sh -- --step_hz 100     extra args go to sim_main.py
#   scripts/run_sim.sh --demo               real-time demo viewing: no camera sensors (no camera data, no recording),
#                                           viewport every 2nd control step, real-time pacing (configs/scene_desk.yaml demo)
#   scripts/run_sim.sh --demo-cams          --demo plus the four policy cameras in the GUI camera panel (slower than real time)
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros

GPU=${G1IV_GPU:-auto} HEADLESS=0 IFACE=lo TASK=G1IV-Desk-Cylinder-G129-Inspire-Wholebody EXT=1 DEMO=0 DEMO_CAMS=0
SCENE_OV=() REC_OV=() RIG_OV=() EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gpu) GPU=$2; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --dds-iface) IFACE=$2; shift 2 ;;        # "" = official auto-detected NIC
    --official-task) TASK=Isaac-Move-Cylinder-G129-Inspire-Wholebody; shift ;;
    --task) TASK=$2; shift 2 ;;
    --no-ext) EXT=0; shift ;;                # plain official sim_main behaviour (no recorder/status)
    --demo) DEMO=1; shift ;;
    --demo-cams) DEMO=1; DEMO_CAMS=1; shift ;;
    --scene-set) SCENE_OV+=("$2"); shift 2 ;;
    --record-set) REC_OV+=("$2"); shift 2 ;;
    --rig-set) RIG_OV+=("$2"); shift 2 ;;
    --) shift; EXTRA+=("$@"); break ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done
if [[ $DEMO == 1 ]]; then             # user --scene-set values come later and still win
  DEMO_OV=("cameras.observer.sensor=false" "cameras.image_server_stream=false" "demo.render_every_n_steps=2"
           "demo.realtime_pacing=true")
  if [[ $DEMO_CAMS == 1 ]]; then
    DEMO_OV+=("cameras.update_period=0.04"); RIG_OV=("gui_panel.every_n_steps=4" ${RIG_OV[@]+"${RIG_OV[@]}"})
  else
    DEMO_OV+=("cameras.policy=[]")
  fi
  SCENE_OV=("${DEMO_OV[@]}" ${SCENE_OV[@]+"${SCENE_OV[@]}"})
fi
if g1iv_pid_alive sim; then echo "simulator already running (pid $(cat "$G1IV_WS/logs/run/sim.pid")); scripts/stop_all.sh first" >&2; exit 1; fi

PY=$(g1iv_py g1_inspire_sim)
[[ -x "$PY" ]] || { echo "env g1_inspire_sim missing: scripts/setup_envs.sh sim" >&2; exit 1; }
g1iv_select_gpu "$GPU"
g1iv_certs
g1iv_eula                                # NVIDIA Isaac Sim EULA accepted on this machine? (scripts/bootstrap.sh)
export G1IV_DDS_IFACE="$IFACE"
export PYTHONUNBUFFERED=1                # Kit ends the process hard; buffered prints (e.g. the error) are lost otherwise
[[ $EXT == 1 ]] && export G1IV_EXT=1
export PYTHONPATH="$G1IV_WS/src:$G1IV_SIM_REPO"
to_json() { local out="[" s; for s in "$@"; do out+="\"${s//\"/\\\"}\","; done; echo "${out%,}]"; }
[[ ${#SCENE_OV[@]} -gt 0 ]] && export G1IV_SCENE_OVERRIDES="$(to_json "${SCENE_OV[@]}")"
[[ ${#REC_OV[@]} -gt 0 ]] && export G1IV_RECORD_OVERRIDES="$(to_json "${REC_OV[@]}")"
[[ ${#RIG_OV[@]} -gt 0 ]] && export G1IV_RIG_OVERRIDES="$(to_json "${RIG_OV[@]}")"

mkdir -p "$G1IV_WS/logs/sim"
LOG="$G1IV_WS/logs/sim/sim_$(date +%Y%m%d_%H%M%S).log"
ARGS=(--device cpu --enable_cameras --task "$TASK" --robot_type g129 --enable_inspire_dds)
# RTX lens-distortion USD schema (calibrated OpenCV lens model of the camera rig); registers only at app start
KIT_ARGS="--enable omni.usd.schema.omni_lens_distortion"
[[ $DEMO == 1 ]] && KIT_ARGS+=" --/app/runLoops/main/rateLimitEnabled=false"
ARGS+=(--kit_args "$KIT_ARGS")
[[ $HEADLESS == 1 ]] && ARGS+=(--headless)
# demo: the project extension paces the loop (cumulative deadline); the official per-step limiter is lifted and
# the GUI app's 60 Hz main run-loop limit is switched off (the headless app files already do that)
[[ $DEMO == 1 ]] && ARGS+=(--step_hz 1000)
echo "[run_sim] task=$TASK gpu=$GPU($CUDA_VISIBLE_DEVICES) headless=$HEADLESS demo=$DEMO dds_iface=${IFACE:-auto} ext=$EXT log=$LOG"
cd "$G1IV_SIM_REPO"                       # official code uses repo-relative paths (assets/, teleimager config)
"$PY" sim_main.py "${ARGS[@]}" "${EXTRA[@]}" > >(tee "$LOG") 2>&1 &
PID=$!
g1iv_pid_write sim "$PID"
HANG_MARK="$G1IV_WS/logs/run/sim.$PID.close_hang"
# Watchdog for the known upstream hang: after the official cleanup ("Current main process PID: <pid>")
# simulation_app.close() never returns (also without the project extension). Kill 5 s after that point.
(
  while kill -0 "$PID" 2>/dev/null; do
    if grep "Current main process PID: $PID" "$LOG" >/dev/null 2>&1; then
      sleep 5
      if kill -0 "$PID" 2>/dev/null; then
        echo "[run_sim] official cleanup finished but simulation_app.close() did not return in 5 s (known) -> SIGKILL"
        : > "$HANG_MARK"; kill -KILL "$PID" 2>/dev/null
      fi
      break
    fi
    sleep 1
  done
) &
WD=$!
trap 'kill -INT $PID 2>/dev/null' INT TERM
RC=0
while true; do                            # wait again if a trapped signal interrupted "wait"
  RC=0; wait $PID || RC=$?            # (plain "wait; RC=$?" would trip set -e)
  kill -0 $PID 2>/dev/null || break
done
kill $WD 2>/dev/null || true
rm -f "$G1IV_WS/logs/run/sim.pid"
if [[ -f "$HANG_MARK" ]]; then
  echo "[run_sim] stopped after the official cleanup (SIGKILL only for the close() hang) -> treated as exit 0"
  rm -f "$HANG_MARK"; RC=0
fi
# A failed start can leave a forked sim_main.py child (e.g. a multiprocessing helper) orphaned. Terminate only
# sim_main.py processes in THIS launcher's process group.
PG=$(ps -o pgid= -p $$ | tr -d ' ')
LEFT=$(pgrep -g "$PG" -f "sim_main.py" | grep -v -x "$$" || true)
if [[ -n "$LEFT" ]]; then
  echo "[run_sim] terminating leftover sim_main children: $(echo $LEFT)"
  kill -TERM $LEFT 2>/dev/null || true; sleep 3; kill -KILL $LEFT 2>/dev/null || true
fi
echo "[run_sim] sim_main exited with code $RC (log: $LOG)"
# The official sim_main catches start-up exceptions, prints them and still exits 0.
sleep 0.5   # let tee flush
if [[ $RC == 0 ]] && ! grep "start controller success" "$LOG" >/dev/null && \
   grep -E "Failed to parse environment configuration|Failed to create environment|action provider creation failed|Failed to create action provider|Failed to create dds|Failed to create image server|^Traceback" "$LOG" >/dev/null; then
  echo "[run_sim] start-up failure found in the log -> exit 3" >&2
  RC=3
fi
exit $RC
