#!/usr/bin/env bash
# Live demo of one v3 ACT checkpoint in the simulator GUI on the RTX 4060 (2026-11-24 user request).
#
#   demo_act.sh start [step=040000]   pause the v3 checkpoint evaluation loop, stop its headless simulator, open the
#                                     simulator window, load the checkpoint and run episodes back to back (random
#                                     robot start / cylinder; a new one after every success, fall or 40 s timeout)
#   demo_act.sh reset                 end the current episode now and start a new one from a new random start
#   demo_act.sh stop                  close the demo and RESUME the evaluation loop (it restarts its own simulator)
#   demo_act.sh panel                 (re)open the button window (리셋 / 정지)
#   demo_act.sh status                last runner lines
# Recordings of the demo episodes: /home/jammin/g1iv_store/demo_rec (about 80 MB per episode; safe to delete).
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
ST=/home/jammin/g1iv_store
STATE=$ST/run/demo.env
RLOG=$ST/logs/demo_runner.log
export DISPLAY=${DISPLAY:-:0} XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority} PYTHONNOUSERSITE=1
unset G1IV_OBSERVER_VIDEO
mkdir -p "$ST/run" "$ST/logs" "$ST/demo_rec" "$ST/demo/runs"
cd "$WS"

runner_pid() { pgrep -f "bin/python[0-9.]* tools/run_act_policy.py .*--label v3_[0-9]+_demo" | head -1; }
eval_loop_pid() { pgrep -f "^bash /home/jammin/g1iv_store/v3_eval_ckpts.sh" | head -1; }

start_runner() {
  local step=$1 seed=$(( $(date +%s) % 100000000 ))
  setsid nohup scripts/run_act_eval.sh run --mode far -n 1000 --seed "$seed" --robot-y -1.0 1.0 --object-y -0.45 0.45 \
    --max-frames 1000 --label "v3_${step}_demo" --out "$ST/demo/runs/$(date +%Y%m%d_%H%M%S)_v3_${step}" \
    >> "$RLOG" 2>&1 < /dev/null &
  echo "=== $(date '+%F %T') runner started (checkpoint $step, seed $seed)" >> "$RLOG"
}

stop_runner() {
  local p; p=$(runner_pid)
  [[ -z $p ]] && return 0
  kill "$p"
  for _ in $(seq 1 50); do kill -0 "$p" 2>/dev/null || return 0; sleep 0.2; done
  kill -KILL "$p" 2>/dev/null
}

open_panel() { setsid nohup "$ST/demo_panel.sh" > /dev/null 2>&1 < /dev/null & }

case "${1:-}" in
  start)
    step=${2:-040000}
    ck=$ST/train/act_g1_walk_wide_v3/checkpoints/$step/pretrained_model
    [[ -f $ck/model.safetensors ]] || { echo "checkpoint not found: $ck" >&2; exit 1; }
    if [[ -n $(runner_pid) ]]; then echo "demo already running (demo_act.sh reset / stop)"; exit 0; fi
    if pgrep -f "bin/python[0-9.]* tools/run_act_policy.py .*--label v3_[0-9]+_nas25" > /dev/null; then
      echo "a checkpoint evaluation is running right now; try again when it has finished" >&2; exit 1
    fi
    ep=$(eval_loop_pid)
    if [[ -n $ep ]]; then kill -STOP "$ep"; echo "evaluation loop paused (pid $ep)"; fi
    echo "EVAL_PID=${ep}" > "$STATE"; echo "STEP=$step" >> "$STATE"
    scripts/run_act_eval.sh stop-server > /dev/null 2>&1; scripts/stop_all.sh > /dev/null 2>&1; sleep 3
    echo "starting the simulator window on the 4060 (about 1-2 min) ..."
    G1IV_GPU=4060 G1IV_POLICY_EVAL_ROOT=$ST/demo_rec scripts/run_act_eval.sh sim > /dev/null
    sleep 5
    for i in $(seq 1 300); do
      grep -q "DDS project topics up" logs/run/act_eval_sim.out 2>/dev/null && break
      [[ -f logs/run/sim.pid ]] && kill -0 "$(cat logs/run/sim.pid)" 2>/dev/null || { echo "simulator exited: logs/run/act_eval_sim.out" >&2; exit 1; }
      sleep 2
    done
    sleep 10
    G1IV_POLICY_GPU=4060 scripts/run_act_eval.sh server "$ck" --n-action-steps 25 | tail -1
    start_runner "$step"
    open_panel
    echo "demo running: checkpoint $step. buttons: the '데모 조작' window; commands: $0 reset | stop"
    ;;
  reset)
    [[ -f $STATE ]] || { echo "no demo running"; exit 1; }
    source "$STATE"
    stop_runner
    echo "=== $(date '+%F %T') reset" >> "$RLOG"
    start_runner "$STEP"
    echo "reset: new random start (checkpoint $STEP)"
    ;;
  stop)
    stop_runner
    pkill -f "^bash /home/jammin/g1iv_store/demo_panel.sh" 2>/dev/null; pkill -f "^zenity --info --title=데모 조작" 2>/dev/null
    scripts/run_act_eval.sh stop-server > /dev/null 2>&1; scripts/stop_all.sh > /dev/null 2>&1
    if [[ -f $STATE ]]; then
      source "$STATE"
      if [[ -n ${EVAL_PID:-} ]] && kill -0 "$EVAL_PID" 2>/dev/null; then
        kill -CONT "$EVAL_PID"; echo "evaluation loop resumed (pid $EVAL_PID)"
      fi
      rm -f "$STATE"
    fi
    echo "=== $(date '+%F %T') demo stopped" >> "$RLOG"
    echo "demo stopped"
    ;;
  panel) open_panel ;;
  status) grep -E "^===|episode [0-9]+:|\] ep [0-9]+ frame" "$RLOG" | tail -6 ;;
  *) sed -n 2,12p "$0"; exit 1 ;;
esac
