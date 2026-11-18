#!/usr/bin/env bash
# Stop ONLY processes started by this project's launchers (PID files in logs/run). Nothing else is signalled.
#
# Known upstream behaviour (Isaac Sim 5.1 + unitree sim_main, also without the project extension): after
# SIGINT the official cleanup finishes ("cleanup completed" / "Current main process PID" in the sim log;
# the recorder has already closed its episode by then) but simulation_app.close() keeps the main thread
# busy and never returns. Once that point is reached the process is killed after a short grace period.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
alive() { kill -0 "$1" 2>/dev/null; }
for name in hub sim; do
  f="$G1IV_WS/logs/run/$name.pid"
  [[ -f "$f" ]] || continue
  pid=$(cat "$f")
  if g1iv_pid_alive "$name"; then
    echo "stopping $name (pid $pid) with SIGINT"; kill -INT "$pid"
    log=""
    [[ $name == sim ]] && log=$(ls -t "$G1IV_WS"/logs/sim/sim_*.log 2>/dev/null | head -1)
    cleaned_at=""
    for _ in $(seq 1 60); do        # up to 30 s
      alive "$pid" || break
      if [[ -n "$log" && -z "$cleaned_at" ]] && grep "Current main process PID: $pid" "$log" >/dev/null 2>&1; then
        cleaned_at=$SECONDS
      fi
      if [[ -n "$cleaned_at" ]] && (( SECONDS - cleaned_at >= 5 )); then
        echo "  official cleanup done; simulation_app.close() did not return within 5 s (known) -> SIGKILL"
        : > "$G1IV_WS/logs/run/sim.$pid.close_hang"   # tells run_sim.sh this was a normal stop
        kill -KILL "$pid"
        for _ in $(seq 1 20); do alive "$pid" || break; sleep 0.1; done
        break
      fi
      sleep 0.5
    done
    if alive "$pid"; then echo "  still alive -> SIGTERM"; kill -TERM "$pid"; sleep 3; fi
    if alive "$pid"; then echo "  still alive -> SIGKILL"; kill -KILL "$pid"; fi
  else
    echo "$name: stale pid file ($pid), removing"
  fi
  rm -f "$f"
done
