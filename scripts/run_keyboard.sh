#!/usr/bin/env bash
# Keyboard walking (+ keyboard hand OPEN/CLOSE and 1 cm arm nudges) through the project hub.
# Keys act only while the hub window has focus (W/S A/D Q/E, SPACE stop, R engage, Z/X/B hands ...).
#   scripts/run_keyboard.sh               hub: loco=keyboard, arms=keyboard nudges
#   scripts/run_keyboard.sh --official    official send_commands_keyboard.py (global pynput listener!)
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
if [[ "${1:-}" == "--official" ]]; then
  g1iv_strip_ros
  echo "official keyboard sender: GLOBAL key capture (typing in any window moves the robot). Q to quit."
  cd "$G1IV_SIM_REPO"
  PYTHONPATH="$G1IV_SIM_REPO" exec "$(g1iv_py g1_inspire_teleop)" send_commands_keyboard.py
fi
exec "$(dirname "${BASH_SOURCE[0]}")/_hub_launch.sh" keyboard --arm-source keyboard --loco-source keyboard "$@"
