#!/usr/bin/env bash
# Smoke dataset for the pipeline test (simulator must be running: scripts/run_sim.sh).
# 3 scripted grasps (recorded), 1 recording aborted mid-way, 1 recording interrupted by a simulator reset.
# All of it goes to data/smoke (source=scripted). This is NOT teleoperation data.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
POSE=(0.70 -0.05 0)
reset() { timeout 30 scripts/record_episode.sh reset --robot-pose "${POSE[@]}" 2>&1 | grep -v multicast; sleep 12; }
for i in 1 2 3; do
  echo "=== scripted grasp $i (recorded)"
  reset
  timeout 400 scripts/run_mock_teleop.sh scripted --record --no-console --instruction "pick up the red cylinder" 2>&1 \
    | grep -E "\[hub\] (control|exited)" | cut -c1-200
done
echo "=== recording aborted mid-way"
reset
( timeout 400 scripts/run_mock_teleop.sh scripted --record --no-console --instruction "pick up the red cylinder" 2>&1 \
    | grep -E "\[hub\] (control|exited)" | cut -c1-200 ) &
sleep 25; timeout 30 scripts/record_episode.sh abort 2>&1 | grep -v multicast; wait
echo "=== recording interrupted by a simulator reset"
reset
( timeout 400 scripts/run_mock_teleop.sh scripted --record --no-console --instruction "pick up the red cylinder" 2>&1 \
    | grep -E "\[hub\] (control|exited|simulator reset)" | cut -c1-200 ) &
sleep 25; timeout 30 scripts/record_episode.sh reset --robot-pose "${POSE[@]}" 2>&1 | grep -v multicast; wait
echo "=== done"
