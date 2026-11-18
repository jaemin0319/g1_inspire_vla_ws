#!/usr/bin/env bash
# Host / environment report for g1_inspire_vla_ws. Read-only: changes nothing.
# Usage: scripts/check_system.sh [--json OUT]
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros

hr() { printf '\n== %s ==\n' "$1"; }
hr "time";        date -Is
hr "os";          lsb_release -ds 2>/dev/null; uname -r
hr "cpu/ram";     echo "cpus=$(nproc)"; free -h | sed -n 1,2p
hr "gpu";         nvidia-smi --query-gpu=index,name,uuid,memory.total,memory.used,driver_version,pci.bus_id --format=csv,noheader 2>&1
hr "gpu procs";   nvidia-smi --query-compute-apps=pid,gpu_uuid,used_memory,process_name --format=csv,noheader 2>&1
hr "display";     echo "DISPLAY=${DISPLAY:-<unset>} XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-<unset>}"
hr "disk";        df -h / "$G1IV_WS" 2>/dev/null | sed 1d
hr "workspace";   echo "G1IV_WS=$G1IV_WS (link: $(readlink -f "$HOME/g1_inspire_vla_ws" 2>/dev/null))"
du -sh "$G1IV_WS"/{.conda,repos,data,logs,policies,assets} 2>/dev/null
hr "conda envs (project root $MAMBA_ROOT_PREFIX)"
for e in g1_inspire_sim g1_inspire_teleop g1_inspire_data g1_inspire_vla; do
  p="$MAMBA_ROOT_PREFIX/envs/$e"
  if [[ -x "$p/bin/python" ]]; then
    printf '%-20s %s  %s\n' "$e" "$("$p/bin/python" -V 2>&1)" "$(du -sh "$p" 2>/dev/null | cut -f1)"
  else
    printf '%-20s (missing)\n' "$e"
  fi
done
hr "ros (must NOT leak into launchers)"
echo "after strip: PYTHONPATH=${PYTHONPATH:-<unset>} ROS_DISTRO=${ROS_DISTRO:-<unset>} opt/ros in LD_LIBRARY_PATH: $(tr ':' '\n' <<<"${LD_LIBRARY_PATH:-}" | grep -c /opt/ros)"
hr "network (for PICO URL)"
ip -4 -o addr show scope global 2>/dev/null | awk '{print $2, $4}'
hr "listening ports used by this project"
ss -ltnp 2>/dev/null | awk 'NR==1 || /:(8012|55555|55556|55557|60001|60002|60003)\b/'
hr "project processes"
pgrep -af "g1_inspire_vla_ws" | grep -v pgrep | cut -c1-200 || true
