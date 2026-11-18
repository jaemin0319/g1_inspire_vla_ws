# Sourced by every project launcher. Selects the project micromamba root and strips ROS 2 Jazzy
# variables inherited from ~/.bashrc so they never leak into the Conda envs (Python 3.11/3.12 vs Jazzy 3.12).
G1IV_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export G1IV_WS
export MAMBA_ROOT_PREFIX="$G1IV_WS/.conda"
export G1IV_MAMBA="${G1IV_MAMBA:-$HOME/.local/bin/micromamba}"
g1iv_strip_ros() {
  unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE \
        ROS_DOMAIN_ID RMW_IMPLEMENTATION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CYCLONEDDS_URI PYTHONPATH
  local v p out
  for v in LD_LIBRARY_PATH CMAKE_PREFIX_PATH PKG_CONFIG_PATH PATH; do
    out=""
    IFS=":" read -ra parts <<< "${!v:-}"
    for p in "${parts[@]}"; do
      [[ -z "$p" || "$p" == /opt/ros/* ]] && continue
      out="${out:+$out:}$p"
    done
    export "$v=$out"
  done
  # any other variable that still points into /opt/ros (GZ_CONFIG_PATH, ...)
  for v in $(compgen -e); do
    [[ "${!v:-}" == *"/opt/ros/"* ]] && unset "$v"
  done
  return 0
}
g1iv_env_prefix() { echo "$MAMBA_ROOT_PREFIX/envs/$1"; }
# Keep pip/conda temp + caches inside the workspace (Isaac Sim wheels need several GB; / may be small).
export TMPDIR="$G1IV_WS/.cache/tmp" PIP_CACHE_DIR="$G1IV_WS/.cache/pip"
mkdir -p "$TMPDIR" "$PIP_CACHE_DIR"

G1IV_SIM_REPO="$G1IV_WS/repos/unitree_sim_isaaclab"
G1IV_XR_REPO="$G1IV_WS/repos/xr_teleoperate"
mkdir -p "$G1IV_WS/logs/run"

g1iv_py() { echo "$MAMBA_ROOT_PREFIX/envs/$1/bin/python"; }

# Select the simulator / CUDA GPU (exported as a UUID, never as a guessed index).
#   auto (default, or $G1IV_GPU) = the first GPU in PCI bus order | 4060 | 5060ti | any part of the GPU name
#   (e.g. 4090, "A6000") | GPU-<uuid>
g1iv_select_gpu() {
  local want="${1:-${G1IV_GPU:-auto}}" uuid list
  list=$(nvidia-smi --query-gpu=pci.bus_id,name,uuid --format=csv,noheader 2>/dev/null | sort) \
    || { echo "nvidia-smi failed: NVIDIA driver missing?" >&2; return 1; }
  case "$want" in
    auto)   uuid=$(awk -F', ' 'NR==1{print $3}' <<<"$list") ;;
    4060)   uuid=$(awk -F', ' '$2 ~ /RTX 4060$/{print $3; exit}' <<<"$list") ;;
    5060ti) uuid=$(awk -F', ' '$2 ~ /5060 Ti/{print $3; exit}' <<<"$list") ;;
    GPU-*)  uuid="$want" ;;
    *)      uuid=$(awk -F', ' -v w="$want" 'index(tolower($2), tolower(w)){print $3; exit}' <<<"$list") ;;
  esac
  [[ -n "$uuid" ]] || { echo "GPU '$want' not found; have: $(cut -d, -f2 <<<"$list" | paste -sd,)" >&2; return 1; }
  export CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES="$uuid"
}

# NVIDIA Isaac Sim EULA: the simulator starts only after the user accepted it once on this machine
# (scripts/bootstrap.sh --accept-nvidia-eula writes the marker) or exported OMNI_KIT_ACCEPT_EULA=YES themselves.
g1iv_eula() {
  if [[ "${OMNI_KIT_ACCEPT_EULA:-}" == YES || -f "$G1IV_WS/.cache/nvidia_eula_accepted" ]]; then
    export OMNI_KIT_ACCEPT_EULA=YES; return 0
  fi
  echo "NVIDIA Isaac Sim EULA not accepted on this machine. Read https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html" >&2
  echo "then run: scripts/bootstrap.sh eula --accept-nvidia-eula   (or export OMNI_KIT_ACCEPT_EULA=YES)" >&2
  return 1
}

# HTTPS/WSS certificates for teleimager (sim) and televuer (hub); project-local, never ~/.config
g1iv_certs() {
  export XR_TELEOP_CERT="$G1IV_WS/certs/cert.pem" XR_TELEOP_KEY="$G1IV_WS/certs/key.pem"
  [[ -f "$XR_TELEOP_CERT" && -f "$XR_TELEOP_KEY" ]] || echo "WARN: certificates missing; run scripts/setup/make_certs.sh" >&2
}

# PID files hold ONLY processes started by these scripts; stop_all.sh signals nothing else.
g1iv_pid_write() { echo "$2" > "$G1IV_WS/logs/run/$1.pid"; }
g1iv_pid_alive() {
  local f="$G1IV_WS/logs/run/$1.pid" p
  [[ -f "$f" ]] || return 1
  # the process must belong to THIS workspace (env python / script path) or carry the project tag; a reused PID
  # of an unrelated program is never treated as ours
  p=$(cat "$f"); [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null && \
    tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | grep -qiF -e "$G1IV_WS" -e "g1iv"
}
