#!/usr/bin/env bash
# Rebuild this workspace on a fresh Ubuntu 24.04 PC (x86_64, NVIDIA GPU + driver). No sudo, nothing outside the
# workspace except micromamba in ~/.local/bin (or $G1IV_MAMBA). Every step is idempotent; run it again after a failure.
#
#   scripts/bootstrap.sh all --accept-nvidia-eula     everything below (the EULA flag is needed for the hand USD build)
#   scripts/bootstrap.sh check                        preflight only (OS, driver, GPU, disk, RAM, tools)
#   scripts/bootstrap.sh micromamba | repos | assets | envs | eula | hand | certs | test | smoke
#   options: --with-vla (also the GR00T loader env), --accept-nvidia-eula
#
# Steps (docs/INSTALL.md):
#   micromamba  pinned micromamba 2.9.0 (sha256-checked) -> ~/.local/bin/micromamba unless present
#   repos       external repositories at the commits of envs/repos.lock.json + project patches (envs/patches/)
#   assets      official Unitree USD/policy bundle, HF revision + sha256 of envs/assets.lock.json
#   envs        Conda envs under .conda/: g1_inspire_sim (Isaac Sim 5.1 + Isaac Lab), _teleop, _data, _cad (+ _vla)
#   eula        record that YOU accepted the NVIDIA Omniverse/Isaac Sim EULA (only with --accept-nvidia-eula)
#   hand        Inspire RH56E2 hand: references (envs/hand_reference.lock.json) -> npz -> robot USD
#   certs       local HTTPS certificates for the PICO/WebXR teleoperation (certs/, git-ignored)
#   test        unit tests (no simulator)
#   smoke       start the simulator headless, wait for its status topic, stop it
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$WS/scripts/_env_common.sh"
g1iv_strip_ros
cd "$WS"

MM_VER=2.9.0-0
MM_SHA=366cd9cd8be14df1ab8ed50352a82111082a36686b2d389fdb79a92c3fafb3e3
STEP=${1:-}; shift || true
VLA=0 EULA=0
for a in "$@"; do
  case "$a" in --with-vla) VLA=1 ;; --accept-nvidia-eula) EULA=1 ;; *) echo "unknown option $a" >&2; exit 2 ;; esac
done
say() { echo -e "\n=== [bootstrap] $* ==="; }

check() {
  say "preflight"
  local fail=0 warn=0
  ok()   { echo "  ok    $*"; }
  bad()  { echo "  FAIL  $*"; fail=1; }
  wrn()  { echo "  warn  $*"; warn=1; }
  . /etc/os-release 2>/dev/null || true
  [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] && ok "Ubuntu 24.04" || wrn "OS ${PRETTY_NAME:-unknown}: tested on Ubuntu 24.04 only"
  [[ "$(uname -m)" == x86_64 ]] && ok "x86_64" || bad "architecture $(uname -m): Isaac Sim pip wheels are x86_64"
  for c in git python3 openssl ip sha256sum; do command -v $c >/dev/null && ok "$c" || bad "$c missing (sudo apt install git python3 openssl iproute2 coreutils)"; done
  if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
    local drv; drv=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)
    [[ ${drv%%.*} -ge 570 ]] && ok "NVIDIA driver $drv (tested: 580.178.04)" || wrn "NVIDIA driver $drv: RTX 50 needs >= 570; tested with 580.178.04"
    while IFS='|' read -r name mem; do
      [[ $mem -ge 7900 ]] && ok "GPU $name ${mem} MiB" || wrn "GPU $name ${mem} MiB: tested on an 8 GB RTX 4060 (four-camera recording ~3 GB)"
    done < <(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader,nounits | awk -F', ' '{print $1"|"$2}')
  else
    bad "nvidia-smi not working: install the NVIDIA driver (Ubuntu: sudo ubuntu-drivers install), reboot"
  fi
  { ls /usr/lib/x86_64-linux-gnu/libvulkan.so.1 /lib/x86_64-linux-gnu/libvulkan.so.1 2>/dev/null | grep -q . || \
    ldconfig -p 2>/dev/null | grep -q "libvulkan.so.1 "; } && ok "Vulkan loader" || wrn "libvulkan.so.1 not found (sudo apt install libvulkan1)"
  local free_gb; free_gb=$(df -BG --output=avail "$WS" | tail -1 | tr -dc 0-9)
  [[ $free_gb -ge 60 ]] && ok "free disk ${free_gb} GB at $WS" || \
    { [[ $free_gb -ge 45 ]] && wrn "free disk ${free_gb} GB (needs ~45 GB: envs 24, pip cache ~8, repos 4.3, assets 3)" || bad "free disk ${free_gb} GB < 45 GB"; }
  local mem_gb; mem_gb=$(awk '/MemTotal/{printf "%d", $2/1048576}' /proc/meminfo)
  [[ $mem_gb -ge 30 ]] && ok "RAM ${mem_gb} GB" || wrn "RAM ${mem_gb} GB: Isaac Sim recommends >= 32 GB"
  [[ -n "${ROS_DISTRO:-}" ]] && echo "  info  ROS ${ROS_DISTRO} sourced in this shell: the project scripts strip its variables" || true
  [[ $fail == 0 ]] || { echo "preflight failed" >&2; return 1; }
  [[ $warn == 0 ]] && echo "  preflight passed" || echo "  preflight passed with warnings"
}

micromamba_step() {
  say "micromamba"
  if [[ -x "$G1IV_MAMBA" ]]; then echo "  present: $G1IV_MAMBA ($("$G1IV_MAMBA" --version))"; return 0; fi
  mkdir -p "$(dirname "$G1IV_MAMBA")"
  python3 - "$G1IV_MAMBA" "$MM_VER" "$MM_SHA" <<'EOF'
import hashlib, os, sys, urllib.request
dst, ver, sha = sys.argv[1:4]
url = f"https://github.com/mamba-org/micromamba-releases/releases/download/{ver}/micromamba-linux-64"
data = urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "g1iv-bootstrap"}), timeout=120).read()
if hashlib.sha256(data).hexdigest() != sha:
    sys.exit(f"micromamba sha256 mismatch for {url}")
open(dst + ".part", "wb").write(data); os.chmod(dst + ".part", 0o755); os.replace(dst + ".part", dst)
print(f"  installed {dst} ({ver})")
EOF
}

envs_step() {
  say "conda envs (Isaac Sim 5.1 wheels ~18 GB download on the first run)"
  for e in sim teleop data cad; do bash scripts/setup_envs.sh $e; done
  [[ $VLA == 1 ]] && bash scripts/setup_envs.sh vla || echo "  (GR00T loader env skipped; add --with-vla)"
}

eula_step() {
  say "NVIDIA EULA"
  if [[ $EULA == 1 ]]; then
    mkdir -p .cache
    echo "accepted by $(id -un) on $(date -Iseconds) via scripts/bootstrap.sh --accept-nvidia-eula" > .cache/nvidia_eula_accepted
    echo "  recorded in .cache/nvidia_eula_accepted"
  elif [[ -f .cache/nvidia_eula_accepted ]]; then
    echo "  already accepted on this machine"
  else
    echo "  not accepted. Read https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html"
    echo "  and re-run with --accept-nvidia-eula (needed by the hand USD build and the simulator)"
  fi
}

test_step() {
  say "unit tests"
  CUDA_VISIBLE_DEVICES="" PYTHONPATH="$WS/src" "$(g1iv_py g1_inspire_teleop)" -m pytest -q tests
}

smoke_step() {
  say "simulator smoke test (headless, no cameras, ~1-3 min)"
  g1iv_eula
  local log=logs/setup/smoke_sim.log PY ok=0
  mkdir -p logs/setup
  bash scripts/run_sim.sh --headless --demo > "$log" 2>&1 &
  PY=$(g1iv_py g1_inspire_teleop)
  for _ in $(seq 1 60); do
    sleep 5
    if CUDA_VISIBLE_DEVICES="" PYTHONPATH="$WS/src" timeout 15 "$PY" -m g1iv_teleop.ctl status 2>/dev/null | grep -q '"sim_time"'; then ok=1; break; fi
    grep -q "exit 3\|Traceback" "$log" && break
  done
  bash scripts/stop_all.sh >/dev/null 2>&1 || true
  [[ $ok == 1 ]] && echo "  simulator answered on DDS (log $log)" || { echo "  simulator did not come up, see $log" >&2; return 1; }
}

case "$STEP" in
  check) check ;;
  micromamba) micromamba_step ;;
  repos) say "external repositories"; python3 scripts/setup/fetch_repos.py ;;
  assets) say "official assets"; python3 scripts/setup/fetch_assets.py ;;
  envs) envs_step ;;
  eula) eula_step ;;
  hand) say "RH56E2 hand"; python3 scripts/setup/fetch_hand_reference.py; bash scripts/setup/build_hand.sh ;;
  certs) say "certificates"; bash scripts/setup/make_certs.sh ;;
  test) test_step ;;
  smoke) smoke_step ;;
  all)
    check; micromamba_step
    say "external repositories"; python3 scripts/setup/fetch_repos.py
    say "official assets"; python3 scripts/setup/fetch_assets.py
    envs_step; eula_step
    say "RH56E2 hand"; python3 scripts/setup/fetch_hand_reference.py; bash scripts/setup/build_hand.sh
    say "certificates"; bash scripts/setup/make_certs.sh
    test_step; smoke_step
    say "done: see README.md 'Run'" ;;
  *) sed -n '2,22p' "$0"; exit 2 ;;
esac
