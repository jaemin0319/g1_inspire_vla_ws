#!/usr/bin/env bash
# Create / complete the project Conda envs (micromamba root: $G1IV_WS/.conda). Idempotent: existing envs
# are reused, never deleted. Every pip call uses the target env's python; nothing goes to base/system.
#
#   scripts/setup_envs.sh sim | teleop | data | vla | all
#
# Pinned sources (see docs/VERSION_MATRIX.md):
#   sim    : Python 3.11, torch 2.7.0+cu128, Isaac Sim 5.1.0.0 (pip from pypi.nvidia.com; or hard-linked
#            from $G1IV_ISAACSIM_LINK_FROM), Isaac Lab 80094be (v2.3.2+7), unitree_sim_isaaclab
#            IsaacSim5.1.0 (+ g1iv patch), unitree_sdk2_python 814556d, teleimager b81de44 [server]
#   teleop : Python 3.10, pinocchio 3.1.0 + numpy 1.26.4 (conda-forge), xr_teleoperate 817fb00
#            (televuer 766de45, teleimager 57cf2a4), unitree_sdk2_python 814556d, pygame 2.6.1
#   data   : Python 3.11, torch 2.7.0+cu128 (hard-linked from the sim env), lerobot 0.3.3 (dataset v2.1)
#   vla    : Python 3.12, Isaac-GR00T N1.7 (3b49067) - optional, see docs/SETUP_REPORT.md
#   cad    : Python 3.12, numpy/scipy/trimesh/opencv for tools/cad (RH56E2 hand npz from the Inspire URDF)
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros
M="$G1IV_MAMBA"
R="$G1IV_WS/repos"
L="$G1IV_WS/logs/setup"; mkdir -p "$L"
exists() { [[ -x "$(g1iv_py "$1")" ]]; }
pipi() { local env=$1; shift; "$(g1iv_py "$env")" -m pip install "$@"; }

setup_sim() {
  local E=g1_inspire_sim PY
  exists $E || "$M" create -y -r "$MAMBA_ROOT_PREFIX" -n $E -c conda-forge --override-channels \
      python=3.11 pip "libstdcxx-ng>=13" cmake make git-lfs unzip
  PY=$(g1iv_py $E)
  "$PY" -c "import torch,sys; sys.exit(0 if torch.__version__=='2.7.0+cu128' else 1)" 2>/dev/null || \
    pipi $E torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128
  # (pip show, not "import isaacsim": the import blocks on the EULA prompt without a TTY and looked "missing")
  if ! "$PY" -m pip show isaacsim 2>/dev/null | grep "^Version: 5.1.0.0" >/dev/null; then
    # Optional (the PC this was built on): hard-link the byte-identical Isaac Sim 5.1.0.0 install of another env
    # instead of downloading ~18 GB again: G1IV_ISAACSIM_LINK_FROM=<env prefix>. Unset = pip from pypi.nvidia.com.
    local SRC="${G1IV_ISAACSIM_LINK_FROM:-}"
    if [[ -n "$SRC" && -x "$SRC/bin/python" ]] && "$SRC/bin/python" -m pip show isaacsim 2>/dev/null | grep "^Version: 5.1.0.0" >/dev/null; then
      "$PY" -m pip uninstall -y numpy pillow sympy mpmath networkx jinja2 MarkupSafe typing-extensions filelock fsspec || true
      "$PY" -I "$G1IV_WS/scripts/setup/link_dists.py" --snapshot "$SRC" --out "$L/igris_env_snapshot_before.json"
      "$PY" -I "$G1IV_WS/scripts/setup/link_dists.py" --src "$SRC" --dst "$MAMBA_ROOT_PREFIX/envs/$E" \
          --report "$L/link_isaacsim_report.json" --exclude torch --exclude torchvision --exclude torchaudio \
          "isaacsim[all,extscache]==5.1.0.0" numpy==1.26.0 pillow==11.3.0 sympy==1.13.3 mpmath networkx jinja2 MarkupSafe typing-extensions filelock fsspec
      "$PY" -I "$G1IV_WS/scripts/setup/link_dists.py" --snapshot "$SRC" --out "$L/igris_env_snapshot_after.json"
      cmp -s <(python3 -c "import json;print(json.load(open('$L/igris_env_snapshot_before.json'))['digest(path,size,mtime_ns,inode,mode)'])") \
             <(python3 -c "import json;print(json.load(open('$L/igris_env_snapshot_after.json'))['digest(path,size,mtime_ns,inode,mode)'])") \
        || { echo "source env $SRC changed during linking!" >&2; exit 1; }
    else
      pipi $E "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
    fi
  fi
  pipi $E packaging==23.0 wheel==0.45.1
  # unitree_sdk2py pins cyclonedds==0.10.2 (Python), which has no cp311 wheel and must be built against a
  # CycloneDDS C library of the releases/0.10.x line (unitree_sdk2_python FAQ). Use the conda-forge C
  # library 0.10.5 inside this env (adds exactly one package; no ROS /opt/ros library is used).
  [[ -e "$MAMBA_ROOT_PREFIX/envs/$E/lib/libddsc.so" ]] || \
    "$M" install -y -r "$MAMBA_ROOT_PREFIX" -n $E -c conda-forge --override-channels "cyclonedds=0.10.5"
  export CYCLONEDDS_HOME="$MAMBA_ROOT_PREFIX/envs/$E"
  # Constraints keep the Isaac Sim / torch pins while installing Isaac Lab + Unitree code.
  #   cryptography==44.0.0 : isaacsim-core pin (aiortc from unitree requirements would pull 50.x)
  #   pyarrow==21.0.0      : rerun-sdk pulls pyarrow; >=22 refuses to import with numpy 1.x
  #   psutil==5.9.8, ipython<9 : isaacsim-kernel pin; ipython 9 (via isaaclab_mimic) needs psutil>=7
  #   starlette==0.49.1    : Isaac Lab pin (security fix, IsaacLab 64ecea24f). Conflicts with isaacsim-kernel's
  #                          fastapi 0.115.7 (starlette<0.46) exactly as in the official isaaclab.sh install;
  #                          that single known line is tolerated by the pip check below.
  printf '%s\n' numpy==1.26.0 torch==2.7.0+cu128 torchvision==0.22.0+cu128 torchaudio==2.7.0+cu128 packaging==23.0 \
    pillow==11.3.0 cryptography==44.0.0 pyarrow==21.0.0 starlette==0.49.1 psutil==5.9.8 "ipython<9" > "$G1IV_WS/envs/constraints_sim.txt"
  local C=(-c "$G1IV_WS/envs/constraints_sim.txt" --extra-index-url https://download.pytorch.org/whl/cu128)
  # all source packages, as "isaaclab.sh --install" does (isaaclab_rl brings h5py, which isaaclab.managers
  # imports unconditionally); the optional RL-framework extras are not installed.
  for p in isaaclab isaaclab_assets isaaclab_contrib isaaclab_mimic isaaclab_rl isaaclab_tasks; do
    "$PY" -m pip show "$p" >/dev/null 2>&1 || pipi $E "${C[@]}" -e "$R/IsaacLab/source/$p"
  done
  pipi $E "${C[@]}" -e "$R/unitree_sdk2_python"
  pipi $E "${C[@]}" -r "$R/unitree_sim_isaaclab/requirements.txt"
  pipi $E "${C[@]}" -e "$R/unitree_sim_isaaclab/teleimager[server]"
  pipi $E "${C[@]}" pyyaml
  # re-apply the pins in case an earlier run left other versions behind
  pipi $E "${C[@]}" cryptography==44.0.0 pyarrow==21.0.0 starlette==0.49.1 psutil==5.9.8 "ipython<9"
  "$PY" -m pip check > "$L/sim_pip_check.txt" || true
  cat "$L/sim_pip_check.txt"
  if grep -v -E '^fastapi 0\.115\.7 has requirement starlette<0\.46\.0,>=0\.40\.0, but you have starlette 0\.49\.1\.$' \
       "$L/sim_pip_check.txt" | grep -v -E '^No broken requirements found\.$' | grep . >/dev/null; then
    echo "pip check: unexpected conflicts in $E (see above)" >&2; exit 1
  fi
}

setup_teleop() {
  local E=g1_inspire_teleop PY
  exists $E || "$M" create -y -r "$MAMBA_ROOT_PREFIX" -n $E -c conda-forge --override-channels \
      python=3.10 pinocchio=3.1.0 numpy=1.26.4 pip openssl pyyaml pytest
  PY=$(g1iv_py $E)
  # params-proto 3.x dropped Flag, which vuer 0.0.60 (televuer) imports; vuer only asks for >=2.13.0
  # pyarrow: rerun-sdk (xr_teleoperate requirements) pulls pyarrow, and >=22 refuses to import with numpy 1.x
  printf 'numpy==1.26.4\nparams-proto==2.13.2\npyarrow==21.0.0\n' > "$G1IV_WS/envs/constraints_teleop.txt"
  local C=(-c "$G1IV_WS/envs/constraints_teleop.txt") X="$R/xr_teleoperate/teleop"
  pipi $E "${C[@]}" -e "$R/unitree_sdk2_python"
  pipi $E "${C[@]}" -e "$X/teleimager" --no-deps
  pipi $E "${C[@]}" logging_mp pyzmq pyyaml opencv-python
  pipi $E "${C[@]}" -e "$X/televuer"
  pipi $E "${C[@]}" -r "$R/xr_teleoperate/requirements.txt"
  pipi $E "${C[@]}" pygame==2.6.1 pynput==1.8.1 pyarrow==21.0.0
  "$PY" -m pip check | tee "$L/teleop_pip_check.txt"
}

setup_data() {
  local E=g1_inspire_data PY
  exists $E || "$M" create -y -r "$MAMBA_ROOT_PREFIX" -n $E -c conda-forge --override-channels python=3.11 pip pyyaml
  PY=$(g1iv_py $E)
  if ! "$PY" -c "import torch" 2>/dev/null; then   # same cp311 torch build as the sim env, hard-linked
    "$PY" -I "$G1IV_WS/scripts/setup/link_dists.py" --src "$MAMBA_ROOT_PREFIX/envs/g1_inspire_sim" --dst "$MAMBA_ROOT_PREFIX/envs/$E" \
        --report "$L/link_torch_data_report.json" torch==2.7.0+cu128 torchvision==0.22.0+cu128
  fi
  # numpy<2 matches the sim env. pyarrow>=22 aborts at import with numpy 1.x (not declared in its metadata,
  # so pip check passes); huggingface_hub 1.x/2.x and pandas 3 postdate lerobot 0.3.3 / datasets 3.6.
  printf 'torch==2.7.0+cu128\ntorchvision==0.22.0+cu128\nnumpy<2\npyarrow==21.0.0\nhuggingface-hub<1.0\npandas<3\n' \
    > "$G1IV_WS/envs/constraints_data.txt"
  pipi $E -c "$G1IV_WS/envs/constraints_data.txt" --extra-index-url https://download.pytorch.org/whl/cu128 lerobot==0.3.3 pyarrow
  "$PY" -m pip check | tee "$L/data_pip_check.txt"
}

setup_vla() {
  # LOADER-ONLY environment: checks that converted datasets load through GR00T's own LeRobotEpisodeLoader
  # (modality.json, parquet, AV1 video via torchcodec + system FFmpeg 6). CPU torch, no model weights,
  # no flash-attn / deepspeed / TensorRT. The full GR00T N1.7 stack (torch 2.9.0 CUDA, flash-attn 2.8.3;
  # inference >=16 GB VRAM, default fine-tuning ~35 GB/GPU) belongs on the training server:
  #   cd repos/Isaac-GR00T && pip install uv && uv pip install -e .
  local E=g1_inspire_vla PY
  exists $E || "$M" create -y -r "$MAMBA_ROOT_PREFIX" -n $E -c conda-forge --override-channels python=3.12 pip
  PY=$(g1iv_py $E)
  printf '%s\n' numpy==1.26.4 pandas==2.2.3 torch==2.9.0+cpu torchvision==0.24.0+cpu > "$G1IV_WS/envs/constraints_vla.txt"
  pipi $E -c "$G1IV_WS/envs/constraints_vla.txt" --index-url https://download.pytorch.org/whl/cpu torch==2.9.0+cpu torchvision==0.24.0+cpu
  pipi $E -c "$G1IV_WS/envs/constraints_vla.txt" numpy==1.26.4 pandas==2.2.3 pyarrow==21.0.0 torchcodec==0.8.0 \
      dm-tree==0.1.9 omegaconf==2.3.0 termcolor==3.2.0 pyyaml
  pipi $E -c "$G1IV_WS/envs/constraints_vla.txt" --no-deps -e "$R/Isaac-GR00T"
  "$PY" -m pip check > "$L/vla_pip_check.txt" 2>&1 || true   # gr00t's full dependency list is intentionally absent
  echo "vla env (loader-only) ready; pip check (expected to list GR00T's omitted deps): $L/vla_pip_check.txt"
}

setup_cad() {
  # tools/cad (Inspire RH56E2 URDF/STL -> data/assets/inspire_rh56e2/rh56e2_hands.npz, docs/HAND_RH56E2.md).
  # Same pins as the 2026-11-12 build (then a throw-away venv of the system Python 3.12). gmsh (STEP import for the
  # camera-rig bracket tools) is optional: G1IV_CAD_GMSH=1.
  local E=g1_inspire_cad
  exists $E || "$M" create -y -r "$MAMBA_ROOT_PREFIX" -n $E -c conda-forge --override-channels python=3.12 pip
  pipi $E numpy==2.5.4 scipy==1.18.1 trimesh==5.2.0 opencv-python-headless==5.0.0.93 networkx==3.7 rtree==1.4.1 \
      shapely==2.2.0 pyyaml==6.0.3 ${G1IV_CAD_GMSH:+gmsh==4.15.2}
}

case "${1:-}" in
  sim) setup_sim ;; teleop) setup_teleop ;; data) setup_data ;; vla) setup_vla ;; cad) setup_cad ;;
  all) setup_sim; setup_teleop; setup_data; setup_cad ;;
  *) echo "usage: $0 sim|teleop|data|vla|cad|all" >&2; exit 2 ;;
esac
