#!/usr/bin/env bash
# Freeze what is ACTUALLY installed: per env conda yaml + explicit spec + pip freeze, and the SHAs of every
# repo / submodule (+ dirty flag) and of the downloaded assets/policies. Writes into envs/.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_strip_ros
O="$G1IV_WS/envs"; mkdir -p "$O"
for e in g1_inspire_sim g1_inspire_teleop g1_inspire_data g1_inspire_vla g1_inspire_cad; do
  [[ -x "$(g1iv_py $e)" ]] || { echo "skip $e (not created)"; continue; }
  "$G1IV_MAMBA" env export -r "$MAMBA_ROOT_PREFIX" -n $e | grep -v "^prefix:" > "$O/$e.yaml"   # no machine path
  "$G1IV_MAMBA" env export -r "$MAMBA_ROOT_PREFIX" -n $e --explicit > "$O/$e.explicit.txt" 2>/dev/null || true
  "$(g1iv_py $e)" -m pip freeze --all > "$O/$e.pip-freeze.txt"
  "$(g1iv_py $e)" -m pip check > "$O/$e.pip-check.txt" 2>&1 || true
  echo "exported $e"
done
python3 - "$G1IV_WS" > "$O/repos.lock.json" <<'EOF'
import json, subprocess, sys, pathlib
ws = pathlib.Path(sys.argv[1])
def g(p, *a):
    return subprocess.run(["git", "-C", str(p), *a], capture_output=True, text=True).stdout.strip()
out = {}
for r in sorted((ws / "repos").iterdir()):
    if not (r / ".git").exists():
        continue
    out[r.name] = {"remote": g(r, "remote", "get-url", "origin"), "commit": g(r, "rev-parse", "HEAD"),
                   "branch": g(r, "rev-parse", "--abbrev-ref", "HEAD"), "describe": g(r, "describe", "--tags", "--always"),
                   "dirty_files": g(r, "status", "--porcelain", "--untracked-files=no").splitlines(),
                   "submodules": g(r, "submodule", "status", "--recursive").splitlines()}
assets = ws / "repos/unitree_sim_isaaclab/assets"
out["_assets"] = {"source": "hf datasets/unitreerobotics/unitree_sim_isaaclab_usds assets.zip",
                  "revision": (ws / ".cache/assets_dl/REVISION").read_text().strip() if (ws / ".cache/assets_dl/REVISION").exists() else None,
                  "policy_sha256": (ws / "policies/official_policy_sha256.txt").read_text().strip().splitlines()
                  if (ws / "policies/official_policy_sha256.txt").exists() else None}
print(json.dumps(out, indent=1))
EOF
# source changes only: the upstream repo tracks __pycache__/*.pyc that Python rewrites (binary, not appliable)
for r in unitree_sim_isaaclab; do git -C "$G1IV_WS/repos/$r" diff -- . ":(exclude)*.pyc" > "$O/patches/${r}_g1iv.patch"; done
echo "wrote $O/{*.yaml,*.explicit.txt,*.pip-freeze.txt,repos.lock.json,patches/}"
