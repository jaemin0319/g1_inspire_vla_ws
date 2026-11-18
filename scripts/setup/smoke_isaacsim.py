"""Isaac Sim 5.1 smoke test: start headless, render one 640x480 RGB frame, report GPU/renderer state.

Usage (g1_inspire_sim env):  python scripts/setup/smoke_isaacsim.py --out logs/setup/smoke_isaacsim_<gpu>.json
Only NVIDIA's pip-installed Isaac Sim is used (no Isaac Lab / Unitree code).
"""
import argparse
import json
import os
import subprocess
import time

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--frames", type=int, default=30)
args = ap.parse_args()

t0 = time.time()
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 640, "height": 480})
t_boot = time.time() - t0

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1000.0)
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.5))
cam = rep.create.camera(position=(3, 3, 2), look_at=(0, 0, 0.5))
rp = rep.create.render_product(cam, (640, 480))
annot = rep.AnnotatorRegistry.get_annotator("rgb")
annot.attach([rp])

t1 = time.time()
shape, mean = None, None
for _ in range(args.frames):
    rep.orchestrator.step(rt_subframes=1, pause_timeline=False)
data = annot.get_data()
if data is not None and hasattr(data, "shape"):
    shape, mean = list(data.shape), float(np.asarray(data)[..., :3].mean())
t_render = time.time() - t1

s = carb.settings.get_settings()
gpu = subprocess.run(["nvidia-smi", "--query-gpu=index,name,memory.used", "--format=csv,noheader"],
                     capture_output=True, text=True).stdout.strip().splitlines()
rep_ = {
    "boot_s": round(t_boot, 1),
    "render_s_for_frames": round(t_render, 2),
    "frames": args.frames,
    "rgb_shape": shape,
    "rgb_mean": mean,
    "CUDA_VISIBLE_DEVICES": os.environ.get("CUDA_VISIBLE_DEVICES"),
    "settings": {k: s.get(k) for k in ("/renderer/activeGpu", "/renderer/multiGpu/enabled",
                                        "/physics/cudaDevice", "/app/version", "/crashreporter/gpu")},
    "nvidia_smi_during": gpu,
}
os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
with open(args.out, "w") as f:
    json.dump(rep_, f, indent=1)
print(json.dumps(rep_))
app.close()
os._exit(0)
