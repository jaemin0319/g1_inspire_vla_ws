"""Camera-related links of the robot USD: pose of each *camera*/d435/head link relative to its parent wrist/torso
link at zero joint angles, and whether it carries visual geometry. Read-only."""
import sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom  # noqa: E402

stage = Usd.Stage.Open(sys.argv[1], Usd.Stage.LoadAll)
cache = UsdGeom.XformCache()
links = {}
for p in stage.Traverse(Usd.TraverseInstanceProxies()):
    if p.GetPath().pathElementCount != 2:
        continue
    if p.IsA(UsdGeom.Xformable):
        links[p.GetName()] = (p, np.array(cache.GetLocalToWorldTransform(p)).T)

def rel(a, b):
    return np.linalg.inv(links[a][1]) @ links[b][1]

def rpy(R):
    return np.degrees([np.arctan2(R[2, 1], R[2, 2]), -np.arcsin(np.clip(R[2, 0], -1, 1)), np.arctan2(R[1, 0], R[0, 0])])

print("top-level links:", len(links))
for n in sorted(links):
    if any(k in n.lower() for k in ("camera", "d435", "head", "mid360", "imu", "logo", "torso")):
        p = links[n][0]
        kids = [str(c.GetPath()) for c in Usd.PrimRange(p, Usd.TraverseInstanceProxies()) if c.IsA(UsdGeom.Gprim)]
        print(f"\n{n}: world pos {np.round(links[n][1][:3, 3], 4).tolist()} visual gprims={len(kids)} {kids[:4]}")
        for ref in ("torso_link", "left_wrist_yaw_link", "right_wrist_yaw_link", "L_hand_base_link", "R_hand_base_link", "left_hand_base_link", "right_hand_base_link"):
            if ref in links and (("left" in n or "L_" in n) == ("left" in ref or "L_" in ref) or "torso" in ref):
                T = rel(ref, n)
                print(f"   in {ref:22s}: pos {np.round(T[:3, 3], 5).tolist()} rpy_deg {np.round(rpy(T[:3, :3]), 2).tolist()}")
                if "torso" in ref and ("camera" in n and "hand" in n):
                    pass
sys.stdout.flush(); import os; os._exit(0)
