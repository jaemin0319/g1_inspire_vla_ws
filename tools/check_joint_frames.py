"""Joint-frame consistency of a robot USD at the authored pose: for every joint, body0 * localPose0 vs body1 *
localPose1 (position [mm], rotation [deg]); for revolute joints also the angle about the axis. Read-only.
Usage: check_joint_frames.py ROBOT.usd [name-substring ...]"""
import math
import sys

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

st = Usd.Stage.Open(sys.argv[1], Usd.Stage.LoadAll)
keys = sys.argv[2:]
cache = UsdGeom.XformCache()


def qmat(q):
    w, x, y, z = q.GetReal(), *q.GetImaginary()
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


worst = (0, 0, "")
for p in st.Traverse():
    if not p.IsA(UsdPhysics.Joint) or (keys and not any(k in p.GetName() for k in keys)):
        continue
    j = UsdPhysics.Joint(p)
    b0, b1 = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
    if not b0 or not b1:
        continue
    T0 = np.array(cache.GetLocalToWorldTransform(st.GetPrimAtPath(b0[0]))).T
    T1 = np.array(cache.GetLocalToWorldTransform(st.GetPrimAtPath(b1[0]))).T
    J0 = np.eye(4); J0[:3, :3] = qmat(j.GetLocalRot0Attr().Get()); J0[:3, 3] = np.array(j.GetLocalPos0Attr().Get())
    J1 = np.eye(4); J1[:3, :3] = qmat(j.GetLocalRot1Attr().Get()); J1[:3, 3] = np.array(j.GetLocalPos1Attr().Get())
    A, B = T0 @ J0, T1 @ J1
    dp = np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000
    Rr = A[:3, :3].T @ B[:3, :3]
    ang = math.degrees(math.acos(np.clip((np.trace(Rr) - 1) / 2, -1, 1)))
    print(f"{p.GetName():34s} dpos {dp:7.3f} mm  drot {ang:7.3f} deg")
    if dp + ang > worst[0] + worst[1]:
        worst = (dp, ang, p.GetName())
print("WORST", worst, flush=True)
import os  # noqa: E402
os._exit(0)
