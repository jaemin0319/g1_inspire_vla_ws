"""Right Inspire hand geometry relative to the official IK end-effector R_ee (= right_wrist_yaw_link + 0.05 m
along its local x, xr_teleoperate robot_arm_ik.py), from the robot USD at zero joint angles. Read-only."""
import sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom  # noqa: E402

stage = Usd.Stage.Open(sys.argv[1], Usd.Stage.LoadAll)
cache = UsdGeom.XformCache()
links = {}
for p in stage.Traverse():
    n = p.GetName()
    if p.IsA(UsdGeom.Xformable) and (n.startswith("R_") or n.startswith("right_wrist") or n.startswith("right_hand")):
        M = np.array(cache.GetLocalToWorldTransform(p)).T   # USD row-major -> column-vector convention
        links[n] = M
W = links["right_wrist_yaw_link"]
Ree = W @ np.array([[1, 0, 0, 0.05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
inv = np.linalg.inv(Ree)
print("R_ee axes in wrist frame: x=fwd along forearm/hand")
for n in sorted(links):
    if n in ("right_wrist_yaw_link",):
        continue
    p = (inv @ links[n])[:3, 3]
    print(f"{n:32s} pos in R_ee frame: {np.round(p, 3).tolist()}")
# hand base axes in R_ee frame
for n in ("right_hand_base_link", "R_hand_base_link"):
    if n in links:
        R = (inv @ links[n])[:3, :3]
        print(n, "axes (cols x,y,z) in R_ee frame:\n", np.round(R, 3))

app.close()
