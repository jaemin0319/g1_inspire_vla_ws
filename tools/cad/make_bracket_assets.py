"""Place Unitree's Inspire/Brainco wrist-camera bracket (xr_teleoperate Device.md 5.2.4, g1-wrist/inspire_or_brainco
STEP -> STL via step_to_stl.py) in the <side>_wrist_yaw_link frame and save visual meshes for the simulator.

Assembly (docs/CAMERA_RIG.md): ring axis = hand axis (+x), ring centre at x = 32 mm (the ring clamps the r = 30.5 mm
wrist-yaw housing next to the wrist/hand seam), camera box on the palm side (+y right hand, -y left hand), lens facing
the fingers; the separate half ring is the mount's half turned 180 deg about the ring axis.
usage: make_bracket_assets.py STL_DIR OUT_DIR"""
import os
import sys

import numpy as np
import trimesh

stl, out = sys.argv[1], sys.argv[2]
RING_X = 0.032
# CAD ring frame -> wrist frame (columns = images of CAD X, Y, Z)
R = {"right": np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float),
     "left": np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)}
ROT180Z = np.diag([-1.0, -1.0, 1.0])
MIRZ = np.diag([1.0, 1.0, -1.0])
parts = {"right": [("g1-camera-mount-inspire-brainco-right", np.eye(3)), ("g1-camera-ring-inspire-brainco", ROT180Z),
                   ("g1-camera-cover-inspire-brainco", np.eye(3))],
         "left": [("g1-camera-mount-inspire-brainco-left", np.eye(3)), ("g1-camera-ring-inspire-brainco", ROT180Z),
                  ("g1-camera-cover-inspire-brainco", MIRZ)]}
os.makedirs(out, exist_ok=True)
for side, plist in parts.items():
    P, F, off = [], [], 0
    for name, pre in plist:
        m = trimesh.load(os.path.join(stl, name + ".stl"), force="mesh")
        m.merge_vertices()
        v = (R[side] @ (pre @ (m.vertices / 1000.0).T)).T + np.array([RING_X, 0.0, 0.0])
        f = m.faces.copy()
        if np.linalg.det(R[side] @ pre) < 0:
            f = f[:, ::-1]                       # keep outward normals after a mirror
        P.append(v); F.append(f + off); off += len(v)
    P, F = np.vstack(P), np.vstack(F)
    lens = R[side] @ np.array([30.49, -0.32, -6.0 if side == "right" else 6.0]) / 1000 + np.array([RING_X, 0, 0])
    np.savez_compressed(os.path.join(out, f"inspire_wrist_bracket_{side}.npz"), points=P.astype(np.float32),
                        faces=F.astype(np.int32), lens_hole_centre=lens)
    print(side, "verts", len(P), "faces", len(F), "bbox", P.min(0).round(4).tolist(), P.max(0).round(4).tolist(), "lens", lens.round(4).tolist())
