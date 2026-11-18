"""Unitree G1 head stereo-camera bracket (xr_teleoperate Device.md 5.2.1, 60 FPS, Classic head 98 mm:
stero-camera-mount-60hz-98 + stero-camera-cover-60hz-98) as one mesh in its CAD frame [m] + the alignment features.
The simulator (camera_rig.head_bracket_pose) puts the two lens holes on the head optical centres and the
camera-plate normal on the optical axis; the M4 screw holes then land on the head's internal top plate.

The cover STEP is not exported in its assembled place: its 4 screw holes coincide with the mount's y = +-39 mm holes
after a pure translation (-3.5, +4.58, +58.56) mm, which also centres its lens holes (Ø18) at y = +-30 mm.
CAD frame: -x forward, +y = robot RIGHT, +z up (right-handed).
usage: make_head_bracket_asset.py STL_DIR OUT_NPZ"""
import sys

import numpy as np
import trimesh

stl, out = sys.argv[1:3]
COVER_T = np.array([-3.5, 4.58, 58.56])
P, F, off = [], [], 0
for name, pre in (("stero-camera-mount-60hz-98", np.zeros(3)), ("stero-camera-cover-60hz-98", COVER_T)):
    m = trimesh.load(f"{stl}/{name}.stl", force="mesh"); m.merge_vertices()
    P.append((m.vertices + pre) / 1000.0); F.append(m.faces + off); off += len(m.vertices)
axis = np.array([-0.652, 0.0, -0.758]); axis /= np.linalg.norm(axis)
np.savez_compressed(out, points=np.vstack(P).astype(np.float32), faces=np.vstack(F).astype(np.int32),
                    lens_holes_cad=np.array([[-71.72, -30.0, 98.2], [-71.72, 30.0, 98.2]]) / 1000.0,  # [robot-left, robot-right]
                    lens_axis_cad=axis, baseline_dir_cad=np.array([0.0, 1.0, 0.0]),
                    screws_cad=np.array([[-14.22, 49.0, 92.11], [-14.22, -49.0, 92.11]]) / 1000.0)
print("saved", out, sum(len(p) for p in P), "verts")
