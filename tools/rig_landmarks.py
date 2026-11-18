"""Arm landmarks for the head-camera extrinsic fit.

Landmarks are 3D points on the arms computed from the rendered robot's link poses (render_camera_rig.py
poses.json, i.e. the same kinematics as the simulator at the real joint values). They are projected into
the rig cameras (OpenCV model incl. distortion) and drawn on the simulated and real images, so the same
physical spots can be annotated in the real frames (logs/camera_rig/annotations.json).

  <data python> tools/rig_landmarks.py --poses logs/camera_rig/render_v0/poses.json --frames-dir logs/camera_rig/frames \
      --render logs/camera_rig/render_v0 --out logs/camera_rig/landmarks [--names ep0_f300 ...] [--set head.pitch_down_deg=40]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from g1iv_common.config import load_config  # noqa: E402
from g1iv_sim.camera_rig import build_rig, project_points  # noqa: E402

# name -> (body, point in body frame [m])
LANDMARKS = {}
for s, P in (("left", "L"), ("right", "R")):
    LANDMARKS[f"{s}_elbow"] = (f"{s}_elbow_link", (0.0, 0.0, 0.0))
    LANDMARKS[f"{s}_wpitch"] = (f"{s}_wrist_pitch_link", (0.0, 0.0, 0.0))
    LANDMARKS[f"{s}_ring"] = (f"{s}_wrist_yaw_link", (0.032, 0.0, 0.0))
    LANDMARKS[f"{s}_palm"] = (f"{s}_wrist_yaw_link", (0.10, 0.0, 0.0))
    LANDMARKS[f"{s}_midpip"] = (f"{P}_middle_intermediate", (0.0, 0.0, 0.0))


def pose_to_T(p):
    x, y, z, qw, qx, qy, qz = p
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                  [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, (x, y, z)
    return T


def landmarks_world(bodies: dict) -> dict:
    out = {}
    for n, (b, p) in LANDMARKS.items():
        out[n] = (pose_to_T(bodies[b]) @ np.r_[p, 1.0])[:3]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", required=True)
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--render", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--names", nargs="*")
    ap.add_argument("--rig", default="configs/camera_rig.yaml")
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--annotations", default="logs/camera_rig/annotations.json")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    P = json.load(open(a.poses))
    rig = {c.name: c for c in build_rig(load_config(a.rig, a.set))}
    ann = json.load(open(a.annotations)) if os.path.exists(a.annotations) else {}
    cam = rig["front_camera"]
    for name in a.names or list(P["frames"]):
        fr = P["frames"][name]
        Tt = pose_to_T(fr["bodies"][cam.parent_link])
        lw = landmarks_world(fr["bodies"])
        keys = list(lw)
        uv, z = project_points(cam, Tt, np.array([lw[k] for k in keys]))
        base = name.replace("_ypr", "")
        imgs = []
        for src in (f"{a.render}/{name}_front_camera.png", f"{a.frames_dir}/{base}_real_head_stereo_left.png"):
            im = cv2.imread(src)
            if im is None:
                continue
            for k, (u, v), d in zip(keys, uv, z):
                if d <= 0:
                    continue
                c = (0, 200, 255) if k.startswith("left") else (255, 120, 0)
                cv2.circle(im, (int(round(u)), int(round(v))), 5, c, 2)
                cv2.putText(im, k.split("_", 1)[1], (int(u) + 6, int(v) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, c, 1, cv2.LINE_AA)
            for k, (u, v) in ann.get(base, {}).items():   # annotated real points (green)
                cv2.drawMarker(im, (int(u), int(v)), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)
            for g in range(0, im.shape[1], 40):
                cv2.line(im, (g, 0), (g, im.shape[0]), (80, 80, 80), 1)
                cv2.putText(im, str(g), (g + 1, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
            for g in range(0, im.shape[0], 40):
                cv2.line(im, (0, g), (im.shape[1], g), (80, 80, 80), 1)
                cv2.putText(im, str(g), (1, g + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
            imgs.append(im)
        cv2.imwrite(f"{a.out}/{name}_head_landmarks.png", np.hstack(imgs))
        print(name, {k: [round(float(u), 1), round(float(v), 1)] for k, (u, v) in zip(keys, uv)})


if __name__ == "__main__":
    main()
