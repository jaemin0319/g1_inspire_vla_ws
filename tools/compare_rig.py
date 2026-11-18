"""Side-by-side real (Unitree dataset) | simulated (render_camera_rig.py) | 50 % blend, per frame and camera.

  <data python> tools/compare_rig.py --frames-dir logs/camera_rig/frames --render logs/camera_rig/render_v1 \
      --names ep0_f0 ep0_f300 --out logs/camera_rig/compare_v1.jpg [--suffix _ypr]
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

PAIRS = [("head_stereo_left", "front_camera"), ("head_stereo_right", "front_right_camera"),
         ("wrist_left", "left_wrist_camera"), ("wrist_right", "right_wrist_camera")]


def label(img, text):
    img = img.copy()
    cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(img, text, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--render", required=True)
    ap.add_argument("--names", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--suffix", default="")
    ap.add_argument("--cams", default="head_stereo_left,wrist_left,wrist_right")
    ap.add_argument("--scale", type=float, default=0.5)
    a = ap.parse_args()
    want = a.cams.split(",")
    rows = []
    for n in a.names:
        for real_key, sim_key in PAIRS:
            if real_key not in want:
                continue
            r = cv2.imread(f"{a.frames_dir}/{n}_real_{real_key}.png")
            s = cv2.imread(f"{a.render}/{n}{a.suffix}_{sim_key}.png")
            if r is None or s is None:
                continue
            b = cv2.addWeighted(r, 0.5, s, 0.5, 0)
            row = np.hstack([label(r, f"{n} real {real_key}"), label(s, f"sim {sim_key}"), label(b, "blend")])
            rows.append(cv2.resize(row, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA))
    cv2.imwrite(a.out, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 88])
    print("saved", a.out, len(rows), "rows")


if __name__ == "__main__":
    main()
