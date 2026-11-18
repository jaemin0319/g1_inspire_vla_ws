"""Pick frames from a Unitree LeRobot v3.0 dataset: joint configuration (for tools/render_camera_rig.py) + the real
head-stereo / wrist images (ffmpeg seek on the HF video, no full download).

  <data python> tools/unitree_frames.py --root data/reference/unitree_hf/G1_WBT_Inspire_Sort_Fruit \
      --repo unitreerobotics/G1_WBT_Inspire_Sort_Fruit --pick 0:0 0:300 --out logs/camera_rig/frames [--wrist-order ypr]
  <data python> tools/unitree_frames.py --layout dex3 --root data/reference/unitree_hf/G1_Dex3_BlockStacking_Dataset \
      --repo unitreerobotics/G1_Dex3_BlockStacking_Dataset --pick 0:60 ... --out logs/camera_rig/dex3_frames

Dataset conventions (dataset card + checks in docs/CAMERA_RIG.md):
  wbt_inspire (G1_WBT_Inspire_*):
    lower_body = 6 left leg, 6 right leg, waist yaw/roll/pitch;  left/right_arm = 4 shoulder/elbow + 3 wrist
    hand_state = left[index, middle, ring, little, thumb_bend, thumb_rotate] + right[...], 1.0 = open
    Inspire command -> joint angle exactly as unitree_sim_isaaclab dds/inspire_dds.py (denormalize).
  dex3 (G1_Dex3_*, fixed base, D405 wrist cameras): observation.state[28] = 7 left arm + 7 right arm (shoulder
    pitch/roll/yaw, elbow, wrist roll/pitch/yaw) + left Dex3 [thumb0..2, middle0..1, index0..1] + right Dex3
    [thumb0..2, index0..1, middle0..1] in rad (info.json names; ranges match the Dex3 joint limits).
Images are saved under the canonical names <frame>_real_{head_stereo_left,head_stereo_right,wrist_left,wrist_right}.png.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess

import numpy as np
import pandas as pd

LEGS = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
ARM4 = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"]
WRIST = {"rpy": ["wrist_roll", "wrist_pitch", "wrist_yaw"],   # G1 motor order
         "ypr": ["wrist_yaw", "wrist_roll", "wrist_pitch"]}   # order as labelled in the v3.0 info.json
FINGERS = ["index", "middle", "ring", "pinky"]
VIDEOS = ["head_stereo_left", "head_stereo_right", "wrist_left", "wrist_right"]
DEX3_VIDEOS = {"head_stereo_left": "cam_left_high", "head_stereo_right": "cam_right_high",
               "wrist_left": "cam_left_wrist", "wrist_right": "cam_right_wrist"}
ARM7 = ARM4 + WRIST["rpy"]
DEX3_HAND = {"left": ["thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"],
             "right": ["thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1"]}


def dex3_frame_joints(r) -> dict:
    s = np.asarray(r["observation.state"], dtype=float)
    j = {}
    for i, n in enumerate(ARM7):
        j[f"left_{n}_joint"] = s[i]
        j[f"right_{n}_joint"] = s[7 + i]
    for k, n in enumerate(DEX3_HAND["left"]):
        j[f"left_hand_{n}_joint"] = s[14 + k]
    for k, n in enumerate(DEX3_HAND["right"]):
        j[f"right_hand_{n}_joint"] = s[21 + k]
    return {k: float(v) for k, v in j.items()}


def inspire_joints(prefix: str, h6) -> dict:
    """h6 = [index, middle, ring, little, thumb_bend, thumb_rotate] normalised, 1 = open."""
    den = lambda q, lo, hi: (1.0 - float(np.clip(q, 0, 1))) * (hi - lo) + lo  # noqa: E731
    j = {}
    for f, q in zip(FINGERS, h6[:4]):
        a = den(q, 0.0, 1.7)
        j[f"{prefix}_{f}_proximal_joint"] = a
        j[f"{prefix}_{f}_intermediate_joint"] = a  # official mimic factor 1
    tb = den(h6[4], 0.0, 0.5)
    j[f"{prefix}_thumb_proximal_pitch_joint"] = tb
    j[f"{prefix}_thumb_intermediate_joint"] = 1.5 * tb
    j[f"{prefix}_thumb_distal_joint"] = 2.4 * tb
    j[f"{prefix}_thumb_proximal_yaw_joint"] = den(h6[5], -0.1, 1.3)
    return j


def frame_joints(r, wrist_order: str) -> dict:
    lb = np.asarray(r["observation.state.lower_body"], dtype=float)
    j = {}
    for i, n in enumerate(LEGS):
        j[f"left_{n}_joint"] = lb[i]
        j[f"right_{n}_joint"] = lb[6 + i]
    j["waist_yaw_joint"], j["waist_roll_joint"], j["waist_pitch_joint"] = lb[12], lb[13], lb[14]
    for side in ("left", "right"):
        a = np.asarray(r[f"observation.state.{side}_arm"], dtype=float)
        for n, v in zip(ARM4 + WRIST[wrist_order], a):
            j[f"{side}_{n}_joint"] = v
    hs = np.asarray(r["observation.state.hand_state"], dtype=float)
    j.update(inspire_joints("L", hs[:6]))
    j.update(inspire_joints("R", hs[6:]))
    return {k: float(v) for k, v in j.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--pick", nargs="+", required=True, help="episode:frame ...")
    ap.add_argument("--out", required=True)
    ap.add_argument("--wrist-order", default="rpy", choices=list(WRIST))
    ap.add_argument("--no-images", action="store_true")
    ap.add_argument("--layout", default="wbt_inspire", choices=["wbt_inspire", "dex3"])
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    eps = pd.read_parquet(os.path.join(a.root, "meta/episodes/chunk-000/file-000.parquet"))
    cache = {}
    frames = []
    for pk in a.pick:
        ep, fi = (int(v) for v in pk.split(":"))
        e = eps[eps.episode_index == ep].iloc[0]
        dfile = os.path.join(a.root, f"data/chunk-{int(e['data/chunk_index']):03d}/file-{int(e['data/file_index']):03d}.parquet")
        if dfile not in cache:
            cache[dfile] = pd.read_parquet(dfile)
        d = cache[dfile]
        r = d[(d.episode_index == ep) & (d.frame_index == fi)].iloc[0]
        name = f"ep{ep}_f{fi}"
        if a.layout == "dex3":
            rec = {"name": name, "episode": ep, "frame": fi, "timestamp": float(r["timestamp"]), "layout": "dex3",
                   "joints": dex3_frame_joints(r), "state": np.asarray(r["observation.state"]).tolist()}
        else:
            rec = {"name": name, "episode": ep, "frame": fi, "timestamp": float(r["timestamp"]), "wrist_order": a.wrist_order,
                   "joints": frame_joints(r, a.wrist_order),
                   "hand_state": np.asarray(r["observation.state.hand_state"]).tolist(),
                   "state_d435": np.asarray(r["observation.state.state_d435"]).tolist(),
                   "state_torso": np.asarray(r["observation.state.state_torso"]).tolist(),
                   "left_ee_torso": np.asarray(r["observation.state.left_ee_pose_gripper_torso"]).tolist(),
                   "right_ee_torso": np.asarray(r["observation.state.right_ee_pose_gripper_torso"]).tolist()}
        frames.append(rec)
        if a.no_images:
            continue
        for v in VIDEOS:
            src = DEX3_VIDEOS[v] if a.layout == "dex3" else v
            key = f"videos/observation.images.{src}"
            t = float(e[f"{key}/from_timestamp"]) + fi / 30.0
            url = (f"https://huggingface.co/datasets/{a.repo}/resolve/main/videos/observation.images.{src}/"
                   f"chunk-{int(e[f'{key}/chunk_index']):03d}/file-{int(e[f'{key}/file_index']):03d}.mp4")
            dst = os.path.join(a.out, f"{name}_real_{v}.png")
            if not os.path.exists(dst):
                subprocess.run(["ffmpeg", "-nostdin", "-loglevel", "error", "-y", "-ss", f"{t:.4f}", "-i", url,
                                "-frames:v", "1", dst], check=True, timeout=300)
        print("frame", name, flush=True)
    json.dump(frames, open(os.path.join(a.out, f"frames_{a.wrist_order}.json"), "w"), indent=1)
    print("wrote", len(frames), "frames")


if __name__ == "__main__":
    main()
