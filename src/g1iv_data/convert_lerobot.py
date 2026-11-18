"""Raw sim episodes -> LeRobot dataset v2.1 ("GR00T LeRobot" flavour: + meta/modality.json, meta/stats.json).

Format choice: Isaac-GR00T (N1.7, 3b49067) loads LeRobot **v2** datasets with meta/modality.json and
documents v3.0 -> v2 conversion; it is the VLA target, so v2.1 (lerobot==0.3.3 writer/loader) is pinned.
unitree_lerobot (41c2805, lerobot 0.4.1 / v3.0) was reviewed: its converter reads xr_teleoperate JSON,
not this simulator recording, but its G1_Inspire feature layout is reused here:
  state/action = [L arm 7, R arm 7, L hand 6, R hand 6]   (arm rad; hand 0..1, 1 = open, Inspire DFX order
                  pinky, ring, middle, index, thumb bend, thumb rotation)
  cameras      = Unitree G1 keys: cam_left_high (+ cam_right_high when the head stereo right eye was recorded),
                 cam_left_wrist, cam_right_wrist  (the keys UnifoLM-VLA's prepare_data reads)
  meta/info.json "camera_params" = intrinsics / stereo extrinsic in Unitree's dataset layout, plus the camera
  mounts ("extrinsics", OpenCV frames in their parent links) from the recording metadata (docs/CAMERA_RIG.md)

The VLA action is upper body + hands only. Walking command, binary OPEN/CLOSE, base velocity and
evaluation labels are kept as SEPARATE columns (not inside "action"):
  action.hand_binary (2: L, R; 1=CLOSE, -1=unknown) , action.loco_cmd (4: vx, vy, wz, height)
  observation.base_vel (6: lin_b xyz, ang_b xyz) , observation.sim_time (1) , observation.wall_time (1)
  eval.object_pos_w (3) , eval.contact_force_total (1) , eval.fallen (1)    <- NOT policy inputs

  python -m g1iv_data.convert_lerobot data/raw/20261111 --name g1iv_pick_v1 [--include-outcomes success]
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np

from .validate import find_episodes, validate_episode

ARM_L, ARM_R = slice(0, 7), slice(7, 14)
HAND_R_DFX, HAND_L_DFX = slice(0, 6), slice(6, 12)
MOTORS = ([f"kLeft{n}" for n in ("ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw")]
          + [f"kRight{n}" for n in ("ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "Elbow", "WristRoll", "WristPitch", "WristYaw")]
          + [f"kLeftHand{n}" for n in ("Pinky", "Ring", "Middle", "Index", "ThumbBend", "ThumbRotation")]
          + [f"kRightHand{n}" for n in ("Pinky", "Ring", "Middle", "Index", "ThumbBend", "ThumbRotation")])
CAM_KEYS = {"head": "cam_left_high", "head_right": "cam_right_high", "left_wrist": "cam_left_wrist",
            "right_wrist": "cam_right_wrist"}   # recording alias -> LeRobot key (order = video order)
MODALITY = {
    "state": {"left_arm": {"start": 0, "end": 7}, "right_arm": {"start": 7, "end": 14},
              "left_hand": {"start": 14, "end": 20}, "right_hand": {"start": 20, "end": 26}},
    "action": {"left_arm": {"start": 0, "end": 7}, "right_arm": {"start": 7, "end": 14},
               "left_hand": {"start": 14, "end": 20}, "right_hand": {"start": 20, "end": 26},
               # walking command actually consumed by the locomotion policy (vx, vy, wz); the constant height
               # command (0.8) is left out (zero variance). GR00T LeRobotEpisodeLoader reads it via original_key.
               "base_command": {"start": 0, "end": 3, "original_key": "action.loco_cmd"}},
    "video": {},   # filled per dataset from the recorded cameras (modality(cams))
    "annotation": {"human.task_description": {"original_key": "task_index"}},
}


def modality(cams: list[str]) -> dict:
    m = json.loads(json.dumps(MODALITY))
    m["video"] = {a: {"original_key": f"observation.images.{CAM_KEYS[a]}"} for a in cams}
    return m


def camera_params(static: dict) -> dict | None:
    """Unitree dataset layout (meta/info.json camera_params, e.g. unitreerobotics/G1_Dex1_Stack_Block):
    head_stereo.{camera_matrix_left/right, dist_coeffs_left/right (k1 k2 p1 p2 k3), R, T [mm], p_right = R p_left + T}
    and wrist_<side>.color.{camera_matrix, dist_coeffs, width, height, model}; + project 'extrinsics'."""
    rig = (static or {}).get("camera_rig") or {}
    if not rig.get("use_rig"):
        return None

    def dist5(d):
        return [float(d.get(k, 0.0)) for k in ("k1", "k2", "p1", "p2", "k3")]

    out = {"source": "g1iv simulator camera rig (configs/camera_rig.yaml, docs/CAMERA_RIG.md)", "standard": rig.get("standard")}
    if "head" in rig:
        hl = rig["head"]
        hs = {"camera_matrix_left": hl["K"], "dist_coeffs_left": dist5(hl["dist"]), "rectified": False}
        if "head_right" in rig:
            hr = rig["head_right"]
            Rl, Rr = np.asarray(hl["R_parent_cam"]), np.asarray(hr["R_parent_cam"])
            R = Rr.T @ Rl                                            # p_right = R p_left + T
            T = Rr.T @ (np.asarray(hl["position_m"]) - np.asarray(hr["position_m"])) * 1000.0
            hs.update({"camera_matrix_right": hr["K"], "dist_coeffs_right": dist5(hr["dist"]), "R": R.tolist(), "T": T.tolist()})
        out["head_stereo"] = hs
    for side in ("left", "right"):
        w = rig.get(f"{side}_wrist")
        if w:
            out[f"wrist_{side}"] = {"color": {"camera_matrix": w["K"], "dist_coeffs": dist5(w["dist"]), "width": w["width"],
                                              "height": w["height"], "model": "opencv_pinhole"}}
    out["extrinsics"] = {CAM_KEYS[a]: {"parent_link": v["parent_link"], "position_m": v["position_m"],
                                       "quat_wxyz_opencv": v["quat_wxyz_ros"], "clipping_range": v["clipping_range"],
                                       "render_dist": v.get("render_dist")}
                         for a, v in rig.items() if a in CAM_KEYS}
    return out


def pack26(arm14, hand12_dfx):
    """[L arm, R arm] + DFX hand order (R first) -> unitree G1_Inspire order (L hand first)."""
    arm14, hand12_dfx = np.asarray(arm14, np.float32), np.asarray(hand12_dfx, np.float32)
    return np.concatenate([arm14[ARM_L], arm14[ARM_R], hand12_dfx[HAND_L_DFX], hand12_dfx[HAND_R_DFX]]).astype(np.float32)


def features(h: int, w: int, cams: list[str]) -> dict:
    vid = {"dtype": "video", "shape": (h, w, 3), "names": ["height", "width", "channels"]}
    f = {"observation.state": {"dtype": "float32", "shape": (26,), "names": MOTORS},
         "action": {"dtype": "float32", "shape": (26,), "names": MOTORS},
         "action.hand_binary": {"dtype": "float32", "shape": (2,), "names": ["left_close", "right_close"]},
         "action.loco_cmd": {"dtype": "float32", "shape": (4,), "names": ["vx", "vy", "wz", "height"]},
         "observation.base_vel": {"dtype": "float32", "shape": (6,), "names": ["vx_b", "vy_b", "vz_b", "wx_b", "wy_b", "wz_b"]},
         "observation.sim_time": {"dtype": "float64", "shape": (1,), "names": None},
         "observation.wall_time": {"dtype": "float64", "shape": (1,), "names": None},
         "eval.object_pos_w": {"dtype": "float32", "shape": (3,), "names": ["x", "y", "z"]},
         "eval.contact_force_total": {"dtype": "float32", "shape": (1,), "names": None},
         "eval.fallen": {"dtype": "float32", "shape": (1,), "names": None}}
    for a in cams:
        f[f"observation.images.{CAM_KEYS[a]}"] = dict(vid)
    return f


def _binary(tel: dict) -> np.ndarray:
    hc = tel.get("hand_cmd") if tel.get("present") and not tel.get("stale") else None
    if not hc:
        return np.array([-1.0, -1.0], np.float32)
    return np.array([1.0 if hc.get("left") == "CLOSE" else 0.0, 1.0 if hc.get("right") == "CLOSE" else 0.0], np.float32)


def frame_from_row(d: Path, row: dict, cams: list[str]) -> dict:
    from PIL import Image

    o, a, ev, tel = row["obs"], row["action"], row.get("eval", {}), row.get("teleop", {})
    fr = {"observation.state": pack26(o["arm_q"], o["hand_q_norm"]),
          "action": pack26(a["arm_q_target"], a["hand_q_norm_target"]),
          "action.hand_binary": _binary(tel),
          "action.loco_cmd": np.asarray(a["loco_cmd"], np.float32),
          "observation.base_vel": np.asarray(list(o["base_lin_vel_b"]) + list(o["base_ang_vel_b"]), np.float32),
          "observation.sim_time": np.array([row["sim_time"]], np.float64),
          "observation.wall_time": np.array([row["wall_time"]], np.float64),
          "eval.object_pos_w": np.asarray(ev.get("object_pos_w", [np.nan] * 3), np.float32),
          "eval.contact_force_total": np.array([ev.get("contact_force_total", np.nan)], np.float32),
          "eval.fallen": np.array([1.0 if ev.get("fallen") else 0.0], np.float32)}
    for alias in cams:
        with Image.open(d / o["image_files"][alias]) as im:
            fr[f"observation.images.{CAM_KEYS[alias]}"] = np.asarray(im.convert("RGB"))
    return fr


def write_stats(root: Path):
    """meta/stats.json (mean/std/min/max/q01/q99) for state/action, like GR00T's demo datasets."""
    import pyarrow.parquet as pq

    cols = {"observation.state": [], "action": [], "action.loco_cmd": []}  # loco_cmd: modality "base_command"
    for p in sorted((root / "data").rglob("*.parquet")):
        t = pq.read_table(p, columns=list(cols)).to_pydict()
        for k in cols:
            cols[k] += t[k]
    out = {}
    for k, v in cols.items():
        x = np.asarray(v, np.float64)
        out[k] = {"mean": x.mean(0).tolist(), "std": x.std(0).tolist(), "min": x.min(0).tolist(), "max": x.max(0).tolist(),
                  "q01": np.quantile(x, 0.01, axis=0).tolist(), "q99": np.quantile(x, 0.99, axis=0).tolist()}
    (root / "meta" / "stats.json").write_text(json.dumps(out, indent=1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--name", required=True)
    ap.add_argument("--out-root", default="data/processed")
    ap.add_argument("--include-outcomes", default="success", help="comma list (success,failure,undetermined,aborted)")
    ap.add_argument("--allow-smoke", action="store_true", help="required to convert mock/scripted data")
    ap.add_argument("--vcodec", default="libsvtav1", choices=["libsvtav1", "h264", "hevc"])
    a = ap.parse_args()

    import lerobot
    from lerobot.datasets import video_utils
    from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset

    if CODEBASE_VERSION != "v2.1":
        sys.exit(f"pinned writer must produce v2.1, got {CODEBASE_VERSION} (lerobot {lerobot.__version__})")
    if a.vcodec != "libsvtav1":
        _enc = video_utils.encode_video_frames
        video_utils.encode_video_frames = lambda *x, **k: _enc(*x, **{**k, "vcodec": a.vcodec})
        import lerobot.datasets.lerobot_dataset as ld
        ld.encode_video_frames = video_utils.encode_video_frames

    keep = set(a.include_outcomes.split(","))
    eps, skipped = [], []
    for d in find_episodes(a.paths):
        r = validate_episode(d)
        if not r.get("ok"):
            skipped.append((str(d), "validation: " + "; ".join(r["errors"][:3])))
        elif r.get("outcome") not in keep:
            skipped.append((str(d), f"outcome {r.get('outcome')} not in {sorted(keep)}"))
        elif r.get("is_smoke_data") and not a.allow_smoke:
            skipped.append((str(d), "smoke data (mock/scripted); pass --allow-smoke for pipeline tests"))
        else:
            eps.append((d, r))
    for s in skipped:
        print("skip", *s)
    if not eps:
        sys.exit("no episodes to convert")
    root = Path(a.out_root) / f"{a.name}_lerobot_v21"
    if root.exists():
        sys.exit(f"{root} exists; refusing to overwrite (choose another --name)")
    hz = {round(r["rate_sim_hz"], 3) for _, r in eps}
    if len(hz) != 1:
        sys.exit(f"episodes have different sim rates {hz}")
    fps = int(round(hz.pop()))
    h, w = (lambda s: (s[1], s[0]))(next(iter(eps[0][1]["image_size"].values())))
    cams = [c for c in CAM_KEYS if all(c in r.get("image_aliases", []) for _, r in eps)]
    missing = {c for _, r in eps for c in r.get("image_aliases", [])} - set(cams)
    if missing:
        print(f"note: cameras not present in every episode are left out: {sorted(missing)}")
    if not {"head", "left_wrist", "right_wrist"} <= set(cams):
        sys.exit(f"need head + both wrist cameras, have {cams}")
    print("cameras:", {c: CAM_KEYS[c] for c in cams})
    ds = LeRobotDataset.create(repo_id=f"local/{a.name}", fps=fps, features=features(h, w, cams), root=root,
                               robot_type="unitree_g1_29dof_inspire_sim", use_videos=True)
    prov = []
    t0 = time.time()
    for d, r in eps:
        meta = json.loads((d / "meta.json").read_text())
        rows = [json.loads(x) for x in (d / "steps.jsonl").read_text().splitlines() if x.strip()]
        ts0 = rows[0]["sim_time"]
        for row in rows:
            ds.add_frame(frame_from_row(d, row, cams), task=meta["instruction"], timestamp=float(row["sim_time"] - ts0))
        ds.save_episode()
        prov.append({"episode_index": len(prov), "raw_dir": str(d), "episode_id": meta["episode_id"], "source": meta["source"],
                     "is_smoke_data": meta.get("is_smoke_data"), "outcome": meta.get("outcome"), "frames": len(rows),
                     "instruction": meta.get("instruction"), "target_object": meta.get("target_object"),
                     "objects_xy": meta.get("objects_xy")})
        print(f"converted {meta['episode_id']} ({len(rows)} frames)")
    (root / "meta" / "modality.json").write_text(json.dumps(modality(cams), indent=1))
    first_static = json.loads((eps[0][0] / "meta.json").read_text()).get("static", {})
    cp = camera_params(first_static)
    if cp is not None:
        rigs = {json.dumps(json.loads((d / "meta.json").read_text()).get("static", {}).get("camera_rig"), sort_keys=True) for d, _ in eps}
        if len(rigs) != 1:
            sys.exit("episodes were recorded with different camera rigs; convert them separately")
        info_p = root / "meta" / "info.json"
        info = json.loads(info_p.read_text())
        info["camera_params"] = cp
        info_p.write_text(json.dumps(info, indent=4))
    write_stats(root)
    (root / "meta" / "g1iv_provenance.json").write_text(json.dumps(
        {"converter": "src/g1iv_data/convert_lerobot.py", "lerobot": lerobot.__version__, "codebase_version": CODEBASE_VERSION,
         "vcodec": a.vcodec, "fps": fps, "episodes": prov, "skipped": skipped, "seconds": time.time() - t0,
         "action_definition": "targets applied by the simulator during the step following the observation",
         "state_order": MOTORS}, indent=1))
    print(f"dataset -> {root}  episodes={len(prov)}")


if __name__ == "__main__":
    main()
