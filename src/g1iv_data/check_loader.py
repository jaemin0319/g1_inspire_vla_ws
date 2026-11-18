"""Load a converted dataset with the pinned LeRobot v2.1 loader, pull one DataLoader batch, decode video,
and compare a few values against the raw episode.

  python -m g1iv_data.check_loader data/processed/<name>_lerobot_v21 [--batch 4]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--batch", type=int, default=4)
    ap.add_argument("--report")
    # pyav: CPU decoding with the PyAV FFmpeg build (AV1 via libdav1d). lerobot's default here is torchcodec,
    # whose 0.5+cu128 wheel needs the CUDA NPP libraries (libnppicc.so.12) that this env does not carry.
    ap.add_argument("--video-backend", default="pyav", choices=["pyav", "torchcodec"])
    a = ap.parse_args()
    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(a.root)
    info = json.loads((root / "meta" / "info.json").read_text())
    ds = LeRobotDataset(repo_id=f"local/{root.name}", root=root, video_backend=a.video_backend)
    fps = ds.fps
    # one-step-ahead action chunk, as VLA trainers request it
    ds_chunk = LeRobotDataset(repo_id=f"local/{root.name}", root=root, video_backend=a.video_backend,
                              delta_timestamps={"action": [i / fps for i in range(8)]})
    dl = torch.utils.data.DataLoader(ds_chunk, batch_size=a.batch, shuffle=True, num_workers=0)
    b = next(iter(dl))
    shapes = {k: list(v.shape) for k, v in b.items() if hasattr(v, "shape")}
    rep = {"root": str(root), "video_backend": a.video_backend, "codebase_version": info["codebase_version"], "fps": fps, "num_episodes": ds.num_episodes,
           "num_frames": ds.num_frames, "batch_shapes": shapes, "tasks": list(ds.meta.tasks.values()) if hasattr(ds.meta, "tasks") else None}
    errors = []
    img_keys = sorted(k for k in info["features"] if k.startswith("observation.images."))
    rep["image_keys"] = img_keys
    for need in ("cam_left_high", "cam_left_wrist", "cam_right_wrist"):
        if f"observation.images.{need}" not in img_keys:
            errors.append(f"dataset lacks observation.images.{need}")
    for k in img_keys:
        if k not in b:
            errors.append(f"missing {k}")
        elif b[k].dtype != torch.float32 or b[k].shape[-3] != 3:
            errors.append(f"{k} unexpected {b[k].dtype} {list(b[k].shape)}")
    if list(b["observation.state"].shape[1:]) != [26] or list(b["action"].shape[1:]) != [8, 26]:
        errors.append("state/action dims")
    # raw comparison on episode 0 frame 0
    prov = json.loads((root / "meta" / "g1iv_provenance.json").read_text())
    raw = Path(prov["episodes"][0]["raw_dir"])
    row0 = json.loads((raw / "steps.jsonl").read_text().splitlines()[0])
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from g1iv_data.convert_lerobot import pack26

    s_ds = ds[0]["observation.state"].numpy()
    s_raw = pack26(row0["obs"]["arm_q"], row0["obs"]["hand_q_norm"])
    rep["state0_max_abs_diff_vs_raw"] = float(np.max(np.abs(s_ds - s_raw)))
    if rep["state0_max_abs_diff_vs_raw"] > 1e-5:
        errors.append("state differs from raw")
    rep["modality_json"] = (root / "meta" / "modality.json").is_file()
    rep["stats_json"] = (root / "meta" / "stats.json").is_file()
    rep["errors"] = errors
    print(json.dumps(rep, indent=1))
    if a.report:
        Path(a.report).write_text(json.dumps(rep, indent=1))
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
