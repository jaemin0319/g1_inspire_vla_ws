"""Offline check of an ACT checkpoint on recorded dataset frames: does it command walking where the expert walked?

    HF_HUB_OFFLINE=1 /mnt/isaac/so101_ws/.venv-train/bin/python tools/act_loco_probe.py \
        <run>/checkpoints/<step>/pretrained_model <act29 v3.0 dataset root> [--episodes 49 50 ...] [--stride 5]

Default episodes = the ones LeRobot held out for evaluation (last ceil(0.1 * n) episodes, the training eval split).
For every --stride-th frame the policy is reset and asked for one action (first step of a fresh chunk, no ensembling;
images go in as uint8 HWC like the policy server gets them, minus its JPEG round trip). Per episode it reports:
  walk recall   fraction of expert walking frames (|cmd| > 1e-3) where the policy's max |vx, vy, wz| > --walk-thr
  false walk    fraction of expert standing frames before the arm starts where the policy walks (> --walk-thr)
  frame-0 cmd   the command at the first frame (the robot is still standing far away: should be ~0.5 m/s forward)
  loco MAE      vx, vy, wz mean absolute error over all probed frames; arm MAE (rad) for reference
The 0.6.1 LeRobot interpreter of the so101 training venv is needed (v3.0 dataset, ACT checkpoint).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("checkpoint")
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--episodes", type=int, nargs="*", default=None)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--walk-thr", type=float, default=0.05)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--json", type=Path, default=None, help="write the per-episode results here")
    a = ap.parse_args()

    import torch
    from lerobot.common.control_utils import predict_action
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    cfg = PreTrainedConfig.from_pretrained(a.checkpoint)
    cfg.temporal_ensemble_coeff, cfg.n_action_steps, cfg.device = None, 1, a.device
    policy = ACTPolicy.from_pretrained(a.checkpoint, config=cfg).to(a.device).eval()
    pre, post = make_pre_post_processors(cfg, pretrained_path=a.checkpoint,
                                         preprocessor_overrides={"device_processor": {"device": a.device}})
    img_keys = [k for k in cfg.input_features if k.startswith("observation.images.")]
    meta = LeRobotDatasetMetadata("local/probe", root=a.dataset)
    n_ep = meta.total_episodes
    eps = a.episodes if a.episodes else list(range(n_ep - math.ceil(0.1 * n_ep), n_ep))
    dev = torch.device(a.device)
    out = []
    for ep in eps:
        ds = LeRobotDataset("local/probe", root=a.dataset, episodes=[ep], return_uint8=True)
        gt_all = np.stack([np.asarray(x) for x in ds.hf_dataset["action"]]).astype(np.float32)
        arm = gt_all[:, :14]
        moved = np.abs(arm - arm[0]).max(1) > 0.02
        arm_on = int(np.argmax(moved)) if moved.any() else len(arm)
        idx = list(range(0, len(ds), a.stride))
        pred, gt = [], []
        for i in idx:
            it = ds[i]
            obs = {"observation.state": it["observation.state"].numpy().astype(np.float32)}
            for k in img_keys:
                obs[k] = it[k].permute(1, 2, 0).numpy().astype(np.uint8)
            policy.reset()
            act = predict_action(obs, policy, dev, pre, post, use_amp=False)
            pred.append(act.squeeze(0).float().cpu().numpy())
            gt.append(gt_all[i])
        pred, gt, idx = np.asarray(pred), np.asarray(gt), np.asarray(idx)
        gw = np.abs(gt[:, 26:29]).max(1) > 1e-3
        pw = np.abs(pred[:, 26:29]).max(1) > a.walk_thr
        stand_pre = (~gw) & (idx < arm_on)
        r = {"episode": ep, "n_probed": int(len(idx)), "arm_onset": arm_on,
             "walk_recall": round(float(pw[gw].mean()), 3) if gw.any() else None,
             "false_walk": round(float(pw[stand_pre].mean()), 3) if stand_pre.any() else None,
             "frame0_pred": np.round(pred[0, 26:29], 3).tolist(), "frame0_gt": np.round(gt[0, 26:29], 3).tolist(),
             "loco_mae": np.round(np.abs(pred[:, 26:29] - gt[:, 26:29]).mean(0), 3).tolist(),
             "arm_mae_rad": round(float(np.abs(pred[:, :14] - gt[:, :14]).mean()), 4),
             "pred_vx_walk_mean": round(float(pred[gw, 26].mean()), 3) if gw.any() else None,
             "gt_vx_walk_mean": round(float(gt[gw, 26].mean()), 3) if gw.any() else None}
        out.append(r)
        print(json.dumps(r), flush=True)
    rec = [r["walk_recall"] for r in out if r["walk_recall"] is not None]
    fw = [r["false_walk"] for r in out if r["false_walk"] is not None]
    print(f"[probe] {a.checkpoint}: walk recall mean {np.mean(rec):.3f}, false walk mean {np.mean(fw):.3f}, "
          f"frame-0 vx {np.round([r['frame0_pred'][0] for r in out], 2).tolist()}", flush=True)
    if a.json:
        a.json.write_text(json.dumps({"checkpoint": a.checkpoint, "episodes": out}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
