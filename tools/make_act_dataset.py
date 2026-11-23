"""Build a LeRobot v3.0 ACT training copy of a converted G1 dataset (data/processed/<name>_lerobot_v21).

    /mnt/isaac/so101_ws/.venv-train/bin/python tools/make_act_dataset.py data/processed/<name>_lerobot_v21 \
        [--out data/processed/<name>_act29_lerobot_v30] [--overwrite]

Needs a LeRobot >= 0.4 interpreter (v3.0 datasets); the so101 training venv (LeRobot 0.6.1) is the one used here.
The source v2.1 folder is only read.

LeRobot ACT predicts the single key `action`, so the walking command has to live inside it:
  action (29) = action[26] (arm q targets 14 + hand targets 12) + action.loco_cmd[:3] (vx, vy, wz)
The recorded loco_cmd is the command the Unitree locomotion policy consumed in that control step (ext.py wraps
run_command_dds.get_run_command), in m/s and rad/s. `height` (always 0.8, ignored by the official policy) is left
out; a runner sends [vx, vy, wz, 0.8] on rt/run_command/cmd. Leg joints are not part of the dataset: the
locomotion policy turns the command into leg targets.
observation.state stays the 26 arm/hand joints. Dropped (not consumed by ACT, would otherwise become policy inputs):
action.loco_cmd, action.hand_binary, observation.base_vel, observation.sim_time, observation.wall_time.
eval.* columns are kept (not observation./action. keys, so policies ignore them).

--loco-loss-weight K (default 1): LeRobot ACT has no per-dimension loss weights; its L1 loss is taken on MEAN_STD
normalised actions. Dividing the stored std of the three loco dims by K makes their normalised targets K times larger,
i.e. the same as weighting their L1 loss by K, while the policy's postprocessor (saved with the checkpoint, built from
the same stats.json) still turns the outputs back into m/s, rad/s. Only meta/stats.json is changed (per-episode stats
stay true); the factor is written to meta/g1iv_act_provenance.json (LeRobot warns about extra info.json keys).

Steps: copy -> official v2.1->v3.0 converter (lerobot.scripts.convert_dataset_v21_to_v30, no push) -> rewrite
data parquet + info.json -> recompute action stats (per episode in meta/episodes and global stats.json, same
min/max/mean/std/count fields) -> checks -> rename into place.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

LOCO_NAMES = ["loco_vx", "loco_vy", "loco_wz"]
DROP = ["action.loco_cmd", "action.hand_binary", "observation.base_vel", "observation.sim_time",
        "observation.wall_time"]
STAT_FIELDS = ["min", "max", "mean", "std", "count"]


def _stats(arr: np.ndarray) -> dict[str, np.ndarray]:
    from lerobot.datasets.compute_stats import get_feature_stats
    s = get_feature_stats(arr, axis=0, keepdims=False, quantile_list=[])
    return {k: s[k] for k in STAT_FIELDS}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", type=Path, help="v2.1 dataset folder (meta/, data/, videos/)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--repo-id", default=None, help="only used as the dataset name (default: local/<out name>)")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--loco-loss-weight", type=float, default=1.0,
                    help="effective L1 weight of the loco dims (std of action[26:29] in stats.json divided by this)")
    a = ap.parse_args()

    from lerobot.datasets.compute_stats import aggregate_stats
    from lerobot.datasets.io_utils import write_stats
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.scripts.convert_dataset_v21_to_v30 import convert_dataset

    src = a.src.resolve()
    info0 = json.loads((src / "meta/info.json").read_text())
    if info0.get("codebase_version") != "v2.1":
        print(f"source is {info0.get('codebase_version')}, expected v2.1", file=sys.stderr)
        return 2
    feats0 = info0["features"]
    if feats0["action"]["shape"] != [26] or feats0["action.loco_cmd"]["names"][:3] != ["vx", "vy", "wz"]:
        print("unexpected action / action.loco_cmd layout", file=sys.stderr)
        return 2
    name = src.name.removesuffix("_lerobot_v21")
    out = (a.out or src.parent / f"{name}_act29_lerobot_v30").resolve()
    repo_id = a.repo_id or f"local/{out.name}"
    if out.exists():
        if not a.overwrite:
            print(f"{out} exists (use --overwrite)", file=sys.stderr)
            return 2
        shutil.rmtree(out)
    work = out.parent / f"{out.name}.building"
    for p in (work, work.parent / f"{work.name}_old", work.parent / f"{work.name}_v30"):
        if p.exists():
            shutil.rmtree(p)

    # 1) official conversion on a copy (the converter works in place and leaves <root>_old = the v2.1 copy)
    shutil.copytree(src, work)
    convert_dataset(repo_id=repo_id, root=work, push_to_hub=False)
    shutil.rmtree(work.parent / f"{work.name}_old")

    # 2) data parquet: action <- [action, loco_cmd[:3]], drop unused columns
    old_action, old_loco = [], []
    for pq in sorted((work / "data").glob("*/*.parquet")):
        df = pd.read_parquet(pq)
        act = np.stack(df["action"].values).astype(np.float32)
        loco = np.stack(df["action.loco_cmd"].values).astype(np.float32)
        old_action.append(act)
        old_loco.append(loco)
        df["action"] = list(np.concatenate([act, loco[:, :3]], axis=1))
        df = df.drop(columns=[c for c in DROP if c in df.columns])
        df.to_parquet(pq, index=False)

    # 3) info.json
    info = json.loads((work / "meta/info.json").read_text())
    for k in DROP:
        info["features"].pop(k, None)
    info["features"]["action"]["shape"] = [29]
    info["features"]["action"]["names"] = list(feats0["action"]["names"]) + LOCO_NAMES
    (work / "meta/info.json").write_text(json.dumps(info, indent=4))

    # 4) stats: per-episode (meta/episodes) and global
    ds_tmp = LeRobotDataset(repo_id=repo_id, root=work)          # loads the rewritten parquet with the new schema
    # float64: float32 stats leave std ~1e-3 instead of 0 on (near-)constant dims after aggregation
    act_all = np.stack(ds_tmp.hf_dataset.with_format("numpy")["action"]).astype(np.float64)
    ep_idx = np.asarray(ds_tmp.hf_dataset.with_format("numpy")["episode_index"])
    ep_stats = []
    for epf in sorted((work / "meta/episodes").glob("*/*.parquet")):
        e = pd.read_parquet(epf)
        e = e.drop(columns=[c for c in e.columns if any(c.startswith(f"stats/{k}/") for k in DROP)])
        rows = {f: [] for f in STAT_FIELDS}
        for i in e["episode_index"]:
            s = _stats(act_all[ep_idx == i])
            ep_stats.append({"action": s})
            for f in STAT_FIELDS:
                rows[f].append(s[f].tolist())
        for f in STAT_FIELDS:
            e[f"stats/action/{f}"] = rows[f]
        e.to_parquet(epf, index=False)
    stats = json.loads((work / "meta/stats.json").read_text())
    for k in DROP:
        stats.pop(k, None)
    stats = {k: {f: np.asarray(v) for f, v in d.items()} for k, d in stats.items()}
    stats["action"] = aggregate_stats(ep_stats)["action"]
    write_stats(stats, work)

    # 5) checks: values, stats consistency with the source, loadable sample
    ds = LeRobotDataset(repo_id=repo_id, root=work)
    act_new = np.stack(ds.hf_dataset.with_format("numpy")["action"])
    ref = np.concatenate([np.concatenate(old_action), np.concatenate(old_loco)[:, :3]], axis=1)
    assert act_new.shape == (info0["total_frames"], 29), act_new.shape
    assert np.array_equal(act_new, ref), "action values changed"
    s_old = json.loads((src / "meta/stats.json").read_text())
    for f in ("min", "max", "mean", "std"):
        got = np.asarray(ds.meta.stats["action"][f])
        np.testing.assert_allclose(got[:26], np.asarray(s_old["action"][f]), rtol=1e-4, atol=1e-5)
        np.testing.assert_allclose(got[26:], np.asarray(s_old["action.loco_cmd"][f])[:3], rtol=1e-4, atol=1e-5)
    item = ds[0]
    assert item["action"].shape == (29,) and not any(k in item for k in DROP)
    k_loco = float(a.loco_loss_weight)
    if k_loco != 1.0:
        assert k_loco > 0
        st = {k: {f: np.asarray(v) for f, v in d.items()} for k, d in ds.meta.stats.items()}
        st["action"]["std"] = st["action"]["std"].astype(np.float64).copy()
        st["action"]["std"][26:29] /= k_loco
        write_stats(st, work)
        chk = LeRobotDataset(repo_id=repo_id, root=work).meta.stats["action"]["std"]
        true_std = np.asarray(ds.meta.stats["action"]["std"])          # loaded before the rewrite
        np.testing.assert_allclose(np.asarray(chk)[26:29] * k_loco, true_std[26:29], rtol=1e-5)
        np.testing.assert_allclose(np.asarray(chk)[:26], true_std[:26], rtol=1e-6)
    (work / "meta/g1iv_act_provenance.json").write_text(json.dumps({
        "tool": "tools/make_act_dataset.py", "source": src.name, "source_codebase_version": "v2.1",
        "action": "action[0:26] = source action; action[26:29] = source action.loco_cmd[0:3] (vx, vy, wz)",
        "loco_cmd_runtime": "send [vx, vy, wz, 0.8] on rt/run_command/cmd; the sim provider zeroes the command after "
                            "every read, so republish faster than the 50 Hz locomotion step",
        "dropped": DROP,
        "loco_loss_weight": k_loco,
        "stats_note": ("stats.json action std[26:29] divided by loco_loss_weight (loss weighting); true std = stored x "
                       "loco_loss_weight; per-episode stats in meta/episodes are unscaled") if k_loco != 1.0 else None},
        indent=1))
    work.rename(out)
    print(f"[make_act_dataset] {out}: {ds.meta.total_episodes} episodes, {ds.meta.total_frames} frames, "
          f"action 29 = 26 joints + {LOCO_NAMES}, state {ds.meta.features['observation.state']['shape']}, "
          f"cameras {ds.meta.video_keys}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
