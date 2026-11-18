"""Validate raw sim-side episodes (data/raw or data/smoke) before conversion.

Checks: required files, frame count, uniform SIMULATION-time spacing, step spacing, camera frame ids
(duplicate = reused image, gap = missed render), image readability/size, array dimensions, NaNs,
hub-metadata freshness, writer drops, wall-clock rate vs sim rate, outcome label, smoke flag.

  python -m g1iv_data.validate data/smoke/20261111 --report logs/validate_xxx.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

EXPECT = {"obs/arm_q": 14, "obs/arm_dq": 14, "obs/hand_q_norm": 12, "obs/hand_q_rad": 12,
          "action/arm_q_target": 14, "action/hand_q_norm_target": 12, "action/loco_cmd": 4,
          "obs/base_lin_vel_b": 3, "obs/base_ang_vel_b": 3, "obs/base_quat_w": 4}


def find_episodes(paths):
    out = []
    for p in paths:
        p = Path(p)
        if (p / "meta.json").is_file():
            out.append(p)
        else:
            out += sorted(q.parent for q in p.rglob("meta.json"))
    return out


def validate_episode(d: Path, check_images: int = 5) -> dict:
    from PIL import Image

    r = {"dir": str(d), "errors": [], "warnings": []}
    E, W = r["errors"].append, r["warnings"].append
    try:
        meta = json.loads((d / "meta.json").read_text())
    except Exception as e:  # noqa: BLE001
        E(f"meta.json unreadable: {e}")
        return r
    r.update({k: meta.get(k) for k in ("episode_id", "source", "is_smoke_data", "outcome", "end_reason", "instruction",
                                        "n_frames", "n_dropped_by_writer", "record_hz_sim")})
    if "outcome" not in meta:
        E("episode not finalised (no outcome) - recorder still running or crashed")
    if not meta.get("instruction"):
        W("empty language instruction")
    if meta.get("source") in ("mock", "scripted") and not meta.get("is_smoke_data"):
        E("mock/scripted episode not flagged as smoke data")
    rows = [json.loads(x) for x in (d / "steps.jsonl").read_text().splitlines() if x.strip()]
    r["rows"] = len(rows)
    if not rows:
        E("no frames")
        return r
    if meta.get("n_frames") is not None and meta["n_frames"] != sum(1 for x in rows if not x.get("dropped_by_writer")):
        E(f"n_frames {meta['n_frames']} != rows written")
    drops = [x["frame_index"] for x in rows if x.get("dropped_by_writer") or x.get("writer_error")]
    if drops:
        E(f"{len(drops)} frames dropped/failed by the writer: {drops[:10]}")
    idx = [x["frame_index"] for x in rows]
    if idx != list(range(len(rows))):
        E("frame_index not contiguous")
    st = np.array([x["step"] for x in rows])
    t = np.array([x["sim_time"] for x in rows])
    wall = np.array([x["wall_time"] for x in rows])
    dst, dt = np.diff(st), np.diff(t)
    hz = meta.get("record_hz_sim")
    r["sim_dt_unique"] = sorted(set(np.round(dt, 6).tolist()))
    if len(set(dst.tolist())) > 1:
        E(f"non-uniform step spacing {sorted(set(dst.tolist()))}")
    if hz and len(dt) and not np.allclose(dt, 1.0 / hz, atol=1e-6):
        E(f"sim-time spacing != 1/{hz}: {r['sim_dt_unique'][:5]}")
    if len(t) > 1:
        r["duration_sim_s"] = float(t[-1] - t[0])
        r["duration_wall_s"] = float(wall[-1] - wall[0])
        r["rate_sim_hz"] = float((len(t) - 1) / (t[-1] - t[0]))
        r["rate_wall_hz"] = float((len(t) - 1) / max(1e-9, wall[-1] - wall[0]))
        r["rtf"] = r["duration_sim_s"] / max(1e-9, r["duration_wall_s"])
    # camera freshness: Isaac Lab camera frame counters must strictly increase. With forced_update (recorder marks the
    # buffer outdated before reading) the counter counts reads, so increments > 1 only mean other readers (image server).
    cams = sorted(rows[0]["obs"].get("cam_meta", {}))
    r["cameras"] = cams
    for c in cams:
        fr = np.array([x["obs"]["cam_meta"][c]["frame"] for x in rows])
        if np.all(fr < 0):
            W(f"{c}: no frame counter available")
            continue
        dfr = np.diff(fr)
        dup = int(np.sum(dfr == 0))
        r[f"{c}_frame_step_unique"] = sorted(set(dfr.tolist()))
        if dup:
            E(f"{c}: {dup} duplicated (stale) camera frames")
        forced = all(x["obs"]["cam_meta"][c].get("forced_update") for x in rows)
        r[f"{c}_forced_update"] = forced
        if len(set(dfr.tolist())) > 1 and not forced:
            # without the recorder's forced read, the counter steps with the renderer: gaps = missed renders
            W(f"{c}: irregular camera frame increments {sorted(set(dfr.tolist()))[:6]}")
    # images
    files = rows[0]["obs"].get("image_files", {})
    for sample in np.linspace(0, len(rows) - 1, min(check_images, len(rows))).astype(int):
        for a, rel in rows[sample]["obs"].get("image_files", {}).items():
            p = d / rel
            if not p.is_file():
                E(f"missing image {rel}")
                continue
            try:
                with Image.open(p) as im:
                    r.setdefault("image_size", {})[a] = list(im.size)
            except Exception as e:  # noqa: BLE001
                E(f"unreadable image {rel}: {e}")
    r["image_aliases"] = sorted(files)
    # arrays
    try:
        arr = np.load(d / "arrays.npz")
        for k, n in EXPECT.items():
            if k not in arr:
                E(f"arrays.npz missing {k}")
            elif arr[k].shape[1:] != (n,):
                E(f"{k} shape {arr[k].shape} expected (*, {n})")
            elif not np.all(np.isfinite(arr[k])):
                E(f"{k} has NaN/inf")
        if "action/hand_q_norm_target" in arr and (arr["action/hand_q_norm_target"].min() < -1e-6 or arr["action/hand_q_norm_target"].max() > 1 + 1e-6):
            E("hand targets outside [0,1]")
    except Exception as e:  # noqa: BLE001
        E(f"arrays.npz unreadable: {e}")
    # hub metadata
    tel = [x.get("teleop", {}) for x in rows]
    present = [x for x in tel if x.get("present")]
    r["teleop_meta_present_frac"] = len(present) / len(rows)
    if present:
        ages = np.array([x["age_ms"] for x in present])
        r["teleop_meta_age_ms"] = {"median": float(np.median(ages)), "p95": float(np.percentile(ages, 95)), "max": float(ages.max())}
        stale = sum(1 for x in present if x.get("stale"))
        if stale:
            W(f"{stale} frames with stale hub metadata")
        lost = sum(1 for x in present if x.get("input_ok") is False)
        r["frames_input_lost"] = lost
    else:
        W("no hub metadata in this episode (raw input / pre-limit targets not available)")
    r["ok"] = not r["errors"]
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--report")
    ap.add_argument("--images", type=int, default=5)
    a = ap.parse_args()
    eps = find_episodes(a.paths)
    res = [validate_episode(d, a.images) for d in eps]
    for x in res:
        flag = "OK  " if x.get("ok") else "FAIL"
        print(f"{flag} {x.get('episode_id')} src={x.get('source')} outcome={x.get('outcome')} frames={x.get('rows')} "
              f"sim_hz={x.get('rate_sim_hz', 0):.2f} wall_hz={x.get('rate_wall_hz', 0):.2f} rtf={x.get('rtf', 0):.2f}")
        for e in x["errors"]:
            print(f"     ERROR {e}")
        for w in x["warnings"]:
            print(f"     warn  {w}")
    if a.report:
        Path(a.report).parent.mkdir(parents=True, exist_ok=True)
        Path(a.report).write_text(json.dumps(res, indent=1))
    print(f"{sum(1 for x in res if x.get('ok'))}/{len(res)} episodes valid")
    sys.exit(0 if res and all(x.get("ok") for x in res) else 1)


if __name__ == "__main__":
    main()
