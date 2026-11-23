"""Select recorded walk -> grasp episodes for training: successful and without excessive swaying ("휘청거림").

    .conda/envs/g1_inspire_data/bin/python tools/filter_walk_episodes.py <raw dir or glob> ... \
        --out-dir <selection dir> [--max-tilt-deg 7] [--min-pelvis-z 0.745] [--max-rp-rate 1.0] [--limit N]

Per raw episode (arrays.npz + meta.json from the simulator recorder) it measures, over the whole episode:
  tilt_max   max angle between the pelvis z axis and vertical (from obs/projected_gravity_b), deg. The pelvis of the
             standing / walking G1 sits at ~5.3-5.8 deg; the 1 s backward step of a scripted re-approach lurches to
             8-9 deg and the one near-fall in walk_grasp_desk065_scripted_v1 reached 15 deg.
  pelvis_z   min pelvis height, m (normal 0.755-0.78; that near-fall dropped to 0.649)
  rp_rate    max roll/pitch angular rate |(wx, wy)| of the pelvis, rad/s (normal walking <= ~0.85)
plus the walking pattern (W = frames with a walking command, S = standing frames, before the arm first moves).
Episodes pass when outcome == success and all three are inside the limits. Passing episodes are linked (symlinks,
nothing is copied or deleted) into --out-dir in recording order, up to --limit; a report.json lists every episode with
its metrics and the reason it was left out.
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np


def metrics(d: Path) -> dict:
    z = np.load(d / "arrays.npz")
    m = json.loads((d / "meta.json").read_text())
    lc = z["action/loco_cmd"][:, :3]
    arm = z["action/arm_q_target"]
    g = z["obs/projected_gravity_b"]
    w = z["obs/base_ang_vel_b"]
    bp = z["obs/base_pos_w"]
    walk = np.abs(lc).max(1) > 1e-3
    moved = np.abs(arm - arm[0]).max(1) > 0.02
    arm_on = int(np.argmax(moved)) if moved.any() else len(arm)
    segs, cur, n = [], bool(walk[0]), 0
    for x in walk[:arm_on]:
        if bool(x) == cur:
            n += 1
        else:
            segs.append(("W" if cur else "S") + str(n))
            cur, n = bool(x), 1
    segs.append(("W" if cur else "S") + str(n))
    tilt = np.degrees(np.arccos(np.clip(-g[:, 2], -1.0, 1.0)))
    return {"episode_id": m["episode_id"], "outcome": m.get("outcome"), "n_frames": int(len(arm)),
            "tilt_max_deg": round(float(tilt.max()), 2), "pelvis_z_min": round(float(bp[:, 2].min()), 3),
            "rp_rate_max": round(float(np.linalg.norm(w[:, :2], axis=1).max()), 3),
            "walk_frames": int(walk.sum()), "arm_onset_frame": arm_on, "pattern": " ".join(segs),
            "n_walk_segments": sum(s.startswith("W") for s in segs),
            "travel_m": round(float(np.linalg.norm(bp[-1, :2] - bp[0, :2])), 3)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="raw episode folders, folders of them, or globs")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--max-tilt-deg", type=float, default=7.0)
    ap.add_argument("--min-pelvis-z", type=float, default=0.745)
    ap.add_argument("--max-rp-rate", type=float, default=1.0)
    ap.add_argument("--limit", type=int, default=None, help="keep at most this many passing episodes (first ones)")
    a = ap.parse_args()
    eps: list[Path] = []
    for p in a.paths:
        for q in sorted(glob.glob(p)) or [p]:
            q = Path(q)
            if (q / "meta.json").exists():
                eps.append(q)
            else:
                eps += sorted(x.parent for x in q.rglob("meta.json"))
    eps = sorted(set(eps), key=lambda x: x.name)
    rows, keep = [], []
    for d in eps:
        if not (d / "arrays.npz").exists():          # still being recorded / written
            print(f"{d.name} incomplete (no arrays.npz), skipped")
            continue
        r = metrics(d)
        why = []
        if r["outcome"] != "success":
            why.append(f"outcome {r['outcome']}")
        if r["tilt_max_deg"] > a.max_tilt_deg:
            why.append(f"tilt {r['tilt_max_deg']} > {a.max_tilt_deg} deg")
        if r["pelvis_z_min"] < a.min_pelvis_z:
            why.append(f"pelvis z {r['pelvis_z_min']} < {a.min_pelvis_z} m")
        if r["rp_rate_max"] > a.max_rp_rate:
            why.append(f"roll/pitch rate {r['rp_rate_max']} > {a.max_rp_rate} rad/s")
        if not why and a.limit is not None and len(keep) >= a.limit:
            why.append("over --limit")
        r["path"] = str(d.resolve())
        r["excluded"] = why
        rows.append(r)
        if not why:
            keep.append(d)
    a.out_dir.mkdir(parents=True, exist_ok=True)
    for old in a.out_dir.iterdir():
        if old.is_symlink():
            old.unlink()
    for d in keep:
        (a.out_dir / d.name).symlink_to(d.resolve(), target_is_directory=True)
    (a.out_dir / "report.json").write_text(json.dumps({
        "limits": {"max_tilt_deg": a.max_tilt_deg, "min_pelvis_z": a.min_pelvis_z, "max_rp_rate": a.max_rp_rate,
                   "limit": a.limit},
        "n_total": len(rows), "n_selected": len(keep), "episodes": rows}, indent=1))
    for r in rows:
        print(f"{r['episode_id']} {r['outcome']:8s} n {r['n_frames']:4d} tilt {r['tilt_max_deg']:5.2f} "
              f"z {r['pelvis_z_min']:.3f} rp {r['rp_rate_max']:.2f} {r['pattern']:28s} "
              f"{'KEEP' if not r['excluded'] else 'drop: ' + '; '.join(r['excluded'])}")
    print(f"[filter] {len(keep)}/{len(rows)} selected -> {a.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
