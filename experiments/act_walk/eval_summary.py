"""Summarise one policy-eval folder (logs/policy_eval/<time>_<label>_<mode>): outcome, walking, where the robot stopped.

    python eval_summary.py <eval dir> [...] [--raw-root /home/jammin/g1iv_store/policy_eval]

Stop error = pelvis xy when the arm starts the reach (|arm - arm[0]| > 0.15 rad) minus the expert's stand point for that
cylinder: object_xy + (-0.292, +0.136) (mean pelvis-minus-object at arm onset over the 55 training episodes,
std 1.3 / 0.6 cm). dx > 0 = too close to the desk, dy > 0 = too far left.

Also at the arm onset, read from the full recording <raw-root>/<date>/<episode_id>/arrays.npz (the simulator records
the pelvis orientation and velocity for every policy episode, so this works for every past evaluation):
  yaw   pelvis world yaw in deg (the stand pose faces 0 deg = square to the desk)
  wait  sim seconds from the last walking command to the arm onset (None if it never stopped walking before)
  v_on  pelvis horizontal speed in m/s (body frame)
Recording frame j is trace frame j + 1 (the trace also holds the observation captured at record start; checked
2026-11-23: identical base positions with that shift).
"""
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np

OFF = np.array([-0.292, 0.136])
FRAME_S = 0.04

args = sys.argv[1:]
raw_root = "/home/jammin/g1iv_store/policy_eval"
if "--raw-root" in args:
    k = args.index("--raw-root")
    raw_root = args[k + 1]
    del args[k:k + 2]


def at_onset(episode_id, on):
    """(yaw deg, pelvis speed m/s) at trace frame `on` from the full recording, or (None, None) if it is missing."""
    P = glob.glob(f"{raw_root}/*/{episode_id}/arrays.npz") if episode_id else []
    if not P:
        return None, None
    a = np.load(P[0])
    j = min(max(on - 1, 0), len(a["obs/base_quat_w"]) - 1)
    w, x, y, q = a["obs/base_quat_w"][j]
    yaw = math.degrees(math.atan2(2 * (w * q + x * y), 1 - 2 * (y * y + q * q)))
    return yaw, float(np.linalg.norm(a["obs/base_lin_vel_b"][j, :2]))


def fmt(v, nd):
    return "-" if v is None else f"{v:.{nd}f}"


for d in map(Path, args):
    rows = [json.loads(l) for l in (d / "summary.jsonl").read_text().splitlines() if l.strip()]
    errs, yaws, waits, vons, n_succ = [], [], [], [], 0
    print(f"== {d.name}")
    for r in rows:
        z = np.load(d / f"ep{r['i']:02d}_trace.npz")
        a, bp = z["action"], z["base_pos"]
        mv = np.abs(a[:, :14] - a[0, :14]).max(1) > 0.15   # reach start (policy arm jitters ~0.04 while walking)
        on = int(np.argmax(mv)) if mv.any() else len(a) - 1
        tgt = np.asarray(r["object_xy"]) + OFF
        e = bp[on, :2] - tgt
        walk = np.abs(z["loco_sent"]).max(1) > 1e-3
        wait_s = (on - int(np.nonzero(walk[:on])[0].max())) * FRAME_S if walk[:on].any() else None
        yaw, v_on = at_onset(r.get("episode_id"), on)
        errs.append(e)
        for lst, v in ((yaws, yaw), (waits, wait_s), (vons, v_on)):
            if v is not None:
                lst.append(v)
        n_succ += r["outcome"] == "success"
        print(f"  ep{r['i']} {r['outcome']:8s} {r.get('reason') or '':8s} frames {r['frames']:4d} lift {r['max_lift_m']:+.3f} "
              f"travel {r['base_travel_m']:.2f} m walk-cmd {r['walk_cmd_frames']:4d} | arm onset f{on:4d} "
              f"stop err dx {e[0]:+.3f} dy {e[1]:+.3f} m yaw {fmt(yaw, 1)} deg wait {fmt(wait_s, 2)} s "
              f"v_on {fmt(v_on, 3)} m/s")
    errs = np.asarray(errs)
    extra = ""
    if yaws:
        extra += f", |yaw| mean {np.abs(yaws).mean():.1f} deg, v_on mean {np.mean(vons):.3f} m/s"
    if waits:
        extra += f", wait median {np.median(waits):.2f} s ({len(waits)} eps)"
    print(f"  -> success {n_succ}/{len(rows)}, |dx| mean {np.abs(errs[:, 0]).mean():.3f}, "
          f"|dy| mean {np.abs(errs[:, 1]).mean():.3f} m{extra}", flush=True)
