"""Summarise one policy-eval folder (logs/policy_eval/<time>_<label>_<mode>): outcome, walking, where the robot stopped.

    python eval_summary.py <eval dir> [...]

Stop error = pelvis xy when the arm starts the reach (|arm - arm[0]| > 0.15 rad) minus the expert's stand point for that
cylinder: object_xy + (-0.292, +0.136) (mean pelvis-minus-object at arm onset over the 55 training episodes,
std 1.3 / 0.6 cm). dx > 0 = too close to the desk, dy > 0 = too far left.
"""
import json
import sys
from pathlib import Path

import numpy as np

OFF = np.array([-0.292, 0.136])

for d in map(Path, sys.argv[1:]):
    rows = [json.loads(l) for l in (d / "summary.jsonl").read_text().splitlines() if l.strip()]
    errs, n_succ = [], 0
    print(f"== {d.name}")
    for r in rows:
        z = np.load(d / f"ep{r['i']:02d}_trace.npz")
        a, bp = z["action"], z["base_pos"]
        mv = np.abs(a[:, :14] - a[0, :14]).max(1) > 0.15   # reach start (policy arm jitters ~0.04 while walking)
        on = int(np.argmax(mv)) if mv.any() else len(a) - 1
        tgt = np.asarray(r["object_xy"]) + OFF
        e = bp[on, :2] - tgt
        errs.append(e)
        n_succ += r["outcome"] == "success"
        print(f"  ep{r['i']} {r['outcome']:8s} {r.get('reason') or '':8s} frames {r['frames']:4d} lift {r['max_lift_m']:+.3f} "
              f"travel {r['base_travel_m']:.2f} m walk-cmd {r['walk_cmd_frames']:4d} | arm onset f{on:4d} "
              f"stop err dx {e[0]:+.3f} dy {e[1]:+.3f} m")
    errs = np.asarray(errs)
    print(f"  -> success {n_succ}/{len(rows)}, |dx| mean {np.abs(errs[:, 0]).mean():.3f}, "
          f"|dy| mean {np.abs(errs[:, 1]).mean():.3f} m", flush=True)
