"""Summarise automatic walk -> grasp runs (hub traces logs/teleop_runs/*.json of ScriptedWalkGraspSource).

    python tools/analyze_walk_grasp.py logs/teleop_runs/<run>.json [...]      one line + phase log per run
"""
import json
import sys

import numpy as np


def summary(path: str, verbose: bool = True) -> dict:
    d = json.load(open(path))
    res = d.get("result") or {}
    log = d.get("source_log") or []
    nav = res.get("nav") or {}
    tr = d.get("trace") or []
    st = [r.get("sim_time") for r in tr if r.get("sim_time") is not None]
    out = {"run": d.get("run_id"), "success": res.get("success"), "lift_m": res.get("object_lift_m"),
           "fallen": res.get("fallen"), "error": res.get("error"), "nav_ok": nav.get("ok"),
           "nav_along": nav.get("along"), "nav_cross": nav.get("cross"), "nav_yaw_deg": nav.get("yaw_err_deg"),
           "n_corr": nav.get("n_corr"), "ik_fail": d.get("n_ik_fail"),
           "sim_s": (st[-1] - st[0]) if len(st) > 1 else None}
    if verbose:
        print(f"== {path}")
        for e in log:
            items = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items() if k not in ("phase", "t")}
            print(f"  t {e.get('t', 0):6.2f}  {e.get('phase'):<12} {json.dumps(items, default=str)[:220]}")
        print("  result:", json.dumps(res, default=lambda o: round(o, 4) if isinstance(o, float) else str(o))[:400])
    return out


if __name__ == "__main__":
    rows = [summary(p, verbose=len(sys.argv) <= 3) for p in sys.argv[1:]]
    if len(rows) > 1:
        ok = [r for r in rows if r["success"]]
        print(f"\n{len(ok)}/{len(rows)} success")
        for r in rows:
            print("  " + "  ".join(f"{k}={(round(v, 3) if isinstance(v, float) else v)}" for k, v in r.items()))
        lifts = [r["lift_m"] for r in ok if r["lift_m"] is not None]
        if lifts:
            print(f"lift on success: mean {np.mean(lifts):.3f} min {np.min(lifts):.3f} m")
