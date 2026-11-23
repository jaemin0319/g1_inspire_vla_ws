#!/usr/bin/env python3
"""Aggregate the v3 checkpoint evaluation (v3_pipeline.sh eval) into data files.

usage: eval_v3_summary.py <logs/policy_eval root> <out dir> [episodes per checkpoint]
Reads <root>/*_v3_<step>_nas25_far/summary.jsonl (one row per episode, tools/run_act_policy.py) and writes
  <out>/episodes.csv       one row per checkpoint x episode (start pose, cylinder, outcome, reason, frames, lift ...)
  <out>/checkpoints.csv    per checkpoint: successes / n, rate, Wilson 95 % interval, failure reasons, median frames
  <out>/summary.json       the same as JSON + breakdowns (cylinder y inside / outside the v2 training range,
                           sideways start offset)
  <out>/success_rate.png   success rate per checkpoint with 95 % intervals
Run with the teleop env python (matplotlib).
"""
from __future__ import annotations

import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

root, out = Path(sys.argv[1]), Path(sys.argv[2])
n_expect = int(sys.argv[3]) if len(sys.argv) > 3 else 30
out.mkdir(parents=True, exist_ok=True)


def wilson(k: int, n: int, z: float = 1.96):
    if n == 0:
        return (float("nan"),) * 2
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


rows, per = [], {}
for d in sorted(root.glob("*_v3_*_nas25_far")):
    if "_incomplete_" in d.name or not (d / "summary.jsonl").is_file():
        continue
    step = d.name.split("_v3_")[1].split("_")[0]
    eps = [json.loads(x) for x in (d / "summary.jsonl").read_text().splitlines() if x.strip()]
    if len(eps) < n_expect:
        print(f"skip {d.name}: {len(eps)} < {n_expect} episodes")
        continue
    per[step] = (d, eps[:n_expect])
for step in sorted(per):
    d, eps = per[step]
    for r in eps:
        rp, ox = r["robot_pose"], r["object_xy"]
        rows.append({"checkpoint": int(step), "episode": r["i"], "robot_x": rp[0], "robot_y": rp[1], "robot_yaw_deg": rp[2],
                     "object_x": ox[0], "object_y": ox[1], "object_y_in_v2_range": -0.30 <= ox[1] <= -0.10,
                     "outcome": r["outcome"], "reason": r["reason"], "frames": r["frames"],
                     "max_lift_m": r["max_lift_m"], "fallen": r["fallen"], "walk_cmd_frames": r.get("walk_cmd_frames"),
                     "base_travel_m": r.get("base_travel_m"), "wall_s": r.get("wall_s"),
                     "episode_id": r.get("episode_id"), "eval_folder": d.name})
if not rows:
    sys.exit("no complete checkpoint evaluations found")
with open(out / "episodes.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

ck = []
for step in sorted(per):
    R = [r for r in rows if r["checkpoint"] == int(step)]
    k = sum(r["outcome"] == "success" for r in R)
    lo, hi = wilson(k, len(R))
    fails = Counter("fallen" if r["fallen"] else ("timeout, nothing lifted" if r["max_lift_m"] < 0.01 else
                    "timeout, lifted < 5 cm or dropped") for r in R if r["outcome"] != "success")
    succ_frames = sorted(r["frames"] for r in R if r["outcome"] == "success")

    def rate(sel):
        s = [r for r in R if sel(r)]
        return {"n": len(s), "success": sum(r["outcome"] == "success" for r in s)}

    ck.append({"checkpoint": int(step), "episodes": len(R), "successes": k, "success_rate": round(k / len(R), 4),
               "ci95_low": round(lo, 4), "ci95_high": round(hi, 4),
               "median_frames_success": succ_frames[len(succ_frames) // 2] if succ_frames else None,
               "fail_fallen": fails.get("fallen", 0), "fail_no_lift": fails.get("timeout, nothing lifted", 0),
               "fail_partial_lift": fails.get("timeout, lifted < 5 cm or dropped", 0),
               "object_y_in_v2_range": rate(lambda r: r["object_y_in_v2_range"]),
               "object_y_outside_v2_range": rate(lambda r: not r["object_y_in_v2_range"]),
               "robot_offset_le_0.5m": rate(lambda r: abs(r["robot_y"] - r["object_y"]) <= 0.5),
               "robot_offset_gt_0.5m": rate(lambda r: abs(r["robot_y"] - r["object_y"]) > 0.5)})
flat = [{k: v for k, v in c.items() if not isinstance(v, dict)} for c in ck]
with open(out / "checkpoints.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(flat[0]))
    w.writeheader()
    w.writerows(flat)
(out / "summary.json").write_text(json.dumps({"episodes_per_checkpoint": n_expect, "checkpoints": ck}, indent=1))
for c in flat:
    print(f"{c['checkpoint']:>6}: {c['successes']:2d}/{c['episodes']} = {100 * c['success_rate']:5.1f} % "
          f"(95 % {100 * c['ci95_low']:.0f}-{100 * c['ci95_high']:.0f} %)  fallen {c['fail_fallen']} "
          f"no-lift {c['fail_no_lift']} partial {c['fail_partial_lift']}")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

INK, INK2, GRID, SURF, S1 = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb", "#2a78d6"
x = [c["checkpoint"] / 1000 for c in flat]
y = [100 * c["success_rate"] for c in flat]
lo = [100 * (c["success_rate"] - c["ci95_low"]) for c in flat]
hi = [100 * (c["ci95_high"] - c["success_rate"]) for c in flat]
fig, ax = plt.subplots(figsize=(8, 4.2), dpi=150, facecolor=SURF)
ax.set_facecolor(SURF)
ax.errorbar(x, y, yerr=[lo, hi], color=S1, lw=2, marker="o", ms=7, mec=SURF, mew=2, capsize=0, elinewidth=1.2,
            ecolor=S1 + "80")
best = max(range(len(y)), key=lambda i: y[i])
ax.annotate(f"{flat[best]['successes']}/{flat[best]['episodes']}", (x[best], y[best]), textcoords="offset points",
            xytext=(8, 6), ha="left", color=INK, fontsize=9)
ax.set_xticks(x)
ax.set_xticklabels([f"{int(v)}k" for v in x])
ax.set_ylim(0, 100)
ax.set_ylabel("success rate (%)", color=INK2)
ax.set_xlabel("training step", color=INK2)
ax.set_title(f"ACT v3 (wide random starts): closed-loop success per checkpoint, {n_expect} episodes each",
             color=INK, fontsize=10, loc="left")
ax.grid(axis="y", color=GRID, lw=0.8)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
for s in ("left", "bottom"):
    ax.spines[s].set_color(GRID)
ax.tick_params(colors=INK2)
fig.text(0.01, 0.01, "whiskers: Wilson 95 % interval; same 30 random starts for every checkpoint", color=INK2, fontsize=8)
fig.tight_layout(rect=(0, 0.03, 1, 1))
fig.savefig(out / "success_rate.png", facecolor=SURF)
print("wrote", out)
