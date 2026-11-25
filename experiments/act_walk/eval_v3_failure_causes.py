#!/usr/bin/env python3
"""Failure causes of the v3 checkpoint evaluation (2026-11-24 user request): every failed episode gets one or more
cause labels (multi-label), counted per cause and drawn as a bar chart (x = cause, y = number of failed episodes).

usage: eval_v3_failure_causes.py [out dir]      (teleop env python: numpy, matplotlib)
Reads logs/policy_eval/*_v3_<step>_nas25_far (summary.jsonl + ep<i>_trace.npz) and the full recordings
/home/jammin/g1iv_store/policy_eval/<date>/<episode_id>/arrays.npz. Writes
  failure_episodes.csv   one row per failed episode: measurements + one 0/1 column per cause + n_causes
  cause_counts.csv       per cause: failed episodes with it, share of all failures
  thresholds.json        the thresholds actually used (derived from the successful episodes, see below)
  failure_causes.png     the bar chart

Every threshold is the range seen in the SUCCESSFUL episodes of the same evaluation ("never happens in a success"),
except the knock-over angle (80 deg = lying on its side) and the empty-hand closure (finger_min < 0.40: the fingers
closed all the way to 0.25; with the cylinder between them they stop at 0.54 or more, also in failed episodes). Measurements:
  stance dx/dy  pelvis xy at the arm onset (|arm - arm[0]| > 0.15 rad) minus the expert stand point
                object_xy + (-0.292, +0.136); dx > 0 = closer to the desk, dy > 0 = further left
  close         first frame the hand command moves > 0.3 from its start (hand_close.py rule)
  cyl_in_hand   cylinder position in the right-hand body frame at the close (alignment of the grip)
  finger_min    smallest mean right-finger closure (obs, 1 open .. 0.25 fully closed) in the 2 s after the close;
                a cylinder between the fingers stops them early, an empty hand closes fully
  push          cylinder xy displacement from the start to the close (to the end if never closed)
  hand_rise     right hand height gain after the close (the lift motion)
  tilt          largest cylinder tilt from upright over the episode
The hand contact sensor is not used: it reads 0 in successful grasps too.
"""
import csv
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np

WS = Path("/mnt/isaac/g1_inspire_vla_ws")
RAW = "/home/jammin/g1iv_store/policy_eval"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/home/jammin/g1iv_store/eval_v3/failure_causes")
OFF = np.array([-0.292, 0.136])
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q):
    w, x, y, z = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def measure(d, r):
    z = np.load(f"{d}/ep{r['i']:02d}_trace.npz")
    act, bp = z["action"], z["base_pos"]
    mv = np.abs(act[:, :14] - act[0, :14]).max(1) > 0.15
    on = int(np.argmax(mv)) if mv.any() else len(act) - 1
    e = bp[on, :2] - (np.asarray(r["object_xy"]) + OFF)
    a = np.load(glob.glob(f"{RAW}/*/{r['episode_id']}/arrays.npz")[0])
    n = len(a["sim_time"])
    tgt = a["action/hand_q_norm_target"]
    closed = np.abs(tgt - tgt[0]).max(1) > 0.3
    ic = int(np.argmax(closed)) if closed.any() else None
    obj, hand = a["eval/object_pos_w"], a["eval/body_pose_w/right_hand"]
    m = {"stance_dx": e[0], "stance_dy": e[1], "closed": ic is not None, "close_frame": ic,
         "tilt_max_deg": max(tilt_deg(q) for q in a["eval/object_quat_w"]),
         "push_m": float(np.linalg.norm(obj[ic if ic is not None else n - 1, :2] - obj[0, :2])),
         "max_lift_m": r["max_lift_m"]}
    if ic is not None:
        loc = rot(hand[ic, 3:7]).T @ (obj[ic] - hand[ic, :3])
        m.update(cyl_in_hand_x=loc[0], cyl_in_hand_y=loc[1], cyl_in_hand_z=loc[2],
                 finger_min=float(a["obs/hand_q_norm"][ic:ic + 50, 0:5].mean(1).min()),
                 hand_rise_m=float(hand[ic:, 2].max() - hand[ic, 2]))
    return m


eps = []
for d in sorted(x for x in glob.glob(str(WS / "logs/policy_eval/*_v3_*_nas25_far")) if "_incomplete_" not in x):
    step = int(d.split("_v3_")[1][:6])
    for line in open(f"{d}/summary.jsonl"):
        r = json.loads(line)
        eps.append({"checkpoint": f"{step // 1000}k", "episode": r["i"] + 1, "success": r["outcome"] == "success",
                    "robot_start_y": r["robot_pose"][1], "cylinder_y": r["object_xy"][1], **measure(d, r)})
S = [e for e in eps if e["success"]]
F = [e for e in eps if not e["success"]]


def rng(k):
    v = [e[k] for e in S if e.get(k) is not None]
    return float(min(v)), float(max(v))


T = {k: rng(k) for k in ("stance_dx", "stance_dy", "cyl_in_hand_x", "cyl_in_hand_y", "cyl_in_hand_z", "finger_min",
                         "hand_rise_m", "push_m")}
T["knock_over_deg"] = 80.0
T["finger_empty_below"] = 0.40   # fully closed = 0.25; with the cylinder between them the fingers stop at >= 0.54


def outside(e, k):
    return e.get(k) is not None and not (T[k][0] <= e[k] <= T[k][1])


grasped = lambda e: e["closed"] and e["finger_min"] >= T["finger_empty_below"]
# (stage, label for the chart, column name, rule)
CAUSES = [
    ("정지", "오른쪽으로\n치우쳐 섬", "stance_right", lambda e: e["stance_dy"] < T["stance_dy"][0]),
    ("정지", "왼쪽으로\n너무 치우쳐 섬", "stance_left", lambda e: e["stance_dy"] > T["stance_dy"][1]),
    ("정지", "책상에\n너무 붙어 섬", "stance_too_close", lambda e: e["stance_dx"] > T["stance_dx"][1]),
    ("정지", "책상에서\n너무 멀리 섬", "stance_too_far", lambda e: e["stance_dx"] < T["stance_dx"][0]),
    ("잡기", "손을\n닫지 않음", "never_closed", lambda e: not e["closed"]),
    ("잡기", "손이 원통과\n어긋난 채 닫음", "misaligned_close",
     lambda e: e["closed"] and any(outside(e, k) for k in ("cyl_in_hand_x", "cyl_in_hand_y", "cyl_in_hand_z"))),
    ("잡기", "빈손으로 닫힘\n(원통이 안 들어옴)", "empty_close", lambda e: e["closed"] and not grasped(e)),
    ("잡기", "닫기 전에\n원통을 밀어냄", "pushed_before_close", lambda e: e["push_m"] > T["push_m"][1]),
    ("들기", "잡고도\n팔을 들지 않음", "grasped_no_lift",
     lambda e: grasped(e) and e["hand_rise_m"] < T["hand_rise_m"][0]),
    ("들기", "들다가\n미끄러져 놓침", "slipped_while_lifting",
     lambda e: grasped(e) and e["hand_rise_m"] >= T["hand_rise_m"][0]),
    ("기타", "원통을\n넘어뜨림", "knocked_over", lambda e: e["tilt_max_deg"] > T["knock_over_deg"]),
]
for e in F:
    for _, _, col, rule in CAUSES:
        e[col] = int(bool(rule(e)))
    e["n_causes"] = sum(e[c] for _, _, c, _ in CAUSES)

OUT.mkdir(parents=True, exist_ok=True)
cols = ["checkpoint", "episode", "robot_start_y", "cylinder_y", "stance_dx", "stance_dy", "close_frame", "cyl_in_hand_x",
        "cyl_in_hand_y", "cyl_in_hand_z", "finger_min", "hand_rise_m", "push_m", "max_lift_m", "tilt_max_deg"]
with open(OUT / "failure_episodes.csv", "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(cols + [c for _, _, c, _ in CAUSES] + ["n_causes"])
    for e in F:
        w.writerow([round(e[k], 4) if isinstance(e.get(k), float) else e.get(k) for k in cols]
                   + [e[c] for _, _, c, _ in CAUSES] + [e["n_causes"]])
counts = [(st, lab, col, sum(e[col] for e in F)) for st, lab, col, _ in CAUSES]
with open(OUT / "cause_counts.csv", "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["stage", "cause", "column", "failed_episodes", "share_of_failures"])
    for st, lab, col, k in counts:
        w.writerow([st, lab.replace("\n", " "), col, k, round(k / len(F), 3)])
(OUT / "thresholds.json").write_text(json.dumps({"episodes": len(eps), "successes": len(S), "failures": len(F),
                                                 "checkpoints": sorted({e["checkpoint"] for e in eps}, key=lambda s: int(s[:-1])),
                                                 "thresholds_success_range": T}, indent=1, ensure_ascii=False))

# ---------------- chart (single series: one hue, value on the cap, stage groups under the axis)
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402

font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
INK, INK2, MUTED, GRID, SURF, S1 = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#fcfcfb", "#2a78d6"
shown = [c for c in counts if c[3] > 0]
none = [c[1].replace("\n", " ") for c in counts if c[3] == 0]
fig, ax = plt.subplots(figsize=(11, 5.6), dpi=150, facecolor=SURF)
ax.set_facecolor(SURF)
ymax = max(c[3] for c in shown)
top = math.ceil((ymax + 2) / 5) * 5
bw, rad = 0.36, 0.06 * top / 5          # bar width (data units), corner radius scaled to the y range
for i, (st, lab, col, k) in enumerate(shown):
    x0 = i - bw / 2
    r = min(rad, k / 2)
    ax.add_patch(FancyBboxPatch((x0, 0), bw, k, boxstyle=f"round,pad=0,rounding_size={r}", mutation_aspect=1,
                                fc=S1, ec="none"))
    ax.add_patch(Rectangle((x0, 0), bw, min(k, r * 1.5), fc=S1, ec="none"))   # square at the baseline
    ax.text(i, k + top * 0.015, f"{k}", ha="center", va="bottom", color=INK, fontsize=11)
ax.set_xlim(-0.6, len(shown) - 0.4)
ax.set_ylim(0, top)
ax.set_xticks(range(len(shown)))
ax.set_xticklabels([c[1] for c in shown], fontsize=9.5, color=INK2, linespacing=1.25)
ax.set_ylabel("실패 시연 수", color=INK2, fontsize=10)
ax.yaxis.set_major_locator(matplotlib.ticker.MultipleLocator(5))
ax.grid(axis="y", color=GRID, lw=0.8)
ax.set_axisbelow(True)
for s in ("top", "right", "left"):
    ax.spines[s].set_visible(False)
ax.spines["bottom"].set_color(GRID)
ax.tick_params(colors=INK2, length=0)
# stage groups under the tick labels
stages = []
for i, c in enumerate(shown):
    if not stages or stages[-1][0] != c[0]:
        stages.append([c[0], i, i])
    else:
        stages[-1][2] = i
for st, a0, a1 in stages:
    ax.annotate("", xy=(a0 - 0.3, -0.17), xytext=(a1 + 0.3, -0.17), xycoords=("data", "axes fraction"),
                arrowprops=dict(arrowstyle="-", color=GRID, lw=1.2), annotation_clip=False)
    ax.text((a0 + a1) / 2, -0.21, {"정지": "서는 위치", "잡기": "손 뻗기·닫기", "들기": "들어 올리기", "기타": "기타"}[st],
            transform=matplotlib.transforms.blended_transform_factory(ax.transData, ax.transAxes),
            ha="center", va="top", color=MUTED, fontsize=9)
cks = json.loads((OUT / "thresholds.json").read_text())["checkpoints"]
ax.set_title(f"ACT v3 실패 원인 (체크포인트 {cks[0]}–{cks[-1]}, 실패 {len(F)}회 / 전체 {len(eps)}회, 한 시연에 원인 여러 개 중복 집계)",
             color=INK, fontsize=11.5, loc="left", pad=12)
multi = sum(e["n_causes"] >= 2 for e in F)
note = (f"기준: 성공한 {len(S)}회에서 한 번도 나오지 않은 범위.  원인 2개 이상인 실패 {multi}/{len(F)}회, "
        f"실패 1회당 평균 {np.mean([e['n_causes'] for e in F]):.1f}개.")
if none:
    note += "  해당 없음(0회): " + ", ".join(none) + "."
fig.text(0.01, 0.012, note, color=INK2, fontsize=8)
fig.tight_layout(rect=(0, 0.04, 1, 1))
fig.subplots_adjust(bottom=0.27)
fig.savefig(OUT / "failure_causes.png", facecolor=SURF)

print(f"{len(eps)} episodes, {len(F)} failures -> {OUT}")
for st, lab, col, k in counts:
    print(f"  {st} {lab.replace(chr(10), ' '):24s} {k:3d}  ({100 * k / len(F):.0f} %)")
