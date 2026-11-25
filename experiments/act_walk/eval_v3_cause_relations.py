#!/usr/bin/env python3
"""Relations between the failure causes of the v3 checkpoint evaluation (2026-11-24 user request): which causes are
seen together in the same failed episode, how strongly, and which causes form groups.

usage: eval_v3_cause_relations.py [failure_causes dir]      (teleop env python: numpy, scipy, matplotlib)
Input: <dir>/failure_episodes.csv from eval_v3_failure_causes.py (one row per failed episode, one 0/1 column per cause).
Indicators (over the failed episodes only; causes that never occurred are left out):
  both            failed episodes with A and B
  P(B|A)          both / n_A: "when A is present, B is present too" (directional)
  jaccard         both / (n_A + n_B - both): symmetric overlap, 1 = always together, 0 = never
  lift            both * N / (n_A * n_B): > 1 together more often than if independent
  fisher_p        two-sided Fisher exact test on the 2 x 2 table (descriptive only: N is small, many pairs)
  exclusive       1 = the two causes cannot occur together by definition (e.g. 'never closed' vs 'empty close')
Groups: average-linkage hierarchical clustering on the Jaccard distance (1 - jaccard), cut at distance 0.8, i.e. causes
in one group co-occur with an average Jaccard of at least 0.2.
Writes cause_pairs.csv, cooccurrence_counts.csv, conditional_prob.csv, jaccard.csv, cause_groups.csv,
cause_combinations.csv, episode_groups.csv, cause_relations.png (matrices + dendrogram), cause_combinations.png
(UpSet-style: how often each exact combination of causes occurs).
"""
import csv
import itertools
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.cluster.hierarchy import dendrogram, fcluster, linkage
from scipy.spatial.distance import squareform
from scipy.stats import fisher_exact

D = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/home/jammin/g1iv_store/eval_v3/failure_causes")
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
CUT = 0.8
SHORT = {"stance_right": "오른쪽 치우침", "stance_left": "왼쪽 치우침", "stance_too_close": "책상에 붙음",
         "stance_too_far": "책상에서 멂", "never_closed": "손 안 닫음", "misaligned_close": "손 어긋남",
         "empty_close": "빈손 닫힘", "pushed_before_close": "닫기 전 밀어냄", "grasped_no_lift": "잡고 안 듦",
         "slipped_while_lifting": "들다 놓침", "knocked_over": "넘어뜨림"}
EXCL = {frozenset(p) for p in [("stance_right", "stance_left"), ("stance_too_close", "stance_too_far"),
                               ("never_closed", "misaligned_close"), ("never_closed", "empty_close"),
                               ("never_closed", "grasped_no_lift"), ("never_closed", "slipped_while_lifting"),
                               ("empty_close", "grasped_no_lift"), ("empty_close", "slipped_while_lifting"),
                               ("grasped_no_lift", "slipped_while_lifting")]}

rows = list(csv.DictReader(open(D / "failure_episodes.csv", encoding="utf-8-sig")))
allc = [c for c in SHORT if c in rows[0]]
C = [c for c in allc if any(int(r[c]) for r in rows)]
X = np.array([[int(r[c]) for c in C] for r in rows])
N, K = len(X), len(C)
B = X.T @ X                                   # both (diagonal = n_A)
n = np.diag(B).astype(float)
P = B / n[:, None]                            # P(col | row)
J = B / (n[:, None] + n[None, :] - B)
lift = B * N / np.outer(n, n)
pv = np.ones((K, K))
for i, j in itertools.combinations(range(K), 2):
    t = [[B[i, j], n[i] - B[i, j]], [n[j] - B[i, j], N - n[i] - n[j] + B[i, j]]]
    pv[i, j] = pv[j, i] = fisher_exact(t)[1]
ex = np.array([[frozenset((a, b)) in EXCL for b in C] for a in C])

# ---- groups
Dm = 1 - J
np.fill_diagonal(Dm, 0)
Z = linkage(squareform(Dm, checks=False), "average")
lab = fcluster(Z, CUT, "distance")
order = dendrogram(Z, no_plot=True, count_sort="descending")["leaves"]
groups = []                                   # in dendrogram order
for i in order:
    if not groups or groups[-1]["id"] != lab[i]:
        groups.append({"id": lab[i], "members": [i]})
    else:
        groups[-1]["members"].append(i)
for k, g in enumerate(groups, 1):
    g["name"] = f"묶음 {k}"
    m = g["members"]
    g["support"] = int(sum(X[e, m].sum() >= 2 for e in range(N))) if len(m) > 1 else int(n[m[0]])
    g["mean_jaccard"] = float(np.mean([J[a, b] for a, b in itertools.combinations(m, 2)])) if len(m) > 1 else None

# ---- data files
w = lambda name: csv.writer(open(D / name, "w", newline="", encoding="utf-8-sig"))
f = w("cause_pairs.csv")
f.writerow(["cause_A", "cause_B", "n_A", "n_B", "both", "P(B|A)", "P(A|B)", "jaccard", "lift", "fisher_p", "exclusive"])
for i, j in itertools.combinations(range(K), 2):
    f.writerow([SHORT[C[i]], SHORT[C[j]], int(n[i]), int(n[j]), int(B[i, j]), round(P[i, j], 3), round(P[j, i], 3),
                round(J[i, j], 3), round(lift[i, j], 2), round(pv[i, j], 4), int(ex[i, j])])
for name, M, fmt in (("cooccurrence_counts.csv", B, int), ("conditional_prob.csv", P, lambda v: round(v, 3)),
                     ("jaccard.csv", J, lambda v: round(v, 3))):
    f = w(name)
    f.writerow(["row \\ col" + (" (P(col|row))" if "cond" in name else "")] + [SHORT[c] for c in C])
    for i in range(K):
        f.writerow([SHORT[C[i]]] + [fmt(M[i, j]) for j in range(K)])
f = w("cause_groups.csv")
f.writerow(["group", "cause", "column", "n", "group_members", "episodes_with_2+_members", "mean_jaccard_in_group"])
for g in groups:
    for i in g["members"]:
        f.writerow([g["name"], SHORT[C[i]], C[i], int(n[i]), " + ".join(SHORT[C[k]] for k in g["members"]),
                    g["support"], None if g["mean_jaccard"] is None else round(g["mean_jaccard"], 3)])
combo = Counter(tuple(i for i in order if X[e, i]) for e in range(N))
eps_of = {}
for e in range(N):
    eps_of.setdefault(tuple(i for i in order if X[e, i]), []).append(f"{rows[e]['checkpoint']}-{rows[e]['episode']}")
combos = sorted(combo.items(), key=lambda kv: (-kv[1], -len(kv[0])))
f = w("cause_combinations.csv")
f.writerow(["causes", "n_causes", "failed_episodes", "share_of_failures", "episodes"])
for cb, k in combos:
    f.writerow([" + ".join(SHORT[C[i]] for i in cb), len(cb), k, round(k / N, 3), " ".join(eps_of[cb])])
f = w("episode_groups.csv")
f.writerow(["checkpoint", "episode", "causes", "groups_touched"])
for e in range(N):
    f.writerow([rows[e]["checkpoint"], rows[e]["episode"], " + ".join(SHORT[C[i]] for i in order if X[e, i]),
                " + ".join(g["name"] for g in groups if X[e, g["members"]].any())])

# ---------------- figures
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402

font_manager.fontManager.addfont(FONT)
plt.rcParams["font.family"] = font_manager.FontProperties(fname=FONT).get_name()
INK, INK2, MUTED, GRID, SURF, ZERO, DIAG = "#0b0b0b", "#52514e", "#8a8984", "#e4e3df", "#fcfcfb", "#f1f0ec", "#e4e3df"
RAMP = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6", "#256abf", "#1c5cab",
        "#184f95", "#104281", "#0d366b"]       # sequential blue 100 -> 700
S1 = "#2a78d6"


def ramp(v):
    if v <= 0:
        return ZERO, INK2
    k = min(len(RAMP) - 1, int(round(v * (len(RAMP) - 1))))
    return RAMP[k], ("#ffffff" if k >= 7 else INK)


names = [SHORT[C[i]] for i in order]
gpos = []                                      # (start, end) index ranges of the groups in plot order
s = 0
for g in groups:
    gpos.append((s, s + len(g["members"]) - 1, g))
    s += len(g["members"])

fig = plt.figure(figsize=(17, 9.6), dpi=150, facecolor=SURF)
gs = fig.add_gridspec(2, 2, height_ratios=[1.1, 5], width_ratios=[1, 1], hspace=0.02, wspace=0.28,
                      left=0.12, right=0.985, top=0.885, bottom=0.17)
axd = fig.add_subplot(gs[0, 0])
axj = fig.add_subplot(gs[1, 0])
axp = fig.add_subplot(gs[1, 1])
# dendrogram above the Jaccard matrix (x in cell units)
dd = dendrogram(Z, no_plot=True, count_sort="descending")
for xs, ys in zip(dd["icoord"], dd["dcoord"]):
    axd.plot([(x - 5) / 10 for x in xs], ys, color=INK2, lw=1.2, solid_capstyle="round")
axd.axhline(CUT, color=MUTED, lw=0.9, ls=(0, (4, 3)))
axd.text(-0.62, CUT, f"묶음 기준 {CUT}", ha="right", va="center", fontsize=8.5, color=MUTED, clip_on=False)
axd.set_xlim(-0.5, K - 0.5)
axd.set_ylim(0, 1.02)
axd.axis("off")
axd.set_title("① 함께 나온 횟수와 자카드 유사도", loc="left", fontsize=12, color=INK, pad=4)


def matrix(ax, M, txt, exclusive_mark):
    gap = 0.04
    for a, i in enumerate(order):
        for b, j in enumerate(order):
            y = a
            if i == j:
                fc, tc = DIAG, INK
                t = f"{int(n[i])}"
            elif ex[i, j]:
                fc, tc, t = SURF, MUTED, exclusive_mark
            else:
                fc, tc = ramp(M[i, j])
                t = txt(i, j)
            ax.add_patch(Rectangle((b - 0.5 + gap, y - 0.5 + gap), 1 - 2 * gap, 1 - 2 * gap, fc=fc, ec="none"))
            if ex[i, j] and i != j:
                ax.add_patch(Rectangle((b - 0.5 + gap, y - 0.5 + gap), 1 - 2 * gap, 1 - 2 * gap, fc="none", ec=GRID, lw=0.8))
            ax.text(b, y, t, ha="center", va="center", fontsize=10 if i == j else 9.5, color=tc,
                    fontweight="bold" if i == j else "normal")
    for s0, s1, g in gpos:                     # group boxes
        ax.add_patch(Rectangle((s0 - 0.5, s0 - 0.5), s1 - s0 + 1, s1 - s0 + 1, fc="none", ec=INK, lw=1.6))
    ax.set_xlim(-0.5, K - 0.5)
    ax.set_ylim(K - 0.5, -0.5)
    ax.set_xticks(range(K))
    ax.set_xticklabels(names, rotation=40, ha="right", rotation_mode="anchor", fontsize=10, color=INK2)
    ax.set_yticks(range(K))
    ax.set_yticklabels(names, fontsize=10, color=INK2)
    ax.tick_params(length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_aspect("equal")


def jtxt(i, j):
    if B[i, j] == 0:
        return "0"
    star = "*" if pv[i, j] < 0.05 else ""
    return f"{int(B[i, j])}{star}\n{J[i, j]:.2f}"


matrix(axj, J, jtxt, "×")
matrix(axp, P, lambda i, j: f"{100 * P[i, j]:.0f}%", "×")
axp.set_title("② 행 원인이 있을 때 열 원인도 있을 확률  P(열 | 행)", loc="left", fontsize=12, color=INK, pad=8)
fig.canvas.draw()                              # aspect applied: align the dendrogram with the matrix columns
pj, pd_ = axj.get_position(), axd.get_position()
axd.set_position([pj.x0, pj.y1 + 0.004, pj.width, pd_.height])
rend = fig.canvas.get_renderer()
x_lab = min(t.get_window_extent(rend).x0 for t in axj.get_yticklabels()) / fig.bbox.width - 0.008
tr = matplotlib.transforms.blended_transform_factory(fig.transFigure, axj.transData)
for s0, s1, g in gpos:                         # group names left of the Jaccard matrix row labels
    axj.text(x_lab, (s0 + s1) / 2, g["name"], ha="right", va="center", fontsize=9.5, color=INK, transform=tr)
fig.suptitle(f"ACT v3 실패 원인 사이의 관계 (실패 {N}회, 체크포인트 20k–140k)", x=0.01, ha="left", y=0.975,
             fontsize=14, color=INK)
gl = "   ".join(f"{g['name']}: " + " + ".join(SHORT[C[i]] for i in g["members"]) +
                (f" (함께 나온 실패 {g['support']}회)" if len(g["members"]) > 1 else f" ({g['support']}회, 단독)")
                for g in groups)
fig.text(0.01, 0.065, gl, fontsize=9, color=INK)
fig.text(0.01, 0.035, "①: 칸 위 숫자 = 두 원인이 같은 실패 시연에 함께 나온 횟수, 아래 = 자카드 유사도(함께 나온 횟수 ÷ 둘 중 하나라도 나온 횟수), "
         "색이 진할수록 자주 함께 나옴, * = 피셔 정확검정 p < 0.05, 대각선 = 원인별 총 횟수.  "
         "굵은 상자 = 묶음(자카드 거리 평균 연결 군집, 평균 자카드 0.2 이상이면 같은 묶음).",
         fontsize=8.5, color=INK2)
fig.text(0.01, 0.012, "②: 행 원인이 있는 실패 중 열 원인도 있는 비율 (비대칭: 행→열 방향으로 읽음).  "
         "× = 정의상 함께 나올 수 없는 쌍 (예: 손 안 닫음 ↔ 빈손 닫힘).  실패가 35회뿐이라 1–2회짜리 관계는 우연일 수 있음.",
         fontsize=8.5, color=INK2)
fig.savefig(D / "cause_relations.png", facecolor=SURF)
plt.close(fig)

# ---- UpSet-style combinations
M = len(combos)
fig = plt.figure(figsize=(max(10, 0.62 * M + 4.5), 8.2), dpi=150, facecolor=SURF)
gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 1], width_ratios=[1, 4.6], hspace=0.04, wspace=0.24,
                      left=0.03, right=0.99, top=0.9, bottom=0.07)
axb = fig.add_subplot(gs[0, 1])
axm = fig.add_subplot(gs[1, 1], sharex=axb)
axt = fig.add_subplot(gs[1, 0], sharey=axm)
top = max(k for _, k in combos)
bw = 0.5
for x, (cb, k) in enumerate(combos):
    r = min(0.1, k / 2)
    axb.add_patch(FancyBboxPatch((x - bw / 2, 0), bw, k, boxstyle=f"round,pad=0,rounding_size={r}", fc=S1, ec="none",
                                 mutation_aspect=top / 12))
    axb.add_patch(Rectangle((x - bw / 2, 0), bw, min(k, 0.3), fc=S1, ec="none"))
    axb.text(x, k + top * 0.02, f"{k}", ha="center", va="bottom", fontsize=10, color=INK)
axb.set_ylim(0, top * 1.15)
axb.set_ylabel("실패 시연 수", color=INK2, fontsize=10)
axb.grid(axis="y", color=GRID, lw=0.8)
axb.set_axisbelow(True)
axb.tick_params(axis="x", bottom=False, labelbottom=False)
axb.tick_params(colors=INK2, length=0)
for sp in ("top", "right", "left", "bottom"):
    axb.spines[sp].set_visible(False)
axb.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
axb.set_title(f"원인 조합별 실패 횟수 (실패 {N}회, 조합 {M}가지; 아래 점 = 그 조합에 들어 있는 원인)", loc="left",
              fontsize=12, color=INK, pad=10)
# group bands + dots
for gi, (s0, s1, g) in enumerate(gpos):
    if gi % 2 == 0:
        axm.add_patch(Rectangle((-0.6, s0 - 0.5), M + 0.2, s1 - s0 + 1, fc="#f4f3ef", ec="none", zorder=0))
for x, (cb, k) in enumerate(combos):
    ys = [order.index(i) for i in cb]
    axm.scatter([x] * K, range(K), s=42, color=GRID, zorder=1, linewidths=0)
    if len(ys) > 1:
        axm.plot([x, x], [min(ys), max(ys)], color=INK, lw=2, zorder=2, solid_capstyle="round")
    axm.scatter([x] * len(ys), ys, s=60, color=INK, zorder=3, edgecolors=SURF, linewidths=1.5)
axm.set_xlim(-0.6, M - 0.4)
axm.set_ylim(K - 0.5, -0.5)
axm.set_xticks([])
for sp in axm.spines.values():
    sp.set_visible(False)
# per-cause totals (left), labels with group names
for y, i in enumerate(order):
    axt.barh(y, n[i], height=0.5, color="#86b6ef", ec="none")
    axt.text(n[i] + 0.4, y, f"{int(n[i])}", va="center", ha="right", fontsize=8.5, color=INK2)
axt.set_xlim(max(n) * 1.25 + 1, 0)
axm.set_yticks(range(K))
axm.set_yticklabels(names, fontsize=10, color=INK)
axm.tick_params(axis="y", length=0, pad=8)
axt.tick_params(axis="y", left=False, labelleft=False)
axt.set_xticks([])
for sp in axt.spines.values():
    sp.set_visible(False)
for s0, s1, g in gpos:
    axt.text(max(n) * 1.25 + 0.6, (s0 + s1) / 2, g["name"], ha="left", va="center", fontsize=9, color=MUTED)
fig.text(0.03, 0.025, "왼쪽 막대 = 원인별 총 횟수(중복 집계).  회색 띠 = 관계 그림의 묶음.  각 실패 시연은 정확히 하나의 조합에 들어가므로 위 막대의 합 = 실패 수.",
         fontsize=8.5, color=INK2)
fig.savefig(D / "cause_combinations.png", facecolor=SURF)

print(f"{N} failures, {K} causes, {len(groups)} groups, {M} combinations -> {D}")
for g in groups:
    print(f"  {g['name']}: " + " + ".join(SHORT[C[i]] for i in g["members"]) + f"  (2+ together: {g['support']}, "
          f"mean J {g['mean_jaccard']})")
for cb, k in combos[:6]:
    print(f"  {k:2d}  " + " + ".join(SHORT[C[i]] for i in cb))
