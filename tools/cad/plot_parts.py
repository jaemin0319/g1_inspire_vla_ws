"""Orthographic views (XY, XZ, YZ) of STL parts in their file frames. Usage: plot_parts.py OUT.png scale a.stl[:color] ..."""
import sys
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
out, scale = sys.argv[1], float(sys.argv[2])
parts = []
for a in sys.argv[3:]:
    p, _, c = a.partition(":")
    m = trimesh.load(p, force="mesh")
    m.apply_scale(scale)
    parts.append((p.split("/")[-1], m, c or None))
fig, axs = plt.subplots(1, 3, figsize=(18, 6))
for ax, (i, j, t) in zip(axs, ((0, 1, "top: X right, Y up"), (0, 2, "side: X right, Z up"), (1, 2, "front: Y right, Z up"))):
    for k, (name, m, c) in enumerate(parts):
        tri = m.vertices[m.faces][:, :, [i, j]]
        col = c or f"C{k}"
        ax.add_collection(PolyCollection(tri, facecolors=col, edgecolors="none", alpha=0.25))
    allv = np.vstack([m.vertices for _, m, _ in parts])
    ax.set_xlim(allv[:, i].min() - 5, allv[:, i].max() + 5); ax.set_ylim(allv[:, j].min() - 5, allv[:, j].max() + 5)
    ax.set_aspect("equal"); ax.grid(True, lw=0.3); ax.set_title(t); ax.plot(0, 0, "k+", ms=15)
axs[0].legend(handles=[plt.Line2D([], [], color=c or f"C{k}", lw=6, label=n) for k, (n, _, c) in enumerate(parts)], fontsize=7, loc="upper left")
fig.tight_layout(); fig.savefig(out, dpi=80)
print("saved", out)
