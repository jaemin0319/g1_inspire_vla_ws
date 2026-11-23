#!/usr/bin/env python3
"""Overfitting check video (2026-11-23 user request): the 10 trials' HEAD camera (cam_left_high) side by side in one
mp4, all playing at the same time (3 x 3 = 1440x1080; user: "333, drop one" -> trial at cylinder y DROP_Y left out); a green circle (success) or red X (failure) in each tile's top-right.
Episodes that end early hold their last frame until the longest one ends.

usage: overfit_grid.py <logs/policy_eval root> <recording root> <out.mp4>   (teleop env python: PIL)"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw

logs, rec, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
TW, TH, FPS = 480, 360, 25
POS = [(c, r) for r in range(3) for c in range(3)]   # 3-3-3
DROP_Y = 0.35
rows = []
for d in sorted(logs.glob("*_overfit_t*_far")):
    s = d / "summary.jsonl"
    if s.is_file():
        rows += [json.loads(x) for x in s.read_text().splitlines() if x.strip()]
rows.sort(key=lambda r: r["object_xy"][1])
assert len(rows) == 10, len(rows)
rows = [r for r in rows if abs(r["object_xy"][1] - DROP_Y) > 1e-3]
N = len(rows)
assert N == 9, N
tmp = Path(tempfile.mkdtemp(prefix="overfit_grid_"))

M = 64   # mark size (px) inside a 480x360 tile
ok_png, ng_png = tmp / "ok.png", tmp / "ng.png"
for path, kind in ((ok_png, "ok"), (ng_png, "ng")):
    im = Image.new("RGBA", (M * 4, M * 4), (0, 0, 0, 0))   # drawn 4x, downsampled = smooth edges
    dr = ImageDraw.Draw(im)
    pad, w = 18, 30
    if kind == "ok":
        dr.ellipse([pad, pad, M * 4 - pad, M * 4 - pad], outline=(0, 0, 0, 150), width=w + 12)
        dr.ellipse([pad, pad, M * 4 - pad, M * 4 - pad], outline=(40, 200, 70, 255), width=w)
    else:
        for col, ww in (((0, 0, 0, 150), w + 12), ((235, 50, 50, 255), w)):
            dr.line([pad + 10, pad + 10, M * 4 - pad - 10, M * 4 - pad - 10], fill=col, width=ww)
            dr.line([pad + 10, M * 4 - pad - 10, M * 4 - pad - 10, pad + 10], fill=col, width=ww)
    im.resize((M, M), Image.LANCZOS).save(path)

inputs, chains, frames = [], [], []
for k, r in enumerate(rows):
    ep = r["episode_id"]
    hd = next(rec.glob(f"*/{ep}/images/head"))
    n = len(list(hd.glob("*.jpg")))
    frames.append(n)
    inputs += ["-framerate", str(FPS), "-start_number", "0", "-i", str(hd / "%06d.jpg")]
    oy = r["object_xy"][1]
    lab = tmp / f"lab{k}.txt"
    lab.write_text(f"{k + 1}   원통 y {oy:+.2f} m" + ("  (학습 범위)" if -0.30 <= oy <= -0.10 else ""))
    chains.append(
        f"[{k}:v]scale={TW}:{TH},tpad=stop_mode=clone:stop=-1,"
        f"drawbox=x=0:y=0:w=iw:h=34:color=black@0.5:t=fill,"
        f"drawtext=fontfile={FONT}:textfile={lab}:x=10:y=5:fontsize=20:fontcolor=white[b{k}]")
    mark = N + k
    chains.append(f"[b{k}][{mark}:v]overlay=x=W-w-8:y=8[t{k}]")
for r in rows:
    inputs += ["-loop", "1", "-i", str(ok_png if r["outcome"] == "success" else ng_png)]
layout = "|".join(f"{c * TW}_{r * TH}" for c, r in POS)
chains.append("".join(f"[t{k}]" for k in range(N)) + f"xstack=inputs={N}:layout={layout}:fill=black[v]")
dur = max(frames) / FPS
cmd = ["ffmpeg", "-y", "-loglevel", "error", *inputs, "-filter_complex", ";".join(chains), "-map", "[v]",
       "-t", f"{dur:.2f}", "-r", str(FPS), "-c:v", "libx264", "-crf", "20", "-preset", "medium", "-pix_fmt", "yuv420p",
       "-movflags", "+faststart", str(out)]
subprocess.run(cmd, check=True)
for k, r in enumerate(rows):
    print(k + 1, f"y {r['object_xy'][1]:+.2f}", r["outcome"], r["reason"], frames[k], "frames", r["episode_id"])
print("wrote", out, f"{dur:.1f} s")
