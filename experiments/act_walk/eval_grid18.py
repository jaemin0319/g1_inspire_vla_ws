#!/usr/bin/env python3
"""Per-checkpoint evaluation video (2026-11-23 user request): the 18 evaluation episodes of one checkpoint as two
3 x 3 head-camera (cam_left_high) grids played one after the other (episodes 1-9, then 10-18), every tile playing at the
same time; green circle = success / red X = failure in each tile's top-right; header with this checkpoint's success
rate and the rates of all checkpoints evaluated so far. Episodes that end early hold their last frame until the
longest of their group ends.

usage: eval_grid18.py <eval folder (logs/policy_eval/*_v3_<step>_nas25_far)> <recording root> <out.mp4>
       (teleop env python: PIL). The header's history line reads every complete *_v3_*_nas25_far folder next to it."""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw
import os as _os
VER = _os.environ.get("G1IV_EVAL_VER", "v3")   # model version label in the eval folder names

ev, rec, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
N, PER = 18, 9
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
TW, TH, FPS, HDR = 480, 360, 25, 96
POS = [(c, r) for r in range(3) for c in range(3)]


def rows_of(d: Path):
    s = d / "summary.jsonl"
    return [json.loads(x) for x in s.read_text().splitlines() if x.strip()] if s.is_file() else []


def step_of(d: Path) -> int:
    return int(d.name.split(f"_{VER}_")[1].split("_")[0])


rows = sorted(rows_of(ev), key=lambda r: r["i"])[:N]
assert len(rows) == N, (ev, len(rows))
step = step_of(ev)
ok = sum(r["outcome"] == "success" for r in rows)
hist = []
for d in sorted(ev.parent.glob(f"*_{VER}_*_nas25_far"), key=step_of):
    R = rows_of(d)
    if len(R) >= N and "_incomplete_" not in d.name:
        hist.append(f"{step_of(d) // 1000}k {sum(r['outcome'] == 'success' for r in R[:N])}/{N}")
tmp = Path(tempfile.mkdtemp(prefix="eval_grid18_"))

M = 64
ok_png, ng_png = tmp / "ok.png", tmp / "ng.png"
for path, kind in ((ok_png, "ok"), (ng_png, "ng")):
    im = Image.new("RGBA", (M * 4, M * 4), (0, 0, 0, 0))
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

parts = []
for g in range(N // PER):
    grp = rows[g * PER:(g + 1) * PER]
    inputs, chains, frames = [], [], []
    for k, r in enumerate(grp):
        hd = next(rec.glob(f"*/{r['episode_id']}/images/head"))
        frames.append(len(list(hd.glob("*.jpg"))))
        inputs += ["-framerate", str(FPS), "-start_number", "0", "-i", str(hd / "%06d.jpg")]
        lab = tmp / f"lab{g}_{k}.txt"
        lab.write_text(f"{r['i'] + 1}   원통 y {r['object_xy'][1]:+.2f}   출발 y {r['robot_pose'][1]:+.2f} m")
        chains.append(f"[{k}:v]scale={TW}:{TH},tpad=stop_mode=clone:stop=-1,"
                      f"drawbox=x=0:y=0:w=iw:h=34:color=black@0.5:t=fill,"
                      f"drawtext=fontfile={FONT}:textfile={lab}:x=10:y=5:fontsize=20:fontcolor=white:expansion=none[b{k}]")
        chains.append(f"[b{k}][{PER + k}:v]overlay=x=W-w-8:y=8[t{k}]")
    for r in grp:
        inputs += ["-loop", "1", "-i", str(ok_png if r["outcome"] == "success" else ng_png)]
    h1, h2 = tmp / f"h1_{g}.txt", tmp / f"h2_{g}.txt"
    h1.write_text(f"ACT {VER}  {step // 1000}k 스텝   성공 {ok}/{N} ({100 * ok / N:.0f}%)   "
                  f"{g + 1}회차: 시연 {g * PER + 1}–{(g + 1) * PER}  (이 회차 {sum(r['outcome'] == 'success' for r in grp)}/{PER})")
    h2.write_text("체크포인트별 성공률:  " + "   ".join(hist))
    layout = "|".join(f"{c * TW}_{r * TH}" for c, r in POS)
    chains.append("".join(f"[t{k}]" for k in range(PER)) + f"xstack=inputs={PER}:layout={layout}:fill=black,"
                  f"pad=iw:ih+{HDR}:0:{HDR}:color=0x111111,"
                  f"drawtext=fontfile={FONT}:textfile={h1}:x=20:y=12:fontsize=32:fontcolor=white:expansion=none,"
                  f"drawtext=fontfile={FONT}:textfile={h2}:x=20:y=58:fontsize=24:fontcolor=0xBBBBBB:expansion=none[v]")
    p = tmp / f"part{g}.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *inputs, "-filter_complex", ";".join(chains), "-map", "[v]",
                    "-t", f"{max(frames) / FPS:.2f}", "-r", str(FPS), "-c:v", "libx264", "-crf", "20", "-preset", "medium",
                    "-pix_fmt", "yuv420p", str(p)], check=True)
    parts.append(p)
lst = tmp / "list.txt"
lst.write_text("".join(f"file '{p}'\n" for p in parts))
out.parent.mkdir(parents=True, exist_ok=True)
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy",
                "-movflags", "+faststart", str(out)], check=True)
print(f"{step}: {ok}/{N} -> {out}")
