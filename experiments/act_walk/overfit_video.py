#!/usr/bin/env python3
"""Join the 10 overfitting-check observer videos into one mp4 with a caption per trial (2026-11-23).
usage: overfit_video.py <runner logs/policy_eval root> <observer video dir> <out.mp4>"""
import json, subprocess, sys, glob, os, tempfile
from pathlib import Path
logs, vdir, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
rows = []
for d in sorted(logs.glob("*_overfit_t*_far")):
    for f in d.glob("*.jsonl"):
        for line in f.read_text().splitlines():
            r = json.loads(line)
            if "outcome" in r and "object_xy" in r:
                rows.append((d.name, r))
rows.sort(key=lambda x: x[0])
print(len(rows), "trials")
tmp = Path(tempfile.mkdtemp(dir=os.environ.get("TMPDIR", "/tmp")))
parts = []
for k, (name, r) in enumerate(rows):
    ep = r.get("episode_id") or r.get("dir", "").rstrip("/").split("/")[-1]
    vid = vdir / f"{ep}_observer.mp4"
    assert vid.is_file(), vid
    oy = r["object_xy"][1]; rp = r["robot_pose"]
    inside = -0.30 <= oy <= -0.10
    res = "성공" if r["outcome"] == "success" else "실패"
    l1 = f"시행 {k + 1}/10   원통 y = {oy:+.2f} m ({'학습 범위 안' if inside else '학습 범위 밖'}, 학습 데이터는 -0.30 ~ -0.10 m)"
    l2 = f"출발 ({rp[0]:+.2f}, {rp[1]:+.2f} m, {rp[2]:+.0f}°)   결과: {res}"
    t1, t2 = tmp / f"l1_{k}.txt", tmp / f"l2_{k}.txt"
    t1.write_text(l1); t2.write_text(l2)
    col = "0x7CFC00" if r["outcome"] == "success" else "0xFF5555"
    vf = (f"drawbox=x=0:y=0:w=iw:h=96:color=black@0.55:t=fill,"
          f"drawtext=fontfile={FONT}:textfile={t1}:x=24:y=14:fontsize=30:fontcolor=white,"
          f"drawtext=fontfile={FONT}:textfile={t2}:x=24:y=54:fontsize=30:fontcolor={col}")
    p = tmp / f"p{k}.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(vid), "-vf", vf, "-c:v", "libx264", "-crf", "20",
                    "-preset", "medium", "-pix_fmt", "yuv420p", "-r", "25", str(p)], check=True)
    parts.append(p)
    print(k, ep, oy, r["outcome"], r.get("reason"))
lst = tmp / "list.txt"
lst.write_text("".join(f"file '{p}'\n" for p in parts))
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy",
                "-movflags", "+faststart", str(out)], check=True)
print("wrote", out)
