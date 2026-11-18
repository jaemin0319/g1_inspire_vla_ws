"""Contact sheet of one raw episode: N evenly spaced frames x 3 cameras, with step / sim time / hand
commands / walking command / ground-truth lift printed under each row. Writes <episode>/preview.jpg.

  python -m g1iv_data.preview [episode_dir]   (default: newest episode under data/raw and data/smoke)
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw


def newest(ws: Path) -> Path:
    eps = sorted((p.parent for root in ("data/raw", "data/smoke") for p in (ws / root).rglob("meta.json")),
                 key=lambda p: p.stat().st_mtime)
    if not eps:
        sys.exit("no episodes")
    return eps[-1]


def main():
    ws = Path(__file__).resolve().parents[2]
    d = Path(sys.argv[1]) if len(sys.argv) > 1 else newest(ws)
    meta = json.loads((d / "meta.json").read_text())
    rows = [json.loads(x) for x in (d / "steps.jsonl").read_text().splitlines() if x.strip()]
    n = min(6, len(rows))
    pick = [rows[round(i * (len(rows) - 1) / max(1, n - 1))] for i in range(n)]
    cams = ("head", "left_wrist", "right_wrist")
    tw, th = 320, 240
    sheet = Image.new("RGB", (tw * len(cams), (th + 34) * n + 40), (20, 20, 20))
    dr = ImageDraw.Draw(sheet)
    dr.text((8, 8), f"{meta['episode_id']}  src={meta['source']}  outcome={meta.get('outcome')}  '{meta.get('instruction')}'",
            fill=(230, 230, 230))
    for r_i, row in enumerate(pick):
        y = 40 + r_i * (th + 34)
        for c_i, c in enumerate(cams):
            p = d / row["obs"]["image_files"].get(c, "")
            if p.is_file():
                with Image.open(p) as im:
                    sheet.paste(im.convert("RGB").resize((tw, th)), (c_i * tw, y))
        tel = row.get("teleop", {})
        ev = row.get("eval", {})
        lift = ev.get("object_lift_m")
        txt = (f"frame {row['frame_index']} step {row['step']} sim_t {row['sim_time']:.2f}s  hands {tel.get('hand_cmd', '-')}"
               f"  loco {[round(v, 2) for v in row['action']['loco_cmd']]}  lift {lift if lift is None else round(lift, 3)}")
        dr.text((8, y + th + 8), txt, fill=(200, 200, 120))
    out = d / "preview.jpg"
    sheet.save(out, quality=90)
    print(out)


if __name__ == "__main__":
    main()
