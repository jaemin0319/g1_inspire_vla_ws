"""Consistent copy of a LeRobot v2.1 dataset that src/g1iv_data/convert_lerobot.py is still writing, finished the way
the converter finishes (meta/modality.json, camera_params in info.json, meta/stats.json, meta/g1iv_provenance.json),
so the episodes converted so far can be checked / uploaded before the conversion ends.

    .conda/envs/g1_inspire_data/bin/python tools/snapshot_partial_dataset.py \
        data/processed/<name>_lerobot_v21 data/processed/<name>_partial_lerobot_v21 <raw episode dir(s) given to the converter>

lerobot writes an episode's parquet, then its videos, then meta (meta.save_episode after encoding), so meta/ is copied
first: its total_episodes N is the number of complete episodes; files of later episodes copied afterwards are removed.
The raw episodes are matched in the converter's order (find_episodes, validated, outcome success) and checked
against the episode lengths in meta/episodes.jsonl.
"""
from __future__ import annotations

import json
import re
import shutil
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src"))

from g1iv_data import convert_lerobot as C  # noqa: E402
from g1iv_data.validate import find_episodes, validate_episode  # noqa: E402


def main() -> int:
    src, dst, raws = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3:]
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src / "meta", dst / "meta")                       # 1) meta first: N complete episodes
    info = json.loads((dst / "meta/info.json").read_text())
    n = int(info["total_episodes"])
    for sub in ("data", "videos"):                                    # 2) then files; drop episodes >= N
        shutil.copytree(src / sub, dst / sub)
        for f in (dst / sub).rglob("episode_*"):
            if int(re.search(r"episode_(\d+)", f.name).group(1)) >= n:
                f.unlink()
    eps_meta = [json.loads(x) for x in (dst / "meta/episodes.jsonl").read_text().splitlines() if x.strip()]
    n_stats = sum(1 for x in (dst / "meta/episodes_stats.jsonl").read_text().splitlines() if x.strip())
    vids = sorted((dst / "videos").rglob("*.mp4"))
    cams = sorted({p.parent.name for p in vids})
    pq = sorted((dst / "data").rglob("*.parquet"))
    assert len(eps_meta) == n == n_stats == len(pq), (n, len(eps_meta), n_stats, len(pq))
    assert len(vids) == n * len(cams), (len(vids), n, cams)
    # raw episodes in the converter's order (successes only, as converted with --include-outcomes success)
    ok = [d for d in find_episodes(raws) if (r := validate_episode(d)).get("ok") and r.get("outcome") == "success"][:n]
    prov = []
    for k, d in enumerate(ok):
        m = json.loads((d / "meta.json").read_text())
        frames = sum(1 for x in (d / "steps.jsonl").read_text().splitlines() if x.strip())
        assert frames == eps_meta[k]["length"], (k, d, frames, eps_meta[k]["length"])
        prov.append({"episode_index": k, "raw_dir": str(d.relative_to(WS)) if d.is_absolute() else str(d),
                     "episode_id": m["episode_id"], "source": m["source"], "is_smoke_data": m.get("is_smoke_data"),
                     "outcome": m.get("outcome"), "frames": frames, "instruction": m.get("instruction"),
                     "target_object": m.get("target_object"), "objects_xy": m.get("objects_xy")})
    aliases = [a for a, key in C.CAM_KEYS.items() if f"observation.images.{key}" in cams]
    (dst / "meta/modality.json").write_text(json.dumps(C.modality(aliases), indent=1))
    cp = C.camera_params(json.loads((ok[0] / "meta.json").read_text()).get("static", {}))
    if cp is not None:
        info["camera_params"] = cp
        (dst / "meta/info.json").write_text(json.dumps(info, indent=4))
    C.write_stats(dst)
    (dst / "meta/g1iv_provenance.json").write_text(json.dumps(
        {"converter": "src/g1iv_data/convert_lerobot.py (+ tools/snapshot_partial_dataset.py: first episodes of a "
                      "running conversion)", "codebase_version": info["codebase_version"], "fps": info["fps"],
         "episodes": prov, "action_definition": "targets applied by the simulator during the step following the observation",
         "state_order": C.MOTORS}, indent=1))
    per = {}
    for p in prov:
        per[p["target_object"]] = per.get(p["target_object"], 0) + 1
    print(f"snapshot -> {dst}  episodes={n} frames={info['total_frames']} cameras={len(cams)} per-target={per}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
