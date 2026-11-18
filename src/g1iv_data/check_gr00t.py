"""Load a converted dataset through Isaac-GR00T's own LeRobotEpisodeLoader (env g1_inspire_vla, loader-only).

    python -m g1iv_data.check_gr00t data/processed/<name>_lerobot_v21 --report logs/validate/gr00t_<name>.json

Checks: meta/modality.json is accepted, every state/action group (incl. base_command from action.loco_cmd)
has the expected width, the three AV1 videos decode with torchcodec, the language annotation resolves to
the instruction, and an action chunk of 16 steps can be indexed. No model weights are involved.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

STATE = ["left_arm", "right_arm", "left_hand", "right_hand"]
ACTION = STATE + ["base_command"]
WIDTH = {"left_arm": 7, "right_arm": 7, "left_hand": 6, "right_hand": 6, "base_command": 3}
VIDEO_REQUIRED = ["head", "left_wrist", "right_wrist"]   # + head_right when the stereo right eye was recorded
LANG = "annotation.human.task_description"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root")
    ap.add_argument("--report")
    ap.add_argument("--horizon", type=int, default=16)
    a = ap.parse_args()

    from gr00t.data.dataset.lerobot_episode_loader import LeRobotEpisodeLoader
    from gr00t.data.types import ModalityConfig

    import json as _json
    from pathlib import Path as _P

    VIDEO = list(_json.loads((_P(a.root) / "meta" / "modality.json").read_text())["video"])
    missing = [k for k in VIDEO_REQUIRED if k not in VIDEO]
    if missing:
        print(f"FAIL modality.json video lacks {missing}")
        return 1
    cfg = {
        "video": ModalityConfig(delta_indices=[0], modality_keys=VIDEO),
        "state": ModalityConfig(delta_indices=[0], modality_keys=STATE),
        "action": ModalityConfig(delta_indices=list(range(a.horizon)), modality_keys=ACTION),
        "language": ModalityConfig(delta_indices=[0], modality_keys=[LANG]),
    }
    rep: dict = {"root": a.root, "ok": False, "checks": []}

    def check(name, cond, detail=""):
        rep["checks"].append({"check": name, "ok": bool(cond), "detail": detail})
        print(("OK   " if cond else "FAIL ") + name + (f"  ({detail})" if detail else ""), flush=True)
        return cond

    t0 = time.time()
    loader = LeRobotEpisodeLoader(a.root, cfg)
    n_ep = len(loader)
    check("loader constructed from meta/ (modality.json accepted)", n_ep > 0, f"{n_ep} episodes")
    instr = {json.loads(l)["task_index"]: json.loads(l)["task"]
             for l in (Path(a.root) / "meta" / "tasks.jsonl").read_text().splitlines() if l.strip()}
    for ei in range(n_ep):
        df = loader[ei]
        cols = list(df.columns)
        if ei == 0:
            rep["columns"] = cols
        T = len(df)
        for g in STATE:
            v = np.stack(df[f"state.{g}"].to_numpy())
            check(f"ep{ei} state.{g} shape", v.shape == (T, WIDTH[g]), str(v.shape))
        for g in ACTION:
            v = np.stack(df[f"action.{g}"].to_numpy())
            check(f"ep{ei} action.{g} shape", v.shape == (T, WIDTH[g]), str(v.shape))
        for k in VIDEO:
            fr = df[f"video.{k}"].iloc[T // 2]
            check(f"ep{ei} video.{k} decoded", isinstance(fr, np.ndarray) and fr.ndim == 3 and fr.shape[2] == 3,
                  f"{getattr(fr, 'shape', None)} {getattr(fr, 'dtype', None)}")
        lang = df[f"language.{LANG}"].iloc[0]
        check(f"ep{ei} language", isinstance(lang, str) and lang in instr.values(), repr(lang))
        if T >= a.horizon:
            chunk = np.stack(df[f"action.right_arm"].to_numpy()[:a.horizon])
            check(f"ep{ei} action chunk [{a.horizon}] indexable", chunk.shape == (a.horizon, 7), str(chunk.shape))
    rep["seconds"] = round(time.time() - t0, 2)
    rep["ok"] = all(c["ok"] for c in rep["checks"])
    print(("GR00T loader check PASSED" if rep["ok"] else "GR00T loader check FAILED") + f" in {rep['seconds']} s")
    if a.report:
        Path(a.report).parent.mkdir(parents=True, exist_ok=True)
        Path(a.report).write_text(json.dumps(rep, indent=1, default=str))
    return 0 if rep["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
