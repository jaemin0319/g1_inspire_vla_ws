"""Recorder + frame pairing tests (numpy + Pillow; no Isaac imports)."""
import importlib.util
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[1]
# load recorder.py directly so that src/g1iv_sim/__init__ (Isaac imports) is not executed
spec = importlib.util.spec_from_file_location("g1iv_recorder", WS / "src/g1iv_sim/recorder.py")
rec_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rec_mod)


def test_pairer_every_2():
    p = rec_mod.FramePairer(2)
    t = p.start(step_now=10)          # obs captured now (after step 10) is for action step 11
    assert t == 11
    log = []
    for s in range(11, 20):
        emit, cap = p.on_step(s)
        log.append((s, emit, cap))
    emits = [s for s, e, _ in log if e]
    caps = [s for s, _, c in log if c]
    assert emits == [11, 13, 15, 17, 19]            # uniform 2-step grid starting at 11
    assert caps == [12, 14, 16, 18]                 # obs taken right before each emitted step
    assert all(c + 1 in emits for c in caps)


def test_pairer_every_1():
    p = rec_mod.FramePairer(1)
    p.start(0)
    out = [p.on_step(s) for s in range(1, 5)]
    assert all(e for e, _ in out) and all(c for _, c in out)


def test_recorder_roundtrip_and_smoke_routing():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        r = rec_mod.EpisodeRecorder(td / "raw", td / "smoke", ("mock", "scripted"), 90, 8)
        d = r.start({"source": "mock", "instruction": "pick up the red cylinder"})
        assert d.parts[-3] == "smoke"
        img = (np.random.default_rng(0).random((48, 64, 3)) * 255).astype(np.uint8)
        for i in range(5):
            row = {"step": 11 + 2 * i, "sim_time": 0.02 * (10 + 2 * i), "obs": {"arm_q": [0.1 * i] * 14},
                   "action": {"arm_q_target": [0.2 * i] * 14, "loco_cmd": [0.0, 0.0, 0.0, 0.8]}}
            assert r.add_frame(row, {"head": img, "left_wrist": img, "right_wrist": img})
        out = r.stop("success", "operator_stop")
        assert out["n_frames"] == 5 and out["outcome"] == "success"
        meta = json.loads((d / "meta.json").read_text())
        assert meta["is_smoke_data"] is True and meta["n_frames"] == 5
        rows = [json.loads(x) for x in (d / "steps.jsonl").read_text().splitlines()]
        assert [x["frame_index"] for x in rows] == list(range(5))
        arr = np.load(d / "arrays.npz")
        assert arr["obs/arm_q"].shape == (5, 14) and arr["action/loco_cmd"].shape == (5, 4)
        assert len(list((d / "images/head").glob("*.jpg"))) == 5
        # a second episode never reuses the directory
        d2 = r.start({"source": "pico"})
        assert d2 != d and d2.parts[-3] == "raw"
        r.stop("aborted", "test")


if __name__ == "__main__":
    import inspect

    fails = 0
    for name, fn in sorted(inspect.getmembers(sys.modules[__name__], inspect.isfunction)):
        if name.startswith("test_"):
            try:
                fn()
                print(f"PASS {name}")
            except Exception as e:  # noqa: BLE001
                fails += 1
                print(f"FAIL {name}: {type(e).__name__}: {e}")
    sys.exit(1 if fails else 0)
