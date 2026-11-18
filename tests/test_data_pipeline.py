"""Validator + state packing on synthetic episodes written by the real recorder (numpy + Pillow)."""
import importlib.util
import sys
import tempfile
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src"))
spec = importlib.util.spec_from_file_location("g1iv_recorder", WS / "src/g1iv_sim/recorder.py")
rec_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rec_mod)
from g1iv_data.convert_lerobot import MOTORS, pack26  # noqa: E402
from g1iv_data.validate import validate_episode  # noqa: E402


def _episode(root: Path, n=6, dup_cam_at=None, bad_step_at=None, hz=25.0):
    r = rec_mod.EpisodeRecorder(root / "raw", root / "smoke")
    d = r.start({"source": "scripted", "instruction": "pick up the red cylinder", "record_hz_sim": hz})
    img = np.full((48, 64, 3), 128, np.uint8)
    for i in range(n):
        step = 11 + 2 * i + (1 if bad_step_at is not None and i >= bad_step_at else 0)
        cam_fr = 100 + 2 * i - (2 if dup_cam_at is not None and i == dup_cam_at else 0)
        cm = {a: {"frame": cam_fr, "ts": step * 0.02} for a in ("head", "left_wrist", "right_wrist")}
        row = {"step": step, "sim_time": (step - 1) * 0.02, "wall_time": 1000.0 + 0.05 * i,
               "obs": {"arm_q": list(np.arange(14) * 0.1), "arm_dq": [0.0] * 14, "hand_q_rad": [0.0] * 12,
                       "hand_q_norm": list(np.arange(12) / 11.0), "base_lin_vel_b": [0, 0, 0], "base_ang_vel_b": [0, 0, 0],
                       "base_quat_w": [1, 0, 0, 0], "cam_meta": cm},
               "action": {"arm_q_target": [0.0] * 14, "hand_q_norm_target": [1.0] * 12, "loco_cmd": [0, 0, 0, 0.8]},
               "teleop": {"present": True, "age_ms": 10.0, "stale": False, "input_ok": True, "hand_cmd": {"left": "OPEN", "right": "CLOSE"}},
               "eval": {"fallen": False}}
        r.add_frame(row, {"head": img, "left_wrist": img, "right_wrist": img})
    r.stop("success", "test")
    return d


def test_pack26_order():
    arm = np.arange(14, dtype=float)
    hand_dfx = np.arange(100, 112, dtype=float)    # R 100..105, L 106..111
    v = pack26(arm, hand_dfx)
    assert v.shape == (26,) and len(MOTORS) == 26
    assert np.allclose(v[:14], arm)
    assert np.allclose(v[14:20], hand_dfx[6:12])   # left hand first (unitree G1_Inspire order)
    assert np.allclose(v[20:26], hand_dfx[0:6])
    assert MOTORS[14] == "kLeftHandPinky" and MOTORS[20] == "kRightHandPinky"


def test_validator_clean_episode():
    with tempfile.TemporaryDirectory() as td:
        d = _episode(Path(td))
        r = validate_episode(d)
        assert r["ok"], r["errors"]
        assert abs(r["rate_sim_hz"] - 25.0) < 1e-6 and r["is_smoke_data"] is True


def test_validator_detects_stale_camera_and_irregular_steps():
    with tempfile.TemporaryDirectory() as td:
        r = validate_episode(_episode(Path(td), dup_cam_at=3))
        assert not r["ok"] and any("duplicated" in e for e in r["errors"])
    with tempfile.TemporaryDirectory() as td:
        r = validate_episode(_episode(Path(td), bad_step_at=3))
        assert not r["ok"] and any("step spacing" in e for e in r["errors"])


if __name__ == "__main__":
    import inspect
    fails = 0
    for n, fn in sorted(inspect.getmembers(sys.modules[__name__], inspect.isfunction)):
        if n.startswith("test_"):
            try:
                fn(); print(f"PASS {n}")
            except Exception as e:  # noqa: BLE001
                fails += 1; print(f"FAIL {n}: {type(e).__name__}: {e}")
    sys.exit(1 if fails else 0)
