"""Inverse of the official controller-mode conversion (numpy only). The forward function here is a
re-statement; tests/test_official_televuer.py checks it against the official televuer code."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from g1iv_common.arm_math import rotz  # noqa: E402
from g1iv_teleop import xr_frames as xf  # noqa: E402


def _rand_pose(rng):
    T = np.eye(4)
    T[:3, :3] = rotz(rng.uniform(-1, 1)) @ np.array([[1, 0, 0], [0, np.cos(0.3), -np.sin(0.3)], [0, np.sin(0.3), np.cos(0.3)]])
    T[:3, 3] = rng.uniform(-0.5, 0.5, 3)
    return T


def test_inverse_roundtrip_identity_head():
    rng = np.random.default_rng(1)
    for _ in range(20):
        P = _rand_pose(rng)
        raw = xf.inverse_controller(xf.CONST_HEAD_POSE, P)
        assert np.allclose(xf.forward_controller(xf.CONST_HEAD_POSE, raw), P, atol=1e-9)


def test_inverse_roundtrip_yawed_head():
    rng = np.random.default_rng(2)
    head = xf.CONST_HEAD_POSE.copy()
    c, s = np.cos(0.7), np.sin(0.7)          # yaw about OpenXR +Y (up)
    head[:3, :3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    for _ in range(20):
        P = _rand_pose(rng)
        assert np.allclose(xf.forward_controller(head, xf.inverse_controller(head, P)), P, atol=1e-9)


def test_neutral_pose_maps_near_waist_front():
    P = xf.forward_controller(xf.CONST_HEAD_POSE, xf.CONST_RIGHT_ARM_POSE)
    # official neutral right controller: 0.25 m forward of the waist origin, 0.15 m to the right, 0.08 m above
    assert np.allclose(P[:3, 3], [0.25, -0.15, 0.08], atol=1e-9)
    assert np.allclose(xf.delta_robot_to_xr([1, 0, 0]), [0, 0, -1])   # robot forward = OpenXR -Z


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
