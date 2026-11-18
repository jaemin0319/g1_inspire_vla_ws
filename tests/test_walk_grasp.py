"""Unit tests for the automatic walk -> grasp source helpers (src/g1iv_teleop/sources.py); no simulator, no DDS."""
import sys
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src"))

from g1iv_common import loco  # noqa: E402
from g1iv_common.config import load_yaml  # noqa: E402
from g1iv_teleop import sources as S  # noqa: E402


def test_stick_inverse_matches_loco_arbiter():
    xr = load_yaml(WS / "configs/teleop.yaml")["loco"]["xr"]
    dz = xr["deadzone"]
    for v, vmax in [(0.5, xr["vx_max"]), (-0.6, xr["vx_max"]), (0.5, xr["vy_max"]), (-0.5, xr["vy_max"]),
                    (1.57, xr["wz_max"]), (-0.4, xr["wz_max"]), (0.0, xr["vx_max"])]:
        st = S._stick(v, vmax, dz)
        assert abs(st) <= 1.0
        assert abs(-loco.deadzone(st, dz) * vmax - v) < 1e-9


def test_plans_carry_their_desk_height():
    assert load_yaml(WS / "configs/scripted_grasp.yaml")["desk_top_z"] == 0.70
    for cm in (55, 65, 70):          # file name (run_mock_teleop.sh --desk) must match the plan's desk height
        p = load_yaml(WS / f"configs/scripted_walk_grasp_desk{cm:03d}.yaml")
        assert abs(p["desk_top_z"] - cm / 100) < 1e-9
        assert "walk" in p and len(p["walk"]["stand_rel"]) == 2


def test_desk_mismatch():
    plan = {"desk_top_z": 0.55, "object_half_height": 0.06}
    ok = {"eval_gt": {"object_pos_w": [1.0, -0.2, 0.612], "object_lift_m": 0.0}}
    lifted = {"eval_gt": {"object_pos_w": [1.0, -0.2, 0.712], "object_lift_m": 0.1}}
    old = {"eval_gt": {"object_pos_w": [1.0, -0.2, 0.762], "object_lift_m": 0.0}}
    assert S._desk_mismatch(plan, ok) is None and S._desk_mismatch(plan, lifted) is None
    assert "mismatch" in S._desk_mismatch(plan, old)
    assert S._desk_mismatch({}, old) is None and S._desk_mismatch(plan, {}) is None


def test_body_error_and_pulses():
    W = load_yaml(WS / "configs/scripted_walk_grasp_desk065.yaml")["walk"]
    sx, sy = W["stand_rel"]
    st = {"base_pos_w": [0.5, 0.1, 0.79], "base_yaw": 0.0, "eval_gt": {"object_pos_w": [0.5 + sx, 0.1 + sy, 0.6]}}
    assert np.allclose(S.ScriptedWalkGraspSource._body_err(st, W), (0.0, 0.0))
    st["base_pos_w"] = [0.4, 0.1, 0.79]            # 10 cm short of the stand pose
    s, e = S.ScriptedWalkGraspSource._body_err(st, W)
    assert np.isclose(s, -0.1) and np.isclose(e, 0.0)
    p = S.ScriptedWalkGraspSource._plan_pulse(s, e, 0.0, 0.04, 0.04, np.radians(10), W)
    assert p["axis"] == "along" and p["cmd"][0] > 0
    p = S.ScriptedWalkGraspSource._plan_pulse(0.0, 0.08, 0.0, 0.04, 0.04, np.radians(10), W)   # robot too far left
    assert p["axis"] == "cross" and p["cmd"][1] < 0
    p = S.ScriptedWalkGraspSource._plan_pulse(0.0, 0.0, np.radians(-20), 0.04, 0.04, np.radians(10), W)
    assert p["axis"] == "yaw" and p["cmd"][2] > 0
    p = S.ScriptedWalkGraspSource._plan_pulse(0.08, 0.0, 0.0, 0.04, 0.04, np.radians(10), W)   # overshoot
    assert p["axis"] == "along" and p["cmd"][0] < 0


def test_target_mismatch_guard():
    plan = {"expect_target": "blue"}
    assert S._target_mismatch(plan, {}) is None                                   # nothing to check yet
    assert S._target_mismatch(plan, {"eval_gt": {"target": "blue"}}) is None
    assert "target mismatch" in S._target_mismatch(plan, {"eval_gt": {"target": "red"}})
    assert "target mismatch" in S._target_mismatch(plan, {"eval_gt": {"fallen": False}})   # sim without targets
    assert S._target_mismatch({}, {"eval_gt": {"target": "red"}}) is None          # single-object plans unchanged


def test_other_cylinder_pushed_is_failure():
    src = S.ScriptedGraspSource(load_yaml(WS / "configs/teleop.yaml")["xr"],
                                load_yaml(WS / "configs/scripted_walk_grasp_desk065.yaml"))
    objs0 = {"red": [1.02, -0.20, 0.71], "blue": [1.02, 0.05, 0.71], "green": [1.02, -0.45, 0.71]}
    src._note_objects_start({"eval_gt": {"objects": objs0}})
    lifted = {"target": "blue", "object_lift_m": 0.06, "fallen": False}
    ok = src._finish_result({"eval_gt": {**lifted, "objects": {**objs0, "blue": [1.02, 0.05, 0.77]}}})
    assert ok["success"] and ok["target"] == "blue" and not ok["others_disturbed"]
    assert set(ok["others_moved_m"]) == {"red", "green"}
    pushed = src._finish_result({"eval_gt": {**lifted, "objects": {**objs0, "red": [1.06, -0.20, 0.71]}}})
    assert not pushed["success"] and pushed["others_disturbed"] and pushed["others_moved_m"]["red"] > 0.03
    single = S.ScriptedGraspSource(load_yaml(WS / "configs/teleop.yaml")["xr"],
                                   load_yaml(WS / "configs/scripted_walk_grasp_desk065.yaml"))
    r = single._finish_result({"eval_gt": {"object_lift_m": 0.06, "fallen": False}})   # one cylinder: as before
    assert r["success"] and r["others_moved_m"] is None
