"""Unit tests for src/g1iv_common (numpy + PyYAML only; no simulator, no DDS)."""
import sys
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src"))

from g1iv_common import arm_math, hands, loco, topics  # noqa: E402
from g1iv_common.config import apply_overrides, load_yaml  # noqa: E402


def test_configs_load_and_desk_interpretation():
    scene = load_yaml("configs/scene_desk.yaml")
    d = scene["desk"]
    assert (d["width"], d["depth"], d["top_surface_height"]) == (1.5, 0.7, 0.55)   # top lowered 0.70 -> 0.55 (2026-11-17)
    tele = load_yaml("configs/teleop.yaml")
    assert tele["loco_source"] in ("keyboard", "xr", "none")
    cfg = apply_overrides(scene, ["desk.width=1.2", "robot.init_pos=[0.5,0,0.8]"])
    assert cfg["desk"]["width"] == 1.2 and cfg["robot"]["init_pos"] == [0.5, 0, 0.8]


def test_hand_presets_and_ranges():
    hp = load_yaml("configs/hands_inspire.yaml")
    q_open = hands.preset_vector(hp["presets"]["open"])
    q_close = hands.preset_vector(hp["presets"]["close"])
    r_open, r_close = hands.norm_to_rad(q_open), hands.norm_to_rad(q_close)
    # digits flex more when closed; thumb rotation identical (no sweep through the object)
    assert np.all(r_close[:4] > r_open[:4]) and r_close[4] > r_open[4]
    assert abs(r_close[5] - r_open[5]) < 1e-9
    # limits from the sim: fingers [0,1.7], thumb bend [0,0.5], thumb rot [-0.1,1.3]
    assert np.allclose(hands.norm_to_rad(np.ones(6)), [0, 0, 0, 0, 0, -0.1])
    assert np.allclose(hands.norm_to_rad(np.zeros(6)), [1.7, 1.7, 1.7, 1.7, 0.5, 1.3])


def test_trigger_hysteresis():
    b = hands.TriggerBinarizer(0.6, 0.4)
    seq = [0.0, 0.5, 0.59, 0.61, 0.5, 0.45, 0.41, 0.39, 0.55]
    out = [b.update(x).name for x in seq]
    assert out == ["OPEN", "OPEN", "OPEN", "CLOSE", "CLOSE", "CLOSE", "CLOSE", "OPEN", "OPEN"]


def test_dual_hand_rate_limit_and_input_lost():
    hp = load_yaml("configs/hands_inspire.yaml")
    dh = hands.DualHand(hp, {"max_rate_per_s": 2.0})
    dh.update_from_triggers(1.0, 0.0, input_ok=True)  # left CLOSE, right OPEN
    assert dh.cmds() == {"left": "CLOSE", "right": "OPEN"}
    q = dh.step(0.02)
    # one 20 ms step moves at most 0.04 normalised units
    assert np.max(np.abs(q[hands.LEFT_SLICE] - dh.left.q_open)) <= 0.04 + 1e-9
    for _ in range(50):
        q = dh.step(0.02)
    assert np.allclose(q[hands.LEFT_SLICE], dh.left.q_close)
    assert np.allclose(q[hands.RIGHT_SLICE], dh.right.q_open)
    # link lost with trigger reading 0 must NOT open the closed hand
    dh.update_from_triggers(0.0, 0.0, input_ok=False)
    assert dh.cmds()["left"] == "CLOSE"
    # explicit release opens
    dh.update_from_triggers(0.0, 0.0, input_ok=True)
    assert dh.cmds()["left"] == "OPEN"
    # both hands at once
    dh.set_cmds(hands.HandCmd.CLOSE, hands.HandCmd.CLOSE)
    assert dh.cmds() == {"left": "CLOSE", "right": "CLOSE"}
    # a resting (released) trigger does not undo a keyboard CLOSE; the next trigger transition does
    dh.update_from_triggers(0.0, 0.0, input_ok=True)
    assert dh.cmds() == {"left": "CLOSE", "right": "CLOSE"}
    dh.update_from_triggers(0.9, None, input_ok=True)   # press -> CLOSE (already)
    dh.update_from_triggers(0.1, None, input_ok=True)   # release -> OPEN
    assert dh.cmds()["left"] == "OPEN"
    dh.toggle("right")
    assert dh.cmds()["right"] == "OPEN"


def test_trigger_transition_needs_live_pose():
    """A link that freezes (no new controller pose) while reporting trigger=0 must not open the hand,
    even before the stale timeout marks the input lost; a live release opens after the confirm delay."""
    hp = load_yaml("configs/hands_inspire.yaml")
    dh = hands.DualHand(hp, {"max_rate_per_s": 2.0, "trigger_confirm_s": 0.05})
    t = 10.0
    dh.update_from_triggers(None, 1.0, True, now=t, last_pose_change=t)
    dh.update_from_triggers(None, 1.0, True, now=t + 0.06, last_pose_change=t + 0.06)
    assert dh.cmds()["right"] == "CLOSE"
    # freeze at t+0.1: pose stops updating, trigger reads 0 for 0.4 s (still inside a 0.5 s stale timeout)
    tf = t + 0.1
    for k in range(20):
        dh.update_from_triggers(None, 0.0, True, now=tf + 0.02 * k, last_pose_change=tf - 0.001)
    assert dh.cmds()["right"] == "CLOSE"
    # live release: trigger 0 with poses still arriving -> OPEN after >= 50 ms
    tl = tf + 1.0
    dh.update_from_triggers(None, 0.0, True, now=tl, last_pose_change=tl)
    assert dh.cmds()["right"] == "CLOSE"                      # not yet confirmed
    dh.update_from_triggers(None, 0.0, True, now=tl + 0.06, last_pose_change=tl + 0.06)
    assert dh.cmds()["right"] == "OPEN"
    # a short blip shorter than the confirm window does nothing
    dh.update_from_triggers(None, 1.0, True, now=tl + 0.10, last_pose_change=tl + 0.10)
    dh.update_from_triggers(None, 0.0, True, now=tl + 0.12, last_pose_change=tl + 0.12)
    dh.update_from_triggers(None, 0.0, True, now=tl + 0.20, last_pose_change=tl + 0.20)
    assert dh.cmds()["right"] == "OPEN"


def _loco_cfg():
    return load_yaml("configs/teleop.yaml")["loco"]


def test_loco_keyboard_ramp_release_stop_focus():
    a = loco.LocoArbiter(_loco_cfg(), source="keyboard")
    t = 0.0
    a.feed_keys(loco.KeyState(forward=True), t)
    for i in range(100):
        t += 0.02
        a.feed_keys(loco.KeyState(forward=True), t)
        cmd = a.step(0.02, t)
    assert abs(cmd[0] - 0.5) < 1e-6 and cmd[1] == 0 and cmd[2] == 0 and cmd[3] == 0.8
    # release -> decays to zero (decel 4 m/s^2 => 0.5 m/s in 0.125 s)
    for i in range(10):
        t += 0.02
        a.feed_keys(loco.KeyState(), t)
        cmd = a.step(0.02, t)
    assert cmd[0] == 0.0
    # explicit stop is immediate
    a.feed_keys(loco.KeyState(forward=True), t)
    for _ in range(20):
        t += 0.02
        a.feed_keys(loco.KeyState(forward=True), t)
        a.step(0.02, t)
    a.feed_keys(loco.KeyState(forward=True, stop=True), t)
    assert a.step(0.02, t)[0] == 0.0
    # focus loss is immediate zero
    a.feed_keys(loco.KeyState(forward=True, focused=False), t)
    assert a.step(0.02, t)[0] == 0.0 and a.reason == "focus_lost"
    # console silent for > 0.5 s => zero
    a.feed_keys(loco.KeyState(forward=True), t)
    a.step(0.02, t)
    assert a.step(0.02, t + 0.6)[0] == 0.0 and a.reason == "stale"


def test_loco_xr_deadzone_limits_timeout():
    cfg = _loco_cfg()
    a = loco.LocoArbiter(cfg, source="xr")
    a.feed_xr([0.05, -0.1], [0.1, 0.0], valid=True, t=0.0)
    assert a.step(0.02, 0.01)[:3] == [0.0, 0.0, 0.0]          # inside dead zone
    a.feed_xr([0.0, -1.0], [-1.0, 0.0], valid=True, t=1.0)       # full forward + turn left
    cmd = a.step(0.02, 1.01)
    vx_lim, wz_lim = cfg["limits"]["vx"][1], cfg["limits"]["wz"][1]
    assert abs(cmd[0] - min(cfg["xr"]["vx_max"], vx_lim)) < 1e-9 and abs(cmd[2] - min(cfg["xr"]["wz_max"], wz_lim)) < 1e-9
    assert a.step(0.02, 1.5)[:3] == [0.0, 0.0, 0.0] and a.reason == "xr_stale"
    # keyboard input is ignored while xr is the active source
    a.feed_keys(loco.KeyState(forward=True), 1.5)
    assert a.step(0.02, 1.51)[0] == 0.0


def test_relative_mapping_and_rate_limit():
    Tee = np.eye(4); Tee[:3, 3] = [0.3, -0.2, 0.1]
    Tc0 = np.eye(4); Tc0[:3, :3] = arm_math.rotz(0.3); Tc0[:3, 3] = [1.0, 2.0, 3.0]
    # no controller motion => target == current EE (no jump on engage)
    assert np.allclose(arm_math.rel_target(Tee, Tc0, Tc0), Tee)
    Tc = Tc0.copy(); Tc[:3, 3] += [0.05, 0.0, 0.02]
    T = arm_math.rel_target(Tee, Tc0, Tc)
    assert np.allclose(T[:3, 3], [0.35, -0.2, 0.12])
    Tc[:3, 3] += [1.0, 0, 0]
    T = arm_math.rel_target(Tee, Tc0, Tc, max_offset=0.45)
    assert abs(np.linalg.norm(T[:3, 3] - Tee[:3, 3]) - 0.45) < 1e-9
    q = arm_math.rate_limit(np.zeros(3), np.array([1.0, 0.1, -0.5]), max_vel=1.0, dt=0.02)
    assert abs(np.max(np.abs(q)) - 0.02) < 1e-12 and np.allclose(q / 0.02, [1.0, 0.1, -0.5])


def test_topics_payloads():
    assert topics.run_command_str(0.5, 0, -0.2, 0.8) == "[0.5, 0.0, -0.2, 0.8]"
    import json

    m = json.loads(topics.control_msg("record_stop", outcome="success"))
    assert m["cmd"] == "record_stop" and m["outcome"] == "success"
    try:
        topics.control_msg("record_stop", outcome="great")
        raise AssertionError("bad outcome accepted")
    except ValueError:
        pass


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
