"""Arm/hand/stick input sources. All XR-like sources (pico, mock, scripted) go through the OFFICIAL
televuer.TeleVuerWrapper.get_tele_data() conversion; only the backend that fills the raw OpenXR
values differs:
  pico     -> official televuer.TeleVuer (Vuer WebXR server, real headset)
  mock     -> FakeTeleVuer driven by a timed scenario (no headset; NOT a WebXR connection test)
  scripted -> FakeTeleVuer driven by a grasp plan that uses simulator ground truth (test tool only)
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from g1iv_common.arm_math import inv_T

from . import xr_frames as xf


@dataclass
class InputFrame:
    t: float
    ok: bool                    # fresh, valid controller data this cycle
    connected: bool             # any data ever received and not stale
    left_wrist: np.ndarray      # 4x4, IK (waist) frame, official convention
    right_wrist: np.ndarray
    head: np.ndarray
    trig: tuple[float, float]   # pressed fraction 0..1 (left, right)
    squeeze: tuple[bool, bool]
    a: tuple[bool, bool]        # left X / right A
    b: tuple[bool, bool]        # left Y / right B
    stick: tuple[np.ndarray, np.ndarray]
    stick_click: tuple[bool, bool]
    raw: dict = field(default_factory=dict)


class FakeTeleVuer:
    """Attribute-compatible stand-in for televuer.TeleVuer in controller mode (OpenXR basis)."""

    def __init__(self):
        self.head_pose = xf.CONST_HEAD_POSE.copy()
        self.left_arm_pose = xf.CONST_LEFT_ARM_POSE.copy()
        self.right_arm_pose = xf.CONST_RIGHT_ARM_POSE.copy()
        self.motion_data_ready = False
        for s in ("left", "right"):
            for k, v in (("trigger", False), ("triggerValue", 0.0), ("squeeze", False), ("squeezeValue", 0.0),
                         ("aButton", False), ("bButton", False), ("thumbstick", False)):
                setattr(self, f"{s}_ctrl_{k}", v)
            setattr(self, f"{s}_ctrl_thumbstickValue", np.zeros(2))

    def render_to_xr(self, img):
        pass

    def close(self):
        pass


def official_wrapper_for(backend, arm_reference_mode="head_yaw"):
    """Official TeleVuerWrapper bound to an arbitrary backend (no Vuer server started)."""
    from televuer.tv_wrapper import TeleVuerWrapper  # official xr_teleoperate submodule

    w = TeleVuerWrapper.__new__(TeleVuerWrapper)
    w.use_hand_tracking = False
    w.return_hand_rot_data = False
    w.arm_reference_mode = arm_reference_mode
    w.tvuer = backend
    return w


class _XRBase:
    name = "xr"

    def __init__(self, wrapper, stale_timeout_s: float):
        self.w = wrapper
        self.stale_timeout = float(stale_timeout_s)
        self._sig = None
        self._last_change = -1e9
        self._ever = False

    def _signature(self):
        tv = self.w.tvuer
        return (np.asarray(tv.left_arm_pose).tobytes() + np.asarray(tv.right_arm_pose).tobytes()
                + np.asarray(tv.head_pose).tobytes())

    def read(self, now: float | None = None) -> InputFrame:
        now = time.monotonic() if now is None else now
        td = self.w.get_tele_data()
        sig = self._signature()
        if sig != self._sig:
            self._sig = sig
            self._last_change = now
        fresh = bool(td.motion_data_ready) and (now - self._last_change) <= self.stale_timeout
        self._ever |= bool(td.motion_data_ready)
        trig = (float(np.clip((10.0 - td.left_ctrl_triggerValue) / 10.0, 0, 1)),
                float(np.clip((10.0 - td.right_ctrl_triggerValue) / 10.0, 0, 1)))
        return InputFrame(
            t=now, ok=fresh, connected=fresh, left_wrist=np.array(td.left_wrist_pose), right_wrist=np.array(td.right_wrist_pose),
            head=np.array(td.head_pose), trig=trig, squeeze=(bool(td.left_ctrl_squeeze), bool(td.right_ctrl_squeeze)),
            a=(bool(td.left_ctrl_aButton), bool(td.right_ctrl_aButton)), b=(bool(td.left_ctrl_bButton), bool(td.right_ctrl_bButton)),
            stick=(np.array(td.left_ctrl_thumbstickValue, float), np.array(td.right_ctrl_thumbstickValue, float)),
            stick_click=(bool(td.left_ctrl_thumbstick), bool(td.right_ctrl_thumbstick)),
            raw={"motion_data_ready": bool(td.motion_data_ready), "since_change_s": now - self._last_change,
                 "trigger_value_wrapper": [td.left_ctrl_triggerValue, td.right_ctrl_triggerValue],
                 "squeeze_value": [td.left_ctrl_squeezeValue, td.right_ctrl_squeezeValue]})

    def close(self):
        try:
            self.w.close()
        except Exception:  # noqa: BLE001
            pass


class PicoSource(_XRBase):
    """Real headset through the official televuer (WebXR over HTTPS/WSS, port 8012)."""

    name = "pico"

    def __init__(self, xr_cfg: dict, cam_cfg: dict, img_server_ip: str):
        from televuer import TeleVuerWrapper  # official

        head = cam_cfg["head_camera"]
        w = TeleVuerWrapper(use_hand_tracking=False, binocular=head["binocular"], img_shape=head["image_shape"],
                            display_mode=xr_cfg.get("display_mode", "immersive"), zmq=head["enable_zmq"],
                            webrtc=head["enable_webrtc"],
                            webrtc_url=f"https://{img_server_ip}:{head['webrtc_port']}/offer",
                            arm_reference_mode=xr_cfg.get("arm_reference_mode", "head_yaw"))
        super().__init__(w, xr_cfg.get("stale_timeout_s", 0.5))


# ---------------------------------------------------------------------------------- mock scenarios
def _bump(t, t0, t1):
    """0 -> 1 -> 0 smooth bump on [t0, t1]."""
    if t <= t0 or t >= t1:
        return 0.0
    x = (t - t0) / (t1 - t0)
    return float(np.sin(np.pi * x) ** 2)


def _ramp(t, t0, t1):
    if t <= t0:
        return 0.0
    if t >= t1:
        return 1.0
    x = (t - t0) / (t1 - t0)
    return float(0.5 - 0.5 * np.cos(np.pi * x))


SCENARIOS = {
    # name: (duration_s, description)
    "dry_run": (13.0, "engage; small wrist moves (3 cm) and return; L/R/both OPEN-CLOSE; disengage"),
    "dropout": (9.0, "engage; move; close; input frozen 2 s (trigger reads 0 while frozen); resume at a new pose"),
    "recenter": (8.0, "engage; move +8 cm; recenter; move back; check target continuity"),
    "walk_xr": (9.0, "left stick forward 2 s, release, right stick turn 2 s, release (loco_source=xr)"),
    "walk_seq": (58.0, "wall-time sequence long enough at RTF~0.35: forward 8 s, stop 4, left 8, stop 4, "
                       "back 6, stop 4, turn left 8, stop 4, turn right 4, stop (loco_source=xr)"),
    "walk_arm": (36.0, "arms + walking at once: engage; walk forward 10 s while the right wrist moves 6 cm and the "
                       "right hand closes/opens; stop; turn left 8 s while the left wrist moves; disengage (loco_source=xr)"),
    "hold": (1e9, "connected and still (stand test)"),
}


class MockSource(_XRBase):
    name = "mock"

    def __init__(self, scenario: str, xr_cfg: dict, seed: int = 0):
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown mock scenario {scenario}; choose from {sorted(SCENARIOS)}")
        self.fake = FakeTeleVuer()
        super().__init__(official_wrapper_for(self.fake, xr_cfg.get("arm_reference_mode", "head_yaw")),
                         xr_cfg.get("stale_timeout_s", 0.5))
        self.scenario = scenario
        self.duration = SCENARIOS[scenario][0]
        self.t0 = None
        self.rng = np.random.default_rng(seed)
        self.done = False
        self.frozen = False

    def _set_pose(self, side: str, dp_robot, jitter=True):
        base = xf.CONST_LEFT_ARM_POSE if side == "left" else xf.CONST_RIGHT_ARM_POSE
        T = base.copy()
        T[:3, 3] += xf.delta_robot_to_xr(dp_robot)
        if jitter:  # real tracking is never bit-identical; keeps the freshness check meaningful
            T[:3, 3] += self.rng.normal(0, 5e-5, 3)
        setattr(self.fake, f"{side}_arm_pose", T)

    def _pulse(self, attr: str, t, *t_on, width=0.15):
        """Button held for `width` s at each time in t_on (one call per button: a second call for the same
        button would overwrite the first pulse with False)."""
        setattr(self.fake, attr, any(a <= t < a + width for a in t_on))

    def step_scenario(self, t: float):
        f = self.fake
        f.motion_data_ready = t >= 0.2
        sc = self.scenario
        dl, dr = np.zeros(3), np.zeros(3)
        tl = tr = 0.0
        if sc == "dry_run":
            self._pulse("left_ctrl_aButton", t, 1.0, 11.5)
            dr += 0.03 * np.array([_bump(t, 2.0, 4.0), 0.0, _bump(t, 4.0, 6.0)])
            dl += 0.03 * np.array([0.0, _bump(t, 2.0, 4.0), 0.0])
            tl = 1.0 if (6.5 <= t < 7.5 or 9.5 <= t < 10.5) else 0.0
            tr = 1.0 if (8.0 <= t < 9.0 or 9.5 <= t < 10.5) else 0.0
        elif sc == "dropout":
            self._pulse("left_ctrl_aButton", t, 1.0, 7.5)
            dr += np.array([0.04 * _ramp(t, 1.5, 3.0), 0.0, 0.0])
            tr = 1.0 if 2.0 <= t < 3.0 else 0.0
            self.frozen = 3.0 <= t < 5.0
            if self.frozen:
                f.right_ctrl_triggerValue = 0.0      # what a dying link may report
                return                               # poses frozen (bit-identical) -> stale
            if t >= 5.0:                             # resume at a different controller position
                dr += np.array([0.10, 0.0, 0.0])
                tr = 1.0 if t < 6.0 else 0.0
        elif sc == "recenter":
            self._pulse("left_ctrl_aButton", t, 1.0, 7.0)
            dr += np.array([0.08 * _ramp(t, 1.5, 3.0) * (1 - _ramp(t, 4.5, 6.0)), 0.0, 0.0])
            self._pulse("left_ctrl_bButton", t, 3.8)
        elif sc == "walk_xr":
            f.left_ctrl_thumbstickValue = np.array([0.0, -1.0 * _ramp(t, 1.0, 1.3) * (1 - _ramp(t, 3.0, 3.1))])
            f.right_ctrl_thumbstickValue = np.array([-1.0 * _ramp(t, 4.5, 4.8) * (1 - _ramp(t, 6.5, 6.6)), 0.0])
        elif sc == "walk_seq":
            def win(a, b):  # smooth on/off window
                return _ramp(t, a, a + 0.3) * (1 - _ramp(t, b, b + 0.3))
            # full deflection: the official policy does not step for small lateral/backward/yaw commands
            ly = -1.0 * win(2, 10) + 1.0 * win(26, 32)          # forward, later backward
            lx = -1.0 * win(14, 22)                              # left (vy > 0)
            rx = -1.0 * win(36, 44) + 1.0 * win(48, 52)          # turn left (wz > 0), then right
            f.left_ctrl_thumbstickValue = np.array([lx, ly])
            f.right_ctrl_thumbstickValue = np.array([rx, 0.0])
        elif sc == "walk_arm":
            def win(a, b):
                return _ramp(t, a, a + 0.3) * (1 - _ramp(t, b, b + 0.3))
            self._pulse("left_ctrl_aButton", t, 1.0, 34.0)
            f.left_ctrl_thumbstickValue = np.array([0.0, -1.0 * win(3, 13)])        # forward while...
            f.right_ctrl_thumbstickValue = np.array([-1.0 * win(18, 26), 0.0])      # ...then turn left
            dr += 0.06 * np.array([_bump(t, 4.0, 12.0), 0.0, 0.5 * _bump(t, 4.0, 12.0)])
            tr = 1.0 if 7.0 <= t < 10.0 else 0.0
            dl += 0.05 * np.array([0.0, _bump(t, 19.0, 25.0), _bump(t, 19.0, 25.0)])
        self._set_pose("left", dl)
        self._set_pose("right", dr)
        f.left_ctrl_triggerValue, f.right_ctrl_triggerValue = tl, tr
        f.left_ctrl_trigger, f.right_ctrl_trigger = tl > 0.5, tr > 0.5

    def read(self, now: float | None = None, ctx=None) -> InputFrame:
        now = time.monotonic() if now is None else now
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        self.done = t >= self.duration
        self.step_scenario(t)
        fr = super().read(now)
        fr.raw["scenario_t"] = t
        fr.raw["frozen"] = self.frozen
        return fr


class ScriptedGraspSource(MockSource):
    """Right-hand reach -> close -> lift using simulator ground truth (object + base pose).
    The plan is expressed as IK-frame targets and converted back into raw controller poses, so it
    exercises exactly the same conversion / relative mapping / IK / limits as a human operator."""

    name = "scripted"

    def __init__(self, xr_cfg: dict, plan: dict, seed: int = 0):
        super().__init__("hold", xr_cfg, seed)
        self.plan = plan
        self.phase = "wait"
        self.t_phase = 0.0
        self.log: list = []
        self._T_from = None
        self.result: dict = {}
        self._corr = np.zeros(3)      # closed-loop approach correction (plan key servo_approach), pelvis frame
        self._t_servo = None
        self._servo_err = None
        self.objects_start: dict | None = None   # every cylinder's position when the source first saw the sim

    def _note_objects_start(self, status: dict) -> None:
        objs = (status.get("eval_gt") or {}).get("objects")
        if self.objects_start is None and objs:
            self.objects_start = {k: list(v) for k, v in objs.items()}

    def _finish_result(self, status: dict) -> dict:
        """Result at the end of the plan: target lifted >= success_lift_m, robot not fallen and (several cylinders)
        no other cylinder pushed more than max_other_displacement_m (default 0.03 m) from where it stood."""
        P, g = self.plan, status.get("eval_gt", {})
        others = {n: round(float(np.linalg.norm(np.array(p[:2]) - np.array(self.objects_start[n][:2]))), 4)
                  for n, p in (g.get("objects") or {}).items()
                  if n != g.get("target") and self.objects_start and n in self.objects_start}
        disturbed = max(others.values(), default=0.0) > float(P.get("max_other_displacement_m", 0.03))
        return {"target": g.get("target"), "object_lift_m": g.get("object_lift_m"),
                "contact_force_total": g.get("contact_force_total"), "fallen": g.get("fallen"),
                "others_moved_m": others or None, "others_disturbed": disturbed,
                "success": bool((g.get("object_lift_m") or 0) >= P["success_lift_m"] and not g.get("fallen")
                                and not disturbed)}

    def _obj_in_pelvis(self, status: dict):
        g = status.get("eval_gt", {})
        if "object_pos_w" not in g:
            return None
        from g1iv_common.arm_math import rotz

        p_b = np.array(status["base_pos_w"])
        Rz = rotz(float(status["base_yaw"]))
        return Rz.T @ (np.array(g["object_pos_w"]) - p_b)

    def _ree_in_pelvis(self, status: dict, side: str):
        """Measured IK end effector (<side>_wrist_yaw_link + 0.05 m along x) in the pelvis frame, from ground truth."""
        g = status.get("eval_gt", {})
        bp = (g.get("body_pose_w") or {}).get(f"{side}_wrist_yaw")
        if bp is None or "base_pos_w" not in status:
            return None
        from g1iv_common.arm_math import rotz

        w, x, y, z = bp[3:]
        Rw = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                       [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                       [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        p = np.array(bp[:3]) + Rw @ np.array([0.05, 0.0, 0.0])
        return rotz(float(status["base_yaw"])).T @ (p - np.array(status["base_pos_w"]))

    def read(self, now: float | None = None, ctx=None) -> InputFrame:
        now = time.monotonic() if now is None else now
        status = ctx.status if ctx is not None else {}
        self._note_objects_start(status)
        # Plan clock = SIMULATOR time when available (the sim runs well below real time on this PC; a wall
        # clock plan would move the arm several times faster in sim time than the durations say).
        if status.get("sim_time") is None and ctx is not None:
            t = 0.0                              # no simulator status yet: the plan clock does not start
        else:
            clock = float(status["sim_time"]) if status.get("sim_time") is not None else now
            if self.t0 is None:
                self.t0 = clock
                self.clock = "sim_time" if status.get("sim_time") is not None else "wall"
            t = clock - self.t0
        f = self.fake
        f.motion_data_ready = True
        # hand-specific geometry (configs/scripted_grasp.yaml hand_models), chosen once from the simulator's
        # status before the plan clock starts (the hand model is a simulator setting, configs/scene_desk.yaml)
        hm = status.get("hand_model")
        if hm and not getattr(self, "_hand_model", None):
            self._hand_model = hm
            ov = (self.plan.get("hand_models") or {}).get(hm, {})
            self.plan = {**self.plan, **ov}
            self.log.append({"phase": "hand_model", "t": float(t), "hand_model": hm, "overrides": sorted(ov)})
        P = self.plan
        side = P.get("side", "right")
        anchors = ctx.anchors if ctx is not None else {}
        dpos = np.zeros(3)
        T_ctrl_ik = None

        def goto(name, dur):
            if self.phase != name:
                self.log.append({"phase": name, "t": t})
                self.phase, self.t_phase = name, t
                self._T_from = None

        if self.phase == "wait" and t > 1.0:
            goto("engage", 0)
        if self.phase == "engage":
            f.left_ctrl_aButton = (t - self.t_phase) < 0.15
            if anchors.get(side) is not None and (t - self.t_phase) > 0.5:
                obj = self._obj_in_pelvis(status)
                desk_err = _desk_mismatch(P, status) or _target_mismatch(P, status)
                if obj is None:
                    self.result = {"error": "no object ground truth in sim status"}
                    goto("done", 0)
                elif desk_err:
                    self.result = {"success": False, "error": desk_err}
                    print(f"[scripted] {desk_err}", flush=True)
                    goto("done", 0)
                else:
                    self.obj0 = obj
                    goto("raise", 0)
        phases = ("raise", "pregrasp", "approach", "close", "lift", "hold")
        if self.phase in phases and anchors.get(side) is not None:
            T_ee0, T_c0 = anchors[side]
            # follow the object (pelvis frame) until the hand closes: the base may shift a few cm while the arm
            # moves; afterwards the plan is frozen so lifting does not chase the lifted object
            if self.phase in ("raise", "pregrasp", "approach"):
                cur = self._obj_in_pelvis(status)
                if cur is not None:
                    self.obj0 = 0.8 * self.obj0 + 0.2 * cur
            obj = self.obj0
            # 1) raise + rotate BEHIND the desk edge (fingers sweep from pointing down to pointing forward without
            #    touching the desk), 2) move over the desk top to the pregrasp pose, 3) approach, close, lift, hold
            # grasp orientation (R_ee in the pelvis/IK frame; rpy 0 = fingers forward, right palm facing left).
            # offsets_frame "hand": the offsets are fixed in that rotated hand frame (the object keeps its place in the
            # palm when the hand is tilted); "pelvis" (default, the original plan): plain pelvis-frame offsets.
            R_goal = _rpy_matrix(*np.radians(P.get("grasp_rpy_deg", [0.0, 0.0, 0.0])))
            R_off = R_goal if P.get("offsets_frame", "pelvis") == "hand" else np.eye(3)
            obj_g = obj + np.array([0.0, 0.0, float(P.get("grasp_point_dz", 0.0))])   # grasp height on the cylinder
            pre = obj_g + R_off @ np.array(P["pregrasp_offset"])
            raise_p = np.array([T_ee0[0, 3] + P.get("raise_dx", 0.0), pre[1], P.get("raise_z", 0.06)])
            g = obj_g + R_off @ np.array(P["grasp_offset"])
            wp = {"raise": raise_p, "pregrasp": pre, "approach": g, "close": g,
                  "lift": g + np.array([0, 0, P["lift_height"]]), "hold": g + np.array([0, 0, P["lift_height"]])}
            dur = {"raise": P.get("t_raise", 3.0), "pregrasp": P["t_pregrasp"], "approach": P["t_approach"],
                   "close": P["t_close"], "lift": P["t_lift"], "hold": P["t_hold"]}
            if self.phase == "raise" and self._T_from is None:
                self._T_from = T_ee0[:3, 3].copy()
            prev = {"raise": self._T_from if self._T_from is not None else T_ee0[:3, 3], "pregrasp": wp["raise"],
                    "approach": wp["pregrasp"], "close": wp["approach"], "lift": wp["close"], "hold": wp["lift"]}
            a = _ramp(t, self.t_phase, self.t_phase + dur[self.phase])
            p_des = (1 - a) * prev[self.phase] + a * wp[self.phase]
            # Optional closed-loop approach (scripted test only, ground truth): the arm does not reach the grasp
            # point exactly (IK residual + tracking lag ~1-2 cm); integrate the measured hand-position error during
            # the second half of the approach and keep the correction for close / lift / hold.
            if P.get("servo_approach", False) and self.phase in ("approach", "close", "lift", "hold"):
                if self.phase == "approach" and a > 0.5:
                    act = self._ree_in_pelvis(status, side)
                    if act is not None:
                        err = g - act
                        dtp = 0.0 if self._t_servo is None else max(0.0, t - self._t_servo)
                        lim = float(P.get("servo_limit_m", 0.04))
                        self._corr = np.clip(self._corr + float(P.get("servo_gain", 2.0)) * err * dtp, -lim, lim)
                        self._servo_err = float(np.linalg.norm(err))
                    self._t_servo = t
                corr = self._corr.copy()
                # The vertical part of the correction can be beyond what the arm reaches (it is pushed down until the
                # hand is low enough, while the IK target sits below the hand): release it during the lift, or the first
                # centimetres of the lift only shrink the IK residual instead of raising the hand.
                if self.phase == "lift":
                    corr[2] *= 1.0 - a
                elif self.phase == "hold":
                    corr[2] = 0.0
                p_des = p_des + corr
            T_des = T_ee0.copy()
            T_des[:3, 3] = p_des
            # hand orientation: rotate from the anchor orientation to the grasp orientation (R_goal) while raising
            a_rot = a if self.phase == "raise" else 1.0
            T_des[:3, :3] = _so3_interp(T_ee0[:3, :3], R_goal, a_rot)
            # controller pose in IK frame that the relative mapping (hub, arm_math.rel_target) turns into T_des:
            # p = p_ee0 + (p_c - p_c0),  R = (R_c R_c0^T) R_ee0
            T_ctrl_ik = T_c0.copy()
            T_ctrl_ik[:3, 3] = T_c0[:3, 3] + (T_des[:3, 3] - T_ee0[:3, 3])
            T_ctrl_ik[:3, :3] = (T_des[:3, :3] @ T_ee0[:3, :3].T) @ T_c0[:3, :3]
            close = self.phase in ("close", "lift", "hold")
            setattr(f, f"{side}_ctrl_triggerValue", 1.0 if close else 0.0)
            settled = True
            if P.get("servo_approach", False) and self.phase == "approach":
                settled = (self._servo_err is not None and self._servo_err <= float(P.get("servo_tol_m", 0.005))) or \
                    (t - self.t_phase) >= dur["approach"] + float(P.get("servo_settle_max_s", 2.0))
            if (t - self.t_phase) >= dur[self.phase] and settled:
                if self.phase == "approach" and P.get("servo_approach", False):
                    self.log.append({"phase": "servo", "t": t, "err_m": self._servo_err, "corr_m": self._corr.tolist()})
                nxt = {"raise": "pregrasp", "pregrasp": "approach", "approach": "close", "close": "lift",
                       "lift": "hold", "hold": "done"}
                if nxt[self.phase] == "done":
                    self.result = self._finish_result(status)
                goto(nxt[self.phase], 0)
        if self.phase == "done":
            self.done = True
        # write raw poses
        head = f.head_pose
        other = "left" if side == "right" else "right"
        if T_ctrl_ik is not None:
            T_raw = xf.inverse_controller(head, T_ctrl_ik)
            T_raw[:3, 3] += self.rng.normal(0, 5e-5, 3)
            setattr(f, f"{side}_arm_pose", T_raw)
        else:
            self._set_pose(side, dpos)
        self._set_pose(other, np.zeros(3))
        fr = _XRBase.read(self, now)
        fr.raw.update({"scenario_t": t, "phase": self.phase, "plan_clock": getattr(self, "clock", "wall")})
        return fr


def _target_mismatch(plan: dict, status: dict) -> str | None:
    """Plan key expect_target (set per episode by tools/batch_walk_grasp.py --plan-set) must equal the cylinder the
    simulator reports as target: the reset message has no reply, so a lost reset would otherwise record the wrong
    cylinder under the instruction's colour. None = fine or nothing to check yet."""
    want, g = plan.get("expect_target"), status.get("eval_gt")
    if not want or not g:
        return None
    if g.get("target") != want:
        return (f"target mismatch: plan expects {want!r} but the simulator target is {g.get('target')!r} "
                "(reset --target lost, unknown name, or extra_objects disabled)")
    return None


def _desk_mismatch(plan: dict, status: dict) -> str | None:
    """The grasp geometry of a plan is made for one desk height (plan key desk_top_z). Compare it with the object's
    resting height from the simulator (ground truth) before the arm moves; None = fine or not checkable."""
    want = plan.get("desk_top_z")
    g = status.get("eval_gt", {})
    if want is None or "object_pos_w" not in g or g.get("object_lift_m") is None:
        return None
    rest_z = float(g["object_pos_w"][2]) - float(g["object_lift_m"])          # object centre at spawn
    desk = rest_z - float(plan.get("object_half_height", 0.06)) - 0.002          # desk_task.py spawns 2 mm above
    if abs(desk - float(want)) > 0.02:
        return (f"desk height mismatch: this plan is for desk_top_z {float(want):.3f} m but the desk top is at "
                f"{desk:.3f} m (configs/scene_desk.yaml desk.top_surface_height); pick the matching plan")
    return None


def _wrap(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)


def _stick(v: float, vmax: float, dz: float) -> float:
    """Raw thumbstick deflection that hub.LocoArbiter.feed_xr maps to velocity v (inverse of deadzone + scale;
    the sign convention v = -stick * vmax is the official G1 motion code's)."""
    u = float(np.clip(-v / vmax, -1.0, 1.0))
    if abs(u) < 1e-6:
        return 0.0
    return float(np.sign(u) * (abs(u) * (1.0 - dz) + dz))


class ScriptedWalkGraspSource(ScriptedGraspSource):
    """Walk -> stand -> scripted grasp + lift, automatic (simulator ground truth; test / sim-data tool only).

    1) walk: from wherever the robot stands to a stand pose defined relative to the object (plan walk.stand_rel, in the
       goal-yaw frame) and facing walk.goal_yaw_deg (world). Walking commands are written as XR THUMBSTICK values
       (hub --loco-source xr), i.e. the same path an operator's PICO sticks take: televuer -> LocoArbiter ->
       rt/run_command/cmd. A Stanley-type heading law steers onto the line through the stand pose, the forward command
       stops walk.stop_dist_m before it (the official policy coasts ~0.09 m), then the robot settles.
    2) correct: if the settled pose is outside walk.tol_*, short single-axis pulses (lateral / forward / back / turn)
       with re-settling, at most walk.max_corrections times. The official policy only steps for large commands
       (|vy| >= 0.5, wz >= ~1.2 in place), hence pulses of full magnitude and variable length.
    3) the scripted grasp of ScriptedGraspSource (engage, raise, pregrasp, approach, close, lift, hold).
    """

    name = "scripted"

    def __init__(self, xr_cfg: dict, plan: dict, loco_xr_cfg: dict, seed: int = 0):
        super().__init__(xr_cfg, plan, seed)
        self.lxr = loco_xr_cfg
        self.nav = "nav_wait"
        self.nav_t0 = None
        self.nav_tp = 0.0
        self.nav_cmd = np.zeros(3)
        self.n_corr = 0
        self.pulse: dict | None = None
        self.record_ready = False      # hub: start recording here (walking is part of the episode)
        self.nav_result: dict = {}

    # ------------------------------------------------------------------ helpers
    def _hm_plan(self, status: dict) -> dict:
        hm = status.get("hand_model")
        return {**self.plan, **((self.plan.get("hand_models") or {}).get(hm, {}))} if hm else self.plan

    def _nav_state(self, status: dict, W: dict):
        """(along s, cross e, yaw error) of the base w.r.t. the stand pose; s < 0 = still behind it."""
        g = status.get("eval_gt", {})
        if "object_pos_w" not in g or "base_pos_w" not in status:
            return None
        psi = np.radians(float(W.get("goal_yaw_deg", 0.0)))
        c, s_ = np.cos(psi), np.sin(psi)
        R = np.array([[c, -s_], [s_, c]])
        goal = np.array(g["object_pos_w"][:2]) - R @ np.array(W["stand_rel"], float)
        dp = np.array(status["base_pos_w"][:2]) - goal
        along, cross = float(dp @ np.array([c, s_])), float(dp @ np.array([-s_, c]))
        return along, cross, _wrap(float(status["base_yaw"]) - psi), goal, psi

    @staticmethod
    def _body_err(status: dict, W: dict):
        """Stand error in the ROBOT frame, the frame the grasp and the pulses work in: (along, cross) with the same
        sign meaning as _nav_state (along < 0: object still too far ahead; cross > 0: robot too far left)."""
        g = status.get("eval_gt", {})
        yaw = float(status["base_yaw"])
        c, s_ = np.cos(yaw), np.sin(yaw)
        d = np.array(g["object_pos_w"][:2]) - np.array(status["base_pos_w"][:2])
        r = np.array([c * d[0] + s_ * d[1], -s_ * d[0] + c * d[1]]) - np.array(W["stand_rel"], float)
        return float(-r[0]), float(-r[1])

    def _set_sticks(self, vx: float, vy: float, wz: float):
        dz = float(self.lxr["deadzone"])
        f = self.fake
        f.left_ctrl_thumbstickValue = np.array([_stick(vy, float(self.lxr["vy_max"]), dz),
                                                _stick(vx, float(self.lxr["vx_max"]), dz)])
        f.right_ctrl_thumbstickValue = np.array([_stick(wz, float(self.lxr["wz_max"]), dz), 0.0])
        self.nav_cmd = np.array([vx, vy, wz])

    def _nav_goto(self, name: str, t: float, **info):
        self.log.append({"phase": name, "t": t, **info})
        self.nav, self.nav_tp = name, t

    # ------------------------------------------------------------------ walking state machine
    def _nav_step(self, t: float, status: dict):
        W = self._hm_plan(status)["walk"]
        ns = self._nav_state(status, W)
        if ns is None:
            self._set_sticks(0, 0, 0)
            return
        s, e, yerr, goal, psi = ns
        dt_ph = t - self.nav_tp
        if t > float(W.get("timeout_s", 40.0)) and self.nav != "nav_done":
            self._set_sticks(0, 0, 0)
            self.nav_result = {"ok": False, "reason": "timeout", "along": s, "cross": e, "yaw_err_deg": np.degrees(yerr)}
            self._nav_goto("nav_failed", t, **self.nav_result)
            return
        if self.nav == "nav_wait":
            self._set_sticks(0, 0, 0)
            if t >= float(W.get("start_delay_s", 1.0)):
                self.record_ready = True
                self._nav_goto("walk", t, along=s, cross=e, yaw_err_deg=np.degrees(yerr), goal=goal.tolist())
        elif self.nav == "walk":
            ramp = min(1.0, dt_ph / float(W.get("ramp_s", 0.3)))
            wmax = float(W.get("wz_walk_max", 0.8))
            if W.get("lateral_mode", "holonomic") == "holonomic":
                # face the goal yaw all the way (yaw-rate feedback works while stepping) and remove the cross-track
                # error with a sideways component (also works while stepping; in place |vy| < 0.5 does not step)
                wz = float(np.clip(-float(W.get("k_yaw", 2.0)) * yerr, -wmax, wmax))
                vy_g = float(np.clip(-float(W.get("k_lat", 3.0)) * e, -W.get("vy_walk_max", 0.5), W.get("vy_walk_max", 0.5)))
                vx_g = ramp * float(W.get("vx_walk", 0.5))
                c, sn = np.cos(yerr), np.sin(yerr)                 # goal frame -> robot frame
                vx, vy = c * vx_g + sn * vy_g, -sn * vx_g + c * vy_g
            else:
                # Stanley-type law: desired heading = goal yaw + atan(-k e); wz proportional to the heading error
                psi_d_err = _wrap(yerr + np.arctan(float(W.get("k_cross", 2.5)) * e))
                wz = float(np.clip(-float(W.get("k_yaw", 2.0)) * psi_d_err, -wmax, wmax))
                vx, vy = ramp * float(W.get("vx_walk", 0.5)), 0.0
            if -s > float(W.get("stop_dist_m", 0.12)):
                self._set_sticks(vx, vy, wz)
            else:
                self._set_sticks(0, 0, 0)
                self._nav_goto("settle", t, along=s, cross=e, yaw_err_deg=np.degrees(yerr))
        elif self.nav == "pulse":
            p = self.pulse
            if dt_ph < p["dur"]:
                self._set_sticks(*p["cmd"])
            else:
                self._set_sticks(0, 0, 0)
                nxt = "pre_walk" if p.get("next") == "walk" else "settle"
                self._nav_goto(nxt, t, along=s, cross=e, yaw_err_deg=np.degrees(yerr))
        elif self.nav == "pre_walk":                  # brief stop between backing off and walking in again
            self._set_sticks(0, 0, 0)
            if dt_ph >= float(W.get("pre_walk_s", 0.6)):
                self._nav_goto("walk", t, along=s, cross=e, yaw_err_deg=np.degrees(yerr))
        elif self.nav == "settle":
            self._set_sticks(0, 0, 0)
            v = np.linalg.norm(np.array(status.get("base_lin_vel_b", [0, 0, 0]))[:2])
            if dt_ph >= float(W.get("settle_s", 1.0)) and (v < 0.03 or dt_ph >= 3.0):
                tx, ty, tyaw = float(W["tol_along_m"]), float(W["tol_cross_m"]), np.radians(float(W["tol_yaw_deg"]))
                s, e = self._body_err(status, W)       # object position in the robot frame vs stand_rel
                info = dict(along=s, cross=e, yaw_err_deg=float(np.degrees(yerr)), n_corr=self.n_corr)
                if abs(s) <= tx and abs(e) <= ty and abs(yerr) <= tyaw:
                    self.nav_result = {"ok": True, **info}
                    self._nav_goto("nav_done", t, **info)
                elif self.n_corr >= int(W.get("max_corrections", 4)):
                    self.nav_result = {"ok": False, "reason": "out_of_tolerance", **info}
                    self._nav_goto("nav_failed", t, **self.nav_result)
                else:
                    self.n_corr += 1
                    if W.get("correction", "reapproach") == "reapproach":
                        # Short corrective pulses do not make the policy step (it sways and returns: measured), so
                        # back off one step and walk in again; the walking law removes cross / yaw errors while
                        # stepping and the stop distance sets the along error.
                        Pp = W.get("pulse", {})
                        self.pulse = {"axis": "backoff", "cmd": [-float(Pp.get("vx_back", 0.6)), 0.0, 0.0],
                                      "dur": float(W.get("backoff_s", 1.0)), "next": "walk"}
                    else:
                        self.pulse = self._plan_pulse(s, e, yerr, tx, ty, tyaw, W)
                    self._nav_goto("pulse", t, **info, pulse=self.pulse)

    @staticmethod
    def _plan_pulse(s, e, yerr, tx, ty, tyaw, W) -> dict:
        """One corrective pulse on the worst axis (relative to its tolerance). Durations from the measured response
        of the official policy (logs/checks/auto_walk_grasp/*probe*.json): lateral 0.5 m/s ~0.13 m/s + ~0.05 m coast,
        forward 0.5 m/s ~0.23 m/s + ~0.09 m coast."""
        P = W.get("pulse", {})
        worst = max((abs(yerr) / tyaw, "yaw"), (abs(e) / ty, "cross"), (abs(s) / tx, "along"))[1]
        if worst == "yaw":
            # in place the policy only twists the pelvis for short turn commands and springs back (0.5-1.2 s left
            # pulses: no step); 0.8 s right turned 10.7 deg -> at least 0.8 s
            dur = float(np.clip(abs(yerr) / float(P.get("yaw_rate", 0.2)), float(P.get("yaw_min_s", 0.8)), 1.5))
            return {"axis": "yaw", "cmd": [0.0, 0.0, -np.sign(yerr) * float(P.get("wz", 1.57))], "dur": dur}
        if worst == "cross":
            d = abs(e) - float(P.get("lat_coast_m", 0.05))
            dur = float(np.clip(d / float(P.get("lat_speed", 0.13)), 0.25, 1.5))
            return {"axis": "cross", "cmd": [0.0, -np.sign(e) * float(P.get("vy", 0.5)), 0.0], "dur": dur}
        if s < 0:   # still short of the stand pose
            d = abs(s) - float(P.get("fwd_coast_m", 0.09))
            dur = float(np.clip(d / float(P.get("fwd_speed", 0.23)), 0.25, 1.2))
            return {"axis": "along", "cmd": [float(P.get("vx_fwd", 0.5)), 0.0, 0.0], "dur": dur}
        d = abs(s) - float(P.get("back_coast_m", 0.05))
        dur = float(np.clip(d / float(P.get("back_speed", 0.2)), 0.25, 1.2))
        return {"axis": "along", "cmd": [-float(P.get("vx_back", 0.6)), 0.0, 0.0], "dur": dur}

    # ------------------------------------------------------------------ main
    def read(self, now: float | None = None, ctx=None) -> InputFrame:
        now = time.monotonic() if now is None else now
        status = ctx.status if ctx is not None else {}
        self._note_objects_start(status)
        if self.nav == "nav_wait" and not self.result:
            mm = _target_mismatch(self.plan, status)
            if mm:                                 # before record_ready: nothing is recorded
                print(f"[scripted] {mm}", flush=True)
                self.result = {"success": False, "error": mm, "target": status["eval_gt"].get("target")}
                self.nav = "nav_failed"
        if self.nav == "nav_done":
            self._set_sticks(0, 0, 0)
            fr = super().read(now, ctx)            # the grasp plan starts its own clock here
            fr.raw["nav_cmd"] = self.nav_cmd.tolist()
            if self.result and "nav" not in self.result:
                self.result["nav"] = self.nav_result
            return fr
        f = self.fake
        f.motion_data_ready = True
        if status.get("sim_time") is None:
            t = 0.0
        else:
            if self.nav_t0 is None:
                self.nav_t0 = float(status["sim_time"])
            t = float(status["sim_time"]) - self.nav_t0
        if self.nav == "nav_failed":
            self._set_sticks(0, 0, 0)
            if not self.result:
                g = status.get("eval_gt", {})
                self.result = {"success": False, "error": "navigation failed", "nav": self.nav_result,
                               "target": g.get("target"), "object_lift_m": g.get("object_lift_m"),
                               "fallen": g.get("fallen")}
            self.done = True
        else:
            self._nav_step(t, status)
        self._set_pose("left", np.zeros(3))
        self._set_pose("right", np.zeros(3))
        fr = _XRBase.read(self, now)
        fr.raw.update({"scenario_t": t, "phase": self.nav, "plan_clock": "sim_time", "nav_cmd": self.nav_cmd.tolist()})
        return fr


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _so3_interp(R0: np.ndarray, R1: np.ndarray, a: float) -> np.ndarray:
    """Geodesic interpolation R0 -> R1 (a in [0, 1])."""
    Rr = R0.T @ R1
    c = float(np.clip((np.trace(Rr) - 1.0) / 2.0, -1.0, 1.0))
    th = float(np.arccos(c))
    if th < 1e-9:
        return R1.copy() if a >= 1.0 else R0.copy()
    w = np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]]) / (2.0 * np.sin(th))
    if np.pi - th < 1e-6:  # 180 deg: axis from the symmetric part
        w = np.sqrt(np.clip((np.diag(Rr) + 1.0) / 2.0, 0.0, None))
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    ang = a * th
    return R0 @ (np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K))
