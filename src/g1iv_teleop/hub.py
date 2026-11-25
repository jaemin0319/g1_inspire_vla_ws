"""Teleop hub: the single owner of upper-body, hand and walking commands for the simulator.

Data flow (DDS domain 1; interface from --dds-iface, default lo):
  input source (pico | mock | scripted | keyboard | none)
    -> official TeleVuerWrapper conversion (XR sources)            [televuer, unchanged]
    -> clutched relative mapping in the IK (waist) frame           [this file]
    -> official G1_29_ArmIK.solve_ik                               [xr_teleoperate, unchanged]
    -> joint-rate limit + joint-limit clip                         [this file; sim mode has none upstream]
    -> official G1_29_ArmController -> rt/lowcmd (250 Hz)          [xr_teleoperate, unchanged]
  triggers / keyboard -> OPEN/CLOSE (hysteresis, hold on input loss) -> rt/inspire/cmd (DFX 12ch)
  keyboard (console window) XOR XR thumbsticks -> LocoArbiter -> rt/run_command/cmd (100 Hz)
  everything -> rt/g1iv/teleop_meta (recorded by the simulator next to each frame)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[2]
XR_ROOT = WS / "repos" / "xr_teleoperate"
sys.path.insert(0, str(WS / "src"))

from g1iv_common import arm_math as AM  # noqa: E402
from g1iv_common import hands as H  # noqa: E402
from g1iv_common import loco as L  # noqa: E402
from g1iv_common import topics  # noqa: E402
from g1iv_common.config import load_config, load_yaml, ws_path  # noqa: E402

SIDES = ("left", "right")
SL = {"left": slice(0, 7), "right": slice(7, 14)}


class Ctx:
    def __init__(self):
        self.status: dict = {}
        self.anchors: dict = {"left": None, "right": None}


class ButtonEdges:
    def __init__(self):
        self.prev: dict = {}

    def update(self, fr) -> set:
        cur = {"left_a": fr.a[0], "right_a": fr.a[1], "left_b": fr.b[0], "right_b": fr.b[1],
               "left_squeeze": fr.squeeze[0], "right_squeeze": fr.squeeze[1],
               "both_thumbsticks": fr.stick_click[0] and fr.stick_click[1]}
        rising = {k for k, v in cur.items() if v and not self.prev.get(k, False)}
        self.prev = cur
        return rising


class Hub:
    def __init__(self, a):
        self.a = a
        self.cfg = load_config(a.config, a.set)
        c = self.cfg
        self.arm_source = a.arm_source or c["arm_source"]
        self.loco_source = a.loco_source or c["loco_source"]
        self.rate = float(c["rate_hz"])
        self.dt = 1.0 / self.rate
        self.dt_sim = self.dt          # this tick's dt in simulator time (see _rtf_scale)
        self.ctx = Ctx()
        self.quit = False
        self.trace: list = []
        self.run_id = time.strftime("%Y%m%d_%H%M%S") + f"_{self.arm_source}_{a.scenario or ''}".rstrip("_")
        self.events_log: list = []

        # ---------------- DDS
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_

        ChannelFactoryInitialize(topics.SIM_DDS_DOMAIN, a.dds_iface or None)
        self.String_, self.MotorCmds_, self.MotorCmd_ = String_, MotorCmds_, unitree_go_msg_dds__MotorCmd_
        self.pub_run = ChannelPublisher(topics.T_RUN_COMMAND, String_); self.pub_run.Init()
        self.pub_hand = ChannelPublisher(topics.T_INSPIRE_CMD, MotorCmds_); self.pub_hand.Init()
        self.pub_meta = ChannelPublisher(topics.T_TELEOP_META, String_); self.pub_meta.Init()
        self.pub_ctrl = ChannelPublisher(topics.T_CONTROL, String_); self.pub_ctrl.Init()
        self._status_lock = threading.Lock()
        self._status_rx = 0.0
        self.sub_status = ChannelSubscriber(topics.T_SIM_STATUS, String_)
        self.sub_status.Init(self._on_status, 8)

        # ---------------- arms (official IK + controller). cwd matters for the official relative paths.
        print("[hub] waiting for the simulator (rt/lowstate) ...", flush=True)
        sys.path.insert(0, str(XR_ROOT))
        prev = os.getcwd()
        os.chdir(XR_ROOT / "teleop")
        try:
            from teleop.robot_control.robot_arm import G1_29_ArmController
            from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

            class _HoldStartArmController(G1_29_ArmController):
                """The official controller starts its 250 Hz publish thread inside __init__ with q_target = 0
                and, in simulation_mode, no velocity clip: the arms would snap to the zero pose when the hub
                starts. Hold the thread until the hub has set q_target to the measured arm pose."""

                def _ctrl_motor_state(self):
                    while not getattr(self, "_g1iv_release", False):
                        time.sleep(0.002)
                    return super()._ctrl_motor_state()

            self.arm_ik = G1_29_ArmIK()
            self.arm_ctrl = _HoldStartArmController(motion_mode=False, simulation_mode=True)
        finally:
            os.chdir(prev)
        import pinocchio as pin

        self.pin = pin
        self.m = self.arm_ik.reduced_robot.model
        self.d = self.m.createData()
        self.fid = {"left": self.m.getFrameId("L_ee"), "right": self.m.getFrameId("R_ee")}
        self.q_lo, self.q_hi = np.array(self.m.lowerPositionLimit), np.array(self.m.upperPositionLimit)
        q0 = self.arm_ctrl.get_current_dual_arm_q()
        with self.arm_ctrl.ctrl_lock:
            self.arm_ctrl.q_target = q0.copy()          # first command = where the arms already are
            self.arm_ctrl.tauff_target = np.zeros(14)
        self.arm_ctrl._g1iv_release = True
        self.q_goal = q0.copy()
        self.q_sent = q0.copy()
        ac = c["arm"]
        self.home_q = np.array(ac["home_q"]["left"] + ac["home_q"]["right"], float)
        self.engaged = False
        self.need_anchor = {s: True for s in SIDES}
        self.kb_off = {s: np.zeros(3) for s in SIDES}
        self.sel_side = "right"
        self.ik_ok, self.ik_err = True, [0.0, 0.0]
        self.n_ik_fail = 0

        # ---------------- hands / walking
        self.hands = H.DualHand(load_yaml(c["hands"]["preset_file"]), c["hands"])
        self.loco = L.LocoArbiter(c["loco"], source=self.loco_source)
        self._loco_lock = threading.Lock()
        self._loco_cmd = [0.0, 0.0, 0.0, self.loco.height]
        self._loco_update_mono = 0.0

        # ---------------- input source
        self.source = self._make_source()
        self.edges = ButtonEdges()
        self.console = None
        if c.get("console", True) and not a.no_console:
            from .console import Console

            self.console = Console(c["keyboard_keys"], title=f"G1IV hub [{self.arm_source}/{self.loco_source}]")
        if self.loco_source == "keyboard" and self.console is None:
            raise SystemExit("loco_source=keyboard needs the console window (drop --no-console or use --loco-source none|xr)")
        self.reset_seen = None
        self.recording = False
        self.seq = 0
        self._loco_thread = threading.Thread(target=self._loco_publisher, daemon=True)
        self._loco_thread.start()
        print(f"[hub] up: arm_source={self.arm_source} loco_source={self.loco_source} rate={self.rate} Hz", flush=True)

    # ============================================================ helpers
    def _make_source(self):
        s, c = self.arm_source, self.cfg
        if s in ("none", "keyboard"):
            return None
        from . import sources as S

        if s == "pico":
            from teleimager.image_client import ImageClient  # official

            ic = ImageClient(host=c["xr"]["img_server_ip"], request_bgr=True)
            self.img_client = ic
            return S.PicoSource(c["xr"], ic.get_cam_config(), c["xr"]["img_server_ip"])
        if s == "mock":
            return S.MockSource(self.a.scenario or "dry_run", c["xr"], seed=0)
        if s == "scripted":
            plan = load_config(self.a.plan, self.a.plan_set)
            if "walk" in plan:          # walk -> stand -> grasp (configs/scripted_walk_grasp_desk*.yaml)
                if self.loco_source != "xr":
                    raise SystemExit(f"plan {self.a.plan} walks: it drives the XR thumbsticks, use --loco-source xr")
                return S.ScriptedWalkGraspSource(c["xr"], plan, c["loco"]["xr"], seed=0)
            return S.ScriptedGraspSource(c["xr"], plan, seed=0)
        raise SystemExit(f"unknown arm_source {s}")

    def fk(self, q14):
        self.pin.framesForwardKinematics(self.m, self.d, np.asarray(q14, float))
        return {s: self.d.oMf[self.fid[s]].homogeneous.copy() for s in SIDES}

    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._status_lock:
            self.ctx.status = st
            self._status_rx = time.monotonic()

    def _rtf_scale(self) -> float:
        """Simulator real-time factor from rt/g1iv/sim_status (1.0 if unknown or stale), clipped to [0.05, 1]."""
        with self._status_lock:
            st, rx = self.ctx.status, getattr(self, "_status_rx", None)
        if not st or rx is None or (time.monotonic() - rx) > 1.0 or st.get("rtf") is None:
            return 1.0
        return float(np.clip(float(st["rtf"]), 0.05, 1.0))

    def send_ctrl(self, cmd, **kw):
        self.pub_ctrl.Write(self.String_(data=topics.control_msg(cmd, **kw)))
        self.events_log.append({"t": time.time(), "ctrl": cmd, **kw})
        print(f"[hub] control -> {cmd} {kw}", flush=True)

    def _loco_publisher(self):
        """100 Hz republisher (the policy zeroes the command after each read; a steady stream avoids
        zero gaps). If the main loop stalls for > 0.2 s it publishes zero instead of a stale command."""
        hz = float(self.cfg["loco"].get("publish_hz", 100.0))
        while not self.quit:
            with self._loco_lock:
                cmd = list(self._loco_cmd)
                age = time.monotonic() - self._loco_update_mono
            if age > 0.2:
                cmd = [0.0, 0.0, 0.0, self.loco.height]
            try:
                self.pub_run.Write(self.String_(data=topics.run_command_str(*cmd)))
            except Exception as e:  # noqa: BLE001
                print(f"[hub] run_command publish failed: {e}")
            time.sleep(1.0 / hz)

    # ============================================================ events
    def on_sim_reset(self):
        self.engaged = False
        self.need_anchor = {s: True for s in SIDES}
        self.kb_off = {s: np.zeros(3) for s in SIDES}
        self.hands.reset()
        q = self.arm_ctrl.get_current_dual_arm_q()
        self.q_goal = q.copy()
        self.q_sent = q.copy()
        self.loco.emergency_stop()
        print("[hub] simulator reset seen: disengaged, hands OPEN, arm targets = measured pose", flush=True)

    def handle(self, ev: str):
        self.events_log.append({"t": time.time(), "event": ev})
        if ev == "engage_toggle":
            self.engaged = not self.engaged
            self.need_anchor = {s: True for s in SIDES}
            self.kb_off = {s: np.zeros(3) for s in SIDES}
            print(f"[hub] {'ENGAGED' if self.engaged else 'DISENGAGED'}", flush=True)
        elif ev == "recenter":
            self.need_anchor = {s: True for s in SIDES}
            print("[hub] recenter: anchors re-captured at the current target pose", flush=True)
        elif ev == "hand_left_toggle":
            self.hands.toggle("left")
        elif ev == "hand_right_toggle":
            self.hands.toggle("right")
        elif ev == "hand_both_toggle":
            closed = self.hands.left.cmd == H.HandCmd.CLOSE and self.hands.right.cmd == H.HandCmd.CLOSE
            tgt = H.HandCmd.OPEN if closed else H.HandCmd.CLOSE
            self.hands.set_cmds(tgt, tgt)
        elif ev == "arm_select":
            self.sel_side = "left" if self.sel_side == "right" else "right"
        elif ev == "record_start":
            src = {"pico": "pico", "keyboard": "keyboard", "none": "keyboard"}.get(self.arm_source, self.arm_source)
            self.send_ctrl("record_start", instruction=self.a.instruction, task_name=self.a.task_name, source=src,
                           arm_source=self.arm_source, loco_source=self.loco_source, operator=self.a.operator)
        elif ev in ("record_stop_success", "record_stop_failure"):
            self.send_ctrl("record_stop", outcome=ev.split("_")[-1])
        elif ev == "record_abort":
            self.send_ctrl("record_abort")
        elif ev == "reset_env":
            self.engaged = False
            self.loco.emergency_stop()
            self.send_ctrl("reset")
        elif ev == "emergency_stop":
            self.engaged = False
            self.loco.emergency_stop()
            print("[hub] EMERGENCY STOP: walking zero, arms disengaged (holding)", flush=True)
        elif ev == "quit":
            self.quit = True

    # ============================================================ main tick
    def tick(self, now: float):
        with self._status_lock:
            st = dict(self.ctx.status)
            st_age = time.monotonic() - self._status_rx if self._status_rx else None
        rc = st.get("reset_count")
        if rc is not None:
            if self.reset_seen is not None and rc != self.reset_seen:
                self.on_sim_reset()
            self.reset_seen = rc
        self.recording = bool(st.get("recording", {}).get("active", False))

        ks, events, nudge = L.KeyState(focused=False), [], [0, 0, 0]
        if self.console is not None:
            ks, events, nudge, q = self.console.poll()
            if q:
                events.append("quit")
        fr = self.source.read(now, ctx=self.ctx) if self.source is not None else None
        ok = bool(fr.ok) if fr is not None else False
        bmap = self.cfg["buttons"]
        if fr is not None and ok:
            rising = self.edges.update(fr)
            inv = {v: k for k, v in bmap.items()}
            for r in rising:
                act = inv.get(r)
                if act == "engage_toggle" or act == "recenter" or act == "emergency_stop":
                    events.append(act)
                elif act == "record_toggle_success":
                    events.append("record_stop_success" if self.recording else "record_start")
                elif act == "record_stop_failure" and self.recording:
                    events.append("record_stop_failure")
        for ev in events:
            self.handle(ev)

        # ---------------- arms
        q_meas = self.arm_ctrl.get_current_dual_arm_q()
        dq_meas = self.arm_ctrl.get_current_dual_arm_dq()
        T_goal = self.fk(self.q_goal)
        clutch = {"left": bool(fr.squeeze[0]) if fr else False, "right": bool(fr.squeeze[1]) if fr else False}
        targets = {}
        for s in SIDES:
            xr_like = self.arm_source in ("pico", "mock", "scripted")
            if self.engaged and xr_like and ok and not clutch[s]:
                T_c = fr.left_wrist if s == "left" else fr.right_wrist
                if self.need_anchor[s]:
                    self.ctx.anchors[s] = (T_goal[s].copy(), T_c.copy())
                    self.need_anchor[s] = False
                T_e0, T_c0 = self.ctx.anchors[s]
                targets[s] = AM.rel_target(T_e0, T_c0, T_c, self.cfg["arm"]["position_scale"],
                                           self.cfg["arm"]["rotation"], self.cfg["arm"]["max_ee_offset_m"])
            elif self.engaged and self.arm_source == "keyboard":
                if self.need_anchor[s]:
                    self.ctx.anchors[s] = (T_goal[s].copy(), None)
                    self.need_anchor[s] = False
                if s == self.sel_side and any(nudge):
                    self.kb_off[s] += 0.01 * np.array(nudge, float)
                    n = np.linalg.norm(self.kb_off[s])
                    if n > self.cfg["arm"]["max_ee_offset_m"]:
                        self.kb_off[s] *= self.cfg["arm"]["max_ee_offset_m"] / n
                T = self.ctx.anchors[s][0].copy()
                T[:3, 3] += self.kb_off[s]
                targets[s] = T
            else:
                if self.engaged and xr_like:
                    self.need_anchor[s] = True  # input lost / clutched: re-anchor on resume (no jump)
        sol_q = None
        if targets:
            TL = targets.get("left", T_goal["left"])
            TR = targets.get("right", T_goal["right"])
            try:
                sol_q, _tau = self.arm_ik.solve_ik(TL, TR, q_meas, dq_meas)
                sol_q = np.asarray(sol_q, float)
                Ts = self.fk(sol_q)
                self.ik_err = [float(np.linalg.norm(Ts[s][:3, 3] - (TL if s == "left" else TR)[:3, 3])) for s in SIDES]
                tracked_err = [e for e, s in zip(self.ik_err, SIDES) if s in targets]
                self.ik_ok = bool(np.all(np.isfinite(sol_q)) and max(tracked_err) < float(self.a.ik_pos_tol))
            except Exception as e:  # noqa: BLE001
                self.ik_ok, sol_q = False, None
                print(f"[hub] IK exception: {e}")
            if self.ik_ok:
                new_goal = self.q_goal.copy()
                for s in targets:
                    new_goal[SL[s]] = sol_q[SL[s]]
                self.q_goal = np.clip(new_goal, self.q_lo, self.q_hi)
            else:
                self.n_ik_fail += 1  # hold_last_valid: q_goal unchanged
        ac = self.cfg["arm"]
        q_prev = self.q_sent.copy()
        # Joint / hand speed limits are meant in SIMULATOR time: scale this tick's wall dt by the measured
        # real-time factor (the sim runs at ~0.35x here; with the wall dt the arm would move ~3x too fast in
        # sim time and in the recorded actions).
        self.dt_sim = self.dt * self._rtf_scale()
        self.q_sent = np.clip(AM.rate_limit(self.q_sent, self.q_goal, float(ac["max_joint_vel_rad_s"]), self.dt_sim),
                              self.q_lo, self.q_hi)
        self.arm_ctrl.ctrl_dual_arm(self.q_sent, np.zeros(14))

        # ---------------- hands
        if self.arm_source in ("pico", "mock", "scripted") and fr is not None:
            since = fr.raw.get("since_change_s")
            self.hands.update_from_triggers(fr.trig[0], fr.trig[1], ok, now=fr.t,
                                            last_pose_change=(fr.t - since) if since is not None else None)
        hm = (self.ctx.status or {}).get("hand_model")
        if hm and hm != self.hands.hand_model:      # simulator's hand model -> its OPEN/CLOSE presets (once)
            info = self.hands.apply_hand_model(hm)
            print(f"[hub] hand model {hm}: presets {json.dumps(info)}", flush=True)
        q12 = self.hands.step(self.dt_sim)
        msg = self.MotorCmds_()
        msg.cmds = [self.MotorCmd_() for _ in range(12)]
        for i in range(12):
            msg.cmds[i].q = float(q12[i])
        self.pub_hand.Write(msg)

        # ---------------- walking
        if self.loco.source == "keyboard":
            self.loco.feed_keys(ks)
        elif self.loco.source == "xr" and fr is not None:
            self.loco.feed_xr(fr.stick[0], fr.stick[1], valid=ok)
        cmd = self.loco.step(self.dt_sim)    # ramps in simulator time
        with self._loco_lock:
            self._loco_cmd = cmd
            self._loco_update_mono = time.monotonic()

        # ---------------- metadata (recorded by the sim next to each frame)
        self.seq += 1
        meta = {"seq": self.seq, "wall_time": time.time(), "arm_source": self.arm_source, "loco_source": self.loco_source,
                "input_ok": ok, "engaged": self.engaged, "clutch": [clutch["left"], clutch["right"]],
                "hand_cmd": self.hands.cmds(), "hand_q_sent": q12.tolist(),
                "trigger": list(fr.trig) if fr else None, "arm_ik_q": (sol_q.tolist() if sol_q is not None else None),
                "arm_goal_q": self.q_goal.tolist(), "arm_sent_q": self.q_sent.tolist(), "ik_ok": self.ik_ok,
                "ik_err": self.ik_err, "targets": {s: targets[s][:3, 3].tolist() for s in targets},
                "loco_raw": self.loco.raw, "loco_sent": cmd, "loco_reason": self.loco.reason, "events": events,
                "phase": (fr.raw.get("phase") if fr else None)}
        self.pub_meta.Write(self.String_(data=topics.dumps(meta)))
        self.trace.append({"t": now, "ok": ok, "engaged": self.engaged, "q_sent": self.q_sent.tolist(),
                           "q_meas": q_meas.tolist(), "dq_sent_max": float(np.max(np.abs(self.q_sent - q_prev))), "dt_sim": self.dt_sim,
                           "targets": meta["targets"], "ik_ok": self.ik_ok, "ik_err": self.ik_err,
                           "hand_cmd": meta["hand_cmd"], "hand_q": q12.tolist(), "loco": cmd, "loco_reason": self.loco.reason,
                           "events": events, "phase": (fr.raw.get("phase") if fr else None),
                           "scenario_t": (fr.raw.get("scenario_t") if fr else None), "status_age_s": st_age,
                           "base_lin_vel_b": st.get("base_lin_vel_b"), "loco_applied": st.get("loco_cmd_applied"),
                           "fallen": st.get("fallen"), "sim_time": st.get("sim_time")})
        if self.console is not None and self.seq % 5 == 0:
            self.console.draw(self._lines(st, st_age, fr, ok, ks))
        if self.a.diag and fr is not None and self.seq % 10 == 0:
            print("[diag]", json.dumps({"ok": ok, "trig": fr.trig, "squeeze": fr.squeeze, "a": fr.a, "b": fr.b,
                                        "stick": [fr.stick[0].round(2).tolist(), fr.stick[1].round(2).tolist()],
                                        "stick_click": fr.stick_click, **{k: v for k, v in fr.raw.items() if k != "frozen"}},
                                       default=str), flush=True)

    def _lines(self, st, st_age, fr, ok, ks):
        g = (90, 220, 120)
        r = (240, 90, 80)
        y = (240, 200, 80)
        rec = st.get("recording", {})
        lines = [f"#G1IV hub  arm={self.arm_source}  loco={self.loco_source}",
                 (f"sim: t={st.get('sim_time', 0):.2f}s rtf={st.get('rtf') or 0:.2f} status_age={st_age or -1:.2f}s"
                  f" fallen={st.get('fallen')}", g if st_age is not None and st_age < 0.5 else r),
                 (f"input: {'OK' if ok else ('LOST' if fr is not None else 'n/a')}"
                  f"  engaged={self.engaged}  sel_arm={self.sel_side}", g if ok or fr is None else r),
                 f"hands: L={self.hands.left.cmd.name} R={self.hands.right.cmd.name}   trig={tuple(round(x, 2) for x in fr.trig) if fr else '-'}",
                 f"ik_ok={self.ik_ok} err={[round(e, 3) for e in self.ik_err]} fails={self.n_ik_fail}",
                 f"walk cmd: {[round(x, 2) for x in self._loco_cmd]}  ({self.loco.reason})  window_focus={ks.focused}",
                 f"applied: {[round(x, 2) for x in (st.get('loco_cmd_applied') or [0, 0, 0, 0])]}  base_v={[round(x, 2) for x in (st.get('base_lin_vel_b') or [0, 0, 0])]}",
                 (f"REC {'ON' if rec.get('active') else 'off'} frames={rec.get('frames', 0)} dropped={rec.get('dropped', 0)}"
                  f" {rec.get('episode_id') or ''}", r if rec.get("active") else (160, 160, 160)),
                 "",
                 "keys (this window only): W/S A/D Q/E walk, SPACE stop | R engage, C recenter",
                 "Z/X/B hand L/R/both toggle | TAB arm, I/K J/L U/O nudge 1 cm",
                 "1 rec start, 2 stop=success, 3 stop=failure, 4 abort | 0 reset env | ESC quit"]
        if not ok and fr is not None and self.engaged:
            lines.append(("INPUT LOST: arms hold last target, hands hold state, walking zero", y))
        return lines

    # ============================================================ run / shutdown
    def run(self):
        t_next = time.monotonic()
        t_end = time.monotonic() + self.a.duration if self.a.duration else None
        if self.a.reset_first:
            self.send_ctrl("reset")
            time.sleep(1.0)
        rec_started = False
        try:
            while not self.quit:
                now = time.monotonic()
                self.tick(now)
                # record from engagement on, or from the moment the source says so (scripted walk: from the
                # first walking command, the walk is part of the episode)
                if self.a.record and not rec_started and (self.engaged or getattr(self.source, "record_ready", False)):
                    self.handle("record_start")
                    rec_started = True
                src_done = getattr(self.source, "done", False)
                if (t_end and now >= t_end) or (self.a.exit_when_done and src_done):
                    break
                t_next += self.dt
                sl = t_next - time.monotonic()
                if sl > 0:
                    time.sleep(sl)
                else:
                    t_next = time.monotonic()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown(rec_started)

    def shutdown(self, rec_started):
        result = getattr(self.source, "result", {}) if self.source is not None else {}
        if rec_started:
            outcome = "undetermined"
            if self.arm_source == "scripted" and "success" in result:
                outcome = "success" if result["success"] else "failure"
            self.send_ctrl("record_stop", outcome=outcome, reason="hub_exit")
        with self._loco_lock:
            self._loco_cmd = [0.0, 0.0, 0.0, self.loco.height]
            self._loco_update_mono = time.monotonic()
        time.sleep(0.3)
        self.quit = True
        out = ws_path("logs/teleop_runs")
        out.mkdir(parents=True, exist_ok=True)
        p = out / f"{self.run_id}.json"
        p.write_text(json.dumps({"run_id": self.run_id, "args": vars(self.a), "arm_source": self.arm_source,
                                 "loco_source": self.loco_source, "n_ik_fail": self.n_ik_fail, "result": result,
                                 "source_log": getattr(self.source, "log", None), "events": self.events_log,
                                 "trace": self.trace}, default=str))
        print(f"[hub] trace -> {p}", flush=True)
        if self.source is not None:
            self.source.close()
        if self.console is not None:
            self.console.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/teleop.yaml")
    ap.add_argument("--set", action="append", default=[], help="config override key=value (repeatable)")
    ap.add_argument("--arm-source", choices=["pico", "mock", "scripted", "keyboard", "none"])
    ap.add_argument("--loco-source", choices=["keyboard", "xr", "none"])
    ap.add_argument("--scenario", help="mock scenario (dry_run|dropout|recenter|walk_xr|hold)")
    ap.add_argument("--plan", default="configs/scripted_grasp.yaml")
    ap.add_argument("--plan-set", action="append", default=[],
                    help="scripted plan override key=value (repeatable), e.g. expect_target=blue")
    ap.add_argument("--dds-iface", default=os.environ.get("G1IV_DDS_IFACE", "lo"))
    ap.add_argument("--no-console", action="store_true")
    ap.add_argument("--diag", action="store_true", help="print raw XR buttons/triggers/sticks (PICO mapping check)")
    ap.add_argument("--record", action="store_true", help="start recording once engaged; stop on exit")
    ap.add_argument("--instruction", default="pick up the red cylinder")
    ap.add_argument("--task-name", default="pick_cylinder")
    ap.add_argument("--operator", default=os.environ.get("USER", ""))
    ap.add_argument("--reset-first", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--exit-when-done", action="store_true")
    ap.add_argument("--ik-pos-tol", type=float, default=0.03)
    a = ap.parse_args()
    os.chdir(WS)
    hub = Hub(a)
    hub.run()
    os._exit(0)


if __name__ == "__main__":
    main()
