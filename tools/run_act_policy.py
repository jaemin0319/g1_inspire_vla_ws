"""Roll out a learned ACT policy in the simulator (replaces the teleop hub as the command owner).

    scripts/run_act_eval.sh ...        (launcher: env, policy server, this runner; see its header)

Needs: the simulator started with G1IV_POLICY_BRIDGE=<socket> (scripts/run_act_eval.sh sim ...), the policy server
(tools/act_policy_server.py) and NO teleop hub (two owners of rt/lowcmd would fight).

Per episode: reset (robot pose / object xy) -> hold still for --settle-s of simulator time -> policy reset ->
record_start (source "policy") -> lockstep loop on the recorder's frames (25 Hz simulator time):
  simulator observation (4 cameras + arm_q + hand_q_norm) -> policy server -> action[29]
  -> arms: official G1_29_ArmController (rt/lowcmd), rate limit as the hub (arm.max_joint_vel_rad_s)
  -> hands: rt/inspire/cmd (DFX order, 0..1)          -> walking: rt/run_command/cmd [vx, vy, wz, 0.8] (200 Hz)
  -> reply to the simulator, which then applies them during the paired step (same frame semantics as the demos).
Stop: object lifted >= --lift-m for --hold-frames frames (success), robot fallen or --max-frames (failure).
The episode is saved by the simulator recorder with that outcome; one JSON line per episode goes to <out>/summary.jsonl
and a per-frame trace to <out>/ep<i>_trace.npz.
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

WS = Path(__file__).resolve().parents[1]
XR_ROOT = WS / "repos" / "xr_teleoperate"
sys.path.insert(0, str(WS / "src"))
from g1iv_common import arm_math as AM  # noqa: E402
from g1iv_common import framing, topics  # noqa: E402
from g1iv_common.config import load_config  # noqa: E402

CAM_KEYS = {"head": "cam_left_high", "head_right": "cam_right_high", "left_wrist": "cam_left_wrist",
            "right_wrist": "cam_right_wrist"}
HAND_START = [1.0, 1.0, 1.0, 1.0, 1.0, 0.45] * 2   # DFX (R 0..5, L 6..11): every demo starts here (thumb pre-rotated)
HEIGHT = 0.8


def pack_state(arm_q, hand_q_norm_dfx) -> np.ndarray:
    """Recording layout -> dataset observation.state: [L arm 7, R arm 7, L hand 6, R hand 6] (convert_lerobot.pack26)."""
    h = np.asarray(hand_q_norm_dfx, np.float32)
    return np.concatenate([np.asarray(arm_q, np.float32), h[6:12], h[0:6]]).astype(np.float32)


def unpack_action(a: np.ndarray):
    a = np.asarray(a, float)
    return a[:14].copy(), np.concatenate([a[20:26], a[14:20]]), a[26:29].copy()   # arm14, hand DFX12, loco3


class Commands:
    """Latest hand / walking command, republished from a thread (the sim zeroes the walking command on every read)."""

    def __init__(self, pub_run, pub_hand, String_, MotorCmds_, MotorCmd_, hz: float):
        self.pub_run, self.pub_hand = pub_run, pub_hand
        self.String_, self.MotorCmds_, self.MotorCmd_ = String_, MotorCmds_, MotorCmd_
        self.lock = threading.Lock()
        self.loco = [0.0, 0.0, 0.0]
        self.hand = list(HAND_START)
        self.hz = hz
        self.quit = False
        threading.Thread(target=self._run, daemon=True).start()

    def set(self, loco=None, hand=None, publish_hand_now=True):
        with self.lock:
            if loco is not None:
                self.loco = [float(x) for x in loco]
            if hand is not None:
                self.hand = [float(x) for x in hand]
        if hand is not None and publish_hand_now:
            self._pub_hand()

    def _pub_hand(self):
        with self.lock:
            q = list(self.hand)
        msg = self.MotorCmds_()
        msg.cmds = [self.MotorCmd_() for _ in range(12)]
        for i in range(12):
            msg.cmds[i].q = q[i]
        self.pub_hand.Write(msg)

    def _run(self):
        k = 0
        while not self.quit:
            with self.lock:
                v = list(self.loco)
            self.pub_run.Write(self.String_(data=topics.run_command_str(v[0], v[1], v[2], HEIGHT)))
            k += 1
            if k % 4 == 0:
                self._pub_hand()
            time.sleep(1.0 / self.hz)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", type=int, default=5, help="episodes")
    ap.add_argument("--seed", type=int, default=20261118)
    ap.add_argument("--mode", choices=["walk", "far", "near"], default="walk",
                    help="walk: random start like walk_grasp_desk065_scripted_v1 (robot 0.6-0.9 m behind the stand "
                         "pose); far: like walk_far_grasp_desk065_scripted_v2 (robot x -1.0..-0.4, y -0.3..0.3 = "
                         "1.1-1.8 m behind it); near: robot reset at the scripted stand pose (grasp part only)")
    ap.add_argument("--robot-x", nargs=2, type=float, default=None, help="walk: -0.10 0.10, far: -1.0 -0.4")
    ap.add_argument("--robot-y", nargs=2, type=float, default=None, help="walk: -0.20 0.20, far: -0.30 0.30")
    ap.add_argument("--robot-yaw", nargs=2, type=float, default=[-15.0, 15.0])
    ap.add_argument("--object-x", nargs=2, type=float, default=[0.98, 1.08])
    ap.add_argument("--object-y", nargs=2, type=float, default=[-0.30, -0.10])
    ap.add_argument("--stand-rel", nargs=2, type=float, default=[0.31, -0.10],
                    help="near mode: object in the robot frame after settling (scripted_walk_grasp_desk065.yaml)")
    ap.add_argument("--reset-slide", nargs=2, type=float, default=[0.051, 0.018],
                    help="near mode: measured drift right after a reset (docs/AUTO_WALK_GRASP.md)")
    ap.add_argument("--settle-s", type=float, default=2.0, help="simulator seconds to stand still after a reset")
    ap.add_argument("--max-frames", type=int, default=1100, help="25 Hz frames (demos: 482-889)")
    ap.add_argument("--lift-m", type=float, default=0.05)
    ap.add_argument("--hold-frames", type=int, default=25)
    ap.add_argument("--loco-deadzone", type=float, default=0.0, help="|cmd| below this -> 0 (per axis)")
    ap.add_argument("--instruction", default="walk to the desk and pick up the red cylinder")
    ap.add_argument("--task-name", default="walk_grasp_desk065_policy_eval")
    ap.add_argument("--label", default="act", help="operator/notes tag in the recording (e.g. the checkpoint)")
    ap.add_argument("--bridge", default=str(WS / "logs/run/policy_bridge.sock"))
    ap.add_argument("--policy", default=str(WS / "logs/run/act_policy.sock"))
    ap.add_argument("--out", default=None, help="default logs/policy_eval/<time>_<label>_<mode>")
    ap.add_argument("--dds-iface", default=os.environ.get("G1IV_DDS_IFACE", "lo"))
    a = ap.parse_args()
    if a.robot_x is None:
        a.robot_x = [-1.0, -0.4] if a.mode == "far" else [-0.10, 0.10]
    if a.robot_y is None:
        a.robot_y = [-0.30, 0.30] if a.mode == "far" else [-0.20, 0.20]

    cfg = load_config(WS / "configs/teleop.yaml")
    max_vel = float(cfg["arm"]["max_joint_vel_rad_s"])
    home_q = np.array(cfg["arm"]["home_q"]["left"] + cfg["arm"]["home_q"]["right"], float)
    frame_dt = 0.04
    out = Path(a.out) if a.out else WS / "logs/policy_eval" / f"{time.strftime('%Y%m%d_%H%M%S')}_{a.label}_{a.mode}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "args.json").write_text(json.dumps(vars(a), indent=1))

    # ---------------- policy server + simulator bridge (fail early, before touching the robot)
    pol = framing.connect_unix(a.policy, 120.0)
    framing.send_msg(pol, {"cmd": "info"})
    info, _ = framing.recv_msg(pol)
    print(f"[runner] policy: {json.dumps({k: info[k] for k in ('checkpoint', 'n_action_steps', 'temporal_ensemble_coeff', 'jpeg', 'gpu')})}",
          flush=True)
    (out / "policy_info.json").write_text(json.dumps(info, indent=1))
    bridge = framing.connect_unix(a.bridge, 60.0)

    # ---------------- DDS (same topics and official arm controller as the hub)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_

    ChannelFactoryInitialize(topics.SIM_DDS_DOMAIN, a.dds_iface or None)
    pub_run = ChannelPublisher(topics.T_RUN_COMMAND, String_); pub_run.Init()
    pub_hand = ChannelPublisher(topics.T_INSPIRE_CMD, MotorCmds_); pub_hand.Init()
    pub_ctrl = ChannelPublisher(topics.T_CONTROL, String_); pub_ctrl.Init()
    status = {"st": {}, "rx": 0.0}
    lock = threading.Lock()

    def on_status(msg):
        try:
            st = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with lock:
            status["st"], status["rx"] = st, time.monotonic()

    sub = ChannelSubscriber(topics.T_SIM_STATUS, String_); sub.Init(on_status, 8)

    def st_now() -> dict:
        with lock:
            return dict(status["st"])

    def ctrl(cmd, **kw):
        pub_ctrl.Write(String_(data=topics.control_msg(cmd, **kw)))

    print("[runner] waiting for the simulator (rt/lowstate) ...", flush=True)
    sys.path.insert(0, str(XR_ROOT))
    prev = os.getcwd()
    os.chdir(XR_ROOT / "teleop")
    try:
        from teleop.robot_control.robot_arm import G1_29_ArmController

        class _HoldStartArmController(G1_29_ArmController):
            """Hold the official 250 Hz publish thread until q_target = measured pose (see hub.py)."""

            def _ctrl_motor_state(self):
                while not getattr(self, "_g1iv_release", False):
                    time.sleep(0.002)
                return super()._ctrl_motor_state()

        arm = _HoldStartArmController(motion_mode=False, simulation_mode=True)
    finally:
        os.chdir(prev)

    def hold_arms_here() -> np.ndarray:
        q = np.asarray(arm.get_current_dual_arm_q(), float)
        with arm.ctrl_lock:
            arm.q_target = q.copy()
            arm.tauff_target = np.zeros(14)
        return q

    q_sent = hold_arms_here()
    arm._g1iv_release = True
    cmds = Commands(pub_run, pub_hand, String_, MotorCmds_, unitree_go_msg_dds__MotorCmd_,
                    float(cfg["loco"].get("publish_hz", 200.0)))
    t_wait = time.time() + 20
    while not st_now() and time.time() < t_wait:
        time.sleep(0.1)
    if not st_now():
        print("[runner] no rt/g1iv/sim_status: is the simulator running?", file=sys.stderr)
        return 2
    if st_now().get("recording", {}).get("active"):
        ctrl("record_abort", reason="policy runner start")
        time.sleep(1.0)

    rng = np.random.default_rng(a.seed)
    n_ok = 0
    for i in range(a.n):
        oxy = [float(rng.uniform(*a.object_x)), float(rng.uniform(*a.object_y))]
        if a.mode in ("walk", "far"):
            rp = [float(rng.uniform(*a.robot_x)), float(rng.uniform(*a.robot_y)), float(rng.uniform(*a.robot_yaw))]
        else:
            rp = [oxy[0] - a.stand_rel[0] - a.reset_slide[0], oxy[1] - a.stand_rel[1] - a.reset_slide[1], 0.0]
        # ---- reset and settle (arms held at the measured pose, hands at the demo start, no walking)
        cmds.set(loco=[0, 0, 0], hand=HAND_START)
        with arm.ctrl_lock:   # the reset puts the arms at the default pose: do not pull them back to the old target
            arm.q_target = home_q.copy()
        rc0 = st_now().get("reset_count")
        ctrl("reset", robot_pose=rp, object_xy=oxy)
        t_wait = time.time() + 30
        while st_now().get("reset_count") == rc0 and time.time() < t_wait:
            time.sleep(0.02)
        if st_now().get("reset_count") == rc0:
            print("[runner] reset not acknowledged by the simulator", file=sys.stderr)
            return 3
        q_sent = hold_arms_here()
        t0 = st_now().get("sim_time", 0.0)
        while st_now().get("sim_time", 0.0) - t0 < a.settle_s:
            time.sleep(0.05)
            q_sent = hold_arms_here()
        framing.send_msg(pol, {"cmd": "reset"})
        framing.recv_msg(pol)
        # ---- episode
        ctrl("record_start", instruction=a.instruction, task_name=a.task_name, source="policy",
             operator=a.label, notes=f"{info['checkpoint']} mode={a.mode} n_action_steps={info['n_action_steps']} "
                                     f"te={info['temporal_ensemble_coeff']}", arm_source="policy", loco_source="policy")
        trace = {k: [] for k in ("sim_time", "action", "loco_sent", "lift", "base_pos", "infer_ms")}
        outcome, reason, held, max_lift, fallen, ep_id = None, "", 0, -1.0, False, None
        wall0 = time.time()
        k = 0
        while True:
            try:
                meta, imgs = framing.recv_msg(bridge)
            except (TimeoutError, OSError) as e:
                print(f"[runner] no frame from the simulator ({e!r}); is it recording?", file=sys.stderr)
                return 4
            ep_id = meta.get("episode_id")
            arr = {"observation.state": pack_state(meta["arm_q"], meta["hand_q_norm"])}
            for alias, key in CAM_KEYS.items():
                arr[f"observation.images.{key}"] = imgs[alias]
            framing.send_msg(pol, {"cmd": "act"}, arr)
            pm, pa = framing.recv_msg(pol)
            if not pm.get("ok"):
                print(f"[runner] policy error: {pm}", file=sys.stderr)
                return 5
            act = pa["action"]
            arm_goal, hand_dfx, loco = unpack_action(act)
            q_sent = AM.rate_limit(q_sent, arm_goal, max_vel, frame_dt)
            arm.ctrl_dual_arm(q_sent, np.zeros(14))
            loco_sent = [0.0 if abs(v) < a.loco_deadzone else float(v) for v in loco]
            cmds.set(loco=loco_sent, hand=np.clip(hand_dfx, 0.0, 1.0))
            ev = meta.get("eval") or {}
            lift = ev.get("object_lift_m")
            lift = float(lift) if lift is not None else float("nan")
            max_lift = max(max_lift, lift) if np.isfinite(lift) else max_lift
            fallen = bool(ev.get("fallen"))
            held = held + 1 if lift >= a.lift_m else 0
            for kk, v in (("sim_time", meta["sim_time"]), ("action", act), ("loco_sent", loco_sent), ("lift", lift),
                          ("base_pos", meta.get("base_pos_w")), ("infer_ms", pm.get("infer_ms"))):
                trace[kk].append(v)
            k += 1
            if fallen:
                outcome, reason = "failure", "fallen"
            elif held >= a.hold_frames:
                outcome, reason = "success", f"lift>={a.lift_m} for {a.hold_frames} frames"
            elif k >= a.max_frames:
                outcome, reason = "failure", "timeout"
            if k % 50 == 0:
                bp = meta.get("base_pos_w") or [0, 0, 0]
                print(f"[runner] ep {i} frame {k} sim {meta['sim_time']:.1f}s base ({bp[0]:.2f},{bp[1]:.2f}) "
                      f"loco {np.round(loco, 2).tolist()} lift {lift:.3f} infer {pm.get('infer_ms', 0):.0f} ms", flush=True)
            time.sleep(0.015)    # let rt/lowcmd (250 Hz), rt/inspire/cmd and rt/run_command reach the simulator
            framing.send_msg(bridge, {"stop": outcome, "reason": reason} if outcome else {})
            if outcome:
                break
        cmds.set(loco=[0, 0, 0])
        n_ok += outcome == "success"
        row = {"i": i, "mode": a.mode, "robot_pose": rp, "object_xy": oxy, "outcome": outcome, "reason": reason,
               "frames": k, "max_lift_m": round(max_lift, 4), "fallen": fallen, "episode_id": ep_id,
               "walk_cmd_frames": int(sum(1 for v in trace["loco_sent"] if any(abs(x) > 1e-3 for x in v))),
               "base_travel_m": float(np.linalg.norm(np.asarray(trace["base_pos"][-1][:2]) - np.asarray(trace["base_pos"][0][:2])))
               if trace["base_pos"] and trace["base_pos"][0] else None,
               "wall_s": round(time.time() - wall0, 1), "infer_ms_median": float(np.median([x for x in trace["infer_ms"] if x is not None]))}
        with open(out / "summary.jsonl", "a") as f:
            f.write(json.dumps(row) + "\n")
        np.savez_compressed(out / f"ep{i:02d}_trace.npz", sim_time=np.asarray(trace["sim_time"]),
                            action=np.asarray(trace["action"]), loco_sent=np.asarray(trace["loco_sent"]),
                            lift=np.asarray(trace["lift"]), base_pos=np.asarray(trace["base_pos"], float))
        print(f"[runner] episode {i}: {outcome} ({reason}) frames {k} max lift {max_lift:.3f} m "
              f"walk-cmd frames {row['walk_cmd_frames']} travel {row['base_travel_m']} m  -> {ep_id}", flush=True)
        time.sleep(1.0)   # let the recorder close the episode
    cmds.quit = True
    print(f"[runner] {n_ok}/{a.n} success -> {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
