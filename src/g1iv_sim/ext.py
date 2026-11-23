"""g1_inspire_vla_ws extension for the official unitree_sim_isaaclab sim_main.py.

Loaded only when the env var G1IV_EXT=1 is set (see the project patch on sim_main.py). Hooks:
  on_env_created(env, env_cfg, args)          after gym.make
  on_controller_ready(env, controller, provider, args)   after DDS objects + action provider exist
  on_step(env)                                 after every controller.step() (= one 20 ms policy step)
  on_shutdown()

What it adds (the official control path is not replaced):
  * project task registration (src/g1iv_sim/desk_task.py)
  * rt/g1iv/control  : record_start / record_stop / record_abort / reset / mark
  * rt/g1iv/sim_status: sim time, base state, applied walking command, recording + reset state
  * sim-side recording of (observation_t, action_t) pairs on the simulation-time grid
  * full reset (robot + object + policy history + command buffers + recorder)
"""
from __future__ import annotations

import ast
import json
import math
import os
import platform
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import torch

from g1iv_common import topics
from g1iv_common.config import WS, load_config, ws_path
from g1iv_common import hand_model as HM
from g1iv_common.hands import CHANNELS, norm_to_rad, preset_vector

from . import desk_task as g1iv_tasks  # noqa: F401  (registers the project task id)
from .observer_video import ObserverVideo
from .policy_bridge import PolicyBridge
from .recorder import EpisodeRecorder, FramePairer

# Diagnostics: "kill -USR1 <sim pid>" prints every thread's Python stack to the sim log (for hangs).
try:
    import faulthandler
    import signal as _signal
    import sys as _sys

    faulthandler.register(_signal.SIGUSR1, file=_sys.stderr, all_threads=True)
except Exception:  # noqa: BLE001
    pass

ARM_NAMES = [f"{s}_{j}_joint" for s in ("left", "right") for j in
             ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")]
LEG_NAMES = [f"{s}_{j}_joint" for s in ("left", "right") for j in
             ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
WAIST_NAMES = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
# Inspire DFX 12-channel order (R 0..5, L 6..11) -> actuated USD joints
_HJ = {"pinky": "pinky_proximal_joint", "ring": "ring_proximal_joint", "middle": "middle_proximal_joint",
       "index": "index_proximal_joint", "thumb_bend": "thumb_proximal_pitch_joint", "thumb_rot": "thumb_proximal_yaw_joint"}
HAND_NAMES = [f"{s}_{_HJ[c]}" for s in ("R", "L") for c in CHANNELS]
HAND_ALL_NAMES = [f"{s}_{j}" for s in ("R", "L") for j in (
    "pinky_proximal_joint", "ring_proximal_joint", "middle_proximal_joint", "index_proximal_joint",
    "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint", "pinky_intermediate_joint", "ring_intermediate_joint",
    "middle_intermediate_joint", "index_intermediate_joint", "thumb_intermediate_joint", "thumb_distal_joint")]
# SDK (unitree hg) motor order: legs L/R(6+6), waist(3), arms L/R(7+7)
SDK29_NAMES = ([f"left_{j}_joint" for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
               + [f"right_{j}_joint" for j in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
               + WAIST_NAMES + ARM_NAMES)


def _rad_to_norm(names: list[str], rad: np.ndarray) -> np.ndarray:
    """Inverse of the sim's denormalisation (same formula the sim uses to publish rt/inspire/state), for the active
    hand model (g1iv_common.hand_model; desk_task activates it from configs/scene_desk.yaml robot.hand_model)."""
    return HM.rad_to_norm(g1iv_tasks.HAND, names, rad)


def _git(path: Path) -> dict:
    def run(*a):
        try:
            return subprocess.run(["git", "-C", str(path), *a], capture_output=True, text=True, timeout=5).stdout.strip()
        except Exception:  # noqa: BLE001
            return ""
    return {"commit": run("rev-parse", "HEAD"), "branch": run("rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(run("status", "--porcelain", "--untracked-files=no"))}


def _quat_yaw(q_wxyz) -> float:
    w, x, y, z = q_wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class G1IVExt:
    def __init__(self):
        self.env = None
        self.provider = None
        self.step = 0
        self.reset_count = 0
        self.episode_index = 0
        self._ctrl_q: deque = deque()
        self._meta_latest: dict | None = None
        self._meta_rx_mono = 0.0
        self._meta_lock = threading.Lock()
        self._last_run_cmd = [0.0, 0.0, 0.0, 0.8]
        self._pending_obs = None
        self._wall_hist: deque = deque(maxlen=100)
        self._last_wall = None
        self._t0_wall = time.time()
        self._marks: list = []
        self._episode_extra: dict = {}
        self.pairer = None
        self.bridge = None
        self.obs_video = None

    # ================================================================== hooks
    def on_env_created(self, env, env_cfg, args):
        self.env, self.env_cfg, self.args = env, env_cfg, args
        ov = json.loads(os.environ.get("G1IV_RECORD_OVERRIDES", "[]") or "[]")
        self.rcfg = load_config(os.environ.get("G1IV_RECORD_CFG", "configs/record.yaml"), ov)
        self.hcfg = load_config("configs/hands_inspire.yaml")
        self.scn = g1iv_tasks.SCN
        self.robot = env.scene["robot"]
        # graspable cylinders: colour name -> scene entity (scene_desk.yaml object: + extra_objects:); the TARGET (chosen
        # per reset, default = the first) is what eval object_pos_w / object_lift_m / contact_force_total describe
        keys = env.scene.keys()
        self.obj_entities = {n: k for n, k in g1iv_tasks.OBJECT_ENTITIES.items() if k in keys}
        self.objs = {n: env.scene[k] for n, k in self.obj_entities.items()}
        self.target = next(iter(self.objs), None)
        self.obj = self.objs.get(self.target)
        try:
            self._apply_hand_appearance()
        except Exception as e:  # noqa: BLE001  (cosmetic: never block the simulator)
            print(f"[g1iv] hand appearance not applied: {e}")
        self.lens_status = self._apply_lens_distortion(env)
        names = list(self.robot.data.joint_names)
        self.joint_names = names
        idx = {n: i for i, n in enumerate(names)}
        missing = [n for n in ARM_NAMES + LEG_NAMES + WAIST_NAMES + HAND_ALL_NAMES if n not in idx]
        if missing:
            raise RuntimeError(f"[g1iv] robot joints missing (wrong asset?): {missing}")
        self.i_arm = [idx[n] for n in ARM_NAMES]
        self.i_leg = [idx[n] for n in LEG_NAMES]
        self.i_waist = [idx[n] for n in WAIST_NAMES]
        self.i_hand = [idx[n] for n in HAND_NAMES]
        self.i_hand_all = [idx[n] for n in HAND_ALL_NAMES]
        self.i_sdk29 = [idx[n] for n in SDK29_NAMES]
        # bodies reported as evaluation ground truth only (never a policy observation)
        bnames = list(self.robot.data.body_names)
        self.i_body_eval = {k: bnames.index(n) for k, n in (("right_hand", "right_hand_base_link"),
                                                             ("left_hand", "left_hand_base_link"),
                                                             ("right_wrist_yaw", "right_wrist_yaw_link"),
                                                             ("left_wrist_yaw", "left_wrist_yaw_link")) if n in bnames}
        self.cams = [c for c in self.rcfg["cameras"] if c in env.scene.keys()]
        self.alias = self.rcfg["camera_aliases"]
        self.control_dt = float(env_cfg.sim.dt) * int(env_cfg.decimation)
        self.pairer = FramePairer(int(self.rcfg["record_every_n_steps"]))
        self.rec = EpisodeRecorder(ws_path(self.rcfg["raw_root"]), ws_path(self.rcfg["smoke_root"]),
                                   self.rcfg["smoke_sources"], self.rcfg["jpeg_quality"], self.rcfg["writer_queue_max"])
        if os.environ.get("G1IV_POLICY_BRIDGE"):  # learned-policy rollouts (tools/run_act_policy.py), lockstep
            self.bridge = PolicyBridge(os.environ["G1IV_POLICY_BRIDGE"],
                                       float(os.environ.get("G1IV_POLICY_BRIDGE_TIMEOUT", "60")))
        if os.environ.get("G1IV_OBSERVER_VIDEO"):  # observer-camera video per recorded episode (viewing only)
            if "world_camera" in env.scene.keys():
                ob = self.scn.get("cameras", {}).get("observer", {})
                self.obs_video = ObserverVideo(
                    ws_path(os.path.expanduser(os.environ["G1IV_OBSERVER_VIDEO"])),
                    1.0 / (self.control_dt * int(self.rcfg["record_every_n_steps"])),
                    camera_info={k: ob.get(k) for k in ("pos", "look_at", "width", "height", "focal_length")})
                print(f"[g1iv] observer video on -> {self.obs_video.dir}")
            else:
                print("[g1iv] G1IV_OBSERVER_VIDEO set but there is no world_camera sensor (demo mode?): no video")
        self.static_meta = self._static_meta()
        (WS / "logs").mkdir(exist_ok=True)
        (WS / "logs" / "sim_static_meta.json").write_text(json.dumps(self.static_meta, indent=1))
        self._panel = None
        try:
            self._panel = self._setup_camera_panel(args)
        except Exception as e:  # noqa: BLE001  (viewer convenience: never block the simulator)
            print(f"[g1iv] camera panel not created: {e}")
        print(f"[g1iv] ext ready: task={args.task} joints={len(names)} cams={self.cams} control_dt={self.control_dt}")

    def on_controller_ready(self, env, controller, provider, args):
        self.provider = provider
        # hand model ranges / couplings into the official DDS + action code (no-op for the official DFX hand)
        self._throttle_render(env)
        self._pace = bool((self.scn.get("demo") or {}).get("realtime_pacing", False))
        self._t_next = None
        if self._pace:
            print("[g1iv] demo mode: real-time pacing on a cumulative 20 ms deadline", flush=True)
        self.hand_patch = HM.patch_official_inspire(g1iv_tasks.HAND, provider, getattr(provider, "inspire_dds", None))
        print(f"[g1iv] hand model {g1iv_tasks.HAND['name']}: {json.dumps(self.hand_patch)}", flush=True)
        if g1iv_tasks.HAND["name"] != "inspire_dfx_official" and getattr(provider, "inspire_dds", None) is not None \
                and "couplings" not in self.hand_patch:
            raise RuntimeError("[g1iv] RH56E2 hand: official action provider layout changed, couplings not applied")
        rc = getattr(provider, "run_command_dds", None)
        if rc is not None:  # capture the walking command the policy actually consumes
            orig = rc.get_run_command

            def _wrapped():
                d = orig()
                self._last_run_cmd = self._parse_run_cmd(d)
                return d
            rc.get_run_command = _wrapped
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        self.String_ = String_
        self.pub_status = ChannelPublisher(topics.T_SIM_STATUS, String_)
        self.pub_status.Init()
        self.sub_ctrl = ChannelSubscriber(topics.T_CONTROL, String_)
        self.sub_ctrl.Init(lambda m: self._ctrl_q.append(m.data), 32)
        self.sub_meta = ChannelSubscriber(topics.T_TELEOP_META, String_)
        self.sub_meta.Init(self._on_meta, 32)
        # observer camera: slow updates only (preview), or every recorded frame with G1IV_OBSERVER_VIDEO;
        # policy cameras untouched
        if "world_camera" in env.scene.keys():
            per = self.control_dt * int(self.rcfg["record_every_n_steps"]) if self.obs_video is not None else 0.5
            try:
                env.scene["world_camera"].cfg.update_period = per
                env.scene["world_camera"]._update_period = per  # sim_main set it to 1e6 when excluded
            except Exception as e:  # noqa: BLE001
                print(f"[g1iv] observer camera period not set: {e}")
        # Deterministic start: the official provider uses a 0 arm target until the first rt/lowcmd arrives
        # (and stale /dev/shm content from a previous run otherwise). Seed default arm pose / OPEN / zero walk.
        self._reset_cmd_shm(provider, self.robot.data.default_joint_pos[0])
        print("[g1iv] DDS project topics up:", topics.T_CONTROL, topics.T_TELEOP_META, topics.T_SIM_STATUS)

    def on_step(self, env):
        self.step += 1
        if _PROF is not None:
            _PROF.on_step(self.step)
        if getattr(self, "_pace", False):
            self._pace_step()
        now = time.time()
        if self._last_wall is not None:
            self._wall_hist.append(now - self._last_wall)
        self._last_wall = now
        while self._ctrl_q:
            self._handle_ctrl(self._ctrl_q.popleft())
        if self._panel is not None and self.step % self._panel["every"] == 0:
            self._update_camera_panel()
        need_status = (self.step % int(self.rcfg["status_every_n_steps"])) == 0
        if not (need_status or self.rec.active):
            return
        st = self._read_state()
        if self.rec.active:
            s = self.step
            # frame for action step s: pair the observation taken just BEFORE step s with the
            # targets that were applied DURING step s
            emit, capture = self.pairer.on_step(s)
            if emit and self._pending_obs is not None:
                obs, imgs, deps, t_obs = self._pending_obs
                row = {"step": s, "sim_time": t_obs["sim_time"], "wall_time": t_obs["wall_time"],
                       "obs": obs, "action": self._read_action(), "teleop": self._meta_snapshot(), "eval": t_obs["eval"]}
                self.rec.add_frame(row, imgs, deps)
                self._pending_obs = None
            if capture:  # next step is a record step -> observe now
                self._pending_obs = self._capture_obs(st)
                self._grab_observer()
                self._bridge_send()
            if st["fallen"] and not self._episode_extra.get("fell"):
                self._episode_extra["fell"] = True
                self._episode_extra["fell_at_sim_time"] = st["sim_time"]
        if need_status:
            self._publish_status(st)

    # ------------------------------------------------------------------ GUI camera panel
    PANEL_CAMS = (("front_camera", "head L  cam_left_high"), ("front_right_camera", "head R  cam_right_high"),
                  ("left_wrist_camera", "left wrist  cam_left_wrist"), ("right_wrist_camera", "right wrist  cam_right_wrist"))

    def _pace_step(self):
        """Cumulative-deadline pacing (demo.realtime_pacing): control step k ends no earlier than t0 + k * 20 ms.
        A step that overran is made up by the following cheaper steps; more than 0.25 s behind -> resynchronise
        instead of running a long catch-up burst."""
        now = time.perf_counter()
        if self._t_next is None or now - self._t_next > 0.25:
            self._t_next = now
        self._t_next += self.control_dt
        d = self._t_next - time.perf_counter()
        if d > 0:
            time.sleep(d)

    def _throttle_render(self, env):
        """Demo mode (no camera sensors): the official wholebody provider renders after EVERY control step and
        ignores sim.render_interval; demo.render_every_n_steps = N > 1 lets only every N-th call through, so the
        GUI viewport (and any camera) is drawn at 50/N Hz sim time. Camera frames then repeat on the skipped steps, so
        recording is refused while this is active (self._render_every)."""
        n = int((self.scn.get("demo") or {}).get("render_every_n_steps", 1) or 1)
        self._render_every = n
        if n <= 1:
            return
        orig, count = env.sim.render, [0]

        def render(*a, **k):
            count[0] += 1
            if count[0] % n == 0:
                return orig(*a, **k)

        env.sim.render = render
        print(f"[g1iv] demo mode: rendering every {n} control steps ({50.0 / n:.0f} Hz sim time)", flush=True)

    def _setup_camera_panel(self, args):
        """GUI only: a window docked under the viewport that shows the policy/recording camera buffers (the exact
        images that are recorded, lens model included). No extra rendering: it reads the existing camera outputs."""
        cfg = (g1iv_tasks.RIG.get("gui_panel") or {})
        if getattr(args, "headless", False) or not cfg.get("enabled", True):
            return None
        import asyncio

        import omni.kit.app
        import omni.ui as ui

        cams = [(n, label) for n, label in self.PANEL_CAMS if n in self.env.scene.keys()]
        if not cams:
            return None
        win = ui.Window("G1IV Cameras", width=1280, height=300)
        providers = {}
        with win.frame:
            with ui.HStack(spacing=4):
                for n, label in cams:
                    with ui.VStack(spacing=2):
                        ui.Label(label, height=16, alignment=ui.Alignment.CENTER)
                        prov = ui.ByteImageProvider()
                        ui.ImageWithProvider(prov, fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)
                        providers[n] = prov
        ratio = float(cfg.get("height_ratio", 0.30))

        async def _dock():
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            vp = ui.Workspace.get_window("Viewport")
            if vp is not None:
                win.dock_in(vp, ui.DockPosition.BOTTOM, ratio)
        asyncio.ensure_future(_dock())
        print(f"[g1iv] camera panel: {[n for n, _ in cams]} docked under the viewport")
        return {"win": win, "providers": providers, "every": max(1, int(cfg.get("every_n_steps", 2)))}

    def _update_camera_panel(self):
        for n, prov in self._panel["providers"].items():
            try:
                rgb = self.env.scene[n].data.output["rgb"][0][..., :3].cpu().numpy()
                rgba = np.empty(rgb.shape[:2] + (4,), np.uint8)
                rgba[..., :3] = rgb
                rgba[..., 3] = 255
                prov.set_data_array(rgba, [rgba.shape[1], rgba.shape[0]])
            except Exception as e:  # noqa: BLE001
                print(f"[g1iv] camera panel update failed ({n}): {e}; panel disabled")
                self._panel = None
                return

    def _apply_lens_distortion(self, env) -> dict:
        """Author the calibrated OpenCV lens model (RTX OmniLensDistortionOpenCvPinholeAPI) on every rig camera
        prim. Requires the omni.usd.schema.omni_lens_distortion extension enabled at app start (run_sim.sh passes
        --kit_args). Not applied => pinhole images, reported loudly and in the episode metadata."""
        rig = g1iv_tasks.RIG
        status = {"enabled": bool(rig.get("use_rig", True) and rig.get("lens_distortion", True)), "applied": {}, "error": None}
        if not status["enabled"]:
            return status
        try:
            import isaaclab.sim as sim_utils
            from pxr import Gf
            from .camera_rig import build_rig, lens_distortion_attributes

            for c in build_rig(rig):
                if c.name not in env.scene.keys():
                    continue
                attrs = lens_distortion_attributes(c.render)
                prims = sim_utils.find_matching_prims(env.scene[c.name].cfg.prim_path)
                for prim in prims:
                    prim.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")
                    prim.GetAttribute("omni:lensdistortion:model").Set("opencvPinhole")
                    for k, v in attrs.items():
                        prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{k}").Set(Gf.Vec2i(*v) if k == "imageSize" else v)
                status["applied"][c.name] = {"prims": len(prims), **{k: (list(v) if isinstance(v, tuple) else v) for k, v in attrs.items()},
                                             "monotonic_refit": c.refit}
            print(f"[g1iv] lens distortion (OpenCV pinhole) applied: {sorted(status['applied'])}")
        except Exception as e:  # noqa: BLE001
            status["error"] = repr(e)
            print(f"[g1iv] WARNING lens distortion NOT applied -> pinhole images ({e}). "
                  "Start through scripts/run_sim.sh (enables omni.usd.schema.omni_lens_distortion).")
        return status

    def _apply_rh56e2_colorway(self, hm: dict):
        """RH56E2 hands carry their own per-part materials (<robot>/Looks/rh56e2_<label>, tools/build_inspire_e2_usd.py);
        set them to the colorway of configs/hand_rh56e2.yaml (switch: --scene-set robot.hand_colorway=e2_black)."""
        import omni.usd
        from pxr import Gf, UsdShade

        name = (self.scn.get("robot") or {}).get("hand_colorway") or hm["colorway"]
        colors = hm["colorways"][name]
        stage = omni.usd.get_context().get_stage()
        n = 0
        for prim in stage.Traverse():
            pn = prim.GetName()
            if not pn.startswith("rh56e2_") or prim.GetParent().GetName() != "Looks":
                continue
            lab = pn[len("rh56e2_"):]
            sh = UsdShade.Shader(prim.GetChild("Shader"))
            if lab not in colors or not sh:
                continue
            r, g, b, metal, rough = colors[lab]
            sh.GetInput("diffuseColor").Set(Gf.Vec3f(r, g, b))
            sh.GetInput("metallic").Set(float(metal))
            sh.GetInput("roughness").Set(float(rough))
            n += 1
        if n == 0:
            raise RuntimeError("no rh56e2_* materials found (robot USD is not the RH56E2 build?)")
        self.hand_colorway = name
        print(f"[g1iv] RH56E2 hands: colorway {name} on {n} materials", flush=True)

    def _apply_hand_appearance(self):
        """Re-colour the Inspire hand links (configs/scene_desk.yaml: hand_appearance). Binds new
        UsdPreviewSurface materials, strongerThanDescendants, on each link's non-instanced 'visuals/<mesh xform>'
        prim, replacing the asset's near-white 'material_191919' there. Stage edit only (USD file untouched)."""
        hm = g1iv_tasks.HAND
        if hm["name"] == "rh56e2":
            self._apply_rh56e2_colorway(hm)
            return
        cfg = (self.scn or {}).get("hand_appearance", {})
        if not cfg.get("enabled", False):
            return
        import omni.usd
        from pxr import Gf, Sdf, UsdShade

        stage = omni.usd.get_context().get_stage()
        mats = {}
        for name, (r, g, b, metal, rough) in cfg["colors"].items():
            path = f"/World/G1IV_Looks/hand_{name}"
            mat = UsdShade.Material.Define(stage, path)
            sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(r, g, b))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            mats[name] = mat
        links = cfg["links"]
        n = 0
        for robot_prim in stage.Traverse():
            if robot_prim.GetName() != "Robot" or not str(robot_prim.GetPath()).startswith("/World/envs/"):
                continue
            for link in robot_prim.GetChildren():
                ln = link.GetName()
                key = ln[2:] if ln[:2] in ("L_", "R_") else ln.split("_", 1)[1] if ln.startswith(("left_", "right_")) else None
                if key not in links:
                    continue
                vis = link.GetChild("visuals")
                if not vis or not vis.IsValid():
                    continue
                for target in [vis] + list(vis.GetChildren()):
                    if target.IsInstanceProxy():
                        continue
                    UsdShade.MaterialBindingAPI.Apply(target).Bind(
                        mats[links[key]], UsdShade.Tokens.strongerThanDescendants)
                    n += 1
        print(f"[g1iv] hand appearance: {n} visual prims re-coloured ({', '.join(sorted(mats))})", flush=True)

    def on_shutdown(self):
        if self.bridge is not None:
            self.bridge.close()
        if self.rec.active:
            self._stop_episode("aborted", "sim_shutdown")
        # Close the project's DDS entities explicitly (the official code stops its own DDS objects). Leaving
        # CycloneDDS readers with listeners open made simulation_app.close() hang at interpreter shutdown.
        for name in ("sub_ctrl", "sub_meta", "pub_status"):
            ch = getattr(self, name, None)
            if ch is None:
                continue
            try:
                ch.Close()
            except Exception as e:  # noqa: BLE001
                print(f"[g1iv] closing {name} failed: {e}")
            setattr(self, name, None)
        print("[g1iv] shutdown: project DDS channels closed", flush=True)

    # ================================================================== state / obs / action
    def _read_state(self) -> dict:
        d = self.robot.data
        pos = d.root_pos_w[0].tolist()
        quat = d.root_quat_w[0].tolist()
        st = {"sim_time": self.step * self.control_dt, "sim_time_kit": float(getattr(self.env.sim, "current_time", float("nan"))),
              "wall_time": time.time(), "base_pos_w": pos, "base_quat_w": quat, "base_yaw": _quat_yaw(quat),
              "base_lin_vel_b": d.root_lin_vel_b[0].tolist(), "base_ang_vel_b": d.root_ang_vel_b[0].tolist(),
              "projected_gravity_b": d.projected_gravity_b[0].tolist()}
        st["fallen"] = bool(pos[2] < float(self.rcfg["fall_height_m"]) or st["projected_gravity_b"][2] > -0.5)
        ev = {"fallen": st["fallen"]}
        if self.i_body_eval:
            bp, bq = d.body_pos_w[0], d.body_quat_w[0]
            ev["body_pose_w"] = {k: bp[i].tolist() + bq[i].tolist() for k, i in self.i_body_eval.items()}  # xyz + wxyz
        if self.obj is not None:
            ev["object_pos_w"] = self.obj.data.root_pos_w[0].tolist()
            ev["object_quat_w"] = self.obj.data.root_quat_w[0].tolist()
            ev["object_lift_m"] = ev["object_pos_w"][2] - float(self.obj.data.default_root_state[0, 2])
            ev["target"] = self.target
            if len(self.objs) > 1:
                ev["objects"] = {n: o.data.root_pos_w[0].tolist() for n, o in self.objs.items()}
        ck = f"{self.obj_entities.get(self.target, 'object')}_contact"
        if ck in self.env.scene.keys():
            try:
                fm = self.env.scene[ck].data.force_matrix_w  # (N, 1, M, 3)
                if fm is not None:
                    f = torch.linalg.norm(fm[0, 0], dim=-1).tolist()
                    ev["contact_force_per_filter"] = f
                    ev["contact_force_total"] = float(sum(f))
            except Exception as e:  # noqa: BLE001
                ev["contact_error"] = repr(e)[:120]
        st["eval"] = ev
        return st

    def _capture_obs(self, st):
        d = self.robot.data
        jp, jv = d.joint_pos[0], d.joint_vel[0]
        hand_rad = jp[self.i_hand].tolist()
        obs = {"arm_q": jp[self.i_arm].tolist(), "arm_dq": jv[self.i_arm].tolist(),
               "hand_q_rad": hand_rad, "hand_q_norm": _rad_to_norm(HAND_NAMES, np.array(hand_rad)).tolist(),
               "joint_pos_all": jp.tolist(), "joint_vel_all": jv.tolist(),
               "base_pos_w": st["base_pos_w"], "base_quat_w": st["base_quat_w"], "base_lin_vel_b": st["base_lin_vel_b"],
               "base_ang_vel_b": st["base_ang_vel_b"], "projected_gravity_b": st["projected_gravity_b"], "cam_meta": {}}
        imgs, deps = {}, {}
        for c in self.cams:
            cam = self.env.scene[c]
            a = self.alias.get(c, c)
            # The official image server (camera_state) also reads front/wrist cameras from its own thread, which can
            # leave a buffer marked up to date although the renderer produced a newer frame in this control step.
            # Mark it outdated so this read always fetches the latest render (image and state from the same step).
            forced = False
            try:
                cam._is_outdated[:] = True
                forced = True
            except Exception:  # noqa: BLE001
                pass
            rgb = cam.data.output["rgb"][0]
            imgs[a] = rgb[..., :3].cpu().numpy().copy()
            fr = getattr(cam, "frame", None)
            obs["cam_meta"][a] = {"frame": int(fr[0]) if fr is not None else -1, "forced_update": forced,
                                  "ts": float(cam._timestamp_last_update[0]) if hasattr(cam, "_timestamp_last_update") else -1.0}
            if self.rcfg.get("save_depth") and "distance_to_image_plane" in cam.data.output:
                deps[a] = cam.data.output["distance_to_image_plane"][0].cpu().numpy().copy()
        return obs, imgs, deps, {"sim_time": st["sim_time"], "wall_time": st["wall_time"], "eval": st["eval"]}

    def _read_action(self) -> dict:
        """Targets that were written to the articulation during the step that just ran."""
        p = self.provider
        fa = p._full_action_buf if p is not None and hasattr(p, "_full_action_buf") else None
        act = {"loco_cmd": list(self._last_run_cmd)}
        if fa is not None:
            fa = fa.detach().cpu()
            hand_t = fa[self.i_hand].tolist()
            act.update({"arm_q_target": fa[self.i_arm].tolist(), "hand_q_rad_target": hand_t,
                        "hand_q_norm_target": _rad_to_norm(HAND_NAMES, np.array(hand_t)).tolist(),
                        "hand_all_rad_target": fa[self.i_hand_all].tolist(),
                        "leg_q_target_policy": fa[self.i_leg].tolist(), "waist_q_target": fa[self.i_waist].tolist()})
        return act

    # ================================================================== DDS in/out
    @staticmethod
    def _parse_run_cmd(d) -> list:
        cmd = [0.0, 0.0, 0.0, 0.8]
        if d and "run_command" in d:
            v = d["run_command"]
            try:
                v = ast.literal_eval(v) if isinstance(v, str) else v
                cmd = [float(x) for x in list(v)[:4]]
            except Exception:  # noqa: BLE001
                pass
        return cmd

    def _on_meta(self, msg):
        with self._meta_lock:
            try:
                self._meta_latest = json.loads(msg.data)
                self._meta_rx_mono = time.monotonic()
            except json.JSONDecodeError:
                return
        self.rec.add_teleop_meta(msg.data)

    def _meta_snapshot(self) -> dict:
        with self._meta_lock:
            m = self._meta_latest
            age = (time.monotonic() - self._meta_rx_mono) * 1000.0 if m else None
        if m is None:
            return {"present": False}
        out = {"present": True, "age_ms": age, "stale": age > float(self.rcfg["teleop_meta_max_age_ms"])}
        for k in ("seq", "wall_time", "arm_source", "loco_source", "input_ok", "engaged", "clutch", "hand_cmd",
                  "hand_q_sent", "trigger", "arm_ik_q", "arm_sent_q", "ik_ok", "loco_raw", "loco_sent", "loco_reason"):
            if k in m:
                out[k] = m[k]
        return out

    def _publish_status(self, st):
        wall = np.array(self._wall_hist) if self._wall_hist else np.array([np.nan])
        d = self.robot.data
        msg = {"wall_time": st["wall_time"], "sim_time": st["sim_time"], "step": self.step,
               "control_hz_wall": float(1.0 / np.mean(wall)) if np.all(np.isfinite(wall)) else None,
               "rtf": float(self.control_dt / np.mean(wall)) if np.all(np.isfinite(wall)) else None,
               "base_pos_w": st["base_pos_w"], "base_yaw": st["base_yaw"], "base_lin_vel_b": st["base_lin_vel_b"],
               "base_ang_vel_b": st["base_ang_vel_b"], "fallen": st["fallen"], "loco_cmd_applied": self._last_run_cmd,
               "arm_q": d.joint_pos[0, self.i_arm].tolist(),
               "hand_q_norm": _rad_to_norm(HAND_NAMES, d.joint_pos[0, self.i_hand].cpu().numpy()).tolist(),
               "reset_count": self.reset_count, "hand_model": g1iv_tasks.HAND["name"],
               "recording": {"active": self.rec.active, "frames": self.rec.n_frames, "dropped": self.rec.n_dropped,
                             "episode_id": self.rec.meta.get("episode_id") if self.rec.active else None},
               "eval_gt": st["eval"]}  # ground truth: for scripted tests / monitoring only
        try:
            self.pub_status.Write(self.String_(data=topics.dumps(msg)))
        except Exception as e:  # noqa: BLE001
            print(f"[g1iv] status publish failed: {e}")

    # ================================================================== control commands
    def _handle_ctrl(self, raw: str):
        try:
            m = json.loads(raw)
        except json.JSONDecodeError:
            print(f"[g1iv] bad control message: {raw[:120]}")
            return
        c = m.get("cmd")
        print(f"[g1iv] control: {c} {({k: v for k, v in m.items() if k not in ('cmd',)})}")
        if c == "record_start":
            if self.rec.active:
                print("[g1iv] record_start ignored: already recording")
                return
            if not self.cams or getattr(self, "_render_every", 1) > 1:
                print("[g1iv] record_start refused: demo mode (no camera sensors or rendering every "
                      f"{getattr(self, '_render_every', 1)} steps; run_sim.sh without --demo to record)")
                return
            self._start_episode(m)
        elif c == "record_stop":
            self._stop_episode(m.get("outcome", "undetermined"), m.get("reason", "operator_stop"))
        elif c == "record_abort":
            self._stop_episode("aborted", m.get("reason", "operator_abort"))
        elif c == "mark":
            if self.rec.active:
                self._marks.append({"step": self.step, "sim_time": self.step * self.control_dt, "label": m.get("label", "")})
        elif c == "reset":
            if self.rec.active:
                self._stop_episode(m.get("outcome", "undetermined"), "reset")
            self._full_reset(m)

    def _start_episode(self, m: dict):
        self.episode_index += 1
        meta = {"task_id": self.args.task, "task_name": m.get("task_name", "pick_cylinder"),
                "instruction": m.get("instruction", ""), "source": m.get("source", "unknown"),
                "operator": m.get("operator", ""), "notes": m.get("notes", ""),
                "arm_source": m.get("arm_source"), "loco_source": m.get("loco_source"),
                "seed": m.get("seed", self.scn.get("seed")), "episode_index_in_session": self.episode_index,
                "record_hz_sim": 1.0 / (self.control_dt * int(self.rcfg["record_every_n_steps"])),
                "control_dt": self.control_dt, "step_at_start": self.step, "reset_count_at_start": self.reset_count,
                "frame_semantics": "frame k = (observation BEFORE step s_k, targets applied DURING step s_k)",
                "target_object": self.target,
                "objects_xy": {n: [round(float(v), 4) for v in o.data.root_pos_w[0, :2].tolist()] for n, o in self.objs.items()},
                "static": self.static_meta}
        d = self.rec.start(meta)
        self._rec_step0 = self.step
        self.pairer.start(self.step)
        self._pending_obs = self._capture_obs(self._read_state())
        self._marks, self._episode_extra = [], {}
        print(f"[g1iv] recording -> {d}")
        if self.obs_video is not None:
            self.obs_video.start(self.rec.meta.get("episode_id", f"episode{self.episode_index}"))
            self._grab_observer()
        self._bridge_send()

    def _grab_observer(self):
        """G1IV_OBSERVER_VIDEO: observer image at the instant of the observation just captured (never a policy input)."""
        if self.obs_video is None or not self.obs_video.active:
            return
        try:
            cam = self.env.scene["world_camera"]
            try:
                cam._is_outdated[:] = True   # same forced read-back as _capture_obs: the render of this step
            except Exception:  # noqa: BLE001
                pass
            self.obs_video.add(cam.data.output["rgb"][0][..., :3].cpu().numpy())
        except Exception as e:  # noqa: BLE001  (viewing aid: never stop an episode)
            print(f"[g1iv] observer video frame failed, video off: {e}", flush=True)
            self.obs_video.stop({"outcome": "video_error", "error": repr(e)})
            self.obs_video = None

    def _bridge_send(self):
        """Policy bridge (G1IV_POLICY_BRIDGE): hand the observation just captured for the next record step to the
        connected policy runner and wait until it has published its commands (lockstep, see policy_bridge.py)."""
        if self.bridge is None or self._pending_obs is None or not self.rec.active:
            return
        policy_episode = self.rec.meta.get("source") == "policy"
        if not self.bridge.connected:
            if policy_episode:   # the runner is gone: nobody commands the robot any more -> do not record forever
                self._stop_episode("aborted", "policy_runner_lost")
            return
        obs, imgs, _deps, t = self._pending_obs
        ev = t.get("eval", {})
        meta = {"step": self.step, "sim_time": t["sim_time"], "episode_id": self.rec.meta.get("episode_id"),
                "frame": self.rec.n_frames, "arm_q": obs["arm_q"], "hand_q_norm": obs["hand_q_norm"],
                "base_pos_w": obs["base_pos_w"],
                "eval": {k: ev.get(k) for k in ("object_lift_m", "fallen", "object_pos_w", "target", "contact_force_total")}}
        reply = self.bridge.exchange(meta, imgs)
        if reply is None and policy_episode:
            self._stop_episode("aborted", "policy_runner_lost")
        elif reply and reply.get("stop"):
            self._stop_episode(reply["stop"], reply.get("reason", "policy_runner"))

    def _stop_episode(self, outcome: str, reason: str):
        if not self.rec.active:
            return
        wall = np.array(self._wall_hist) if self._wall_hist else np.array([np.nan])
        out = self.rec.stop(outcome, reason, {"marks": self._marks, **self._episode_extra,
                                              "step_at_stop": self.step,
                                              "control_hz_wall_recent": float(1.0 / np.nanmean(wall))})
        self._pending_obs = None
        self.pairer.stop()
        print(f"[g1iv] episode saved: {out}")
        if self.obs_video is not None and self.obs_video.active:
            self.obs_video.stop({"episode_id": self.rec.meta.get("episode_id"), "outcome": outcome, "end_reason": reason,
                                 "recorded_frames": self.rec.meta.get("n_frames"),
                                 "target_object": self.rec.meta.get("target_object")})

    def _full_reset(self, m: dict):
        env, robot = self.env, self.robot
        rng = np.random.default_rng(int(m.get("seed", self.scn.get("seed", 0))) + self.reset_count)
        origin = env.scene.env_origins[0]
        rs = robot.data.default_root_state.clone()
        r = self.scn["robot"]
        nx, ny = r.get("reset_noise_xy", [0, 0])
        dyaw = math.radians(r.get("reset_noise_yaw_deg", 0.0)) * rng.uniform(-1, 1)
        pose = m.get("robot_pose")  # optional [x, y, yaw_deg] override for stop-pose experiments
        if pose is not None:
            rs[0, 0], rs[0, 1] = float(pose[0]), float(pose[1])
            yaw = math.radians(float(pose[2]))
        else:
            rs[0, 0] += float(rng.uniform(-nx, nx)) if nx else 0.0
            rs[0, 1] += float(rng.uniform(-ny, ny)) if ny else 0.0
            yaw = math.radians(r["init_yaw_deg"]) + dyaw
        rs[0, 3:7] = torch.tensor([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)], device=rs.device)
        rs[0, :3] += origin
        rs[0, 7:] = 0.0
        robot.write_root_pose_to_sim(rs[:, :7])
        robot.write_root_velocity_to_sim(rs[:, 7:])
        jp, jv = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
        robot.write_joint_state_to_sim(jp, jv)
        robot.set_joint_position_target(jp)
        robot.reset()
        want = m.get("target")
        if want is not None and want not in self.objs:
            print(f"[g1iv] reset: unknown target {want!r} (have {list(self.objs)}) -> {next(iter(self.objs), None)}")
            want = None
        self.target = want if want is not None else next(iter(self.objs), None)
        self.obj = self.objs.get(self.target)
        for name, ob in self.objs.items():
            # every cylinder back to its spawn pose; noise / --object-xy apply to the target only (extra cylinders are
            # at fixed positions)
            os_ = ob.data.default_root_state.clone()
            if name == self.target:
                o = self.scn["object"]
                onx, ony = o.get("reset_noise_xy", [0, 0])
                if m.get("object_xy") is not None:
                    os_[0, 0], os_[0, 1] = float(m["object_xy"][0]), float(m["object_xy"][1])
                else:
                    os_[0, 0] += float(rng.uniform(-onx, onx)) if onx else 0.0
                    os_[0, 1] += float(rng.uniform(-ony, ony)) if ony else 0.0
            os_[0, :3] += origin
            os_[0, 7:] = 0.0
            ob.write_root_pose_to_sim(os_[:, :7])
            ob.write_root_velocity_to_sim(os_[:, 7:])
            ob.reset()
        p = self.provider
        if p is not None:  # locomotion policy history + delayed actions
            try:
                p.actor_obs_buffer.reset()
                p.action_buffer.reset()
                p.action_buffer.compute(torch.zeros(1, p.num_actions_all, device=env.device))
                if hasattr(p, "_full_action_buf"):
                    p._full_action_buf.copy_(jp[0])
            except Exception as e:  # noqa: BLE001
                print(f"[g1iv] policy buffer reset failed: {e}")
            self._reset_cmd_shm(p, jp[0])
        env.scene.write_data_to_sim()
        self.reset_count += 1
        print(f"[g1iv] full reset #{self.reset_count}: robot {rs[0, :3].tolist()} yaw {math.degrees(yaw):.1f} deg "
              f"target {self.target}")

    def _reset_cmd_shm(self, p, default_q):
        """Overwrite the last DDS commands kept in shared memory so the sim does not jump back to a
        stale arm/hand target before the hub publishes its own reset targets."""
        try:
            if getattr(p, "robot_dds", None) is not None and p.robot_dds.output_shm:
                q29 = default_q[self.i_sdk29].tolist()
                z = [0.0] * 29
                p.robot_dds.output_shm.write_data({"motor_cmd": {"positions": q29, "velocities": z, "torques": z, "kp": z, "kd": z}})
            if getattr(p, "inspire_dds", None) is not None and p.inspire_dds.output_shm:
                qn = preset_vector(self.hcfg["presets"][self.hcfg.get("active_open", "open")])
                rad = norm_to_rad(qn).tolist()
                z = [0.0] * 12
                p.inspire_dds.output_shm.write_data({"positions": rad + rad, "velocities": z, "torques": z, "kp": z, "kd": z})
            if getattr(p, "run_command_dds", None) is not None:
                p.run_command_dds.write_run_command([0.0, 0.0, 0.0, 0.8])
        except Exception as e:  # noqa: BLE001
            print(f"[g1iv] command shm reset failed: {e}")

    # ================================================================== provenance
    def _camera_rig_meta(self) -> dict:
        """Intrinsics / extrinsics of the recorded cameras (OpenCV frames), keyed by recording alias."""
        rig = g1iv_tasks.RIG
        if not rig.get("use_rig", True):
            return {"use_rig": False}
        from .camera_rig import build_rig

        out = {"use_rig": True, "standard": rig.get("standard"), "config": rig}
        for c in build_rig(rig):
            alias = self.alias.get(c.name, c.name)
            out[alias] = {"scene_camera": c.name, "lerobot_key": c.alias, "parent_link": c.parent_link,
                          "position_m": c.pos.tolist(), "quat_wxyz_ros": list(c.quat_wxyz()), "R_parent_cam": c.R.tolist(),
                          "width": c.intr.width, "height": c.intr.height, "K": c.intr.K().tolist(), "dist": c.intr.dist,
                          "render_dist": c.render.dist, "clipping_range": list(c.clipping)}
        return out

    def _static_meta(self) -> dict:
        import isaacsim  # noqa: F401

        def ver(mod):
            try:
                return __import__(mod).__version__
            except Exception:  # noqa: BLE001
                return None
        robot_usd = self.robot.cfg.spawn.usd_path
        pol = Path(os.environ.get("PROJECT_ROOT", "")) / self.args.model_path
        return {
            "task_id": self.args.task, "robot": f"unitree_g1_29dof_rev_1_0 (G1 EDU Plus body) + inspire {g1iv_tasks.HAND['name']} (DFX interface), wholebody",
            "hand_model": {k: v for k, v in g1iv_tasks.HAND.items() if k != "colorways"},
            "robot_usd": robot_usd, "locomotion_policy": str(pol),
            "locomotion_policy_sha256": _sha256(pol) if pol.is_file() else None,
            "joint_names_isaac_order": self.joint_names, "arm_names": ARM_NAMES, "hand_names_dfx_order": HAND_NAMES,
            "hand_all_names": HAND_ALL_NAMES, "leg_names": LEG_NAMES, "waist_names": WAIST_NAMES,
            "sdk29_names": SDK29_NAMES, "cameras": {self.alias.get(c, c): c for c in self.cams},
            "camera_rig": self._camera_rig_meta(), "lens_distortion": getattr(self, "lens_status", None),
            "camera_resolution": [int(self.scn["cameras"]["width"]), int(self.scn["cameras"]["height"])],
            "control_dt": self.control_dt, "physics_dt": float(self.env_cfg.sim.dt), "decimation": int(self.env_cfg.decimation),
            "sim_device": str(self.env.device), "scene_cfg": self.scn, "record_cfg": self.rcfg,
            "hands_cfg": self.hcfg, "host": platform.node(), "torch": torch.__version__, "numpy": np.__version__,
            "repos": {n: _git(WS / "repos" / n) for n in ("unitree_sim_isaaclab", "IsaacLab", "xr_teleoperate", "unitree_sdk2_python")},
            "workspace_git": _git(WS),
            "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
        }


def _sha256(p: Path) -> str:
    import hashlib

    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


class _MainLoopProfiler:
    """Opt-in cProfile of the simulator main thread (whole sim_main loop iterations, incl. controller.step()).
    G1IV_PROFILE="START:END" -> profile policy steps START..END, write logs/profile/sim_<time>.{prof,txt}."""

    def __init__(self, spec: str):
        a, b = (int(x) for x in spec.split(":"))
        self.start, self.end, self.prof, self.t0 = a, b, None, None

    def on_step(self, step: int):
        import cProfile

        if step == self.start:
            self.prof, self.t0 = cProfile.Profile(), time.perf_counter()
            self.prof.enable()
        elif step == self.end and self.prof is not None:
            self.prof.disable()
            wall = time.perf_counter() - self.t0
            import io
            import pstats

            out = Path(os.environ.get("G1IV_WS", ".")) / "logs" / "profile"
            out.mkdir(parents=True, exist_ok=True)
            stem = out / time.strftime("sim_%Y%m%d_%H%M%S")
            self.prof.dump_stats(str(stem) + ".prof")
            buf = io.StringIO()
            n = self.end - self.start
            buf.write(f"steps {n}  wall {wall:.2f} s  {1000 * wall / n:.1f} ms/step (profiled)\n")
            st = pstats.Stats(self.prof, stream=buf)
            st.sort_stats("cumulative").print_stats(60)
            st.sort_stats("tottime").print_stats(40)
            Path(str(stem) + ".txt").write_text(buf.getvalue())
            print(f"[g1iv] profile written: {stem}.txt ({1000 * wall / n:.1f} ms/step)", flush=True)
            self.prof = None


_PROF = _MainLoopProfiler(os.environ["G1IV_PROFILE"]) if os.environ.get("G1IV_PROFILE") else None

EXT = G1IVExt()
on_env_created = EXT.on_env_created
on_controller_ready = EXT.on_controller_ready
on_step = EXT.on_step
on_shutdown = EXT.on_shutdown
