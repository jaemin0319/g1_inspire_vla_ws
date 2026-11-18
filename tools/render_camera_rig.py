"""Render the project camera rig (configs/camera_rig.yaml) at given joint configurations.

Used to compare the simulated head / wrist views with real Unitree G1+Inspire frames at the SAME joint
state (tools/unitree_frames.py makes the frame list) and to validate the lens model.
The robot is the official G1-29DoF+Inspire (or, --robot dex3, +Dex3) wholebody USD with a FIXED base and gravity
off: this is a kinematic rendering fixture, not a simulation of the task. Read-only w.r.t. the project.

  PYTHONPATH=src:repos/unitree_sim_isaaclab <sim python> tools/render_camera_rig.py \
      --frames logs/camera_rig/frames.json --out logs/camera_rig/render [--set wrist.axial_x=0.04 ...]
Writes <out>/<frame>_<camera>.png and <out>/poses.json (link + camera world poses, rig used).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--frames", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--rig", default="configs/camera_rig.yaml")
ap.add_argument("--set", action="append", default=[], help="rig override key=value (repeatable)")
ap.add_argument("--no-distortion", action="store_true")
ap.add_argument("--steps", type=int, default=6, help="render steps per frame")
ap.add_argument("--extra-cameras", default="", help="json list of extra test cameras {name,parent,pos,quat_wxyz,fx}")
ap.add_argument("--robot", default="inspire", choices=["inspire", "dex3"],
                help="official wholebody USD: G1-29DoF + Inspire (default) or + Dex3 (Unitree D405-wrist datasets)")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.enable_cameras = True
# the RTX lens-distortion USD schema (OmniLensDistortion*API) registers only when enabled at app start
LENS_EXT = "--enable omni.usd.schema.omni_lens_distortion"
if LENS_EXT not in (args.kit_args or ""):
    args.kit_args = f"{args.kit_args or ''} {LENS_EXT}".strip()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

WS = os.environ.get("G1IV_WS") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src"))
sys.path.insert(0, os.path.join(WS, "repos", "unitree_sim_isaaclab"))
os.environ.setdefault("PROJECT_ROOT", os.path.join(WS, "repos", "unitree_sim_isaaclab"))
from g1iv_common.config import load_config  # noqa: E402
from g1iv_sim.camera_rig import apply_rig_hardware, build_rig, lens_distortion_attributes, pinhole_spawn_params  # noqa: E402
from robots.unitree import G129_CFG_WITH_DEX3_WHOLEBODY, G129_CFG_WITH_INSPIRE_WHOLEBODY  # noqa: E402  (official)

rig_cfg = load_config(args.rig, args.set)
rig = build_rig(rig_cfg)
os.makedirs(args.out, exist_ok=True)
ROBOT_CFG = {"inspire": G129_CFG_WITH_INSPIRE_WHOLEBODY, "dex3": G129_CFG_WITH_DEX3_WHOLEBODY}[args.robot]

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cpu"))
sim_utils.GroundPlaneCfg(color=(0.35, 0.35, 0.35)).func("/World/ground", sim_utils.GroundPlaneCfg(color=(0.35, 0.35, 0.35)))
sim_utils.DomeLightCfg(intensity=1500.0, color=(1.0, 1.0, 1.0)).func("/World/dome", sim_utils.DomeLightCfg(intensity=1500.0))
dl = sim_utils.DistantLightCfg(intensity=2500.0, angle=1.0)
dl.func("/World/sun", dl, orientation=(0.92, 0.0, 0.38, 0.0))

rcfg = ROBOT_CFG.replace(prim_path="/World/Robot")
rcfg.spawn.rigid_props.disable_gravity = True
rcfg.spawn.articulation_props.fix_root_link = True
rcfg.init_state.pos = (0.0, 0.0, 0.793)
robot = Articulation(rcfg)
hw_log = apply_rig_hardware(sim.stage.GetPrimAtPath("/World/Robot"), rig_cfg, os.path.join(WS, "assets", "camera_rig"))
print("[render] camera hardware", hw_log, flush=True)

cams = {}
for c in rig:
    sp = pinhole_spawn_params(c.render)
    cfg = CameraCfg(prim_path=f"/World/Robot/{c.parent_link}/{c.name}", update_period=0.0, height=c.intr.height,
                    width=c.intr.width, data_types=["rgb"],
                    spawn=sim_utils.PinholeCameraCfg(clipping_range=c.clipping, **sp),
                    offset=CameraCfg.OffsetCfg(pos=tuple(float(v) for v in c.pos), rot=c.quat_wxyz(), convention="ros"))
    cams[c.name] = Camera(cfg)
for e in json.loads(args.extra_cameras or "[]"):
    cfg = CameraCfg(prim_path=f"/World/Robot/{e['parent']}/{e['name']}", update_period=0.0, height=480, width=640,
                    data_types=["rgb"], spawn=sim_utils.PinholeCameraCfg(focal_length=e.get("focal", 12.0), horizontal_aperture=20.955,
                                                                         clipping_range=(0.01, 100.0)),
                    offset=CameraCfg.OffsetCfg(pos=tuple(e["pos"]), rot=tuple(e["quat_wxyz"]), convention="world"))
    cams[e["name"]] = Camera(cfg)

sim.reset()
stage = sim.stage
applied = {}
if not args.no_distortion and rig_cfg.get("lens_distortion", True):
    for c in rig:
        prim = stage.GetPrimAtPath(f"/World/Robot/{c.parent_link}/{c.name}")
        prim.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")
        prim.GetAttribute("omni:lensdistortion:model").Set("opencvPinhole")
        for k, v in lens_distortion_attributes(c.render).items():
            attr = prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{k}")
            attr.Set(Gf.Vec2i(*v) if k == "imageSize" else v)
        applied[c.name] = lens_distortion_attributes(c.render)

frames = json.load(open(args.frames))
names = robot.joint_names
out_poses = {"rig": {c.name: {"parent": c.parent_link, "pos": c.pos.tolist(), "R": c.R.tolist(), "alias": c.alias,
                              "K": c.render.K().tolist(), "dist": c.render.dist, "calib_dist": c.intr.dist,
                              "refit": c.refit} for c in rig},
             "lens_distortion_applied": applied, "rig_cfg": rig_cfg, "robot": args.robot, "frames": {}}
missing_report = set()
for fr in frames:
    q = robot.data.default_joint_pos.clone()
    for jn, v in fr["joints"].items():
        if jn in names:
            q[0, names.index(jn)] = float(v)
        else:
            missing_report.add(jn)
    for _ in range(args.steps):
        robot.write_joint_state_to_sim(q, torch.zeros_like(q))
        robot.set_joint_position_target(q)
        robot.write_data_to_sim()
        sim.step(render=True)
        robot.update(sim.get_physics_dt())
    for cam in cams.values():
        cam.update(sim.get_physics_dt(), force_recompute=True)
    rec = {"bodies": {}, "cameras": {}}
    bp = robot.data.body_link_pose_w[0].cpu().numpy()
    for i, bn in enumerate(robot.body_names):
        rec["bodies"][bn] = bp[i].tolist()  # x y z qw qx qy qz
    for n, cam in cams.items():
        img = cam.data.output["rgb"][0].cpu().numpy()[..., :3]
        from PIL import Image
        Image.fromarray(img).save(os.path.join(args.out, f"{fr['name']}_{n}.png"))
        rec["cameras"][n] = {"pos_w": cam.data.pos_w[0].tolist(), "quat_w_ros": cam.data.quat_w_ros[0].tolist(),
                             "K_isaaclab": cam.data.intrinsic_matrices[0].tolist()}
    out_poses["frames"][fr["name"]] = rec
    print(f"[render] {fr['name']}: {len(cams)} cameras", flush=True)
if missing_report:
    print("[render] joints not in the robot:", sorted(missing_report))
json.dump(out_poses, open(os.path.join(args.out, "poses.json"), "w"), indent=1)
print("[render] done", args.out, flush=True)
sys.stdout.flush()
os._exit(0)
