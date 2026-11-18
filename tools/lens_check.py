"""Numerical check of the rendered camera model: RTX OmniLensDistortionOpenCvPinholeAPI + rig pose vs the OpenCV
projection used everywhere else (g1iv_sim.camera_rig.project_points).

Small emissive spheres are placed at known 3D points inside each rig camera's field of view (robot visuals hidden,
no lights => black background), the images are rendered, sphere centroids detected, and compared with the OpenCV
projection of the sphere centres. Writes <out>/lens_check.json and one annotated PNG per camera.

  PYTHONPATH=src:repos/unitree_sim_isaaclab <sim python> tools/lens_check.py --headless --out logs/camera_rig/lens_check
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--rig", default="configs/camera_rig.yaml")
ap.add_argument("--set", action="append", default=[])
ap.add_argument("--no-distortion", action="store_true", help="control run: pinhole only")
ap.add_argument("--tangential", default="as_is", choices=["as_is", "swap", "negate", "swap_negate", "zero", "neg_p1", "neg_p2"],
                help="how p1/p2 are AUTHORED for RTX (the OpenCV prediction always uses the calibration as is)")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.enable_cameras = True
LENS_EXT = "--enable omni.usd.schema.omni_lens_distortion"
if LENS_EXT not in (args.kit_args or ""):
    args.kit_args = f"{args.kit_args or ''} {LENS_EXT}".strip()
app = AppLauncher(args).app

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

WS = os.environ.get("G1IV_WS") or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "src"))
sys.path.insert(0, os.path.join(WS, "repos", "unitree_sim_isaaclab"))
os.environ.setdefault("PROJECT_ROOT", os.path.join(WS, "repos", "unitree_sim_isaaclab"))
from g1iv_common.config import load_config  # noqa: E402
from g1iv_sim.camera_rig import build_rig, lens_distortion_attributes, pinhole_spawn_params, project_points  # noqa: E402
from robots.unitree import G129_CFG_WITH_INSPIRE_WHOLEBODY  # noqa: E402

rig_cfg = load_config(args.rig, args.set)
rig = build_rig(rig_cfg)
os.makedirs(args.out, exist_ok=True)
sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cpu"))
rcfg = G129_CFG_WITH_INSPIRE_WHOLEBODY.replace(prim_path="/World/Robot")
rcfg.spawn.rigid_props.disable_gravity = True
rcfg.spawn.articulation_props.fix_root_link = True
rcfg.init_state.pos = (0.0, 0.0, 0.793)
robot = Articulation(rcfg)
cams = {}
for c in rig:
    sp = pinhole_spawn_params(c.render)
    cams[c.name] = Camera(CameraCfg(prim_path=f"/World/Robot/{c.parent_link}/{c.name}", update_period=0.0,
                                    height=c.intr.height, width=c.intr.width, data_types=["rgb"],
                                    spawn=sim_utils.PinholeCameraCfg(clipping_range=c.clipping, **sp),
                                    offset=CameraCfg.OffsetCfg(pos=tuple(float(v) for v in c.pos), rot=c.quat_wxyz(), convention="ros")))
sim.reset()
for _ in range(3):
    sim.step(render=True); robot.update(0.005)

stage = sim.stage
# hide every robot visual (cameras stay); no lights at all => black background
for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot")):
    if p.GetName() == "visuals":
        UsdGeom.Imageable(p).MakeInvisible()

bodies = robot.body_names
bp = robot.data.body_link_pose_w[0].cpu().numpy()


def T_of(name):
    x, y, z, qw, qx, qy, qz = bp[bodies.index(name)]
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                  [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    T = np.eye(4); T[:3, :3], T[:3, 3] = R, (x, y, z)
    return T


# targets: a grid of rays (undistorted normalised coords) covering the image, one depth per camera
markers = {}
k = 0
for c in rig:
    depth = 0.6 if c.name.startswith("front") else 0.25
    rad = 0.004 if c.name.startswith("front") else 0.0016
    Tw = T_of(c.parent_link) @ c.T_parent_cam()
    pts = []
    for v in np.linspace(40, c.intr.height - 40, 5):
        for u in np.linspace(40, c.intr.width - 40, 7):
            # undistort the pixel with OpenCV on the RENDER model to get a ray that lands inside the image
            K = c.render.K(); D = c.render.opencv_dist()
            xn = cv2.undistortPointsIter(np.array([[[u, v]]], np.float64), K, D, None, None,
                                         (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-12))[0, 0]
            p_cam = np.array([xn[0], xn[1], 1.0]) * depth
            pts.append((Tw @ np.r_[p_cam, 1.0])[:3])
    markers[c.name] = np.array(pts)
    for p in pts:
        cfg = sim_utils.SphereCfg(radius=rad, visual_material=sim_utils.PreviewSurfaceCfg(emissive_color=(1.0, 1.0, 1.0), diffuse_color=(1.0, 1.0, 1.0)))
        cfg.func(f"/World/Marker_{k:03d}", cfg, translation=tuple(float(x) for x in p))
        k += 1

applied = False
if not args.no_distortion and rig_cfg.get("lens_distortion", True):
    for c in rig:
        prim = stage.GetPrimAtPath(f"/World/Robot/{c.parent_link}/{c.name}")
        prim.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")
        prim.GetAttribute("omni:lensdistortion:model").Set("opencvPinhole")
        la = lens_distortion_attributes(c.render)
        p1, p2 = la["p1"], la["p2"]
        la["p1"], la["p2"] = {"as_is": (p1, p2), "swap": (p2, p1), "negate": (-p1, -p2), "swap_negate": (-p2, -p1),
                              "zero": (0.0, 0.0), "neg_p1": (-p1, p2), "neg_p2": (p1, -p2)}[args.tangential]
        for kk, vv in la.items():
            prim.GetAttribute(f"omni:lensdistortion:opencvPinhole:{kk}").Set(Gf.Vec2i(*vv) if kk == "imageSize" else vv)
    applied = True
for _ in range(8):
    sim.step(render=True)
for cam in cams.values():
    cam.update(0.005, force_recompute=True)

report = {"lens_distortion_applied": applied, "tangential_authoring": args.tangential, "cameras": {}}
for c in rig:
    img = cams[c.name].data.output["rgb"][0].cpu().numpy()[..., :3].copy()
    uv_pred, _ = project_points(c, T_of(c.parent_link), markers[c.name], use_render=True)
    g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    _, bw = cv2.threshold(g, 60, 255, cv2.THRESH_BINARY)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(bw)
    det = cent[1:][stats[1:, cv2.CC_STAT_AREA] >= 3]
    errs, signed = [], []
    vis = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    for p in uv_pred:
        if not np.all(np.isfinite(p)) or abs(p[0]) > 1e5 or abs(p[1]) > 1e5:
            errs.append(None)
            continue
        cv2.drawMarker(vis, (int(round(p[0])), int(round(p[1]))), (0, 0, 255), cv2.MARKER_CROSS, 14, 1)
        if len(det):
            dd = np.linalg.norm(det - p, axis=1)
            j = int(np.argmin(dd))
            errs.append(float(dd[j]) if dd[j] < 25 else None)
            if dd[j] < 25:
                signed.append((det[j] - p).tolist())
        else:
            errs.append(None)
    ok = [e for e in errs if e is not None]
    report["cameras"][c.name] = {"markers": len(uv_pred), "detected_blobs": int(len(det)), "matched": len(ok),
                                 "err_px_mean": float(np.mean(ok)) if ok else None, "err_px_max": float(np.max(ok)) if ok else None,
                                 "errors_px": errs, "mean_signed_du_dv": np.mean(signed, 0).tolist() if signed else None,
                                 "signed_du_dv": signed}
    cv2.imwrite(os.path.join(args.out, f"{c.name}_markers.png"), vis)
    print(f"[lens] {c.name}: matched {len(ok)}/{len(uv_pred)} mean {report['cameras'][c.name]['err_px_mean']} max {report['cameras'][c.name]['err_px_max']} mean_du_dv {np.round(report['cameras'][c.name]['mean_signed_du_dv'], 2).tolist()}", flush=True)
json.dump(report, open(os.path.join(args.out, "lens_check.json"), "w"), indent=1)
sys.stdout.flush()
os._exit(0)
