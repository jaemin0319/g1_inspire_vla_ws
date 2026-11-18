"""Fit a rig camera pose (head stereo left eye or a wrist camera; optionally intrinsics) to real Unitree G1 frames
by silhouette-edge alignment.

For each frame: the robot's visual meshes (exported from the official USD, tools/export_link_meshes.py) are
placed with the link poses of render_camera_rig.py (same kinematics as the simulator at the recorded joint
values), projected with the candidate camera (OpenCV model incl. distortion) and rasterised to a silhouette.
Cost = mean truncated distance from the silhouette boundary to the nearest Canny edge of the real image
(one-sided chamfer), summed over frames. Nelder-Mead over the selected parameters.

  <teleop python> tools/fit_rig_camera.py [--camera head|left_wrist|right_wrist] --poses logs/camera_rig/calib_render_v0/poses.json \
      --frames-dir logs/camera_rig/calib_frames --meshes data/reference/usd_link_meshes \
      --out logs/camera_rig/fit_head --free x y z pitch yaw roll f
"""
from __future__ import annotations

import argparse
import copy
import glob
import json
import os
import re
import sys
import time
from multiprocessing import get_context

import cv2
import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from g1iv_common.config import load_config  # noqa: E402
from g1iv_sim.camera_rig import Intrinsics, distort_radius, head_cameras, wrist_camera, wrist_profile  # noqa: E402

REAL = {"head": "head_stereo_left", "left_wrist": "wrist_left", "right_wrist": "wrist_right"}


def make_cam(cfg, which):
    if which == "head":
        return head_cameras(cfg["head"], 640, 480)[0]
    return wrist_camera(cfg["wrist"], which.split("_")[0], 640, 480)

TRUNC = 25.0


def pose_to_T(p):
    x, y, z, qw, qx, qy, qz = p
    R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                  [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                  [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    T = np.eye(4); T[:3, :3], T[:3, 3] = R, (x, y, z)
    return T


def decimate(P, F, cell=0.003):
    key = np.round(P / cell).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    Pn = np.zeros((len(uniq), 3)); np.add.at(Pn, inv, P); Pn /= np.bincount(inv)[:, None]
    Fn = inv[F]
    ok = (Fn[:, 0] != Fn[:, 1]) & (Fn[:, 1] != Fn[:, 2]) & (Fn[:, 0] != Fn[:, 2])
    Fn = np.unique(np.sort(Fn[ok], axis=1), axis=0)
    return Pn, Fn


def project(P_cam, intr):
    z = P_cam[:, 2]
    x, y = P_cam[:, 0] / z, P_cam[:, 1] / z
    r = np.hypot(x, y)
    s = np.where(r > 1e-9, distort_radius(r, intr.dist) / np.maximum(r, 1e-9), 1.0)
    p1, p2 = intr.dist.get("p1", 0.0), intr.dist.get("p2", 0.0)
    r2 = x * x + y * y
    xd = x * s + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * s + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.c_[intr.fx * xd + intr.cx, intr.fy * yd + intr.cy], z


class Problem:
    def __init__(self, a):
        self.a = a
        self.base = load_config(a.rig, a.set)
        P = json.load(open(a.poses))
        names = a.names or sorted(P["frames"])
        self.meshes = {}
        for f in glob.glob(os.path.join(a.meshes, "*.npz")):
            n = os.path.basename(f)[:-4]
            if a.links and not re.fullmatch(a.links, n):
                continue
            d = np.load(f)
            self.meshes[n] = decimate(d["points"], d["faces"], cell=a.cell)
        self.frames = []
        for n in names:
            img = cv2.imread(os.path.join(a.frames_dir, f"{n}_real_{REAL[a.camera]}.png"))
            g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0)
            med = float(np.median(g))
            e = cv2.Canny(g, int(max(10, 0.5 * med)), int(min(255, 1.2 * med)))
            dt = cv2.distanceTransform(255 - e, cv2.DIST_L2, 3)
            bodies = P["frames"][n]["bodies"]
            Tt_inv = np.linalg.inv(pose_to_T(bodies[make_cam(self.base, a.camera).parent_link]))
            pts, faces, off = [], [], 0
            for b, Tb in bodies.items():
                if b not in self.meshes:
                    continue
                Pm, Fm = self.meshes[b]
                Tl = Tt_inv @ pose_to_T(Tb)               # link in the camera parent frame
                pts.append(Pm @ Tl[:3, :3].T + Tl[:3, 3]); faces.append(Fm + off); off += len(Pm)
            self.frames.append({"name": n, "img": img, "edges": e, "dt": dt,
                                "pts": np.vstack(pts), "faces": np.vstack(faces)})
        print(f"[fit] {len(self.frames)} frames, mesh faces/frame ~{len(self.frames[0]['faces'])}", flush=True)

    def cfg_from(self, x):
        c = copy.deepcopy(self.base)
        h = c["head"]
        cx, cy, cz = h["center_xyz"]
        p = dict(zip(self.a.free, x))
        h["center_xyz"] = [cx + p.get("x", 0.0) / 1000, cy + p.get("y", 0.0) / 1000, cz + p.get("z", 0.0) / 1000]
        h["pitch_down_deg"] = h["pitch_down_deg"] + p.get("pitch", 0.0)
        h["yaw_deg"] = h.get("yaw_deg", 0.0) + p.get("yaw", 0.0)
        h["roll_deg"] = h.get("roll_deg", 0.0) + p.get("roll", 0.0)
        mount, w = wrist_profile(c["wrist"])        # w = the active wrist model's parameters (a view into c)
        side = self.a.camera.split("_")[0]
        if self.a.camera == "head":
            pass
        elif mount == "unitree_d405_c":
            # CAD-derived pose: free = bracket turn about the hand axis, clamp position along it, and (diagnostic)
            # a camera-frame rotation on top of the CAD camera pose
            w["phi_deg"][side] = float(w["phi_deg"][side]) + p.get("phi", 0.0)
            w["ring_mid_x_mm"] = float(w["ring_mid_x_mm"]) + p.get("ax", 0.0)
            corr = list(w.get("cam_correction_deg") or [0.0, 0.0, 0.0])
            w["cam_correction_deg"] = [corr[0] + p.get("rx", 0.0), corr[1] + p.get("ry", 0.0), corr[2] + p.get("rz", 0.0)]
        else:
            w["axial_x"] = w["axial_x"] + p.get("ax", 0.0) / 1000
            w["radial_offset"] = w["radial_offset"] + p.get("rad", 0.0) / 1000
            w["z_offset"] = w.get("z_offset", 0.0) + p.get("zo", 0.0) / 1000
            w["outward_tilt_deg"] = w.get("outward_tilt_deg", 0.0) + p.get("out", 0.0)
            w["tangential_tilt_deg"] = w.get("tangential_tilt_deg", 0.0) + p.get("tan", 0.0)
            w["roll_deg"] = w.get("roll_deg", 0.0) + p.get("wroll", 0.0)
        L = h["calibration"]["left"] if self.a.camera == "head" else w["calibration"][side]
        sf = 1.0 + p.get("f", 0.0) / 100.0
        L["fx"], L["fy"] = L["fx"] * sf, L["fy"] * sf
        L["cx"], L["cy"] = L["cx"] + p.get("cx", 0.0), L["cy"] + p.get("cy", 0.0)
        L["k1"] = L.get("k1", 0.0) + p.get("k1", 0.0) / 100.0
        return c

    def silhouette(self, fr, cam):
        T = np.linalg.inv(cam.T_parent_cam())
        Pc = fr["pts"] @ T[:3, :3].T + T[:3, 3]
        uv, z = project(Pc, cam.render)   # what the simulator renders (monotonic radial re-fit if the calibration folds)
        F = fr["faces"]
        ok = (z[F] > self.a.near).all(1)
        tri = uv[F[ok]]
        inb = ((tri[:, :, 0] > -400) & (tri[:, :, 0] < 1040) & (tri[:, :, 1] > -400) & (tri[:, :, 1] < 880)).all(1)
        tri = np.round(tri[inb] * 4).astype(np.int32)   # 2 fractional bits
        m = np.zeros((480, 640), np.uint8)
        # one triangle at a time: cv2.fillPoly with many polygons fills with the even-odd rule, so overlapping
        # triangles (front + back faces of a closed mesh) cancel and the "silhouette" becomes a wireframe
        # (bug fixed 2026-11-12; fits before that date used the wireframe masks, docs/CAMERA_RIG.md)
        for t in tri:
            cv2.fillConvexPoly(m, t, 255, lineType=cv2.LINE_8, shift=2)
        return m

    def cost_frame(self, fr, cam):
        m = self.silhouette(fr, cam)
        bd = m & ~cv2.erode(m, np.ones((3, 3), np.uint8))
        bd[:2, :] = 0; bd[-2:, :] = 0; bd[:, :2] = 0; bd[:, -2:] = 0   # image border is not an edge
        ys, xs = np.nonzero(bd)
        if len(xs) < 50:
            return TRUNC, m
        return float(np.minimum(fr["dt"][ys, xs], TRUNC).mean()), m

    def cost(self, x):
        cam = make_cam(self.cfg_from(x), self.a.camera)
        if POOL is not None:
            return float(np.mean(POOL.map(_worker_cost, [(i, cam) for i in range(len(self.frames))])))
        return float(np.mean([self.cost_frame(fr, cam)[0] for fr in self.frames]))


    def overlay(self, x, path):
        cam = make_cam(self.cfg_from(x), self.a.camera)
        tiles = []
        for fr in self.frames:
            c, m = self.cost_frame(fr, cam)
            im = fr["img"].copy()
            im[fr["edges"] > 0] = (0, 0, 255)
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(im, cnts, -1, (0, 255, 0), 2)
            cv2.putText(im, f"{fr['name']} cost {c:.2f}px", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            tiles.append(cv2.resize(im, (320, 240)))
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        grid = np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])
        cv2.imwrite(path, grid)


PB = None
POOL = None


def _worker_cost(arg):
    i, cam = arg
    return PB.cost_frame(PB.frames[i], cam)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses", required=True)
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--meshes", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rig", default="configs/camera_rig.yaml")
    ap.add_argument("--camera", default="head", choices=list(REAL))
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--names", nargs="*")
    ap.add_argument("--free", nargs="+", default=["x", "y", "z", "pitch", "yaw", "roll"])
    ap.add_argument("--init", nargs="*", type=float)
    ap.add_argument("--maxiter", type=int, default=600)
    ap.add_argument("--grid", action="store_true", help="coarse grid over pitch and f first")
    ap.add_argument("--cell", type=float, default=0.005, help="mesh vertex clustering cell [m]")
    ap.add_argument("--procs", type=int, default=14)
    ap.add_argument("--links", default="", help="regex of link names to include (default all)")
    ap.add_argument("--scan", default="", help="NAME:start:stop:step - scan one free parameter before Nelder-Mead (e.g. phi:-180:180:10)")
    ap.add_argument("--near", type=float, default=0.03,
                    help="drop triangles closer than this [m] (0.03 for the head; ~0.01 for D405 wrist cameras, the palm is 2-5 cm away)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    global PB, POOL
    pb = PB = Problem(a)
    POOL = get_context("fork").Pool(a.procs) if a.procs > 1 else None
    step = {"ax": 5.0, "rad": 5.0, "zo": 5.0, "out": 4.0, "tan": 4.0, "wroll": 4.0, "x": 15.0, "y": 15.0, "z": 15.0, "pitch": 4.0, "yaw": 3.0, "roll": 3.0, "f": 8.0, "cx": 15.0, "cy": 15.0, "k1": 5.0,
            "phi": 8.0, "rx": 3.0, "ry": 3.0, "rz": 3.0}
    x0 = np.array(a.init if a.init else [0.0] * len(a.free), dtype=float)
    t0 = time.time()
    c0 = pb.cost(x0)
    print(f"[fit] initial cost {c0:.3f} px ({time.time() - t0:.2f}s/eval)", flush=True)
    pb.overlay(x0, os.path.join(a.out, "overlay_initial.jpg"))
    if a.grid:
        best = (c0, x0)
        g1 = "pitch" if a.camera == "head" else "out"
        for dp in (-10, -5, 0, 5, 10):
            for df in ((-25, -15, -5, 5) if "f" in a.free else (0,)):
                x = x0.copy()
                if g1 in a.free: x[a.free.index(g1)] = dp
                if "f" in a.free: x[a.free.index("f")] = df
                c = pb.cost(x)
                if c < best[0]:
                    best = (c, x)
        print(f"[fit] grid best {best[0]:.3f} at {dict(zip(a.free, best[1]))}", flush=True)
        x0 = best[1]
    if a.scan:
        k, lo, hi, st = a.scan.split(":")
        best, table = (pb.cost(x0), x0), []
        for val in np.arange(float(lo), float(hi) + 1e-9, float(st)):
            x = x0.copy(); x[a.free.index(k)] = val
            c = pb.cost(x); table.append((round(float(val), 2), round(c, 3)))
            if c < best[0]:
                best = (c, x)
        print(f"[fit] scan {k}: " + " ".join(f"{v}:{c}" for v, c in table), flush=True)
        print(f"[fit] scan best {best[0]:.3f} at {dict(zip(a.free, best[1]))}", flush=True)
        x0 = best[1]
        pb.overlay(x0, os.path.join(a.out, "overlay_scan.jpg"))
    simplex = [x0] + [x0 + np.eye(len(a.free))[i] * step[k] for i, k in enumerate(a.free)]
    res = minimize(pb.cost, x0, method="Nelder-Mead",
                   options={"initial_simplex": np.array(simplex), "maxiter": a.maxiter, "xatol": 0.05, "fatol": 1e-3, "disp": True})
    x = res.x
    pb.overlay(x, os.path.join(a.out, "overlay_fit.jpg"))
    cfg = pb.cfg_from(x)
    per = {fr["name"]: round(pb.cost_frame(fr, make_cam(cfg, a.camera))[0], 3) for fr in pb.frames}
    out = {"free": a.free, "delta": dict(zip(a.free, x.tolist())), "cost_initial_px": c0, "cost_fit_px": float(res.fun),
           "per_frame_px": per, "head": cfg["head"], "nit": int(res.nit), "set": a.set, "camera": a.camera, "wrist": cfg["wrist"]}
    json.dump(out, open(os.path.join(a.out, "fit.json"), "w"), indent=1)
    print(json.dumps({k: out[k] for k in ("delta", "cost_initial_px", "cost_fit_px")}, indent=1))
    if a.camera != "head":
        mount, w = wrist_profile(cfg["wrist"])
        keys = (("phi_deg", "ring_mid_x_mm", "cam_correction_deg") if mount == "unitree_d405_c" else
                ("axial_x", "radial_offset", "z_offset", "outward_tilt_deg", "tangential_tilt_deg", "roll_deg"))
        cam = make_cam(cfg, a.camera)
        print(mount, {k: w[k] for k in keys}, "camera pos [mm]", (cam.pos * 1000).round(1).tolist(), "axis", cam.R[:, 2].round(4).tolist())
    print("center_xyz", cfg["head"]["center_xyz"], "pitch", cfg["head"]["pitch_down_deg"], "yaw", cfg["head"]["yaw_deg"],
          "roll", cfg["head"]["roll_deg"], "fx", cfg["head"]["calibration"]["left"]["fx"])


if __name__ == "__main__":
    main()
