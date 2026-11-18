"""Camera rig of the Unitree G1 + Inspire DFX data-collection setup (configs/camera_rig.yaml).

Pure numpy (no Isaac imports at module level) so the geometry can be unit-tested and reused by
offline tools. Sources and the reasoning behind every number: docs/CAMERA_RIG.md.

Frames
  * camera frames are OpenCV / ROS optical frames (x right, y down, z forward) = Isaac Lab "ros" convention
  * head poses are given in ``torso_link`` (the G1 head is rigidly fixed to the torso)
  * wrist poses are given in ``<side>_wrist_yaw_link`` (x along the hand, the Inspire palm faces +y on the
    right hand and -y on the left hand)
  * wrist camera models (``wrist.mount``): ``unitree_d405_c`` = Unitree D405 wrist mount (CAD) + RealSense D405,
    pose from the bracket CAD (assets/camera_rig/wrist_d405_mount_c.npz) and its clamp position on the wrist;
    ``inspire_ring_mono160`` = 160 deg monocular camera on the Inspire/Brainco ring bracket (parametric)
Quaternions returned for Isaac Lab 2.x are (w, x, y, z).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

ASSETS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "assets", "camera_rig")


# ----------------------------------------------------------------------------------------------
# small rotation helpers
# ----------------------------------------------------------------------------------------------
def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def rot_from_forward_down(forward, down) -> np.ndarray:
    """Rotation matrix (columns = camera x/y/z axes in the parent frame) of an OpenCV camera that looks
    along ``forward`` with the image-down direction as close as possible to ``down``."""
    z = _unit(forward)
    y = np.asarray(down, dtype=float) - z * float(np.dot(down, z))
    y = _unit(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)


def rotvec_to_matrix(rv) -> np.ndarray:
    rv = np.asarray(rv, dtype=float)
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def axis_angle(axis, deg) -> np.ndarray:
    return rotvec_to_matrix(_unit(axis) * math.radians(deg))


def matrix_to_quat_wxyz(R) -> tuple[float, float, float, float]:
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    n = math.sqrt(sum(c * c for c in q))
    q = tuple(c / n for c in q)
    return q if q[0] >= 0 else tuple(-c for c in q)


# ----------------------------------------------------------------------------------------------
# intrinsics / lens distortion
# ----------------------------------------------------------------------------------------------
@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: dict = field(default_factory=dict)  # OpenCV pinhole: k1 k2 p1 p2 k3 [k4 k5 k6]

    @classmethod
    def from_cfg(cls, c: dict, width: int, height: int) -> "Intrinsics":
        sw, sh = width / float(c.get("width", width)), height / float(c.get("height", height))
        return cls(width, height, float(c["fx"]) * sw, float(c["fy"]) * sh, float(c["cx"]) * sw, float(c["cy"]) * sh,
                   {k: float(c[k]) for k in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6") if k in c})

    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def opencv_dist(self) -> np.ndarray:
        d = self.dist
        v = [d.get("k1", 0.0), d.get("k2", 0.0), d.get("p1", 0.0), d.get("p2", 0.0), d.get("k3", 0.0)]
        if any(k in d for k in ("k4", "k5", "k6")):
            v += [d.get("k4", 0.0), d.get("k5", 0.0), d.get("k6", 0.0)]
        return np.array(v, dtype=float)

    def fov_deg(self) -> tuple[float, float]:
        """Pinhole field of view from the focal lengths (no distortion)."""
        return (2 * math.degrees(math.atan(self.width / 2 / self.fx)), 2 * math.degrees(math.atan(self.height / 2 / self.fy)))


def distort_radius(r, d: dict):
    k1, k2, k3 = d.get("k1", 0.0), d.get("k2", 0.0), d.get("k3", 0.0)
    k4, k5, k6 = d.get("k4", 0.0), d.get("k5", 0.0), d.get("k6", 0.0)
    r2 = r * r
    return r * (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) / (1 + k4 * r2 + k5 * r2 ** 2 + k6 * r2 ** 3)


def monotonic_radial_fit(intr: Intrinsics):
    """Re-fit a radial polynomial that folds back inside the image (still OpenCV pinhole).

    Same idea as unitree_sim_isaaclab tasks/common_config/camera_calibration.py (Apache-2.0, 2026-11-09):
    strong barrel calibrations (wrist cameras, k1 ~ -0.4) give r_d(r) whose maximum lies below the radius of
    the image corners; a renderer would show folded garbage there. The radial part is re-fitted in the
    rational form r(1+k1 r^2+k2 r^4+k3 r^6)/(1+k4 r^2+k5 r^4+k6 r^6), matching the calibration where it is
    trustworthy (r_d <= 0.7) and monotonic up to the corners. fx, fy, cx, cy, p1, p2 are unchanged.

    Returns None if the calibrated polynomial is already fine, else {k1..k6, max_error_px, r_fold}.
    """
    d = intr.dist
    if any(abs(d.get(k, 0.0)) > 0 for k in ("k4", "k5", "k6")):
        return None
    W, H = intr.width, intr.height
    rd_need = max(math.hypot((u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy) for u, v in ((0, 0), (W, 0), (0, H), (W, H)))
    r = np.linspace(0.0, 3.0, 30001)
    rd = distort_radius(r, d)
    falling = np.nonzero(np.diff(rd) <= 0)[0]
    if len(falling) == 0 or rd[falling[0]] >= rd_need:
        return None
    r_fold = float(r[falling[0]])
    r_ok, rd_ok = r[: falling[0]], rd[: falling[0]]
    rr = np.linspace(0.0, float(np.interp(0.7, rd_ok, r_ok)), 200)
    t = np.linspace(0.0, 6.0, 60001)
    best = None
    for frac in (0.6, 0.7, 0.8, 0.9):
        x = np.linspace(1e-4, frac * r_fold, 400)
        y = distort_radius(x, d)
        A = np.stack([x ** 3, x ** 5, x ** 7, -y * x ** 2, -y * x ** 4, -y * x ** 6], 1)
        for keep in ((0, 3), (0, 1, 3, 4), (0, 1, 2, 3), (0, 1, 2, 3, 4, 5)):
            a = np.zeros(6)
            a[list(keep)] = np.linalg.lstsq(A[:, keep], y - x, rcond=None)[0]
            num = t * (1 + a[0] * t ** 2 + a[1] * t ** 4 + a[2] * t ** 6)
            den = 1 + a[3] * t ** 2 + a[4] * t ** 4 + a[5] * t ** 6
            ok = den > 0
            yy = np.where(ok, num / np.where(ok, den, 1.0), -np.inf)
            if yy.max() < 1.02 * rd_need:
                continue
            reach = int(np.argmax(yy >= 1.02 * rd_need))
            if not (np.all(ok[: reach + 1]) and np.all(np.diff(yy[: reach + 1]) > 0)):
                continue
            f = rr * (1 + a[0] * rr ** 2 + a[1] * rr ** 4 + a[2] * rr ** 6) / (1 + a[3] * rr ** 2 + a[4] * rr ** 4 + a[5] * rr ** 6)
            err = float(np.max(np.abs(f - distort_radius(rr, d))) * intr.fx)
            if best is None or err < best[0]:
                best = (err, a.copy())
    if best is None:
        return {"failed": True, "r_fold": r_fold}
    err, a = best
    return {**{f"k{i + 1}": float(a[i]) for i in range(6)}, "max_error_px": err, "r_fold": r_fold}


def render_intrinsics(intr: Intrinsics) -> tuple[Intrinsics, dict | None]:
    """Intrinsics handed to the renderer: the calibration, with a monotonic radial re-fit if needed."""
    fit = monotonic_radial_fit(intr)
    if not fit or fit.get("failed"):
        return intr, fit
    dist = {k: fit[k] for k in ("k1", "k2", "k3", "k4", "k5", "k6")}
    dist.update({"p1": intr.dist.get("p1", 0.0), "p2": intr.dist.get("p2", 0.0)})
    return Intrinsics(intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy, dist), fit


# ----------------------------------------------------------------------------------------------
# rig
# ----------------------------------------------------------------------------------------------
@dataclass
class RigCamera:
    name: str             # Isaac Lab scene entity name (front_camera, front_right_camera, left_wrist_camera, ...)
    alias: str            # recorded / LeRobot key (cam_left_high, ...)
    parent_link: str
    pos: np.ndarray       # optical centre in the parent link [m]
    R: np.ndarray         # camera (OpenCV) axes in the parent link
    intr: Intrinsics      # calibration (as measured)
    render: Intrinsics    # what the renderer gets (monotonic radial re-fit if needed)
    refit: dict | None
    clipping: tuple[float, float]

    def quat_wxyz(self):
        return matrix_to_quat_wxyz(self.R)

    def T_parent_cam(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R, self.pos
        return T


def head_cameras(h: dict, width: int, height: int) -> list[RigCamera]:
    """Stereo pair: the left eye at center - baseline/2 (robot +y side), right eye = left eye + the
    calibrated right-in-left extrinsic (OpenCV: p_left = R(rvec) p_right + t)."""
    th = math.radians(float(h["pitch_down_deg"]))
    fwd = np.array([math.cos(th), 0.0, -math.sin(th)])
    down = np.array([-math.sin(th), 0.0, -math.cos(th)])
    R_left = rot_from_forward_down(fwd, down)
    roll = float(h.get("roll_deg", 0.0)); yaw = float(h.get("yaw_deg", 0.0))
    if yaw:
        R_left = axis_angle([0, 0, 1], yaw) @ R_left
    if roll:
        R_left = R_left @ axis_angle([0, 0, 1], roll)
    cal = h["calibration"]
    ext = cal["right_in_left"]
    R_rl = rotvec_to_matrix(ext["rvec"])
    t_rl = np.asarray(ext["tvec_mm"], dtype=float) / 1000.0
    c = np.asarray(h["center_xyz"], dtype=float)
    # midpoint between optical centres = c  ->  left = c - R_left t_rl / 2
    p_left = c - R_left @ t_rl / 2.0
    p_right = p_left + R_left @ t_rl
    R_right = R_left @ R_rl
    clip = tuple(h.get("clipping_range", (0.05, 1000.0)))
    # the prims hang under mount_link (head_link: fixed child of torso_link, no rotation, URDF head_joint)
    link = h.get("mount_link") or h["parent_link"]
    off = np.asarray(h.get("mount_link_in_parent_xyz", (0.0, 0.0, 0.0)), dtype=float) if h.get("mount_link") else np.zeros(3)
    out = []
    for name, alias, p, R, key in (("front_camera", "cam_left_high", p_left, R_left, "left"),
                                   ("front_right_camera", "cam_right_high", p_right, R_right, "right")):
        intr = Intrinsics.from_cfg(cal[key], width, height)
        rend, fit = render_intrinsics(intr)
        out.append(RigCamera(name, alias, link, p - off, R, intr, rend, fit, clip))
    return out


WRIST_MOUNTS = ("unitree_d405_c", "inspire_ring_mono160")


def wrist_profile(w: dict) -> tuple[str, dict]:
    """(mount name, its parameter dict) of the active wrist camera model. Flat legacy configs (no ``mount``) are
    the mono160 ring model."""
    mount = w.get("mount", "inspire_ring_mono160")
    if mount not in WRIST_MOUNTS:
        raise ValueError(f"wrist.mount must be one of {WRIST_MOUNTS}, got {mount!r}")
    return mount, (w.get(mount) or w)


@lru_cache(maxsize=8)
def load_cad_asset(name: str) -> dict:
    with np.load(os.path.join(ASSETS_DIR, name)) as d:
        return {k: d[k] for k in d.files}


def d405_c_mount_pose(p: dict, side: str) -> tuple[np.ndarray, np.ndarray, dict]:
    """(R, t, asset) mapping the D405 wrist-mount CAD frame [m] into <side>_wrist_yaw_link.
    The clamp bore axis lies on the wrist-yaw housing axis (x, clamp_axis_yz_mm), its hand-side end (CAD bore axis
    direction) points to +x (towards the hand), the half-ring mid-plane sits at x = ring_mid_x_mm, and the bracket
    is turned about the hand axis so that the CAD +y direction (clamp -> camera) is (0, cos phi, sin phi)."""
    d = load_cad_asset(p.get("asset", "wrist_d405_mount_c.npz"))
    key = p["cad_variant"][side]
    phi = math.radians(float(p["phi_deg"][side]))
    cz = np.array([1.0, 0.0, 0.0]) * float(np.sign(d[f"{key}_bore_axis"][2]))
    cy = np.array([0.0, math.cos(phi), math.sin(phi)])
    R = np.stack([np.cross(cy, cz), cy, cz], 1)
    yz = np.asarray(p["clamp_axis_yz_mm"][side], dtype=float) / 1000.0
    target = np.array([float(p["ring_mid_x_mm"]) / 1000.0, yz[0], yz[1]])
    t = target - R @ d[f"{key}_ring_mid"]
    return R, t, d


def wrist_camera(w: dict, side: str, width: int, height: int) -> RigCamera:
    mount, p = wrist_profile(w)
    if mount == "unitree_d405_c":
        R, t, d = d405_c_mount_pose(p, side)
        key = p["cad_variant"][side]
        pos = R @ d[f"{key}_cam_pos"] + t
        Rc = R @ d[f"{key}_cam_R"]
        corr = p.get("cam_correction_deg") or [0.0, 0.0, 0.0]     # camera-frame x, y, z (fit diagnostics only)
        for ax, deg in zip(np.eye(3), corr):
            if deg:
                Rc = Rc @ axis_angle(ax, float(deg))
        intr = Intrinsics.from_cfg(p["calibration"][side], width, height)
        rend, fit = render_intrinsics(intr)
        return RigCamera(f"{side}_wrist_camera", f"cam_{side}_wrist", w["parent_link"][side], pos, Rc, intr, rend, fit,
                         tuple(p.get("clipping_range", w.get("clipping_range", (0.01, 1000.0)))))
    return mono160_wrist_camera({**w, **p}, side, width, height)


def mono160_wrist_camera(w: dict, side: str, width: int, height: int) -> RigCamera:
    """Monocular wrist camera on the Inspire/Brainco ring bracket, on the palm side of the wrist.
    side: 'left' | 'right'. Palm side is +y (right) / -y (left) in <side>_wrist_yaw_link."""
    s = 1.0 if side == "right" else -1.0
    radial = np.array([0.0, s, 0.0])                     # wrist axis -> camera (palm side)
    pos = np.array([float(w["axial_x"]), 0.0, 0.0]) + float(w["radial_offset"]) * radial + np.array([0, 0, float(w.get("z_offset", 0.0))])
    fwd = np.array([1.0, 0.0, 0.0])
    # outward tilt: optical axis turned away from the hand axis (towards the camera side), about z
    fwd = axis_angle([0, 0, s], float(w.get("outward_tilt_deg", 0.0))) @ fwd
    # tangential tilt: towards +z on both sides (left/right wrist frames are mirror images across y)
    if w.get("tangential_tilt_deg"):
        fwd = axis_angle([0, -1, 0], float(w["tangential_tilt_deg"])) @ fwd
    R = rot_from_forward_down(fwd, radial)               # image down = away from the hand axis
    if w.get("roll_deg"):
        R = R @ axis_angle([0, 0, s], float(w["roll_deg"]))
    intr = Intrinsics.from_cfg(w["calibration"][side], width, height)
    rend, fit = render_intrinsics(intr)
    return RigCamera(f"{side}_wrist_camera", f"cam_{side}_wrist", w["parent_link"][side], pos, R, intr, rend, fit,
                     tuple(w.get("clipping_range", (0.01, 1000.0))))


def build_rig(cfg: dict) -> list[RigCamera]:
    W, H = (int(v) for v in cfg.get("resolution", (640, 480)))
    cams = head_cameras(cfg["head"], W, H)
    if not cfg["head"].get("right_eye", True):
        cams = cams[:1]
    cams += [wrist_camera(cfg["wrist"], "left", W, H), wrist_camera(cfg["wrist"], "right", W, H)]
    return cams


# Conventions of the RTX OmniLensDistortionOpenCvPinholeAPI (Isaac Sim 5.1) relative to OpenCV, measured with
# tools/lens_check.py (emissive markers at known 3D points, logs/camera_rig/lens_check*):
#   * the tangential coefficient p1 has the opposite sign (RTX evaluates the model with image y pointing up:
#     every term odd in y flips, i.e. exactly the p1 terms; p2 is unchanged)
#   * the principal point is offset by half a pixel (RTX pixel origin at the pixel corner, OpenCV at the centre)
RTX_P1_SIGN = -1.0
RTX_PIXEL_OFFSET = 0.5


def lens_distortion_attributes(intr: Intrinsics) -> dict:
    """USD attributes of the RTX OmniLensDistortionOpenCvPinholeAPI (Isaac Sim 5.x) that reproduce the OpenCV
    model ``intr``: omni:lensdistortion:opencvPinhole:{cx,cy,fx,fy,imageSize,k1..k6,p1,p2,s1..s4}."""
    a = {"cx": float(intr.cx) + RTX_PIXEL_OFFSET, "cy": float(intr.cy) + RTX_PIXEL_OFFSET,
         "fx": float(intr.fx), "fy": float(intr.fy), "imageSize": (int(intr.width), int(intr.height))}
    for k in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2"):
        a[k] = float(intr.dist.get(k, 0.0))
    a["p1"] *= RTX_P1_SIGN
    for k in ("s1", "s2", "s3", "s4"):
        a[k] = 0.0
    return a


def pinhole_spawn_params(intr: Intrinsics, horizontal_aperture: float = 20.955) -> dict:
    """Focal length / apertures consistent with fx, fy (square-ish pixels, principal point offset) so that
    Isaac Lab's CameraData.intrinsic_matrices and the viewport preview match the calibration."""
    W, H = intr.width, intr.height
    focal = intr.fx * horizontal_aperture / W
    vertical_aperture = horizontal_aperture * H / W * intr.fx / intr.fy
    return {"focal_length": focal, "horizontal_aperture": horizontal_aperture, "vertical_aperture": vertical_aperture,
            "horizontal_aperture_offset": (intr.cx - W / 2.0) * horizontal_aperture / W,
            "vertical_aperture_offset": (intr.cy - H / 2.0) * vertical_aperture / H}


def project_points(cam: RigCamera, T_parent_world: np.ndarray, pts_world: np.ndarray, use_render: bool = True):
    """OpenCV projection (with distortion) of world points into ``cam``. T_parent_world: 4x4 pose of the
    parent link in the world. Returns (uv [N,2], depth [N])."""
    T = T_parent_world @ cam.T_parent_cam()
    Tinv = np.linalg.inv(T)
    P = (Tinv @ np.c_[pts_world, np.ones(len(pts_world))].T).T[:, :3]
    z = P[:, 2]
    x, y = P[:, 0] / z, P[:, 1] / z
    intr = cam.render if use_render else cam.intr
    d = intr.dist
    r = np.hypot(x, y)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(r > 0, distort_radius(r, d) / r, 1.0)
    p1, p2 = d.get("p1", 0.0), d.get("p2", 0.0)
    r2 = x * x + y * y
    xd = x * s + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * s + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.c_[intr.fx * xd + intr.cx, intr.fy * yd + intr.cy], z


# ----------------------------------------------------------------------------------------------
# camera hardware visuals (USD; called by desk_task right after the robot USD is spawned, before physics starts)
# ----------------------------------------------------------------------------------------------
def head_bracket_pose(eyes, d) -> tuple[np.ndarray, np.ndarray]:
    """(R, t) mapping the head-bracket CAD frame into the head cameras' link frame: lens-hole midpoint -> midpoint
    of the optical centres, camera-plate normal -> optical axis, CAD baseline (+y = robot right) -> camera +x."""
    def frame(a, b):
        a = a / np.linalg.norm(a)
        b = b - a * np.dot(a, b)
        b = b / np.linalg.norm(b)
        return np.stack([a, b, np.cross(a, b)], 1)

    L = eyes[0]
    R = frame(L.R[:, 2], L.R[:, 0]) @ frame(np.asarray(d["lens_axis_cad"], float), np.asarray(d["baseline_dir_cad"], float)).T
    mid = (eyes[0].pos + eyes[1].pos) / 2
    t = mid - R @ np.asarray(d["lens_holes_cad"], float).mean(0)
    return R, t


def apply_rig_hardware(robot_prim, rig_cfg: dict, assets_dir) -> dict:
    """On one spawned robot prim:
      * deactivate the official asset's <side>_hand_camera_base_link visuals + collisions (Unitree's Dex3 mono-camera
        ring mount, ~10 cm tall, not the rig modelled here),
      * add the wrist camera hardware under <side>_wrist_yaw_link: wrist.mount = unitree_d405_c -> Unitree D405 mount
        + half ring + RealSense D405 (assets/camera_rig/wrist_d405_mount_c.npz, same pose as the wrist camera);
        inspire_ring_mono160 -> Inspire/Brainco ring bracket + camera-module stand-in,
      * add Unitree's head stereo bracket (CAD, aligned to the head optical centres) under the head camera link,
      * add camera-module stand-ins (board + lens barrel) at the head cameras (hardware_visuals.camera_modules).
    Visual meshes only: no collision / rigid-body APIs. Returns what was done (for the log / metadata)."""
    import os

    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    hw = rig_cfg.get("hardware_visuals", {}) or {}
    stage = robot_prim.GetStage()
    done = {"hidden": [], "added": []}

    def material(name, rgb, rough):
        path = Sdf.Path(f"/World/G1IV_Looks/{name}")
        if not stage.GetPrimAtPath(path):
            mat = UsdShade.Material.Define(stage, path)
            sh = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return UsdShade.Material(stage.GetPrimAtPath(path))

    black = material("camera_hardware_black", (0.025, 0.025, 0.025), 0.55)
    glass = material("camera_lens_glass", (0.01, 0.01, 0.012), 0.08)
    if hw.get("hide_official_hand_camera_base", True):
        for side in ("left", "right"):
            link = robot_prim.GetChild(f"{side}_hand_camera_base_link")
            for child in ("visuals", "collisions"):
                c = link.GetChild(child) if link else None
                if c and c.IsValid():
                    c.SetActive(False)
                    done["hidden"].append(str(c.GetPath()))
    wrist_mount, wrist_p = wrist_profile(rig_cfg["wrist"])
    if hw.get("wrist_brackets", True) and wrist_mount == "unitree_d405_c":
        alu = material("d405_aluminium", (0.32, 0.33, 0.35), 0.35)
        for side in ("left", "right"):
            link = robot_prim.GetChild(rig_cfg["wrist"]["parent_link"][side])
            if not link:
                continue
            R, t, d = d405_c_mount_pose(wrist_p, side)
            key = wrist_p["cad_variant"][side]
            for part, mat in (("mount", black), ("ring", black), ("d405", alu)):
                pts = d[f"{key}_{part}_points"].astype(float) @ R.T + t
                mesh = UsdGeom.Mesh.Define(stage, link.GetPath().AppendChild(f"G1IV_wrist_d405_{part}"))
                mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
                f = d[f"{key}_{part}_faces"]
                mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(f)))
                mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(f.reshape(-1).astype(np.int32)))
                mesh.CreateSubdivisionSchemeAttr("none")
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
                done["added"].append(str(mesh.GetPath()))
            done.setdefault("wrist_d405_front_centre_in_link", {})[side] = (R @ d[f"{key}_d405_front_centre"] + t).round(4).tolist()
    elif hw.get("wrist_brackets", True):
        for side in ("left", "right"):
            f = os.path.join(str(assets_dir), f"inspire_wrist_bracket_{side}.npz")
            link = robot_prim.GetChild(f"{side}_wrist_yaw_link")
            if not (os.path.exists(f) and link):
                continue
            d = np.load(f)
            mesh = UsdGeom.Mesh.Define(stage, link.GetPath().AppendChild("G1IV_wrist_camera_bracket"))
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(d["points"].astype(np.float32)))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(d["faces"])))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(d["faces"].reshape(-1).astype(np.int32)))
            mesh.CreateSubdivisionSchemeAttr("none")
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(black)
            done["added"].append(str(mesh.GetPath()))
    def add_mesh(link, name, pts, faces, mat):
        mesh = UsdGeom.Mesh.Define(stage, link.GetPath().AppendChild(name))
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(pts, np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(faces, np.int32).reshape(-1)))
        mesh.CreateSubdivisionSchemeAttr("none")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        done["added"].append(str(mesh.GetPath()))

    def add_box(link, name, R, centre, size, mat):
        box = UsdGeom.Cube.Define(stage, link.GetPath().AppendChild(name))
        box.CreateSizeAttr(1.0)
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = centre
        xf = UsdGeom.Xformable(box)
        xf.AddTransformOp().Set(Gf.Matrix4d(M.T.tolist()))
        xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
        UsdShade.MaterialBindingAPI.Apply(box.GetPrim()).Bind(mat)
        done["added"].append(str(box.GetPath()))

    def add_lens(link, name, R, front, d, length, mat):
        cyl = UsdGeom.Cylinder.Define(stage, link.GetPath().AppendChild(name))
        cyl.CreateRadiusAttr(d / 2); cyl.CreateHeightAttr(length); cyl.CreateAxisAttr("Z")
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = front - R[:, 2] * length / 2
        UsdGeom.Xformable(cyl).AddTransformOp().Set(Gf.Matrix4d(M.T.tolist()))
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(mat)
        done["added"].append(str(cyl.GetPath()))

    W, H = (int(v) for v in rig_cfg.get("resolution", (640, 480)))
    eyes = head_cameras(rig_cfg["head"], W, H)          # both eyes, even if the right one is not rendered
    head_link = robot_prim.GetChild(eyes[0].parent_link)
    mods = hw.get("camera_modules", {}) or {}
    if head_link and hw.get("head_bracket", True):
        f = os.path.join(str(assets_dir), hw.get("head_bracket_asset", "head_stereo_bracket_60fps_classic98_cad.npz"))
        if os.path.exists(f):
            d = np.load(f)
            R, t = head_bracket_pose(eyes, d)
            add_mesh(head_link, "G1IV_head_stereo_bracket", d["points"] @ R.T + t, d["faces"], black)
            done["head_bracket_screws_in_link"] = (d["screws_cad"] @ R.T + t).round(4).tolist()
    hm = mods.get("head")
    if head_link and hm:
        L = eyes[0]
        mid = (eyes[0].pos + eyes[1].pos) / 2
        add_box(head_link, "G1IV_head_stereo_board", L.R, mid - L.R[:, 2] * hm["board_behind_mm"] / 1000,
                np.array(hm["board_mm"]) / 1000, black)
        for c in eyes:
            if hm.get("holder_mm"):
                add_box(head_link, f"G1IV_head_lens_holder_{c.name}", c.R, c.pos - c.R[:, 2] * hm["holder_behind_mm"] / 1000,
                        np.array(hm["holder_mm"]) / 1000, black)
            add_lens(head_link, f"G1IV_head_lens_{c.name}", c.R, c.pos + c.R[:, 2] * hm["lens_front_mm"] / 1000,
                     hm["lens_d_mm"] / 1000, hm["lens_len_mm"] / 1000, glass)
    wm = mods.get("wrist") if wrist_mount == "inspire_ring_mono160" else None   # the D405 model is its own module
    if wm:
        for side in ("left", "right"):
            c = wrist_camera(rig_cfg["wrist"], side, W, H)
            link = robot_prim.GetChild(c.parent_link)
            if not link:
                continue
            add_box(link, "G1IV_wrist_camera_board", c.R, c.pos - c.R[:, 2] * wm["board_behind_mm"] / 1000,
                    np.array(wm["board_mm"]) / 1000, black)
            if wm.get("holder_mm"):
                add_box(link, "G1IV_wrist_camera_lens_holder", c.R, c.pos - c.R[:, 2] * wm["holder_behind_mm"] / 1000,
                        np.array(wm["holder_mm"]) / 1000, black)
            add_lens(link, "G1IV_wrist_camera_lens", c.R, c.pos + c.R[:, 2] * wm["lens_front_mm"] / 1000,
                     wm["lens_d_mm"] / 1000, wm["lens_len_mm"] / 1000, glass)
    return done
