"""Small URDF reader + forward kinematics + painter's-algorithm mesh renderer (numpy/trimesh/OpenCV, no simulator).

Used by tools/cad/make_inspire_e2_hand.py to build the Inspire RH56E2 hand and to render colour previews.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np


def rpy_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis, ang) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


def homog(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _vec(s, n=3, default=0.0):
    return np.array([float(v) for v in s.split()]) if s else np.full(n, default)


@dataclass
class Link:
    name: str
    mass: float = 0.0
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    com_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    mesh: str | None = None            # visual mesh file (absolute)
    mesh_origin: np.ndarray = field(default_factory=lambda: np.eye(4))
    rgba: tuple | None = None


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray                 # 4x4 parent -> joint frame
    axis: np.ndarray
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0
    mimic: tuple | None = None         # (joint, multiplier, offset)


def parse_urdf(path: str, mesh_dir: str) -> tuple[dict, dict]:
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        L = Link(l.get("name"))
        ine = l.find("inertial")
        if ine is not None:
            o = ine.find("origin")
            L.mass = float(ine.find("mass").get("value"))
            L.com = _vec(o.get("xyz")) if o is not None else np.zeros(3)
            L.com_rpy = _vec(o.get("rpy")) if o is not None else np.zeros(3)
            i = ine.find("inertia")
            g = {k: float(i.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
            L.inertia = np.array([[g["ixx"], g["ixy"], g["ixz"]], [g["ixy"], g["iyy"], g["iyz"]], [g["ixz"], g["iyz"], g["izz"]]])
        v = l.find("visual")
        if v is not None and v.find("geometry/mesh") is not None:
            L.mesh = os.path.join(mesh_dir, os.path.basename(v.find("geometry/mesh").get("filename")))
            o = v.find("origin")
            if o is not None:
                L.mesh_origin = homog(rpy_matrix(_vec(o.get("rpy"))), _vec(o.get("xyz")))
            c = v.find("material/color")
            if c is not None:
                L.rgba = tuple(_vec(c.get("rgba"), 4))
        links[L.name] = L
    for j in root.findall("joint"):
        o = j.find("origin")
        T = homog(rpy_matrix(_vec(o.get("rpy") if o is not None else None)), _vec(o.get("xyz") if o is not None else None))
        a = j.find("axis")
        J = Joint(j.get("name"), j.get("type"), j.find("parent").get("link"), j.find("child").get("link"), T,
                  _vec(a.get("xyz")) if a is not None else np.array([1.0, 0, 0]))
        lim = j.find("limit")
        if lim is not None:
            J.lower, J.upper = float(lim.get("lower", 0)), float(lim.get("upper", 0))
            J.effort, J.velocity = float(lim.get("effort", 0)), float(lim.get("velocity", 0))
        m = j.find("mimic")
        if m is not None:
            J.mimic = (m.get("joint"), float(m.get("multiplier", 1)), float(m.get("offset", 0)))
        joints[J.name] = J
    return links, joints


def root_link(links: dict, joints: dict) -> str:
    children = {j.child for j in joints.values()}
    roots = [n for n in links if n not in children]
    assert len(roots) == 1, roots
    return roots[0]


def resolve_q(joints: dict, q: dict) -> dict:
    """Fill mimic joints (chains allowed) from the active ones; missing active joints = 0."""
    out = {n: float(q.get(n, 0.0)) for n, j in joints.items() if j.type != "fixed" and j.mimic is None}
    for _ in range(len(joints)):
        for n, j in joints.items():
            if j.mimic and n not in out and j.mimic[0] in out:
                out[n] = out[j.mimic[0]] * j.mimic[1] + j.mimic[2]
    return out


def fk(links: dict, joints: dict, q: dict | None = None) -> dict:
    """Link frames in the root link frame."""
    qq = resolve_q(joints, q or {})
    T = {root_link(links, joints): np.eye(4)}
    pending = list(joints.values())
    while pending:
        rest = []
        for j in pending:
            if j.parent not in T:
                rest.append(j)
                continue
            M = T[j.parent] @ j.origin
            if j.type in ("revolute", "continuous"):
                M = M @ homog(axis_angle(j.axis, qq.get(j.name, 0.0)), np.zeros(3))
            elif j.type == "prismatic":
                M = M @ homog(np.eye(3), j.axis * qq.get(j.name, 0.0))
            T[j.child] = M
        assert len(rest) < len(pending), "disconnected joints"
        pending = rest
    return T


# ---------------------------------------------------------------- rendering
def look_rotation(view_dir, up) -> np.ndarray:
    """Rows = camera x (right), y (down), z (forward = view_dir) in the model frame."""
    z = np.asarray(view_dir, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z])


def render(parts, view_dir, up, size=(900, 900), px_per_m=3500.0, center=None, bg=(255, 255, 255),
           light=(-0.4, -0.6, -1.0)) -> np.ndarray:
    """parts: list of (V (n,3), F (m,3), face_rgb (m,3) floats 0..1). Orthographic, painter's algorithm, Lambert."""
    import cv2

    R = look_rotation(view_dir, up)
    allv = np.concatenate([p[0] for p in parts])
    c = allv.mean(0) if center is None else np.asarray(center)
    W, H = size
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = bg
    tris, depth, cols = [], [], []
    Lw = np.asarray(light, float)
    Lw /= np.linalg.norm(Lw)
    Lc = R @ Lw
    for V, F, C in parts:
        P = (V - c) @ R.T
        tri = P[F]                                       # (m,3,3)
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        nn = np.linalg.norm(n, axis=1, keepdims=True)
        n = n / np.maximum(nn, 1e-12)
        shade = 0.35 + 0.65 * np.abs(n @ -Lc)            # two-sided
        spec = np.clip(n @ -Lc, 0, 1) ** 24 * 0.25
        col = np.clip(C * shade[:, None] + spec[:, None], 0, 1)
        tris.append(tri)
        depth.append(tri[:, :, 2].mean(1))
        cols.append(col)
    tri = np.concatenate(tris)
    d = np.concatenate(depth)
    col = np.concatenate(cols)
    order = np.argsort(-d)                               # far first
    pix = np.stack([tri[:, :, 0] * px_per_m + W / 2, tri[:, :, 1] * px_per_m + H / 2], -1)
    pix = np.round(pix * 4).astype(np.int32)             # 2-bit sub-pixel
    col8 = (col[:, ::-1] * 255).astype(np.uint8)         # BGR
    for i in order:
        cv2.fillConvexPoly(img, pix[i], tuple(int(v) for v in col8[i]), lineType=cv2.LINE_AA, shift=2)
    return img
