"""Build the Inspire RH56E2 (= RH56DFTP family) hand pair for the G1 simulator from Inspire's URDF.

    make_inspire_e2_hand.py E2_R_DIR OUT.npz [--left-ref E2_L_DIR] [--unitree-ftp DIR] [--preview OUT.png]

E2_R_DIR      Inspire RH56E2 right-hand URDF package (RH56E2_R_2026_1_5: urdf/, meshes/)
--left-ref    Inspire RH56E2 left-hand package (RH56E2_L_2025_9_10) -> checks that left = mirror(right)
--unitree-ftp unitree_ros g1_description FTP hand (inspire_hand/FTP_right_hand.urdf + meshes/) -> checks the mount
--preview     colour preview (back / palm / side) for comparison with Inspire's product photos

Output (npz, metres, every array in its LINK frame unless noted):
  meta                    json: sources, links, joints (DFX names), mount, label names, checks
  {side}/{link}/V,F,L     visual mesh (float32, int32, uint8 colour label per face)
  {side}/{link}/HV,HF     convex hull of the visual mesh (collision)
  {side}/{link}/T         4x4 link pose in the <side>_wrist_yaw_link frame at q = 0
  {side}/{link}/mass,com,I

Link and joint names follow the official G1 Inspire (RH56DFX-style) asset so the articulation, the DDS mapping
and the recorded data keep their names: E2 X_1 -> X_proximal, X_2 -> X_intermediate, little -> pinky, thumb_1..4 ->
thumb_proximal_base / thumb_proximal / thumb_intermediate / thumb_distal (joints thumb_proximal_yaw / _pitch /
thumb_intermediate / thumb_distal). Tactile pads, palm casing and palm pad are merged into their parent link.
The left hand is the right hand mirrored in the wrist frame (y -> -y), verified against Inspire's left URDF.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np
import trimesh
from trimesh.graph import connected_components

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from urdf_mesh_model import fk, homog, parse_urdf, render  # noqa: E402

LABELS = ["black", "shell", "cover", "pad", "metal", "lens"]
BLACK, SHELL, COVER, PAD, METAL, LENS = range(6)
FINGERS = {"index": "index", "middle": "middle", "ring": "ring", "little": "pinky"}

# wrist_yaw_link -> E2 base frame. Unitree mounts its FTP hand at x = 41.5 mm (hand flange) with rpy (0, pi/2, 0)
# (unitree_ros g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf); Inspire's E2 base frame is the Unitree FTP base frame
# turned +90 deg about the hand axis (index at +y instead of -x), checked against the FTP base mesh below.
MOUNT_XYZ = (0.0415, 0.0, 0.0)


def mount_right() -> np.ndarray:
    ry = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)      # Ry(+90 deg)
    rz = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)      # Rz(+90 deg)
    return homog(ry @ rz, MOUNT_XYZ)


def components(m: trimesh.Trimesh) -> list[np.ndarray]:
    cc = connected_components(m.face_adjacency, nodes=np.arange(len(m.faces)), min_len=1)
    return sorted(cc, key=lambda c: -len(c))


def label_faces(e2_link: str, m: trimesh.Trimesh) -> np.ndarray:
    """Colour labels from Inspire's photos of the RH56E2/RH56DFTP (docs/HAND_RH56E2.md): white back cover, white
    finger/thumb shells with black fingertip caps, black housing / palm / finger undersides, black tactile pads,
    aluminium flange, hinge brackets, pins and screws, round status light on the back. Link-frame rules (mm)."""
    C = m.triangles_center * 1000.0
    N = m.face_normals
    lab = np.full(len(m.faces), BLACK, np.uint8)
    if "force_sensor" in e2_link:
        lab[:] = PAD
        return lab
    if e2_link.endswith("plam_casing"):
        lab[:] = COVER
        return lab
    comps = components(m)
    if e2_link.endswith("base_link"):
        for i, c in enumerate(comps):
            ext = np.ptp(m.vertices[np.unique(m.faces[c])], 0) * 1000
            if i == 0:
                lab[c] = BLACK                                   # housing (palm, sides, lower back)
            elif i == 1:
                lab[c] = METAL                                   # inner frame: wrist ring + knuckle hinge forks
            elif abs(ext[0] - 50) < 1 and abs(ext[1] - 50) < 1 and ext[2] > 30:
                lab[c] = METAL                                   # 50 mm mounting flange
            elif len(c) > 1500:
                lab[c] = BLACK                                   # internal boxes / linear actuators (hidden)
            else:
                lab[c] = METAL                                   # pins, screws, small linkages
        # status light on the back: raised disc at y = 0, z = 52.5 mm, x < -24 mm (ring = metal, centre = lens)
        r = np.hypot(C[:, 1], C[:, 2] - 52.5)
        back = (C[:, 0] < -24.0) & np.isin(np.arange(len(lab)), comps[0])
        lab[back & (r < 7.5)] = METAL
        lab[back & (r < 4.6)] = LENS
        return lab
    if "thumb_1" in e2_link:
        lab[:] = METAL                                           # thumb rotation carrier inside the palm
        return lab
    if "thumb_" in e2_link:
        k = int(e2_link[-1])
        rules = {2: {0: BLACK, 1: BLACK, 2: SHELL, 3: SHELL},    # big proximal: black body, white end plates
                 3: {0: SHELL, 1: SHELL, 2: BLACK},              # middle: white side plates, black core
                 4: {0: BLACK, 1: SHELL}}[k]                     # tip: black cap, white collar
        for i, c in enumerate(comps):
            lab[c] = rules.get(i, METAL if len(c) < 1500 else BLACK)
        return lab
    # finger segments (one closed shell each). Link frame: x palmar(+)/dorsal(-), y along the finger, z lateral.
    dorsal = N[:, 0] < -0.3
    side = (np.abs(N[:, 0]) <= 0.3) & (C[:, 0] < -5.0)
    lab[dorsal | side] = SHELL
    ymax = C[:, 1].max()
    if e2_link.endswith("_2"):
        lab[C[:, 1] > ymax - 18.5] = BLACK                       # black fingertip cap (photo: ~1/3 of the segment)
    lab[(C[:, 1] < 3.0) & (C[:, 0] > 0.0)] = METAL               # hinge at the joint (palm side)
    if e2_link.endswith("_1"):
        lab[(C[:, 1] > ymax - 6.0) & (C[:, 0] > 0.0)] = METAL    # hinge to the distal segment
    return lab


def merge_inertia(parts):
    """parts: (mass, com (3,), I (3,3) about com, in the common frame)."""
    M = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / M
    I = np.zeros((3, 3))
    for m, c, Ic in parts:
        d = c - com
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return M, com, I


def package_urdf(pkg: str) -> str:
    """The package's URDF (urdf/<name>.urdf; Inspire's '_abs' variant only differs in mesh paths)."""
    c = sorted(f for f in os.listdir(os.path.join(pkg, "urdf")) if f.endswith(".urdf") and not f.endswith("_abs.urdf"))
    assert len(c) == 1, c
    return os.path.join(pkg, "urdf", c[0])


def build_right(e2_dir: str):
    links, joints = parse_urdf(package_urdf(e2_dir), os.path.join(e2_dir, "meshes"))
    T0 = fk(links, joints)
    child_of = {j.child: j for j in joints.values()}
    base = [n for n in links if n not in child_of][0]

    def dfx(e2: str) -> str:
        if e2 == base:
            return "hand_base_link"
        f, k = e2.split("_")[1], e2.split("_")[-1]
        if f == "thumb":
            return {"1": "thumb_proximal_base", "2": "thumb_proximal", "3": "thumb_intermediate", "4": "thumb_distal"}[k]
        return f"{FINGERS[f]}_{'proximal' if k == '1' else 'intermediate'}"

    # fixed children (pads, casing) -> the moving link they are bolted to
    owner = {}
    for n in links:
        o = n
        while o in child_of and child_of[o].type == "fixed":
            o = child_of[o].parent
        owner[n] = o
    out = {}
    for main in sorted({owner[n] for n in links}):
        key = dfx(main)
        Tm_inv = np.linalg.inv(T0[main])
        V, F, L, inert = [], [], [], []
        off = 0
        for n in sorted(n for n in links if owner[n] == main):
            Lk = links[n]
            Tn = Tm_inv @ T0[n]
            if Lk.mesh:
                m = trimesh.load(Lk.mesh, process=True)
                v = (Tn @ Lk.mesh_origin @ np.c_[m.vertices, np.ones(len(m.vertices))].T).T[:, :3]
                V.append(v)
                F.append(m.faces + off)
                L.append(label_faces(n, m))
                off += len(v)
            if Lk.mass > 0:
                from urdf_mesh_model import rpy_matrix
                Rc = Tn[:3, :3] @ rpy_matrix(Lk.com_rpy)
                inert.append((Lk.mass, (Tn @ np.r_[Lk.com, 1])[:3], Rc @ Lk.inertia @ Rc.T))
        mesh = trimesh.Trimesh(np.concatenate(V), np.concatenate(F), process=False)
        hull = mesh.convex_hull
        mass, com, I = merge_inertia(inert)
        out[key] = dict(e2=main, V=mesh.vertices, F=mesh.faces, L=np.concatenate(L), HV=hull.vertices, HF=hull.faces,
                        T0=T0[main], mass=mass, com=com, I=I)
    js = []
    for j in joints.values():
        if j.type == "fixed":
            continue
        pj, cj = dfx(owner[j.parent]), dfx(j.child)
        nm = cj.replace("_base", "")
        if cj.startswith("thumb"):
            nm = {"thumb_proximal_base": "thumb_proximal_yaw", "thumb_proximal": "thumb_proximal_pitch",
                  "thumb_intermediate": "thumb_intermediate", "thumb_distal": "thumb_distal"}[cj]
        origin = np.linalg.inv(T0[owner[j.parent]]) @ T0[j.parent] @ j.origin   # parent LINK frame -> joint frame
        js.append(dict(name=nm + "_joint", e2=j.name, parent=pj, child=cj, origin=origin.tolist(),
                       axis=(j.axis / np.linalg.norm(j.axis)).tolist(), lower=j.lower, upper=j.upper,
                       mimic=None if j.mimic is None else [j.mimic[0], j.mimic[1], j.mimic[2]]))
    e2_to_dfx = {j["e2"]: j["name"] for j in js}
    for j in js:
        if j["mimic"]:
            j["mimic"][0] = e2_to_dfx[j["mimic"][0]]
    return out, js, links, joints, base


def mirror_side(right: dict, js: list) -> tuple[dict, list]:
    """Left hand = right hand mirrored in the wrist frame (y -> -y). Proper link frames R' = M R M."""
    M = np.diag([1.0, -1.0, 1.0])
    M4 = np.diag([1.0, -1.0, 1.0, 1.0])
    left = {}
    for k, d in right.items():
        left[k] = dict(d, V=d["V"] @ M, F=d["F"][:, ::-1].copy(), HV=d["HV"] @ M, HF=d["HF"][:, ::-1].copy(),
                       T=M4 @ d["T"] @ M4, com=M @ d["com"], I=M @ d["I"] @ M)
    jl = []
    for j in js:
        jl.append(dict(j, origin=(M4 @ np.array(j["origin"]) @ M4).tolist(), axis=(-M @ np.array(j["axis"])).tolist()))
    return left, jl


def check_unitree_mount(right: dict, ftp_dir: str) -> dict:
    """E2 base (with casing) vs Unitree's FTP base, both in the right wrist_yaw_link frame."""
    from scipy.spatial import cKDTree
    f = trimesh.load(os.path.join(ftp_dir, "meshes", "right_base_link.STL"))
    Tf = homog(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float), MOUNT_XYZ)
    pf = (Tf @ np.c_[f.vertices, np.ones(len(f.vertices))].T).T[:, :3]
    d = right["hand_base_link"]
    pe = (d["T"] @ np.c_[d["V"], np.ones(len(d["V"]))].T).T[:, :3]
    a = cKDTree(pe).query(pf)[0]
    return {"ftp_base_to_e2_median_mm": float(np.median(a) * 1000), "ftp_base_to_e2_p95_mm": float(np.percentile(a, 95) * 1000)}


def check_left(left: dict, l_dir: str) -> dict:
    from scipy.spatial import cKDTree
    links, joints = parse_urdf(package_urdf(l_dir), os.path.join(l_dir, "meshes"))
    T = fk(links, joints)
    ref = []
    for n, L in links.items():
        if L.mesh:
            m = trimesh.load(L.mesh)
            ref.append((T[n] @ L.mesh_origin @ np.c_[m.vertices, np.ones(len(m.vertices))].T).T[:, :3])
    ref = np.concatenate(ref)
    ours = np.concatenate([(d["T"] @ np.c_[d["V"], np.ones(len(d["V"]))].T).T[:, :3] for d in left.values()])
    # Inspire's left package: flange frame flipped (rpy pi about x) -> bring both to the hand frame by a rigid fit
    ours_h = (np.linalg.inv(left["hand_base_link"]["T"]) @ np.c_[ours, np.ones(len(ours))].T).T[:, :3]
    best = None
    for Rf in (np.eye(3), np.diag([1, -1, -1]), np.diag([-1, 1, -1]), np.diag([-1, -1, 1])):
        d = cKDTree(ref @ Rf.T).query(ours_h[::7])[0]
        if best is None or np.median(d) < best[0]:
            best = (float(np.median(d)), float(np.percentile(d, 99)), Rf)
    return {"left_ref_median_mm": best[0] * 1000, "left_ref_p99_mm": best[1] * 1000}


def preview(hand: dict, path: str, colors: dict):
    import cv2
    pal = np.array([colors[n] for n in LABELS], float)
    parts = []
    for d in hand.values():
        v = (d["T"] @ np.c_[d["V"], np.ones(len(d["V"]))].T).T[:, :3]
        parts.append((v, d["F"], pal[d["L"]]))
    # wrist frame: x = hand axis, right hand palm towards -z? -> views from the E2 base axes mapped by the mount
    Tm = hand["hand_base_link"]["T"]
    ex, ez = Tm[:3, 0], Tm[:3, 2]                          # E2 x = palm normal, z = hand axis
    views = [(ex, ez), (-ex, ez), (np.cross(ez, ex), ez)]
    ims = [render(parts, v, u, size=(560, 860), px_per_m=3300, bg=(235, 235, 235)) for v, u in views]
    cv2.imwrite(path, np.hstack(ims))


DEFAULT_COLORS = {"black": (0.04, 0.04, 0.045), "shell": (0.92, 0.92, 0.92), "cover": (0.94, 0.94, 0.94),
                  "pad": (0.02, 0.02, 0.02), "metal": (0.74, 0.75, 0.77), "lens": (0.88, 0.88, 0.88)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("e2_right")
    ap.add_argument("out")
    ap.add_argument("--left-ref")
    ap.add_argument("--unitree-ftp")
    ap.add_argument("--preview")
    a = ap.parse_args()
    right, js, links, joints, base = build_right(a.e2_right)
    Tm = mount_right()
    for d in right.values():
        d["T"] = Tm @ d["T0"]
    left, jl = mirror_side(right, js)
    checks = {}
    if a.unitree_ftp:
        checks.update(check_unitree_mount(right, a.unitree_ftp))
    if a.left_ref:
        checks.update(check_left(left, a.left_ref))
    # thumb on the palm side and towards the body midline like the official asset (right: +y, +z in the wrist frame)
    jt = next(j for j in js if j["name"] == "thumb_proximal_yaw_joint")
    checks["thumb_yaw_joint_in_wrist_mm"] = (Tm @ np.array(jt["origin"]))[:3, 3].round(5).tolist()
    checks["mass_kg"] = float(sum(d["mass"] for d in right.values()))
    print(json.dumps(checks, indent=1))
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    src = os.path.abspath(a.e2_right)                  # workspace-relative when inside it: identical npz on every PC
    src = os.path.relpath(src, ws) if src.startswith(ws + os.sep) else src
    meta = dict(source=src, labels=LABELS, links=sorted(right), joints={"right": js, "left": jl},
                mount_right=Tm.tolist(), checks=checks,
                e2_links={k: d["e2"] for k, d in right.items()})
    arrs = {"meta": np.array(json.dumps(meta))}
    for side, hand in (("right", right), ("left", left)):
        for k, d in hand.items():
            p = f"{side}/{k}/"
            arrs[p + "V"] = d["V"].astype(np.float32)
            arrs[p + "F"] = d["F"].astype(np.int32)
            arrs[p + "L"] = d["L"].astype(np.uint8)
            arrs[p + "HV"] = d["HV"].astype(np.float32)
            arrs[p + "HF"] = d["HF"].astype(np.int32)
            arrs[p + "T"] = d["T"]
            # render meshes: one per colour label, vertices split at creases > 35 deg (smooth curved shells, sharp
            # CAD edges) with per-vertex normals -> the USD builder writes them as they are
            for li, lname in enumerate(LABELS):
                sel = np.nonzero(d["L"] == li)[0]
                if len(sel) == 0:
                    continue
                sub = trimesh.Trimesh(d["V"], d["F"][sel], process=False)
                sub.remove_unreferenced_vertices()
                sm = trimesh.graph.smooth_shade(sub, angle=np.radians(35.0), facet_minarea=None)
                arrs[p + f"S/{lname}/V"] = sm.vertices.astype(np.float32)
                arrs[p + f"S/{lname}/F"] = sm.faces.astype(np.int32)
                arrs[p + f"S/{lname}/N"] = sm.vertex_normals.astype(np.float32)
            arrs[p + "mass"] = np.array(d["mass"])
            arrs[p + "com"] = d["com"]
            arrs[p + "I"] = d["I"]
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    np.savez_compressed(a.out, **arrs)
    print("wrote", a.out, f"{os.path.getsize(a.out) / 1e6:.1f} MB")
    if a.preview:
        preview(right, a.preview, DEFAULT_COLORS)
        print("preview", a.preview)


if __name__ == "__main__":
    main()
