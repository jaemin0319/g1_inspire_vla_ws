"""Compose the G1 29-DoF (rev 1.0) wholebody robot with Inspire RH56E2 hands (docs/HAND_RH56E2.md).

    build_inspire_e2_usd.py [--official USD] [--hands NPZ] [--out USD] [--colorway dftp]

The output layer sub-layers Unitree's official G1 + Inspire USD (unitree_sim_isaaclab, untouched) and overrides the
24 hand links IN PLACE: same prim paths, link and joint names and articulation order, so the official joint index
tables, the DDS mapping and the recorded data keep working. Per hand link it
  - deactivates the official 'visuals' / 'collisions' children and adds 'visuals_rh56e2' (one mesh per colour label,
    bound to <robot>/Looks/rh56e2_<label>) and 'collisions_rh56e2' (convex hull, like the official convexHull colliders),
  - sets the link pose at q = 0, mass, centre of mass and principal inertia from Inspire's URDF,
and per hand joint it sets the joint frames (axis = joint Z), limits, and the wrist -> hand mount (palm joint).
Run with the simulator env (isaacsim). Default paths from configs/hand_rh56e2.yaml.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OFFICIAL = os.path.join(WS, "repos/unitree_sim_isaaclab/assets/robots/g1-29dof_wholebody_inspire/g1_29dof_with_inspire_rev_1_0.usd")
CFG = yaml.safe_load(open(os.path.join(WS, "configs/hand_rh56e2.yaml")))

ap = argparse.ArgumentParser()
ap.add_argument("--official", default=OFFICIAL)
ap.add_argument("--hands", default=os.path.join(WS, CFG["hands_npz"]))
ap.add_argument("--out", default=os.path.join(WS, CFG["usd"]))
ap.add_argument("--colorway", default=CFG["colorway"])
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: E402


def quat_wxyz(R: np.ndarray) -> tuple:
    """Rotation matrix (column vectors) -> unit quaternion (w, x, y, z)."""
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    q = np.array(q)
    return tuple(q / np.linalg.norm(q))


def z_to(a: np.ndarray) -> np.ndarray:
    """Rotation taking +Z onto the unit vector a."""
    a = a / np.linalg.norm(a)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, a)
    c = float(np.dot(z, a))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * (1.0 / (1.0 + c))


def gf_quat(attr, q):
    tn = str(attr.GetTypeName())
    return Gf.Quatd(*q) if tn == "quatd" else Gf.Quatf(*q)


def gf_vec(attr, v):
    tn = str(attr.GetTypeName())
    return Gf.Vec3d(*map(float, v)) if tn in ("double3",) else Gf.Vec3f(*map(float, v))


def link_prim_name(side: str, key: str) -> str:
    return f"{side}_hand_base_link" if key == "hand_base_link" else f"{'R' if side == 'right' else 'L'}_{key}"


def main():
    H = np.load(args.hands)
    meta = json.loads(str(H["meta"]))
    src = Usd.Stage.Open(args.official, Usd.Stage.LoadAll)
    root = src.GetDefaultPrim()
    rname = root.GetName()
    T_wrist = {}
    for side in ("right", "left"):
        M = UsdGeom.Xformable(root.GetChild(f"{side}_wrist_yaw_link")).GetLocalTransformation()
        T_wrist[side] = np.array(M).T
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    if os.path.exists(args.out):
        os.remove(args.out)          # generated file (data/assets, git-ignored); rebuilt from the npz every time
    layer = Sdf.Layer.CreateNew(args.out)
    layer.subLayerPaths.append(os.path.relpath(args.official, os.path.dirname(args.out)))
    layer.defaultPrim = rname
    stage = Usd.Stage.Open(layer)
    stage.SetEditTarget(layer)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(src))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src))
    layer.customLayerData = {"g1iv": {"hands": "Inspire RH56E2 (RH56DFTP family)", "source_npz": os.path.relpath(args.hands, WS),
                                      "colorway": args.colorway, "built_by": "tools/build_inspire_e2_usd.py"}}
    rp = Sdf.Path(f"/{rname}")
    # ---- materials
    colors = CFG["colorways"][args.colorway]
    mats = {}
    for lab in meta["labels"]:
        r, g, b, metal, rough = colors[lab]
        mp = rp.AppendPath(f"Looks/rh56e2_{lab}")
        mat = UsdShade.Material.Define(stage, mp)
        sh = UsdShade.Shader.Define(stage, mp.AppendChild("Shader"))
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(r, g, b))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metal))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        mats[lab] = mat
    report = {"links": 0, "meshes": 0, "faces": 0, "joints": 0}
    for side in ("right", "left"):
        for key in meta["links"]:
            p = f"{side}/{key}/"
            path = rp.AppendChild(link_prim_name(side, key))
            prim = stage.GetPrimAtPath(path)
            assert prim and prim.IsValid(), f"missing link {path}"
            for ch in ("visuals", "collisions"):
                c = prim.GetChild(ch)
                assert c and c.IsValid(), f"missing {path}/{ch}"
                c.SetActive(False)
            T = T_wrist[side] @ H[p + "T"]
            ta, oa = prim.GetAttribute("xformOp:translate"), prim.GetAttribute("xformOp:orient")
            ta.Set(gf_vec(ta, T[:3, 3]))
            oa.Set(gf_quat(oa, quat_wxyz(T[:3, :3])))
            vis = UsdGeom.Xform.Define(stage, path.AppendChild("visuals_rh56e2"))
            for lab in meta["labels"]:
                if p + f"S/{lab}/V" not in H.files:
                    continue
                V, F, N = H[p + f"S/{lab}/V"], H[p + f"S/{lab}/F"], H[p + f"S/{lab}/N"]
                m = UsdGeom.Mesh.Define(stage, vis.GetPath().AppendChild(lab))
                m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V.astype(np.float32)))
                m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
                m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1).astype(np.int32)))
                m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(N.astype(np.float32)))
                m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
                m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
                m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, V.min(0))), Gf.Vec3f(*map(float, V.max(0)))]))
                UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mats[lab])
                report["meshes"] += 1
                report["faces"] += len(F)
            col = UsdGeom.Xform.Define(stage, path.AppendChild("collisions_rh56e2"))
            col.CreatePurposeAttr(UsdGeom.Tokens.guide)
            hull = UsdGeom.Mesh.Define(stage, col.GetPath().AppendChild("hull"))
            HV, HF = H[p + "HV"], H[p + "HF"]
            hull.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(HV.astype(np.float32)))
            hull.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(HF), 3, np.int32)))
            hull.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(HF.reshape(-1).astype(np.int32)))
            hull.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(hull.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(hull.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
            # mass properties (link frame)
            I = H[p + "I"]
            w, Vv = np.linalg.eigh(I)
            if np.linalg.det(Vv) < 0:
                Vv[:, 0] *= -1
            prim.GetAttribute("physics:mass").Set(float(H[p + "mass"]))
            prim.GetAttribute("physics:centerOfMass").Set(Gf.Vec3f(*map(float, H[p + "com"])))
            prim.GetAttribute("physics:diagonalInertia").Set(Gf.Vec3f(*map(float, w)))
            prim.GetAttribute("physics:principalAxes").Set(Gf.Quatf(*quat_wxyz(Vv)))
            report["links"] += 1
        # ---- joints
        pre = "R" if side == "right" else "L"
        rng = CFG["ranges"]
        cpl = CFG["couplings"]
        for j in meta["joints"][side]:
            jp = stage.GetPrimAtPath(rp.AppendPath(f"joints/{pre}_{j['name']}"))
            assert jp and jp.IsValid(), j["name"]
            b0 = UsdPhysics.Joint(jp).GetBody0Rel().GetTargets()[0].name
            b1 = UsdPhysics.Joint(jp).GetBody1Rel().GetTargets()[0].name
            assert (b0, b1) == (link_prim_name(side, j["parent"]), link_prim_name(side, j["child"])), (j["name"], b0, b1)
            assert jp.GetAttribute("physics:axis").Get() == "Z", j["name"]
            O = np.array(j["origin"])
            A = z_to(np.array(j["axis"]))
            jp.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, O[:3, 3])))
            jp.GetAttribute("physics:localRot0").Set(Gf.Quatf(*quat_wxyz(O[:3, :3] @ A)))
            jp.GetAttribute("physics:localPos1").Set(Gf.Vec3f(0.0, 0.0, 0.0))
            jp.GetAttribute("physics:localRot1").Set(Gf.Quatf(*quat_wxyz(A)))
            n = j["name"]
            # driven joints: the configured range; coupled joints: the coupled range (Inspire's URDF limits of the
            # mimic joints are 0.3..5 % below ratio x driver range; the simulator drives them at exactly the ratio)
            if n == "thumb_proximal_yaw_joint":
                lo, hi = rng["thumb_yaw"]
            elif n == "thumb_proximal_pitch_joint":
                lo, hi = rng["thumb_pitch"]
            elif n == "thumb_intermediate_joint":
                lo, hi = 0.0, max(j["upper"], cpl["thumb_intermediate"] * rng["thumb_pitch"][1])
            elif n == "thumb_distal_joint":
                lo, hi = 0.0, max(j["upper"], cpl["thumb_distal"] * rng["thumb_pitch"][1])
            elif n.endswith("_intermediate_joint"):
                lo, hi = 0.0, max(j["upper"], cpl["finger_intermediate"] * rng["finger"][1])
            else:
                lo, hi = rng["finger"]
            jp.GetAttribute("physics:lowerLimit").Set(float(math.degrees(lo)))
            jp.GetAttribute("physics:upperLimit").Set(float(math.degrees(hi)))
            report["joints"] += 1
        # wrist -> hand mount
        pj = stage.GetPrimAtPath(rp.AppendPath(f"joints/{side}_hand_palm_joint"))
        assert UsdPhysics.Joint(pj).GetBody1Rel().GetTargets()[0].name == f"{side}_hand_base_link"
        Tm = H[f"{side}/hand_base_link/T"]
        pj.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, Tm[:3, 3])))
        pj.GetAttribute("physics:localRot0").Set(Gf.Quatf(*quat_wxyz(Tm[:3, :3])))
        pj.GetAttribute("physics:localPos1").Set(Gf.Vec3f(0.0, 0.0, 0.0))
        pj.GetAttribute("physics:localRot1").Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    layer.Save()
    print("[build_inspire_e2_usd] wrote", args.out, f"{os.path.getsize(args.out) / 1e6:.1f} MB", json.dumps(report), flush=True)
    # ---- verification on the composed stage
    chk = Usd.Stage.Open(args.out, Usd.Stage.LoadAll)
    r = chk.GetDefaultPrim()
    joints = [p.GetName() for p in r.GetChild("joints").GetChildren()]
    src_joints = [p.GetName() for p in root.GetChild("joints").GetChildren()]
    links = [c.GetName() for c in r.GetChildren()]
    bad = [n for n in links if (n.startswith(("L_", "R_")) or n.endswith("_hand_base_link"))
           and (r.GetChild(n).GetChild("visuals").IsActive() or not r.GetChild(n).GetChild("visuals_rh56e2").IsValid())]
    out = {"joint_order_unchanged": joints == src_joints, "link_order_unchanged": links[:len(src.GetDefaultPrim().GetChildren())]
           == [c.GetName() for c in root.GetChildren()], "links_not_replaced": bad,
           "hand_mass_kg": {s: round(sum(float(r.GetChild(link_prim_name(s, k)).GetAttribute("physics:mass").Get())
                                         for k in meta["links"]), 4) for s in ("right", "left")}}
    print("[build_inspire_e2_usd] check", json.dumps(out), flush=True)
    sys.stdout.flush()
    os._exit(0 if out["joint_order_unchanged"] and not bad else 1)   # simulation_app.close() can hang (known)


main()
