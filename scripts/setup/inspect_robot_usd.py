"""List joints (type, limits, drive, mimic) and links of the official G1+Inspire wholebody USD.

Usage: python scripts/setup/inspect_robot_usd.py <usd> --out logs/setup/robot_usd_joints.json
Uses only pxr (bundled with Isaac Sim); opens the layer read-only and never saves.
"""
import argparse
import json
import math

ap = argparse.ArgumentParser()
ap.add_argument("usd")
ap.add_argument("--out", required=True)
a = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics  # noqa: E402

stage = Usd.Stage.Open(a.usd, Usd.Stage.LoadAll)
joints, links, mimic = [], [], []
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        j = UsdPhysics.RevoluteJoint(prim) if prim.IsA(UsdPhysics.RevoluteJoint) else UsdPhysics.PrismaticJoint(prim)
        lo, hi = j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get()
        rad = prim.IsA(UsdPhysics.RevoluteJoint)
        d = {"name": prim.GetName(), "path": str(prim.GetPath()), "type": "revolute" if rad else "prismatic",
             "lower": (math.radians(lo) if (rad and lo is not None) else lo),
             "upper": (math.radians(hi) if (rad and hi is not None) else hi),
             "body0": [str(t) for t in j.GetBody0Rel().GetTargets()],
             "body1": [str(t) for t in j.GetBody1Rel().GetTargets()],
             "apis": [str(s) for s in prim.GetAppliedSchemas()]}
        drv = UsdPhysics.DriveAPI.Get(prim, "angular" if rad else "linear")
        if drv:
            d["drive"] = {"stiffness": drv.GetStiffnessAttr().Get(), "damping": drv.GetDampingAttr().Get(),
                          "max_force": drv.GetMaxForceAttr().Get()}
        joints.append(d)
        if any("Mimic" in s for s in d["apis"]):
            attrs = {at.GetName(): at.Get() for at in prim.GetAttributes() if "mimic" in at.GetName().lower()}
            rels = {r.GetName(): [str(t) for t in r.GetTargets()] for r in prim.GetRelationships() if "mimic" in r.GetName().lower()}
            mimic.append({"joint": prim.GetName(), "attrs": {k: (v if isinstance(v, (int, float, str, bool)) else str(v)) for k, v in attrs.items()}, "rels": rels})
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        m = UsdPhysics.MassAPI(prim)
        links.append({"name": prim.GetName(), "path": str(prim.GetPath()),
                      "mass": (m.GetMassAttr().Get() if m else None)})
out = {"usd": a.usd, "n_joints": len(joints), "n_links": len(links), "joints": joints, "links": links, "mimic": mimic}
with open(a.out, "w") as f:
    json.dump(out, f, indent=1, default=str)
print(f"joints={len(joints)} links={len(links)} mimic={len(mimic)}")
app.close()
import os  # noqa: E402

os._exit(0)
