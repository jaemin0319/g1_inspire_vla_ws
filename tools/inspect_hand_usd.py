"""Dump the USD structure of one Inspire hand in a G1 robot USD: link prims (xform, schemas, children, mass) and
hand joints (bodies, local frames, axis, limits, drive). Read-only. Usage: inspect_hand_usd.py ROBOT.usd [R|L]"""
import sys
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics  # noqa: E402

stage = Usd.Stage.Open(sys.argv[1], Usd.Stage.LoadAll)
side = sys.argv[2] if len(sys.argv) > 2 else "R"
full = "right" if side == "R" else "left"
root = stage.GetDefaultPrim()
print("defaultPrim", root.GetPath(), "schemas", root.GetAppliedSchemas())
print("root children:", [c.GetName() for c in root.GetChildren()])
keep = lambda n: n.startswith(side + "_") or n in (f"{full}_hand_base_link", f"{full}_wrist_yaw_link")
for c in root.GetChildren():
    if not keep(c.GetName()):
        continue
    attrs = {a.GetName(): a.Get() for a in c.GetAttributes() if a.GetName().startswith(("xformOp", "physics:mass", "physics:center", "physics:diag", "physics:princ"))}
    print(f"LINK {c.GetPath()} type={c.GetTypeName()} schemas={c.GetAppliedSchemas()}")
    for k, v in attrs.items():
        print(f"    {k} = {v}")
    for ch in c.GetChildren():
        print(f"    child {ch.GetName()} type={ch.GetTypeName()} inst={ch.IsInstanceable()} refs={ch.GetMetadata('references')} schemas={ch.GetAppliedSchemas()}")
for p in stage.Traverse():
    if not p.IsA(UsdPhysics.Joint):
        continue
    n = p.GetName()
    if not (n.startswith(side + "_") or n.startswith(f"{full}_hand") or n == f"{full}_wrist_yaw_joint"):
        continue
    j = UsdPhysics.Joint(p)
    print(f"JOINT {p.GetPath()} type={p.GetTypeName()} schemas={p.GetAppliedSchemas()}")
    print(f"    body0={j.GetBody0Rel().GetTargets()} body1={j.GetBody1Rel().GetTargets()}")
    for a in p.GetAttributes():
        v = a.Get()
        if v is not None:
            print(f"    {a.GetName()} = {v}")
app.close()
