"""Print collision/material details of the hand links and the left/right hand mount (read-only).
Usage: inspect_hand_collision.py ROBOT.usd"""
import sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdShade  # noqa: E402

st = Usd.Stage.Open(sys.argv[1], Usd.Stage.LoadAll)
r = st.GetDefaultPrim()
print("metersPerUnit", UsdGeom.GetStageMetersPerUnit(st), "upAxis", UsdGeom.GetStageUpAxis(st))
c = r.GetChild("R_index_proximal").GetChild("collisions")
for p in Usd.PrimRange(c, Usd.TraverseInstanceProxies()):
    print("COL", p.GetPath(), p.GetTypeName(), p.GetAppliedSchemas())
    for a in p.GetAttributes():
        if a.HasAuthoredValue() and a.GetName() not in ("points", "faceVertexIndices", "faceVertexCounts", "normals", "extent"):
            print("   ", a.GetName(), a.Get())
    for rel in p.GetRelationships():
        print("   rel", rel.GetName(), rel.GetTargets())
v = r.GetChild("R_index_proximal").GetChild("visuals")
for p in Usd.PrimRange(v, Usd.TraverseInstanceProxies()):
    print("VIS", p.GetPath(), p.GetTypeName(), [a.GetName() for a in p.GetAttributes() if a.HasAuthoredValue()][:12])
cache = UsdGeom.XformCache()
for s in ("right", "left"):
    W = np.array(cache.GetLocalToWorldTransform(r.GetChild(f"{s}_wrist_yaw_link"))).T
    H = np.array(cache.GetLocalToWorldTransform(r.GetChild(f"{s}_hand_base_link"))).T
    print(s, "wrist R=\n", np.round(W[:3, :3], 4), "\n hand_base in wrist:\n", np.round(np.linalg.inv(W) @ H, 5))
    j = r.GetChild("joints").GetChild(f"{s}_hand_palm_joint")
    print(" palm joint", {a.GetName(): a.Get() for a in j.GetAttributes() if "local" in a.GetName()},
          j.GetRelationship("physics:body0").GetTargets(), j.GetRelationship("physics:body1").GetTargets())
app.close()
