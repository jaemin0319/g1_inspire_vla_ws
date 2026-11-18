"""List visual prims of the right Inspire hand links in the robot USD: type, instanceable, material bindings. Read-only."""
import sys
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdShade  # noqa: E402

stage = Usd.Stage.Open(sys.argv[1], Usd.Stage.LoadAll)
for p in stage.Traverse(Usd.TraverseInstanceProxies()):
    path = str(p.GetPath())
    name = path.split("/")[2] if path.count("/") >= 2 else ""
    if not (name.startswith("R_") or name.startswith("right_hand")):
        continue
    mb = UsdShade.MaterialBindingAPI(p)
    rel = mb.GetDirectBindingRel()
    tgt = [str(t) for t in rel.GetTargets()] if rel else []
    strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(rel) if rel and tgt else ""
    print(f"{path:70s} {p.GetTypeName():10s} inst={p.IsInstanceable()} proxy={p.IsInstanceProxy()} mat={tgt} {strength}", flush=True)
app.close()
