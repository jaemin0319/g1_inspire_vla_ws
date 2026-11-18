"""Print the shaders of the robot USD's /Looks materials (id / MDL source / inputs). Read-only."""
import os, sys
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdShade  # noqa: E402

stage = Usd.Stage.Open(sys.argv[1], Usd.Stage.LoadAll)
for p in stage.Traverse():
    if "/Looks/" not in str(p.GetPath()) or p.GetTypeName() != "Shader":
        continue
    sh = UsdShade.Shader(p)
    src = sh.GetSourceAsset("mdl") if sh.GetImplementationSource() == "sourceAsset" else None
    ins = {i.GetBaseName(): i.Get() for i in sh.GetInputs() if i.Get() is not None}
    print(p.GetPath(), "| id:", sh.GetIdAttr().Get(), "| impl:", sh.GetImplementationSource(), "| mdl:", src,
          "| sub:", p.GetAttribute("info:mdl:sourceAsset:subIdentifier").Get(), "| inputs:", ins, flush=True)
os._exit(0)
