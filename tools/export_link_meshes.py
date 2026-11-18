"""Export visual meshes of selected USD links to OBJ-like npz (points+faces in the LINK frame, metres). Read-only.
Usage: export_link_meshes.py ROBOT.usd OUT_DIR link1 link2 ...   (or ALL)"""
import os, sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom  # noqa: E402

usd, out, names = sys.argv[1], sys.argv[2], sys.argv[3:]
os.makedirs(out, exist_ok=True)
stage = Usd.Stage.Open(usd, Usd.Stage.LoadAll)
cache = UsdGeom.XformCache()
root = [p for p in stage.GetPseudoRoot().GetChildren()][0]
if names == ["ALL"]:   # every direct child of the robot root (links)
    names = [c.GetName() for c in root.GetChildren()]
for n in names:
    link = root.GetChild(n)
    if not link:
        print("missing", n); continue
    Tl = np.array(cache.GetLocalToWorldTransform(link)).T
    P, F, off = [], [], 0
    for p in Usd.PrimRange(link, Usd.TraverseInstanceProxies()):
        if "collisions" in str(p.GetPath()) or not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = np.array(m.GetPointsAttr().Get(), dtype=float)
        cnt = np.array(m.GetFaceVertexCountsAttr().Get()); idx = np.array(m.GetFaceVertexIndicesAttr().Get())
        Tm = np.array(cache.GetLocalToWorldTransform(p)).T
        pl = (np.linalg.inv(Tl) @ Tm @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
        tris, k = [], 0
        for c in cnt:
            for j in range(1, c - 1):
                tris.append([idx[k], idx[k + j], idx[k + j + 1]])
            k += c
        P.append(pl); F.append(np.array(tris) + off); off += len(pl)
    if P:
        P = np.vstack(P); F = np.vstack(F)
        np.savez(os.path.join(out, n + ".npz"), points=P, faces=F)
        print(f"{n:32s} verts={len(P):6d} bbox min={np.round(P.min(0), 4).tolist()} max={np.round(P.max(0), 4).tolist()}", flush=True)
sys.stdout.flush(); os._exit(0)
