"""Convert STEP parts to STL (mm, CAD frame preserved) and print bounding boxes. Usage: step_to_stl.py OUT_DIR a.step ..."""
import os, sys
import gmsh
out = sys.argv[1]
os.makedirs(out, exist_ok=True)

def convert(path, opts):
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setString("Geometry.OCCTargetUnit", "MM")
    for k, v in opts.items():
        gmsh.option.setNumber(k, v)
    gmsh.model.occ.importShapes(path)
    gmsh.model.occ.synchronize()
    bb = gmsh.model.getBoundingBox(-1, -1)
    vols = gmsh.model.getEntities(3)
    try:
        gmsh.model.mesh.generate(2)
        name = os.path.splitext(os.path.basename(path))[0]
        gmsh.write(os.path.join(out, name + ".stl"))
        ok = True
    except Exception as e:
        ok = str(e)
    gmsh.finalize()
    return bb, len(vols), ok

for path in sys.argv[2:]:
    name = os.path.splitext(os.path.basename(path))[0]
    for opts in ({"Mesh.MeshSizeMax": 2.0}, {"Mesh.MeshSizeMax": 1.0, "Mesh.Algorithm": 1},
                 {"Mesh.MeshSizeMax": 3.0, "Mesh.Algorithm": 5, "Geometry.OCCFixDegenerated": 1, "Geometry.OCCFixSmallEdges": 1}):
        bb, nv, ok = convert(path, opts)
        if ok is True:
            break
    print(f"{name:45s} vols={nv} mesh={'ok' if ok is True else 'FAIL'} bbox_mm min={[round(v,1) for v in bb[:3]]} max={[round(v,1) for v in bb[3:]]}", flush=True)
