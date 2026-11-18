"""List cylindrical / conical / planar faces of STEP parts (OCC via gmsh): axis, radius, extent. Usage: cylinders.py a.step ..."""
import sys
import numpy as np
import gmsh
for path in sys.argv[1:]:
    gmsh.initialize(); gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.occ.importShapes(path); gmsh.model.occ.synchronize()
    print("==", path.split("/")[-1])
    rows = []
    for dim, tag in gmsh.model.getEntities(2):
        t = gmsh.model.getType(dim, tag)
        if t not in ("Cylinder", "Cone", "Plane"):
            continue
        # sample the surface on its parametric bounds
        lo, hi = gmsh.model.getParametrizationBounds(dim, tag)
        uv = np.array([[u, v] for u in np.linspace(lo[0], hi[0], 9) for v in np.linspace(lo[1], hi[1], 5)]).ravel()
        pts = np.array(gmsh.model.getValue(dim, tag, uv)).reshape(-1, 3)
        nrm = np.array(gmsh.model.getNormal(tag, uv)).reshape(-1, 3)
        bb = gmsh.model.getBoundingBox(dim, tag)
        area = gmsh.model.occ.getMass(dim, tag)
        if t == "Cylinder":
            # axis = direction orthogonal to all normals (smallest singular vector of normals)
            _, _, vt = np.linalg.svd(nrm - 0.0); ax = vt[-1]
            # radius & centre: least squares circle in plane orthogonal to the axis
            e1 = np.cross(ax, [1, 0, 0] if abs(ax[0]) < 0.9 else [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(ax, e1)
            P = np.c_[pts @ e1, pts @ e2]
            A = np.c_[2 * P, np.ones(len(P))]; sol = np.linalg.lstsq(A, (P ** 2).sum(1), rcond=None)[0]
            r = np.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2)
            c3 = sol[0] * e1 + sol[1] * e2 + ax * (pts @ ax).mean()
            rows.append((t, tag, round(float(r), 2), np.round(ax, 3), np.round(c3, 2), round(float(np.ptp(pts @ ax)), 2), round(area, 1)))
        elif t == "Plane" and area > 80:
            rows.append((t, tag, None, np.round(nrm.mean(0), 3), np.round(pts.mean(0), 2), None, round(area, 1)))
    for r in sorted(rows, key=lambda r: (r[0], r[2] or 0)):
        print("  ", *r)
    gmsh.finalize()
