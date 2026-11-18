"""Assemble Unitree's D405 wrist-camera mount ("C": xr_teleoperate hardware @ 7cd188c, Dex3-1 / M4010 wrist; STEP ->
STL via step_to_stl.py) with an Intel RealSense D405 and save it, in the bracket CAD frame, for the simulator.

Parts (the GitHub file names are shifted, docs/CAMERA_RIG.md):
  variant a = wrist_ring_mount.STEP            (camera looks along CAD +z)
  variant b = right_wrist_D405_camera_mount.STEP (mirror image of a across CAD z; camera looks along CAD -z)
  half ring = left_wrist_D405_camera_mount.STEP  (bolted to the mount at CAD z = +-20 mm, M3 at x = 27.86 / -34)
  D405      = realsense-ros realsense2_description/meshes/d405.stl (Apache-2.0, ros2-development @ 489265d)

Geometry read from the STEP (tools/cad/cylinders.py):
  clamp bore r = 30.0 mm, 42.34 mm long, axis CAD z through (0, 0, +-2.83); half ring bore r = 30.4 mm, 8 mm wide
  D405 seat: 3 mm plate, two M3 clearance holes 20.0 mm apart (D405 back: 2 x M3 at +-10 mm on the baseline axis,
  realsense mesh), 43 x 43 mm pocket on the plate's +n side, the pocket wall towards the clamp (v-) has a
  20 x 15 mm opening at the plate = USB-C exit. The D405 USB-C is on its LEFT side face (realsense xacro usb_plug,
  same side as the left imager), so the camera's left (= colour / depth origin, 9 mm from the centre line) is v-.
  Colour optical centre: 3.8 mm behind the front plate (0.1 mm glass + 3.7 mm zero-depth, D400 datasheet table 4-16,
  realsense xacro), 23 mm deep body -> 19.2 mm in front of the back face.

usage: make_d405_c_asset.py STL_DIR D405_STL OUT.npz
"""
import sys

import numpy as np
import trimesh

stl, d405_path, out = sys.argv[1:4]
MM = 1e-3


def load(name):
    m = trimesh.load(f"{stl}/{name}.stl", force="mesh")
    m.merge_vertices()
    return m


def seat(sz):
    """D405 seat frame of variant a (sz=+1) / b (sz=-1) in CAD mm: hole midpoint, plate normal n (towards the pocket),
    v (hole axis, +v = away from the USB opening), u = v x n."""
    n = np.array([0.31, -0.423, 0.852 * sz]); n /= np.linalg.norm(n)
    h1, h2 = np.array([4.96, 110.78, 1.63 * sz]), np.array([2.07, 92.66, -6.31 * sz])
    v = h1 - h2; v -= n * np.dot(v, n); v /= np.linalg.norm(v)
    u = np.cross(v, n)
    return (h1 + h2) / 2, n, v, u, np.stack([h1, h2])


PLATE_FRONT = 1.5      # plate face on the pocket side, mm along n from the hole-axis midpoint (STEP plane 19)
D405_DEPTH = 23.0
OPT_BEHIND_FRONT = 3.8
IMAGER_OFF = 9.0

mount_a, mount_b, ring = load("wrist_ring_mount"), load("right_wrist_D405_camera_mount"), load("left_wrist_D405_camera_mount")
d405 = trimesh.load(d405_path, force="mesh"); d405.merge_vertices()
# mount b is the mirror image of mount a across CAD z (checked, not assumed)
mir = mount_a.vertices * np.array([1, 1, -1])
dist = trimesh.proximity.ProximityQuery(mount_b).signed_distance(mir[:: max(1, len(mir) // 3000)])
ad = np.abs(dist)
print("mirror check: |mirror(a) - b| median %.4f  p99 %.4f  max %.4f mm (STL chord tolerance)" % (np.median(ad), np.percentile(ad, 99), ad.max()))
assert np.median(ad) < 0.02 and ad.max() < 0.5

# half ring -> variant a: ring x -> x, ring y (bore axis, centre y=118) -> -z (centre z=+20), ring z (bolt axis) -> +y
R_ring = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)
ring_a_v = (ring.vertices - [0, 118.0, 0]) @ R_ring.T + [0, 0, 20.0]

res = {}
for key, sz, mount in (("a", 1.0, mount_a), ("b", -1.0, mount_b)):
    mid, n, v, u, holes = seat(sz)
    L = -v                                  # D405 left (USB side, colour imager) towards the USB opening
    up = np.cross(n, L)
    R_mesh = np.stack([L, up, n], 1)        # realsense d405.stl: x = left, y = up, z = forward, origin at the front plate
    front = mid + n * (PLATE_FRONT + D405_DEPTH)
    d_v = d405.vertices @ R_mesh.T + front
    cam = mid + n * (PLATE_FRONT + D405_DEPTH - OPT_BEHIND_FRONT) + L * IMAGER_OFF
    R_cv = np.stack([-L, -up, n], 1)        # OpenCV camera axes (x right, y down, z forward) in CAD
    if sz > 0:
        rv, rf = ring_a_v, ring.faces.copy()
    else:                                    # ring is symmetric about its mid-plane: mirror it with the mount
        rv, rf = ring_a_v * [1, 1, -1], ring.faces[:, ::-1].copy()
    # parts must not intersect the D405 (sanity of the seat): D405 vertices vs mount
    pen = trimesh.proximity.ProximityQuery(mount).signed_distance(d_v[::4])
    print(f"variant {key}: D405 penetration into the mount max {pen.max():.2f} mm (positive = inside)")
    res.update({
        f"{key}_mount_points": (mount.vertices * MM).astype(np.float32), f"{key}_mount_faces": mount.faces.astype(np.int32),
        f"{key}_ring_points": (rv * MM).astype(np.float32), f"{key}_ring_faces": rf.astype(np.int32),
        f"{key}_d405_points": (d_v * MM).astype(np.float32), f"{key}_d405_faces": d405.faces.astype(np.int32),
        f"{key}_bore_centre": np.array([0, 0, 2.83 * sz]) * MM, f"{key}_bore_axis": np.array([0, 0, sz]),
        f"{key}_ring_mid": np.array([0, 0, 20.0 * sz]) * MM,
        f"{key}_cam_pos": cam * MM, f"{key}_cam_R": R_cv, f"{key}_holes": holes * MM,
        f"{key}_d405_front_centre": front * MM,
    })
    print(f"variant {key}: optical centre CAD [mm] {cam.round(2).tolist()} axis {n.round(4).tolist()} "
          f"image right {(-L).round(3).tolist()} image down {(-up).round(3).tolist()}")
res["bore_radius"] = np.array(30.0 * MM)
res["ring_bore_radius"] = np.array(30.4 * MM)
res["sources"] = np.array("Unitree xr_teleoperate hardware 7cd188c (D405 wrist mount, Dex3-1); "
                          "IntelRealSense/realsense-ros ros2-development 489265d realsense2_description/meshes/d405.stl (Apache-2.0)")
np.savez_compressed(out, **res)
print("wrote", out)
