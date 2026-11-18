"""Camera rig geometry (src/g1iv_sim/camera_rig.py) against configs/camera_rig.yaml and OpenCV."""
import math

import numpy as np
import pytest

from g1iv_common.config import load_config
from g1iv_sim.camera_rig import (RTX_P1_SIGN, RTX_PIXEL_OFFSET, build_rig, distort_radius, lens_distortion_attributes,
                                 matrix_to_quat_wxyz, monotonic_radial_fit, pinhole_spawn_params, project_points)

RIG = load_config("configs/camera_rig.yaml")
CAMS = {c.name: c for c in build_rig(RIG)}
RIG_A = load_config("configs/camera_rig.yaml", ["wrist.mount=inspire_ring_mono160"])   # previous wrist model
CAMS_A = {c.name: c for c in build_rig(RIG_A)}
M_Y = np.diag([1.0, -1.0, 1.0])


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_rig_has_four_cameras_with_unitree_keys():
    assert {c.alias for c in CAMS.values()} == {"cam_left_high", "cam_right_high", "cam_left_wrist", "cam_right_wrist"}
    for c in CAMS.values():
        assert np.allclose(c.R.T @ c.R, np.eye(3), atol=1e-9) and abs(np.linalg.det(c.R) - 1) < 1e-9
        assert np.allclose(quat_to_matrix(c.quat_wxyz()), c.R, atol=1e-9)


def test_head_stereo_geometry():
    L, R = CAMS["front_camera"], CAMS["front_right_camera"]
    h = RIG["head"]
    off = np.asarray(h.get("mount_link_in_parent_xyz", [0, 0, 0])) if h.get("mount_link") else np.zeros(3)
    assert L.parent_link == h.get("mount_link", h["parent_link"])
    assert np.allclose((L.pos + R.pos) / 2 + off, h["center_xyz"], atol=1e-9)  # pair centred (config in torso_link)
    base = np.linalg.norm(R.pos - L.pos)
    assert abs(base - np.linalg.norm(h["calibration"]["right_in_left"]["tvec_mm"]) / 1000) < 1e-9
    assert L.pos[1] > R.pos[1]                                                   # left eye on the robot's +y side
    z = L.R[:, 2]                                                                # optical axis, pitched down
    assert abs(math.degrees(math.atan2(-z[2], z[0])) - h["pitch_down_deg"]) < 1e-6
    assert L.R[:, 0][1] < -0.99                                                  # image right = robot -y
    assert L.R[:, 1][2] < 0                                                      # image down points down


def test_default_wrist_is_unitree_d405_c():
    assert RIG["wrist"]["mount"] == "unitree_d405_c"
    for c in (CAMS["left_wrist_camera"], CAMS["right_wrist_camera"]):
        assert 432 < c.intr.fx < 438 and c.refit is None          # RealSense D405 colour 640x480, mild distortion


def test_d405_c_cameras_mirror_and_look_at_the_hand():
    l, r = CAMS["left_wrist_camera"], CAMS["right_wrist_camera"]
    p = RIG["wrist"]["unitree_d405_c"]
    assert p["phi_deg"]["left"] == pytest.approx(180.0 - p["phi_deg"]["right"])   # mirror-symmetric mounting
    assert np.allclose(M_Y @ l.pos, r.pos, atol=1e-9)
    # the left bracket is the mirror part but the D405 is not mirrored: its USB side (= colour imager side) follows the
    # mirrored cable opening, so image right mirrors and image down flips
    assert np.allclose(M_Y @ l.R[:, 2], r.R[:, 2], atol=1e-9)
    assert np.allclose(M_Y @ l.R[:, 0], r.R[:, 0], atol=1e-9)
    assert np.allclose(-M_Y @ l.R[:, 1], r.R[:, 1], atol=1e-9)
    for c in (l, r):
        assert 0.80 < c.R[:, 2][0] < 0.90                        # 31.6 deg off the hand axis (bracket CAD)
        assert np.hypot(c.pos[1], c.pos[2]) > 0.08               # D405 outside the R38.4 clamp, ~86 mm from the axis
        uv, z = project_points(c, np.eye(4), np.array([[0.15, 0.0, 0.0]]))   # point on the hand axis 15 cm out
        assert z[0] > 0 and 0 <= uv[0, 0] < 640 and 0 <= uv[0, 1] < 480


def test_d405_c_asset_seat_geometry():
    from g1iv_sim.camera_rig import load_cad_asset
    d = load_cad_asset("wrist_d405_mount_c.npz")
    for k in ("a", "b"):
        holes = d[f"{k}_holes"]
        assert np.linalg.norm(holes[0] - holes[1]) == pytest.approx(0.020, abs=2e-4)      # D405 back M3 pair, 20 mm
        n = d[f"{k}_cam_R"][:, 2]
        rel = d[f"{k}_cam_pos"] - holes.mean(0)
        assert np.dot(rel, n) == pytest.approx(0.0015 + 0.023 - 0.0038, abs=1e-6)   # plate + body - optical depth
        lateral = rel - n * np.dot(rel, n)
        assert np.linalg.norm(lateral) == pytest.approx(0.009, abs=1e-6)              # colour imager 9 mm off centre
        assert np.dot(d[f"{k}_d405_front_centre"] - d[f"{k}_cam_pos"], n) == pytest.approx(0.0038, abs=1e-6)
    assert np.allclose(d["b_cam_pos"], d["a_cam_pos"] * [1, 1, -1], atol=1e-9)        # mirror pair across CAD z


def test_d405_c_clamp_on_the_wrist_yaw_housing():
    from g1iv_sim.camera_rig import d405_c_mount_pose
    p = RIG["wrist"]["unitree_d405_c"]
    for side in ("left", "right"):
        R, t, d = d405_c_mount_pose(p, side)
        k = p["cad_variant"][side]
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-12) and abs(np.linalg.det(R) - 1) < 1e-12
        assert np.allclose(R @ d[f"{k}_bore_axis"], [1, 0, 0], atol=1e-12)            # sleeve along the hand axis
        ring = R @ d[f"{k}_ring_mid"] + t
        assert np.allclose(ring, [p["ring_mid_x_mm"] / 1000, *(np.array(p["clamp_axis_yz_mm"][side]) / 1000)], atol=1e-12)
        # the bracket's internal key flat (CAD +x, r 23.88 mm) sits on the wrist-yaw housing flat (+y right, -y left)
        assert np.allclose(R @ [1.0, 0.0, 0.0], [0.0, 1.0 if side == "right" else -1.0, 0.0], atol=1e-9)
        assert R @ [0.0, 1.0, 0.0] @ [0, 0, 1] > 0.99            # -> camera on top of the wrist (+z)


def test_mono160_wrist_cameras_mirror_and_palm_side():
    l, r = CAMS_A["left_wrist_camera"], CAMS_A["right_wrist_camera"]
    M = np.diag([1.0, -1.0, 1.0])
    assert np.allclose(M @ l.pos, r.pos, atol=1e-12)                             # mirror images across y
    assert np.allclose(M @ l.R[:, 2], r.R[:, 2], atol=1e-12)                     # optical axes mirrored
    assert r.pos[1] > 0 > l.pos[1]                                               # Inspire palm: +y right, -y left
    for c in (l, r):
        assert c.R[:, 2][0] > 0.99                                               # looks along the hand (+x)
        # a point on the hand axis 12 cm ahead appears in the upper half of the image (palm at the top)
        uv, z = project_points(c, np.eye(4), np.array([[0.12, 0.0, 0.0]]))
        assert z[0] > 0 and uv[0, 1] < c.intr.cy


def test_projection_matches_opencv():
    cv2 = pytest.importorskip("cv2")
    rng = np.random.default_rng(0)
    for c in CAMS.values():
        pts_cam = np.c_[rng.uniform(-0.6, 0.6, (50, 2)), np.ones(50)] * rng.uniform(0.3, 1.0, (50, 1))
        world = (c.T_parent_cam() @ np.c_[pts_cam, np.ones(50)].T).T[:, :3]     # parent frame = world
        uv, _ = project_points(c, np.eye(4), world, use_render=False)
        ref, _ = cv2.projectPoints(pts_cam, np.zeros(3), np.zeros(3), c.intr.K(), c.intr.opencv_dist())
        assert np.abs(uv - ref[:, 0]).max() < 1e-6, c.name


def test_wrist_radial_refit_is_monotonic_and_close():
    c = CAMS_A["right_wrist_camera"]                          # 160 deg wide-angle model (D405 needs no re-fit)
    fit = monotonic_radial_fit(c.intr)
    assert fit and not fit.get("failed")                     # the calibrated polynomial folds inside the image
    assert fit["max_error_px"] < 1.0
    r = np.linspace(0, 2.5, 5000)
    rd = distort_radius(r, c.render.dist)
    corner = max(math.hypot((u - c.intr.cx) / c.intr.fx, (v - c.intr.cy) / c.intr.fy) for u in (0, 640) for v in (0, 480))
    reach = np.argmax(rd >= corner)
    assert reach > 0 and np.all(np.diff(rd[: reach + 1]) > 0)
    assert monotonic_radial_fit(CAMS["front_camera"].intr) is None  # head polynomial is fine as calibrated


def test_rtx_attribute_conventions_and_spawn_params():
    c = CAMS["front_camera"]
    a = lens_distortion_attributes(c.render)
    assert a["p1"] == pytest.approx(RTX_P1_SIGN * c.render.dist["p1"]) and a["p2"] == pytest.approx(c.render.dist["p2"])
    assert a["cx"] == pytest.approx(c.render.cx + RTX_PIXEL_OFFSET) and a["imageSize"] == (640, 480)
    sp = pinhole_spawn_params(c.render)
    assert sp["focal_length"] * 640 / sp["horizontal_aperture"] == pytest.approx(c.render.fx)
    assert sp["focal_length"] * 480 / sp["vertical_aperture"] == pytest.approx(c.render.fy)
    assert matrix_to_quat_wxyz(np.eye(3)) == (1.0, 0.0, 0.0, 0.0)


def test_head_bracket_aligns_with_eyes_and_head_plate():
    from g1iv_sim.camera_rig import head_bracket_pose, head_cameras
    d = np.load("assets/camera_rig/head_stereo_bracket_60fps_classic98_cad.npz")
    eyes = head_cameras(RIG["head"], 640, 480)
    R, t = head_bracket_pose(eyes, d)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9) and abs(np.linalg.det(R) - 1) < 1e-9   # proper rotation
    holes = d["lens_holes_cad"] @ R.T + t                       # [robot-left, robot-right]
    for h, e in zip(holes, eyes):
        assert np.linalg.norm(h - e.pos) < 0.0015               # lens holes on the optical centres (< 1.5 mm)
    up = R @ np.array([0.0, 0.0, 1.0])
    assert math.degrees(math.acos(up[2])) < 6.0                 # bracket upright (CAD plate 49.3 vs fitted 46.3 deg)
    screws = d["screws_cad"] @ R.T + t                          # M4 holes on the head's internal top plate
    assert np.all(np.abs(screws[:, 2] - 0.465) < 0.006) and np.all(np.abs(screws[:, 0]) < 0.02)


def test_wrist_bracket_lens_hole_matches_camera():
    for side in ("left", "right"):
        d = np.load(f"assets/camera_rig/inspire_wrist_bracket_{side}.npz")
        assert np.linalg.norm(d["lens_hole_centre"] - CAMS_A[f"{side}_wrist_camera"].pos) < 0.001
