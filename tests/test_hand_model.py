"""Inspire RH56E2 hand model (g1iv_common.hand_model, configs/hand_rh56e2.yaml) and its generated asset
(tools/cad/make_inspire_e2_hand.py -> data/assets/inspire_rh56e2/rh56e2_hands.npz, skipped when not built)."""
import json
import math
import os

import numpy as np
import pytest

from g1iv_common import hand_model as HM
from g1iv_common import hands as H
from g1iv_common.config import load_config, ws_path

E2 = HM.spec("rh56e2")
DFX = HM.spec("inspire_dfx_official")
NPZ = ws_path(load_config("configs/hand_rh56e2.yaml")["hands_npz"])
need_npz = pytest.mark.skipif(not os.path.isfile(NPZ), reason="RH56E2 asset not built (docs/HAND_RH56E2.md)")


def test_default_scene_uses_rh56e2():
    assert load_config("configs/scene_desk.yaml")["robot"]["hand_model"] == "rh56e2"


def test_e2_ranges_match_the_rh56e2_manual():
    # RH56E2 manual: thumb rotation 75..170 deg, thumb bend 28.5..64 deg; Inspire URDF finger 0..1.4381 rad
    assert math.isclose(math.degrees(E2["ranges"]["thumb_yaw"][1] - E2["ranges"]["thumb_yaw"][0]), 95.0, abs_tol=0.1)
    assert math.isclose(math.degrees(E2["ranges"]["thumb_pitch"][1] - E2["ranges"]["thumb_pitch"][0]), 35.5, abs_tol=0.1)
    assert E2["ranges"]["finger"] == (0.0, 1.4381)
    assert math.isclose(E2["couplings"]["thumb_distal"], 0.8392 * 0.891, rel_tol=1e-4)


def test_rad_norm_round_trip_and_open_is_one():
    names = ["R_index_proximal_joint", "R_thumb_proximal_pitch_joint", "R_thumb_proximal_yaw_joint"]
    for s in (E2, DFX):
        lo = np.array([HM.joint_range(s, n)[0] for n in names])
        hi = np.array([HM.joint_range(s, n)[1] for n in names])
        assert np.allclose(HM.rad_to_norm(s, names, lo), 1.0)          # q = 1 is open (angle = lower)
        assert np.allclose(HM.rad_to_norm(s, names, hi), 0.0)
        mid = (lo + hi) / 2
        assert np.allclose(HM.rad_to_norm(s, names, mid), 0.5)


def test_official_dds_patch_maps_dfx_literals_to_e2_ranges():
    class FakeInspireDDS:   # same formulas as unitree_sim_isaaclab dds/inspire_dds.py
        def normalize(self, val, min_val, max_val):
            return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

        def denormalize(self, norm_val, min_val, max_val):
            return (1.0 - np.clip(norm_val, 0.0, 1.0)) * (max_val - min_val) + min_val

    d = FakeInspireDDS()
    assert HM.patch_official_inspire(E2, None, d) == {"dds": True}
    assert math.isclose(d.denormalize(0.0, 0.0, 1.7), 1.4381)
    assert math.isclose(d.denormalize(0.0, 0.0, 0.5), 0.62)
    assert math.isclose(d.denormalize(0.0, -0.1, 1.3), 1.658)
    assert math.isclose(d.denormalize(1.0, -0.1, 1.3), 0.0)
    assert math.isclose(d.normalize(0.31, 0.0, 0.5), 0.5)
    HM.patch_official_inspire(E2, None, d)                            # idempotent
    assert math.isclose(d.denormalize(0.0, 0.0, 1.7), 1.4381)
    d2 = FakeInspireDDS()
    assert HM.patch_official_inspire(DFX, None, d2) == {}             # official hand: untouched
    assert math.isclose(d2.denormalize(0.0, 0.0, 1.7), 1.7)


def test_hub_channel_ranges_follow_the_model():
    saved = dict(H._RANGES)
    try:
        H.set_ranges(HM.channel_ranges(E2))
        assert np.allclose(H.norm_to_rad(np.zeros(6)), [1.4381] * 4 + [0.62, 1.658])
        H.set_ranges(HM.channel_ranges(DFX))
        assert np.allclose(H.norm_to_rad(np.zeros(6)), [1.7] * 4 + [0.5, 1.3])
    finally:
        H._RANGES.update(saved)


@need_npz
def test_asset_links_joints_mass_and_mirror():
    d = np.load(NPZ)
    meta = json.loads(str(d["meta"]))
    assert len(meta["links"]) == 13 and len(meta["joints"]["right"]) == 12
    names = {j["name"] for j in meta["joints"]["right"]}
    assert names == {f"{f}_{k}_joint" for f in ("index", "middle", "ring", "pinky") for k in ("proximal", "intermediate")} | {
        "thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint", "thumb_intermediate_joint", "thumb_distal_joint"}
    mass = sum(float(d[f"right/{k}/mass"]) for k in meta["links"])
    assert 0.85 < mass < 0.95                                         # Inspire URDF 0.899 kg (spec 790 +-10 g, w/o adapter)
    c = meta["checks"]
    assert c["ftp_base_to_e2_median_mm"] < 0.2                        # mount = Unitree's FTP mount (base meshes coincide)
    assert c["left_ref_median_mm"] < 0.2                              # left = mirror(right) = Inspire's left URDF
    M = np.diag([1.0, -1.0, 1.0, 1.0])
    for k in meta["links"]:
        Tr, Tl = d[f"right/{k}/T"], d[f"left/{k}/T"]
        assert np.allclose(M @ Tr @ M, Tl, atol=1e-9)
        R = Tl[:3, :3]
        assert abs(np.linalg.det(R) - 1) < 1e-9


@need_npz
def test_asset_thumb_on_palm_side_like_the_official_hand():
    d = np.load(NPZ)
    meta = json.loads(str(d["meta"]))
    t = np.array(meta["checks"]["thumb_yaw_joint_in_wrist_mm"])
    # official DFX right thumb joint in right_wrist_yaw_link: (0.1106, 0.0141, 0.0205); Unitree FTP (0.1104, 0.0210, 0.0269)
    assert t[1] > 0.01 and t[2] > 0.01 and 0.10 < t[0] < 0.13


@need_npz
def test_asset_colour_labels_cover_the_photo_parts():
    d = np.load(NPZ)
    meta = json.loads(str(d["meta"]))
    labels = meta["labels"]
    area = {}
    for k in meta["links"]:
        V, F, L = d[f"right/{k}/V"], d[f"right/{k}/F"], d[f"right/{k}/L"]
        a = 0.5 * np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
        for i, lab in enumerate(labels):
            area[(k, lab)] = area.get((k, lab), 0.0) + float(a[L == i].sum())
    assert area[("hand_base_link", "cover")] > 5e-3                  # white back cover
    assert area[("hand_base_link", "lens")] > 1e-5                   # status light
    for f in ("index", "middle", "ring", "pinky"):
        assert area[(f"{f}_intermediate", "shell")] > area[(f"{f}_intermediate", "pad")] > 0   # white back, black pads
        assert area[(f"{f}_intermediate", "black")] > 0                                       # black tip cap
