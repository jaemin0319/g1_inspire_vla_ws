"""Inspire hand model of the simulated G1 (docs/HAND_RH56E2.md).

  rh56e2                 Inspire RH56E2 (= RH56DFTP family) built from Inspire's URDF (configs/hand_rh56e2.yaml)
  inspire_dfx_official   the hand in Unitree's official unitree_sim_isaaclab asset (RH56DFX-style URDF)

Both keep the official joint names and the official normalised DDS interface (rt/inspire/cmd, q in [0, 1], 1 = open);
only the angle range behind q and the coupling of the passive joints differ.
"""
from __future__ import annotations

import numpy as np

from g1iv_common.config import load_config, ws_path

MODELS = ("rh56e2", "inspire_dfx_official")
DEFAULT = "rh56e2"

# unitree_sim_isaaclab dds/inspire_dds.py + action_provider_wh_dds.py (special_joint_mapping)
DFX = {"ranges": {"finger": (0.0, 1.7), "thumb_pitch": (0.0, 0.5), "thumb_yaw": (-0.1, 1.3)},
       "couplings": {"finger_intermediate": 1.0, "thumb_intermediate": 1.5, "thumb_distal": 2.4}}


def spec(name: str | None = None) -> dict:
    name = name or DEFAULT
    if name not in MODELS:
        raise ValueError(f"unknown hand model {name!r} (one of {MODELS})")
    if name == "inspire_dfx_official":
        return {"name": name, "ranges": dict(DFX["ranges"]), "couplings": dict(DFX["couplings"]), "usd": None}
    c = load_config("configs/hand_rh56e2.yaml")
    return {"name": name, "ranges": {k: tuple(map(float, v)) for k, v in c["ranges"].items()},
            "couplings": {k: float(v) for k, v in c["couplings"].items()}, "usd": str(ws_path(c["usd"])),
            "hands_npz": str(ws_path(c["hands_npz"])), "colorway": c["colorway"], "colorways": c["colorways"]}


def from_scene(scn: dict) -> dict:
    return spec((scn.get("robot") or {}).get("hand_model", DEFAULT))


def joint_range(s: dict, joint_name: str) -> tuple[float, float]:
    if "thumb_proximal_yaw" in joint_name:
        return s["ranges"]["thumb_yaw"]
    if "thumb_proximal_pitch" in joint_name:
        return s["ranges"]["thumb_pitch"]
    return s["ranges"]["finger"]


def rad_to_norm(s: dict, names: list[str], rad: np.ndarray) -> np.ndarray:
    """Joint angle -> normalised q (1 = open): the inverse of the sim's command mapping, as published on rt/inspire/state."""
    out = np.empty(len(names))
    for i, n in enumerate(names):
        lo, hi = joint_range(s, n)
        out[i] = np.clip((hi - rad[i]) / (hi - lo), 0.0, 1.0)
    return out


def channel_ranges(s: dict) -> dict:
    """g1iv_common.hands channel names -> (lo, hi)."""
    r = s["ranges"]
    return {"finger": r["finger"], "thumb_bend": r["thumb_pitch"], "thumb_rot": r["thumb_yaw"]}


def coupling_for(s: dict, joint_name: str) -> float | None:
    """Ratio of a passive joint to its driver (None for driven joints)."""
    c = s["couplings"]
    if joint_name.endswith("thumb_intermediate_joint"):
        return c["thumb_intermediate"]
    if joint_name.endswith("thumb_distal_joint"):
        return c["thumb_distal"]
    if joint_name.endswith("_intermediate_joint"):
        return c["finger_intermediate"]
    return None


def patch_official_inspire(s: dict, provider=None, dds_obj=None) -> dict:
    """Make the official sim code use this hand's ranges/couplings without editing it:
    - inspire_dds normalize/denormalize: the official code passes the DFX literal ranges per channel; they are
      mapped to this model's range (same channel semantics),
    - the wholebody action provider's coupled-joint scales (special_joint_mapping order)."""
    done = {}
    if s["name"] == "inspire_dfx_official":
        return done
    dfx_to = {DFX["ranges"]["finger"]: s["ranges"]["finger"], DFX["ranges"]["thumb_pitch"]: s["ranges"]["thumb_pitch"],
              DFX["ranges"]["thumb_yaw"]: s["ranges"]["thumb_yaw"]}
    if dds_obj is not None and not getattr(dds_obj, "_g1iv_hand_patched", False):
        orig_n, orig_d = dds_obj.normalize, dds_obj.denormalize

        def normalize(val, min_val, max_val, _o=orig_n):
            lo, hi = dfx_to.get((float(min_val), float(max_val)), (min_val, max_val))
            return _o(val, lo, hi)

        def denormalize(norm_val, min_val, max_val, _o=orig_d):
            lo, hi = dfx_to.get((float(min_val), float(max_val)), (min_val, max_val))
            return _o(norm_val, lo, hi)

        dds_obj.normalize, dds_obj.denormalize = normalize, denormalize
        dds_obj._g1iv_hand_patched = True
        done["dds"] = True
    if provider is not None and hasattr(provider, "special_joint_mapping") and hasattr(provider, "_inspire_special_scales_t"):
        import torch

        names = list(provider.special_joint_mapping.keys())
        scales = [coupling_for(s, n) for n in names]
        if any(v is None for v in scales):
            raise RuntimeError(f"[g1iv] unexpected passive joints in the official mapping: {names}")
        t = provider._inspire_special_scales_t
        provider._inspire_special_scales_t = torch.tensor(scales, dtype=t.dtype, device=t.device)
        done["couplings"] = dict(zip(names, scales))
    return done
