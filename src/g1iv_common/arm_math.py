"""Pose / joint helpers for the clutched relative arm mapping (numpy only)."""
from __future__ import annotations

import numpy as np


def inv_T(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


def rel_target(T_ee_anchor: np.ndarray, T_ctrl_anchor: np.ndarray, T_ctrl_now: np.ndarray,
               pos_scale: float = 1.0, use_rotation: bool = True, max_offset: float | None = None) -> np.ndarray:
    """Target EE pose from controller motion since the anchor, all in the same (IK/waist) frame.

    Translation: p = p_ee0 + s * (p_ctrl - p_ctrl0)           (frame-aligned delta, not body-relative)
    Rotation:    R = (R_ctrl * R_ctrl0^T) * R_ee0              (world-frame rotation delta)
    """
    dp = pos_scale * (T_ctrl_now[:3, 3] - T_ctrl_anchor[:3, 3])
    if max_offset is not None:
        n = float(np.linalg.norm(dp))
        if n > max_offset:
            dp *= max_offset / n
    T = np.eye(4)
    T[:3, 3] = T_ee_anchor[:3, 3] + dp
    if use_rotation:
        dR = T_ctrl_now[:3, :3] @ T_ctrl_anchor[:3, :3].T
        T[:3, :3] = dR @ T_ee_anchor[:3, :3]
    else:
        T[:3, :3] = T_ee_anchor[:3, :3]
    return T


def rate_limit(q_prev: np.ndarray, q_goal: np.ndarray, max_vel: float, dt: float) -> np.ndarray:
    """Uniformly scale the joint step so that no joint exceeds max_vel*dt (keeps the direction)."""
    d = np.asarray(q_goal, float) - np.asarray(q_prev, float)
    m = float(np.max(np.abs(d))) if d.size else 0.0
    lim = max_vel * dt
    if m <= lim or m == 0.0:
        return np.asarray(q_goal, float).copy()
    return np.asarray(q_prev, float) + d * (lim / m)


def rotz(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
