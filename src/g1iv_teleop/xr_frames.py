"""OpenXR <-> IK-frame helpers for MOCK / SCRIPTED input.

Mock and scripted sources do not fake TeleData directly: they produce *raw* OpenXR-convention head and
controller matrices (what Vuer delivers from a real headset) and let the OFFICIAL
televuer.TeleVuerWrapper.get_tele_data() convert them. This module only provides the inverse of that
official conversion (controller mode, head_yaw reference) so a scripted IK-frame target can be turned
into the raw controller pose that produces it. Constants are copied from
xr_teleoperate/teleop/televuer/src/televuer/tv_wrapper.py (televuer 766de45).
"""
from __future__ import annotations

import numpy as np

T_ROBOT_OPENXR = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
T_OPENXR_ROBOT = np.array([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], float)
R_ROBOT_OPENXR = T_ROBOT_OPENXR[:3, :3]
R_OPENXR_ROBOT = T_OPENXR_ROBOT[:3, :3]
HEAD_TO_WAIST = np.array([0.15, 0.0, 0.45])          # added by the official transform

# official neutral poses (OpenXR basis): head 1.5 m up / 0.2 m back, controllers at waist height
CONST_HEAD_POSE = np.array([[1, 0, 0, 0], [0, 1, 0, 1.5], [0, 0, 1, -0.2], [0, 0, 0, 1]], float)
CONST_RIGHT_ARM_POSE = np.array([[1, 0, 0, 0.15], [0, 1, 0, 1.13], [0, 0, 1, -0.3], [0, 0, 0, 1]], float)
CONST_LEFT_ARM_POSE = np.array([[1, 0, 0, -0.15], [0, 1, 0, 1.13], [0, 0, 1, -0.3], [0, 0, 0, 1]], float)


def head_yaw_rot(B_robot_head_rot: np.ndarray) -> np.ndarray:
    x = B_robot_head_rot[:, 0].copy()
    x[2] = 0.0
    n = np.linalg.norm(x)
    if not np.isfinite(n) or n < 1e-6:
        return np.eye(3)
    x /= n
    z = np.array([0.0, 0.0, 1.0])
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, z])


def forward_controller(head_xr: np.ndarray, arm_xr: np.ndarray) -> np.ndarray:
    """Re-statement of the official controller-mode conversion (used only to test the inverse)."""
    Bh = T_ROBOT_OPENXR @ head_xr @ T_OPENXR_ROBOT
    Ba = T_ROBOT_OPENXR @ arm_xr @ T_OPENXR_ROBOT
    Ry = head_yaw_rot(Bh[:3, :3])
    out = Ba.copy()
    out[:3, :3] = Ry.T @ Ba[:3, :3]
    out[:3, 3] = Ry.T @ (Ba[:3, 3] - Bh[:3, 3]) + HEAD_TO_WAIST
    return out


def inverse_controller(head_xr: np.ndarray, P_ik: np.ndarray) -> np.ndarray:
    """Raw OpenXR controller matrix that the official conversion maps to P_ik (IK/waist frame)."""
    Bh = T_ROBOT_OPENXR @ head_xr @ T_OPENXR_ROBOT
    Ry = head_yaw_rot(Bh[:3, :3])
    Ba = np.eye(4)
    Ba[:3, :3] = Ry @ P_ik[:3, :3]
    Ba[:3, 3] = Ry @ (P_ik[:3, 3] - HEAD_TO_WAIST) + Bh[:3, 3]
    return T_OPENXR_ROBOT @ Ba @ T_ROBOT_OPENXR


def delta_robot_to_xr(dp_robot) -> np.ndarray:
    """Translation delta in robot basis -> OpenXR basis (head yaw = 0)."""
    return R_OPENXR_ROBOT @ np.asarray(dp_robot, float)
