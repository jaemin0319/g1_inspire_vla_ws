"""DDS topics and JSON payload helpers.

Official topics (unitree_sim_isaaclab, DDS domain 1 = simulator domain; real robots use domain 0):
  rt/lowcmd            hg LowCmd     arms: motor_cmd[15:29].q  (written by official G1_29_ArmController)
  rt/lowstate          hg LowState   measured body joints (sim -> clients)
  rt/inspire/cmd       go MotorCmds_ 12 normalised Inspire targets (1 = open)
  rt/inspire/state     go MotorStates_
  rt/run_command/cmd   std String    "[vx, vy, wz, height]" consumed by the wholebody locomotion policy
  rt/reset_pose/cmd    std String    official object reset ("1")
Project topics (std String carrying JSON, same domain):
  rt/g1iv/teleop_meta  hub -> sim recorder: raw input, pre/post-limit targets, OPEN/CLOSE, sources
  rt/g1iv/control      any -> sim: {"cmd": "record_start"|"record_stop"|"reset"|...}
  rt/g1iv/sim_status   sim -> clients: sim time, base state, recording/reset state
"""
from __future__ import annotations

import json
import time
from typing import Any

SIM_DDS_DOMAIN = 1

T_LOWCMD = "rt/lowcmd"
T_LOWSTATE = "rt/lowstate"
T_INSPIRE_CMD = "rt/inspire/cmd"
T_INSPIRE_STATE = "rt/inspire/state"
T_RUN_COMMAND = "rt/run_command/cmd"
T_RESET_POSE = "rt/reset_pose/cmd"

T_TELEOP_META = "rt/g1iv/teleop_meta"
T_CONTROL = "rt/g1iv/control"
T_SIM_STATUS = "rt/g1iv/sim_status"

CONTROL_CMDS = ("record_start", "record_stop", "record_abort", "reset", "mark")
OUTCOMES = ("success", "failure", "aborted", "undetermined")


def _default(o: Any):
    try:
        import numpy as np  # local import: keep this module importable everywhere

        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer, np.bool_)):
            return o.item()
    except ImportError:
        pass
    raise TypeError(f"not JSON serialisable: {type(o)}")


def dumps(d: dict) -> str:
    return json.dumps(d, default=_default, separators=(",", ":"))


def loads(s: str) -> dict:
    return json.loads(s)


def control_msg(cmd: str, **kw) -> str:
    if cmd not in CONTROL_CMDS:
        raise ValueError(f"unknown control cmd {cmd!r}")
    if cmd == "record_stop" and kw.get("outcome", "undetermined") not in OUTCOMES:
        raise ValueError(f"outcome must be one of {OUTCOMES}")
    return dumps({"cmd": cmd, "wall_time": time.time(), **kw})


def run_command_str(vx: float, vy: float, wz: float, height: float) -> str:
    # Same textual format as the official send_commands_keyboard.py (parsed with ast.literal_eval).
    return str([float(vx), float(vy), float(wz), float(height)])
