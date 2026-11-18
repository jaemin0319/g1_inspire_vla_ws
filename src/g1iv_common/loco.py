"""Walking-command arbitration: exactly one active source, limits, dead zone, timeout, ramps.

Output is the official wholebody command [vx, vy, wz, height] (robot base frame, m/s, rad/s, m),
published on rt/run_command/cmd. The simulator's action provider resets the command to zero after
each read, so a silent publisher also stops the robot within one 20 ms policy step; this module
adds an explicit staleness timeout on the INPUT side as well.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np


def deadzone(v: float, dz: float) -> float:
    if abs(v) <= dz:
        return 0.0
    return float(np.sign(v) * (abs(v) - dz) / (1.0 - dz))


@dataclass
class KeyState:
    forward: bool = False
    back: bool = False
    left: bool = False
    right: bool = False
    turn_left: bool = False
    turn_right: bool = False
    stop: bool = False
    focused: bool = True


@dataclass
class LocoArbiter:
    cfg: dict
    source: str = "keyboard"            # keyboard | xr | none
    height: float = 0.8
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))  # current ramped [vx, vy, wz]
    last_xr_t: float = -1e9
    last_key_t: float = -1e9
    raw: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    reason: str = "init"

    def __post_init__(self):
        if self.source not in ("keyboard", "xr", "none"):
            raise ValueError(f"loco source must be keyboard|xr|none, got {self.source}")
        self.height = float(self.cfg.get("height_default", 0.8))
        lim = self.cfg["limits"]
        self.lo = np.array([lim["vx"][0], lim["vy"][0], lim["wz"][0]], float)
        self.hi = np.array([lim["vx"][1], lim["vy"][1], lim["wz"][1]], float)
        self._xr_target = np.zeros(3)
        self._keys = KeyState()

    # ---- inputs -------------------------------------------------------------------------------
    def feed_keys(self, keys: KeyState, t: float | None = None):
        self._keys = keys
        self.last_key_t = time.monotonic() if t is None else t

    def feed_xr(self, left_stick_xy, right_stick_xy, valid: bool, t: float | None = None):
        """televuer thumbstick convention: x right, y forward?  The official G1 motion code uses
        vx = -left[1], vy = -left[0], wz = -right[0]; the same signs are used here."""
        if not valid:
            return
        c = self.cfg["xr"]
        lx, ly = (deadzone(float(a), c["deadzone"]) for a in left_stick_xy)
        rx = deadzone(float(right_stick_xy[0]), c["deadzone"])
        self._xr_target = np.array([-ly * c["vx_max"], -lx * c["vy_max"], -rx * c["wz_max"]])
        self.raw = [float(left_stick_xy[0]), float(left_stick_xy[1]), float(right_stick_xy[0])]
        self.last_xr_t = time.monotonic() if t is None else t

    def emergency_stop(self):
        self.v[:] = 0.0
        self._xr_target[:] = 0.0
        self.reason = "emergency_stop"

    # ---- output -------------------------------------------------------------------------------
    def step(self, dt: float, now: float | None = None) -> list[float]:
        now = time.monotonic() if now is None else now
        if self.source == "none":
            self.v[:] = 0.0
            self.reason = "source_none"
        elif self.source == "keyboard":
            k = self._keys
            kc = self.cfg["keyboard"]
            stale = (now - self.last_key_t) > 0.5
            if k.stop or stale or (not k.focused and kc.get("stop_on_focus_loss", True)):
                self.v[:] = 0.0  # explicit stop / lost focus / console silent: immediate zero
                self.reason = "stop_key" if k.stop else ("stale" if stale else "focus_lost")
                self.raw = [0.0, 0.0, 0.0]
            else:
                tgt = np.array([
                    kc["vx_fwd"] * k.forward - kc["vx_back"] * k.back,
                    kc["vy"] * (k.left - k.right),
                    kc["wz"] * (k.turn_left - k.turn_right),
                ], float)
                self.raw = tgt.tolist()
                acc = np.where(np.abs(tgt) > np.abs(self.v), kc["accel"], kc["decel"])
                acc = np.where(np.sign(tgt) * np.sign(self.v) < 0, kc["decel"], acc)
                self.v += np.clip(tgt - self.v, -acc * dt, acc * dt)
                self.reason = "keyboard"
        else:  # xr
            if (now - self.last_xr_t) > float(self.cfg["xr"]["stale_timeout_s"]):
                self.v[:] = 0.0
                self.reason = "xr_stale"
            else:
                self.v = self._xr_target.copy()
                self.reason = "xr"
        self.v = np.clip(self.v, self.lo, self.hi)
        self.v[np.abs(self.v) < 1e-4] = 0.0
        return [float(self.v[0]), float(self.v[1]), float(self.v[2]), self.height]
