"""Binary OPEN/CLOSE -> Inspire 12-channel targets.

The user command is exactly two states per hand. Internally the 6 actuated channels per hand are
ramped (rate-limited) between the OPEN and CLOSE presets, so the published motor target is smooth
while the operator's input stays discrete. Channel semantics follow the simulator's DFX-compatible
interface (see configs/hands_inspire.yaml).
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field

import numpy as np

CHANNELS = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rot")
# 12-vector layout used on rt/inspire/cmd: right hand 0..5, left hand 6..11.
RIGHT_SLICE = slice(0, 6)
LEFT_SLICE = slice(6, 12)

# normalised q -> joint angle (rad), from dds/inspire_dds.py (sim) and robot_hand_inspire.py (teleop)
# (official DFX hand; the simulator switches them to the active hand model with set_ranges, g1iv_common.hand_model)
_RANGES = {"finger": (0.0, 1.7), "thumb_bend": (0.0, 0.5), "thumb_rot": (-0.1, 1.3)}


def set_ranges(ranges: dict) -> None:
    """Channel ranges of the simulated hand model: {"finger": (lo, hi), "thumb_bend": ..., "thumb_rot": ...}."""
    missing = set(_RANGES) - set(ranges)
    if missing:
        raise ValueError(f"hand ranges miss {sorted(missing)}")
    _RANGES.update({k: (float(v[0]), float(v[1])) for k, v in ranges.items()})


class HandCmd(enum.IntEnum):
    OPEN = 0
    CLOSE = 1


def channel_range(ch: str) -> tuple[float, float]:
    if ch in ("pinky", "ring", "middle", "index"):
        return _RANGES["finger"]
    return _RANGES[ch]


def norm_to_rad(q6: np.ndarray) -> np.ndarray:
    """6 normalised targets (1=open) -> proximal joint angles in rad (same formula as the sim)."""
    out = np.empty(6)
    for i, ch in enumerate(CHANNELS):
        lo, hi = channel_range(ch)
        out[i] = (1.0 - np.clip(q6[i], 0.0, 1.0)) * (hi - lo) + lo
    return out


def preset_vector(preset: dict) -> np.ndarray:
    missing = [c for c in CHANNELS if c not in preset]
    if missing:
        raise ValueError(f"hand preset misses channels {missing}")
    v = np.array([float(preset[c]) for c in CHANNELS])
    if np.any(v < 0.0) or np.any(v > 1.0):
        raise ValueError(f"hand preset values must be in [0,1] (1=open): {preset}")
    return v


@dataclass
class TriggerBinarizer:
    """Analog trigger (0 released .. 1 pressed) -> OPEN/CLOSE with hysteresis."""

    close_th: float = 0.6
    open_th: float = 0.4
    state: HandCmd = HandCmd.OPEN

    def __post_init__(self):
        if not (0.0 <= self.open_th < self.close_th <= 1.0):
            raise ValueError("need 0 <= open_th < close_th <= 1")

    def update(self, pressed: float) -> HandCmd:
        if self.state == HandCmd.OPEN and pressed >= self.close_th:
            self.state = HandCmd.CLOSE
        elif self.state == HandCmd.CLOSE and pressed <= self.open_th:
            self.state = HandCmd.OPEN
        return self.state


@dataclass
class BinaryHand:
    """One hand: discrete command + rate-limited 6-channel normalised target."""

    q_open: np.ndarray
    q_close: np.ndarray
    max_rate_per_s: float = 2.0
    cmd: HandCmd = HandCmd.OPEN
    q: np.ndarray = field(default=None)  # current (ramped) target

    def __post_init__(self):
        self.q_open = np.asarray(self.q_open, float)
        self.q_close = np.asarray(self.q_close, float)
        if self.q is None:
            self.q = self.q_open.copy()

    @property
    def goal(self) -> np.ndarray:
        return self.q_close if self.cmd == HandCmd.CLOSE else self.q_open

    def set(self, cmd: HandCmd):
        self.cmd = HandCmd(cmd)

    def reset(self, cmd: HandCmd = HandCmd.OPEN):
        self.cmd = HandCmd(cmd)
        self.q = self.goal.copy()

    def step(self, dt: float) -> np.ndarray:
        dmax = self.max_rate_per_s * max(dt, 0.0)
        self.q = self.q + np.clip(self.goal - self.q, -dmax, dmax)
        return self.q.copy()


class DualHand:
    """Left/right BinaryHand + input-lost handling; produces the 12-vector for rt/inspire/cmd."""

    def __init__(self, presets_cfg: dict, hands_cfg: dict):
        self.presets_cfg = presets_cfg
        self.hand_model = None
        q_open, q_close = self._preset_pair(presets_cfg)
        rate = float(hands_cfg.get("max_rate_per_s", 2.0))
        self.left = BinaryHand(q_open, q_close, rate)
        self.right = BinaryHand(q_open, q_close, rate)
        cth = float(hands_cfg.get("trigger_close_threshold", 0.6))
        oth = float(hands_cfg.get("trigger_open_threshold", 0.4))
        self.bin_left = TriggerBinarizer(cth, oth)
        self.bin_right = TriggerBinarizer(cth, oth)
        self.on_lost = hands_cfg.get("on_input_lost", "hold")
        self.confirm_s = float(hands_cfg.get("trigger_confirm_s", 0.05))
        self._pending: dict[str, tuple[HandCmd, float] | None] = {"left": None, "right": None}

    @staticmethod
    def _preset_pair(cfg: dict) -> tuple[np.ndarray, np.ndarray]:
        p = cfg["presets"]
        return preset_vector(p[cfg.get("active_open", "open")]), preset_vector(p[cfg.get("active_close", "close")])

    def apply_hand_model(self, model: str) -> dict:
        """Switch OPEN/CLOSE to the simulated hand model's presets (configs/hands_inspire.yaml hand_models.<model>:
        presets / active_open / active_close overrides). The current command is kept; the target ramps over."""
        ov = (self.presets_cfg.get("hand_models") or {}).get(model, {})
        cfg = {**self.presets_cfg, **{k: v for k, v in ov.items() if k != "presets"},
               "presets": {**self.presets_cfg["presets"], **(ov.get("presets") or {})}}
        q_open, q_close = self._preset_pair(cfg)
        for h in (self.left, self.right):
            h.q_open, h.q_close = q_open.copy(), q_close.copy()
        self.hand_model = model
        return {"open": q_open.tolist(), "close": q_close.tolist(), "overrides": sorted(ov)}

    def update_from_triggers(self, left_pressed: float | None, right_pressed: float | None, input_ok: bool,
                             now: float | None = None, last_pose_change: float | None = None):
        """Analog triggers -> discrete commands, applied on hysteresis TRANSITIONS only (so a keyboard
        toggle is not immediately overwritten by a resting trigger; last event wins).
        When the input is LOST the commands are held: a dropped link must not look like a released
        trigger and drop the object.

        With `now` and `last_pose_change` (time of the last controller-pose update, same clock) a transition
        is only committed once it has persisted for `trigger_confirm_s` AND a newer pose sample arrived after
        it: a link that freezes while reporting trigger=0 (the stale timeout has not expired yet) then never
        opens the hand. Without them (keyboard tools, tests) transitions apply immediately."""
        if not input_ok:
            if self.on_lost != "hold":
                raise ValueError(f"unsupported on_input_lost={self.on_lost}")
            return
        confirm = now is not None and last_pose_change is not None
        for side, hand, b, v in (("left", self.left, self.bin_left, left_pressed),
                                 ("right", self.right, self.bin_right, right_pressed)):
            if v is None:
                continue
            nxt = TriggerBinarizer(b.close_th, b.open_th, b.state).update(float(v))  # what b would do
            if nxt == b.state:
                self._pending[side] = None
                continue
            if not confirm:
                b.state = nxt
                hand.set(nxt)
                continue
            if (now - last_pose_change) > self.confirm_s:   # pose not live: do not accumulate a pending change
                self._pending[side] = None
                continue
            p = self._pending[side]
            if p is None or p[0] != nxt:
                self._pending[side] = p = (nxt, float(now))
            if (now - p[1]) >= self.confirm_s and last_pose_change > p[1]:
                b.state = nxt
                hand.set(nxt)
                self._pending[side] = None

    def set_cmds(self, left: HandCmd | None = None, right: HandCmd | None = None):
        """Direct command (keyboard / scripted). Does not touch the trigger hysteresis state."""
        if left is not None:
            self.left.set(left)
        if right is not None:
            self.right.set(right)

    def toggle(self, side: str):
        h = self.left if side == "left" else self.right
        h.set(HandCmd.OPEN if h.cmd == HandCmd.CLOSE else HandCmd.CLOSE)

    def reset(self):
        for h, b in ((self.left, self.bin_left), (self.right, self.bin_right)):
            h.reset(HandCmd.OPEN)
            b.state = HandCmd.OPEN
        self._pending = {"left": None, "right": None}

    def step(self, dt: float) -> np.ndarray:
        q12 = np.empty(12)
        q12[RIGHT_SLICE] = self.right.step(dt)
        q12[LEFT_SLICE] = self.left.step(dt)
        return q12

    def cmds(self) -> dict:
        return {"left": self.left.cmd.name, "right": self.right.cmd.name}
