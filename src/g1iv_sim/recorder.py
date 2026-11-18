"""Episode recorder (pure python + numpy + PIL; no Isaac imports so it is unit-testable).

On-disk layout of one episode (never overwritten; ids are unique):
  <root>/<YYYYMMDD>/<episode_id>/
      meta.json                 task, instruction, source, versions, joint names, outcome, statistics
      steps.jsonl               one JSON object per recorded frame (see docs/DATASET_SCHEMA.md)
      arrays.npz                the numeric fields of steps.jsonl stacked (written at stop)
      images/<cam>/<frame:06d>.jpg
      depth/<cam>/<frame:06d>.png   (optional, uint16 millimetres)
      teleop_meta.jsonl         every hub metadata message received during the episode
"""
from __future__ import annotations

import datetime as _dt
import json
import os
import queue
import threading
import time
import uuid
from pathlib import Path

import numpy as np

SCHEMA_VERSION = "g1iv-raw-1"


def _flatten(prefix: str, d: dict, out: dict):
    for k, v in d.items():
        key = f"{prefix}/{k}" if prefix else k
        if isinstance(v, dict):
            _flatten(key, v, out)
        else:
            out[key] = v


class EpisodeRecorder:
    def __init__(self, raw_root: Path, smoke_root: Path, smoke_sources=("mock", "scripted"),
                 jpeg_quality: int = 95, queue_max: int = 256):
        self.raw_root, self.smoke_root = Path(raw_root), Path(smoke_root)
        self.smoke_sources = set(smoke_sources)
        self.jpeg_quality = int(jpeg_quality)
        self.q: queue.Queue = queue.Queue(maxsize=int(queue_max))
        self.active = False
        self.dir: Path | None = None
        self.meta: dict = {}
        self.n_frames = 0
        self.n_dropped = 0
        self._rows: list[dict] = []
        self._meta_f = None
        self._steps_f = None
        self._lock = threading.Lock()
        self._writer = threading.Thread(target=self._writer_loop, daemon=True, name="g1iv-recorder")
        self._writer.start()

    # ------------------------------------------------------------------ episode lifecycle
    def start(self, meta: dict) -> Path:
        if self.active:
            raise RuntimeError("episode already active")
        src = meta.get("source", "unknown")
        root = self.smoke_root if src in self.smoke_sources else self.raw_root
        now = _dt.datetime.now()
        eid = f"{now:%Y%m%d_%H%M%S}_{src}_{uuid.uuid4().hex[:8]}"
        d = root / f"{now:%Y%m%d}" / eid
        d.mkdir(parents=True, exist_ok=False)  # never reuse / overwrite
        self.dir = d
        self.meta = {"schema": SCHEMA_VERSION, "episode_id": eid, "is_smoke_data": src in self.smoke_sources,
                     "start_wall_time": time.time(), "start_iso": now.isoformat(timespec="milliseconds"), **meta}
        self.n_frames = self.n_dropped = 0
        self._rows = []
        self._steps_f = open(d / "steps.jsonl", "w", encoding="utf-8")
        self._meta_f = open(d / "teleop_meta.jsonl", "w", encoding="utf-8")
        (d / "meta.json").write_text(json.dumps(self.meta, indent=1))
        self.active = True
        return d

    def add_frame(self, row: dict, images: dict[str, np.ndarray], depths: dict[str, np.ndarray] | None = None) -> bool:
        """row: JSON-able dict (numbers / lists). images: alias -> HxWx3 uint8 (already copied)."""
        if not self.active:
            return False
        idx = self.n_frames
        row = {"frame_index": idx, **row}
        files = {a: f"images/{a}/{idx:06d}.jpg" for a in images}
        row.setdefault("obs", {})["image_files"] = files
        if depths:
            row["obs"]["depth_files"] = {a: f"depth/{a}/{idx:06d}.png" for a in depths}
        try:
            self.q.put_nowait(("frame", self.dir, idx, row, images, depths or {}))
        except queue.Full:
            self.n_dropped += 1
            row["dropped_by_writer"] = True
            with self._lock:
                self._steps_f.write(json.dumps(row) + "\n")
            return False
        self.n_frames += 1
        return True

    def add_teleop_meta(self, raw: str):
        if self.active and self._meta_f is not None:
            self._meta_f.write(raw.strip() + "\n")

    def stop(self, outcome: str, end_reason: str, extra: dict | None = None) -> dict:
        if not self.active:
            return {}
        self.active = False
        done = threading.Event()
        self.q.put(("close", self.dir, done))
        done.wait(timeout=120)
        rows = self._rows
        self._steps_f.close()
        self._meta_f.close()
        arrays = {}
        if rows:
            flat = []
            for r in rows:
                f = {}
                _flatten("", {k: v for k, v in r.items() if k not in ("teleop_raw",)}, f)
                flat.append(f)
            keys = sorted({k for f in flat for k in f})
            for k in keys:
                vals = [f.get(k) for f in flat]
                if all(isinstance(v, (int, float, bool)) for v in vals) or all(
                        isinstance(v, list) and v and all(isinstance(x, (int, float, bool)) for x in v) for v in vals):
                    try:
                        arrays[k] = np.asarray(vals)
                    except ValueError:
                        pass
            np.savez_compressed(self.dir / "arrays.npz", **arrays)
        self.meta.update({"end_wall_time": time.time(), "outcome": outcome, "end_reason": end_reason,
                          "n_frames": self.n_frames, "n_dropped_by_writer": self.n_dropped,
                          "array_keys": sorted(arrays)})
        if extra:
            self.meta.update(extra)
        (self.dir / "meta.json").write_text(json.dumps(self.meta, indent=1))
        out = {"dir": str(self.dir), **{k: self.meta[k] for k in ("episode_id", "outcome", "end_reason", "n_frames")}}
        return out

    # ------------------------------------------------------------------ writer thread
    def _writer_loop(self):
        from PIL import Image  # pillow is part of the Isaac Sim env

        while True:
            item = self.q.get()
            if item[0] == "close":
                item[2].set()
                continue
            _, d, idx, row, images, depths = item
            try:
                for alias, img in images.items():
                    p = d / "images" / alias
                    p.mkdir(parents=True, exist_ok=True)
                    Image.fromarray(img).save(p / f"{idx:06d}.jpg", quality=self.jpeg_quality)
                for alias, dep in depths.items():
                    p = d / "depth" / alias
                    p.mkdir(parents=True, exist_ok=True)
                    mm = np.clip(np.nan_to_num(dep, nan=0.0, posinf=0.0) * 1000.0, 0, 65535).astype(np.uint16)
                    Image.fromarray(mm.squeeze()).save(p / f"{idx:06d}.png")
                with self._lock:
                    self._rows.append(row)
                    self._steps_f.write(json.dumps(row) + "\n")
            except Exception as e:  # noqa: BLE001  (never kill the sim because of disk I/O)
                row["writer_error"] = repr(e)
                with self._lock:
                    self._rows.append(row)
                    self._steps_f.write(json.dumps(row) + "\n")


class FramePairer:
    """Decides, per completed control step s, whether to emit a frame and whether to capture an
    observation for the next step. Frame for action step s = (obs captured after step s-1, action of s).
    Frames are taken every N steps of SIMULATION time starting at the step after record_start."""

    def __init__(self, every_n: int):
        if every_n < 1:
            raise ValueError("every_n must be >= 1")
        self.n = int(every_n)
        self.k0 = None
        self.pending_target = None

    def start(self, step_now: int) -> int:
        """Call at record start (between steps). Returns the target step of the obs captured now."""
        self.k0 = step_now + 1
        self.pending_target = self.k0
        return self.k0

    def on_step(self, s: int) -> tuple[bool, bool]:
        emit = self.pending_target == s
        if emit:
            self.pending_target = None
        capture = self.k0 is not None and (s + 1 - self.k0) % self.n == 0
        if capture:
            self.pending_target = s + 1
        return emit, capture

    def stop(self):
        self.k0 = None
        self.pending_target = None
