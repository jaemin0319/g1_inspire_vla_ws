"""Optional video of the external observer camera (scene entity world_camera) during recorded episodes.

Enabled by the env var G1IV_OBSERVER_VIDEO=<dir> (scripts/run_act_eval.sh sim passes it through). For every recorded
frame the observer image is taken at the same simulation instant as the policy observation (25 Hz sim time) and
appended to <dir>/<episode_id>_observer.mp4 (H.264 yuv420p, played at the record rate = real time in simulation time),
plus <episode_id>_observer.json (outcome, frame count, camera pose).

Viewing only: the image never reaches the recorder, the policy bridge or a dataset. Camera pose / size / focal length:
scene_desk.yaml cameras.observer (pos, look_at, width, height, focal_length).
"""
from __future__ import annotations

import json
import queue
import threading
from pathlib import Path

import numpy as np


class ObserverVideo:
    def __init__(self, out_dir: Path, fps: float, crf: int = 18, camera_info: dict | None = None):
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.fps, self.crf, self.camera_info = float(fps), int(crf), camera_info or {}
        self._q: queue.Queue | None = None
        self._th: threading.Thread | None = None
        self._err: list[str] = []
        self.path: Path | None = None
        self.n = 0

    @property
    def active(self) -> bool:
        return self._q is not None

    def start(self, episode_id: str) -> Path:
        self.stop({"outcome": "superseded"})
        self.path = self.dir / f"{episode_id}_observer.mp4"
        self._q, self.n, self._err = queue.Queue(maxsize=64), 0, []
        self._th = threading.Thread(target=self._run, args=(self._q, self.path), daemon=True, name="g1iv-observer-video")
        self._th.start()
        return self.path

    def add(self, rgb: np.ndarray) -> None:
        if self._q is None:
            return
        self._q.put(np.ascontiguousarray(rgb, dtype=np.uint8))   # blocks if the encoder lags: no frame is dropped
        self.n += 1

    def stop(self, info: dict | None = None) -> Path | None:
        if self._q is None:
            return None
        self._q.put(None)
        self._th.join(timeout=300)
        self._q = None
        side = {"video": self.path.name, "frames": self.n, "fps": self.fps, "camera": self.camera_info,
                "encoder_errors": self._err, **(info or {})}
        self.path.with_suffix(".json").write_text(json.dumps(side, indent=1))
        print(f"[g1iv] observer video: {self.path} ({self.n} frames{', ERRORS ' + str(self._err) if self._err else ''})",
              flush=True)
        return self.path

    def _run(self, q: queue.Queue, path: Path) -> None:
        import imageio_ffmpeg

        gen = None
        try:
            while True:
                f = q.get()
                if f is None:
                    break
                if gen is None:
                    h, w = f.shape[:2]
                    gen = imageio_ffmpeg.write_frames(
                        str(path), (w, h), pix_fmt_in="rgb24", pix_fmt_out="yuv420p", fps=self.fps, codec="libx264",
                        quality=None, macro_block_size=2,
                        output_params=["-crf", str(self.crf), "-preset", "medium", "-movflags", "+faststart"])
                    gen.send(None)
                gen.send(f.tobytes())
        except Exception as e:  # noqa: BLE001  (viewing aid: never take the simulator down)
            self._err.append(repr(e))
            while q.get() is not None:   # drain so add() never blocks forever
                pass
        finally:
            if gen is not None:
                try:
                    gen.close()
                except Exception as e:  # noqa: BLE001
                    self._err.append(repr(e))
