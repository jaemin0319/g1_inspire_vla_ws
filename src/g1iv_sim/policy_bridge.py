"""Lockstep observation bridge for running a learned policy in the simulator (opt-in, G1IV_POLICY_BRIDGE=<socket>).

While an episode is RECORDED and a client is connected, every observation the recorder captures (the observation
BEFORE control step s_k, i.e. exactly what becomes frame k of the recording) is sent to the client: the four policy
camera images (uint8 HxWx3, recording aliases) + arm_q / hand_q_norm + evaluation ground truth (logging only). The
simulator then WAITS for the client's reply before stepping on. The client publishes its commands on the normal DDS
topics (rt/lowcmd, rt/inspire/cmd, rt/run_command/cmd) before replying, so they are applied during step s_k, the same
pairing as in the recorded demonstrations, independent of inference time. A reply {"stop": outcome} ends the episode.

No client connected / not recording -> no effect. A client that does not answer within the timeout is dropped.
"""
from __future__ import annotations

import os
import socket
import threading
from pathlib import Path

from g1iv_common import framing


class PolicyBridge:
    def __init__(self, path: str, timeout_s: float = 60.0):
        self.path, self.timeout = str(path), float(timeout_s)
        self.conn: socket.socket | None = None
        self.lock = threading.Lock()
        self.n_frames = 0
        p = Path(self.path)
        p.parent.mkdir(parents=True, exist_ok=True)
        if p.exists() or p.is_socket():
            p.unlink()
        self.srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.srv.bind(self.path)
        os.chmod(self.path, 0o600)
        self.srv.listen(1)
        threading.Thread(target=self._accept, daemon=True, name="g1iv-policy-bridge").start()
        print(f"[g1iv] policy bridge listening on {self.path} (reply timeout {self.timeout:.0f} s)", flush=True)

    def _accept(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            c.settimeout(self.timeout)
            with self.lock:
                old, self.conn = self.conn, c
            if old is not None:
                try:
                    old.close()
                except OSError:
                    pass
            print("[g1iv] policy bridge: client connected", flush=True)

    @property
    def connected(self) -> bool:
        return self.conn is not None

    def exchange(self, meta: dict, arrays: dict) -> dict | None:
        """Send one observation and wait for the reply. None if no client (or the client was dropped)."""
        with self.lock:
            c = self.conn
        if c is None:
            return None
        try:
            framing.send_msg(c, meta, arrays)
            reply, _ = framing.recv_msg(c)
            self.n_frames += 1
            return reply
        except Exception as e:  # noqa: BLE001
            print(f"[g1iv] policy bridge: client dropped ({e!r})", flush=True)
            with self.lock:
                if self.conn is c:
                    self.conn = None
            try:
                c.close()
            except OSError:
                pass
            return None

    def close(self):
        try:
            self.srv.close()
            Path(self.path).unlink(missing_ok=True)
        except OSError:
            pass
