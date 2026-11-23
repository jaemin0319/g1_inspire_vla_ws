"""Length-prefixed messages over a stream socket: 8-byte header (JSON length, payload length) + JSON + raw arrays.

Used by the policy bridge (simulator, src/g1iv_sim/ext.py), the policy runner (tools/run_act_policy.py, teleop env)
and the policy server (tools/act_policy_server.py, LeRobot env). No pickle: the three sides run different Python
(3.10 / 3.11 / 3.12) and numpy (1.x / 2.x) versions. Arrays travel as dtype.str + shape + bytes.
"""
from __future__ import annotations

import json
import socket
import struct

import numpy as np

_HDR = struct.Struct("!II")


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray(n)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if k == 0:
            raise ConnectionError("peer closed the connection")
        got += k
    return bytes(buf)


def send_msg(sock: socket.socket, meta: dict, arrays: dict | None = None) -> None:
    specs, blobs = [], []
    for name, a in (arrays or {}).items():
        a = np.ascontiguousarray(a)
        specs.append({"name": name, "dtype": a.dtype.str, "shape": list(a.shape)})
        blobs.append(a.tobytes())
    head = json.dumps({**meta, "_arrays": specs}).encode()
    payload = b"".join(blobs)
    sock.sendall(_HDR.pack(len(head), len(payload)) + head)
    if payload:
        sock.sendall(payload)


def recv_msg(sock: socket.socket) -> tuple[dict, dict]:
    hl, pl = _HDR.unpack(_recv_exact(sock, _HDR.size))
    meta = json.loads(_recv_exact(sock, hl))
    payload = _recv_exact(sock, pl) if pl else b""
    arrays, off = {}, 0
    for s in meta.pop("_arrays", []):
        dt = np.dtype(s["dtype"])
        count = int(np.prod(s["shape"], dtype=np.int64))
        arrays[s["name"]] = np.frombuffer(payload, dt, count=count, offset=off).reshape(s["shape"])
        off += count * dt.itemsize
    return meta, arrays


def connect_unix(path: str, timeout_s: float | None = None) -> socket.socket:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(path)
    s.settimeout(timeout_s)
    return s
