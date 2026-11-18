"""Read-only DDS probe for the running simulator (domain 1, default interface lo).

    python -m g1iv_teleop.probe --seconds 10
Prints, once per second: rt/lowstate rate, IMU roll/pitch/yaw, leg/arm joint ranges, and (when the project
extension is on) the base height / fallen flag from rt/g1iv/sim_status. Publishes nothing.
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time

from g1iv_common import topics


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--dds-iface", default="lo")
    ap.add_argument("--json", default="", help="write the samples to this file")
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(topics.SIM_DDS_DOMAIN, a.dds_iface or None)
    lock = threading.Lock()
    st = {"n": 0, "last": None, "status": None}

    st["changed"] = 0
    st["prev_q"] = None

    def on_low(msg):
        q = tuple(round(msg.motor_state[i].q, 6) for i in range(29))
        with lock:
            st["n"] += 1
            st["last"] = msg
            if q != st["prev_q"]:   # official tick increments on every publish, even with unchanged data
                st["changed"] += 1
                st["prev_q"] = q

    def on_status(msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with lock:
            st["status"] = d

    s1 = ChannelSubscriber("rt/lowstate", LowState_)
    s1.Init(on_low, 10)
    s2 = ChannelSubscriber(topics.T_SIM_STATUS, String_)
    s2.Init(on_status, 10)

    samples = []
    t0 = time.time()
    n_prev = c_prev = 0
    while time.time() - t0 < a.seconds:
        time.sleep(1.0)
        with lock:
            n, m, status, c = st["n"], st["last"], st["status"], st["changed"]
        row = {"t": round(time.time() - t0, 1), "lowstate_hz": n - n_prev, "q_changes_per_s": c - c_prev}
        n_prev, c_prev = n, c
        if m is not None:
            # The official sim leaves imu_state.rpy at 0 and fills quaternion as [x, y, z, w] (g1_robot_dds.py).
            x, y, z, w = (float(v) for v in m.imu_state.quaternion)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
            q = [m.motor_state[i].q for i in range(29)]
            row.update(tilt_deg=round(tilt, 2), quat_xyzw=[round(v, 4) for v in (x, y, z, w)],
                       acc=[round(float(v), 2) for v in m.imu_state.accelerometer],
                       gyro=[round(float(v), 3) for v in m.imu_state.gyroscope],
                       leg_q_minmax=[round(min(q[:12]), 3), round(max(q[:12]), 3)],
                       arm_q_minmax=[round(min(q[15:29]), 3), round(max(q[15:29]), 3)])
        if status:
            pos = status.get("base_pos_w") or [None, None, None]
            row.update(base_xyz=[round(v, 3) for v in pos] if pos[0] is not None else None,
                       fallen=status.get("fallen"), sim_time=status.get("sim_time"), rtf=status.get("rtf"),
                       control_hz_wall=status.get("control_hz_wall"), loco_cmd=status.get("loco_cmd_applied"))
        samples.append(row)
        print(json.dumps(row), flush=True)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(samples, f, indent=1)
    return 0 if st["n"] > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
