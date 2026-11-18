"""Record rt/g1iv/sim_status (project extension) to a JSON file for N seconds. Read-only.

    python -m g1iv_teleop.status_log --seconds 30 --out logs/checks/x.json
"""
from __future__ import annotations

import argparse
import json
import time

from g1iv_common import topics


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dds-iface", default="lo")
    a = ap.parse_args()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

    ChannelFactoryInitialize(topics.SIM_DDS_DOMAIN, a.dds_iface or None)
    rows = []

    def cb(m):
        try:
            d = json.loads(m.data)
        except Exception:
            return
        d["recv_wall"] = time.time()
        rows.append(d)

    sub = ChannelSubscriber(topics.T_SIM_STATUS, String_)
    sub.Init(cb, 256)
    time.sleep(a.seconds)
    with open(a.out, "w") as f:
        json.dump(rows, f)
    print(f"[status_log] {len(rows)} status messages -> {a.out}", flush=True)
    return 0 if rows else 2


if __name__ == "__main__":
    raise SystemExit(main())
