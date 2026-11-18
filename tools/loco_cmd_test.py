"""Publish walking commands on rt/run_command/cmd exactly like the official send_commands_keyboard.py
(String_ "[vx, vy, wz, height]", 50 Hz), one phase after another, then zero. Simulator only (DDS domain 1, lo).

    python tools/loco_cmd_test.py --phase 6 -- "0,0,1.57" "0,0,0" "0,0.5,0" "0,0,0" "-0.6,0,0" "0,0,0"
"""
import argparse
import time

ap = argparse.ArgumentParser()
ap.add_argument("--phase", type=float, default=6.0, help="wall seconds per phase")
ap.add_argument("--height", type=float, default=0.8)
ap.add_argument("--dds-iface", default="lo")
ap.add_argument("cmds", nargs="+")
a = ap.parse_args()

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher  # noqa: E402
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_  # noqa: E402

ChannelFactoryInitialize(1, a.dds_iface or None)
pub = ChannelPublisher("rt/run_command/cmd", String_)
pub.Init()
for c in a.cmds + ["0,0,0"]:
    v = [float(x) for x in c.split(",")]
    msg = [v[0], v[1], v[2], a.height]
    print(f"[loco_cmd_test] {msg} for {a.phase:.1f} s", flush=True)
    t_end = time.time() + a.phase
    while time.time() < t_end:
        pub.Write(String_(data=str(msg)))
        time.sleep(0.02)
