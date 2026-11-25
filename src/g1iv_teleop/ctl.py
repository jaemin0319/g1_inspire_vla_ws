"""Command-line control of the simulator extension (rt/g1iv/control) and status probe.

  python -m g1iv_teleop.ctl start --instruction "pick up the red cylinder" --source keyboard
  python -m g1iv_teleop.ctl stop --outcome success|failure|undetermined
  python -m g1iv_teleop.ctl abort
  python -m g1iv_teleop.ctl reset [--robot-pose X Y YAW_DEG] [--object-xy X Y] [--seed N] [--target blue]
  python -m g1iv_teleop.ctl mark --label grasp_closed
  python -m g1iv_teleop.ctl status [--seconds 2]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from g1iv_common import topics  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["start", "stop", "abort", "reset", "mark", "status"])
    ap.add_argument("--instruction", default="pick up the red cylinder")
    ap.add_argument("--task-name", default="pick_cylinder")
    ap.add_argument("--source", default="keyboard", help="pico|keyboard|mock|scripted (mock/scripted -> data/smoke)")
    ap.add_argument("--operator", default=os.environ.get("USER", ""))
    ap.add_argument("--notes", default="")
    ap.add_argument("--outcome", default="undetermined", choices=list(topics.OUTCOMES))
    ap.add_argument("--robot-pose", nargs=3, type=float)
    ap.add_argument("--object-xy", nargs=2, type=float)
    ap.add_argument("--objects-xy", nargs="+", help="reset: NAME=X,Y per cylinder, e.g. red=1.02,0.05 blue=1.02,-0.45")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--target", help="reset: cylinder to pick by colour name (scene_desk.yaml extra_objects; default "
                                     "the primary one); --object-xy then moves this one")
    ap.add_argument("--label", default="")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--dds-iface", default=os.environ.get("G1IV_DDS_IFACE", "lo"))
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

    ChannelFactoryInitialize(topics.SIM_DDS_DOMAIN, a.dds_iface or None)
    if a.cmd == "status":
        got = []
        sub = ChannelSubscriber(topics.T_SIM_STATUS, String_)
        sub.Init(lambda m: got.append(json.loads(m.data)), 4)
        t_end = time.time() + a.seconds
        while time.time() < t_end:
            time.sleep(0.05)
        if not got:
            print("no rt/g1iv/sim_status received (is scripts/run_sim.sh running with the same --dds-iface?)")
            sys.exit(2)
        st = got[-1]
        print(json.dumps({k: st[k] for k in ("sim_time", "step", "rtf", "control_hz_wall", "base_pos_w", "fallen",
                                             "loco_cmd_applied", "reset_count", "recording") if k in st}, indent=1))
        print(f"messages in {a.seconds:.1f}s: {len(got)}")
        return
    pub = ChannelPublisher(topics.T_CONTROL, String_)
    pub.Init()
    time.sleep(0.5)  # discovery
    kw = {}
    if a.cmd == "start":
        name, kw = "record_start", dict(instruction=a.instruction, task_name=a.task_name, source=a.source,
                                        operator=a.operator, notes=a.notes)
    elif a.cmd == "stop":
        name, kw = "record_stop", dict(outcome=a.outcome)
    elif a.cmd == "abort":
        name = "record_abort"
    elif a.cmd == "mark":
        name, kw = "mark", dict(label=a.label)
    else:
        name = "reset"
        if a.robot_pose:
            kw["robot_pose"] = a.robot_pose
        if a.object_xy:
            kw["object_xy"] = a.object_xy
        if a.objects_xy:
            kw["objects_xy"] = {n: [float(v) for v in xy.split(",")] for n, xy in (s.split("=", 1) for s in a.objects_xy)}
        if a.seed is not None:
            kw["seed"] = a.seed
        if a.target:
            kw["target"] = a.target
    for _ in range(3):  # best-effort delivery without a reply channel; the sim de-duplicates by state
        pub.Write(String_(data=topics.control_msg(name, **kw)))
        break
    time.sleep(0.3)
    print(f"sent {name} {kw}")


if __name__ == "__main__":
    main()
