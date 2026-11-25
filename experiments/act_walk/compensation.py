"""Arm compensation ratio: at the frame the policy first closes the hand, regress (right hand - pelvis) on
(object - pelvis) in the robot frame, x (fore-aft) and y (sideways). 1.0 = the arm fully absorbs the stance error,
0 = not at all. v2 reference (8 folders *_v2_0[3-7]0000_nas25_*far, 81 hand closes, 2026-11-23): y 0.53 (corr 0.95),
x 0.73 (corr 0.91); 30k+40k seed20261120 only: y 0.56, x 0.72. (docs/ACT_EVAL.md's earlier 0.47 / 0.68 came from an
unsaved calculation and is not reproduced by any variant tried: hand or wrist point, 30k-40k / 30k-70k subsets.)
    python compensation.py <logs/policy_eval/..._far> [...] [--raw-root /home/jammin/g1iv_store/policy_eval]
Reads the full recordings <raw-root>/<date>/<episode_id>/arrays.npz (same keys as hand_close.py)."""
import argparse
import glob
import json
import math

import numpy as np


def at_close(npz_path):
    a = np.load(npz_path)
    tgt = a["action/hand_q_norm_target"]
    closed = np.abs(tgt - tgt[0]).max(1) > 0.3          # same rule as hand_close.py
    if not closed.any():
        return None
    i = int(np.argmax(closed))
    w, x, y, z = a["obs/base_quat_w"][i]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    R = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])   # world -> robot (xy)
    p = a["obs/base_pos_w"][i, :2]
    obj = R @ (a["eval/object_pos_w"][i, :2] - p)
    hand = R @ (a["eval/body_pose_w/right_hand"][i, :2] - p)
    return obj, hand


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--raw-root", default="/home/jammin/g1iv_store/policy_eval")
    a = ap.parse_args()
    O, Hn, ok = [], [], []
    for d in a.dirs:
        for line in open(f"{d}/summary.jsonl"):
            r = json.loads(line)
            P = glob.glob(f"{a.raw_root}/*/{r['episode_id']}/arrays.npz")
            res = at_close(P[0]) if P else None
            if res is None:
                continue
            O.append(res[0]); Hn.append(res[1]); ok.append(r["outcome"] == "success")
    if len(O) < 3:
        raise SystemExit(f"only {len(O)} episodes with a hand close: need at least 3")
    O, Hn = np.array(O), np.array(Hn)
    for k, name in ((1, "y (sideways)"), (0, "x (fore-aft)")):
        slope = np.polyfit(O[:, k], Hn[:, k], 1)[0]
        corr = np.corrcoef(O[:, k], Hn[:, k])[0, 1]
        print(f"{name}: compensation {slope:.2f} (corr {corr:.2f}), object-pelvis std {O[:, k].std() * 100:.1f} cm")
    print(f"episodes with a hand close: {len(O)}, successes {sum(ok)}")


if __name__ == "__main__":
    main()
