"""Right hand - object position (robot frame, cm) at the frame the policy first closes the hand, per eval episode.
Expert (walk_far_v2_sel, 55 episodes): (-16.8, -5.8) cm, std < 0.1 cm.
    python hand_close.py <logs/policy_eval/..._far> [...]
Reads the full recordings under /home/jammin/g1iv_store/policy_eval/<date>/<episode_id>/arrays.npz."""
import glob
import json
import math
import sys

import numpy as np

for D in sys.argv[1:]:
    out = []
    for line in open(D + "/summary.jsonl"):
        r = json.loads(line)
        P = glob.glob(f"/home/jammin/g1iv_store/policy_eval/*/{r['episode_id']}/arrays.npz")
        if not P:
            out.append(f"ep{r['i']}:missing")
            continue
        a = np.load(P[0])
        tgt = a["action/hand_q_norm_target"]
        closed = np.abs(tgt - tgt[0]).max(1) > 0.3
        tag = "S" if r["outcome"] == "success" else "F"
        if not closed.any():
            out.append(f"{tag}(open)")
            continue
        ic = int(np.argmax(closed))
        w, x, y, z = a["obs/base_quat_w"][ic]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        d = a["eval/body_pose_w/right_hand"][ic, :3] - a["eval/object_pos_w"][ic]
        c, s = math.cos(-yaw), math.sin(-yaw)
        out.append(f"{tag}({(c * d[0] - s * d[1]) * 100:+.1f},{(s * d[0] + c * d[1]) * 100:+.1f})")
    print(f"{D.rstrip('/').split('/')[-1]}\n  hand-obj at close (x,y cm): " + " ".join(out))
