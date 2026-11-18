"""Scripted-grasp diagnosis from a status_log JSON (+ the latest hub trace): actual R_ee (sim) vs object.
Usage: python tools/analyze_scripted.py <status.json> [every_n]"""
import glob, json, sys
import numpy as np

def qm(q):  # wxyz -> R
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)], [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)], [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])

rows = json.load(open(sys.argv[1])); n = int(sys.argv[2]) if len(sys.argv) > 2 else 25
f = sorted(glob.glob("logs/teleop_runs/*_scripted.json"))[-1]; d = json.load(open(f))
print(f.split("/")[-1], "result:", d["result"], "ik_fail", d["n_ik_fail"])
print("phases (plan clock):", [(p["phase"], round(p["t"], 1)) for p in d["source_log"]])
for r in rows[::n]:
    ev = r.get("eval_gt", {}); bp = ev.get("body_pose_w", {}).get("right_wrist_yaw")
    if not bp:
        continue
    pb = np.array(r["base_pos_w"]); yaw = r["base_yaw"]
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    Rw = qm(bp[3:]); p_ree_b = Rz.T @ (np.array(bp[:3]) + Rw @ np.array([0.05, 0, 0]) - pb); R_b = Rz.T @ Rw
    obj_b = Rz.T @ (np.array(ev["object_pos_w"]) - pb)
    print("sim_t %6.2f R_ee %s fingers(x) %s | obj in R_ee %s | contact %5.1f N lift %+.3f base %s" % (
        r["sim_time"], np.round(p_ree_b, 3).tolist(), np.round(R_b[:, 0], 2).tolist(), np.round(R_b.T @ (obj_b - p_ree_b), 3).tolist(),
        ev.get("contact_force_total") or 0.0, ev.get("object_lift_m") or 0.0, np.round(pb[:2], 3).tolist()))
