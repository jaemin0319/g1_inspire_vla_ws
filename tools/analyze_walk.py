"""Per-phase walking response from a status_log JSON (project tool). Usage: python tools/analyze_walk.py <status.json>"""
import json, math, sys
import numpy as np

rows = json.load(open(sys.argv[1]))
st = np.array([r["sim_time"] for r in rows]); pos = np.array([r["base_pos_w"] for r in rows])
yaw = np.unwrap(np.array([r["base_yaw"] for r in rows]))
cmd = np.array([r["loco_cmd_applied"] or [0, 0, 0, 0.8] for r in rows])
vel = np.array([r["base_lin_vel_b"] for r in rows]); wz = np.array([r["base_ang_vel_b"][2] for r in rows])
print("rows", len(rows), "sim %.1f..%.1f" % (st[0], st[-1]), "fallen any:", any(r["fallen"] for r in rows),
      "z %.3f..%.3f" % (pos[:, 2].min(), pos[:, 2].max()), "rtf mean %.2f" % np.nanmean([r["rtf"] or np.nan for r in rows]))
segs, cur = [], None
for i in range(len(rows)):
    key = tuple(int(v) for v in np.sign(np.round(cmd[i, :3], 2)))
    if key != cur:
        segs.append([key, i, i]); cur = key
    else:
        segs[-1][2] = i
names = {(1, 0, 0): "forward", (0, 1, 0): "left", (0, -1, 0): "right", (-1, 0, 0): "back", (0, 0, 1): "turn L", (0, 0, -1): "turn R", (0, 0, 0): "stop"}
for key, a, b in segs:
    if st[b] - st[a] < 0.3:
        continue
    c = cmd[a:b + 1, :3]; dp = pos[b, :2] - pos[a, :2]; y0 = yaw[a]
    R = np.array([[math.cos(y0), math.sin(y0)], [-math.sin(y0), math.cos(y0)]]); db = R @ dp
    ext = lambda v: v.max() if abs(v.max()) >= abs(v.min()) else v.min()
    print("%-8s sim %5.2fs cmd vx %+.2f vy %+.2f wz %+.2f | body dx %+.3f dy %+.3f m dyaw %+6.1f deg | mean v_b x %+.2f y %+.2f wz %+.2f" % (
        names.get(key, str(key)), st[b] - st[a], ext(c[:, 0]), ext(c[:, 1]), ext(c[:, 2]), db[0], db[1],
        math.degrees(yaw[b] - yaw[a]), vel[a:b + 1, 0].mean(), vel[a:b + 1, 1].mean(), wz[a:b + 1].mean()))
