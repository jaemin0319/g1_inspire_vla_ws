"""Batch of automatic walk -> grasp -> lift episodes (scripted expert, simulator ground truth) with randomised start
pose (and object position). Needs a running simulator (scripts/run_sim.sh); runs scripts/run_mock_teleop.sh
scripted_walk once per episode and appends one JSON line per episode to <out>.

    scripts/run_sim.sh --headless --scene-set desk.top_surface_height=0.65        (in another terminal)
    python tools/batch_walk_grasp.py -n 20 --record --desk 0.65 --out logs/checks/auto_walk_grasp/batch.jsonl

Several cylinders (scene_desk.yaml extra_objects, fixed positions): --targets red,blue,green --per-target 30
--fixed-objects. Each episode picks the colour with the fewest successes so far (ties at random), resets with
--target <colour>, passes expect_target=<colour> to the plan (the episode fails before recording if the simulator
reports another target) and draws the instruction from --instruction-template (uniform; "{color}" is replaced).

Recorded episodes (--record) go to data/smoke/<date>/ with source=scripted and outcome success|failure from the plan
result (target lifted >= success_lift_m, robot not fallen, no other cylinder pushed > 3 cm). Ranges are uniform;
defaults keep the object 8-18 cm from the desk front edge and the robot 0.6-0.9 m behind its stand pose.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src"))

# instruction phrasings for language-conditioned picking (one is drawn per episode; {color} = target colour)
DEFAULT_TEMPLATES = (
    "walk to the desk and pick up the {color} cylinder",
    "pick up the {color} cylinder",
    "go to the table and grab the {color} cylinder",
    "walk over and lift the {color} cylinder",
    "grab the {color} cylinder on the desk",
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", type=int, default=5, help="maximum number of episodes")
    ap.add_argument("--until-success", type=int, help="stop early once this many episodes succeeded")
    ap.add_argument("--plan", help="plan file (default: the launcher picks configs/scripted_walk_grasp_desk<cm>.yaml)")
    ap.add_argument("--desk", help="desk height for the launcher's plan choice (match the running simulator)")
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--robot-x", nargs=2, type=float, default=[-0.10, 0.10])
    ap.add_argument("--robot-y", nargs=2, type=float, default=[-0.20, 0.20])
    ap.add_argument("--robot-yaw", nargs=2, type=float, default=[-15.0, 15.0], help="degrees")
    ap.add_argument("--object-x", nargs=2, type=float, default=[0.98, 1.08])
    ap.add_argument("--object-y", nargs=2, type=float, default=[-0.30, -0.10])
    ap.add_argument("--fixed-objects", action="store_true", help="keep every cylinder at its scene position")
    ap.add_argument("--shuffle-objects", action="store_true",
                    help="with --targets: put the cylinders on the 3 scene slots in a random order (+-2 cm) per episode")
    ap.add_argument("--targets", help="comma list of cylinder colours (scene_desk.yaml extra_objects), e.g. red,blue,green")
    ap.add_argument("--per-target", type=int, help="with --targets: stop once every colour has this many successes")
    ap.add_argument("--instruction", default="walk to the desk and pick up the red cylinder")
    ap.add_argument("--instruction-template", action="append", default=[],
                    help="with --targets (repeatable; default: 5 built-in phrasings); {color} = target colour")
    ap.add_argument("--task-name", default="walk_pick_cylinder", help="{color} is replaced with --targets")
    ap.add_argument("--out", default="logs/checks/auto_walk_grasp/batch.jsonl")
    ap.add_argument("--timeout", type=float, default=600.0, help="wall seconds per episode")
    ap.add_argument("--hub-arg", action="append", default=[], help="extra hub argument (repeatable)")
    ap.add_argument("--stand-jitter", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    help="uniform +/- (along m, cross m, yaw deg) added per episode to the plan's walk.stand_rel / "
                         "walk.goal_yaw_deg (read from --plan or the --desk plan, with --hub-arg --plan-set overrides); "
                         "0 = off")
    a = ap.parse_args()
    stand0 = None
    if any(a.stand_jitter):
        from g1iv_common.config import load_config
        plan = a.plan or (f"configs/scripted_walk_grasp_desk{int(float(a.desk) * 100 + 0.5):03d}.yaml" if a.desk else None)
        if plan is None:
            ap.error("--stand-jitter needs --plan or --desk (the plan's stand pose is jittered)")
        sets = [v.split("=", 1)[1] for v in a.hub_arg if v.startswith("--plan-set=")]
        sets += [a.hub_arg[k + 1] for k in range(len(a.hub_arg) - 1) if a.hub_arg[k] == "--plan-set"]
        W = load_config(plan, sets).get("walk") or {}
        stand0 = [float(W["stand_rel"][0]), float(W["stand_rel"][1]), float(W.get("goal_yaw_deg", 0.0))]
        print(f"[batch] stand pose jitter +/-{a.stand_jitter} around stand_rel {stand0[:2]}, goal yaw {stand0[2]} deg "
              f"({plan})", flush=True)
    targets = [t.strip() for t in a.targets.split(",") if t.strip()] if a.targets else []
    if a.per_target and not targets:
        ap.error("--per-target needs --targets")
    templates = a.instruction_template or list(DEFAULT_TEMPLATES)
    os.chdir(WS)
    rng = np.random.default_rng(a.seed)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    n_ok = 0
    ok_per = {t: 0 for t in targets}
    i = -1
    for i in range(a.n):
        rp = [float(rng.uniform(*a.robot_x)), float(rng.uniform(*a.robot_y)), float(rng.uniform(*a.robot_yaw))]
        oxy = None if a.fixed_objects else [float(rng.uniform(*a.object_x)), float(rng.uniform(*a.object_y))]
        so = [0.0, 0.0, 0.0]
        if any(a.stand_jitter):          # no random draw when off -> earlier seeds reproduce
            so = [float(rng.uniform(-j, j)) for j in a.stand_jitter]
        target, tmpl_i = None, None
        instruction, task_name = a.instruction, a.task_name
        if targets:
            todo = [t for t in targets if not a.per_target or ok_per[t] < a.per_target]
            least = min(ok_per[t] for t in todo)
            target = str(rng.choice([t for t in todo if ok_per[t] == least]))
            tmpl_i = int(rng.integers(len(templates)))
            instruction = templates[tmpl_i].format(color=target)
            task_name = a.task_name.format(color=target) if "{color}" in a.task_name else f"{a.task_name}_{target}"
        reset = ["bash", "scripts/record_episode.sh", "reset", "--robot-pose", *map(str, rp)]
        if oxy is not None:
            reset += ["--object-xy", *map(str, oxy)]
        if target:
            reset += ["--target", target]
        objs = None
        if a.shuffle_objects and targets:
            slots = [(1.02, -0.20), (1.02, 0.05), (1.02, -0.45)]          # scene_desk.yaml: red(primary), blue, green
            perm = rng.permutation(3)
            objs = {c: [slots[j][0] + float(rng.uniform(-0.02, 0.02)), slots[j][1] + float(rng.uniform(-0.02, 0.02))]
                    for c, j in zip(("red", "blue", "green"), perm)}
            reset += ["--objects-xy", *[f"{c}={x:.4f},{y:.4f}" for c, (x, y) in objs.items()]]
        subprocess.run(reset, check=True, capture_output=True, env=env)
        time.sleep(1.0)
        cmd = ["bash", "scripts/run_mock_teleop.sh", "scripted_walk", "--no-console",
               *(["--plan", a.plan] if a.plan else []), *(["--desk", a.desk] if a.desk else []),
               "--instruction", instruction, "--task-name", task_name, *a.hub_arg]
        if target:
            cmd += ["--plan-set", f"expect_target={target}"]
        if stand0 is not None:
            cmd += ["--plan-set", f"walk.stand_rel=[{stand0[0] + so[0]:.4f}, {stand0[1] + so[1]:.4f}]",
                    "--plan-set", f"walk.goal_yaw_deg={stand0[2] + so[2]:.2f}"]
        if a.record:
            cmd.append("--record")
        t0 = time.time()
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=a.timeout, env=env)
            log = p.stdout + p.stderr
        except subprocess.TimeoutExpired as e:
            log = (e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
            pidf = WS / "logs/run/hub.pid"          # stop only this project's hub (never the simulator)
            if pidf.exists():
                try:
                    os.kill(int(pidf.read_text().strip()), 2)
                    time.sleep(3.0)
                except (ProcessLookupError, ValueError):
                    pass
        trace = next((ln.split("trace -> ")[1].strip() for ln in log.splitlines() if "trace -> " in ln), None)
        res = json.load(open(trace)).get("result", {}) if trace else {"error": "no hub trace"}
        ok = bool(res.get("success"))
        if target and ok and res.get("target") != target:   # cannot happen with expect_target; keep the check
            ok, res["error"] = False, f"target mismatch {res.get('target')!r} != {target!r}"
        n_ok += ok
        if target and ok:
            ok_per[target] += 1
        row = {"i": i, "seed": a.seed, "robot_pose": rp, "object_xy": oxy, "stand_offset": so, "objects_xy": objs,
               "target": target, "instruction": instruction,
               "template": tmpl_i, "success": ok, "object_lift_m": res.get("object_lift_m"), "fallen": res.get("fallen"),
               "others_moved_m": res.get("others_moved_m"), "error": res.get("error"), "nav": res.get("nav"),
               "trace": trace, "wall_s": round(time.time() - t0, 1), "plan": a.plan, "recorded": a.record}
        with out.open("a") as f:
            f.write(json.dumps(row, default=float) + "\n")
        nav = res.get("nav") or {}
        print(f"[batch] {i + 1}/{a.n} start {np.round(rp, 2).tolist()} "
              f"{'target ' + target if target else 'obj ' + str(np.round(oxy, 3).tolist() if oxy else 'fixed')} -> "
              f"{'SUCCESS' if ok else 'failure'} lift {res.get('object_lift_m')} nav_ok {nav.get('ok')} "
              f"err ({nav.get('along')}, {nav.get('cross')}, {nav.get('yaw_err_deg')}) "
              f"corr {nav.get('n_corr')} others {res.get('others_moved_m')} {res.get('error') or ''} "
              f"[{row['wall_s']} s]" + (f" per-target {ok_per}" if targets else ""), flush=True)
        if a.until_success and n_ok >= a.until_success:
            break
        if a.per_target and all(v >= a.per_target for v in ok_per.values()):
            break
    print(f"[batch] {n_ok}/{i + 1} success -> {out}" + (f" per-target {ok_per}" if targets else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
