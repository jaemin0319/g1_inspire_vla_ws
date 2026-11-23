"""LeRobot ACT policy server for simulator rollouts (tools/run_act_policy.py is the client).

    CUDA_VISIBLE_DEVICES=<GPU> /mnt/isaac/so101_ws/.venv-train/bin/python tools/act_policy_server.py \
        --checkpoint <run>/checkpoints/040000/pretrained_model [--socket logs/run/act_policy.sock] \
        [--temporal-ensemble 0.01 | --n-action-steps N [--n-action-steps-still M]] [--jpeg 95]

Runs in a LeRobot >= 0.4 environment (the so101 training venv, LeRobot 0.6.1). Loads the policy and its saved
pre/post-processors (normalisation stats) from the checkpoint, then serves requests over a Unix socket
(src/g1iv_common/framing.py):
  {"cmd": "reset"}                                    -> policy.reset() (new episode: clears the action queue)
  {"cmd": "act"} + observation.state (26 float32) + observation.images.<key> (HxWx3 uint8)
                                                      -> {"action": float32[29]} (unnormalised)
  {"cmd": "info"}                                     -> feature shapes, chunking settings
--jpeg Q re-encodes each camera image as JPEG (quality Q) before inference, like the recorder does before the
dataset conversion (0 = off). Inference = lerobot.common.control_utils.predict_action (same as lerobot-record).

--n-action-steps-still M (with --n-action-steps N): a freshly predicted chunk whose walking command stays below
--walk-thr (|vx|, |vy|, |wz|, physical units) over the whole chunk is executed for M steps instead of N. Why
(docs/ACT_EVAL.md §5): the scripted expert keeps the closed hand still for ~35 frames before the lift starts. Replanning
every N=25 frames from that motionless scene re-plans "wait ~35 more frames" each time and the lift never starts;
executing the whole 2 s chunk (M=50) carries the plan into the lift. While walking, N=25 keeps the steering closed-loop.
"""
from __future__ import annotations

import argparse
import io
import os
import socket
import sys
import time
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src"))
from g1iv_common import framing  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", required=True, type=Path, help=".../checkpoints/<step>/pretrained_model")
    ap.add_argument("--socket", default=str(WS / "logs/run/act_policy.sock"))
    ap.add_argument("--temporal-ensemble", type=float, default=None,
                    help="ACT temporal ensembling coefficient (e.g. 0.01; forces n_action_steps=1, infers every frame)")
    ap.add_argument("--n-action-steps", type=int, default=None, help="actions executed per inferred chunk")
    ap.add_argument("--n-action-steps-still", type=int, default=None,
                    help="actions executed from a chunk without walking (default: same as --n-action-steps)")
    ap.add_argument("--walk-thr", type=float, default=0.05, help="loco command magnitude counted as walking")
    ap.add_argument("--jpeg", type=int, default=95)
    ap.add_argument("--device", default="cuda")
    a = ap.parse_args()

    import torch
    from lerobot.common.control_utils import predict_action
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    ckpt = str(a.checkpoint.resolve())
    cfg = PreTrainedConfig.from_pretrained(ckpt)
    if a.temporal_ensemble is not None:
        cfg.temporal_ensemble_coeff, cfg.n_action_steps = float(a.temporal_ensemble), 1
    elif a.n_action_steps is not None:
        cfg.n_action_steps = int(a.n_action_steps)
    n_move = int(cfg.n_action_steps)
    n_still = int(a.n_action_steps_still) if a.n_action_steps_still and a.temporal_ensemble is None else None
    if n_still is not None:
        assert 1 <= n_still <= cfg.chunk_size, f"--n-action-steps-still must be in [1, {cfg.chunk_size}]"
        cfg.n_action_steps = max(n_move, n_still)       # select_action slices the chunk to this; we cut it shorter
    cfg.device = a.device
    policy = ACTPolicy.from_pretrained(ckpt, config=cfg)
    policy.to(a.device).eval()
    pre, post = make_pre_post_processors(cfg, pretrained_path=ckpt,
                                         preprocessor_overrides={"device_processor": {"device": a.device}})
    device = torch.device(a.device)
    chunk_log = {"move": 0, "still": 0}
    if n_still is not None:
        from safetensors.torch import load_file
        st = load_file(str(Path(ckpt) / "policy_postprocessor_step_0_unnormalizer_processor.safetensors"))
        loco_mean = st["action.mean"][26:29].to(device)
        loco_std = st["action.std"][26:29].to(device)
        predict_chunk = policy.predict_action_chunk

        def predict_chunk_adaptive(batch):
            acts = predict_chunk(batch)                       # (B, chunk, 29), normalised
            loco = acts[0, :, 26:29] * loco_std + loco_mean    # same MEAN_STD inverse as the postprocessor
            walking = bool((loco.abs() > a.walk_thr).any())
            chunk_log["move" if walking else "still"] += 1
            return acts[:, : (n_move if walking else n_still)]

        policy.predict_action_chunk = predict_chunk_adaptive
    img_keys = [k for k in cfg.input_features if k.startswith("observation.images.")]
    info = {"checkpoint": ckpt, "input_features": {k: list(v.shape) for k, v in cfg.input_features.items()},
            "output_features": {k: list(v.shape) for k, v in cfg.output_features.items()},
            "chunk_size": cfg.chunk_size, "n_action_steps": n_move, "n_action_steps_still": n_still,
            "walk_thr": a.walk_thr if n_still is not None else None,
            "temporal_ensemble_coeff": cfg.temporal_ensemble_coeff, "jpeg": a.jpeg,
            "gpu": torch.cuda.get_device_name(0) if device.type == "cuda" else "cpu"}
    print(f"[act_server] {info}", flush=True)

    def jpeg_roundtrip(img: np.ndarray) -> np.ndarray:
        if not a.jpeg:
            return img
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="JPEG", quality=int(a.jpeg))
        buf.seek(0)
        with Image.open(buf) as im:
            return np.asarray(im.convert("RGB"))

    sock_path = Path(a.socket)
    sock_path.parent.mkdir(parents=True, exist_ok=True)
    sock_path.unlink(missing_ok=True)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(str(sock_path))
    os.chmod(sock_path, 0o600)
    srv.listen(1)
    print(f"[act_server] listening on {sock_path}", flush=True)
    try:
        while True:
            c, _ = srv.accept()
            print("[act_server] client connected", flush=True)
            n = 0
            try:
                while True:
                    meta, arr = framing.recv_msg(c)
                    cmd = meta.get("cmd")
                    if cmd == "reset":
                        if n_still is not None and any(chunk_log.values()):
                            print(f"[act_server] chunks last episode: walking {chunk_log['move']} "
                                  f"(x{n_move}), still {chunk_log['still']} (x{n_still})", flush=True)
                        chunk_log.update(move=0, still=0)
                        policy.reset()
                        framing.send_msg(c, {"ok": True})
                    elif cmd == "info":
                        framing.send_msg(c, {"ok": True, **info})
                    elif cmd == "act":
                        missing = [k for k in ["observation.state", *img_keys] if k not in arr]
                        if missing:
                            framing.send_msg(c, {"ok": False, "error": f"missing {missing}"})
                            continue
                        obs = {"observation.state": np.asarray(arr["observation.state"], np.float32).copy()}
                        for k in img_keys:
                            obs[k] = jpeg_roundtrip(np.asarray(arr[k], np.uint8)).copy()
                        t0 = time.perf_counter()
                        act = predict_action(obs, policy, device, pre, post, use_amp=False)
                        if device.type == "cuda":
                            torch.cuda.synchronize()
                        dt = (time.perf_counter() - t0) * 1000.0
                        act = act.squeeze(0).float().cpu().numpy().astype(np.float32)
                        framing.send_msg(c, {"ok": True, "infer_ms": dt}, {"action": act})
                        n += 1
                    else:
                        framing.send_msg(c, {"ok": False, "error": f"unknown cmd {cmd!r}"})
            except ConnectionError:
                print(f"[act_server] client left after {n} actions", flush=True)
            finally:
                c.close()
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        sock_path.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
