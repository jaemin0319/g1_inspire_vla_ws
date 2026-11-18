"""Config loading shared by every env (needs only PyYAML)."""
from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Iterable

import yaml

WS = Path(os.environ.get("G1IV_WS") or Path(__file__).resolve().parents[2])


def ws_path(p: str | os.PathLike) -> Path:
    p = Path(p)
    return p if p.is_absolute() else WS / p


def load_yaml(p: str | os.PathLike) -> dict:
    with open(ws_path(p), "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def deep_update(base: dict, upd: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (upd or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_update(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def apply_overrides(cfg: dict, overrides: Iterable[str] | None) -> dict:
    """Apply CLI overrides of the form a.b.c=<yaml value>."""
    cfg = copy.deepcopy(cfg)
    for ov in overrides or []:
        if "=" not in ov:
            raise ValueError(f"override must be key=value, got {ov!r}")
        key, val = ov.split("=", 1)
        node = cfg
        parts = key.split(".")
        for k in parts[:-1]:
            node = node.setdefault(k, {})
        node[parts[-1]] = yaml.safe_load(val)
    return cfg


def load_config(path: str | os.PathLike, overrides: Iterable[str] | None = None) -> dict:
    return apply_overrides(load_yaml(path), overrides)


def get(cfg: dict, dotted: str, default: Any = None) -> Any:
    node: Any = cfg
    for k in dotted.split("."):
        if not isinstance(node, dict) or k not in node:
            return default
        node = node[k]
    return node
