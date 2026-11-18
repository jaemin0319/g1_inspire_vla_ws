"""Clone the external repositories at the commits pinned in envs/repos.lock.json, initialise the submodules that
were checked out on the reference PC (lock lines without "-"), and apply the project patches envs/patches/<repo>_g1iv.patch.

    python3 scripts/setup/fetch_repos.py [--only NAME ...] [--skip NAME ...]

System python3 (stdlib only) + git. Idempotent and never destructive: an existing repos/<name> at the pinned commit is
kept (the patch is applied only if not yet applied); an existing checkout at ANOTHER commit or with foreign changes
stops the script instead of being reset.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[2]


def git(*a, cwd=None, check=True, capture=False):
    r = subprocess.run(["git", *a], cwd=cwd, check=False, text=True,
                       capture_output=capture)
    if check and r.returncode != 0:
        raise SystemExit(f"git {' '.join(a)} failed in {cwd or WS}:\n{(r.stderr or '')[-2000:]}")
    return (r.stdout or "").strip() if capture else r.returncode


def submodules(entry: dict) -> list[tuple[str, str]]:
    """(path, commit) of the submodules that were initialised on the reference PC."""
    out = []
    for ln in entry.get("submodules", []):
        if not ln or ln[0] == "-":           # "-<sha> path" = not initialised there either
            continue
        sha, path = ln[1:].split()[:2] if ln[0] in " +U" else ln.split()[:2]
        out.append((path, sha))
    return out


def fetch(name: str, entry: dict) -> None:
    dst = WS / "repos" / name
    patch = WS / "envs" / "patches" / f"{name}_g1iv.patch"
    if (dst / ".git").exists():
        head = git("rev-parse", "HEAD", cwd=dst, capture=True)
        if head != entry["commit"]:
            raise SystemExit(f"{dst} is at {head[:12]}, lock wants {entry['commit'][:12]}; "
                             "move it away or check out the pinned commit yourself (nothing was changed)")
        print(f"[repos] {name}: present at {head[:12]}")
    else:
        dst.parent.mkdir(parents=True, exist_ok=True)
        print(f"[repos] {name}: cloning {entry['remote']} @ {entry['commit'][:12]}", flush=True)
        git("clone", "--filter=blob:none", "--no-checkout", entry["remote"], str(dst))
        git("checkout", "-q", "-B", f"g1iv/pinned-{entry['commit'][:7]}", entry["commit"], cwd=dst)
    subs = submodules(entry)
    listed = {p for p, _ in subs}
    for path, sha in sorted(subs, key=lambda s: s[0].count("/")):   # parents before nested submodules
        parent = max((p for p in listed if path.startswith(p + "/")), key=len, default="")
        rel = path[len(parent) + 1:] if parent else path
        git("submodule", "update", "--init", "--filter=blob:none", "--", rel, cwd=dst / parent if parent else dst)
        got = git("rev-parse", "HEAD", cwd=dst / path, capture=True)
        if got != sha:
            raise SystemExit(f"{name}/{path}: submodule at {got[:12]}, lock says {sha[:12]}")
        print(f"[repos] {name}/{path}: {sha[:12]}")
    if patch.exists() and patch.stat().st_size > 0:
        already = subprocess.run(["git", "apply", "--reverse", "--check", str(patch)], cwd=dst,
                                 capture_output=True).returncode == 0          # quiet: "does not apply" is expected
        if already:
            print(f"[repos] {name}: project patch already applied")
        else:
            if git("apply", "--check", str(patch), cwd=dst, check=False) != 0:
                raise SystemExit(f"{patch.name} does not apply to {dst} (local changes?); nothing was changed")
            git("apply", str(patch), cwd=dst)
            print(f"[repos] {name}: applied {patch.relative_to(WS)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", default=[])
    ap.add_argument("--skip", nargs="*", default=[])
    a = ap.parse_args()
    lock = json.loads((WS / "envs" / "repos.lock.json").read_text())
    names = [n for n in lock if not n.startswith("_")]
    for n in a.only + a.skip:
        if n not in names:
            raise SystemExit(f"unknown repo {n!r}; have {names}")
    for n in names:
        if (a.only and n not in a.only) or n in a.skip:
            continue
        fetch(n, lock[n])
    print("[repos] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
