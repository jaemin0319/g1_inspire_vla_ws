"""Download the official Unitree simulator assets pinned in envs/assets.lock.json (Hugging Face dataset revision +
sha256), unpack them into repos/unitree_sim_isaaclab/assets like the official fetch_assets.sh, and verify key files.

    python3 scripts/setup/fetch_assets.py [--keep-zip]

System python3 (stdlib only). Resumes a partial download, re-uses a verified zip in .cache/assets_dl/, never touches
an existing assets/ folder whose key files already match (a mismatching one stops the script).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path

WS = Path(__file__).resolve().parents[2]


def sha256(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def verify(root: Path, lock: dict) -> list[str]:
    bad = []
    for rel, want in lock["verify_sha256"].items():
        f = root / rel
        if not f.is_file() or sha256(f) != want:
            bad.append(rel)
    return bad


def download(url: str, out: Path, size: int) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    have = out.stat().st_size if out.exists() else 0
    if have == size:
        return
    if have > size:
        out.unlink(); have = 0
    req = urllib.request.Request(url, headers={"User-Agent": "g1iv-bootstrap", **({"Range": f"bytes={have}-"} if have else {})})
    with urllib.request.urlopen(req, timeout=60) as r, out.open("ab" if have else "wb") as f:
        if have and r.status != 206:                     # server ignored the range: start over
            f.seek(0); f.truncate(); have = 0
        done, last = have, 0
        while chunk := r.read(1 << 22):
            f.write(chunk); done += len(chunk)
            if done - last >= 100 << 20 or done == size:
                print(f"[assets] {done / 1e9:.2f} / {size / 1e9:.2f} GB", flush=True); last = done


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--keep-zip", action="store_true", help="keep .cache/assets_dl/*.zip after unpacking (1.3 GB)")
    a = ap.parse_args()
    lock = json.loads((WS / "envs" / "assets.lock.json").read_text())
    root = WS / lock["extract_into"]
    if not (root / ".git").exists():
        raise SystemExit(f"{root} missing: run scripts/setup/fetch_repos.py first")
    if (root / "assets").exists():
        bad = verify(root, lock)
        if not bad:
            print(f"[assets] {root / 'assets'} present and verified"); return 0
        raise SystemExit(f"{root / 'assets'} exists but does not match the lock ({bad}); move it away first")
    z = WS / ".cache" / "assets_dl" / f"assets_{lock['revision'][:10]}.zip"
    if not (z.exists() and z.stat().st_size == lock["size"] and sha256(z) == lock["sha256"]):
        url = f"https://huggingface.co/datasets/{lock['repo']}/resolve/{lock['revision']}/{lock['file']}"
        print(f"[assets] downloading {url} ({lock['size'] / 1e9:.2f} GB)", flush=True)
        download(url, z, lock["size"])
        got = sha256(z)
        if got != lock["sha256"]:
            z.unlink()
            raise SystemExit(f"[assets] sha256 mismatch {got} != {lock['sha256']} (download removed, run again)")
    print(f"[assets] unpacking into {root}", flush=True)
    tmp = root / ".assets_unpack"
    shutil.rmtree(tmp, ignore_errors=True)
    with zipfile.ZipFile(z) as zf:
        zf.extractall(tmp)
    if not (tmp / "assets").is_dir():
        raise SystemExit("[assets] zip has no top-level assets/ folder")
    (tmp / "assets").rename(root / "assets")
    shutil.rmtree(tmp)
    n = sum(1 for p in (root / "assets").rglob("*") if p.is_file())
    bad = verify(root, lock)
    if bad or n != lock["expect_files"]:
        raise SystemExit(f"[assets] verification failed: files {n} (want {lock['expect_files']}), bad {bad}")
    (WS / ".cache" / "assets_dl" / "REVISION").write_text(lock["revision"] + "\n")
    if not a.keep_zip:
        z.unlink()
    print(f"[assets] ok: {n} files, key files verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
