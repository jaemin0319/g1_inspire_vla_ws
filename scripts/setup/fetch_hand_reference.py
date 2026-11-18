"""Fetch the Inspire RH56E2 / Unitree FTP hand references listed in envs/hand_reference.lock.json (file-by-file from
the pinned GitHub commits, verified by git blob hash) into data/reference/inspire_ftp/ for tools/cad/make_inspire_e2_hand.py.

    python3 scripts/setup/fetch_hand_reference.py

These files are third-party (Inspire-Robots URDF/STL as published in two community repositories, Unitree unitree_ros,
see docs/HAND_RH56E2.md section 2); they are downloaded on each machine and never committed to this repository.
"""
from __future__ import annotations

import hashlib
import json
import sys
import urllib.parse
import urllib.request
from collections import Counter
from pathlib import Path

WS = Path(__file__).resolve().parents[2]
OUT = WS / "data" / "reference" / "inspire_ftp"


def git_blob(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def main() -> int:
    lock = json.loads((WS / "envs" / "hand_reference.lock.json").read_text())
    got = skipped = 0
    for e in lock["files"]:
        dst = OUT / e["local"]
        if dst.is_file() and git_blob(dst.read_bytes()) == e["git_blob"]:
            skipped += 1
            continue
        url = f"https://raw.githubusercontent.com/{e['repo']}/{e['commit']}/{urllib.parse.quote(e['path'])}"
        with urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "g1iv-bootstrap"}), timeout=60) as r:
            data = r.read()
        if git_blob(data) != e["git_blob"]:
            raise SystemExit(f"[hand-ref] hash mismatch for {e['repo']}@{e['commit'][:7]}:{e['path']}")
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(data)
        got += 1
    by_src = Counter((e["repo"], e["commit"]) for e in lock["files"])
    (OUT / "rh56e2").mkdir(parents=True, exist_ok=True)
    (OUT / "rh56e2" / "SOURCE.txt").write_text(
        "hychen-naza/isaaclab_env cccdf11f2f481b05f20d96123605c9a4e9c09f37 RH56E2_R_2026_1_5\n"
        "hoon001218/contact_rich_manipulation_task_learning cdc4300c9210b8e5422bd07334afbf729f96c0bd "
        "assets/robots/Roas_provided_inspire_urdf/RH56E2 URDF/RH56E2_L_2025_9_10\n"
        "hoon001218/contact_rich_manipulation_task_learning cdc4300c9210b8e5422bd07334afbf729f96c0bd "
        "assets/robots/Roas_provided_inspire_urdf/RH56E2 URDF/RH56E2_R_2025_9_11\n")
    (OUT / "unitree_ros").mkdir(parents=True, exist_ok=True)
    (OUT / "unitree_ros" / "SOURCE.txt").write_text("unitree_ros master 725b59b6de39a26c425632120c89f23c49a36f1e\n")
    print(f"[hand-ref] {got} downloaded, {skipped} already present, all {len(lock['files'])} verified "
          f"({', '.join(f'{r}@{c[:7]}: {n}' for (r, c), n in by_src.items())})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
