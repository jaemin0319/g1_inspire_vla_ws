"""Upload a converted LeRobot v2.1 dataset (data/processed/<name>_lerobot_v21) to the Hugging Face Hub.
Run only on an explicit user request (project rule: nothing is published automatically). Private by default.

    .conda/envs/g1_inspire_data/bin/python tools/push_dataset_hf.py data/processed/<name>_lerobot_v21 \
        --repo-id <hf_user>/<name> --card-file <description.md> [--public]

Uses lerobot's own LeRobotDataset.push_to_hub (uploads meta/, data/, videos/; skips images/; tags the revision
"v2.1" so LeRobotDataset(repo_id) can load it), then (re)writes the dataset card with the given description.
Refuses to upload when meta/ contains absolute local paths (/home/, /mnt/, /tmp/).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path)
    ap.add_argument("--repo-id", required=True)
    ap.add_argument("--card-file", type=Path, required=True, help="markdown for the card's description section")
    ap.add_argument("--public", action="store_true", help="default: private repository")
    ap.add_argument("--license", default="apache-2.0")
    ap.add_argument("--tag", action="append", default=[], help="extra card tag (repeatable)")
    a = ap.parse_args()

    from huggingface_hub import HfApi
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.utils import create_lerobot_dataset_card

    root = a.root.resolve()
    leaks = [f"{p.name}: {m}" for p in sorted((root / "meta").glob("*")) if p.is_file()
             for m in sorted(set(re.findall(r"/(?:home|mnt|tmp)/[^\"'\s]*", p.read_text(errors="ignore"))))]
    if leaks:
        print("refusing to upload, absolute local paths in meta/:\n  " + "\n  ".join(leaks[:20]), file=sys.stderr)
        return 2
    desc = a.card_file.read_text()
    ds = LeRobotDataset(repo_id=a.repo_id, root=root)          # loads the local copy (validates meta + parquet)
    print(f"[push] {root.name}: {ds.meta.total_episodes} episodes, {ds.meta.total_frames} frames -> "
          f"{a.repo_id} ({'public' if a.public else 'private'})", flush=True)
    tags = ["unitree-g1", "inspire-hand", "isaac-sim", "simulation", "humanoid", "loco-manipulation", *a.tag]
    ds.push_to_hub(tags=tags, license=a.license, private=not a.public, dataset_description=desc)
    card = create_lerobot_dataset_card(tags=tags, dataset_info=ds.meta.info, license=a.license,
                                       dataset_description=desc)
    card.push_to_hub(repo_id=a.repo_id, repo_type="dataset")    # also refreshes the card on a re-upload
    api = HfApi()
    info = api.dataset_info(a.repo_id)
    refs = api.list_repo_refs(a.repo_id, repo_type="dataset")
    files = api.list_repo_files(a.repo_id, repo_type="dataset")
    print(f"[push] done: https://huggingface.co/datasets/{a.repo_id}  private={info.private} "
          f"files={len(files)} tags={[t.name for t in refs.tags]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
