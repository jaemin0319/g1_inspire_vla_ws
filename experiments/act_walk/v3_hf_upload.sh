#!/usr/bin/env bash
# 2026-11-23 user request "수집 끝나면 되는대로 허깅페이스에 업로드": v3 dataset (LeRobot v2.1) -> public Hub dataset,
# same procedure as the v2 upload (2026-11-21): waits for v3_pipeline.sh to finish the v2.1 conversion, stages a
# hard-linked copy whose provenance raw_dir paths are relative (the push tool refuses absolute local paths), validates
# it (LeRobot v2.1 loader + GR00T loader), writes the card with the measured numbers, pushes, verifies hashes and
# anonymous access. Training (v3_pipeline.sh) continues in parallel; this only reads the dataset.
set -uo pipefail
WS=/mnt/isaac/g1_inspire_vla_ws
ST=/home/jammin/g1iv_store
NAME=walk_wide_grasp_desk065_scripted_v3
SRC=$ST/processed/${NAME}_lerobot_v21
DST=$ST/hf_upload/${NAME}_lerobot_v21
RID=jammin0319/g1_inspire_${NAME}
CARD=$WS/docs/hf_cards/$NAME.md
VLOG=$WS/logs/validate
cd "$WS"
source scripts/_env_common.sh
log() { echo "=== $(date '+%F %T') $*"; }

log "waiting for the v2.1 conversion ($SRC)"
until grep -q "make_act_dataset ->" "$ST/logs/v3_pipeline.log" 2>/dev/null && [[ -f $SRC/meta/info.json ]]; do
  grep -qE "convert failed|giving up" "$ST/logs/v3_pipeline.log" 2>/dev/null && { log "pipeline reported a failure"; exit 1; }
  sleep 60
done
log "conversion done"

if [[ ! -d $DST ]]; then
  mkdir -p "$DST/meta"
  cp -al "$SRC/data" "$SRC/videos" "$DST/"
  cp -p "$SRC"/meta/* "$DST/meta/"
  python3 - "$DST/meta/g1iv_provenance.json" <<'EOF'
import json, sys
p = sys.argv[1]; d = json.load(open(p))
pre = "/home/jammin/g1iv_store/"
n = 0
for e in d["episodes"]:
    if e.get("raw_dir", "").startswith(pre):
        e["raw_dir"] = e["raw_dir"][len(pre):]; n += 1
d["raw_dir_note"] = "raw_dir is relative to the uploader's local data store (raw recordings are not uploaded)"
json.dump(d, open(p, "w"), indent=1)
print("rewrote", n, "raw_dir entries")
EOF
fi
left=$(grep -o "/\(home\|mnt\|tmp\)/[^\"' ]*" "$DST"/meta/* | wc -l)
log "staged $DST ($(du -sh "$DST" | cut -f1), $(find "$DST" -type f | wc -l) files, absolute paths left: $left)"
(( left == 0 )) || { log "absolute paths remain in meta/; not uploading"; exit 1; }

mkdir -p "$VLOG"
export PYTHONPATH=$WS/src CUDA_VISIBLE_DEVICES=""
timeout 1800 "$(g1iv_py g1_inspire_data)" -m g1iv_data.check_loader "$SRC" > "$VLOG/loader_$NAME.txt" 2>&1
rc=$?; log "LeRobot loader check exit $rc"; tail -4 "$VLOG/loader_$NAME.txt" | cut -c1-200
(( rc == 0 )) || exit 1
timeout 1800 "$(g1iv_py g1_inspire_vla)" -m g1iv_data.check_gr00t "$DST" --report "$VLOG/gr00t_$NAME.json" > "$VLOG/gr00t_$NAME.txt" 2>&1
rc=$?; log "GR00T loader check exit $rc, OK lines $(grep -c '^OK' "$VLOG/gr00t_$NAME.txt"), other lines $(grep -vc '^OK' "$VLOG/gr00t_$NAME.txt")"
(( rc == 0 )) || exit 1

# card: fill the measured numbers
python3 - "$DST" "$ST/raw/walk_wide_v3_sel/report.json" "$CARD.in" "$CARD" <<'EOF'
import json, sys, glob
from pathlib import Path
import statistics as S
dst, rep, tin, tout = Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4]
eps = [json.loads(l) for l in open(dst / "meta/episodes.jsonl")]
L = [e["length"] for e in eps]
prov = json.load(open(dst / "meta/g1iv_provenance.json"))["episodes"]
r = json.load(open(rep))["episodes"]
kept = [e for e in r if not e["excluded"]]
info = json.load(open(dst / "meta/info.json"))
codecs = {v.get("info", {}).get("video.codec") for k, v in info["features"].items() if v.get("dtype") == "video"}
import numpy as np
sy, oy, lat, fwd = [], [], [], []
for e in kept:
    z = np.load(Path(e["path"]) / "arrays.npz")
    m = json.load(open(Path(e["path"]) / "meta.json"))
    bp = z["obs/base_pos_w"]; ox, oyy = m["objects_xy"]["red"]
    sy.append(bp[0, 1]); oy.append(oyy)
    stand = (ox - 0.31, oyy + 0.10)
    lat.append(abs(stand[1] - bp[0, 1])); fwd.append(stand[0] - bp[0, 0])
walk = [e["walk_frames"] for e in kept]
n_walk1 = sum(e["n_walk_segments"] == 1 for e in kept)
vals = dict(n_eps=len(eps), n_frames=sum(L), len_min=min(L), len_max=max(L), sy_min=min(sy), sy_max=max(sy),
            oy_min=min(oy), oy_max=max(oy), lat_min=min(lat), lat_max=max(lat), fwd_min=min(fwd), fwd_max=max(fwd),
            walk_med=int(S.median(walk)), walk_med_s=S.median(walk) / 25.0, n_rec=len(r), n_walk1=n_walk1)
assert len(kept) == len(eps), (len(kept), len(eps))
txt = Path(tin).read_text().format(**vals)
if codecs != {"av1"}:
    txt = txt.replace("(640×480, AV1)", f"(640×480, {'/'.join(sorted(map(str, codecs)))})")
Path(tout).write_text(txt)
print("card:", vals, "codecs", codecs)
EOF
[[ -f $CARD ]] || { log "card not written"; exit 1; }

export HF_HUB_DISABLE_PROGRESS_BARS=1
timeout 5400 "$(g1iv_py g1_inspire_data)" tools/push_dataset_hf.py "$DST" --repo-id "$RID" --card-file "$CARD" --public \
  --tag unitree-g1 --tag inspire-hand --tag isaac-sim --tag simulation --tag humanoid --tag loco-manipulation \
  > "$VLOG/hf_push_$NAME.log" 2>&1
rc=$?; log "push exit $rc"; tail -5 "$VLOG/hf_push_$NAME.log" | cut -c1-200
(( rc == 0 )) || exit 1

"$(g1iv_py g1_inspire_data)" - "$RID" "$DST" <<'EOF' 2>&1 | tee "$VLOG/hf_verify_$NAME.txt"
import hashlib, json, sys, urllib.request
from pathlib import Path
from huggingface_hub import HfApi, hf_hub_download
rid, root = sys.argv[1], Path(sys.argv[2])
api = HfApi()
info = api.dataset_info(rid, files_metadata=True)
hub = {s.rfilename: s for s in info.siblings}
local = {str(p.relative_to(root)): p for p in root.rglob("*") if p.is_file()}
print("private:", info.private, "hub files:", len(hub), "local files:", len(local))
print("hub-only:", sorted(set(hub) - set(local)))
print("local-only:", sorted(set(local) - set(hub)))
bad = 0
for k, p in local.items():
    s = hub.get(k)
    if s is None:
        bad += 1; continue
    if s.lfs:
        if hashlib.sha256(p.read_bytes()).hexdigest() != s.lfs.sha256:
            bad += 1; print("LFS mismatch", k)
    elif Path(hf_hub_download(rid, k, repo_type="dataset", revision="main")).read_bytes() != p.read_bytes():
        bad += 1; print("mismatch", k)
print("mismatches:", bad)
print("tags:", [t.name for t in api.list_repo_refs(rid, repo_type="dataset").tags])
u = f"https://huggingface.co/datasets/{rid}/resolve/main/meta/info.json"
d = json.loads(urllib.request.urlopen(urllib.request.Request(u, headers={"User-Agent": "dataset-check"}), timeout=60).read())
print("anonymous info.json: episodes", d["total_episodes"], "frames", d["total_frames"], "version", d["codebase_version"])
EOF
log "done"
