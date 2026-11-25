#!/usr/bin/env bash
# v4 copy (2026-11-24, same as v3). 2026-11-23 user request "모델마다 테스트할 때마다 ... 전부 다운로드 경로에 저장": every per-checkpoint video that
# v4_eval_ckpts.sh writes to ~/다운로드/g1_act_v4_eval/ is also hard-linked into ~/다운로드 itself (same file, no extra
# space), plus the success-rate table/chart (updated in place by the eval loop, so the links stay current).
# Stops after the 200k video, or when the eval loop reports "all checkpoints done".
set -uo pipefail
SRC=/home/jammin/다운로드/g1_act_v4_eval
DST=/home/jammin/다운로드
LOG=/home/jammin/g1iv_store/logs/v4_eval_ckpts.log
link() { [[ -f $1 && ! -e $2 ]] && ln "$1" "$2" && echo "$(date '+%F %T') linked $(basename "$2")"; }
while :; do
  for f in "$SRC"/g1_act_v4_*k_eval18.mp4; do [[ -f $f ]] && link "$f" "$DST/$(basename "$f")"; done
  link "$SRC/success_rate_by_checkpoint.png" "$DST/g1_act_v4_success_rate_by_checkpoint.png"
  link "$SRC/success_rate_by_checkpoint.csv" "$DST/g1_act_v4_success_rate_by_checkpoint.csv"
  [[ -e $DST/g1_act_v4_200k_eval18.mp4 ]] && break
  grep -q "all checkpoints done" "$LOG" 2>/dev/null && { sleep 120; continue_once=${continue_once:-1}; ((continue_once--)) || break; }
  sleep 60
done
echo "$(date '+%F %T') done"
