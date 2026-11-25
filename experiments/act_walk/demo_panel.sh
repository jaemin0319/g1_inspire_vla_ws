#!/usr/bin/env bash
# Button window for demo_act.sh: 리셋 = new random start now, 정지 = close the demo and resume the evaluation loop,
# 창 닫기 = hide this window only (the demo keeps running; reopen with demo_act.sh panel).
D=/home/jammin/g1iv_store/demo_act.sh
while true; do
  last=$(grep -E "episode [0-9]+:" /home/jammin/g1iv_store/logs/demo_runner.log 2>/dev/null | tail -1 | sed -E 's/.*episode [0-9]+: ([a-z]+).*/직전 시연: \1/')
  b=$(zenity --info --title="데모 조작" --width=360 \
        --text="v3 ACT 데모 ($(. /home/jammin/g1iv_store/run/demo.env 2>/dev/null; echo "${STEP:-?}" | sed 's/000$/k/;s/^0*//'))\n${last}\n\n리셋: 지금 시연을 끝내고 새 출발 위치로\n정지: 데모를 끄고 체크포인트 평가 재개" \
        --ok-label="창 닫기" --extra-button="리셋" --extra-button="정지" 2>/dev/null)
  case "$b" in
    리셋) "$D" reset > /dev/null 2>&1 ;;
    정지) "$D" stop > /dev/null 2>&1; exit 0 ;;
    *) exit 0 ;;
  esac
done
