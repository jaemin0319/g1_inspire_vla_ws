# ACT 보행·파지 실험 스크립트 (사본, 2026-11-25 갱신)

`docs/ACT_EVAL.md`와 `docs/AUTO_WALK_GRASP.md` §8–9에 나오는 `/home/jammin/g1iv_store/` 스크립트의 사본입니다(처음 2026-11-23, 갱신 2026-11-25).
실제로 실행되는 원본은 `/home/jammin/g1iv_store/`에 있습니다. `/mnt/isaac` 공간이 부족해서 원본 데이터, 변환본, 학습 출력을 그쪽에 둡니다.
`eval_summary.py`와 `compensation.py`는 이 폴더에서 직접 고친 것으로, 원본보다 새 버전입니다.

각 스크립트 맨 위에 이 PC 경로가 고정돼 있습니다(`WS`, `ST`, so101 `.venv-train`, 학습 GPU UUID). 다른 PC에서 쓰려면 그 부분을 고쳐야 합니다.

**데이터 생성 → 학습 파이프라인**

| 파일 | 내용 |
|---|---|
| `v4_pipeline.sh` | v4(손↔몸 충돌·보행 팔 자세 켬, v3와 같은 시드·범위): 수집 → 휘청 필터 → 변환 → 허깅페이스 업로드(백그라운드) + 학습 200k·2만마다 저장. 학습·시뮬·업로드는 각각 systemd scope(메모리 상한) |
| `v3_pipeline.sh` | v3 넓은 무작위 출발: 수집 → 필터 → LeRobot v2.1 → ACT v3.0 → 학습 → 평가 → 요약. `collect\|convert\|train\|eval\|summary\|all`, 끝난 단계는 건너뜀 (30회 평가 단계는 11-23에 `v3_eval_ckpts.sh`로 바뀜) |
| `v3_hf_upload.sh`, `v4_hf_upload.sh` | 수집이 끝난 데이터셋을 허깅페이스에 공개 업로드하고 해시 대조(카드 `docs/hf_cards/*.md.in`) |
| `aug_tfs.json` | ACT 학습 이미지 증강(`--dataset.image_transforms.tfs`) |

**체크포인트 평가·정리**

| 파일 | 내용 |
|---|---|
| `v3_eval_ckpts.sh`, `v4_eval_ckpts.sh` | 학습과 동시에 4060에서 체크포인트마다 18회 평가(seed 20261124) + 3×3 헤드 영상 |
| `v3_link_videos.sh`, `v4_link_videos.sh` | 평가 영상·성공률 표를 `~/다운로드`에 하드링크 |
| `eval_v3_summary.py`, `eval_grid18.py` | 평가 집계(CSV/JSON/PNG, Wilson 95% 구간)와 18회 3×3 영상. `G1IV_EVAL_VER`로 v3/v4 선택 |
| `eval_v3_failure_causes.py`, `eval_v3_cause_relations.py` | 실패 원인 다중 라벨(성공 시연 범위 기준)과 원인 사이 관계(자카드·lift·피셔 p·군집) |
| `eval_ckpts.sh`, `eval_ckpts2.sh`, `eval_ckpts3.sh` | v2 체크포인트 순차 평가 대기열(동시에 하나만 실행) |
| `eval_summary.py`, `fail_types.py`, `hand_close.py`, `compensation.py` | 평가 폴더 하나의 성공·정지 오차·실패 유형·손 오므림 순간·팔 보정 비율 |
| `overfit_check.sh`, `overfit_grid.py`, `overfit_video.py` | v2 30k 과적합 검사(원통 y −0.45~+0.45, 10회)와 3×3 헤드 영상 |

**시연·시험·모델 업로드**

| 파일 | 내용 |
|---|---|
| `demo_act.sh`, `demo_panel.sh` | GUI 정책 시연 `start\|reset\|stop\|panel\|status`(시연 중에는 평가 루프를 SIGSTOP) + zenity 버튼 창 |
| `front_v4_100k.sh` | v4 100k 정면 카메라 영상(평가 시작점 3개) |
| `handcol_test.sh` | 손↔몸 충돌과 보행 팔 자세 시험 |
| `hf_push_models.sh`, `hf_push_v3.sh`, `hf_model_cards/` | 학습한 ACT 모델의 허깅페이스 비공개 업로드와 모델 카드 |
