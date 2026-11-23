# ACT 보행·파지 실험 스크립트 (사본, 2026-11-23)

`docs/ACT_EVAL.md`와 `docs/AUTO_WALK_GRASP.md` §8–9에 나오는 `/home/jammin/g1iv_store/` 스크립트를 2026-11-23에 복사해 둔 것입니다.
실제로 실행되는 원본은 `/home/jammin/g1iv_store/`에 있습니다. `/mnt/isaac` 공간이 부족해서 원본 데이터, 변환본, 학습 출력을 그쪽에 둡니다.

각 스크립트 맨 위에 이 PC 경로가 고정돼 있습니다(`WS`, `ST`, so101 `.venv-train`, 학습 GPU UUID). 다른 PC에서 쓰려면 그 부분을 고쳐야 합니다.

| 파일 | 내용 |
|---|---|
| `v3_pipeline.sh` | v3 전체 자동화: 넓은 무작위 출발 수집 → 휘청 필터 → LeRobot v2.1 → ACT v3.0 → 학습 → 체크포인트별 30회 평가 → 요약. `collect\|convert\|train\|eval\|summary\|all`, 끝난 단계는 건너뜀 |
| `eval_v3_summary.py` | v3 평가 정리(`checkpoints.csv`, `episodes.csv`, `summary.json`, `success_rate.png`, Wilson 95% 구간) |
| `eval_ckpts.sh`, `eval_ckpts2.sh`, `eval_ckpts3.sh` | v2 체크포인트 순차 평가 대기열(동시에 하나만 실행) |
| `eval_summary.py`, `fail_types.py`, `hand_close.py` | 평가 폴더 하나의 성공·실패 유형·손 오므림 순간 정리 |
| `overfit_check.sh`, `overfit_grid.py`, `overfit_video.py` | v2 30k 과적합 검사(원통 y −0.45~+0.45, 10회)와 3×3 헤드 영상 |
| `aug_tfs.json` | ACT 학습 이미지 증강(`--dataset.image_transforms.tfs`) |
| `hf_push_models.sh`, `hf_model_cards/` | 학습한 ACT 모델의 허깅페이스 비공개 업로드와 모델 카드 |
