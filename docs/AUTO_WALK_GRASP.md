# 자동 보행 → 파지 → 들기 (sim-to-sim 데이터 자동 생성 검증)

작성 2026-11-17. 사용자 요청: "책상 높이를 15 cm 내리고, 보행하여 다가가서 파지 후 리프트까지 자동화할 수 있는지 검증. 가능하면 sim-to-sim 데이터셋을 전부 자동으로 만들려고 함."

## 1. 결론

| 질문 | 답 | 근거 |
|---|---|---|
| 책상 15 cm 낮추기 | **적용함** (`configs/scene_desk.yaml` `desk.top_surface_height` 0.70 → **0.55**) | 이전 값은 `--scene-set desk.top_surface_height=0.70` |
| 보행 접근 자동화 | **된다** | 무작위 시작(±10 cm, ±20 cm, ±15°)에서 정렬 23/24, 성공 시 오차 앞뒤·좌우 ≤ 2.4 cm, 방향 ≤ 5.9° |
| 보행 → 파지 → 들기 전체 자동화 | **0.70 m·0.65 m 책상에서는 된다** | 0.70 m 9/10, 0.65 m 7/7 (들림 5.4–6.9 cm, 넘어짐 0) |
| 0.55 m 책상에서 파지 | **아직 안 된다** (0회 성공) | 팔이 닿으려면 손을 35° 이상 숙여야 하는데, 그러면 RH56E2 엄지가 원통 윗면에 얹혀 손가락이 원통을 밀어냄 (§4) |
| 데이터셋 자동 생성 | **된다** (0.70 m에서 확인) | 4대 카메라 녹화 4/4 성공 → 검증 4/4 → LeRobot v2.1 변환 → 로더 오류 0. 행동에 보행 명령 포함 |

0.62 m(손 20° 숙임)는 0/6입니다. 지금 손 프리셋으로 자동 파지가 되는 하한은 **0.62–0.65 m 사이**입니다.

## 2. 구성

```
허브 (scripts/run_mock_teleop.sh scripted_walk, --loco-source xr)
  ScriptedWalkGraspSource (src/g1iv_teleop/sources.py)  ← 시뮬 정답(물체·몸 위치)으로 계획하는 스크립트 전문가
   1) walk   : XR 스틱 값 → 공식 televuer 변환 → LocoArbiter → rt/run_command/cmd   (사람이 PICO 스틱을 미는 경로와 같음)
   2) settle : 정지 1 s 후 몸 기준 원통 위치를 stand_rel과 비교 → 허용오차 밖이면 1 s 후진 후 다시 걸어 들어감 (최대 3회)
   3) grasp  : 기존 ScriptedGraspSource (engage, raise, pregrasp, approach + 폐루프 보정, close, lift, hold)
  녹화(--record)는 걷기 시작 시점부터. 결과(success = 들림 ≥ 5 cm 이고 넘어지지 않음)로 outcome 자동 기록
```

- 계획 파일: `configs/scripted_walk_grasp_desk<cm>.yaml`. `run_mock_teleop.sh scripted_walk`가 `--desk H`(기본: `scene_desk.yaml` 값)로 고릅니다. 계획에는 `desk_top_z`가 있고, 팔을 움직이기 전에 물체의 정지 높이로 실제 책상 높이를 확인해 다르면 멈춥니다(단위 테스트만 함, 실기 미확인).
- 반복 생성: `tools/batch_walk_grasp.py -n N [--until-success K] [--record] [--desk H] [--seed S]`. 에피소드마다 로봇 시작 자세 x ±0.1 m, y ±0.2 m, yaw ±15°, 원통 x 0.98–1.08 m, y −0.30–−0.10 m를 무작위로 정합니다. 결과는 JSONL 한 줄씩.
- 결과 요약: `tools/analyze_walk_grasp.py logs/teleop_runs/<run>.json ...`
- 데이터 표기: 스크립트 전문가 데이터라 `source=scripted`, `data/smoke/`에 저장됩니다(기존 규칙 그대로). 정답 위치는 계획에만 쓰고 관측에는 넣지 않습니다.

## 3. 측정한 사실 (logs/checks/auto_walk_grasp/)

**공식 보행 정책**
- height 명령(4번째 값)은 0.30–0.80 전부 무시합니다. 골반 0.788–0.789 m 그대로입니다(`crouch_probe2/3.json`, 250 Hz 발행). 처음 측정(`crouch_probe.json`)에서 내려간 것은 50 Hz 발행이 RTF 2에서 기본값 0.8과 번갈아 들어간 부산물이었습니다.
- 전진 0.5 m/s 2 s → 0.45 m, 명령을 끊은 뒤 0.09 m 더 밀림. 0.3 m/s는 발을 떼지 않음.
- 걸으면서 회전 −0.4 rad/s → 2 s에 −23°: 걷는 중에는 작은 회전 명령이 먹힘.
- 제자리 회전 1.57 rad/s 0.5–1.2 s → 골반만 비틀었다가 원위치(왼쪽 3회), 오른쪽 0.8 s만 −10.7°.
- 전진 0.5 + 옆 −0.3 → 2 s에 (0.53, −0.25) m, 방향 유지. 그래서 정면을 유지한 채 대각으로 걷는 방식을 씀(`lateral_mode: holonomic`).
- 0.25 s짜리 보정 펄스는 발을 떼지 않고 흔들렸다 돌아옴 → 미세 보정은 "1 s 후진 후 재접근"으로 바꿈.
- 리셋 직후 로봇은 스스로 약 5 cm 앞으로 미끄러짐: `reset --robot-pose 0.64 -0.12 0` → 실제 (0.691, −0.102). 따라서 11-12 성공 때의 실제 상대 위치는 원통 기준 (0.33, −0.10)입니다(문서의 (0.38, −0.08)은 명령값). 이 값을 `stand_rel`로 씀.
- 카메라 없는 시뮬은 RTF 약 2(공식 `--step_hz` 기본 100). 시험할 때는 `-- --step_hz 50`으로 실시간에 맞춤. 같은 이유로 보행 명령 재발행을 50 → 200 Hz로 올림(`configs/teleop.yaml loco.publish_hz`).

**팔 도달 범위**(공식 G1_29_ArmIK, `reach_scan_desk055.txt`, `reach_scan_mid.txt`)
- 손이 수평(손가락 앞)이면 IK 끝점 R_ee는 골반 아래 약 0.04–0.06 m까지만 감. 0.55 m 책상의 원통 중심은 골반 아래 0.18 m.
- 0.55 m: 손을 35–40° 숙이면 도달(오차 ≤ 4.5 mm, 서는 위치 ±4 cm 여유). 0.65 m: 15–20°면 2–5 mm. 0.62 m: 20°면 5–9 mm.

## 4. 0.55 m에서 파지가 안 되는 이유와 시도한 것

- RH56E2 엄지는 손 좌표계에서 검지 위쪽(+z, R_ee 기준 약 +0.05–0.09 m)에 있습니다. 손을 θ만큼 숙이면 손 좌표계에서 원통 축이 기울어, 원통 윗면이 엄지 높이보다 아래로 내려갑니다(40°에서 윗면이 손 좌표계 z ≈ +0.036).
- 손목 카메라(`logs/captures/auto_walk_grasp/desk055_tilt35_thumb_on_top.png`)에서 엄지가 원통 윗면에 얹히고, 손가락이 오므라들며 원통을 손바닥 반대쪽으로 밀어 쓰러뜨리는 것이 보입니다. 쥐기 직전 원통 위치는 설계와 1–2 cm 안이었습니다.
- 시도(모두 0회 성공, 보행·정렬은 매번 정상):

| 시도 | 결과 |
|---|---|
| 기울기 25 / 30 / 35°, 잡는 높이 +0.03–0.04 m, 손 좌표계 오프셋 z·x 변경 (6회) | 쥘 때 원통이 넘어져 바닥으로 떨어짐 |
| 쥠 프리셋의 엄지 회전 0.20 / 0.05 / 0.0, 굽힘 0.65 / 0.30 / 0.0 (5회) | 넘어지지는 않지만 20° 기울었다가 손에서 빠짐, 들림 0 |
| 0.62 m, 20° (잡는 높이 0 / +0.02, 6회) | 들림 0 또는 넘어짐 |

## 5. 결과 표

| 책상 | 계획 | 시행 | 성공 | 비고 |
|---|---|---|---|---|
| 0.70 m | 고치기 전 (서는 위치 (0.38, −0.08), 짧은 보정 펄스) | 6 | 0 | 정렬 2/6. 이 결과로 서는 위치와 보정 방식을 고침 |
| 0.70 m | `desk070` (수평 손) | 6 (카메라 없음) | 5 | 1회는 정렬은 됐지만 쥐지 못함 |
| 0.70 m | `desk070` | 4 (카메라 4대 녹화) | 4 | 516–756 프레임, 에피소드당 90–137 s(실제 시간), RTF 0.23–0.26 |
| 0.65 m | 손 10° | 3 | 3 | |
| 0.65 m | 손 15° (`desk065`) | 4 | 4 | 런처 자동 선택 확인 1회 포함 |
| 0.65 m | `desk065` | 11 (카메라 4대 녹화, 2026-11-18) | 10 | 정렬 11/11. 실패 1회: 들림 4.8 cm(기준 5 cm), 원통 y −0.289(범위 끝)에서 접근 보정이 z 한계에 걸림 |
| 0.62 m | 손 20° | 6 | 0 | |
| 0.55 m | 손 25–40° 외 | 12 | 0 | §4 |

정렬 실패 1회(24회 중): 1 s 후진할 때 옆으로 약 6 cm 밀리고 방향이 −11~−14° 돌아, 짧은 재접근으로 다 바로잡지 못함.

데이터: `data/smoke/20261117/20261117_1530*`–`_1536*` (성공 4) → 검증 4/4(25.00 Hz 균일) → `data/processed/walk_grasp_desk070_smoke_v1_lerobot_v21`(39 MB), lerobot 로더 오류 0. 보행 명령이 있는 프레임 57–192개(후진 보정 −0.6 m/s 포함). `20261117_151335_scripted_fa5e018f`는 0.55 m 실패 녹화(outcome failure)입니다.

0.65 m(2026-11-18, 사용자 요청 "0.65 m로 10개 생성 후 허깅페이스 업로드"): `data/smoke/20261118/` 11개(성공 10, 실패 1) → 검증 11/11 → `data/processed/walk_grasp_desk065_scripted_v1_lerobot_v21`(성공 10, 5,349 프레임, 영상 83 MB) → 허깅페이스 `jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1`(비공개로 올린 뒤 사용자 요청으로 **공개** 전환, 카드 `docs/hf_cards/walk_grasp_desk065_scripted_v1.md`). 이어서 사용자 요청 "총 50개"로 추가 녹화하다가 "된 것까지만 올리고 세 원통 데이터로 넘어가자"는 요청으로 중단: 추가 성공 5개(끊긴 1개는 outcome aborted로 변환 제외)를 더해 `walk_grasp_desk065_scripted_v1_ep15_lerobot_v21`(성공 15, 8,362 프레임)로 같은 저장소를 갱신(공개, 다시 받아 로더 오류 0). 에피소드당 실제 시간 94–151 s. 중간에 Claude 세션이 끝나며 시뮬·배치가 같이 종료되어 녹화 중이던 1개는 결과 없이 끊겼고, `data/smoke/20261118_incomplete/`로 옮겨 두었습니다(변환 제외). 이후 재개는 `setsid nohup`으로 띄워 세션과 분리했습니다.

업로드: `.conda/envs/g1_inspire_data/bin/python tools/push_dataset_hf.py data/processed/<name>_lerobot_v21 --repo-id <user>/<name> --card-file <카드.md> [--public]` (사용자가 요청할 때만. 기본 비공개, meta에 절대 경로가 있으면 거부). 받아 쓸 때 이 환경의 data env는 torchcodec이 안 되므로 `video_backend="pyav"`.

## 6. 세 원통(빨강·파랑·초록) 언어 조건 데이터 (2026-11-18)

사용자 요청: "파란색·초록색 원통도 넣고 각 원통 집는 시연을 30개씩, 총 90개. 언어 입력이 필요하면 적절히 섞어서. 원통 위치는 전부 고정."

- **장면**: `configs/scene_desk.yaml` `extra_objects`(기본 켜짐). 같은 원통(Ø 60 mm, 높이 0.12 m) 셋을 고정 위치에: 파랑 (1.02, 0.05), 빨강 (1.02, −0.20, 기존 위치), 초록 (1.02, −0.45). 장면 엔티티는 `object`(빨강), `object_1`(파랑), `object_2`(초록)와 각 접촉 센서. 끄면(`--scene-set extra_objects.enabled=false`) 예전 단일 원통.
- **목표 지정**: `ctl reset --target <색>`(`record_episode.sh reset`도 같음). 시뮬은 목표 원통을 기존 eval 필드(`object_pos_w`, `object_lift_m`, `contact_force_total`)로, 세 원통 위치를 `eval.objects`로, 목표 이름을 `eval.target`으로 내보냄. 스크립트 전문가·녹화·변환 코드는 그대로 목표를 따라감. 원시 meta에 `target_object`, `objects_xy`, 변환 provenance에 에피소드별 `instruction`·`target_object`.
- **안전장치**: 리셋 메시지는 응답이 없어서, 배치가 허브에 `--plan-set expect_target=<색>`을 넘기고 스크립트가 시뮬의 목표와 다르면 걷기 전(녹화 전)에 실패로 끝냄(실제로 일부러 어긋나게 해 확인). 다른 원통이 3 cm 넘게 밀리면 실패(`max_other_displacement_m`).
- **지시문**: 에피소드마다 5가지 표현 중 하나를 고름 — "walk to the desk and pick up the {color} cylinder", "pick up the {color} cylinder", "go to the table and grab the {color} cylinder", "walk over and lift the {color} cylinder", "grab the {color} cylinder on the desk". task_name은 `walk_pick_<색>_cylinder`.
- **생성**: `G1IV_RECORD_OVERRIDES='["smoke_root=data/smoke/multi_cyl_v1"]' scripts/run_sim.sh --headless --scene-set desk.top_surface_height=0.65` 후 `tools/batch_walk_grasp.py -n 120 --targets red,blue,green --per-target 30 --fixed-objects --record --desk 0.65 --seed 2026111810 --task-name "walk_pick_{color}_cylinder"`. 성공이 가장 적은 색을 다음 목표로 고름.

| 목표 | 시도 | 성공 | 정렬 실패 | 들림 < 5 cm | 평균 보정 횟수 | 성공 시 들림 |
|---|---|---|---|---|---|---|
| 빨강 | 33 | 30 (91%) | 1 | 2 | 0.30 | 5.1–6.2 cm |
| 파랑 | 40 | 30 (75%) | 6 | 4 | 1.43 | 5.0–6.4 cm |
| 초록 | 38 | 30 (79%) | 2 | 6 | 0.97 | 5.0–6.2 cm |
| 합계 | 111 | 90 | 9 | 12 | | |

- 다른 원통이 밀린 경우 0회(최대 이동 0.0 cm). 시뮬 시험(카메라 없음)에서 색마다 1/1, 2026-11-18 11:3x.
- 들림 실패 12회는 모두 4.6–5.0 cm(기준 5 cm 바로 아래): 원통이 손 안에서 기울어진 채 들림(`logs/captures/multi_cyl/ep01_green_head_wrist.jpg`). 성공 회차의 들림도 5–6 cm로 계획(10 cm)보다 낮은 같은 이유.
- 파랑 정렬 실패가 많음: 정지 명령 후 관성으로 더 가는 거리가 회차마다 −5~+7 cm로 흩어지고 방향이 −5~−12° 틀어져 보정 3회로 허용오차(앞뒤·좌우 4 cm)에 못 들어감. 개선 후보: 목표 근처 감속.
- 실제 시간 평균 97 s/에피소드, 총 3.0 h. 원본 111개 검증 111/111.
- 변환 `walk_pick_3color_desk065_scripted_v1_lerobot_v21`(성공 90만, 에피소드당 약 1.1분). 사용자 요청 "된 것은 지금 올리고 나머지는 다 되면 한 번에": 변환 중인 폴더에서 `tools/snapshot_partial_dataset.py`로 일관된 사본(앞 18개, 색마다 6, 10,260 프레임)을 떠서 LeRobot·GR00T N1.7 로더 검사 통과 후 허깅페이스 `jammin0319/g1_inspire_walk_pick_3color_desk065_scripted_v1`에 **공개**(카드 `docs/hf_cards/walk_pick_3color_desk065_scripted_v1.md`). 16:00 변환 완료(90개, 50,876 프레임, 영상 360, 지시문 15종) → LeRobot·GR00T N1.7 로더 검사 통과 → 같은 저장소를 90개로 갱신(16:06, 공개). 먼저 올린 18개 파일은 90개 변환본과 바이트 단위로 같았고, 갱신 후 허브 파일 459개를 로컬과 SHA-256/내용으로 대조해 남는·빠진·다른 파일 0.

## 7. 0.55 m를 계속하려면 (선택지)

1. **파지 방식을 새로 설계**: 손등이 위를 향하게 굴리거나 위에서 집는 자세 등 엄지가 원통 옆면을 받치는 자세. 손-원통 접촉 기하를 따로 계산해야 하는 연구 과제입니다.
2. **물체를 더 긴 것으로**: 수평 손이 닿는 높이(책상 위 약 0.17–0.19 m)를 잡을 수 있는 키 큰 물체(예: 높이 0.25 m 병).
3. **허리 숙이기**: 공식 액션 프로바이더가 허리를 기본자세로 고정하고, 허브 IK도 허리를 잠급니다. 둘 다 바꾸고 보행 균형을 다시 검증해야 합니다.
4. **웅크리기 되는 보행 정책**: 공식 정책은 height 명령을 무시합니다. 재학습이 필요합니다.
5. **책상 0.65 m**: 지금 바로 동작합니다(5 cm 낮춤).

## 8. 먼 출발 데이터 (ACT 보행 학습용, 2026-11-19)

사용자 요청: "멀리서 열심히 걸어가서 파지까지 성공하는 시연. 데이터 약 50개 새로, 지나치게 휘청거리는 시연은 빼고."

**왜 바꿨나**: 15개 데이터로 학습한 ACT는 보행 명령을 항상 0으로 냈습니다(docs/ACT_EVAL.md). 그 데이터는 에피소드마다 걷는 프레임이 약 60개(2.4 s)인데, 도착 후 팔이 움직이기 전 서 있는 프레임이 약 75개(3 s)로 더 많았습니다(정지 1 s 이상 + 파지 대기 1 s + engage 0.5 s). 팔·손 상태가 같은 구간에서 "걷기"보다 "서 있기"가 많으면 L1 손실의 최적값이 0이 됩니다.

**바꾼 것** (`configs/scripted_walk_far_grasp_desk065.yaml`, 걷기·파지 기하는 desk065와 같음)
- 출발: `tools/batch_walk_grasp.py --robot-x -1.0 -0.4 --robot-y -0.3 0.3`(yaw ±15°, 원통 x 0.98–1.08, y −0.30–−0.10) → 서는 자리까지 1.1–1.8 m. 머리 카메라(아래로 47°, 수직 화각 약 82°)는 출발점에서도 책상과 원통을 봅니다.
- 도착 후 대기: `grasp_wait_s` 1.0 → 0.2 s, `engage_s` 0.5 → 0.3 s(새 계획 키, `src/g1iv_teleop/sources.py`; 키가 없는 기존 계획은 예전 값 그대로).
- 정렬 허용오차: 앞뒤 2.5 → 4 cm, 좌우 2.5 → 3 cm. 먼 거리에서 0.5 m/s로 오다 멈추면 관성 거리가 ±5 cm로 흩어져, 2.5 cm로는 시험 5회 중 3회가 "1 s 후진 → 재접근"을 했습니다. 후진할 때 골반이 8–9° 기울어(평소 5.3–5.8°) 눈에 띄게 휘청입니다.
- 걷기 시간 제한 40 → 60 s.

**휘청거림 필터** `tools/filter_walk_episodes.py` (원본은 그대로 두고 통과한 에피소드만 심볼릭 링크로 모음)
- 성공 + 골반 기울기 최대 ≤ 7° + 골반 높이 최소 ≥ 0.745 m + 롤·피치 각속도 최대 ≤ 1.0 rad/s.
- 후진 재접근이 있는 에피소드는 기울기 8–9°라 모두 빠집니다. 예전 15개 데이터의 넘어질 뻔한 회차(재접근 3회)는 15.4°, 골반 0.649 m였습니다.

**저장 위치**: `/mnt/isaac`가 12 GB만 남아 원본은 `/home/jammin/g1iv_store/raw/walk_far_v2/`, 변환본은 `/home/jammin/g1iv_store/processed/`, 학습 출력은 `/home/jammin/g1iv_store/train/`.

**수집 결과 (2026-11-19 21:09–22:43)**: 허용오차 4/3 cm 배치 `logs/checks/walk_far/batch_v2b.jsonl`(seed 1119003) 63회 중 성공 62, 넘어짐 0, 재접근 0회 57 / 1회 5 / 2회 1. 에피소드당 약 85 s(4060, 헤드리스). 원본 69개(2.5 cm 허용오차 시험분과 중단된 마지막 1개 포함)를 필터에 넣어 **55개 선택**(`/home/jammin/g1iv_store/raw/walk_far_v2_sel/report.json`).
- 빠진 14개: 재접근(기울기 8.4–9.0°) 8, 단일 보행이지만 롤·피치 각속도 1.05–1.12 rad/s 3, 실패 2, 중단 1.
- 선택된 55개: 전부 "걷기 1구간 → 서서 파지" 패턴. 걷기 80–128프레임(중앙값 111 = 4.4 s), 팔 움직이기 전 서 있기 30–56프레임(중앙값 50), 이동 1.11–1.72 m, 에피소드 474–532프레임(합계 27,931). 기울기 최대 5.70°, 골반 최저 0.754 m, 각속도 최대 0.82 rad/s.
- 예전 15개 데이터 대비 걷기:서 있기 비율이 약 0.8 → 2.2.

## 9. 넓은 무작위 출발 (v3, 2026-11-23)

- 사용자 요청: 로봇 출발을 좌우 1 m, 원통을 책상 폭 20–80%로 무작위화(과적합 방지).
- 출발: `tools/batch_walk_grasp.py --robot-x -1.0 -0.4 --robot-y -1.0 1.0 --object-y -0.45 0.45`.
- 계획: `configs/scripted_walk_wide_grasp_desk065.yaml`. 걷기·파지 기하와 허용오차는 §8과 같고, 걷기 규칙만 `lateral_mode: pursuit`로 바꿨습니다.
  - holonomic 규칙(정면 유지 + 옆걸음 vy ≤ 0.5)은 옆 오프셋이 1.5 m일 때 앞으로 먼저 도착해 버립니다.
  - pursuit는 서는 자리 `pursuit_pre_m`(0.8 m) 앞의 점을 향해 걷고, 몸을 진행 방향으로 최대 `pursuit_heading_max_deg`(40°) 돌립니다.
  - 좌우 오차가 `pursuit_blend_m`([0.05, 0.20] m) 구간에서 줄어드는 동안 holonomic 규칙으로 섞여 넘어갑니다.
  - 키가 없는 기존 계획은 예전과 같이 동작합니다.
- 카메라 없는 시뮬 시험 결과와 수집·학습·평가는 docs/ACT_EVAL.md §7에 있습니다.
