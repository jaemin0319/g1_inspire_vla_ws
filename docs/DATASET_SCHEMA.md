# DATASET_SCHEMA

두 단계로 저장합니다. ① 시뮬 프로세스가 쓰는 **raw 에피소드**(`data/raw`, mock/scripted는 `data/smoke`)와
② 학습용 **LeRobot v2.1 + GR00T 메타**(`data/processed/<name>_lerobot_v21`)입니다. raw는 덮어쓰지 않고,
에피소드 ID는 `YYYYmmdd_HHMMSS_<source>_<uuid8>` 형식으로 고유합니다.

## 시간 대응

- 보행 정책 때문에 제어 주기는 50 Hz(시뮬 시간 20 ms)로 고정입니다. 녹화는 그 격자 위에서 `record_every_n_steps`(기본 2)마다 하므로 **시뮬 시간 기준 25 Hz로 균일**합니다. 30 Hz(33.3 ms)는 20 ms 격자에서 균일하게 만들 수 없습니다. 50 Hz가 필요하면 `--record-set record_every_n_steps=1`을 쓰세요.
- **프레임 k의 의미**: `obs` = 제어 스텝 s_k가 실행되기 *직전* 상태(이미지·관절·base), `action` = 스텝 s_k 동안 관절에 *실제로 적용된* 목표입니다. 두 값을 같은 스텝에서 잡으므로 오래된 영상을 다시 쓰는 일이 없습니다.
- `sim_time` = 스텝 × 0.02 s(시뮬 시간), `wall_time` = 해당 관측을 잡은 순간의 `time.time()`. 실시간 계수 = Δsim / Δwall. 느린 시뮬에서는 시뮬 시간 25 Hz가 벽시계 25 Hz보다 느립니다. 두 값을 섞지 않고 각각 기록합니다.
- 카메라 프레임 번호(`cam_meta.<cam>.frame`)가 늘지 않으면 중복(stale) 영상으로 보고 검증에서 오류로 처리합니다. 녹화기는 기록 순간 카메라 버퍼를 강제로 최신 렌더로 갱신하고(`cam_meta.<cam>.forced_update = true`), 이때 번호는 읽은 횟수라 증가폭이 1 이상이면 정상입니다(공식 이미지 서버도 같은 카메라를 읽음).

## raw 에피소드 (`schema: g1iv-raw-1`)

```
<root>/<YYYYmmdd>/<episode_id>/
  meta.json          작업·언어 지시·소스·결과·종료 사유·정적 메타(관절명, 카메라, 버전, 설정 스냅샷, 정책 sha256)
  steps.jsonl        프레임당 1줄 (아래 키)
  arrays.npz         steps.jsonl의 숫자 필드를 쌓은 것 (키: "obs/arm_q" 등)
  images/{head,head_right,left_wrist,right_wrist}/000000.jpg   640×480 RGB, JPEG q95 (head_right = 헤드 스테레오 오른눈, 2026-11-11 20시 이후 녹화)
  depth/<cam>/000000.png                             (옵션) uint16 mm
  teleop_meta.jsonl  에피소드 동안 받은 허브 메타 전체
  preview.jpg        (scripts/record_episode.sh preview)
```

| 키 | 차원 | 단위·의미 |
|---|---|---|
| `frame_index`, `step` | 1 | 0부터 / 시뮬 제어 스텝 번호 |
| `sim_time`, `wall_time` | 1 | s |
| `obs.arm_q`, `obs.arm_dq` | 14 | rad, rad/s. 순서 L(shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw) 다음 R(같은 순서) |
| `obs.hand_q_rad` | 12 | Inspire 구동 관절 측정값, rad. DFX 순서: R(pinky, ring, middle, index, thumb pitch, thumb yaw), 그다음 L |
| `obs.hand_q_norm` | 12 | 위 값을 0..1로 정규화(1=펴기). sim의 `rt/inspire/state`와 같은 식 |
| `obs.joint_pos_all`, `obs.joint_vel_all` | 53 | Isaac 관절 순서 전체(`meta.static.joint_names_isaac_order`) |
| `obs.base_pos_w`, `obs.base_quat_w` | 3, 4 | m, (w,x,y,z) |
| `obs.base_lin_vel_b`, `obs.base_ang_vel_b`, `obs.projected_gravity_b` | 3 each | body frame |
| `obs.cam_meta.<cam>` | — | `frame`(카메라 프레임 번호), `forced_update`, `ts`(센서 내부 시각: 공식 이미지 서버가 추가로 진행시켜 시뮬 시각과 다른 시계이므로 동기 판단에 쓰지 않음) |
| `obs.image_files.<cam>` | — | 이미지 상대 경로 |
| `action.arm_q_target` | 14 | **실제 적용된** 팔 목표 rad (허브의 변화량 제한 후, DDS 경유) |
| `action.hand_q_rad_target`, `action.hand_q_norm_target` | 12 | 적용된 손 구동 목표 (rad / 0..1) |
| `action.hand_all_rad_target` | 24 | 연동 관절까지 펼친 손 목표 |
| `action.loco_cmd` | 4 | 정책이 이 스텝에 실제로 읽은 보행 명령 [vx m/s, vy m/s, wz rad/s, height m] |
| `action.leg_q_target_policy`, `action.waist_q_target` | 12, 3 | 보행 정책 출력과 허리 목표. **분석용이며 VLA action이 아닙니다** |
| `teleop.*` | — | 프레임 시점의 최신 허브 메타: `seq`, `age_ms`, `stale`, `arm_source`, `loco_source`, `input_ok`, `engaged`, `clutch`, `hand_cmd`(OPEN/CLOSE), `hand_q_sent`, `trigger`(원 입력), `arm_ik_q`(제한 전), `arm_sent_q`(제한 후), `ik_ok`, `loco_raw`, `loco_sent`, `loco_reason` |
| `eval.*` | — | 시뮬 ground truth: `object_pos_w`, `object_quat_w`, `object_lift_m`, `contact_force_per_filter`, `contact_force_total`, `fallen`. **평가·라벨 전용이고 정책 관측에 넣지 않습니다** |

`meta.json`의 주요 키는 `task_id`, `task_name`, `instruction`, `source`(pico|keyboard|mock|scripted), `is_smoke_data`,
`outcome`(success|failure|aborted|undetermined), `end_reason`(operator_stop|reset|sim_shutdown|hub_exit),
`seed`, `record_hz_sim`, `n_frames`, `n_dropped_by_writer`, `marks`, `fell`, `static.*`(버전·설정 전체)입니다.

## LeRobot v2.1 (+ GR00T) 변환 결과

GR00T N1.7 로더는 LeRobot **v2**와 `meta/modality.json`을 읽습니다. 그래서 v2.1(lerobot 0.3.3)로 고정했습니다.
특징 이름과 순서는 unitree_lerobot의 `Unitree_G1_Inspire` 규약을 따릅니다.

| feature | shape | 내용 |
|---|---|---|
| `observation.state` | 26 | [L arm 7, R arm 7, **L hand 6, R hand 6**] — 팔 rad, 손 0..1(1=펴기). 이름 `kLeftShoulderPitch … kRightHandThumbRotation` |
| `action` | 26 | 같은 순서의 **적용 목표** (상체+손만. 다리는 넣지 않음) |
| `action.hand_binary` | 2 | 사용자 OPEN/CLOSE (L, R; 1=CLOSE, −1=허브 메타 없음/오래됨) |
| `action.loco_cmd` | 4 | 보행 명령 (별도 열. VLA action 기본 범위 밖) |
| `observation.base_vel` | 6 | base 선·각속도 (body) |
| `observation.sim_time`, `observation.wall_time` | 1 | s |
| `observation.images.cam_left_high` | 480×640×3 video | 헤드 스테레오 왼눈 (Unitree/UnifoLM-VLA 키) |
| `observation.images.cam_right_high` | 480×640×3 video | 헤드 스테레오 오른눈 (녹화에 있을 때만) |
| `observation.images.cam_left_wrist`, `cam_right_wrist` | 480×640×3 video | 손목 (손바닥 쪽 광각, Inspire 링 브래킷) |
| `eval.object_pos_w`, `eval.contact_force_total`, `eval.fallen` | 3, 1, 1 | 평가 라벨 전용 |
| `task_index` → `meta/tasks.jsonl` | 1 | 언어 지시 |

- `meta/modality.json`: state/action은 `left_arm 0:7, right_arm 7:14, left_hand 14:20, right_hand 20:26`, action에는 추가로 `base_command 0:3 ← action.loco_cmd`(정책이 실제로 받은 vx, vy, wz. 상수인 높이 0.8은 제외), video는 `head/[head_right/]left_wrist/right_wrist`, annotation은 `human.task_description ← task_index`입니다.
- `meta/stats.json`: `observation.state`·`action`·`action.loco_cmd`의 mean/std/min/max/q01/q99. GR00T 전용 `relative_stats.json`은 GR00T 도구에서 만듭니다(미검증).
- 확인 도구 2개(2026-11-11 smoke_v1에서 실행, 둘 다 PASS): `python -m g1iv_data.check_loader <root>`(data env, lerobot 0.3.3, 영상은 `pyav`로 디코딩. lerobot 기본 torchcodec 0.5+cu128은 CUDA NPP 라이브러리가 없어 이 env에서 로드 실패) · `python -m g1iv_data.check_gr00t <root>`(vla env, GR00T `LeRobotEpisodeLoader` + torchcodec 0.8.0으로 그룹 차원·AV1 영상·언어·16스텝 action chunk 확인)
- `meta/g1iv_provenance.json`: raw 경로, 소스, 결과, 제외된 에피소드와 사유.
- 기본으로 `outcome=success`만 넣고, mock/scripted는 `--allow-smoke`를 줄 때만 넣습니다. 검증에 실패한 에피소드(프레임 드롭, stale 영상, 비균일 간격)는 제외합니다.
- 영상 코덱은 lerobot 0.3.3 기본값인 AV1(`libsvtav1`)이고 GR00T 예제 데이터와 같습니다. 인코더가 없으면 `--vcodec h264`를 쓰세요.
- 정규화 통계는 성공 에피소드만으로 계산합니다. 패딩·action chunk는 학습기 설정에서 정합니다(`check_loader.py`는 `delta_timestamps`로 8스텝 chunk를 시험).

## 카메라 파라미터 (2026-11-11 카메라 리그, docs/CAMERA_RIG.md)

- 원본: `meta.json` → `static.camera_rig.<alias>` = `scene_camera`, `lerobot_key`, `parent_link`, `position_m`, `quat_wxyz_ros`(OpenCV 광학 좌표), `R_parent_cam`, `K`, `dist`(OpenCV k1 k2 p1 p2 k3), `render_dist`(렌더러에 넘긴 값, 손목은 단조 재맞춤), `clipping_range`. `static.lens_distortion`에 렌즈 적용 여부가 있습니다.
- 변환본: `meta/info.json` → `camera_params`, Unitree 데이터셋(예: G1_Dex1_Stack_Block)과 같은 배치입니다.
  - `head_stereo`: `camera_matrix_left/right`, `dist_coeffs_left/right`, `R`, `T`[mm] (p_right = R p_left + T), `rectified: false`
  - `wrist_left|wrist_right`: `color.{camera_matrix, dist_coeffs, width, height, model}`
  - `extrinsics.<lerobot_key>`: 장착 링크·위치·쿼터니언 (프로젝트 확장)
- 서로 다른 리그로 녹화한 에피소드는 한 데이터셋으로 변환하지 않습니다(변환기가 거부).

