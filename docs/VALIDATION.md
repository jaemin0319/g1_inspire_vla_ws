# VALIDATION

기준 시각 2026-11-11 21:10 KST (카메라 리그 반영). 상태 표기:
- **PASS**: 이 PC에서 실제로 실행했고 기준을 통과함
- **PASS (모의)**: 모의 XR 입력(`MockSource`, 공식 televuer 변환 경로 사용) 또는 스크립트 입력으로 통과. **실제 PICO 검증이 아님**
- **PARTIAL**: 일부만 실행함(남은 부분을 적음)
- **NOT RUN (장비/자원)**: PICO가 없거나 GPU 메모리가 부족해 실행할 수 없음

헤드리스 명령이 종료 코드 0으로 끝났다는 이유만으로 통과로 보지 않습니다. 공식 `sim_main.py`는 시작 실패도 종료 코드 0으로 끝내므로 `run_sim.sh`가 로그의 실패 표시를 보고 3을 반환합니다.

## 완료 판정 목표 (요청서 §11)

| # | 항목 | 상태 | 근거 |
|---|---|---|---|
| 1 | 새 터미널에서 런처가 올바른 env로 시뮬 시작 | **PASS** | `scripts/run_sim.sh` 헤드리스·GUI 모두 기동(캐시 후 20–32 s). env `g1_inspire_sim`, ROS 변수 제거, DDS `lo`. GUI 창은 사용자가 직접 보고 화면 녹화함 |
| 2 | G1+양손 Inspire, 책상, 검정 바닥, 물체, 벽·천장 없음 | **PASS** | 카메라 캡처 `logs/captures/desk_head_nospec_dome1200.jpg`. 바닥 반사(Fresnel) 때문에 회색으로 보이던 것을 무반사 재질로 바꿔 바닥 픽셀 ≈47/255(가까운 곳·먼 곳 동일). 공식 창고 USD·포장 테이블 제거. 손 색상은 Inspire 실물 배색으로 덧입힘(`hand_appearance`, `logs/captures/hands_colored_grasp_head.jpg`) |
| 3 | 보행 정책으로 서기 유지 | **PASS** | 공식 장면 5분 이상 기울기 0.71°(`logs/checks/official_stand_probe.json`), 책상 장면 4.7°·base 0.789 m. 이후 모든 실행에서 `fallen=false` |
| 4 | 키보드 이동·회전·정지, 정지 후 균형 | **PARTIAL** | 같은 보행 명령 경로(LocoArbiter → `rt/run_command/cmd`)를 XR 스틱 모의(`walk_seq`)와 공식 형식 직접 명령(`tools/loco_cmd_test.py`)으로 확인: 전진 1.05 m, 좌 0.37 m, 후진 −0.82 m, 좌회전 +150°, 우회전 −52°, 정지 후 정지(`logs/checks/status_walk_seq_desk_v2.json`). **pygame 키 입력 자체는 사람이 눌러야 해서 미실행**(중재 로직은 단위 테스트 PASS). 공식 정책은 작은 명령에 발을 떼지 않음 → 키·스틱 상한을 공식 범위로 올림. 최대 회전 명령에서도 회전이 시작되지 않은 경우가 5회 중 2회 |
| 5 | 좌우 독립·동시 OPEN/CLOSE가 실제 관절에 반영 | **PASS (모의)** | `dry_run`: 왼손→오른손→양손 순서로 명령, 시뮬 손가락 정규값 오른손 0.15·왼손 0.25까지 닫히고 양손 동시 닫힘 구간 확인(`logs/checks/status_dry_run_hands.json`). 실제 파지는 스크립트로 성공(#8) |
| 6 | 모의 컨트롤러 pose → 공식 IK → 팔 목표 | **PASS (모의)** | IK 실패 0, IK 위치 오차 ≤ 7.3 mm, 시뮬 팔 추종 오차 중앙값 5.5 mrad. 허브 시작 시 팔 순간이동 버그(공식 제어기가 목표 0으로 발행 시작)를 발견해 수정, 수정 후 첫 명령-측정 차이 0 |
| 7 | 보행 중 상체 입력, 명령 충돌 없음 | **PASS (모의)** | `walk_arm`: 전진 1.42 m 하는 동안 오른손목 이동·CLOSE/OPEN, IK 실패 0, 걷는 중 팔 추종 오차 중앙값 6 mrad(최대 0.067 rad), 넘어짐 없음 |
| 8 | 접근 → 닫기 → 들어올리기 | **PASS (스크립트)** | 시뮬 GT 물체 위치를 쓰는 스크립트 계획(사람 텔레옵 아님, 로봇은 책상 앞에 리셋, 보행 없음). 녹화 3/3 + 시연 5/5 성공, 들림 7.2–7.4 cm(기준 5 cm). 손 기하는 USD에서 계산(`logs/checks/hand_geometry.txt`) |
| 9 | 테스트 에피소드 3개 이상(정상·리셋·중단 구분) | **PASS** | `data/smoke/20261111/`: success 3, aborted 1(`operator_abort`), undetermined 1(`reset`), 수정 전 failure 1. 모두 `source=scripted`, `is_smoke_data=true` |
| 10 | 동기·FPS·누락·차원 검사, 변환 후 loader batch | **PASS** | 검증기 6/6(시뮬 시간 25.00 Hz 균일, 드롭 0, `logs/validate/raw_20261111_154009.json`). 성공 3개 → LeRobot v2.1(`data/processed/smoke_v1_lerobot_v21`, 1,031 프레임) → lerobot 로더 배치(8스텝 chunk, 상태 원본과 차이 0.0, `logs/validate/loader_smoke_v1.json`) → **GR00T `LeRobotEpisodeLoader`**로 그룹 차원·AV1 영상·언어·16스텝 chunk 확인(`logs/validate/gr00t_smoke_v1.json`) |
| 11 | 입력 중단·recenter·재개 시 점프 없음, 보행 timeout | **PASS (모의)** | `dropout`: 2 s 동결 중 팔 변화 0.0017 rad, 손 CLOSE 유지(동결 순간의 "트리거 0"에 손이 열리던 문제를 찾아 확인 단계 추가), 재개 시 재앵커. `recenter`: 틱당 목표 변화 ≤ 7 mm. XR 0.3 s 무입력 → `xr_stale`로 보행 0 |
| 12 | GPU/VRAM, 카메라·기록·제어 주기, sim/wall 비율 | **PASS** | 아래 성능 표. 카메라 리그(4카메라+렌즈) 반영 후 RTF 0.329(서 있음), 녹화 중 0.27–0.29 (CAMERA_RIG.md §7) |
| 13 | 설치·내보내기·고정된 저장소로 재실행 경로 | **PASS** | `setup_envs.sh sim/teleop/data/vla` 재실행 시 종료 코드 0(이미 있는 것은 다시 설치하지 않음), `export_envs.sh` 실행. 설치 전후 IGRIS env 해시 동일 |
| 14 | 실제 PICO | **NOT RUN (장비)** | PICO_PENDING_CHECKLIST.md. 위 "모의" 결과와 섞지 않음 |
| 15 | VLA: loader / 모델 smoke / 학습 | loader **PASS** · 모델 smoke **NOT RUN** · 학습 **NOT RUN (자원)** | GR00T 로더까지 통과. 가중치는 받지 않음(N1.7 추론 ≥16 GB, 파인튜닝 ~35 GB/GPU) |

## 카메라 리그 (2026-11-11, docs/CAMERA_RIG.md)

Unitree G1+Inspire 데이터 수집 구성(헤드 스테레오 + 손바닥 쪽 광각 손목, Inspire 링 브래킷)을 시뮬에 반영한 검증입니다.

| ID | 무엇 | 결과 | 근거 |
|---|---|---|---|
| C1 | 시뮬 FK = 실제 로봇 FK | **PASS** | Unitree가 기록한 EE와 `wrist_yaw_link+(0.1659,0,0)`이 14프레임·양팔 RMS 1.2 mm |
| C2 | 헤드 카메라 정합 (실제 G1_WBT_Inspire 14프레임) | **PASS** | 실루엣-에지 비용 4.78 px (공식 프리셋 7.97, upstream 신버전 8.81). 6자유도 자유 맞춤도 대칭 해로 수렴 (`logs/camera_rig/fit_head_v6`, `v7`) |
| C3 | 손목 카메라 정합 (실제 28장) | **PASS (제한적)** | CAD 위치 + 손 축 쪽 5°가 후보 중 최소(30.74 px), 반대 기울기·G1-D 배치는 +3.6 px로 기각. 손목 맞춤은 헤드보다 덜 결정적 |
| C4 | RTX 렌즈 모델 = OpenCV 보정 | **PASS** | 35표식×4카메라, 오차 중앙값 0.17–0.22 px, 90% ≤ 0.43 px. 렌즈 없는 대조군은 대부분 25 px 초과 (`logs/camera_rig/lens_check_final`) |
| C5 | 실제 vs 시뮬 영상 비교 (22프레임) | **PASS (육안)** | `logs/camera_rig/compare_final_A.jpg`, `compare_final_B.jpg`, `compare_final_head_hw.jpg` |
| C6 | 책상 장면 기동 (4카메라, 렌즈, 브래킷) | **PASS** | 로그 `lens distortion ... applied`, `camera hardware`, 캡처 `logs/captures/rig_desk_{approach,lift}.jpg` |
| C7 | 스크립트 파지 (공식 손목 브래킷 충돌체 제거 후) | **PASS (스크립트)** | 3/3 성공, 들림 7.6–7.8 cm |
| C8 | 4카메라 녹화 → 검증 → 변환 → 로더 | **PASS** | `data/smoke/20261111/20261111_2045*`, 검증 2/2(경고 0), `smoke_rig4_v2_lerobot_v21`, lerobot 로더 오류 0, GR00T 로더 31/31, `info.json camera_params` 왕복 일치 |
| C9 | 기존 3카메라 데이터 호환 | **PASS** | `smoke_v1` lerobot 로더 오류 0 |
| C10 | 단위 테스트 | **PASS** | 24개 (카메라 리그 6개 추가) |

2026-11-12 개정 (손목 = Unitree D405 마운트 "C" + RealSense D405, 헤드 재맞춤):

| ID | 무엇 | 결과 | 근거 |
|---|---|---|---|
| C2′ | 헤드 재맞춤 (실루엣 채우기 버그 수정 후, 같은 14프레임) | **PASS** | 새 값 (0.0587, 0, 0.4310), 47.4°: 3.48 px. 이전 값 3.84, CAD 4.55, 공식 프리셋 9.23. 6자유도 해와 2.6 mm, 0.3° 차이 (`fit_head_v8_fixsil`, `v9_fixsil_sym`) |
| C11 | C + D405 조립 = CAD | **PASS** | 마운트 a/b 거울 대응 중앙값 0.000 mm. 판 M3 쌍 20.0 mm = D405 뒷면, D405-포켓 간섭 ≤ 0.01 mm. USB 개구 쪽 = 컬러 이미저 (`make_d405_c_asset.py`) |
| C12 | C의 손목 장착 = CAD 키 | **PASS** | φ 90°에서 하우징 겹침 ≤ 0.43 mm(예압), φ 108°는 3.7 mm(불가). Inspire 손 베이스 1.3–2.0 mm, 손가락 > 50 mm, 간섭 0 |
| C13 | 실제 D405 손목 영상 대조 (G1_Dex3_BlockStacking 16프레임 × 양손, 공식 Dex3 USD) | **부분 일치** | 구도(손바닥·손가락·반대 손 위치)는 일치. 손바닥 비용: CAD 정위치 19.2 / 24.9 px(오른 / 왼), 자유 맞춤 5.9 / 7.2 px. 그 로봇은 축 둘레 약 18°, 좌우 틸트 약 9° 다른 장착으로 보임. CAD 정위치를 기본으로 둠 (CAMERA_RIG §3.3, §9) |
| C14 | RTX 렌즈 = OpenCV (D405, 새 헤드) | **PASS** | 중앙값: 헤드 0.17 / 0.19 px, D405 0.29 / 0.37 px. 90% ≤ 0.56 px (`lens_check_d405c`) |
| C15 | 책상 장면 기동 + 스크립트 파지 녹화 → 검증 → 변환 → 로더 (C + D405) | **PASS** | 하드웨어·렌즈 4개 적용 로그. 파지 성공(들림 7.2 cm). `data/smoke/20261112/20261112_101305_scripted_0a91cc9e` 검증 1/1, `smoke_d405c_lerobot_v21` 검증 오류 0, lerobot·GR00T 로더 통과, `info.json camera_params.wrist_*.color` fx 433.85 / 434.36 |
| C16 | 단위 테스트 | **PASS** | 30개 (C + D405 4개 추가, A 모델 테스트는 `wrist.mount=inspire_ring_mono160`로 유지) |

2026-11-12에 발견하고 고친 것:
- ⑥ 피팅 도구의 `cv2.fillPoly` 일괄 채우기가 겹친 삼각형을 지워 실루엣이 와이어프레임이 됨 → 삼각형별 채우기로 고치고 헤드를 재맞춤. 2026-11-11 C2/C3 비용은 이 버그가 있는 상태의 값입니다.
- ⑦ 공식 USD의 `hand_camera_base_link`는 D405 마운트가 아니라 Dex3 단안 링 브래킷입니다(ICP 0.003 mm). 공식 USD는 그 링을 y ∓3 mm에 두었지만, 실제 하우징 축은 y 0입니다.

발견하고 고친 것: ① 공식 Inspire 손목 프리셋이 Dex3 값을 복사해 무릎을 찍음 → 리그로 교체. ② Isaac Sim 5.1 RTX OpenCV 렌즈의 p1 부호·0.5 px 규약 → 변환 반영. ③ 공식 이미지 서버와 녹화기가 같은 카메라를 읽어 최대 1 제어 스텝 지연 가능 → 기록 순간 강제 갱신. ④ Unitree v3.0 info.json의 손목 관절 이름 순서 오기(값은 roll, pitch, yaw). ⑤ Inspire WBT 데이터의 `wrist_*_ir`는 전부 0 (D405 아님).

## 손 = Inspire RH56E2 (2026-11-12, docs/HAND_RH56E2.md)

| ID | 항목 | 판정 | 근거 |
|---|---|---|---|
| H1 | E2 = DFTP, 관절 범위 = E2 매뉴얼 | **PASS** | E2 매뉴얼 2.6.21 DFTP 품번, CN 사이트 경로. URDF 엄지 회전 1.658 rad = 95°(75–170°), 굽힘 0.62 rad = 35.5°(28.5–64°) |
| H2 | 장착 = Unitree FTP 장착 | **PASS** | E2 베이스 vs Unitree FTP 베이스 메시(손목 좌표계) 중앙값 0.05 mm, p95 0.32 mm |
| H3 | 왼손 = 오른손 거울 = Inspire 왼손 URDF | **PASS** | 중앙값 0.001 mm (p99 6.3 mm, 베이스 국소) |
| H4 | 로봇 USD 덮어쓰기 | **PASS** | 관절·링크 순서 공식과 동일, 손 링크 26개 교체, 손 0.8987 kg × 2, 관절 좌표계 불일치 ≤ 0.000 mm / 0.022° (`tools/check_joint_frames.py`) |
| H5 | 시뮬 기동 | **PASS** | 관절 53개, 배색 재질 6개, E2 연동 12개가 공식 프로바이더에 들어감, DDS 범위 패치 |
| H6 | E2 스크립트 들어 올리기 (0.64, −0.12) | **PASS** | 폐루프 접근 3/3 성공 6.9 cm, IK 실패 0/0/83, 녹화 1개 검증 1/1 (`20261112_181339_scripted_bca0b0b7`, 메타에 hand_model) |
| H7 | 공식 DFX 손 회귀 | **PASS** | `robot.hand_model=inspire_dfx_official`, (0.70, −0.05) 성공 7.2 cm, IK 실패 0, 패치 없음 |
| H8 | 단위 테스트 | **PASS** | 38개 (`tests/test_hand_model.py` 8개 추가) |

- ⑧ DFX용 파지 위치를 E2에 그대로 쓰면 엄지(미리 회전 81°)가 원통 자리를 차지해 원통이 넘어집니다. 서는 위치 (0.70, −0.05)에서는 IK가 손을 충분히 당기지 못합니다(실패 589–1132회). 손 모델별 프리셋·경로·서는 위치로 고쳤습니다.
- ⑨ 폐루프 접근의 수직 보정이 팔이 닿지 못하는 곳까지 누적돼(IK 잔차 2.9 cm), 들어 올리기 처음 3 cm가 잔차를 메우는 데 쓰였습니다(들림 4.0 cm). 들어 올리는 동안 수직 보정을 풀어 6.9 cm가 됐습니다.

## 성능 (헤드리스, 서 있는 상태, 20 s 평균, `tools/perf_matrix.sh`, `logs/checks/perf_*.json`)

| 조건 | RTF(sim/wall) | 제어 주기(실제 시간) | GPU 메모리 | 시뮬 CPU |
|---|---|---|---|---|
| 4060, 정책 카메라 3개(기본) | 0.366 | 18.3 Hz | 2.8 GB | 519 % |
| 5060 Ti, 카메라 3개 | 0.420 | 21.0 Hz | 2.0 GB | 569 % |
| 4060, 카메라 제외 | 0.371 | 18.5 Hz | 2.8 GB | 522 % |
| 4060, 렌더링 끔 | 0.306 | 15.3 Hz | 3.2 GB | 422 % |
| 4060, GPU 물리(`--device cuda:0`, 실험) | 0.291 | 14.5 Hz | 4.7 GB | 239 % |
| 4060 GUI(참고) | 0.29 | 14.3 Hz | — | — |

- 시뮬은 실시간의 약 0.3–0.4배로 돕니다(GUI + 카메라 패널 + 화면 녹화 동시 실행이면 0.22).
- **정정(2026-11-16, 메인 스레드 cProfile 측정):** 위 표만 보고 "병목은 CPU 물리·정책"이라고 결론 냈던 것은 틀렸습니다.
  - 표의 "카메라 제외"는 공식 `--camera_exclude`와 같은 방식이라 읽기 주기만 늘렸고, 렌더 대상(render product)은 그대로 남아 렌더링 비용이 줄지 않았습니다.
  - 헤드리스 카메라 4대 + 렌즈 조건에서 제어 1스텝은 59.2 ms였습니다.

    | 항목 | ms/스텝 |
    |---|---|
    | 렌더링(`omni.kit.app.update`) | 27.3 |
    | 카메라 버퍼 읽기 | 14.2 |
    | 텐서 복제 | 4.7 |
    | PhysX(4 서브스텝) | 5.4 |
    | 보행 정책 | 0.2 |
    | 나머지 | ~7 |

  - 렌더링과 카메라 처리가 78%를 차지합니다.
  - 기록: `G1IV_PROFILE=300:800 scripts/run_sim.sh ...` → `logs/profile/sim_*.txt`.
- **시연 모드 `scripts/run_sim.sh --demo`:** 카메라 센서를 하나도 만들지 않으므로 카메라 데이터가 없고 녹화도 거부됩니다. 대신 실시간으로 볼 수 있습니다.
  - 헤드리스 9.9 ms/스텝(실시간의 2배까지 가능).
  - GUI 20.0 ms/스텝 = RTF 1.00. 뷰포트는 2스텝마다 그리고, 누적 마감 방식으로 페이싱하며, Kit 메인 루프의 60 Hz 제한을 끕니다(렌더 1회 21.6 → 13.5 ms). 대기 시간이 26%로 여유가 있습니다.
  - 스크립트 파지(2026-11-16): 동작 구간 RTF 1.00(sim 15.7 s = 실제 15.7 s). success, 들림 6.9 cm로 느린 모드와 같습니다.
- **`--demo-cams`:** `--demo`에 정책 카메라 4대와 GUI 카메라 패널을 더한 모드입니다.
  - 카메라는 2스텝마다 렌더·갱신하고, 패널은 4스텝마다 갱신합니다. 공식 카메라 관측(이미지 서버/PICO 스트림)은 끕니다.
  - 측정: 35.1 ms/스텝 = RTF 0.57. 렌더 1회 32.6 ms(카메라 4대 + 뷰포트), 패널 갱신 1회 33.5 ms(카메라 읽기 포함).
  - 스크립트 파지: RTF 0.53, success, 들림 7.0 cm.
  - 이 모드도 녹화는 거부합니다(카메라 프레임이 2스텝마다 반복되므로).
- CPU 성능 프로필은 powersave입니다(바꾸려면 sudo 필요, 이번 작업에서 바꾸지 않음).
- 녹화는 시뮬 시간 25 Hz로 균일하고(실제 시간 7.7–8.5 Hz), 이미지 서버는 같은 프레임을 약 30 Hz로 재전송합니다.
- 허브의 관절·손 속도 제한은 RTF를 곱해 시뮬 시간 기준으로 적용합니다. 사람이 텔레옵하면 로봇이 손보다 약 3배 느리게 따라옵니다.

## 실행 기록 (주요)

| ID | 무엇 | 결과 | 로그 |
|---|---|---|---|
| S1–S3 | Isaac Sim 단독 렌더 4060/5060 Ti, 로봇 USD 검사 | PASS | `logs/setup/smoke_isaacsim_*.json`, `robot_usd_joints.json` |
| S4 | 설치 전후 IGRIS env 스냅샷 | 동일(77,823 파일) | `logs/setup/igris_env_snapshot_{pre,post}_install_*.json` |
| S5 | `pip check` 4개 env | sim: 알려진 fastapi 1줄, teleop/data: 통과, vla: 의도적 미설치 목록 | `envs/*.pip-check.txt` |
| R1 | 공식 태스크(확장 없음) 서기 | PASS | `logs/checks/official_stand_probe.json` |
| R2 | 프로젝트 장면 기동·카메라 3개 | PASS | `logs/captures/desk_*.jpg` |
| T1–T6 | 모의 dry_run/dropout/recenter/walk_xr/walk_seq/walk_arm | PASS (모의) | `logs/teleop_runs/*.json`, `logs/checks/status_*.json` |
| G1 | 스크립트 파지(녹화 포함) | 3/3 + 시연 5/5 | `data/smoke/20261111/*/meta.json`, `preview.jpg` |
| D1–D3 | 원본 검증 → 변환 → LeRobot/GR00T 로더 | PASS | `logs/validate/` |
| U | 단위 테스트 18개 | 18 PASS | `logs/tests/unit_20261111_154455.log` |

## 알려진 제한

- **종료 멈춤(공식 동작)**: 정리가 끝난 뒤 `simulation_app.close()`가 돌아오지 않습니다(프로젝트 확장 없이도 같음). `run_sim.sh`/`stop_all.sh`가 그 시점 5초 뒤 종료하고 정상 종료로 보고합니다. 녹화는 그 전에 닫힙니다.
- **접촉력 평가 라벨**: 손-물체 필터 접촉 센서 값이 쥐고 있을 때도 0입니다(원인 미확인). 성공 판정은 GT 들림 높이로 하므로 영향은 없지만, `eval.contact_force_total` 열은 지금 쓸모가 없습니다.
- **공식 보행 정책**: 작은 명령에는 걷지 않고, 최대 회전 명령에서도 회전이 시작되지 않는 경우가 있습니다(5회 중 2회).
- **스크립트 파지**: 로봇을 책상 앞 (0.70, −0.05)에 리셋한 뒤에만 동작합니다(보행 없이 팔만 씀).

## 자동 보행 → 파지 → 들기 (2026-11-17, docs/AUTO_WALK_GRASP.md)

책상 상판을 0.70 → 0.55 m로 낮춤(사용자 요청). 스크립트 전문가(시뮬 정답 사용, `source=scripted`)가 XR 스틱 경로로 걸어가 파지·들기까지 자동으로 하는지 확인. 근거 파일은 `logs/checks/auto_walk_grasp/`.

| ID | 무엇 | 결과 | 근거 |
|---|---|---|---|
| W1 | 공식 정책 height 명령(웅크리기) | **동작 안 함** | 0.30–0.80 모두 골반 0.788–0.789 m (`crouch_probe2/3.json`) |
| W2 | 보행 접근·정렬 (무작위 시작 ±0.1/±0.2 m, ±15°) | **PASS (스크립트)** | 23/24, 성공 시 \|앞뒤\| ≤ 2.2 cm, \|좌우\| ≤ 2.4 cm, \|방향\| ≤ 5.9° (`batch_*.jsonl`) |
| W3 | 보행→파지→들기, 0.70 m | **PASS (스크립트)** | 9/10, 들림 6.0–6.9 cm, 넘어짐 0 (`batch_desk070_rt2.jsonl`, `batch_desk070_rec.jsonl`) |
| W4 | 보행→파지→들기, 0.65 m (손 10° / 15°) | **PASS (스크립트)** | 3/3, 4/4 (`batch_d065_*.jsonl`, `batch_launcher_check.jsonl`) |
| W5 | 보행→파지→들기, 0.62 m (손 20°) | **FAIL** | 0/6 (`batch_d062_*.jsonl`) |
| W6 | 보행→파지→들기, 0.55 m (현재 기본 책상) | **FAIL** | 0/12. 손을 35–40° 숙여야 닿고, 그러면 엄지가 원통 윗면에 얹혀 원통이 밀려남 (`logs/captures/auto_walk_grasp/desk055_tilt35_thumb_on_top.png`) |
| W7 | 걷기 포함 4카메라 녹화 → 검증 → 변환 → 로더 | **PASS** | `data/smoke/20261117/` 성공 4, 검증 4/4(25.00 Hz), `walk_grasp_desk070_smoke_v1_lerobot_v21` 로더 오류 0, 보행 명령 프레임 57–192 |
| W8 | 계획-책상 높이 불일치 검사 | **단위 테스트만** | `tests/test_walk_grasp.py::test_desk_mismatch`. 실기 확인 1회는 정렬 단계 실패로 검사까지 가지 못함 |
| W9 | 단위 테스트 | **PASS** | 42개 (자동 보행·파지 4개 추가, 책상 높이 기대값 0.55로 수정) |
| W10 | 0.65 m 4카메라 녹화 배치 (2026-11-18) | **PASS** | 10/11 성공(실패 1회는 들림 4.8 cm < 5 cm, 접근 보정이 z 한계 −0.04 m에 걸림), 정렬 11/11. 원시 검증 11/11(25.00 Hz) → `walk_grasp_desk065_scripted_v1_lerobot_v21`(10 에피소드, 5,349 프레임) 로더 오류 0 (`batch_desk065_rec.jsonl`, `batch_desk065_rec*.log`) |
| W11 | 허깅페이스 업로드 후 다시 받기 | **PASS** | `jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1`(`v2.1` 태그, 파일 59개, 비공개로 올린 뒤 사용자 요청으로 **공개** 전환, 로그인 없이 info.json·영상 200 확인). 새 폴더로 받아 `check_loader` 오류 0, pyav로 4개 영상 프레임 디코딩 확인 |
| W12 | 세 원통 장면·목표 지정 (카메라 없는 시뮬) | **PASS** | 엔티티 `object`/`object_1`/`object_2` 생성, `reset --target blue` 로그 확인, 색마다 보행→파지→들기 1/1, 다른 원통 이동 0 (`batch_multi_test.jsonl`) |
| W13 | 목표 불일치 안전장치 | **PASS** | 리셋 red + `expect_target=blue` → 걷기 전에 "target mismatch" 실패, 녹화 없음 (`logs/teleop_runs/20261118_113739_scripted.json`). 단위 테스트 2개 추가(44개 통과) |
| W14 | 세 원통 4카메라 녹화 90개 | **PASS** | 111회 중 90 성공(빨강 30/33, 파랑 30/40, 초록 30/38), 다른 원통 이동 0, 원시 검증 111/111 (`batch_multi_rec.jsonl`, AUTO_WALK_GRASP.md §6) |
| W15 | 보행 명령 기록 확인 | **PASS** | `action.loco_cmd`는 보행 정책이 `get_run_command()`로 실제 읽은 값(ext.py 래퍼). 15개 데이터에서 걷는 프레임 16%, 명령 vx와 0.4 s 뒤 몸 vx 상관 0.84–0.95 |
| W16 | 세 원통 부분 사본(18개) LeRobot·GR00T N1.7 로더 | **PASS** | 로더 오류 0, GR00T `LeRobotEpisodeLoader`: 그룹 차원(팔 7·손 6·base_command 3), 4개 AV1 영상, 지시문, 16스텝 행동 묶음. 공개 업로드 후 로그인 없이 info.json 18 에피소드 확인 |
| W17 | 세 원통 90개 LeRobot·GR00T N1.7 로더 + 허브 대조 | **PASS** | `walk_pick_3color_desk065_scripted_v1_lerobot_v21` 90개(빨강·파랑·초록 30씩), 로더 오류 0, GR00T 검사 통과 (`logs/validate/gr00t_walk_pick_3color_desk065_scripted_v1.txt`). 허브 459개 파일 = 로컬 + 카드, 불일치 0, 로그인 없이 90 에피소드 |
| W18 | 새 복제본에서 `scripts/bootstrap.sh` 전 단계 (GitHub 업로드 전) | **PASS (같은 PC)** | repos 1분 48초·파일 차이 0, assets 2분·423개 차이 0, envs 22분 47초(pip 경로, Isaac Sim 5.1 포함, 핵심 버전 동일), hand USD SHA-256 동일, 테스트 44개, smoke 통과, 세 원통 자동 파지 2/2. 다른 PC·GUI·vla/ACT 환경은 미확인 (docs/INSTALL.md §6, `logs/checks/repro_test/`) |
| W19 | 먼 출발 55개 LeRobot·GR00T N1.7 로더 + 허브 대조 (2026-11-21) | **PASS** | `walk_far_grasp_desk065_scripted_v2_lerobot_v21`(55 에피소드, 27,931프레임). LeRobot 로더 오류 0·원시 대비 상태 차이 0 (`logs/validate/loader_walk_far_grasp_desk065_scripted_v2.txt`), GR00T 검사 통과(826항목, `gr00t_..._v2.txt`). 업로드 사본은 `/home/jammin/g1iv_store/hf_upload/`(데이터·영상 하드링크, provenance의 raw_dir만 상대 경로로 바꿈). 공개 `jammin0319/g1_inspire_walk_far_grasp_desk065_scripted_v2` 허브 284개 = 로컬 282 + 카드·.gitattributes, 불일치 0, `v2.1` 태그, 로그인 없이 info.json 55 에피소드·LeRobotDatasetMetadata 로드 확인 (`hf_verify_..._v2.txt`) |
| W20 | 넓은 무작위 출발 100개 LeRobot·GR00T N1.7 로더 + 허브 대조 (2026-11-23) | **PASS** | `walk_wide_grasp_desk065_scripted_v3_lerobot_v21`(100 에피소드, 51,804프레임, 녹화 131개 중 휘청 필터 통과 앞 100개). LeRobot 로더 오류 0·원시 대비 상태 차이 0 (`logs/validate/loader_walk_wide_grasp_desk065_scripted_v3.txt`), GR00T 검사 통과(1,501항목, 258 s, `gr00t_..._v3.txt`). 공개 `jammin0319/g1_inspire_walk_wide_grasp_desk065_scripted_v3` 허브 509개 = 로컬 507 + 카드·.gitattributes, 불일치 0, `v2.1` 태그, 로그인 없이 info.json 100 에피소드 확인 (`hf_verify_..._v3.txt`). 업로드 자동화 `/home/jammin/g1iv_store/v3_hf_upload.sh` |
