# SETUP_REPORT

마지막 갱신: 2026-11-18 17:45 KST

## 1. 현재 상태 요약

| 구분 | 상태 |
|---|---|
| 작업공간 | `/mnt/isaac/g1_inspire_vla_ws` (`~/g1_inspire_vla_ws`는 링크). Git 저장소는 만들었지만 커밋하지 않음 |
| env 4개 | `g1_inspire_sim`(Isaac Sim 5.1 + Isaac Lab + Unitree 시뮬), `g1_inspire_teleop`(televuer·IK·SDK), `g1_inspire_data`(LeRobot 0.3.3), `g1_inspire_vla`(GR00T 로더 전용) **모두 설치·실행됨** (VERSION_MATRIX.md) |
| 시뮬 | 프로젝트 책상 장면 + 공식 보행 정책: 서기, 보행, 팔 IK, 양손 OPEN/CLOSE, 녹화 **실행 검증** |
| 로봇·손 | 몸체 = G1 29자유도 rev 1.0(= G1 EDU Plus 몸체, 유지 이유 HAND_RH56E2.md §5). 손 = **Inspire RH56E2(RH56DFTP 계열)**, Inspire URDF 형상·질량·관절 범위·연동, 사진 기준 배색. 공식 손은 `--scene-set robot.hand_model=inspire_dfx_official` |
| 카메라 | 헤드 스테레오 2(Unitree 60 FPS 모듈·브래킷) + 손목 **RealSense D405 2 (Unitree D405 손목 마운트 "C", CAD 정위치)**, 실측 렌즈 왜곡. 실제 Unitree 데이터 프레임과 대조 (CAMERA_RIG.md). 이전 손목(Inspire 링 + 160° 단안)은 `--rig-set wrist.mount=inspire_ring_mono160` |
| 텔레옵 | 모의 XR 입력(공식 televuer 변환 경로)과 스크립트 입력으로 검증. **실제 PICO는 장비가 없어 미검증** |
| 데이터 | smoke 에피소드 6개(스크립트) → 검증 → LeRobot v2.1 + GR00T 메타 → lerobot 로더와 **GR00T 로더 통과** |
| 책상 | 상판 **0.55 m** (2026-11-17 사용자 요청으로 0.70에서 15 cm 낮춤) |
| 자동 보행→파지→들기 | 스크립트 전문가(시뮬 정답)가 XR 스틱 경로로 걸어가 파지·들기: **0.70 m 9/10, 0.65 m 7/7, 0.62 m 0/6, 0.55 m 0/12**. 걷기 포함 4카메라 녹화 → 검증 → 변환 → 로더 통과 (AUTO_WALK_GRASP.md) |
| 남은 것 | 0.55 m 책상 파지 방식, 실제 PICO 검증, 사람 키보드 조작 확인, 실제 텔레옵 데이터, GR00T 모델 smoke·학습(서버) |

검증 항목별 상태는 VALIDATION.md에 있습니다.

## 2. 진행 기록

| 시각 | 내용 |
|---|---|
| 08:41–09:40 | 호스트 확인, 버전 결정(unitree 5.1 브랜치, IsaacLab 80094be, torch cu128), Isaac Sim 하드링크, 에셋 고정, 프로젝트 코드·문서 작성, 단위 테스트. 외부 저장소 설치가 자동 권한 정책에 막혀 중단 |
| 14:20 | 사용자가 권한 규칙을 추가(`~/.claude/settings.json`, 백업 `settings.json.bak-20261111`) → 설치 재개 |
| 14:21–14:45 | sim/teleop/data 설치. 해결한 문제: cyclonedds 0.10.2 cp311 빌드(conda-forge C 0.10.5), pyarrow 26 ↔ numpy 1.x, starlette/cryptography/psutil 고정, `pip show … \| grep -q`가 pipefail에서 실패해 Isaac Sim을 매번 재설치하던 스크립트 버그, Isaac Lab source 패키지 누락(h5py) |
| 14:30 | 공식 태스크 기동·서기 확인. 공식 sim_main이 시작 실패도 종료 코드 0으로 끝내는 것 확인 → 런처가 로그로 판정 |
| 14:37–14:46 | 프로젝트 장면: 책상 prim 경로 버그 수정, 바닥을 무반사 재질로(검정), 종료 멈춤 분석(공식 동작) → 런처 처리 |
| 14:48–15:10 | 모의 텔레옵: vuer/params-proto 충돌 수정, 허브 시작 시 팔 순간이동 수정, 모의 시나리오 버튼 덮어쓰기 버그 수정, 보행 명령 문턱 측정 → 키·스틱 범위 상향, 끊김 순간 손 열림 수정 |
| 15:11–15:40 | 스크립트 파지: 시뮬 시간 기준 계획·속도 제한, USD에서 손 기하 계산, 손 방향 회전 + 책상 모서리 회피 단계 추가 → 성공. smoke 세트 녹화 |
| 15:40–15:46 | 검증 6/6, 변환, lerobot 로더(pyav), vla env(로더 전용), GR00T 로더 통과 |
| 15:43–15:49 | 사용자 요청으로 GUI 시연(파지·들어 올리기), 관찰 시점 변경 |
| 15:52–16:00 | 성능 측정(4060/5060 Ti, 카메라 유무, 렌더링 끔, GPU 물리), 보행 중 팔 입력 시험 |
| 16:00–16:10 | 사용자 요청으로 Inspire 손 배색 적용(흰 마디·검은 끝·금속 엄지), 파지 재확인 |
| 18:50–21:10 | 사용자 요청: UnifoLM-VLA·Unitree 표준 손목 브래킷/헤드 카메라 조사 → 반영 → 검증. UnifoLM-VLA 저장소, HF 데이터 206개 메타, xr_teleoperate Device.md·CAD, upstream 보정 파일, 논문(조사 에이전트) 확인. 실제 G1_WBT_Inspire 프레임으로 헤드·손목 자세 맞춤, RTX 렌즈 규약 측정, 브래킷 메시, 4카메라 녹화·변환·로더 (CAMERA_RIG.md) |
| 11-12 16:40–18:25 | 사용자 요청: 손을 Inspire RH56E2(RH56DFTP) 기준으로 다시 불러오고 배색을 그 모델대로, 이어서 G1 → G1 EDU Plus 교체 검토. Inspire E2 URDF 3종·Unitree FTP URDF·제품 사진·매뉴얼 조사(에이전트 2), 손 자산 도구·로봇 USD 덮어쓰기 빌드, DDS 범위·연동 패치, 손 모델별 파지 프리셋·경로·폐루프 접근, 헤드리스 들어 올리기 3/3, 공식 손 회귀, 테스트 38개 (HAND_RH56E2.md, VALIDATION H1–H8) |
| 11-17 14:50–15:55 | 사용자 요청: 책상 15 cm 낮춤 + 보행→파지→들기 자동화 검증(sim-to-sim 데이터 자동 생성 목적). 공식 정책 웅크리기·보행 응답 측정, IK 도달 범위 스캔, 자동 보행 입력(ScriptedWalkGraspSource)·배치 도구·책상별 계획, 0.55/0.62/0.65/0.70 m 시험, 4카메라 녹화·검증·변환 (AUTO_WALK_GRASP.md, VALIDATION W1–W9) |
| 11-18 10:30–11:10 | 사용자 요청: 0.65 m로 10개 생성 후 허깅페이스 업로드. 11회 녹화(성공 10) → 검증 → 변환 → `jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1` 업로드·재다운로드 확인, 사용자 요청으로 공개 전환. 업로드 도구 `tools/push_dataset_hf.py`, 배치 `--until-success` 추가 (VALIDATION W10–W11) |
| 11-18 11:15–11:25 | 사용자 요청 "총 50개"로 추가 녹화 중 "된 것까지만 올리자"로 중단 → 성공 15개로 공개 저장소 갱신 |
| 11-18 11:25–15:00 | 사용자 요청: 파랑·초록 원통 추가(위치 고정), 색마다 30개씩 90개, 지시문 섞기, 새 데이터셋. 장면 `extra_objects`, 리셋 `--target`, 목표 불일치·옆 원통 밀림 판정, 배치 `--targets/--per-target`, 111회 녹화(성공 90) → 검증 → 변환 → 부분(18개) 공개 업로드 → 90개 로더·GR00T 검사 후 공개 갱신 (AUTO_WALK_GRASP.md §6, VALIDATION W12–W17) |
| 11-18 16:10–17:45 | 사용자 요청: 구성 전체를 GitHub 새 저장소로, 어느 Ubuntu 24.04에서도 재현 가능하게. `scripts/bootstrap.sh`(점검·micromamba·외부 저장소·공식 에셋·env·EULA·손 빌드·인증서·테스트·기동), 잠금 파일(`envs/assets.lock.json`, `envs/hand_reference.lock.json`), GPU 자동 선택, EULA 명시 동의, 패치에서 .pyc 제외, `cad` env. 새 복제본에서 전 단계 실행·비교 (INSTALL.md, VALIDATION W18) |
| 11-12 09:10–10:40 | 사용자 요청: 손목을 C(Unitree D405 마운트) + D405로, 브래킷 정위치에, D405 형상 반영. realsense-ros D405 메시·xacro, C STEP 분석(포켓·USB 개구·키), Dex3 USD 메시, Unitree G1_Dex3_BlockStacking 실제 D405 손목 프레임 대조, 피팅 도구 실루엣 버그 수정과 헤드 재맞춤, 렌즈 검증, 책상 장면 녹화·변환·로더 (CAMERA_RIG.md §3.3, VALIDATION C11–C16) |

## 3. 주요 결정과 공식 구성 대비 편차

| # | 내용 | 이유 |
|---|---|---|
| D1 | torch 2.7.0 **cu128** | cu126은 5060 Ti(sm_120) 미지원 |
| D2 | Isaac Sim 5.1 하드링크 재사용 | 이미 같은 버전이 있음. 원본 해시 불변 확인 |
| D3 | 에셋 zip HF revision 394cf 고정 | 5.1 브랜치와 같은 시점 |
| D4 | 녹화 25 Hz(시뮬 시간) | 보행 정책 50 Hz 격자 |
| D5 | 녹화를 시뮬 프로세스 안에서 | 공식 이미지 서버에 프레임 번호·시각이 없음(실제로 같은 프레임을 ~30 Hz로 재전송) |
| D6 | DDS 인터페이스 `lo`, 도메인 1 유지 | 연구실 망으로 나가지 않게. 같은 PC 안 통신은 확인됨(멀티캐스트 꺼짐 경고는 정상) |
| D7 | sim_main 종료 정리를 자기 자식만으로 | 공식은 같은 이름의 모든 프로세스 종료 |
| D8 | 확장이 시작 시 기본 팔 자세·OPEN·보행 0을 넣음 | 공식은 팔 목표 0에서 시작 |
| D9 | 키보드는 허브 창 포커스 시에만 | 다른 창 타이핑으로 로봇이 움직이는 위험 제거 |
| D10 | 시험 물체 r 30 mm·h 120 mm·0.15 kg | Inspire 한 손 파지 크기 |
| D11 | CLOSE 프리셋 wrap | 원기둥 감싸기 |
| D12 | LeRobot **v2.1** + modality.json | GR00T N1.7이 v2를 읽음 |
| D13 | Isaac Lab source 패키지 6개를 constraints로 `pip -e`(RL 프레임워크 extras 제외) | `isaaclab.sh --install`이 torch·numpy를 바꾸지 않게. 처음 3개만 깔았다가 h5py 누락으로 실패해 6개로 맞춤 |
| D14 | Inspire DFX 인터페이스 + 자체 OPEN/CLOSE 어댑터 | 공식 xr_teleoperate는 Inspire+컨트롤러 조합을 거부 |
| D15 | cyclonedds C 라이브러리를 conda-forge 0.10.5로 | unitree SDK FAQ 방식(0.10.x C 라이브러리 + `CYCLONEDDS_HOME`), ROS 것은 쓰지 않음 |
| D16 | 바닥 무반사(UsdPreviewSurface specular workflow, specular 0) | 반사 때문에 검정 바닥이 회색으로 보임 |
| D17 | `run_sim.sh`/`stop_all.sh`의 종료 처리 | 공식 시뮬이 시작 실패에도 0을 반환하고, 종료 시 `close()`에서 멈춤 |
| D18 | 허브가 공식 팔 제어기의 발행 시작을 붙잡음 | 공식 제어기가 목표 0으로 즉시 발행해 팔이 순간이동 |
| D19 | 키·스틱 보행 상한을 공식 키보드 범위로 | 공식 정책이 작은 명령에 발을 떼지 않음(측정) |
| D20 | 손 트리거 전이 확인(50 ms + 새 포즈) | 링크가 멈추는 순간의 "트리거 0"이 손을 열 수 있음 |
| D21 | 허브 속도 제한·스크립트 계획을 시뮬 시간 기준으로 | RTF ~0.35에서 실제 시간 기준이면 시뮬 속도가 3배 |
| D22 | modality.json action에 `base_command ← action.loco_cmd`(vx, vy, wz) | 걷기+조작 과제라 보행 명령도 행동에 포함 |
| D23 | vla env는 GR00T 로더 전용(CPU torch, `--no-deps`) | 이 PC는 학습용이 아님. 형식 호환만 확인 |
| D24 | 손 배색 덧입히기(`hand_appearance`) | 사용자 요청. 공식 에셋은 손이 거의 흰색. 시각만 바뀜 |
| D25 | 카메라를 공식 프리셋 대신 `configs/camera_rig.yaml` 리그로 (헤드 스테레오 + 손목 광각) | Unitree G1+Inspire 실제 데이터 수집 구성. 공식 Inspire 손목 프리셋은 Dex3 값 복사로 무릎을 찍음 |
| D26 | 헤드 내부 파라미터 = G1_Dex1_Stack_Block 장비 실측, 장착 = 실제 Inspire 프레임 맞춤 (46.3°) | 공개 보정 11대 중 중앙값 장비, UnifoLM-VLA 학습 데이터. 초점거리·위치는 영상만으로 분리 불가 |
| D27 | 손목 = Inspire/Brainco 링 브래킷 CAD 위치, 내부 파라미터 = Unitree G1-D 손목 실측 | Inspire WBT 데이터의 손목은 D405가 아님(IR 0). 같은 계열 광각 카메라의 유일한 공개 보정 |
| D28 | 공식 USD `*_hand_camera_base_link` 시각·충돌체 비활성, 실제 브래킷 시각 메시 추가 | 실제 Inspire 로봇에 없는 10 cm 브래킷이 헤드 영상과 접촉에 영향 |
| D29 | RTX 렌즈 속성에 p1 부호 반전·주점 +0.5 px | Isaac Sim 5.1 규약 실측 (lens_check) |
| D30 | 녹화 순간 카메라 버퍼 강제 갱신 | 공식 이미지 서버의 별도 읽기 때문에 생길 수 있는 1 스텝 지연 제거 |
| D31 | 손목 기본 = Unitree D405 마운트 C + RealSense D405 (2026-11-12) | 사용자 결정. D405 형상 = realsense-ros 공식 메시(Apache-2.0), 내부 파라미터 = Unitree Dex1 데이터의 실측 D405 중앙값 장비 |
| D32 | C 자세 = CAD 정위치 (포켓 안 D405, 키 맞물림 φ 90°, 반링이 손 플랜지에 닿는 x 36.5 mm, 하우징 축) | 다른 회전은 하우징과 3.7 mm 겹쳐 장착 불가. Unitree Dex3 실데이터는 약 18°/9° 다른 장착을 가리키지만, 이 브래킷으로는 만들 수 없음 |
| D33 | 헤드 (0.0587, 0, 0.4310), 47.4° (재맞춤) | 피팅 도구 실루엣 채우기 버그 수정 후 같은 14프레임으로 다시 맞춤 (3.84 → 3.48 px) |
| D34 | 공식 `hand_camera_base_link`는 Dex3 단안 링 브래킷으로 정정 | Dex3 링 STEP과 ICP 0.003 mm 일치 |
| D35 | 손 = Inspire RH56E2 (2026-11-12) | 사용자 요청. Inspire가 만든 E2 URDF(2026-01-05)가 E2 매뉴얼의 엄지 범위와 일치(Unitree FTP URDF는 이전 엄지). 공식 USD를 sublayer로 두고 손 링크를 제자리에서 덮어써 이름·순서·DDS·데이터 형식 유지 |
| D36 | 공식 Inspire DDS·액션 코드는 고치지 않고 프로젝트 확장에서 범위·연동만 바꿔 끼움 | 공식 저장소 패치 최소화. 구조가 바뀌면 시작 시 오류로 멈춤 |
| D37 | 몸체는 G1 29자유도 rev 1.0 유지 (G1 EDU Plus 요청) | EDU Plus = 같은 29자유도 몸체. 새 하드웨어 리비전(mode 11–18)은 실물 리비전 미확인, 공식 USD·보행 정책 없음 (HAND_RH56E2.md §5) |
| D38 | E2 스크립트 파지: 서는 위치 (0.64, −0.12), 엄지 미리 회전 0.91 rad, 폐루프 접근 | E2 손이 커서 DFX용 위치·프리셋으로는 원통이 넘어짐. 손-원통 기하 탐색 + 정답 손목 위치 보정 (smoke 전용) |

| D39 | 책상 상판 0.55 m (2026-11-17) | 사용자 요청(15 cm 낮춤). 이전 값은 `--scene-set desk.top_surface_height=0.70` |
| D40 | 자동 보행은 XR 스틱 값으로 명령 (`--loco-source xr`) | 사람이 PICO 스틱을 미는 것과 같은 변환·중재·토픽 경로. 녹화되는 `action.loco_cmd`도 같은 형식 |
| D41 | 정면 유지 + 대각 보행, 정렬 보정은 1 s 후진 후 재접근 | 공식 정책은 제자리 회전과 0.25 s 펄스에 발을 떼지 않음. 걷는 중의 작은 회전·옆 명령은 먹힘(측정) |
| D42 | 서는 위치를 몸 기준 원통 위치(`stand_rel`)로 정의 | IK 여유와 보정 축이 그대로 대응. 11-12 성공 위치는 리셋 후 미끄러짐을 반영한 (0.33, −0.10) |
| D43 | 계획 파일에 `desk_top_z`, 팔을 움직이기 전 물체 정지 높이로 확인 | 파지 기하가 책상 높이마다 다름(0.55 / 0.65 / 0.70 m 계획 분리) |
| D44 | 보행 명령 재발행 50 → 200 Hz | 카메라 없는 시뮬(RTF 1–2)에서 정책이 읽을 때 명령이 비어 기본값으로 돌아가는 것을 측정 |
| D45 | 자동 생성 데이터는 기존 규칙대로 `source=scripted`, `data/smoke/` | 시뮬 정답을 쓰는 스크립트 전문가 데이터. 2026-11-18 사용자 요청으로 0.65 m 성공 10개를 허깅페이스에 업로드·공개(카드에 scripted·시뮬 정답 사용 명시). 원시 데이터 위치는 그대로 |
| D46 | 세 원통은 목표만 리셋으로 지정하고 기존 eval 필드가 목표를 가리키게 함 | 스크립트·녹화·변환 코드를 거의 그대로 쓰고 예전 단일 원통 데이터와 열 구성이 같음. 목표 이름은 `eval.target`·meta·provenance에 남김 |

책상 치수 해석: "1500x70x70"을 **가로 1.5 m × 깊이 0.7 m × 바닥에서 상판 윗면까지 0.7 m**로 해석했습니다. `configs/scene_desk.yaml`의 `desk.width/depth/top_surface_height`에서 바꿀 수 있습니다(`scripts/run_sim.sh --scene-set desk.width=1.2`). 2026-11-17부터 상판 높이는 0.55 m입니다(D39).

## 4. 권한

2026-11-11 14:20 사용자가 `~/.claude/settings.json`에 `envs/claude_permissions_fragment.json` 내용을 병합했습니다(이 작업공간 env의 `pip install`과 `scripts/*` 실행 허용, 자동 모드 판단 기준에 이 작업공간 범위의 허가 추가). 되돌리려면 `~/.claude/settings.json.bak-20261111`로 복원하면 됩니다.

## 5. 디스크

- `/mnt/isaac`: 작업 전 59 GB → 현재 약 36 GB 여유. env 합계 약 25 GB(sim 14 GB 중 11 GB는 IGRIS와 공유하는 하드링크), data/smoke 158 MB, logs 75 MB.
- `/`: 약 3.8 GB 여유. 프로젝트 파일은 `/`에 쓰지 않지만 Isaac Sim이 `~/.cache/ov`(텍스처 캐시, 1.2 GB)에 씁니다.

## 6. 다음 사용자 동작

0. 자동 보행→파지 데이터(공개): 단일 원통 15개 `jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1`, 세 원통 색 지정 90개 `jammin0319/g1_inspire_walk_pick_3color_desk065_scripted_v1`. GR00T N1.7 파인튜닝은 35 GB 이상 GPU 서버에서. 0.55 m에서는 아직 파지가 안 됩니다. 책상 높이(0.65 m면 바로 동작)나 물체·파지 방식 중 무엇을 바꿀지 정해 주세요(AUTO_WALK_GRASP.md §6). 생성은 `scripts/run_sim.sh --headless --scene-set desk.top_surface_height=0.65` 후 `tools/batch_walk_grasp.py -n 20 --record --desk 0.65` → `scripts/validate_dataset.sh data/smoke/<날짜>` → `scripts/convert_dataset.sh --name <이름> --allow-smoke data/smoke/<날짜>`. 녹화 중 에피소드당 약 1.5–2.3분(RTF 0.23–0.26).

1. 키보드 보행·손 조작을 직접 확인: `scripts/run_sim.sh` + `scripts/run_keyboard.sh` (허브 창을 클릭해 포커스).
2. PICO 도착 후: PICO_CONNECTION.md → PICO_PENDING_CHECKLIST.md.
3. 실제 텔레옵 데이터가 모이면 `scripts/validate_dataset.sh` → `scripts/convert_dataset.sh --name <이름> data/raw/<날짜>`.
4. 카메라: 실물 로봇을 쓰게 되면 그 로봇의 헤드 스테레오·D405를 보정해 `configs/camera_rig.yaml`의 `calibration`만 바꾸면 됩니다(CAMERA_RIG.md §9). 실물 C를 단 로봇의 손목 영상이 있으면 `tools/fit_rig_camera.py --free phi ax rx ry rz`로 CAD 정위치와의 차이(Unitree Dex3 데이터 기준 약 18°/9°)를 확인할 수 있습니다.
5. 손: 실물 G1의 손 배색(흰 손가락 DFTP / 검은 손가락 E2)과 하드웨어 리비전(앱 Machine Type)을 알려 주면 `robot.hand_colorway`, 몸체 모델을 맞춥니다(HAND_RH56E2.md §5). E2 손 스크립트 파지는 `record_episode.sh reset --robot-pose 0.64 -0.12 0`.
6. 학습: 변환된 `data/processed/*_lerobot_v21`과 `envs/` 명세를 35 GB 이상 GPU 서버로 옮겨 GR00T 전체 설치(`uv pip install -e .`) 후 파인튜닝.
