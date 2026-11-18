# G1 + Inspire RH56E2 + PICO 텔레옵 → VLA 데이터: Isaac Sim 작업 정리

작성 2026-11-14. 작업공간 `~/g1_inspire_vla_ws` (실체 `/mnt/isaac/g1_inspire_vla_ws`)의 문서(`README.md`, `docs/*.md`)를 바탕으로 정리했습니다.
처음 보는 사람이나 다른 AI가 이 문서만 읽고 질문에 답할 수 있게 쓰는 것이 목적입니다.

결과 표기는 다음 넷으로 구분합니다.
- **PASS**: 이 PC에서 실제로 실행했고 기준을 통과함
- **PASS (모의)**: 모의 XR 입력 또는 스크립트 입력으로 통과. 실제 PICO 검증이 아님
- **PARTIAL**: 일부만 실행함
- **NOT RUN**: 장비나 자원이 없어 실행하지 못함

---

## 0. 요약

- Unitree G1 휴머노이드(29자유도)에 Inspire RH56E2 손 두 개를 단 로봇을 Isaac Sim 5.1 / Isaac Lab에서 돌립니다. 장면은 책상과 원통 물체입니다.
- 하체는 Unitree 공식 보행 정책(ONNX)이 맡습니다. 상체(팔 14개, 손 12채널)는 텔레옵 허브가 DDS로 명령합니다.
- 텔레옵 입력은 PICO 4 Ultra 컨트롤러(WebXR), 키보드, 모의 입력, 스크립트 중에서 고릅니다.
- 시연은 (관측, 실제로 적용된 목표) 쌍으로 녹화합니다. 녹화본은 LeRobot v2.1과 GR00T 메타데이터로 변환하고, GR00T N1.7 로더가 읽는지까지 확인했습니다.
- 확인한 것:
  - 서기, 보행, 팔 IK, 손 개폐, 4카메라 녹화, 데이터 변환·로더
  - 스크립트 파지(접근 → 쥐기 → 7 cm 들어 올리기)
- 확인하지 못한 것:
  - 실제 PICO 장비 연결
  - 사람의 키보드 조작
  - 실제 텔레옵 데이터
  - GR00T 모델 추론과 파인튜닝(이 PC로는 자원 부족)

## 1. 목표와 제약

원 요청서(G1_Inspire_PICO_Sim_Setup)의 요지는 다음과 같습니다.
- 시뮬에서 PICO 텔레옵으로 걷기와 조작을 함께 하는 시연을 모은다.
- 모은 시연을 VLA(GR00T) 학습 데이터로 만든다.
- 모든 항목을 이 PC에서 실제로 구축하고 실행해 검증한다.

지켜야 하는 제약은 다음과 같습니다. 해결책을 제안할 때도 이 제약 안에서 해야 합니다.
- **기존 환경 보호**: ROS 2 Jazzy, SO-ARM 등 다른 연구 프로젝트, 기존 Conda 환경을 손상시키지 않는다.
- **기존 파일 보존**: 기존 파일은 재사용한다. 지우거나 초기화하지 않고, `reset --hard`나 강제 checkout을 하지 않는다.
- **pip 사용**: 해당 env의 Python으로만 설치한다. sudo pip와 시스템 Python 패키지 교체는 금지다.
- **재실행**: 자동 git pull이나 무조건 업그레이드를 하지 않는다.
- **시스템 설정**: 드라이버 제거, OS 업그레이드, 디스플레이 설정 변경을 설치 수단으로 쓰지 않는다.
- **실물 로봇 격리**: 실물 G1·Inspire에는 명령하지 않는다. DDS 도메인과 네트워크를 연구실 실물 로봇 경로와 분리한다. 그렇다고 기본 도메인을 무작정 바꿔 공식 클라이언트 연결을 깨뜨리지도 않는다.
- **정직한 데이터와 결과**:
  - 숨은 attach, 용접, 순간이동으로 성공률을 부풀리지 않는다.
  - 이동 환경에서 base를 고정하거나 숨은 지지대로 받치지 않는다.
  - 시뮬 정답(ground truth) 물체 위치는 평가 라벨로만 쓰고 정책 관측에 넣지 않는다.
  - mock·scripted 데이터는 `data/smoke`와 `source` 메타로 구분한다.
  - 모의 입력 통과를 실제 PICO 검증으로 보고하지 않는다.
- **외부 공개와 인증**: Hugging Face 업로드 등 외부 공개는 자동으로 하지 않는다. 라이선스나 인증이 필요하면 우회하지 않는다.
- **Git 추적 제외**: 데이터, 가중치, 캐시, 개인키, 대용량 로그는 Git에 넣지 않는다.
- **프로세스 종료**: 이 프로젝트가 띄운 프로세스만 PID로 종료한다. 전체 Python 프로세스를 pkill하지 않는다.

## 2. 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04.5, 커널 6.8, X11 |
| CPU / RAM | Intel Core Ultra 7 265K (20 스레드) / 62 GiB. CPU 성능 프로필이 powersave(바꾸려면 sudo가 필요해 그대로 둠) |
| GPU | RTX 4060 8 GB(모니터 연결, 시뮬 기본) + RTX 5060 Ti 16 GB(학습용, `--gpu 5060ti`로 시뮬도 가능). 드라이버 580.178.04 |
| 디스크 | 루트 `/` 48 GB는 거의 가득 참. 프로젝트는 `/mnt/isaac`(98 GB, 약 35 GB 여유)에 있음 |
| ROS 2 | Jazzy가 `~/.bashrc`에서 source됨. 그래서 모든 런처가 `/opt/ros` 관련 변수를 지움 |

**공식 저장소(고정 커밋)**
- `unitreerobotics/unitree_sim_isaaclab`: `IsaacSim5.1.0` 브랜치 `e30c25b`. main은 Isaac Sim 6.2 / Lab 3.0 / Python 3.12로 넘어가서 쓰지 않음. 로컬 패치는 `envs/patches/`에 있음.
- `isaac-sim/IsaacLab`: `80094be`(v2.3.2+7)
- `unitreerobotics/xr_teleoperate`: `817fb00`(televuer `766de45`)
- `unitreerobotics/unitree_sdk2_python`: `814556d`
- `NVIDIA/Isaac-GR00T`: `3b49067`(N1.7, 로더만 사용)
- 에셋: HF `unitreerobotics/unitree_sim_isaaclab_usds` revision `394cf24…`
  - 로봇 USD `g1_29dof_with_inspire_rev_1_0.usd`(관절 53개)
  - 보행 정책 `assets/model/policy.onnx`

**Conda env 4개**(micromamba, 프로젝트 안 `.conda/`. 기존 env와 분리)

| env | Python | 핵심 |
|---|---|---|
| `g1_inspire_sim` | 3.11 | isaacsim 5.1.0(기존 IGRIS env에서 하드링크, 원본 해시 불변), Isaac Lab source 6개 editable, torch 2.7.0+**cu128**(5060 Ti sm_120 때문), unitree_sdk2py, cyclonedds 0.10.2(conda-forge C 라이브러리 0.10.5에 대해 빌드) |
| `g1_inspire_teleop` | 3.10 | pinocchio 3.1, televuer 4.0 / vuer 0.0.60, params-proto 2.13.2, pygame |
| `g1_inspire_data` | 3.11 | lerobot 0.3.3(dataset v2.1), av 18.1(AV1) |
| `g1_inspire_vla` | 3.12 | GR00T 로더 전용(CPU torch, `--no-deps`) |

- 버전 함정:
  - pyarrow 22 이상은 numpy 1.x에서 import가 실패해 21.0.0으로 고정했습니다.
  - params-proto 3.x에서는 vuer가 깨집니다.
  - starlette·fastapi 충돌 1줄은 공식 Isaac Lab 설치에서도 똑같이 나와 허용했습니다.

## 3. 전체 구조

```
sim 프로세스 (g1_inspire_sim)                          hub 프로세스 (g1_inspire_teleop)
 공식 sim_main.py + 프로젝트 확장(G1IV_EXT=1)           입력: pico | mock | scripted | keyboard
  ├ 하체: 공식 보행 정책 policy.onnx (50 Hz)             ├ 공식 televuer 좌표 변환 (head_yaw 기준)
  ├ 팔: rt/lowcmd[15:29]   ◀──────────────────────────── ├ 공식 G1_29_ArmIK → 변화량 제한 → 공식 G1_29_ArmController
  ├ 손: rt/inspire/cmd (12ch, q 0..1, 1=펴기) ◀─────────── ├ OPEN/CLOSE (히스테리시스, 끊김 시 유지)
  ├ 보행 명령: rt/run_command/cmd [vx,vy,wz,h] ◀─────────── ├ 보행 입력은 1개만 (keyboard XOR XR 스틱)
  ├ 카메라: 헤드 스테레오 2 + 손목 D405 2 (640×480, 렌즈 왜곡)  └ rt/g1iv/teleop_meta (원 입력, 제한 전후 목표)
  └ 녹화기: (관측_t, 적용 목표_t)를 시뮬 시간 25 Hz로 저장  ◀── rt/g1iv/control (녹화·리셋)
        DDS 도메인 1 (공식 시뮬 값), 인터페이스 lo (연구실 망으로 나가지 않음)
```

**관절별 제어 주체**(관절마다 최종 목표를 쓰는 주체는 하나입니다)

| 관절 | 개수 | 누가 정하나 | 주기 |
|---|---|---|---|
| 다리 | 12 | 공식 보행 정책(출력 × 0.25 + 기본자세) | 50 Hz (물리 200 Hz, decimation 4) |
| 허리 | 3 | 기본자세 고정 | – |
| 팔 | 14 | 허브: IK → 변화량 제한 → DDS `rt/lowcmd` | 허브 50 Hz, lowcmd 250 Hz |
| 손 구동 관절 | 12 | 허브: OPEN/CLOSE 프리셋 → 속도 제한 → `rt/inspire/cmd` | 50 Hz |
| 손 연동 관절 | 12 | 시뮬이 구동 관절 × 비율로 계산 | 50 Hz |
| 보행 명령 | 4 | 허브의 LocoArbiter 1개 | 송신 50 Hz. 정책이 읽을 때마다 0으로 초기화 |

- 보행 정책의 관측에는 다리와 팔의 관절 위치·속도, 그리고 29차원 직전 행동이 들어갑니다(10프레임 이력). 따라서 팔 움직임이 보행 정책에 영향을 줍니다. 결합 방식은 공식 코드 그대로입니다.
- 공식 코드를 고친 곳은 최소한입니다. 대부분은 프로젝트 확장(`src/g1iv_sim/ext.py`, `desk_task.py`)이 실행 중에 바꿔 끼웁니다.

## 4. 구성 요소

### 4.1 장면
- 책상: 1.5 × 0.7 m, 상판 윗면 높이 0.7 m
- 원통: r 30 mm, h 120 mm, 0.15 kg
- 바닥: 무반사 검정. 반사 때문에 회색으로 보여 specular를 0으로 둠
- 벽과 천장은 없고, 공식 창고 USD와 포장 테이블은 뺐습니다.
- 설정은 `configs/scene_desk.yaml`이고, `--scene-set key=value`로 바꿉니다.

### 4.2 로봇 몸체: G1 29자유도 rev 1.0 (G1 EDU Plus 요청은 몸체 유지)
- G1 EDU Plus는 허리 3자유도와 팔 7자유도(손목 3자유도)를 갖는 29자유도 EDU 몸체입니다. 공식 사양표 기준으로 지금 모델과 관절, 축, 범위, 링크 길이가 같습니다. "EDU Plus"라는 별도 공식 모델 파일은 없고, Unitree는 하드웨어 리비전(`mode_machine`)으로 모델을 나눕니다.
- 2025-06 이후 리비전(mode 11–18)은 지금 모델과 다릅니다.
  - `wrist_yaw` 위치가 46 → 51 mm
  - 총질량 33.3 → 36.7 kg
  - 고관절·손목 토크 한계가 다름
- 하지만 이 리비전용 공식 USD와 보행 정책이 없고, 실물이 어느 리비전인지도 모릅니다(앱 Device → Data → Robot → Machine Type에서 확인 가능). 그래서 rev 1.0을 유지했습니다.

### 4.3 손: Inspire RH56E2 (= RH56DFTP 계열)
- **같은 제품 근거**: E2 매뉴얼 §2.6.21에 DFTP 품번이 나오고, Inspire CN 사이트의 E2 페이지 경로가 `rh56dftp-series`입니다.
- **형상 출처**: Inspire가 만든 E2 오른손 URDF(2026-01-05)를 썼습니다. 다만 공식 사이트가 아니라 제3자 GitHub 저장소(`hychen-naza/isaaclab_env` @ cccdf11, 패키지 안 Inspire 저작권 LICENSE)에서 받았습니다.
  - 진위는 엄지 범위가 E2 매뉴얼과 맞는지로 판단했습니다(회전 95°, 굽힘 35.5°).
  - Unitree의 FTP URDF는 이전 엄지 설계라 맞지 않았습니다.
  - 파생 메시는 내부 연구용으로만 쓰고 Git에 넣지 않습니다.
- **장착**: Unitree FTP와 같이 `wrist_yaw_link` x = 41.5 mm 손목 플랜지에 붙입니다. E2 베이스 좌표계는 손 축 둘레로 90° 돌아가 있습니다. FTP 베이스 메시와 겹치는 정도는 중앙값 0.05 mm, p95 0.32 mm입니다.
- **왼손**: 오른손을 y 방향으로 거울 반전해 만들었습니다. Inspire 왼손 URDF와 중앙값 0.001 mm로 일치합니다.
- **질량**: 0.899 kg/손(Inspire URDF. 사양 790 g에 플랜지 어댑터 포함). 이전 공식 손은 0.54 kg입니다.
- **관절 범위**(q 1=펼침 ↔ 각도 하한, 선형 대응)

| 채널 | RH56E2 (지금) | 공식 DFX 손 (이전) |
|---|---|---|
| 손가락 4개 (proximal) | 0–1.4381 rad | 0–1.7 |
| 엄지 굽힘 (pitch) | 0–0.62 rad (35.5°) | 0–0.5 |
| 엄지 회전 (yaw) | 0–1.658 rad (95°) | −0.1–1.3 |

- **연동 비율**(Inspire URDF `<mimic>`)
  - 손가락 intermediate = 1.0843 × proximal
  - 엄지 intermediate = 0.8392 × pitch
  - 엄지 distal = 0.891 × 엄지 intermediate(= 0.74773 × pitch)
  - 이전 공식 손은 1.0 / 1.5 / 2.4였습니다.
  - 선형 근사이고, 실제 링크 기구의 비선형성은 반영하지 않았습니다.
- **USD 만드는 방법**: 공식 G1 USD를 sublayer로 두고, 손 링크 26개를 제자리에서 덮어씁니다. 기존 시각·충돌체를 끄고, E2 메시와 링크별 convex hull, 질량, 관절 좌표계, 한계를 넣습니다.
  - 관절·링크 이름과 순서는 공식과 같습니다. 그래서 공식 DDS, 상태 발행의 고정 인덱스, 데이터 형식이 그대로 동작합니다.
  - 관절 좌표계 불일치는 위치 0.000 mm, 회전 ≤ 0.022°입니다.
- **공식 코드 대응**: 공식 `inspire_dds`의 normalize/denormalize는 DFX 범위가 하드코딩돼 있습니다. 공식 액션 프로바이더에는 연동 비율 텐서가 있습니다. 둘 다 파일은 고치지 않고 시작할 때 E2 값으로 바꿔 끼웁니다(`src/g1iv_common/hand_model.py`). 구조가 바뀌어 바꿔 끼우지 못하면 시작 단계에서 오류로 멈춥니다.
- **배색**
  - 라벨 6가지를 면 단위로 나눴습니다: 검정, 흰 껍데기, 손등 덮개, 촉각 패드, 금속, 표시등.
  - 기본 배색 `dftp`(Inspire 사진 기준): 손등 덮개 흰색, 하우징 검정, 손가락 등 흰색, 손가락 끝 캡 18.5 mm 검정, 손바닥 쪽과 패드 검정, 힌지와 플랜지 은색입니다.
  - 대안 배색 `e2_black`: 손가락도 검정입니다(E2 제품 렌더).
  - 로고 글자는 넣지 않았습니다.
- **선택 스위치**: `robot.hand_model = rh56e2 | inspire_dfx_official`, `robot.hand_colorway = dftp | e2_black`

### 4.4 카메라 리그 (정책·녹화 카메라 4대, `configs/camera_rig.yaml`)

| 카메라 (LeRobot 키) | 실제 장비 | 시뮬 장착 | 내부 파라미터 출처 |
|---|---|---|---|
| 헤드 왼눈 `cam_left_high` | Unitree 60 FPS 스테레오 모듈(대각 125°, 기준선 60 mm) + 헤드 브래킷 | `head_link`, 몸통 기준 (0.0587, 0, 0.4310) m 중심, 47.4° 아래 | Unitree G1_Dex1_Stack_Block 실측(공개 11대 중 fx 중앙값 장비) fx 273.7 |
| 헤드 오른눈 `cam_right_high` | 같은 모듈 오른쪽 | 왼눈 + 실측 외부 파라미터(기준선 58.86 mm) | 같은 장비 |
| 왼/오른 손목 `cam_left_wrist`, `cam_right_wrist` | Intel RealSense D405 + Unitree D405 손목 마운트 "C"(xr_teleoperate STEP) | `*_wrist_yaw_link`, 손목 위 85 mm, CAD 포켓 안 정위치. 광축이 손 축에서 31.6° | Unitree Dex1 데이터의 실측 D405 중앙값 장비 fx ≈ 434 |

- **헤드 자세**: 실제 Unitree G1_WBT_Inspire 데이터 14프레임에서 로봇 실루엣과 영상 에지를 맞춰 정했습니다(Nelder–Mead). 비용은 3.48 px이고, 공식 프리셋은 9.23 px입니다.
  - 팔 기구학은 실제 데이터의 EE와 RMS 1.2 mm로 일치합니다. 따라서 영상 차이는 카메라 장착에서만 생깁니다.
- **손목 자세**: CAD로 정했습니다. 마운트 안쪽 평면이 손목 하우징 평면에 맞물리는 키 각도 φ = 90°만 가능하고, 다른 각도는 하우징과 3.7 mm 겹칩니다.
  - 그런데 Unitree의 실제 D405 손목 데이터(G1_Dex3_BlockStacking, Dex3 손)는 CAD보다 손목 축 둘레 약 18°, 화면 좌우 약 9° 더 돌아간 장착과 맞습니다.
  - C 마운트를 G1 + Inspire에 단 실제 영상은 공개 자료에 없어 기본값은 CAD로 두었습니다.
- **렌즈**: RTX 카메라의 OpenCV 핀홀 왜곡 모델을 씁니다.
  - Isaac Sim 5.1은 p1 부호가 반대이고 주점이 0.5 px 어긋나는 것을 실측해 보정했습니다.
  - 35점 검사 오차 중앙값은 0.17–0.37 px입니다.
- **하드웨어 형상**: 공식 USD의 `*_hand_camera_base_link`(실제로는 Dex3 단안 링 브래킷)는 끄고, 실제 브래킷과 D405 시각 메시(realsense-ros 공식, Apache-2.0)를 붙였습니다.
- **이전 손목**(Inspire 링 + 160° 단안 광각): `--rig-set wrist.mount=inspire_ring_mono160`으로 남아 있습니다. Unitree가 G1+Inspire 데이터에 실제로 쓴 구성은 이쪽입니다.
- **공개 데이터 조사에서 확인한 사실**
  - UnifoLM-VLA 학습 데이터는 G1 + Dex1 그리퍼(손목 D405)입니다.
  - Unitree Inspire WBT 데이터의 `wrist_*_ir`는 전부 0입니다. 즉 그 손목 카메라는 D405가 아닙니다.

### 4.5 텔레옵 허브 (`src/g1iv_teleop/`)
- **입력 소스**: `pico`(televuer WebXR 컨트롤러 모드), `keyboard`(pygame, 허브 창 포커스 시에만), `mock`(시나리오: dry_run, dropout, recenter, walk_xr, walk_seq, hold), `scripted`
- **팔**
  - 상대 매핑: engage·recenter·clutch 해제 시점의 EE와 컨트롤러 자세를 기준으로 잡습니다. 그 뒤 `p = p_ee0 + s·Δp_ctrl`, `R = ΔR_ctrl · R_ee0`로 계산합니다.
  - IK 프레임은 pelvis에 붙어 있어 걸으면 목표도 함께 움직입니다.
  - IK 실패(오차 > 3 cm, NaN)면 직전 목표를 유지합니다.
  - 목표는 연결 시점 기준 0.45 m 이내로 자릅니다.
  - 관절 속도는 시뮬 시간 기준 1.5 rad/s로 제한합니다.
- **손**: 트리거 OPEN/CLOSE(히스테리시스 0.6 / 0.4). 전이는 50 ms 유지되고 새 포즈가 도착해야 확정합니다. 트래킹이 끊기는 순간의 "트리거 0"이 손을 여는 것을 막기 위해서입니다.
- **보행**
  - 공식 정책은 작은 명령에 발을 떼지 않습니다. 측정 결과 |vy| ≥ 0.5 m/s, wz ≥ 1.2 rad/s 정도는 돼야 걷습니다.
  - 그래서 키와 스틱 최대값을 공식 키보드 범위로 뒀습니다. 키는 전진 0.5 / 후진 0.6 / 횡 0.5 / 회전 1.57이고, XR 스틱은 vx 0.6 / vy 0.5 / wz 1.57입니다.
  - XR 0.3 s 무입력이면 보행 명령을 0으로 보냅니다.
- **공식 코드 함정 대응**
  - 공식 `G1_29_ArmController`는 생성되자마자 목표 0을 발행합니다(팔 순간이동). 허브가 발행 스레드를 붙잡아 두고 측정 자세로 맞춘 뒤 풀어 줍니다.
  - 허브는 하나만 뜨게 막았습니다. 공식 키보드·텔레옵 스크립트도 같은 토픽에 쓰므로 동시에 실행하지 않아야 합니다.
- **PICO 버튼(설정만 해 둠, 실기 미확인)**
  - 왼쪽 X: 연결/해제, 왼쪽 Y: recenter
  - grip: clutch, trigger: 손
  - 오른쪽 A: 녹화 시작 / 성공 종료, 오른쪽 B: 실패 종료
  - 양 스틱 클릭: 비상 정지
- **접속**: `https://<PC_IP>:8012/?ws=wss://<PC_IP>:8012`. 로컬 루트 CA와 서버 인증서는 `certs/`에 있고 Git에서 제외합니다.

### 4.6 녹화와 데이터
- **시간 격자**: 녹화는 시뮬 시간 25 Hz로 균일합니다(50 Hz 제어 격자에서 2스텝마다). 시뮬이 느리므로 실제 시간으로는 약 8 Hz입니다.
- **녹화 위치**: 녹화는 시뮬 프로세스 안에서 합니다. 공식 이미지 서버에는 프레임 번호와 시각이 없고 같은 프레임을 다시 보내기 때문입니다.
- **프레임 k의 뜻**: `obs`는 제어 스텝 s_k가 실행되기 직전 상태이고, `action`은 그 스텝에 실제로 적용된 목표입니다(허브의 제한을 거친 뒤 DDS로 들어온 값). 기록 순간 카메라 버퍼를 강제로 갱신해 1스텝 지연을 없앴습니다.
- **raw 형식**(`g1iv-raw-1`): `meta.json`, `steps.jsonl`, `arrays.npz`, `images/<cam>/*.jpg`(640×480), `teleop_meta.jsonl`
  - `eval.*`(정답 물체 위치, 들림 높이, 접촉력, 넘어짐)은 평가 라벨 전용입니다.
- **LeRobot v2.1**(GR00T N1.7이 v2를 읽음). 이름은 unitree_lerobot의 `Unitree_G1_Inspire` 규약을 따릅니다.
  - `observation.state` 26 = [왼팔 7, 오른팔 7, 왼손 6, 오른손 6]. 팔은 rad, 손은 0..1
  - `action` 26: 같은 순서의 적용 목표. 다리는 넣지 않음
  - `action.loco_cmd` 4: 보행 명령. `modality.json`에서 `base_command`(vx, vy, wz)로 action에 포함
  - 영상 4개(AV1). 언어 지시는 `task_index`
  - `meta/info.json`의 `camera_params`는 Unitree 데이터셋과 같은 배치(K, dist, 스테레오 R/T)입니다.
- **확인 결과**: 검증기(25.00 Hz 균일, 드롭 0, stale 영상 0), lerobot 로더 배치, GR00T `LeRobotEpisodeLoader`(그룹 차원, AV1 디코딩, 16스텝 action chunk) 모두 통과했습니다.
- **현재 데이터는 전부 smoke**(스크립트 파지, `source=scripted`, `is_smoke_data=true`)입니다.
  - raw 에피소드: 2026-11-11 9개, 2026-11-12 8개
  - 변환본: `smoke_v1`, `smoke_rig4_v1`, `smoke_rig4_v2`, `smoke_d405c`(각각 `_lerobot_v21`)
  - 사람 텔레옵 데이터는 0개입니다.

### 4.7 스크립트 파지 (파이프라인 시험용, 학습 데이터 아님)
- 시뮬 정답 물체 위치로 접근 → 쥐기 → 들어 올리기 경로를 계획합니다. 로봇은 책상 앞에 리셋하고 걷지 않습니다.
- E2 손은 DFX 손보다 큽니다. 그래서 DFX용 위치와 프리셋으로는 엄지가 원통을 쳐서 넘어뜨렸습니다. 손 모델별로 다음을 따로 정했습니다.
  - 서는 위치: (0.64, −0.12)
  - 엄지 미리 회전: q 0.45 = 0.91 rad
  - 쥠 프리셋: 손가락 0.15, 엄지 굽힘 0.65
  - 손바닥 쪽(+y)에서 곧게 들어오는 경로
- **폐루프 접근**: 접근 후반에 정답 손목 위치로 위치 오차를 적분해 보정하고, 5 mm 안에 들면 쥡니다. 들어 올리는 동안에는 수직 보정을 풉니다. 풀지 않으면 IK 잔차가 쌓여 들림이 4.0 cm에 그쳤습니다.
- **결과**: E2 손은 헤드리스 3/3과 GUI 1회 성공(약 6.9–7.0 cm, 기준 5 cm, 넘어짐 없음)입니다. 공식 DFX 손 회귀는 (0.70, −0.05)에서 7.2 cm 성공입니다.
- **실행 명령**: `scripts/record_episode.sh reset --robot-pose 0.64 -0.12 0` → `scripts/run_mock_teleop.sh scripted [--record]`

## 5. 주요 결정 (공식 구성과 다른 점)

| 결정 | 이유 |
|---|---|
| torch cu128 | 공식 cu126은 RTX 5060 Ti(sm_120) 미지원 |
| Isaac Sim 5.1 하드링크 재사용 | 같은 버전이 이미 있음. 원본 env 해시 불변 확인 |
| DDS 인터페이스 `lo`, 도메인 1 유지 | 연구실 실물 로봇 망과 분리하면서 공식 클라이언트 설정은 유지 |
| 녹화 25 Hz(시뮬 시간), 시뮬 프로세스 안에서 | 50 Hz 정책 격자. 공식 이미지 서버에 프레임 번호가 없음 |
| 허브 속도 제한과 스크립트 계획을 시뮬 시간 기준으로 | RTF 약 0.35. 실제 시간 기준이면 시뮬에서 3배 빠르게 움직임 |
| 키보드는 허브 창 포커스 시에만 | 다른 창 타이핑으로 로봇이 움직이는 위험 제거 |
| Inspire는 DFX 인터페이스 + 자체 OPEN/CLOSE 어댑터 | 공식 xr_teleoperate가 Inspire + 컨트롤러 조합을 거부 |
| LeRobot v2.1 + `modality.json`, action에 `base_command` | GR00T N1.7 호환. 걷기+조작 과제라 보행 명령도 행동에 포함 |
| vla env는 로더 전용 | 이 PC는 학습용이 아님. N1.7 추론 ≥ 16 GB, 파인튜닝 약 35 GB/GPU |
| 카메라를 공식 프리셋 대신 리그로 | 공식 Inspire 손목 프리셋은 Dex3 값을 복사해 무릎을 찍고 있었음 |
| 손목 = D405 + 마운트 C, CAD 정위치 | 사용자 결정. 실데이터와 맞는 각도는 이 브래킷으로 장착할 수 없음 |
| 손 = RH56E2, 공식 USD 위에 제자리 덮어쓰기 | 사용자 요청. 이름·순서·DDS·데이터 형식 유지 |
| 공식 Inspire 코드는 고치지 않고 실행 중 바꿔 끼우기 | 공식 저장소 패치 최소화. 구조가 바뀌면 시작 시 오류 |
| 몸체 rev 1.0 유지 | EDU Plus = 같은 몸체. 새 리비전은 공식 USD·정책 없음 |
| 런처가 공식 시뮬의 종료 코드 대신 로그로 판정 | 공식 `sim_main.py`는 시작 실패도 종료 코드 0으로 끝내고, 종료 시 `simulation_app.close()`에서 멈춤 |

## 6. 검증 현황

| 항목 | 상태 | 근거 요약 |
|---|---|---|
| 런처로 올바른 env에서 시뮬 시작 | PASS | 헤드리스·GUI 모두 기동(캐시 후 20–32 s) |
| 장면 구성 (G1 + 양손, 책상, 검정 바닥, 물체) | PASS | 캡처 |
| 보행 정책으로 서기 | PASS | 공식 장면 5분 이상 기울기 0.71°, 이후 모든 실행에서 넘어짐 없음 |
| 키보드 보행 | PARTIAL | 같은 명령 경로를 모의 스틱과 직접 명령으로 확인(전진 1.05 m, 좌회전 +150° 등). 사람이 키를 누르는 시험은 안 함. 최대 회전 명령에서도 5회 중 2회는 회전이 시작되지 않음 |
| 좌우 손 OPEN/CLOSE | PASS (모의) | 정규값이 실제 관절에 반영됨 |
| 모의 컨트롤러 → 공식 IK → 팔 | PASS (모의) | IK 실패 0, 위치 오차 ≤ 7.3 mm, 추종 오차 중앙값 5.5 mrad |
| 걷는 중 팔 입력 | PASS (모의) | 1.42 m 걷는 동안 손목 이동·손 개폐, IK 실패 0, 넘어짐 없음 |
| 입력 끊김, recenter, 재개 | PASS (모의) | 2 s 동결 중 팔 변화 0.0017 rad, 손 유지, 재개 시 점프 없음 |
| 접근 → 쥐기 → 들어 올리기 | PASS (스크립트) | §4.7 |
| 녹화 → 검증 → 변환 → 로더 | PASS | §4.6 |
| 카메라 정합과 렌즈 | PASS (손목은 CAD 기준, 실데이터와 부분 일치) | §4.4 |
| RH56E2 손 자산 검사 | PASS | 장착 0.05 mm, 거울 0.001 mm, 관절 좌표계 0.000 mm / 0.022° |
| 단위 테스트 | PASS | 38개 |
| 실제 PICO | NOT RUN (장비 없음) | 확인 항목 10개는 `docs/PICO_PENDING_CHECKLIST.md` |
| GR00T 모델 smoke / 학습 | NOT RUN (자원) | 로더까지만 통과. 가중치는 받지 않음 |

## 7. 성능 (헤드리스, 서 있는 상태)

| 조건 | RTF (sim/wall) | 제어 주기 (실제 시간) | GPU 메모리 |
|---|---|---|---|
| 4060, 카메라 3대 | 0.366 | 18.3 Hz | 2.8 GB |
| 5060 Ti, 카메라 3대 | 0.420 | 21.0 Hz | 2.0 GB |
| 4060, 카메라 제외 | 0.371 | 18.5 Hz | 2.8 GB |
| 4060, 렌더링 끔 | 0.306 | 15.3 Hz | 3.2 GB |
| 4060, GPU 물리(실험) | 0.291 | 14.5 Hz | 4.7 GB |
| 4060, 카메라 4대 + 렌즈 (현재 리그) | 0.329 | – | – |

- 카메라와 렌더링을 빼도 빨라지지 않습니다. 병목은 CPU 쪽입니다(PhysX CPU 물리, 정책, Python 루프. 시뮬 CPU 약 520 %).
- 녹화 중 RTF는 0.27–0.29입니다. 사람이 텔레옵하면 로봇이 손보다 약 3배 느리게 따라옵니다.

## 8. 알려진 문제와 한계

1. **접촉력 라벨이 항상 0**: 손-물체 필터 접촉 센서 값(`eval.contact_force_total`)이 쥐고 들어 올리는 중에도 0입니다. 원인은 찾지 못했습니다. 성공 판정은 정답 들림 높이로 하므로 영향은 없지만, 이 열은 지금 쓸모가 없습니다. E2 손으로 바꾸기 전부터 있던 문제입니다.
2. **공식 보행 정책의 불감대**: 작은 명령에는 발을 떼지 않고 몸만 흔들립니다(횡 0.35 m/s, 회전 0.8 rad/s, 후진 0.26 m/s). 최대 회전 명령에서도 회전이 시작되지 않는 경우가 있습니다(5회 중 2회). 정책을 다시 학습하지 않고 명령 크기만 키워 대응했습니다.
3. **느린 시뮬**: RTF 0.3–0.4이고 CPU가 병목입니다. CPU 성능 프로필은 powersave이고 sudo가 필요해 바꾸지 않았습니다.
4. **손목 D405 장착 각도의 불확실성**: CAD 정위치와 Unitree 실데이터(Dex3 손)가 약 18° / 9° 다릅니다. 실물 C를 단 로봇의 손목 영상 몇 장이면 `tools/fit_rig_camera.py --free phi ax rx ry rz`로 확인할 수 있습니다.
5. **헤드 내부 파라미터**: 같은 모델의 다른 장비 값을 썼습니다. 장비마다 fx가 186–296으로 다르므로 실물을 쓰게 되면 보정해야 합니다.
6. **RH56E2 자산**
   - 출처가 제3자 저장소입니다.
   - 연동 비율은 선형 근사입니다.
   - 촉각 센서 17개는 형상만 있고 출력이 없습니다.
   - 배색 경계는 사진으로 정했습니다(몇 mm 오차 가능).
   - 실물 DFTP 제어 토픽은 `rt/inspire_hand/ctrl/{l,r}`(0–1000)이지만, 시뮬은 공식 DFX 인터페이스 `rt/inspire/cmd`(0–1)를 씁니다. 실물로 옮길 때는 xr_teleoperate `--ee inspire_ftp`를 쓰면 됩니다.
7. **스크립트 파지는 정답 위치를 씁니다**: 파이프라인 시험용일 뿐 학습 데이터가 아닙니다. 서는 위치를 정해 둔 경우에만 동작하고, 보행하지 않습니다.
8. **PICO 헤드셋 영상은 단안**: 공식 이미지 서버는 헤드 왼눈만 보냅니다. 입체 표시(side-by-side)는 구현하지 않았습니다.
9. **종료 멈춤(공식 동작)**: 정리가 끝난 뒤 `simulation_app.close()`가 돌아오지 않습니다. 런처가 5초 뒤 종료시키고 정상 종료로 보고합니다.
10. **공유 메모리**: 공식 시뮬이 고정 이름의 공유 메모리를 쓰므로 시뮬은 한 번에 하나만 띄울 수 있습니다.
11. **루트 디스크 여유가 거의 없음**: Isaac Sim 텍스처 캐시가 `~/.cache/ov`(1.2 GB)에 쌓입니다.

## 9. 남은 일

1. 실제 PICO 4 Ultra 연결(`docs/PICO_CONNECTION.md` → `docs/PICO_PENDING_CHECKLIST.md` 10개 항목)
   - 같은 망 접속, 인증서, 축 방향, 실제 버튼 필드명, 끊김 처리, 지연, RTF 0.3에서 조작 가능한지
2. 사람의 키보드·PICO 텔레옵으로 실제 시연 데이터 수집 → 검증 → 변환
3. GR00T N1.7 파인튜닝: 변환 데이터와 env 명세를 35 GB 이상 GPU 서버로 옮겨 전체 설치 후 학습
4. 실물 정보로 맞추기: 손 배색(흰 손가락 DFTP / 검은 손가락 E2), G1 하드웨어 리비전, 헤드·D405 보정값

## 10. 다른 AI에게 의견을 구할 만한 열린 질문 (참고용)

- **접촉 센서**: Isaac Lab 2.3 / Isaac Sim 5.1에서 로봇 손 링크와 물체 사이 필터 접촉 센서(`filter_prim_paths_expr`)가 실제로 쥐고 있을 때도 0을 내는 흔한 원인은 무엇인가?
  - 참고: 손 충돌체는 USD에서 기존 충돌체를 끄고 새 convex hull로 바꾼 것이고, 접촉 센서 문제는 공식 손일 때도 있었습니다.
- **보행 정책 불감대**: 공식 wholebody 보행 정책의 불감대와 회전 시작 실패를 재학습 없이 줄이는 방법이 있는가? 예: 명령 성형, 시작 펄스. 재학습한다면 unitree_rl_lab 기준으로 무엇을 바꿔야 하는가?
- **시뮬 속도**: PhysX CPU 물리 + 50 Hz ONNX 정책 + 카메라 4대에서 RTF 0.3–0.4다. sudo 없이 시도할 수 있는 개선은? GPU 물리는 오히려 느렸다(0.29).
- **E2 연동 비율**: E2 손가락 연동을 선형 비율로 둔 것이 파지 학습이나 sim-to-real에 주는 영향은? 비선형 연동을 Isaac Lab에서 넣는 현실적인 방법은?
- **행동 공간 설계**: GR00T N1.7 파인튜닝에서 걷기+조작 데이터의 action에 `base_command`(vx, vy, wz)를 넣는 설계가 적절한가? 상태 26차원, 영상 4개, 25 Hz, 16스텝 chunk 기준이다.
- **손목 카메라 각도**: CAD 정위치와 실데이터 적합값이 18° 다를 때 어느 쪽을 학습 데이터 기본으로 삼는 것이 sim-to-real에 유리한가? 카메라 자세를 무작위화(domain randomization)하는 범위는 어느 정도가 적당한가?
- **새 하드웨어 리비전**: G1 mode 11–18로 몸체를 옮길 때 rev 1.0용 보행 정책을 그대로 써도 되는가(질량 +3.4 kg, 손목 +5 mm, 토크 한계 다름)?

## 11. 파일 지도

| 경로 | 내용 |
|---|---|
| `README.md` | 구조, 명령, 키 매핑 |
| `docs/SETUP_REPORT.md` | 상태 요약, 진행 기록, 결정 D1–D38, 다음 동작 |
| `docs/VALIDATION.md` | 항목별 실행 결과와 근거 로그 |
| `docs/VERSION_MATRIX.md` | 설치된 버전, 고정 이유 |
| `docs/CONTROL_OWNERSHIP.md` | 관절별 제어 주체, 좌표계, 실패 처리 |
| `docs/DATASET_SCHEMA.md` | raw와 LeRobot 형식 |
| `docs/CAMERA_RIG.md` | 카메라 조사, 정합, 렌즈 |
| `docs/HAND_RH56E2.md` | 손 자산, 배색, EDU Plus 판단, 파지 |
| `docs/PICO_CONNECTION.md`, `docs/PICO_PENDING_CHECKLIST.md` | PICO 접속 방법, 실기로만 확인할 항목 |
| `configs/` | `scene_desk`, `camera_rig`, `hand_rh56e2`, `hands_inspire`, `teleop`, `record`, `scripted_grasp` |
| `src/g1iv_sim/` | 시뮬 확장(장면, 카메라 리그, 녹화기, 상태 발행) |
| `src/g1iv_teleop/` | 허브(입력 소스, IK, 손, 보행 중재) |
| `src/g1iv_data/` | 검증, 변환, 로더 확인 |
| `src/g1iv_common/` | 공용(손 모델, 손 채널, 설정) |
| `tools/`, `tools/cad/` | 정합, 렌즈 검사, CAD → 자산 도구 |
| `scripts/` | `run_sim.sh`, `run_keyboard.sh`, `run_mock_teleop.sh`, `run_pico_teleop.sh`, `record_episode.sh`, `validate_dataset.sh`, `convert_dataset.sh`, `stop_all.sh` |
