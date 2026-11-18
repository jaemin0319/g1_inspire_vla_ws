# CAMERA_RIG — Unitree G1 + Inspire 데이터 수집 카메라 구성의 시뮬 반영

작성 2026-11-11, 개정 2026-11-12(손목 = Unitree D405 마운트 "C" + RealSense D405, 헤드 재맞춤). 설정 `configs/camera_rig.yaml`,
기하 계산 `src/g1iv_sim/camera_rig.py`, 장면 반영 `src/g1iv_sim/desk_task.py`, 렌즈 적용 `src/g1iv_sim/ext.py`.
산출물(영상·로그)은 `logs/camera_rig/`에 있고, 내려받은 원자료는 `data/reference/`에 있습니다(git 제외).

## 1. 결론: 시뮬에 들어간 카메라

| 카메라 (LeRobot 키) | 실제 장비 (Unitree 공식 목록) | 시뮬 장착 링크 · 광학 중심 [m] · 방향 | 640×480 내부 파라미터 | 화각 (왜곡 반영, H×V / 대각) |
|---|---|---|---|---|
| 헤드 왼눈 `cam_left_high` | 60 FPS 스테레오 RGB 모듈, 125° FOV(대각), 기준선 60 mm, 3D 프린트 헤드 브래킷 (xr_teleoperate Device.md 5.2.1) | **머리**(`head_link`) (0.0550, +0.0294, 0.4748) = 몸통 기준 (0.0590, +0.0294, 0.4308), **47.4° 아래**, 좌우 대칭 | fx 273.66, fy 273.88, cx 338.23, cy 268.34, k1 0.0578 k2 −0.0616 p1 0.0069 p2 0.0013 k3 0.0140 | 98.9°×81.6° / 112° |
| 헤드 오른눈 `cam_right_high` | 위 모듈의 오른쪽 렌즈 | 왼눈 + 실측 외부 파라미터 (기준선 58.86 mm, 안쪽으로 0.93°) | fx 275.09, cx 384.27, cy 281.02 (같은 장비 보정값) | 97.7°×81.1° / 110° |
| 왼손목 `cam_left_wrist` | **Intel RealSense D405**(컬러 = 왼쪽 이미저) + **Unitree D405 손목 마운트 "C"**(xr_teleoperate hardware 7cd188c, Dex3-1용 STEP) | `left_wrist_yaw_link` (0.0282, −0.0086, 0.0848): 손목 위(+z) 85 mm, 브래킷 CAD 포켓 안 정위치. 광축 (0.852, −0.310, −0.423) = 손 축에서 31.6° | fx 433.85, fy 433.32, cx 322.36, cy 242.78, k1 −0.052 k2 0.058 p1 −0.0008 p2 0.0002 k3 −0.018 (Unitree D405 실측, 중앙값 장비) | 72.8°×58.0° (왜곡 작음) |
| 오른손목 `cam_right_wrist` | 같은 카메라·마운트 (마운트는 거울 부품, D405는 같은 부품) | `right_wrist_yaw_link` (0.0282, +0.0086, 0.0848), 광축 (0.852, +0.310, −0.423) | fx 434.36, fy 433.81, cx 323.70, cy 243.15, k1 −0.053 … (중앙값 장비) | 72.8°×57.9° |

- 헤드 카메라는 머리 링크(`head_link`)에 붙습니다. G1 머리는 목 관절 없이 몸통(`torso_link`)에 고정된 링크라, 설정 파일의 위치는 몸통 기준으로 적고 머리 링크로 옮겨 붙입니다(`mount_link`). 광학 중심은 얼굴 개구부 위쪽, 머리 내장 D435 자리와 같은 높이입니다.
- 헤드 왼눈 영상은 공식 이름 `front_camera`를 그대로 써서 공식 이미지 서버(텔레옵 헤드셋 영상)도 이 카메라를 보냅니다.
- 손목은 2026-11-12부터 **C + D405**입니다(사용자 선택). 마운트·반링·D405 형상을 손목에 붙였고, 카메라는 D405가 브래킷 포켓에 앉는 CAD 위치 그대로입니다(§3.3). 공식 USD의 `*_hand_camera_base_link`(Unitree Dex3 단안 링 브래킷, §2)는 껐습니다. 헤드에는 스테레오 보드(검은 상자)와 렌즈 두 개를 시각용으로 놓았습니다.
- 이전 손목 모델(A: Inspire/Brainco 링 + 160° 단안)은 남아 있습니다: `scripts/run_sim.sh --rig-set wrist.mount=inspire_ring_mono160`.
- 렌즈 왜곡은 RTX 카메라의 OpenCV 핀홀 모델로 실제처럼 렌더링합니다(§5).
- `use_rig: false`(또는 `scripts/run_sim.sh --rig-set use_rig=false`)이면 공식 프리셋(헤드 단안, `d435_link`)으로 돌아갑니다.
- 2026-11-12에 피팅 도구의 실루엣 채우기 버그(겹친 삼각형이 지워짐)를 고치고 헤드를 다시 맞췄습니다(§3.2). 이전 값 (0.0608, 0, 0.4308), 46.3°와 2.1 mm, 1.1° 차이입니다.

## 2. 무엇을 찾았나 (출처)

### UnifoLM-VLA
- `github.com/unitreerobotics/unifolm-vla` (2026-01-29 공개): 학습 데이터 12종은 모두 **G1 + Dex1(그리퍼)** 데이터(G1_Stack_Block, G1_Bag_Insert, …).
- 데이터 준비 코드(`prepare_data/hdf5_to_rlds/rlds_dataset.py`)는 `cam_left_high`, `cam_right_high`, `cam_left_wrist`, `cam_right_wrist` 네 개를 읽습니다. 정책 입력은 헤드 왼눈 + 양 손목입니다(`configs.py`: primary = left_top).
- 그 데이터의 손목 카메라는 **RealSense D405**입니다. 보정값에 IR1–IR2 기준선 18 mm가 있고, 640×480 컬러 fx 433–437(176개 값, 수평 72.7°)입니다.

### Unitree 공개 데이터 전수 조사 (HF `unitreerobotics`, info.json 206개)

| 묶음 | 데이터셋 / 에피소드 | 카메라 | 보정값(`camera_params`) |
|---|---|---|---|
| G1 Dex1 v3.0 | 88 / 약 57,000 | head_stereo L/R + wrist L/R (D405, 일부는 IR 포함) | 있음 (헤드 장비 11대, 손목 D405) |
| G1 WBT **Inspire** v3.0 | 33 / 7,128 | head_stereo L/R + wrist L/R (9종은 `wrist_*_ir` 열이 있음) | **없음** |
| G1 WBT Brainco | 40 / 약 11,000 | 위와 같음 | 1종만 헤드 |
| G1 Dex1 v2.1 (UnifoLM-VLA 12종 중 9종) | 9 / 1,800 | `cam_left_high`, `cam_right_high`, `cam_left_wrist`, `cam_right_wrist` | 없음 |

- **Inspire 손목은 D405가 아닙니다.** `wrist_*_ir` 열이 있어도 G1_WBT_Inspire_Sort_Fruit 첫 데이터 파일 102,054프레임 전부 `infrared_valid = 0`이고, IR 영상 픽셀도 모두 0입니다(조사 에이전트가 Pick_Up_Drinks에서도 확인). 처음에 "IR 열이 있으니 D405"로 판단했다가 이 확인으로 고쳤습니다.
- 실제 Inspire 손목 영상은 강한 배럴 왜곡의 광각 영상입니다. Unitree 실측 손목 보정(k1 −0.40)으로 펴면 직선이 곧게 펴집니다(`logs/camera_rig/` 비교 영상).
- v3.0 info.json은 손목 관절 이름을 `wrist_yaw, wrist_roll, wrist_pitch` 순으로 적었지만, 실제 값은 G1 모터 순서(roll, pitch, yaw)입니다. 영상으로 판정했고, 아래 FK 일치로도 확인했습니다.

### 하드웨어 문서와 CAD (xr_teleoperate `Device.md`, 817fb00)
- 헤드: 스테레오 60 FPS / 30 FPS 모듈, 브래킷 Classic(98 mm 나사 간격) / Renewed(88 mm). CAD zip 4개 → `data/reference/unitree_cad/`. 카메라 판은 브래킷 좌표에서 49.3° 아래로 기울어 있습니다.
- 손목 D405 브래킷: Dex3-1용 STEP과 Dex1-1용(Google Drive)뿐입니다. GitHub STEP 링크 이름은 표에서 서로 뒤바뀌어 있습니다(`left_wrist_D405_camera_mount.STEP`이 실제로는 링). Unitree는 issue #99에서 "Brainco(Inspire)/Dex3/Dex1 마운트는 서로 호환되지 않는다"고 답했습니다.
- 손목 단안(160°) 브래킷: Dex1-1, Dex3-1, **Inspire DFX/Brainco** 세트(zip `ff287f8f…`: mount L/R, ring, cover, cable clip). Inspire용으로 공개된 손목 브래킷은 이것뿐입니다.
- 설치 지침: 링 브래킷을 손목 이음매에 맞춘다(사진 `img/real_right_hand.jpg`).

### 실제 카메라 모듈과 다른 브래킷 (2026-11-11 22시 추가 조사)
- **카메라 모듈 제조사·CAD:** 공개 자료 없음. Device.md 링크는 타오바오 상품(60 FPS 스테레오 941079164958 ¥651–666, 30 FPS 680371983853 ¥209–256, 손목 단안 679094218803 ¥135–140)이고 로그인 벽 때문에 사양 페이지를 열 수 없었습니다.
- **확인된 렌즈:** xr_teleoperate issue #132(상품 렌즈 옵션 캡처)와 #45(Unitree 답변)에 따르면 헤드는 "125° 无畸变 2.96 mm"(H103°/V86°/D125°, 2560×720 MJPEG, 녹화 1280×480), 손목은 같은 계열 "160° 有畸变广角 3.15 mm"입니다. 초점거리 구성으로 보아 M12 보드 렌즈로 추정합니다.
- **시뮬 모듈:** 위 정보와 브래킷 STEP의 보드 고정 구멍으로 만든 대용 모델입니다(`hardware_visuals.camera_modules`).
  - 헤드: 보드 Ø1.8 구멍 85.0×28.2 mm, 카메라 판 앞면(광학 중심 뒤 10.25 mm)에 놓임 → 보드 90×33 mm + 렌즈 홀더 + M12 렌즈 Ø14
  - 손목: 보드 구멍 16.7×10.8 mm, 커버 개구 Ø11·바깥 Ø14 → 보드 21×15 mm + 홀더 + M12 렌즈
- **실제로 쓰이는 손목 브래킷은 두 계열입니다** (그림 `logs/captures/wrist_bracket_candidates.png`).
  - A) Unitree Inspire/Brainco 링형(160° 단안): Unitree의 Inspire 데이터와 공식 문서가 쓰는 것. 2026-11-11 시뮬 기본, 지금은 선택 사항
  - B) 원래 시뮬 모델의 `hand_camera_base_link`(손목 위 약 10 cm): 공개 URDF에는 없고 Unitree 내부 URDF에서 변환된 것입니다. **2026-11-12 확인: Unitree Dex3용 단안 링 브래킷입니다.** 작은 부품이 `g1-camera-ring-dex3.STEP`과 ICP 평균 0.003 mm로 겹칩니다(처음의 "D405 마운트 재사용" 추정은 틀렸습니다). Inspire 모델에도 같은 Dex3 브래킷이 그대로 들어 있습니다.
  - C) Unitree D405 손목 마운트(Dex3-1, xr_teleoperate 7cd188c STEP, 길이 약 123 mm): **2026-11-12부터 시뮬 기본.** UnifoLM-VLA가 쓴 Dex1 데이터도 D405 손목입니다(Dex1용 마운트).
  - D) 커뮤니티 G1 + Inspire FTP용 D405 마운트(Zulkhuu/g1-wrist-camera-tools, STL만 있음, 실기 검증·라이선스 미확인)
  - Inspire(因时) 공식 카메라 액세서리는 찾지 못했습니다. Unitree는 Brainco(Inspire)/Dex3/Dex1 마운트가 서로 호환되지 않는다고 답했습니다(issue #99).
- **D405 공식 CAD:** realsenseai 다운로드에 SolidWorks SLDPRT만 있고 STEP은 없습니다. 광학 중심은 왼쪽 이미저(1/4-20 중심선에서 9 mm, 전면 유리 뒤 3.7 mm), 외형 42×42×23 mm입니다.

### C + D405 (2026-11-12)
- **D405 형상:** Intel `realsense-ros` `realsense2_description/meshes/d405.stl`(Apache-2.0, ros2-development 489265d)과 `_d405.urdf.xacro`를 썼습니다(`data/reference/realsense/`).
  - xacro 기준: 메시 원점은 앞판 중앙, 컬러(= 깊이 원점 = 왼쪽 IR) 광학 중심은 앞판에서 3.8 mm 뒤(유리 0.1 + datasheet 3.7 mm), 중심선에서 9 mm 왼쪽입니다. USB-C 플러그는 왼쪽 옆면에 있습니다.
  - 메시 뒷면에는 M3 구멍 2개가 기준선 방향으로 ±10 mm에 있습니다.
- **C 브래킷 STEP 분석** (`tools/cad/cylinders.py`, `tools/cad/make_d405_c_asset.py`):
  - 파일 이름이 서로 밀려 있습니다. `wrist_ring_mount.STEP`(a)과 `right_wrist_D405_camera_mount.STEP`(b)이 좌우 거울 마운트이고(거울 대응 편차 중앙값 0.000 mm, p99 0.05 mm), `left_wrist_D405_camera_mount.STEP`은 반링입니다.
  - 클램프 내경은 R30.0, 길이 42.3 mm입니다. 반링은 R30.4 × 8 mm이고 슬리브 끝에서 M3 2개(x 27.86 / −34)로 조입니다.
  - **D405 자리:** 3 mm 판에 M3 통과 구멍 2개가 20.0 mm 간격으로 있어 D405 뒷면 구멍과 같습니다. 43×43 mm 포켓이 있고, 클램프 쪽 벽에만 20×15 mm 개구가 있습니다. 이 개구가 USB-C 출구이므로 D405의 왼쪽(컬러 이미저 쪽)이 그쪽을 향합니다. 이렇게 놓은 D405는 마운트와 0.01 mm 이내로 닿기만 합니다.
  - **키:** 마운트와 반링 안쪽에 반지름 23.88 mm 평면이 있고, 손목 요 하우징 바깥에 y = ±23.86 mm 평면(x 31–36 mm)이 있습니다. 이 평면끼리 맞물려 브래킷의 손목 축 둘레 각도가 하나로 정해집니다(§3.3).
- **Unitree D405 손목 실데이터:** `unitreerobotics/G1_Dex3_BlockStacking_Dataset`(v3.0, 301 에피소드, robot_type `Unitree_G1`)에 손목 영상(`cam_left_wrist`, `cam_right_wrist`)이 있습니다.
  - 영상은 왜곡 거의 없는 직선형이고, 헤드 영상에도 손목 위 검은 D405 상자가 보입니다. 그래서 D405 손목으로 판단했습니다(이 데이터셋에는 보정값과 IR이 없음).
  - 다른 Dex3 데이터셋(PickApple 등)에는 헤드 카메라만 있습니다.
  - D405 컬러 내부 파라미터는 Dex1 데이터셋 16개의 `camera_params.wrist_*.color`에서 얻었습니다(실물 왼쪽 9대, 오른쪽 7대, fx 432.9–437.5). librealsense `rs2_project_point_to_pixel`이 `inverse_brown_conrady` 계수를 정방향 다항식으로 쓰는 것을 소스로 확인했으므로, OpenCV 계수로 그대로 씁니다.
- **헤드 98/88 mm:** 브래킷 M4 나사 간격입니다(Classic/Renewed 머리의 나사 간격 차이, issue #85). 공식 G1 간이 STEP(Unitree docs "Get SDK")과 URDF 머리 메시에는 이 구멍이 모델링돼 있지 않습니다.

### Unitree 시뮬레이터 upstream (`unitree_sim_isaaclab` main f42c081, 2026-11-09, Isaac Sim 6.2/Lab 3.0)
- `robots/uniclib_calibration.json`: **G1-D**(바퀴형) 실측 보정입니다. 헤드 스테레오(fx 273.3, 기준선 60.1 mm), 손목 광각(fx 418–427, k1 ≈ −0.4), hand-eye(`wrist_yaw_link` 기준, Dex3 손).
- G1용 헤드 카메라는 "왼눈 = 기존 `d435_link` 자세, 오른눈 = 왼눈 + 보정 외부 파라미터"이고, 렌즈 왜곡은 RTX OpenCV 모델입니다. 손목 강한 배럴 다항식은 영상 모서리 전에 접히므로 단조 유리함수로 다시 맞춥니다. 이 재맞춤 방법을 이 프로젝트에도 옮겼습니다(Apache-2.0).
- G1-D URDF `Body_Depth_Camera_Link`: 몸통 기준 (0.060, 0.0325, 0.4165), 47.6° 아래, 모듈 중심 y=0입니다. 우리 G1의 `d435_link`(0.0576, 0.0175, 0.4299, 47.6°)와 거의 같은 자리입니다.

### 논문·기타 (조사 에이전트)
- G1 + Inspire + 손목 카메라의 사양을 밝힌 논문은 찾지 못했습니다(Humanoid Everyday, TWIST2, GR00T-WholeBodyControl, arXiv 2603.28422는 모델·장착을 밝히지 않았거나 다른 구성).
- 따라서 "가장 많이 참조된 표준"은 Unitree 자체 공개 데이터·문서입니다. UnifoLM-VLA가 쓴 것은 G1+Dex1(헤드 스테레오 + D405 손목)입니다. Unitree가 G1+Inspire 데이터에 쓴 것은 공식 Inspire 브래킷 + 단안 광각 손목 + 같은 헤드 스테레오입니다(모델 A). 2026-11-12부터는 사용자 결정에 따라 손목 기본을 Unitree D405 마운트(C) + D405로 바꿨고, A는 선택으로 남겼습니다.

## 3. 위치를 어떻게 정했나

### 3.1 기구학 확인
실제 데이터(G1_WBT_Inspire_Sort_Fruit)의 관절값을 시뮬 로봇에 넣고 FK를 계산했습니다. 데이터에 기록된 Unitree EE 위치(`*_ee_pose_gripper_torso`)는 우리 FK의 `wrist_yaw_link + (0.1659, 0, 0) m`와 **14개 프레임·양팔에서 RMS 1.2 mm**로 일치합니다. 시뮬 로봇과 실제 로봇의 팔 기구학이 같으므로, 영상 차이는 카메라 장착에서만 생깁니다.

### 3.2 헤드
- 방법: 팔이 비교적 느린(≤0.25 rad/s) 실제 프레임 14개에서 로봇 메시를 투영한 실루엣 경계와 실제 영상 에지 사이 거리(잘린 chamfer, 평균 px)를 최소화했습니다(`tools/fit_rig_camera.py`, Nelder–Mead).
- 초점거리와 위치는 영상만으로 분리되지 않습니다(fx 253 ↔ 291, z 0.416 ↔ 0.452가 같은 비용). 그래서 내부 파라미터를 실측 장비값으로 고정했습니다. 공개 보정 11대 중 fx 중앙값 장비(G1_Dex1_Stack_Block, UnifoLM-VLA 학습 데이터)이고, 주점도 내부 파라미터를 풀어서 맞춘 값(cx 341–344)과 일치합니다.
- **2026-11-12 정정:** `cv2.fillPoly`에 삼각형을 한꺼번에 넘기면 홀짝 규칙으로 채워서, 겹친 삼각형(닫힌 메시의 앞뒷면)이 서로 지워집니다. 그래서 실루엣이 와이어프레임처럼 그려졌습니다. 2026-11-11 맞춤은 모두 이 상태로 계산됐습니다. 삼각형을 하나씩 채우도록 고치고(`fit_rig_camera.py silhouette`) 같은 14프레임으로 다시 맞췄습니다.
- 결과: 대칭 제약 맞춤(x, z, 피치)과 6자유도 자유 맞춤이 2.6 mm, 0.3° 안에서 모였습니다. 6자유도 해는 (0.0613, −0.0007, 0.4321), 47.7°, yaw 0.54°, roll 0.08°입니다. 최종값은 대칭 해인 중심 **(0.0587, 0, 0.4310) m, 피치 47.4°**입니다(URDF `d435_link` 47.6°와 0.2° 차이). 이전 값 (0.0608, 0, 0.4308), 46.3°와는 2.1 mm, 1.1° 다릅니다.

| 헤드 카메라 모델 (14프레임, 고친 실루엣) | 정합 비용 | (이전 와이어프레임 비용) |
|---|---|---|
| **최종 리그 (0.0587, 0, 0.4310), 47.4°** | **3.48 px** | — |
| 이전 리그 (0.0608, 0, 0.4308), 46.3° | 3.84 px | 4.78 px |
| CAD 초기 추정 (0.068, 0, 0.425), 49.3° | 4.55 px | 5.16 px |
| 현재 공식 프리셋 (`d435_link` 단안, fx 243) | 9.23 px | 7.97 px |
| upstream 신버전 (`d435_link` + uniclib 렌즈) | 9.23 px | 8.81 px |

### 3.3 손목: C + D405 (기본, 2026-11-12)
카메라 자세는 세 단계를 이어서 정합니다. 그림 `logs/captures/rig_d405c_closeup.jpg`(근접), `d405c_dex3_real_vs_sim.jpg`(실데이터 비교), `rig_d405c_desk_smoke.jpg`(책상 장면 녹화).

1. **D405 → 브래킷:** CAD 포켓 안 정위치입니다(§2: 판의 M3 쌍 = D405 뒷면 M3 쌍, USB 개구 = D405 왼쪽). 컬러 광학 중심은 판 앞면에서 23 − 3.8 = 19.2 mm 앞, 구멍 중점에서 9 mm 옆입니다. 광축은 판 법선으로, 클램프 축에서 31.6° 벗어나 있습니다. 시뮬은 이 값을 바꾸지 않습니다(`cam_correction_deg = 0`).
2. **브래킷 → 손목:**
   - 클램프 축은 손목 요 하우징 축(y = z = 0)입니다. 하우징은 x 0–40 mm에서 이 축 기준 R30.5 원통입니다.
   - 축 방향은 반링이 손 플랜지에 닿는 위치(반링 중심 x = 36.5 mm, 플랜지까지 1.3 mm)입니다. 공식 USD가 Unitree Dex3 링을 둔 x와 같습니다(ICP). 공식 USD는 그 링을 y = ∓3 mm(손 베이스 오프셋)에 두었는데, 하우징과 겹치므로 따르지 않았습니다.
   - 손목 축 둘레 각도 φ는 키가 정합니다. 마운트 안쪽 평면(r 23.88)이 하우징 평면(±y, r 23.86)에 맞물리는 각도는 **φ = 90°**뿐이고, 이때 카메라가 손목 위(+z)에 옵니다.

| φ (오른손, 왼손은 180−φ) | 하우징과의 최대 반지름 겹침 (축 위치 33–40 mm 전 구간) |
|---|---|
| **90° (키 맞물림, 최종)** | **0.36–0.43 mm** (클램프 예압: 하우징 R30.5 대 내경 R30.0) |
| 108° (실데이터 맞춤값, 아래) | 3.6–3.7 mm (키가 하우징을 파고듦, 장착 불가) |

3. **Inspire 손과의 간섭:** 없습니다(trimesh 점 포함 판정, 공식 Inspire USD 링크 메시). 마운트와 손 베이스 사이 1.4–2.0 mm, 반링 1.3–1.4 mm, D405와 손 47 mm, 손가락 50 mm 이상입니다.

**실데이터 대조 (Unitree G1_Dex3_BlockStacking, 실제 D405 손목, 16프레임 × 양손).** 공식 Dex3 USD를 기록된 관절값으로 렌더링하고(`render_camera_rig.py --robot dex3`) 실루엣-에지 비용을 쟀습니다.
- 손가락은 손바닥 기준 위치가 맞아도 끝이 어긋납니다. 관절값 의미 차이나 영상-상태 지연으로 보입니다. 그래서 손바닥만 쓴 비용을 함께 봅니다.

| 손목 C 자세 | 손바닥 비용 (왼 / 오른) | 손 전체 비용 (왼 / 오른) |
|---|---|---|
| **CAD 정위치 φ 90°, x 36.5 (최종)** | **24.9 / 19.2 px** | **16.9 / 15.6 px** |
| φ 108° / 72° (데이터 최적 회전, 장착 불가) | 16.1 / 14.3 | 13.4 / 13.8 |
| + 카메라 틸트까지 자유 (CAD 포켓 무시) | 7.2 / 5.9 | — |

- 구도는 CAD 정위치에서도 실제와 같습니다(손바닥 몸체가 왼손은 위, 오른손은 아래, 손가락 방향, 반대 손이 보이는 자리).
- 그러나 이 데이터를 찍은 로봇은 C를 CAD대로 끼운 것보다 손목 축 둘레로 약 18°(좌우 거울 대칭), 화면 좌우로 약 9° 더 돌아간 카메라와 맞습니다. φ 108°는 키 때문에 이 브래킷으로 만들 수 없습니다. 그래서 그 로봇의 브래킷 리비전, 손 장착 각도, 하우징 중 하나가 다르다고 봅니다. 실물 C를 끼운 사용자 로봇은 CAD 정위치를 따른다고 보고 기본값을 그렇게 두었습니다.

### 3.4 손목: 이전 모델 A (Inspire/Brainco 링 + 160° 단안, `wrist.mount=inspire_ring_mono160`)
아래 수치는 2026-11-11 와이어프레임 실루엣으로 계산한 값입니다(§3.2 정정). 순위 판단용으로만 보세요.
- CAD 조립: 링 안지름 60 mm가 손목 요 하우징(반지름 30.5 mm, x 20–36 mm)에 꼭 맞습니다. 링 중심을 x=32 mm에 두면 손 쪽 테두리가 손목/손 이음매(x≈38–40 mm 홈)에 닿습니다. 렌즈 구멍은 링 축에서 30.5 mm, 링 평면에서 손 쪽으로 6 mm이므로 광학 중심은 (0.038, ±0.0305, 0)입니다. 손바닥 쪽(오른손 +y, 왼손 −y)에 두는 것은 실제 영상(손바닥 아래에서 손가락 끝을 바라봄)과 Unitree G1-D hand-eye(같은 방향)에 근거합니다.
- 실제 손목 영상 28장(양손 × 14)으로 후보를 평가했습니다.

| 손목 후보 (좌우 대칭) | 비용 합 (왼+오른) |
|---|---|
| **CAD 위치 + 손 축 쪽 5° 기울기 (최종)** | **30.74 px** |
| CAD 위치, 기울기 0 | 31.06 |
| + 접선 +5° (CAD 상자 기울기) / −5° | 32.31 / 31.93 |
| 손 축 바깥쪽 5° | 34.35 |
| G1-D 실측 hand-eye 배치 (x 47 mm, 바깥 2.9°) | 34.48 |

손목 영상은 가깝고 흐리며 배경 에지가 많아 헤드보다 잘 정해지지 않습니다(좌우 개별 자유 맞춤은 서로 엇갈림). 그래서 큰 차이가 나는 후보만 기각하고, CAD 기하와 데이터가 함께 지지하는 값을 택했습니다.

## 4. 시뮬 하드웨어 표현

### 헤드 브래킷 CAD 정렬과 독립 검증
- Unitree 헤드 브래킷(60 FPS, Classic 98 mm) STEP에서 카메라 판 법선(49.3° 아래)과 커버 렌즈 구멍(Ø18, y=±30 mm)을 뽑았습니다. 커버는 조립 위치가 아니라 따로 그려져 있어, 커버 나사 구멍 4개가 마운트의 y=±39 mm 구멍에 겹치는 평행이동(−3.5, +4.58, +58.56) mm로 복원했습니다.
- `camera_rig.head_bracket_pose`는 렌즈 구멍 중점을 두 눈 광학 중심의 중점에, 판 법선을 광축에, CAD +y를 로봇 오른쪽에 맞춥니다. CAD는 −x 앞, z 위인 오른손 좌표계입니다.
- **독립 검증:** 실제 영상으로만 맞춘 카메라 자세에 브래킷을 붙이면, 브래킷의 M4 나사 구멍(y=±49 mm)이 머리 링크 기준 z=467.0 mm, x=−2.5 mm에 떨어집니다(재맞춤 전 465.7 mm). 이는 머리 메시 내부 선반(z≈465 mm) 위치이고, 공식 설치 그림의 배치(얼굴 개구부 위 카메라 판, 머리 안쪽 판에 나사 고정)와 같습니다. CAD z축과 머리 z축의 차이는 1.9°입니다(재맞춤 전 3.0°, 테스트 `test_head_bracket_aligns_with_eyes_and_head_plate`).
- 근접 렌더링: `logs/captures/rig_hardware_closeup.jpg`(A 시절), `logs/captures/rig_d405c_closeup.jpg`(C + D405)

- `camera_rig.apply_rig_hardware`가 로봇 USD를 생성한 직후, 물리 시작 전에 다음을 합니다.
  1. 공식 `*_hand_camera_base_link`(Dex3 단안 링 브래킷)의 visuals·collisions를 비활성화합니다.
  2. 손목 하드웨어를 붙입니다.
     - `wrist.mount = unitree_d405_c`: `assets/camera_rig/wrist_d405_mount_c.npz`(`tools/cad/make_d405_c_asset.py`)의 마운트·반링(검정)과 D405(알루미늄)를 손목 카메라와 같은 변환으로 `<side>_wrist_yaw_link` 아래에 둡니다. 실제 D405 형상이 카메라 모듈 역할을 하므로 대용 모듈은 없습니다.
     - `inspire_ring_mono160`: Inspire 링 브래킷(`inspire_wrist_bracket_{left,right}.npz`)과 보드·렌즈 대용 모델을 둡니다.
  3. 헤드 스테레오 브래킷, 보드 상자, 렌즈를 놓습니다.
- 모두 시각용(충돌 없음)입니다. 손목 카메라는 D405 앞판 3.8 mm 뒤에 있고, 근거리 클리핑 10 mm로 자기 앞판을 보지 않습니다. 포켓 벽(앞판보다 2 mm 높음)은 시야 밖(광축에서 61°)입니다.
- 공식 브래킷 충돌체를 끈 뒤에도 스크립트 파지가 성공했습니다. A일 때 3회, 들림 7.6–7.8 cm였고, C + D405에서는 2026-11-12에 1회, 들림 7.2 cm였습니다.

## 4.1 GUI 카메라 패널
GUI로 띄우면 뷰포트 아래에 "G1IV Cameras" 창이 붙어 4개 카메라 영상을 보여줍니다. 녹화·정책에 들어가는 바로 그 버퍼(렌즈 왜곡 포함)를 읽어 2 제어 스텝마다 갱신하며, 추가 렌더링은 없습니다. `camera_rig.yaml gui_panel`에서 끄거나(`--rig-set gui_panel.enabled=false`) 갱신 주기와 높이 비율을 바꿀 수 있습니다. 캡처: `logs/captures/gui_camera_panel_v1.png`.

## 5. 렌즈 모델 검증 (RTX ↔ OpenCV)
`tools/lens_check.py`는 카메라마다 시야 안 35개 지점(알려진 3D 위치)에 발광 구를 놓고, 렌더링된 중심을 OpenCV 투영과 비교합니다. 이 과정에서 Isaac Sim 5.1 `OmniLensDistortionOpenCvPinholeAPI`의 규약 두 가지를 실측으로 찾아 반영했습니다.
- p1 부호가 반대입니다. RTX가 영상 y를 위쪽으로 잡아 y 홀수차 항(= p1 항)만 뒤집힙니다.
- 주점이 0.5 px 어긋납니다(픽셀 원점 규약).

2026-11-12 (C + D405, 새 헤드 자세, `logs/camera_rig/lens_check_d405c`):

| 카메라 | 오차 평균 / 중앙값 / 90% / 최대 (px) |
|---|---|
| 헤드 왼눈 | 0.20 / 0.17 / 0.39 / 0.50 |
| 헤드 오른눈 | 0.22 / 0.19 / 0.41 / 0.49 |
| 왼손목 D405 | 0.42 / 0.29 / 0.50 / 2.65 |
| 오른손목 D405 | 0.42 / 0.37 / 0.56 / 2.00 |

2026-11-11 (A 손목, `logs/camera_rig/lens_check_final`):

| 카메라 | 오차 평균 / 중앙값 / 90% (px) | 규약 보정 전 평균 | 렌즈 모델 없이 (대조) |
|---|---|---|---|
| 헤드 왼눈 | 0.17 / 0.17 / 0.35 | 5.9 | 35개 중 4개만 25 px 안 |
| 헤드 오른눈 | 0.25 / 0.22 / 0.43 | 5.1 | 20개 |
| 왼손목 (160°) | 0.32 / 0.19 / 0.37 (최대 4.7, 다른 카메라 표식과 겹침) | 3.1 | 13개 |
| 오른손목 (160°) | 0.33 / 0.19 / 0.33 | 0.7 | 7개 |

이 확장(`omni.usd.schema.omni_lens_distortion`)은 앱 시작 때만 등록되므로 `run_sim.sh`가 `--kit_args`로 켭니다. 렌즈를 적용하지 못하면 로그에 크게 경고하고, 에피소드 메타(`static.lens_distortion`)에 남깁니다.

## 6. 데이터 형식
- 녹화 별칭 → LeRobot 키: `head → cam_left_high`, `head_right → cam_right_high`, `left_wrist → cam_left_wrist`, `right_wrist → cam_right_wrist`(UnifoLM-VLA가 읽는 이름). `modality.json` video에 `head_right`가 추가됩니다.
- 에피소드 `meta.json`의 `static.camera_rig`에 카메라별 링크, 위치, 쿼터니언(OpenCV), K, 왜곡, 렌더 왜곡이 들어갑니다. 변환하면 `meta/info.json`에 Unitree 데이터셋과 같은 배치의 `camera_params`(head_stereo의 K/dist/R/T, wrist_*.color)와 `extrinsics`가 기록됩니다.
- 왕복 확인: 변환된 T = [−58.862, −0.239, −0.315] mm는 원본 G1_Dex1_Stack_Block의 T와 소수점까지 같습니다.
- 녹화기는 기록 순간 카메라 버퍼를 강제로 최신 렌더로 갱신합니다(`cam_meta.forced_update`). 공식 이미지 서버가 같은 카메라를 따로 읽어 생길 수 있던 1 제어 스텝 지연을 없앴습니다.

## 7. 성능 (4060, 서 있는 상태, 헤드리스, `tools/perf_rig.sh`)
| 조건 | RTF |
|---|---|
| 리그 4카메라 + 렌즈 | 0.329 |
| 리그 3카메라(오른눈 끔, `--rig-set head.right_eye=false`) | 0.360 |
| 리그 4카메라, 렌즈 없음 | 0.331 |
| 공식 프리셋 3카메라 | 0.378 |

렌즈 모델 비용은 측정되지 않을 만큼 작고, 오른눈 카메라가 약 9%를 차지합니다. 스크립트 파지 녹화 중에는 RTF 0.27–0.29입니다.

## 8. 도구
| 도구 | 하는 일 |
|---|---|
| `tools/unitree_frames.py` | Unitree 데이터셋 프레임의 관절값 + 실제 헤드/손목 영상(HF 영상 seek, 전체 다운로드 없음). `--layout dex3`: G1_Dex3_* (28차원 상태, `cam_*` 영상) |
| `tools/render_camera_rig.py` | 같은 관절 자세(고정 베이스)로 리그 카메라 렌더링 + 링크 자세 저장. `--robot dex3`: 공식 G1+Dex3 USD |
| `tools/compare_rig.py` | 실제 / 시뮬 / 겹친 영상 비교 그림 |
| `tools/fit_rig_camera.py` | 실루엣-에지 정합으로 헤드/손목 카메라 자세(·내부 파라미터) 맞춤과 비용 평가. C 모델은 `--free phi ax [rx ry rz]`, `--scan phi:a:b:step`, 손목 근거리 `--near 0.01` |
| `tools/lens_check.py` | RTX 렌즈 모델과 OpenCV 투영의 픽셀 오차 |
| `tools/rig_landmarks.py`, `tools/crop_grid.py` | 팔 기준점 투영, 확대 격자(수동 확인용) |
| `tools/export_link_meshes.py`, `tools/inspect_camera_links.py` | USD 링크 메시·카메라 링크 추출 (`ALL` = 모든 링크; Dex3 메시 → `data/reference/usd_link_meshes_dex3`) |
| `tools/cad/*.py` | STEP → STL, 형상 그림, 원통·평면 특징, 브래킷 조립 메시(`make_bracket_assets.py` A 손목, `make_d405_c_asset.py` C + D405 손목, `make_head_bracket_asset.py` 헤드) (gmsh 4.15.2 + trimesh 5.2.0, 임시 venv에서 실행) |
| `tools/xwd_to_png.py` | X 화면 덤프 변환 (화면 캡처는 `ffmpeg -f x11grab` 사용) |

## 9. 한계
- Inspire WBT 데이터에는 보정값이 없습니다. 그래서 헤드 내부 파라미터는 같은 모델의 다른 G1 장비(Dex1 데이터)에서 가져왔습니다. D405 손목은 Unitree Dex1 데이터의 실측 D405 중앙값 장비이고, A 손목은 G1-D 장비 값입니다. 실제 장비마다 헤드 fx가 186–296으로 다릅니다. 실물 로봇을 쓰게 되면 그 로봇을 보정해 `camera_rig.yaml`의 `calibration`만 바꾸면 됩니다(D405는 `rs-enumerate-devices -c`의 컬러 640×480 값).
- 헤드 피치는 주점 cy와 함께 움직입니다(내부 파라미터를 풀면 36–40°). 최종 47.4°는 고정한 장비 보정값과 짝을 이루는 값입니다.
- **C + D405 손목은 CAD로 정한 자세입니다.** C를 G1 + Inspire에 단 실제 영상은 공개 자료에 없습니다. 가장 가까운 실데이터(Unitree G1 + Dex3 + D405 손목)는 CAD 정위치보다 손목 축 둘레 약 18°, 화면 좌우 약 9° 더 돌아간 카메라와 맞습니다. 손바닥 실루엣 비용은 CAD 정위치에서 19.2 / 24.9 px(오른 / 왼)입니다. 위치와 카메라 틸트를 모두 풀면 5.9 / 7.2 px까지 내려갑니다. 이때 φ는 112° / 80°, 좌우 방향 틸트 보정은 −10.4° / −7.7°입니다(`logs/camera_rig/fit_d405c_tilt_*`). 실물 C를 단 로봇의 손목 영상이 몇 장 있으면 `tools/fit_rig_camera.py --free phi ax rx ry rz`로 바로 확인할 수 있습니다.
- Dex3 실데이터의 손가락 관절값은 손가락 끝 위치가 시뮬과 어긋납니다(관절 의미 또는 영상-상태 지연). 그래서 손목 대조는 손바닥 위주로 했습니다.
- A 손목: 광각 영상 모서리는 보정 범위 밖이라 단조 재맞춤 모델로 근사합니다. 자세 불확실성은 대략 ±4 mm, ±3°입니다(와이어프레임 실루엣으로 맞춘 값).
- 헤드 카메라 모듈(보드·홀더·렌즈)은 제조사 CAD가 없어 브래킷 구멍과 렌즈 사양으로 만든 대용 모델입니다(시각용, 정책 카메라에는 보이지 않음). D405는 Intel 공식 메시입니다.
- 헤드셋으로 가는 영상(공식 이미지 서버)은 아직 왼눈 단안입니다. PICO 입체 표시는 PICO_PENDING_CHECKLIST에 남겼습니다.
- Isaac Lab `CameraData.intrinsic_matrices`는 주점 오프셋을 무시합니다(Omniverse가 aperture offset 미지원). 정확한 K는 에피소드 메타와 `info.json camera_params`를 쓰세요.
