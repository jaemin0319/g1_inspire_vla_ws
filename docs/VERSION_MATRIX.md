# VERSION_MATRIX

기준 시각 2026-11-11 15:45 KST. 모두 이 PC에 **실제로 설치된** 버전입니다. 정확한 전체 목록은 `envs/*.pip-freeze.txt`, `envs/*.explicit.txt`, 저장소 커밋은 `envs/repos.lock.json`이 정본입니다.

## 호스트 (scripts/check_system.sh, logs/setup/check_system_initial.txt)

| 항목 | 값 |
|---|---|
| OS / 커널 | Ubuntu 24.04.5 LTS / 6.8.0-147-generic, X11 (`DISPLAY=:0`) |
| CPU / RAM | Intel Core Ultra 7 265K (20 threads) / 62 GiB. CPU 성능 프로필 **powersave**(Isaac Sim 경고, 아래 성능 참고) |
| GPU 0 | RTX 4060 8 GB, PCI 02:00.0, `GPU-e7a12b9a-…` — 모니터 연결, 기본 GPU |
| GPU 1 | RTX 5060 Ti 16 GB, PCI 81:00.0, `GPU-64746556-…` — 학습용 |
| 드라이버 | 580.178.04 (CUDA 13.0 지원; Isaac Sim 5.1 요구 580.65.06 이상 충족) |
| 디스크 | `/` 48 GB 중 약 3.8 GB 여유(Isaac Sim 텍스처 캐시 `~/.cache/ov` 1.2 GB가 여기에 쌓임) · `/mnt/isaac` 98 GB 중 약 36 GB 여유 |
| ROS 2 | Jazzy가 ~/.bashrc에서 source됨 → 모든 런처가 `/opt/ros` 관련 변수를 제거(`scripts/_env_common.sh`) |
| micromamba | 2.9.0 (`~/.local/bin`, 새 PC는 `scripts/bootstrap.sh micromamba`가 릴리스 `2.9.0-0`을 SHA-256 `366cd9cd…`로 확인해 설치), 프로젝트 root `.conda/` (다른 env와 분리) |
| FFmpeg(시스템) | libavcodec 60 (FFmpeg 6) + libdav1d — vla env의 torchcodec 0.8.0이 사용 |

## 공식 저장소 (envs/repos.lock.json이 정본)

| 저장소 | 커밋 | 비고 |
|---|---|---|
| unitreerobotics/unitree_sim_isaaclab | `e30c25b` (브랜치 `IsaacSim5.1.0`, 2026-03-30) → 로컬 브랜치 `g1iv/isaacsim5.1` | main(`c0d2e68`, 2026-11-10)은 Isaac Sim 6.2/Lab 3.0/Py3.12/cu130으로 전환되어 채택하지 않음. 패치: `envs/patches/unitree_sim_isaaclab_g1iv.patch` |
| └ teleimager (submodule, branch sim) | `b81de44` (2026-02-25) | 이미지 서버(ZMQ 55555-7, WebRTC 60001-3) |
| isaac-sim/IsaacLab | `80094be` (= v2.3.2+7, 2026-02-21) | unitree 5.1 설치 문서가 지정한 커밋 |
| unitreerobotics/xr_teleoperate | `817fb00` (main, 2026-09-07) | |
| └ televuer | `766de45` (2026-05-27) | WebXR(Vuer 0.0.60), 컨트롤러 모드 |
| └ teleimager | `57cf2a4` (2026-06-26) | 클라이언트 |
| unitreerobotics/unitree_sdk2_python | `814556d` (2026-09-21) | |
| unitreerobotics/unitree_lerobot | `41c2805`, lerobot submodule `a5b29d4` | 검토만(변환 형식 근거) |
| NVIDIA/Isaac-GR00T | `3b49067` (main = N1.7, 2026-11-10) | vla env에 `--no-deps` editable(로더만 사용) |

## 에셋 · 가중치

| 항목 | 출처 | 고정값 |
|---|---|---|
| USD/정책 묶음 `assets.zip` | HF `datasets/unitreerobotics/unitree_sim_isaaclab_usds` revision `394cf2448f8a9ed815c77c701a761f3d1ff1c8fb` | sha256 `06fbf145…` (LFS oid와 일치) |
| 로봇 USD | `assets/robots/g1-29dof_wholebody_inspire/g1_29dof_with_inspire_rev_1_0.usd` | 관절 53개. 손 기하는 `logs/checks/hand_geometry.txt` |
| 보행 정책 | `assets/model/policy.onnx` | sha256 `d067b393…` |
| 팔 IK 모델 | xr_teleoperate `assets/g1/g1_body29_hand14.urdf` | EE `L_ee/R_ee` = wrist_yaw_link + 0.05 m x |
| GR00T 가중치 | — | 받지 않음(로더 확인만) |

## 재현용 고정 파일 (scripts/bootstrap.sh가 읽음, 2026-11-18)

| 파일 | 고정하는 것 |
|---|---|
| `envs/repos.lock.json` | 외부 저장소 6개 커밋과 초기화된 서브모듈 커밋 (`scripts/setup/fetch_repos.py`) |
| `envs/patches/unitree_sim_isaaclab_g1iv.patch` | 프로젝트 패치(소스 2개: `dds/dds_master.py`, `sim_main.py`). upstream이 추적하는 `.pyc` 변경은 뺌 |
| `envs/assets.lock.json` | 공식 에셋 HF 리비전, zip SHA-256 `06fbf145…`, 파일 수 423, 보행 정책 2개와 로봇 USD의 SHA-256 (`scripts/setup/fetch_assets.py`) |
| `envs/hand_reference.lock.json` | RH56E2/FTP 손 원자료 232개 파일의 저장소·커밋·경로·git blob 해시 (`scripts/setup/fetch_hand_reference.py`) |
| `envs/<env>.yaml`, `*.explicit.txt`, `*.pip-freeze.txt` | 설치된 실제 패키지 기록 (`scripts/export_envs.sh`, `prefix:` 줄 없음) |
| `envs/constraints_*.txt` | `scripts/setup_envs.sh`의 pip 제약 |

## Conda env (모두 설치됨)

`g1_inspire_cad`(2026-11-18 추가): Python 3.12, numpy 2.5.4, scipy 1.18.1, trimesh 5.2.0, opencv-python-headless 5.0.0.93, networkx 3.7, rtree 1.4.1, shapely 2.2.0, pyyaml 6.0.3 (gmsh 4.15.2는 `G1IV_CAD_GMSH=1`일 때). 2026-11-12 손 빌드에 쓴 임시 venv와 같은 버전.

| env | Python | 주요 패키지 | `pip check` |
|---|---|---|---|
| `g1_inspire_sim` | 3.11.18 | isaacsim 5.1.0.0(IGRIS env에서 하드링크), torch 2.7.0+cu128, Isaac Lab 0.54.3(source 6개 editable: isaaclab, _assets, _contrib, _mimic, _rl, _tasks), unitree_sdk2py 1.0.1, cyclonedds 0.10.2(conda-forge C 라이브러리 cyclonedds 0.10.5에 대해 빌드), teleimager[server] 1.5.0, onnxruntime 1.22.1, numpy 1.26.0, pillow 11.3.0, h5py 3.16.0 | fastapi↔starlette 1줄(공식 Isaac Lab 설치와 동일, 아래) |
| `g1_inspire_teleop` | 3.10.22 | pinocchio 3.1.0(conda-forge), numpy 1.26.4, unitree_sdk2py 1.0.1, cyclonedds 0.10.2(wheel), televuer 4.0.0, vuer 0.0.60, params-proto 2.13.2, teleimager 1.6.0, pygame 2.6.1, pyarrow 21.0.0 | 통과 |
| `g1_inspire_data` | 3.11.18 | torch 2.7.0+cu128(sim env에서 하드링크), lerobot 0.3.3(dataset **v2.1**), datasets 3.6.0, av 18.1.0(libsvtav1·libdav1d 포함), pyarrow 21.0.0, pandas 2.3.3, huggingface-hub 0.36.2 | 통과 |
| `g1_inspire_vla` | 3.12.16 | **로더 전용**: torch 2.9.0+cpu, torchvision 0.24.0+cpu, torchcodec 0.8.0, numpy 1.26.4, pandas 2.2.3, gr00t 0.1.0(`--no-deps`) | GR00T 전체 의존성(flash-attn, deepspeed, TensorRT 등)을 일부러 빼서 "미설치" 목록이 나옴 |

### 공식 구성 대비 고정값과 이유 (constraints: `envs/constraints_*.txt`)

| 고정 | env | 이유 |
|---|---|---|
| torch 2.7.0 **cu128** | sim, data | 공식 cu126은 RTX 5060 Ti(sm_120) 미지원. 버전 번호는 Isaac Sim 요구와 같음 |
| conda-forge `cyclonedds=0.10.5`(C) + `CYCLONEDDS_HOME` | sim | unitree_sdk2py가 고정한 cyclonedds 0.10.2는 cp311 wheel이 없어 소스 빌드 필요(공식 FAQ 방식). ROS의 CycloneDDS는 쓰지 않음 |
| starlette 0.49.1 | sim | Isaac Lab 보안 수정 고정(IsaacLab `64ecea24f`). Isaac Sim의 fastapi 0.115.7(starlette<0.46)과 충돌하는 것은 `isaaclab.sh --install`과 동일 — `pip check`에서 이 한 줄만 허용 |
| cryptography 44.0.0, psutil 5.9.8, ipython<9 | sim | Isaac Sim 고정값. aiortc·ipython 9가 올리려는 것을 막음 |
| pyarrow 21.0.0 | sim, teleop, data, vla | pyarrow 22 이상은 numpy 1.x에서 import 자체가 실패(메타데이터에 표시되지 않아 `pip check`가 못 잡음) |
| params-proto 2.13.2 | teleop | 3.x에서 `Flag`가 사라져 vuer 0.0.60(televuer) import 실패 |
| huggingface-hub<1.0, pandas<3 | data | lerobot 0.3.3 / datasets 3.6.0 시점 버전 |

선택 이유(요약):
- Isaac Sim 5.1은 IGRIS env에 이미 있어 하드링크로 재사용했습니다. 설치 전후 IGRIS env 스냅샷 해시(파일 77,823개)가 같습니다(`logs/setup/igris_env_snapshot_{pre,post}_install_*.json`).
- 물리는 공식대로 `--device cpu`(PhysX CPU), GPU는 렌더링과 카메라에만 씁니다.

외부(Conda로 격리되지 않는) 의존성: NVIDIA 드라이버, Vulkan ICD, X11, `/dev/shm`(공식 시뮬이 고정 이름 공유메모리 사용 → 시뮬은 한 번에 한 개), 시스템 FFmpeg 6(vla env), gcc(cyclonedds 빌드), openssl(인증서), 네트워크(PICO 접속 시 같은 망).
