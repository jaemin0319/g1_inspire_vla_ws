# 새 PC에 설치하기 (Ubuntu 24.04)

이 문서는 저장소를 새 Ubuntu 24.04 PC에 받아 시뮬레이션·텔레옵·데이터 변환까지 그대로 다시 만드는 방법입니다. 모든 외부 입력(저장소, 에셋, 파이썬 패키지, 손 원자료)은 커밋·리비전·해시로 고정돼 있어서, 같은 명령이면 이 저장소를 만든 PC와 같은 환경이 됩니다.

## 1. 요구 사항

| 항목 | 필요 | 이 저장소를 만든 PC |
|---|---|---|
| OS | Ubuntu 24.04 x86_64 | Ubuntu 24.04.5, 커널 6.8 |
| GPU | NVIDIA, 8 GB 이상 | RTX 4060 8 GB(시뮬), RTX 5060 Ti 16 GB |
| 드라이버 | RTX 50 계열은 570 이상 | 580.178.04 |
| RAM | 32 GB 이상 권장 | 62 GB |
| 디스크 | 저장소를 받은 파티션에 **45 GB 이상** 여유 | 환경 24 GB, pip 캐시 약 8 GB, 외부 저장소 4.3 GB, 에셋 3 GB |
| 네트워크 | github.com, huggingface.co, pypi.org, pypi.nvidia.com, download.pytorch.org, conda.anaconda.org(conda-forge) | |
| 시스템 패키지 | git, python3, openssl, iproute2, libvulkan1 (보통 기본 설치) | |

sudo가 필요한 것은 NVIDIA 드라이버와 위 시스템 패키지뿐입니다. 부트스트랩은 sudo를 쓰지 않고, 작업공간 밖에는 micromamba 하나만 `~/.local/bin`에 둡니다(이미 있으면 그대로 씀, 다른 곳은 `G1IV_MAMBA=<경로>`). ROS 2가 셸에 source돼 있어도 됩니다. 프로젝트 스크립트가 ROS 변수를 지우고 실행합니다.

## 2. 설치

```bash
git clone https://github.com/jaemin0319/g1_inspire_vla_ws.git
cd g1_inspire_vla_ws
scripts/bootstrap.sh check                       # 점검만 (바꾸는 것 없음)
scripts/bootstrap.sh all --accept-nvidia-eula    # 전체 설치 + 테스트 + 시뮬 기동 확인
```

`--accept-nvidia-eula`는 [NVIDIA Omniverse/Isaac Sim 라이선스](https://docs.isaacsim.omniverse.nvidia.com/latest/common/NVIDIA_Omniverse_License_Agreement.html)를 읽고 동의할 때만 붙이세요. 동의는 `.cache/nvidia_eula_accepted`에 기록되고, 그 뒤 시뮬레이터 실행과 손 USD 빌드가 이 기록을 확인합니다. 동의하지 않으면 손 USD 빌드와 시뮬 기동 확인 단계에서 멈춥니다.

`all`은 아래 단계를 차례로 실행합니다. 각 단계는 따로 실행할 수 있고, 실패하면 같은 명령을 다시 실행하면 이어서 합니다(이미 된 것은 건너뜀, 기존 것을 지우거나 되돌리지 않음).

| 단계 | 하는 일 | 고정 기준 | 시간(참고) |
|---|---|---|---|
| `check` | OS, 아키텍처, 드라이버, GPU 메모리, 디스크, RAM, 필요한 명령 점검 | | 수 초 |
| `micromamba` | micromamba 2.9.0을 받아 SHA-256 확인 후 `~/.local/bin/micromamba` | `scripts/bootstrap.sh` `MM_SHA` | 수 초 |
| `repos` | 외부 저장소 6개를 고정 커밋으로 받고(부분 복제), 서브모듈 초기화, 프로젝트 패치 적용 | `envs/repos.lock.json`, `envs/patches/` | 수 분 |
| `assets` | Unitree 공식 USD·보행 정책 묶음(1.3 GB)을 받아 해시 확인 후 `repos/unitree_sim_isaaclab/assets/`에 풂 | `envs/assets.lock.json` | 다운로드 속도에 따라 |
| `envs` | conda 환경 4개(`sim`, `teleop`, `data`, `cad`). `--with-vla`면 GR00T 로더 환경도 | `scripts/setup_envs.sh`, `envs/constraints_*.txt` | 처음 30–60분(Isaac Sim 약 18 GB) |
| `eula` | EULA 동의 기록(`--accept-nvidia-eula`일 때만) | | |
| `hand` | Inspire RH56E2 원자료 232개 파일을 고정 커밋에서 받아 git blob 해시로 확인 → 손 npz → 로봇 USD | `envs/hand_reference.lock.json` | 1–3분 |
| `certs` | PICO(WebXR) 접속용 로컬 HTTPS 인증서(`certs/`, 저장소에 안 올라감) | | 수 초 |
| `test` | 단위 테스트(시뮬 없이) | | 수 초 |
| `smoke` | 시뮬을 화면 없이 띄워 DDS 상태 토픽이 오는지 확인하고 끔 | | 1–3분 |

설치되는 곳:

| 경로 | 내용 |
|---|---|
| `.conda/envs/g1_inspire_sim` | Python 3.11, Isaac Sim 5.1.0.0, torch 2.7.0+cu128, Isaac Lab(80094be, editable), unitree_sim_isaaclab, unitree_sdk2py, cyclonedds 0.10.x |
| `.conda/envs/g1_inspire_teleop` | Python 3.10, pinocchio 3.1.0, xr_teleoperate(televuer, teleimager), unitree_sdk2py, pygame |
| `.conda/envs/g1_inspire_data` | Python 3.11, LeRobot 0.3.3(dataset v2.1), torch 2.7.0+cu128 |
| `.conda/envs/g1_inspire_cad` | Python 3.12, numpy·scipy·trimesh·opencv (손 원자료 → npz) |
| `.conda/envs/g1_inspire_vla` | (선택) Python 3.12, Isaac-GR00T N1.7 로더만 |
| `repos/` | IsaacLab, unitree_sim_isaaclab(+패치, +공식 에셋), unitree_sdk2_python, xr_teleoperate, Isaac-GR00T, unitree_lerobot(참고용) |
| `data/reference/inspire_ftp/`, `data/assets/` | 손 원자료와 빌드 결과 |
| `.cache/` | pip 캐시, 임시 파일, EULA 기록 |

환경별 실제 설치 목록은 `envs/<env>.yaml`, `*.explicit.txt`, `*.pip-freeze.txt`에 있습니다(만든 PC에서 `scripts/export_envs.sh`로 내보낸 것).

## 3. 실행 확인

```bash
scripts/run_sim.sh                     # GUI. 첫 실행은 셰이더 캐시 때문에 몇 분 걸림. 창을 한 번 클릭해야 렌더링 시작
scripts/run_keyboard.sh                # 다른 터미널: 키보드로 보행·팔·손 (README.md 키 표)
scripts/run_mock_teleop.sh scripted_walk --desk 0.65 --record    # 0.65 m 책상으로 띄운 시뮬에서 자동 보행→파지→들기
```

- GPU는 기본으로 PCI 순서상 첫 번째를 씁니다. 다른 GPU는 `--gpu 5060ti`, `--gpu 4090`(이름 일부), `--gpu GPU-<uuid>` 또는 `export G1IV_GPU=...`.
- DDS는 도메인 1, 인터페이스 `lo`로 고정입니다. 같은 PC에서 다른 Unitree 시뮬레이터(도메인 1)를 동시에 돌리지 마세요.
- 기본 장면은 0.55 m 책상 + 원통 세 개(빨강·파랑·초록)입니다. 예전 단일 원통 장면은 `--scene-set extra_objects.enabled=false`, 책상 높이는 `--scene-set desk.top_surface_height=0.65`.

## 4. 선택 구성

- **GR00T 로더 환경**: `scripts/bootstrap.sh envs --with-vla`. 이 환경은 변환 데이터가 GR00T N1.7 로더로 읽히는지 확인하는 용도입니다. N1.7 파인튜닝은 GPU당 약 35 GB가 필요해 서버에서 해야 합니다.
- **ACT 학습용 데이터 변환**(`tools/make_act_dataset.py`, LeRobot v3.0): LeRobot 0.4 이상 환경이 필요합니다. 2026-11-18 ACT 학습에 쓴 환경은 Python 3.12.3, torch 2.11.0+cu128, lerobot 0.6.1이었습니다. 이 저장소의 부트스트랩은 이 환경을 만들지 않습니다. 예: `micromamba create -r .conda -n g1_inspire_act -c conda-forge python=3.12 pip` 후 그 환경의 pip으로 `torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128`, `lerobot==0.6.1`.
- **PICO 4 Ultra 텔레옵**: `docs/PICO_CONNECTION.md`.

## 5. 저장소에 없는 것

- 녹화·변환 데이터(`data/raw`, `data/smoke`, `data/processed`)와 로그(`logs/`). 공개 데이터셋은 Hugging Face에 있습니다(README.md).
- Inspire RH56E2 손 원자료와 그 파생 메시(`data/reference/`, `data/assets/`): 제3자 파일이라 재배포하지 않고 `hand` 단계가 각 PC에서 받아 만듭니다(docs/HAND_RH56E2.md §2).
- 인증서·키(`certs/`), conda 환경, 외부 저장소, 공식 에셋.

## 6. 검증 상태

2026-11-18, 이 저장소를 만든 PC(Ubuntu 24.04.5, 드라이버 580.178.04, RTX 4060)에서 첫 커밋을 빈 폴더(`/home/jammin/g1iv_repro_test`)에 새로 복제해 단계별로 실행했습니다.

| 단계 | 결과 | 비교 |
|---|---|---|
| `check` | 통과 | |
| `micromamba` | 릴리스를 받아 SHA-256 확인 후 설치(임시 경로) | 2.9.0 |
| `repos` | 1분 48초, 6개 저장소·서브모듈·패치 | 원래 저장소와 파일 차이 0(실행 중 생기는 IK 캐시 파일 1개 제외) |
| `assets` | 2분, 1.31 GB 다운로드·해시 확인 | 423개 파일 차이 0 |
| `envs` | 22분 47초, pip(pypi.nvidia.com)로 Isaac Sim 5.1 포함 4개 환경, 24 GB | 핵심 버전 동일. 고정하지 않은 하위 의존성 몇 개만 최신 패치 버전(PyJWT, certifi, grpcio, regex 등) |
| `hand` | 원자료 232개 해시 확인, npz·USD 빌드 | USD SHA-256 동일. npz는 숫자 배열 528개 동일(경로 문자열 차이는 이후 상대 경로로 고쳐 PC와 무관하게 바이트까지 같음) |
| `certs`, `test` | 단위 테스트 44개 통과 | |
| `smoke` | 시뮬이 DDS 상태 토픽에 응답 | |
| 기능 확인 | 같은 복제본으로 0.65 m 책상 + 세 원통, 파랑·초록을 자동 보행→파지→들기 2/2 성공 | 다른 원통 이동 0 |

확인하지 못한 것:
- **다른 PC**: 드라이버·시스템 라이브러리가 이미 있는 같은 PC에서 한 시험입니다. 새로 설치한 Ubuntu에서는 드라이버 설치 뒤 `check`가 알려 주는 항목을 먼저 맞추세요.
- **GUI 실행**: 복제본에서는 화면 없는 실행만 확인했습니다(원래 작업공간에서는 GUI 실행을 계속 써 왔습니다).
- **선택 구성**: `--with-vla` 환경, ACT 환경, PICO 실기는 복제본에서 다시 만들어 보지 않았습니다.
- **하위 의존성 버전**: 고정하지 않은 하위 의존성은 시간이 지나면 더 올라갈 수 있습니다. 문제가 생기면 `envs/<env>.pip-freeze.txt`의 버전으로 맞추세요.

## 7. 문제 해결

| 증상 | 원인과 해결 |
|---|---|
| `preflight failed: free disk` | 저장소를 여유 있는 파티션에 받으세요(같은 파티션에 환경·캐시가 생깁니다). |
| `GPU '4060' not found` | 다른 PC의 GPU 이름입니다. `--gpu` 없이 실행하거나 `--gpu <이름 일부>`. |
| `NVIDIA Isaac Sim EULA not accepted` | 라이선스를 읽고 `scripts/bootstrap.sh eula --accept-nvidia-eula`. |
| `... is at <sha>, lock wants <sha>` | `repos/<이름>`이 다른 커밋입니다. 스크립트는 기존 체크아웃을 바꾸지 않으니 폴더를 옮기고 다시 실행하세요. |
| `assets ... does not match the lock` | 다른 버전의 공식 에셋이 이미 풀려 있습니다. `repos/unitree_sim_isaaclab/assets`를 옮기고 다시 실행하세요. |
| cyclonedds 빌드 실패 | `sim` 환경은 conda-forge의 CycloneDDS C 라이브러리 0.10.5로 파이썬 cyclonedds 0.10.2를 빌드합니다. `/opt/ros`의 라이브러리를 쓰지 않습니다. 셸에 `CYCLONEDDS_HOME`을 따로 지정했다면 지우세요. |
| 첫 시뮬 실행이 오래 걸림 | Isaac Sim이 셰이더·확장 캐시를 만듭니다(수 분). 두 번째부터 빨라집니다. |

고정값을 바꾼 뒤에는(외부 저장소 커밋, 패치, 환경) 만든 PC에서 `scripts/export_envs.sh`를 실행해 `envs/`를 갱신하고 커밋하세요.
