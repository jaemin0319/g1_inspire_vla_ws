# g1_inspire_vla_ws

Unitree G1(29-DoF, EDU Plus 몸체) + 좌우 Inspire RH56E2(RH56DFTP) 손 + PICO 4 Ultra 컨트롤러 텔레옵을 Isaac Sim 5.1 / Isaac Lab에서 돌리고,
시연을 VLA(GR00T) 학습용 데이터로 남기는 작업공간입니다.

## 설치 (Ubuntu 24.04, NVIDIA GPU)

```bash
git clone https://github.com/jaemin0319/g1_inspire_vla_ws.git && cd g1_inspire_vla_ws
scripts/bootstrap.sh check                         # OS·드라이버·GPU·디스크(45 GB 이상)·RAM 점검, 바꾸는 것 없음
scripts/bootstrap.sh all --accept-nvidia-eula      # NVIDIA Isaac Sim EULA를 읽고 동의할 때만 이 옵션을 붙입니다
```

sudo 없이 작업공간 안에만 설치합니다(micromamba만 `~/.local/bin`). 외부 저장소·에셋·환경은 모두 커밋·리비전·해시로 고정돼 있습니다. 단계별 설명, 소요 시간, 문제 해결은 **[docs/INSTALL.md](docs/INSTALL.md)**.

공개 데이터셋(이 작업공간으로 자동 생성, LeRobot v2.1 + GR00T `modality.json`):
[단일 원통 15개](https://huggingface.co/datasets/jammin0319/g1_inspire_walk_grasp_desk065_scripted_v1) ·
[세 원통 색 지정 90개](https://huggingface.co/datasets/jammin0319/g1_inspire_walk_pick_3color_desk065_scripted_v1) ·
[먼 출발 보행·파지 55개](https://huggingface.co/datasets/jammin0319/g1_inspire_walk_far_grasp_desk065_scripted_v2)

- **현재 상태는 [docs/SETUP_REPORT.md](docs/SETUP_REPORT.md)부터 보세요.** 각 항목을 실제로 실행했는지, 모의 입력인지 실제 PICO인지는 [docs/VALIDATION.md](docs/VALIDATION.md)에 구분해 두었습니다.

## 구성 한눈에

```
sim 프로세스 (g1_inspire_sim, Python 3.11)            hub 프로세스 (g1_inspire_teleop, Python 3.10)
 official sim_main.py + 프로젝트 확장(G1IV_EXT=1)        입력: pico | mock | scripted | keyboard
  ├ 하체: 공식 보행 정책 policy.onnx (50 Hz)              ├ 공식 televuer 좌표 변환
  ├ 팔: rt/lowcmd[15:29]   ◀───────────────────────────── ├ 공식 G1_29_ArmIK → 변화량 제한 → 공식 G1_29_ArmController
  ├ 손: rt/inspire/cmd (DFX 12ch) ◀──────────────────────  ├ OPEN/CLOSE (히스테리시스, 끊김 시 유지)
  ├ 보행 명령: rt/run_command/cmd ◀──────────────────────  ├ 보행 입력 1개만 선택 (keyboard XOR XR 스틱)
  ├ 카메라: 헤드 스테레오 2 + 좌우 손목 640×480 (렌즈 왜곡)  └ rt/g1iv/teleop_meta (원 입력·제한 전/후 목표)
  └ 녹화기: (관측_t, 적용 목표_t) 쌍을 시뮬 시간 격자로 저장  ◀── rt/g1iv/control (녹화·리셋)
                       DDS 도메인 1 (공식 시뮬 값), 인터페이스 lo (연구실 망으로 나가지 않음)
```

## 명령

| 하는 일 | 명령 |
|---|---|
| 시스템 점검 | `scripts/check_system.sh` |
| 새 PC 설치 | `scripts/bootstrap.sh check\|all\|repos\|assets\|envs\|hand\|certs\|test\|smoke` (docs/INSTALL.md) |
| env 생성/보완 | `scripts/setup_envs.sh sim\|teleop\|data\|cad\|vla\|all` |
| 시뮬레이션 | `scripts/run_sim.sh` (GUI, 첫 번째 GPU. `--gpu 4060\|5060ti\|<이름 일부>` 또는 `G1IV_GPU`. 창을 한 번 클릭해야 렌더링 시작) · `--demo`(실시간 시연: 카메라 센서 없음·녹화 불가, docs/VALIDATION.md 성능) · `--demo-cams`(시연 + 카메라 패널, RTF 약 0.55) · `--headless` · `--gpu 5060ti` · `--official-task` · 카메라 리그 `--rig-set head.right_eye=false` / `--rig-set wrist.mount=inspire_ring_mono160`(이전 손목: Inspire 링 + 160° 단안) / `--rig-set use_rig=false`(공식 프리셋) · 손 `--scene-set robot.hand_model=inspire_dfx_official`(공식 손) / `--scene-set robot.hand_colorway=e2_black` · 관찰 시점 `--scene-set "cameras.observer.pos=[1.75,-1.05,1.30]" --scene-set "cameras.observer.look_at=[0.90,-0.12,0.85]"` |
| 키보드 보행/손/팔 | `scripts/run_keyboard.sh` (허브 창에 포커스가 있을 때만 키 입력) |
| 모의 텔레옵 | `scripts/run_mock_teleop.sh dry_run\|dropout\|recenter\|walk_xr\|walk_seq\|hold` |
| 스크립트 파지 테스트 | 0.70 m 책상(`run_sim.sh --scene-set desk.top_surface_height=0.70`)에서 `scripts/record_episode.sh reset --robot-pose 0.64 -0.12 0`(책상 앞, RH56E2 손. 공식 손이면 0.70 -0.05) → `scripts/run_mock_teleop.sh scripted [--record]` (data/smoke, source=scripted) |
| 자동 보행→파지→들기 | `scripts/run_mock_teleop.sh scripted_walk [--desk 0.65] [--record]` (책상 높이에 맞는 `configs/scripted_walk_grasp_desk<cm>.yaml` 자동 선택: 0.65·0.70 동작, 0.55 미동작) · 무작위 반복 생성 `tools/batch_walk_grasp.py -n 20 --record --desk 0.65` · 결과 보기 `tools/analyze_walk_grasp.py logs/teleop_runs/<run>.json` (docs/AUTO_WALK_GRASP.md) |
| 시뮬 상태 보기 | `PYTHONPATH=src .conda/envs/g1_inspire_teleop/bin/python -m g1iv_teleop.probe` (읽기 전용) · `-m g1iv_teleop.status_log --seconds 30 --out x.json` |
| 보행 명령 직접 시험 | `tools/loco_cmd_test.py -- "0,0,1.57" "0,0.5,0"` (공식 키보드와 같은 형식) · 분석 `tools/analyze_walk.py` |
| GR00T 로더 확인 | `PYTHONPATH=src .conda/envs/g1_inspire_vla/bin/python -m g1iv_data.check_gr00t data/processed/<name>_lerobot_v21` |
| PICO 대기 | `scripts/run_pico_teleop.sh [--loco-source xr] [--diag]` |
| 녹화 제어 | `scripts/record_episode.sh start\|stop --outcome success\|abort\|reset\|status\|preview` |
| 세 원통 목표 지정 | `scripts/record_episode.sh reset --target red\|blue\|green` (장면 `extra_objects`, 위치 고정) · 색별 자동 생성 `tools/batch_walk_grasp.py -n 120 --targets red,blue,green --per-target 30 --fixed-objects --record --desk 0.65` (docs/AUTO_WALK_GRASP.md §6) |
| 검증 | `scripts/validate_dataset.sh [data/raw/...]` · `--lerobot data/processed/<name>_lerobot_v21` |
| 변환 | `scripts/convert_dataset.sh --name <name> data/raw/<날짜> [--include-outcomes success]` |
| 허깅페이스 업로드 (요청 시만) | `.conda/envs/g1_inspire_data/bin/python tools/push_dataset_hf.py data/processed/<name>_lerobot_v21 --repo-id <user>/<name> --card-file docs/hf_cards/<카드>.md [--public]` (기본 비공개) |
| env 고정 내보내기 | `scripts/export_envs.sh` |
| 종료 | 각 창에서 Ctrl+C, 또는 `scripts/stop_all.sh` (이 스크립트들이 띄운 PID만 종료) |

## 키보드 (허브 창)

`W/S` 전후 · `A/D` 좌우 · `Q/E` 회전 (공식 보행 정책은 작은 명령에 발을 떼지 않아 키 입력은 공식 최대 범위를 씀) · `SPACE` 즉시 정지 (키를 떼면 0으로 감속, 창이 포커스를 잃으면 즉시 0)
`R` 팔 연결/해제 · `C` recenter · `Z/X/B` 왼손/오른손/양손 OPEN↔CLOSE · `TAB` 팔 선택 · `I/K J/L U/O` 1 cm 이동
`1` 녹화 시작 · `2` 성공으로 종료 · `3` 실패로 종료 · `4` 폐기 · `0` 환경 리셋 · `ESC` 종료

## 설정 파일

`configs/scene_desk.yaml` 책상(1.5×0.7 m, 상판 높이 0.55 m: 2026-11-17 0.70에서 15 cm 낮춤)·바닥·물체·조명·로봇 시작 위치·관찰 카메라 · `configs/camera_rig.yaml` 정책/녹화 카메라 4대(헤드 스테레오 + 손목 RealSense D405 on Unitree D405 마운트 C, 실측 렌즈, 브래킷; [docs/CAMERA_RIG.md](docs/CAMERA_RIG.md)) ·
`configs/hand_rh56e2.yaml` Inspire RH56E2 손(범위·연동·배색, 빌드: [docs/HAND_RH56E2.md](docs/HAND_RH56E2.md)) · `configs/teleop.yaml` 입력 소스·속도·버튼/키 매핑·제한 · `configs/hands_inspire.yaml` OPEN/CLOSE 프리셋 ·
`configs/record.yaml` 녹화 주기·저장 위치 · `configs/scripted_grasp.yaml` 스크립트 파지 경로(0.70 m, 제자리) · `configs/scripted_walk_grasp_desk*.yaml` 자동 보행→파지 계획

## 테스트

`PYTHONPATH=src .conda/envs/g1_inspire_teleop/bin/python -m pytest -q tests/` (38개)

종료 참고: Isaac Sim 5.1 + 공식 sim_main은 정리(`cleanup completed`)가 끝난 뒤 `simulation_app.close()`에서 돌아오지 않습니다(프로젝트 확장 없이도 동일). `run_sim.sh`/`stop_all.sh`가 그 시점 5초 뒤 종료시키고 정상 종료(0)로 보고합니다. 녹화는 그 전에 닫힙니다.
