# CONTROL_OWNERSHIP

관절마다 **최종 목표값을 쓰는 주체는 하나**입니다. 시뮬 쪽 최종 기록은 공식 `action_provider_wh_dds.py`의
`get_action()`이 매 제어 스텝마다 `set_joint_position_target(full_action)`으로 씁니다. 아래 표는 그 `full_action`의
각 구간이 어디에서 오는지 정리한 것입니다.

| 관절 | 개수 | 최종 목표를 정하는 곳 | 명령 경로 | 주기 |
|---|---|---|---|---|
| 다리 (hip pitch/roll/yaw, knee, ankle pitch/roll × 2) | 12 | 공식 보행 정책 `assets/model/policy.onnx` (onnxruntime). 출력 × 0.25 + 기본자세, 5스텝 지연 버퍼 | sim 프로세스 내부 | 50 Hz (physics 200 Hz, decimation 4) |
| 허리 yaw/roll/pitch | 3 | 기본자세로 고정 (`default_waist_positions`) — 외부 입력 없음 | sim 내부 | 50 Hz |
| 팔 (shoulder ×3, elbow, wrist ×3) × 2 | 14 | **허브 1개**: IK → 변화량 제한 → 공식 `G1_29_ArmController` | DDS `rt/lowcmd` `motor_cmd[15:29].q` (CRC 검사) → shm `dds_robot_cmd` → `full_action[arm]` | 허브 50 Hz, lowcmd 250 Hz, sim 50 Hz |
| Inspire 구동 관절 (pinky/ring/middle/index proximal, thumb pitch/yaw) × 2 | 12 | **허브 1개**: OPEN/CLOSE → 프리셋 → 속도 제한 | DDS `rt/inspire/cmd` (MotorCmds_ 12, q 0..1, 1=펴기) → sim에서 rad 변환 | 허브 50 Hz |
| Inspire 연동 관절 (intermediate ×4, thumb intermediate, thumb distal) × 2 | 12 | 공식 sim이 구동 관절에서 배율로 계산 (손가락 ×1.0, 엄지 중간 ×1.5, 엄지 끝 ×2.4) | sim 내부 | 50 Hz |
| 머리 | 0 | G1 29-DoF에는 머리 관절 없음 (머리 카메라는 `d435_link`에 고정) | — | — |
| 보행 속도 명령 | (4) | **허브의 LocoArbiter 1개** (keyboard 또는 XR 스틱 중 하나만) | DDS `rt/run_command/cmd` "[vx, vy, wz, height]" → 정책 관측의 command 항 | 송신 100 Hz, 정책이 읽을 때마다 0으로 초기화 |

보행 정책과 상체의 결합 방식은 공식 코드 그대로입니다.
- 정책의 관찰에는 다리 12개와 팔 14개의 관절 위치·속도, 그리고 29차원 직전 행동이 들어갑니다(10프레임 이력). 팔 목표는 정책이 관찰하는 "직전 행동"에 섞여 들어가지만, 팔의 최종 목표는 DDS 값이 그대로 쓰입니다.
- 관절 인덱스를 새로 자르거나 붙이지 않았습니다. 이름으로 찾고, 시작할 때 USD 관절 53개에서 누락이 없는지 검사합니다(`src/g1iv_sim/ext.py`).

## 쓰는 프로세스가 둘이 되지 않게 하는 장치

- `rt/lowcmd`, `rt/inspire/cmd`, `rt/run_command/cmd`를 쓰는 것은 허브 프로세스 하나뿐입니다. `_hub_launch.sh`는 허브가 이미 떠 있으면 시작을 거부합니다.
- 공식 `send_commands_keyboard.py`(`run_keyboard.sh --official`)와 공식 `teleop_hand_and_arm.py`는 비교용입니다. **허브와 동시에 실행하지 마세요.** 둘 다 같은 토픽에 쓰기 때문입니다.
- 보행 입력 소스는 `configs/teleop.yaml`의 `loco_source`(또는 `--loco-source`) 하나만 활성화됩니다. 키보드와 XR 스틱 입력을 합산하지 않습니다.
- 앞으로 VLA 상체 정책을 붙일 때는 허브의 `arm_source`에 새 소스를 추가하면 됩니다. 그러면 텔레옵 대신 같은 제한·기록 경로를 거칩니다.

## 좌표계

| 이름 | 정의 |
|---|---|
| IK 프레임 | 팔 IK 축소 모델의 루트(pelvis, 허리 관절은 0으로 잠김). x 전방, y 왼쪽, z 위. **몸통에 붙어 있으므로 걸을 때 팔 목표가 함께 이동합니다** (세계 좌표에 고정되지 않음) |
| EE | `L_ee` / `R_ee` = `*_wrist_yaw_link` 원점 + x 0.05 m (공식 IK) |
| XR 원시값 | OpenXR(+Y 위, −Z 전방). 공식 televuer가 Robot 기준으로 바꾸고, **head_yaw 기준**(머리 yaw만 보정하고 pitch/roll은 무시)으로 바꾼 뒤 원점을 head → waist(+0.15 x, +0.45 z)로 옮깁니다 |
| 상대 매핑 | 연결(engage)·recenter·clutch 해제·입력 복귀 시 `T_ee0`(현재 목표 자세의 FK)과 `T_ctrl0`를 다시 잡고, 이후 `p = p_ee0 + s·(p_ctrl − p_ctrl0)`, `R = (R_ctrl R_ctrl0ᵀ) R_ee0`로 계산합니다. 헤드셋의 시작 원점을 로봇 절대 목표로 쓰지 않습니다 |
| base 상태 | sim `root_pos_w/root_quat_w`, 속도는 body frame(`root_lin_vel_b`, `root_ang_vel_b`) |

## 제한과 실패 처리

| 상황 | 동작 |
|---|---|
| IK 오차 > 3 cm, 해가 NaN, 예외 | 직전 유효 목표 유지(`hold_last_valid`), 실패 횟수를 표시·기록 |
| 목표가 연결 시점 기준 0.45 m 이상 | 0.45 m로 자름 |
| 관절 속도 | 허브에서 **시뮬 시간 기준** 1.5 rad/s로 제한하고 URDF 한계로 자름(공식 제어기는 sim 모드에서 제한하지 않음). 이 PC의 시뮬은 실시간의 약 0.3배라, 틱의 실제 시간 간격에 `rt/g1iv/sim_status`의 RTF를 곱해 씀(손 개폐 속도도 같음) |
| 허브 시작 | 공식 `G1_29_ArmController`는 생성자 안에서 목표 0으로 250 Hz 발행을 시작합니다(팔이 0 자세로 순간이동). 허브는 하위 클래스로 발행 스레드를 붙잡아 두었다가 목표를 측정 자세로 맞춘 뒤 풀어 줌(첫 명령 = 현재 자세, 실측 차이 0) |
| XR 입력 끊김(포즈 0.5 s 이상 갱신 없음) | 팔: 마지막 목표 유지, 손: 현재 OPEN/CLOSE 유지(끊김을 "트리거 놓음"으로 해석하지 않음), 보행: 0, 복귀 시 자동 재앵커(점프 없음) |
| 트리거 전이 확인 | 끊김 판정(0.5 s) 전의 "트리거 0"도 손을 열 수 있으므로, 전이는 50 ms 유지 **그리고** 그 뒤 새 컨트롤러 포즈가 도착해야 확정(`hands.trigger_confirm_s`). 포즈가 50 ms 넘게 멈추면 대기 중인 전이를 버림 |
| 보행 명령 크기 | 공식 보행 정책은 작은 명령에 발을 떼지 않음(측정: 횡 0.35 m/s·회전 0.8 rad/s·후진 0.26 m/s에서 몸만 흔들리고 제자리 복귀). 키 입력과 스틱 최대값을 공식 키보드 범위로 둠: 전진 0.5(XR 0.6), 후진 0.6, 횡 0.5, 회전 1.57. 스틱을 조금만 기울이면 걷지 않을 수 있음 |
| 허브 루프 0.2 s 이상 정지 | 보행 송신 스레드가 0을 보냄 |
| 허브 종료·충돌 | 보행: 정책이 읽을 때마다 0으로 초기화하므로 다음 스텝에 정지. 팔·손: sim 공유메모리의 마지막 목표 유지 |
| 시뮬 리셋 | 로봇·물체 기본 자세(+시드 노이즈), 정책 관측 이력과 지연 버퍼 초기화, 공유메모리 명령을 기본 팔 자세·OPEN·보행 0으로 덮어씀, 녹화 종료. 허브는 `reset_count` 변화를 보고 연결 해제, 손 OPEN, 팔 목표를 측정값으로 맞춤 |
| 시뮬 시작 | 공식 코드는 lowcmd가 없으면 팔 목표가 0(팔이 내려감)입니다. 확장이 켜지면 기본 팔 자세를 먼저 넣습니다(SETUP_REPORT의 편차 목록 참고) |
