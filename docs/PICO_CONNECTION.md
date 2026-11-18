# PICO_CONNECTION

대상 장비는 PICO 4 Ultra(일반판)와 좌우 컨트롤러입니다. 공식 xr_teleoperate의 televuer(WebXR, Vuer 0.0.60)를
**컨트롤러 모드**로 씁니다. 손가락 hand tracking은 쓰지 않습니다. **아직 실제 장비로는 한 번도 연결해 보지 않았습니다.**

## 1. PC 쪽

```bash
scripts/run_sim.sh                 # 시뮬 (이미지 서버 ZMQ 55555-7, WebRTC 60001-3 포함)
scripts/run_pico_teleop.sh --diag  # 허브: WebXR 서버 :8012 + 접속 URL 출력 + 원시 버튼 진단 로그
```

- PC IP는 런처가 매번 `ip -4 addr`에서 읽어 출력합니다. IP를 코드에 고정하지 않았습니다. 2026-11-11 기준 값은 eth0 `163.180.131.92`(경희대 유선 고정 IP)와 tailscale0 `100.88.36.103`입니다.
- 헤드셋 브라우저에서 엽니다: `https://<PC_IP>:8012/?ws=wss://<PC_IP>:8012`
- 사용하는 포트(TCP)는 8012(WebXR HTTPS/WSS), 60001(머리 영상 WebRTC 시그널링, HTTPS)이고, WebRTC 미디어는 UDP를 씁니다(aiortc, 임시 포트). 55555-55557(ZMQ 영상)은 같은 PC 안에서만 씁니다. 방화벽이 켜져 있다면 같은 망 안에서 8012/tcp와 60001/tcp를 허용하세요(`sudo ufw status`로 확인. 이 작업에서는 방화벽 설정을 바꾸지 않았습니다).
- PICO와 PC가 같은 L2/L3 망에 있어야 합니다. 학교 유선망에 PICO Wi-Fi가 닿지 않으면, 연구실 공유기 또는 Tailscale(PICO에 앱 설치 필요) 중 하나를 골라야 합니다(미검증).

## 2. 인증서

- `scripts/setup/make_certs.sh`가 로컬 루트 CA(`certs/rootCA.pem`)와 서버 인증서(`certs/cert.pem`, SAN = localhost, 127.0.0.1, 현재 PC의 모든 IPv4)를 만듭니다. IP가 바뀌면 `--force`로 다시 만드세요.
- 시뮬(teleimager)과 허브(televuer)는 `XR_TELEOP_CERT`/`XR_TELEOP_KEY` 환경변수로 이 파일을 씁니다(공식 방식처럼 `~/.config`에 복사하지 않음).
- 헤드셋에서 경고를 피하려면 `certs/rootCA.pem`을 PICO에 신뢰 인증서로 설치하거나, 브라우저 경고 화면에서 "계속"을 누르세요. `:8012`와 `:60001` 두 주소 모두 한 번씩 허용해야 영상이 나옵니다(공식 문서 기준).
- `certs/`는 Git 추적에서 제외되어 있습니다. 개인키를 다른 곳에 올리지 마세요.

## 3. 버튼 매핑 (configs/teleop.yaml `buttons`)

| 동작 | 기본 입력 | 비고 |
|---|---|---|
| 팔 연결/해제 | 왼쪽 X (televuer `left_aButton`) | 연결 순간의 로봇 목표 자세에 상대 오프셋을 잡음 |
| recenter | 왼쪽 Y (`left_bButton`) | 양팔 앵커를 다시 잡음 (점프 없음) |
| clutch | 왼쪽/오른쪽 grip(squeeze), 누르고 있는 동안 | 누르는 동안 그 팔은 멈추고, 놓으면 다시 앵커 |
| 손 CLOSE | 왼쪽/오른쪽 trigger | 누르면 CLOSE, 놓으면 OPEN (0.6 / 0.4 히스테리시스) |
| 녹화 시작 / 성공 종료 | 오른쪽 A | 녹화 중이 아니면 시작, 녹화 중이면 success로 종료 |
| 실패 종료 | 오른쪽 B | 녹화 중일 때만 |
| 비상 정지 | 양쪽 스틱 동시 클릭 | 보행 0, 팔 연결 해제(목표 유지) |
| 보행 (`--loco-source xr`) | 왼쪽 스틱 전후·좌우, 오른쪽 스틱 좌우 = 회전 | dead zone 0.15, 최대 0.5 m/s · 0.3 m/s · 0.8 rad/s, 0.3 s 갱신 없으면 0 |

- 공식 xr_teleoperate의 실제 로봇 모션 모드는 오른쪽 A를 종료로, 양 스틱 클릭을 damp로 씁니다. 이 허브는 그 프로그램을 같이 실행하지 않으므로 충돌하지 않습니다. 다만 **같은 버튼의 의미가 다르니** 두 프로그램을 번갈아 쓸 때 주의하세요.
- **버튼 이름은 televuer가 보고하는 이름입니다.** PICO에서 X/Y가 실제로 `left_aButton`/`left_bButton`으로 들어오는지는 장비로 확인해야 합니다. `--diag` 로그와 허브 창에서 눌렀을 때 바뀌는 필드를 확인하고, 다르면 YAML만 고치면 됩니다.

## 4. 처음 정렬 순서

1. 시뮬과 허브를 실행하고, 허브 창 첫 줄의 `status_age`가 0.5 s 미만인지 확인합니다.
2. 헤드셋에서 URL을 열고 VR 권한(immersive-vr, 컨트롤러)을 허용합니다. 허브 창의 `input: OK`를 확인합니다.
3. 로봇 팔이 편한 자세(기본 자세)일 때 양손 컨트롤러를 몸 앞 편한 위치에 두고 **왼쪽 X**를 눌러 연결합니다. 이때 팔은 움직이지 않아야 합니다.
4. 작게 움직여 좌우·전후·상하 방향이 맞는지 봅니다. 방향이 틀리면 연결을 해제하고 docs/PICO_PENDING_CHECKLIST.md 2번 항목대로 기록해 주세요.
5. 트리거로 손 OPEN/CLOSE, 오른쪽 A로 짧은 녹화, `scripts/record_episode.sh preview`로 확인합니다.
