# HAND_RH56E2: Inspire RH56E2 (RH56DFTP) 손과 G1 몸체 선택

작성 2026-11-12. 설정은 `configs/hand_rh56e2.yaml`이고, 선택 스위치는 `configs/scene_desk.yaml`의 `robot.hand_model`과 `robot.hand_colorway`입니다.
자산은 두 단계로 만듭니다. `tools/cad/make_inspire_e2_hand.py`가 npz를, `tools/build_inspire_e2_usd.py`가 로봇 USD를 만듭니다.
시뮬 연결 코드는 `src/g1iv_common/hand_model.py`, `src/g1iv_sim/desk_task.py`, `src/g1iv_sim/ext.py`에 있고, 테스트는 `tests/test_hand_model.py`입니다.
내려받은 원자료(`data/reference/inspire_ftp/`)와 생성 자산(`data/assets/`)은 git에서 제외합니다.

## 1. 결론

| 항목 | 이전 | 지금 (기본) |
|---|---|---|
| 손 | Unitree 공식 자산의 Inspire 손 (RH56DFX 계열 URDF, 링크 `Link11..22`) | **Inspire RH56E2 = RH56DFTP 계열**. Inspire가 만든 URDF(2026-01-05) 형상, 질량, 관절 범위, 연동 비율 |
| 손 무게 | 0.54 kg (DFX 공칭) | 0.899 kg/손 (Inspire URDF. 사양표 790±10 g + 플랜지 어댑터) |
| 관절 범위 (q=0..1 ↔ 각) | 손가락 0–1.7, 엄지 굽힘 0–0.5, 엄지 회전 −0.1–1.3 rad | 손가락 0–1.4381, 엄지 굽힘 0–0.62(35.5°), 엄지 회전 0–1.658(95°) rad. E2 매뉴얼 75–170°, 28.5–64°와 일치 |
| 수동 관절 비율 | 손가락 1.0, 엄지 1.5 / 2.4 | 손가락 1.0843, 엄지 0.8392 / 0.7477 (Inspire URDF `<mimic>`) |
| 배색 | 흰 손 위에 링크 단위로 색을 덮음 | 부품·면 단위 6가지 라벨, Inspire 사진 기준 (§4) |
| 몸체 | G1 29자유도 rev 1.0 | **그대로**. 이것이 G1 EDU Plus 몸체입니다 (§5) |

- 관절 이름, 링크 이름, 관절 순서는 공식 자산과 같습니다(빌드 검사 `joint_order_unchanged: true`). 그래서 공식 DDS(`rt/inspire/cmd`, 12채널, q 1=펼침), 공식 상태 발행(고정 인덱스 표), 녹화 데이터 형식, xr_teleoperate의 `inspire_dfx` 인터페이스가 그대로 동작합니다.
- 손가락 이름은 E2 → DFX 순서로 이렇게 대응합니다: `X_1`→`X_proximal`, `X_2`→`X_intermediate`, `little`→`pinky`, `thumb_1..4`→`thumb_proximal_base / thumb_proximal / thumb_intermediate / thumb_distal`. 관절은 `thumb_proximal_yaw`(회전), `thumb_proximal_pitch`(굽힘), `thumb_intermediate`, `thumb_distal`입니다.
- 이전 손으로 되돌리려면 `scripts/run_sim.sh --scene-set robot.hand_model=inspire_dfx_official`을 쓰면 됩니다.
- 배색을 바꾸려면 `--scene-set robot.hand_colorway=e2_black`을 쓰면 됩니다. 이 배색은 손가락이 검은 E2 제품 렌더입니다.

## 2. 원자료와 라이선스

| 자료 | 위치 (고정 커밋) | 라이선스 | 쓴 곳 |
|---|---|---|---|
| Inspire RH56E2 오른손 URDF + STL, 2026-01-05 | github.com/hychen-naza/isaaclab_env `RH56E2_R_2026_1_5` @ cccdf11 | 패키지 안 LICENSE "Copyright (c) 2025 INSPIRE-ROBOTS", BSD-3 형식 | 형상, 질량, 관절 (기준) |
| Inspire RH56E2 왼손/오른손 URDF, 2025-09 | github.com/hoon001218/contact_rich_manipulation_task_learning `Roas_provided_inspire_urdf/RH56E2 URDF` @ cdc4300 | 위와 같음 | 왼손 거울 검증, 2026 판과 비교 |
| Unitree G1 FTP 손 URDF + STL | github.com/unitreerobotics/unitree_ros `robots/g1_description` @ 725b59b | BSD-3-Clause | 장착 위치 검증 |
| 제품 사진 (색) | Inspire EN/CN 제품 페이지(RH56E2), robosavvy RH56DFTP 이미지 | 참고용 | 배색 (`data/reference/inspire_ftp/*.jpg`) |
| E2 매뉴얼 / DFTP 매뉴얼 | en.inspire-robots.com …/Dexterous-Hands-User-Manual-for-RH56E2_V1.0.0.pdf, CN 다운로드 센터 DFTP 매뉴얼 | – | E2 = DFTP 확인, 관절 범위 |

- **RH56E2와 RH56DFTP의 관계**: E2 매뉴얼 2.6.21에 "RH56DFTP-0L/0R, 2L/2R …"가 그대로 나오고, Inspire CN 사이트의 RH56E2 페이지 주소가 `rh56dftp-series`입니다. E2 URDF의 내보내기 로그도 `RH56DFTP-0R_R_R.SLDASM`을 읽습니다. 손가락 기구학은 Unitree FTP URDF와 같습니다. 엄지는 E2에서 바뀌었습니다(회전 95°, 굽힘 35.5°, 엄지 기저가 13.7 mm 위).
- Inspire 공식 사이트에는 E2/DFTP URDF가 없습니다. 위 두 저장소는 대리점이 배포한 Inspire 파일을 올려 둔 것으로 보입니다. 그래서 파생 메시는 내부 연구용으로만 쓰고, 저장소에는 올리지 않습니다(`data/assets/`는 git 제외). 공개하려면 Inspire 라이선스 고지를 붙이고 출처를 다시 확인해야 합니다.

## 3. 형상과 장착

- **기준 URDF**: Inspire RH56E2 오른손 2026-01-05 판입니다. 2025-09 판과 비교하면 메시는 중앙값 0.4 mm(점 샘플 간격 수준)로 같고, 2026 판은 베이스가 한 링크로 합쳐져 있고 질량값이 갱신됐습니다.
- **장착**: Unitree FTP와 같이 `wrist_yaw_link` x = 41.5 mm(손목 플랜지 면)에 붙입니다. E2 베이스 좌표계는 Unitree FTP 베이스 좌표계를 손 축 둘레로 +90° 돌린 것입니다. 검사 결과 FTP 베이스 메시와 E2 베이스 메시가 **중앙값 0.05 mm, p95 0.32 mm**로 겹칩니다. 공식 DFX 손의 y ∓3 mm 오프셋은 쓰지 않습니다(Unitree FTP URDF 값 0).
- **엄지 위치**(오른손, 손목 좌표계): E2 엄지 회전 관절은 (0.1241, 0.0210, 0.0269) m입니다. Unitree FTP는 (0.1104, 0.0210, 0.0269), DFX는 (0.1106, 0.0141, 0.0205)입니다. 같은 쪽(손바닥 쪽, 몸 안쪽)에 있고, 손 축 방향으로만 13.7 mm 더 나가 있습니다.
- **왼손**: 오른손을 손목 좌표계에서 y→−y 거울 반전해 만들고, 링크 좌표계는 다시 오른손 좌표계가 되게 했습니다. Inspire 왼손 URDF와 **중앙값 0.001 mm**(p99 6.3 mm, 베이스 국소 부위)로 일치합니다. 좌우 질량은 같습니다.
- **질량**: 링크별 Inspire URDF 값을 그대로 쓰고, 촉각 패드·손바닥 덮개는 붙은 링크에 합쳤습니다(평행축 정리). 총 0.8987 kg입니다.
- **충돌체**: 공식 자산과 같이 링크마다 convex hull 하나입니다.
- **관절 한계**: 구동 관절은 §1 범위를 씁니다. 수동 관절은 비율 × 구동 범위로 둡니다. Inspire URDF의 수동 관절 한계는 이 값보다 0.3–5 % 작은데, 시뮬은 정확히 비율대로 목표를 주므로 한계에 걸리지 않게 했습니다.

## 4. 배색 (사진 기준)

라벨은 6가지(검정, 흰 껍데기, 손등 덮개, 촉각 패드, 금속, 표시등)입니다. 규칙은 `tools/cad/make_inspire_e2_hand.py`의 `label_faces`에 있습니다.

| 부위 | DFTP 배색 (기본, `dftp`) | 근거 |
|---|---|---|
| 손등 덮개 (`plam_casing`) | 흰색 광택 | 모든 사진 |
| 하우징 (손바닥, 옆면, 손목 쪽) | 검정 | 모든 사진 |
| 손등 표시등 (y 0, z 52.5 mm 원형 돌출) | 은색 링 + 밝은 중앙 | 손등 사진 |
| 손목 플랜지·어댑터, 손가락 힌지, 핀, 나사, 엄지 회전부 | 은색 금속 | 손바닥 쪽 사진 |
| 손가락 등 (손등 쪽 면 + 손등 쪽 옆면) | 흰색 | DFTP 사진, Inspire RH56E2 실사 |
| 손가락 끝 캡 (끝마디 끝 18.5 mm) | 검정 | 같음 (끝마디의 약 1/3) |
| 손가락 손바닥 쪽, 촉각 패드 17개 | 검정 | 손바닥 사진 |
| 엄지: 큰 마디 몸통 / 양 끝판 | 검정 / 흰색 | 엄지 확대 사진 |
| 엄지: 중간 마디 측판 / 가운데 | 흰색 (은색 육각나사) / 검정 | 같음 |
| 엄지: 끝 캡 / 아래 띠 | 검정 / 흰색 | 같음 |

- `e2_black` 배색은 흰 껍데기 라벨만 검정으로 바꿉니다. 이것이 Inspire 영문 제품 페이지의 E2 렌더입니다(손가락·엄지 검정, 손등 덮개 흰색).
- 손등 "INSPIRE-ROBOTS" 글자는 메시에 없어 넣지 않았습니다.
- 미리보기(손등 / 손바닥 / 옆)는 `logs/captures/rh56e2/e2_colour_preview_dftp.png`에 있습니다.

## 5. G1 EDU Plus 요청에 대한 판단: 몸체 유지

요청은 "G1 모델을 G1-EDU Plus 모델로 교체, 안 되면 기존 G1 유지 + 이유"였습니다. 결론은 **기존 몸체(G1 29자유도 rev 1.0) 유지**입니다. 이유는 다음과 같습니다.

1. **EDU Plus는 이미 이 몸체입니다.** G1 EDU Plus(대리점 코드 U2)는 29자유도 EDU 몸체로, 허리 3자유도와 팔 7자유도(손목 3자유도)를 갖습니다. Unitree 공식 사양표(unitree.com/g1)의 EDU 열은 허리 Z±155°·X±30°·Y±30°, 손목 P±92.5°·Y±92.5°이고, 지금 시뮬 모델의 관절, 축, 범위, 링크 길이가 이와 같습니다. EDU Plus 기본품의 손은 고무 더미 손입니다. Inspire 손을 단 구성은 대리점 표기로 EDU Ultimate C/D이고, 몸체는 같습니다.
2. **"EDU Plus"라는 이름의 별도 공식 모델 파일은 없습니다.** unitree_ros, unitree_sim_isaaclab(HF USD), unitree_rl_lab, Isaac Lab 어디에도 G1 "edu/plus" 모델이 없습니다. Unitree는 판매 등급이 아니라 하드웨어 리비전(`mode_machine`)으로 모델을 나눕니다.
3. **더 새로운 하드웨어 리비전(mode 11–18, 2025-06 이후)은 지금 바꾸지 않았습니다.** 이 리비전들은 손목 모터가 5010이고 `wrist_yaw` 위치가 46→51 mm로 +5 mm 다릅니다. 링크 질량은 33.3 kg → 36.7 kg(2026-11-09 갱신)이고, 고관절·손목 토크 한계도 다릅니다. 그런데 다음 문제가 있습니다.
   - 사용자의 실물이 어느 리비전인지 모릅니다. 확인 방법은 Unitree 앱 Device → Data → Robot → Machine Type입니다.
   - Unitree가 이 리비전용 시뮬 USD나 Inspire 손 결합 모델, 보행 정책을 공개하지 않았습니다. 시뮬의 보행 정책과 공식 USD는 모두 rev 1.0 기준입니다.
   - 몸체를 바꾸면 보행 안정성과 카메라 리그(손목 +5 mm)를 다시 검증해야 합니다.
4. 실물이 mode 11–18로 확인되면 다음 순서로 바꿀 수 있습니다. 관절 이름과 순서는 같으므로 DDS와 데이터는 그대로입니다.
   1. 해당 `g1_29dof_mode_XX.urdf`를 USD로 변환합니다.
   2. 이 문서의 손 덮어쓰기를 적용합니다(도구가 공식 USD 경로만 입력으로 받습니다).
   3. `effort_limit_sim`을 갱신합니다.
   4. 보행과 들어 올리기를 다시 검증합니다.

## 6. 스크립트 파지(파이프라인 시험)와 손 프리셋

E2 손은 DFX 손보다 큽니다. 손가락 MCP가 2 cm 더 앞이고, 손바닥 폭이 93 mm이며, 엄지가 길고 기저가 높습니다. 그래서 DFX용 파지 위치에서는 엄지가 원통을 쳐서 넘어뜨렸습니다. 손 모델별 값을 따로 두었습니다.

- **손 프리셋**(`configs/hands_inspire.yaml` `hand_models.rh56e2`, 허브가 시뮬 상태의 `hand_model`을 보고 한 번 적용):
  - 펼침: 엄지 회전 q 0.45(0.91 rad)로 미리 돌립니다.
  - 쥠: 손가락 0.15, 엄지 굽힘 0.65, 엄지 회전 0.45입니다.
- **파지 위치**(`configs/scripted_grasp.yaml` `hand_models.rh56e2`): 원통 중심은 IK 끝점(R_ee = `right_wrist_yaw_link` + 0.05 m) 기준 (0.170, 0.060, −0.005) m입니다. 접근은 손바닥 쪽(+y)에서 곧게 들어옵니다.
  - 손-원통 기하 검사: 펼친 손과의 여유는 16.4 mm, 7 cm 접근 경로에서는 15.8 mm입니다. 쥠 목표는 손가락 10.3 mm, 엄지 6.5 mm 안쪽이고, 접촉 방향이 119° 떨어져 있습니다.
  - 렌더는 `logs/captures/rh56e2/e2_close_sequence.png`에 있습니다.
- **폐루프 접근**(`servo_approach`): 팔이 이 자리에 1–2 cm 못 미치고(IK 잔차와 추종 지연) 손가락이 약 7° 숙여집니다. 그래서 접근 후반에 시뮬 정답의 손목 위치로 위치 오차를 적분해 보정합니다. 오차가 5 mm 안에 들거나 2 s가 지나면 쥡니다. 들어 올리는 동안에는 수직 보정을 풀어 줍니다. 시뮬 정답을 쓰는 것은 기존 스크립트 파지(물체 위치)와 같고, 데이터는 `data/smoke`, `source=scripted`로 남습니다.
- **서는 위치**: 원통이 손목에서 9 cm 더 멀어서, DFX용 자리 (0.70, −0.05)에서는 팔이 손을 충분히 당기지 못합니다(IK 실패 589–1132회). 측정 결과는 이렇습니다.

| 로봇 위치 (x, y) | IK 잔차 (접근) | 결과 |
|---|---|---|
| 0.70, −0.05 | 최대 6–10 cm | 실패 (엄지·손가락에 밀려 넘어짐) |
| 0.57, −0.05 | 4.3 cm | 실패 |
| 0.62, −0.05 / −0.10 | 1.5–1.9 cm | 실패 (오므릴 때 밀림) |
| **0.64, −0.12** | 0.9–1.1 cm | 보정 전 3회 중 2회 성공 → **보정 후 3회 모두 성공 (6.9 cm)** |

2026-11-17 측정: 리셋 직후 로봇은 스스로 약 5 cm 앞으로 미끄러집니다. `--robot-pose 0.64 -0.12 0`으로 리셋하면 실제로는 (0.691, −0.102)에 서므로, 위 표의 위치는 명령값이고 성공한 실제 상대 위치는 원통 기준 (0.33, −0.10) m입니다. 자동 보행 계획(`configs/scripted_walk_grasp_desk070.yaml`)은 이 값을 씁니다. 이 파지는 0.70 m 책상용이며, 기본 책상이 0.55 m가 된 뒤에는 `--scene-set desk.top_surface_height=0.70`이 필요합니다(docs/AUTO_WALK_GRASP.md).

E2 손 들어 올리기는 다음 명령으로 합니다.
```bash
bash scripts/record_episode.sh reset --robot-pose 0.64 -0.12 0 && sleep 8 && bash scripts/run_mock_teleop.sh scripted
```

## 7. 시뮬 연결

- `desk_task.py`: `robot.hand_model`이 `rh56e2`면 로봇 USD 경로를 E2 빌드로 바꿉니다. 파일이 없으면 빌드 방법을 알려 주는 오류로 멈춥니다.
- `hand_model.patch_official_inspire`(공식 파일은 고치지 않음):
  - 공식 `inspire_dds`의 `normalize`/`denormalize`가 받는 DFX 고정 범위(0–1.7, 0–0.5, −0.1–1.3)를 E2 범위로 바꿔 끼웁니다.
  - 공식 wholebody 액션 프로바이더의 수동 관절 비율 텐서를 E2 비율로 바꿉니다.
  - 공식 코드 구조가 달라져 비율을 못 바꾸면 시작할 때 오류로 멈춥니다.
- `ext.py`:
  - 상태와 데이터의 `hand_q_norm`은 모델 범위로 계산합니다.
  - 시작할 때 배색을 재질 6개에 적용합니다.
  - `sim_static_meta`에 `hand_model`을 기록합니다.
  - 상태 토픽에 `hand_model`을 싣습니다.
- 빌드 방법:
```bash
# 새 PC: scripts/bootstrap.sh hand  (= 아래 세 줄)
#   python3 scripts/setup/fetch_hand_reference.py   # 원자료 232개를 고정 커밋에서 받아 git blob 해시로 확인 (envs/hand_reference.lock.json)
#   scripts/setup_envs.sh cad                       # numpy 2.5.4, scipy 1.18.1, trimesh 5.2.0, opencv-headless 5.0.0.93 ... (2026-11-12 빌드와 같은 고정)
#   scripts/setup/build_hand.sh [--force]           # 아래 두 명령
.conda/envs/g1_inspire_cad/bin/python tools/cad/make_inspire_e2_hand.py data/reference/inspire_ftp/rh56e2/R_2026_1_5 data/assets/inspire_rh56e2/rh56e2_hands.npz \
    --left-ref data/reference/inspire_ftp/rh56e2/L_2025_9_10 --unitree-ftp data/reference/inspire_ftp/unitree_ros --preview logs/setup/rh56e2_preview.png
# 시뮬 env: 공식 G1 USD를 sublayer로 두고 손 링크를 제자리에서 덮어쓴 USD (13.5 MB)
.conda/envs/g1_inspire_sim/bin/python tools/build_inspire_e2_usd.py
# 2026-11-18 확인: cad env로 다시 만든 npz가 2026-11-12 빌드와 SHA-256 동일(28bf3549…). 이후 meta.source를 작업공간 상대 경로로
# 저장하도록 바꿔, 어느 PC·폴더에서 만들어도 같은 바이트(95ac3c48…). 숫자 배열은 그 전과 동일, USD SHA-256 4d1fb21b…
```

## 8. 검증

| 항목 | 결과 |
|---|---|
| FTP 베이스 메시 대비 장착 | 중앙값 0.05 mm, p95 0.32 mm |
| 왼손 = Inspire 왼손 URDF | 중앙값 0.001 mm |
| USD 빌드 검사 | 관절 순서·링크 순서 공식과 동일, 손 링크 26개 모두 교체, 손 질량 좌우 0.8987 kg |
| 관절 좌표계 일치 (`tools/check_joint_frames.py`) | 손 관절 위치 0.000 mm, 회전 ≤ 0.022° (URDF rpy 소수 반올림) |
| 시뮬 시작 | 관절 53개, 배색 재질 6개 적용, E2 비율 12개가 공식 프로바이더에 들어감 |
| `tests/test_hand_model.py` 등 | 전체 38개 통과 |
| E2 스크립트 들어 올리기 (0.64, −0.12) | 3/3 성공, 6.9 cm, 넘어짐 없음 |
| 공식 DFX 손 회귀 (`--scene-set robot.hand_model=inspire_dfx_official`, 0.70, −0.05) | 성공 7.2 cm, IK 실패 0 (이전과 동일). 허브는 DFX 프리셋 유지, 공식 코드 패치 없음 (`{}`) |

## 9. 한계

- E2 URDF는 Inspire 공식 사이트가 아니라 제3자 저장소에서 받았습니다. 엄지 범위가 E2 매뉴얼과 맞는 것으로 진위를 판단했습니다.
- 배색은 사진에서 정한 면 단위 규칙입니다. 정확한 부품 경계(특히 손가락 옆면, 엄지 끝판)는 실물과 몇 mm 다를 수 있고, 로고는 없습니다.
- 손가락 연동은 선형 비율이고, 실제 E2 링크 기구의 비선형성은 반영하지 않았습니다.
- 촉각 센서 17개는 형상(검은 패드)만 있고 센서 출력은 없습니다. 손-물체 접촉 라벨(`contact_force_total`)이 0만 나오는 문제는 그대로 남아 있습니다.
- 실물 DFTP 손의 제어 토픽은 `rt/inspire_hand/ctrl/{l,r}`(0–1000)입니다. 시뮬은 공식 DFX 인터페이스 `rt/inspire/cmd`(0–1)를 그대로 씁니다. 정규화 각은 같은 뜻이므로, 실물로 옮길 때는 xr_teleoperate `--ee inspire_ftp`를 쓰면 됩니다.
