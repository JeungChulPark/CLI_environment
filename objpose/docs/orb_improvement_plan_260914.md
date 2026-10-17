# ORB-SLAM3 기반 객체 위치 추정 개선 계획 (2026-09-14)

대상 데이터: `260910_object` (실내, 카트 주행 159 s, D455F SLAM 카메라 + D455F SAM 카메라 + VLP-16).
비교 기준 실행: `objpose/output/one_orb_260914_124614` (ORB on .3), `one_orb-mac_260914_125210` (ORB on Mac),
`one_hdl-mac_260914_123836` (LiDAR hdl_graph_slam, 기준선).

이 문서의 모든 수치는 위 실행의 `slam_poses.jsonl` / `kf_updates.jsonl` / `sam6d_estimates.jsonl` / `summary.json`
을 직접 계산한 값이거나 `objpose/pc/compare_runs.py` 의 출력이다. 코드에 대한 모든 주장은 파일:줄로 근거를 단다.
파일 경로는 모두 `/home/jucpark/DeepLearning/CLI_environment` 기준이다.

---

## 1. 현재 상태 요약

### 1.1 세 실행의 핵심 지표

| 지표 | LiDAR hdl_graph_slam | ORB-SLAM3 (.3) | ORB-SLAM3 (Mac) |
|---|---|---|---|
| 포즈 OK / 버림 | 1,577 / 0 | 4,772 / 0 | 4,772 / 0 |
| 추적 시간 평균 / p99 | 13.37 / 20.22 ms | 15.54 / 20.37 ms | 10.76 / 13.65 ms |
| 도착 지연 중앙 | 58.2 ms | 19.1 ms | 17.1 ms |
| 키프레임 수 | 105 (0.66/s) | 183 (1.15/s) | 213 (1.34/s) |
| 루프 클로저 (`*Loop detected`) | 12 (`reason:"loop"`) | **1** | **2** |
| 지도 보정 최대 이동 | **2.67 cm** | **41.39 cm** | **158.63 cm** |
| 실시간 vs 최종 궤적 (중앙/p95/최대) | 0.73 / 2.49 / 3.73 cm | 0.60 / 15.18 / 55.52 cm | 0.62 / 31.1 / 210.61 cm |
| 정지 객체 관측 흩어짐 (중앙/p90) | **1.62 / 2.84 cm** | 4.31 / 9.54 cm | 5.68 / 13.09 cm |
| 보간 예측 오차 (간격 3–10 s) | 3.49 cm | 7.16 cm | 3.46 cm |
| 객체 메모리 (유지/삭제) | 8 / 0 | 8 / 2 | 9 / 1 (mug 가 두 개체로 분리) |

`compare_runs.py` 로 LiDAR 를 기준으로 정렬했을 때 (반드시 외부 파라미터를 넘겨야 한다 — 안 넘기면
LiDAR 센서 좌표와 카메라 좌표 차이 때문에 RMSE 가 6.4 cm 대신 33 cm 로 나온다):

- 최종 궤적 정렬 RMSE **6.36 cm** (실시간 궤적으로는 9.60 cm)
- 같은 객체 위치 차 1.95 – 8.42 cm (중앙 4.74 cm)
- 좌표계와 무관한 객체 쌍간 거리 차: 중앙 **2.13 cm**, 최대 8.56 cm

### 1.2 드리프트는 "언제" 생기는가 — 5 초 구간 실시간 vs 최종 궤적 (중앙값, cm)

```
       0–35 s  35   40   45   50   55   60   65 |  70   75   80   85   90 …
ORB .3   0.0   2.4  0.6  3.1  7.2 11.4 15.2 21.8| 14.3  6.8  4.8  2.0  0.8 …
ORB Mac  0.0   3.4  4.9  5.7 15.3 24.0 30.3 38.7|  9.1  4.6  3.7  4.5  1.1 …
HDL      0.0   0.2  0.9  1.4  1.5  0.6  0.5  0.9|  2.6  1.6  2.3  1.4  1.3 …
```

20 초 창 요약 (같은 계산, 각 창의 회전율·속도 동반):

| 창 | ORB .3 중앙/p90 | ORB Mac 중앙/p90 | HDL 중앙/p90 | 회전율 중앙/p90 | 속도 중앙 |
|---|---|---|---|---|---|
| 0–20 s | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.2 °/s | 0.01 m/s |
| 20–40 s | 0.0 / 2.8 | 0.0 / 3.4 | 0.0 / 0.5 | 0.2 / 7.3 °/s | 0.01 m/s |
| **40–60 s** | **7.0 / 12.0** | **12.7 / 25.1** | 1.1 / 1.6 | **11.0 / 18.9 °/s** | 0.29 m/s |
| **60–80 s** | **14.7 / 22.2** | **29.8 / 38.9** | 1.1 / 3.3 | **17.3 / 33.4 °/s** | 0.31 m/s |
| 80–100 s | 1.3 / 7.1 | 1.3 / 4.9 | 1.8 / 3.0 | 17.7 / 33.0 °/s | 0.34 m/s |
| 100–160 s | ≤ 1.1 / 2.2 | ≤ 0.7 / 2.3 | ≤ 1.4 / 2.5 | 10–11 / 24–29 °/s | 0.16–0.35 m/s |

읽는 법:

- **0–35 s 는 카트 정지**(속도 0.01 m/s, 회전율 0.0–0.2 °/s). 드리프트가 정확히 0 이고, 이 구간에는 SAM-6D
  관측이 **한 건도 없다** (아래 1.3). IMU 초기화에 필요한 여기(excitation)가 전혀 없는 구간이기도 하다 (§4).
- 드리프트는 **첫 회전이 시작되는 45 s 부터 자라 65–70 s 에 최대**(ORB .3 21.8 cm, Mac 38.7 cm)가 되고,
  루프 클로저 순간에 한꺼번에 사라진다. 이 구간의 회전율 중앙 11–17 °/s, p90 19–33 °/s — **드리프트는 이동이 아니라
  회전에 붙어 자란다**. 같은 구간 LiDAR 는 1.1 cm 로 평평하다. 즉 데이터나 궤적이 어려운 게 아니라 **RGB-D 단안
  추적이 회전에서 지는 것**이다.
- 회전율만 보면 100–140 s 도 10 °/s 대로 비슷한데 드리프트가 없다. 이미 루프가 닫혀 지도가 고정된 뒤이기 때문이다.
  **그래서 문제는 "루프가 닫히기 전 40 초"에 국한된다.**

### 1.3 관측은 "언제" 들어오는가 — 이게 왜 치명적인가

`observations_final` 의 시각 분포 (전체 326 / 324 / 336 건):

| 구간 | ORB .3 | ORB Mac | HDL |
|---|---|---|---|
| 0–35 s (정지) | 0 | 0 | 1 |
| **35–70 s (드리프트 최대)** | **129 (40 %)** | **126 (39 %)** | 124 |
| 70–90 s | 0 | 0 | 0 |
| 90–160 s (루프 이후) | 197 (60 %) | 198 (61 %) | 211 |

**전체 관측의 40 %가 드리프트가 7–39 cm 인 구간에서 들어온다.** 객체별 관측 흩어짐을 루프 전/후로 쪼개면
차이가 그대로 드러난다 (ORB .3, 전체 중앙값 기준 거리, cm):

| 객체 | n | 루프 전 n | 흩어짐(루프 전) | 흩어짐(루프 후) |
|---|---|---|---|---|
| Sikhye_high | 63 | 18 | **10.39** | 1.46 |
| milk | 52 | 17 | **9.96** | 1.06 |
| saffron | 49 | 15 | **7.24** | 1.18 |
| choco_hazelnut_high | 58 | 24 | **5.53** | 1.48 |
| Mugcup_high | 29 | 14 | 4.24 | 3.17 |
| Bear | 30 | 17 | 6.24 | 4.77 |

ORB Mac 은 더 심하다 (Sikhye 15.38 / 1.07, milk 15.55 / 0.59). LiDAR 는 루프 전 1.2–2.3 cm, 루프 후 1.1–2.1 cm 로
**차이가 없다.**

결정적으로, LiDAR 실행을 기준으로 각 객체의 최종 위치 오차를 재면 **"그 객체 관측 중 루프 전 비율"과 오차의
상관계수가 +0.69 (.3) / +0.54 (Mac)** 이다. Bear(루프 전 57 %) 7.29 cm, Dinosaur(58 %) 9.76 cm,
Febreze(52 %) 8.53 cm vs milk(33 %) 2.96 cm, Sikhye(29 %) 3.83 cm. **오차의 주된 원인은 SAM-6D 도 앵커링 방식도
아니고, "루프 전 관측"이다.**

### 1.4 루프 클로저 진단 (`mac_slam.log` 의 `LOOP_GAP` 줄)

```
.3  : cur_id=111 matched_id=48  ts 637.41 ← 612.91 (24.5 s)  gap_m=2.703  sim3_scale=1.000000
Mac : cur_id=75  matched_id=3   ts 623.28 ← 584.40 (38.9 s)  gap_m=1.427  sim3_scale=1.000000
Mac : cur_id=119 matched_id=51  ts 638.75 ← 612.97 (25.8 s)  gap_m=2.187  sim3_scale=1.000000
```

- 159 초 동안 루프 검출이 **한 번(.3) / 두 번(Mac)** 뿐이다. LiDAR 는 12 번이다.
- 세 번 모두 **회전이 끝난 뒤 623–639 s 대에 몰려 있다**. 45–70 s 의 드리프트 구간에는 루프가 하나도 없다.
- `sim3_scale=1.0` — RGB-D 라 스케일은 고정이고, 보정은 순수 회전+평행이동이다.
- 허브가 세는 `slam_loop_events` (2 / 4) 는 포즈 메시지의 `map_changed` 플래그 개수라 `*Loop detected` 보다 크다
  (`objpose/pc/fusion.py:162-163`, `slam_stream.cc:560-566` 이 루프 후 +0.5 s / +3 s 추가 `kf_update` 를 보낸다).

### 1.5 코드에서 확인한 사실 (계획의 전제)

**(a) `--slam-param` 은 ORB-SLAM3 레인에서 동작하지 않는다. 실행이 즉시 실패한다.**

- `objpose/pc/hub.py:1032-1034` 는 `--slam-param Key=Value` 를 받고 도움말에 대놓고
  `"ORB-SLAM3 settings override ... e.g. ORBextractor.nFeatures=1500"` 이라고 쓰여 있다.
- `hub.py:320-322` 가 orbslam3 레인의 원격 명령을 만들 때 `--param Key=Value` 로 붙인다.
- 그런데 `objpose/mac_slam/slam_stream.cc:177-200` 의 인자 루프에 **`--param` 분기가 없다.**
  `slam_stream.cc:200` 이 `else { usage(); return 2; }` 이므로 스트리머가 종료 코드 2 로 죽는다.
- `--param` 은 RTAB-Map 쪽에만 구현돼 있다 (`objpose/mac_slam/rtab_stream.cc:327-332`).
- `mac_slam/run_slam.sh:32-40` 과 `linux_slam/run_slam.sh:22-30` 은 인자를 그대로 통과시키기만 한다.

→ **오늘 재컴파일 없이 바꿀 수 있는 ORB 파라미터는 `--features` 하나뿐이다.**
`slam_stream.cc:139-159 settings_override()` 가 설정 yaml 사본에서 `ORBextractor.nFeatures` 한 줄만 정규식으로
바꾸고, atlas 키만 덧붙인다.

**(b) 그 대신 `--settings` 는 살아 있다.** 두 러너 모두 `--settings` 가 주어지지 않았을 때만 기본값을 넣는다
(`mac_slam/run_slam.sh:38`, `linux_slam/run_slam.sh:28`). 하지만 `hub.py:320-322` 는 orbslam3 레인에
`--settings` 를 넘기지 않으므로, 허브를 거쳐 다른 yaml 을 쓰려면 SLAM 호스트의 기본 파일을 직접 고치거나
hub 에 키를 추가해야 한다.

**(c) 변환본 bag 은 카메라 블록을 무시한다.** `slam_stream.cc:106-136 settings_with_camera()` 가 converted bag 의
`camera_info` 로 `Camera1.*` / `Camera.width/height` 를 덮어쓴다. 따라서 새 설정 yaml 을 만들 때 카메라 값은
아무거나 둬도 되지만 **키는 반드시 존재해야 한다** (없으면 `settings_with_camera` 가 예외를 던진다,
`slam_stream.cc:113`). `ORBextractor.nFeatures` 줄도 반드시 있어야 한다 (`slam_stream.cc:149`).

**(d) 이 저장소의 ORB-SLAM3 는 이미 루프 클로저 문턱값이 완화돼 있다.** 초기 커밋부터 그렇다:

| 위치 | 현재 | 원본 |
|---|---|---|
| `orbslam_ws/src/ORB_SLAM3/src/LoopClosing.cc:418` | `mnLoopNumCoincidences >= 1` | `>= 3` |
| `.../LoopClosing.cc:603-607` | 12 / 8 / 12 / 30 / 40 | 20 / 15 / 20 / 50 / 80 |
| `.../LoopClosing.cc:902` | `nNumCoincidences >= 1` | `>= 3` |
| `.../KeyFrameDatabase.cc:149,290,378,512,648,769` | `maxCommonWords*0.5f` | `*0.8f` |

**즉 "루프 문턱값을 낮추자"는 이미 한 번 한 일이고, 그런데도 159 초에 1–2 번밖에 안 걸린다.** 아직 원본
그대로인 곳은 `LoopClosing.cc:563-565` 의 `DetectAndReffineSim3FromLastKF` 경로 (30 / 50 / **100**) 와
병합 경로 `LoopClosing.cc:466` (`>= 3`) 이다. `LoopClosing.cc:505` 는 키프레임당 후보를 **3 개**로 자르고,
`LoopClosing.cc:379-384` 는 지도 키프레임이 12 개 미만이면 아예 건너뛴다. 전부 컴파일 타임 상수다.

**(e) Mac 빌드는 이 클론에서 컴파일하지 않는다.** `objpose/mac_slam/CMakeLists.txt:11` 이
`ORB_SLAM3_SRC = $ENV{HOME}/objpose/ORB_SLAM3_src` — "a COPY of the repo clone, never the clone".
소스를 고치면 `objpose/linux_slam/deploy.sh:32` 식의 rsync 로 각 호스트에 다시 밀어야 한다.

**(f) 추적된 특징점 수가 로그에 안 남는다.** `slam_stream.cc:294,331` 이 `n_tracked_mp` 를 계산하는데
오프라인 CSV 에만 쓰고(`slam_stream.cc:345,364`), 라이브 포즈 JSON(`slam_stream.cc:535-556`)에는 넣지 않는다.
그래서 `slam_poses.jsonl` 로는 "45–70 s 에 특징점이 몇 개였나"를 지금 알 수 없다. **한 줄 패치로 얻을 수 있는
가장 값싼 관측치다.**

**(g) 재컴파일 없이 yaml 로 바꿀 수 있는 것의 전체 목록** (`Settings.cc` 를 읽어 확인):

| 키 | 필수? | 근거 | RGB-D 에서의 효과 |
|---|---|---|---|
| `ORBextractor.nFeatures / scaleFactor / nLevels / iniThFAST / minThFAST` | 필수 | `Settings.cc:443-450` | 추출 단계. `System` 생성 시 1회 반영 (`Tracking.cc:620-634`) |
| `Stereo.ThDepth`, `Stereo.b` | 필수 | `Settings.cc:438-439` | `mThDepth = b × ThDepth` (`Tracking.cc:604-605`). 현재 0.0745×40 = **2.98 m** |
| `RGBD.DepthMapFactor` | 필수 | `Settings.cc:437` | mm→m |
| `Camera.fps` | 필수 | `Settings.cc:410` | `mMaxFrames`(`Tracking.cc:617`) → 키프레임 강제 삽입 주기 |
| `Camera.newWidth / newHeight` | 선택 | `Settings.cc:365,384` | RGB·depth 동시 리사이즈, 내부 파라미터 자동 재스케일 |
| `System.thFarPoints` | 선택 | `Settings.cc:482` | 먼 맵포인트 생성 억제 (`LocalMapping.cc:684`) |
| `System.LoadAtlasFromFile / SaveAtlasToFile` | 선택 | `Settings.cc:475-476` | 지도 저장/적재 |
| `loopClosing` (0/1) | 선택 | **`System.cc:101-106`** (Settings.cc 아님) | 루프 클로저+병합 전체 on/off |
| `Optimizer.PoseIterations`, `Optimizer.PoseEarlyExit` | 선택 | **`Tracking.cc:55-81`** (이 저장소의 로컬 추가) | 포즈 최적화 반복 스케줄 |
| `IMU.*` | IMU_RGBD 면 필수 | `Settings.cc:414-432` | §4 |

**재컴파일이 필요한 것**: `NeedNewKeyFrame()` 의 모든 비율 — `nMinObs=3`(`Tracking.cc:3221-3223`),
근접점 판정 `100 / 70`(`Tracking.cc:3251`), `thRefRatio=0.75/0.4`(`3254-3256`), `c1c` 의 `0.25`(`3284`),
`c2` 의 `>15`(`3286`), `mMinFrames=0`(`616`), 매퍼 큐 `<3`(`3324`) — 및 §1.5(d) 의 모든 루프 문턱값.

---

## 2. 방향 1 — 특징점을 늘리고 ORB 파라미터를 조정해 루프 전 드리프트를 줄인다

### 2.1 원리

45–70 s 의 드리프트는 회전 중 누적된다 (§1.2). RGB-D 추적에서 회전 오차를 잡아 주는 것은
(i) 프레임당 매칭되는 맵포인트의 **수와 화면 내 분포**, (ii) **깊이가 유효한 "근접(close) 스테레오 점"의 수**
— 근접점만 완전한 3D 제약을 주고 먼 점은 방향만 준다 — (iii) 회전 중 새 키프레임이 얼마나 자주 삽입되어
로컬 BA 가 오차를 흡수하는가, 이 셋이다. `nFeatures` 는 (i)만 건드리지만 `Stereo.ThDepth` 는 (ii)와 (iii)을
동시에 건드린다: 근접점 판정이 `NeedNewKeyFrame()` 의 `bNeedToInsertClose`(`Tracking.cc:3238-3251`)와
`c1c`(`Tracking.cc:3284`)에 직접 들어가기 때문이다.

### 2.2 이 데이터에서 기대 효과 (정량 근거)

**연산 여유가 실제로 있다.** 30 Hz 예산은 33 ms 인데 현재 추적 시간은 .3 15.54 ms(p99 20.37),
Mac 10.76 ms(p99 13.65). ORB 추출은 특징 수에 대략 선형이고 추적 시간의 절반 정도를 차지하므로:

| 설정 | .3 예상 평균 / p99 | Mac 예상 평균 / p99 | 판정 |
|---|---|---|---|
| f2000 (현재) | 15.5 / 20.4 | 10.8 / 13.7 | 기준 |
| f3000 | ~19 / ~25 | ~13 / ~17 | 둘 다 안전 |
| f4000 | ~23 / ~30 | ~16 / ~20 | .3 는 위험, Mac 은 안전 |

**얼마나 좋아질까.** 정직하게 말하면 `nFeatures` 단독 효과는 크지 않을 가능성이 높다. 근거:
이 저장소가 이미 한 RTAB-Map 튜닝(README.md 276–291행)에서 특징 수가 아니라 **`Vis/MaxDepth=4`(깊이 지평)**
와 특징 종류 변경이 흩어짐을 3.94 → 2.35 cm 로 줄였다. 같은 교훈이 ORB 쪽 `Stereo.ThDepth` 에 그대로 적용된다.
현재 근접/원거리 경계가 **2.98 m** 인데, 실내 카트 주행에서 D455 의 유효 깊이는 4 m 이상이다. 즉 3–4 m 의
쓸 만한 깊이 관측이 전부 "먼 점"으로 강등돼 회전 제약에서 빠지고 있다.

목표치 (LiDAR 를 상한, 현 ORB 를 하한으로 둔 현실적 목표):

| 지표 | 현재 (.3 / Mac) | 목표 | LiDAR (상한) |
|---|---|---|---|
| 실시간 vs 최종 p95 | 15.2 / 31.1 cm | **≤ 8 cm** | 2.5 cm |
| 실시간 vs 최종 최대 | 55.5 / 210.6 cm | **≤ 25 cm** | 3.7 cm |
| 60–80 s 창 중앙 | 14.7 / 29.8 cm | **≤ 8 cm** | 1.1 cm |
| 객체 흩어짐 중앙 / p90 | 4.31 / 9.54, 5.68 / 13.09 cm | **≤ 3.0 / 7.0 cm** | 1.62 / 2.84 cm |
| 지도 보정 최대 | 41.4 / 158.6 cm | **≤ 20 cm** | 2.7 cm |
| 추적 p99 | 20.4 / 13.7 ms | **≤ 25 ms** | — |

### 2.3 구체 작업 단계

**1단계 (코드 변경 0, 지금 당장) — `--features` 스윕.**
`--features` 만은 이미 끝까지 연결돼 있다 (`hub.py:1030` → `hub.py:320` → `run_slam.sh` →
`slam_stream.cc:145-152`). 명령은 부록 A.1.

**2단계 (slam_stream.cc 약 15줄) — `--param Key=Value` 를 구현해 문서와 코드를 일치시킨다.**
`rtab_stream.cc:327-332` 를 그대로 본떠서:

1. `Args` 구조체(`slam_stream.cc:82-95`)에 `std::vector<std::pair<std::string,std::string>> params;` 추가.
2. 인자 루프 `slam_stream.cc:199` 와 `:200` 사이에 분기 추가 —
   `else if (k == "--param") { auto kv = val(); auto eq = kv.find('='); if (eq == npos) { usage(); return 2; } a.params.push_back({kv.substr(0,eq), kv.substr(eq+1)}); }`
3. `settings_override()`(`slam_stream.cc:139-159`)를 `params` 도 받도록 확장. 이미 `ORBextractor.nFeatures` 를
   정규식으로 갈아끼우고 있으므로(`:147-151`) 같은 로직을 일반화한다: 각 키에 대해
   `(^|\n)<key 이스케이프>:[^\n]*` 를 찾아 있으면 치환, 없으면 파일 끝에 `\n<key>: <value>\n` 추가.
   `settings_with_camera()` 의 `set()` 람다(`slam_stream.cc:113-117`)가 이미 정확히 이 정규식을 쓴다.
4. `main()` 의 호출부(`slam_stream.cc:268`)에 `a.params` 를 넘긴다.
5. 사용법 문자열(`slam_stream.cc:97-102`)에 `[--param Key=Value ...]` 추가.
6. 각 SLAM 호스트에 배포: `.3` 는 `objpose/linux_slam/deploy.sh`, Mac 은
   `$HOME/objpose/ORB_SLAM3_src` 는 건드릴 필요 없고 `slam_stream.cc` 만 다시 밀고 재빌드
   (`objpose/mac_slam/CMakeLists.txt:11` 참조).

이게 되면 `--slam-param Stereo.ThDepth=55` 같은 것이 도움말대로 동작한다.

**3단계 — 실제 스윕할 파라미터 (효과 큰 순).**

| # | 파라미터 | 현재 → 시도 | 왜 |
|---|---|---|---|
| A | `Stereo.ThDepth` | 40 → **55, 70** (2.98 → 4.10, 5.22 m) | 근접점 급증 → 회전 제약 + 회전 중 키프레임 증가. 가장 유망 |
| B | `ORBextractor.nFeatures` | 2000 → 3000 (Mac 은 4000까지) | 매칭 수·분포 |
| C | `ORBextractor.nLevels` / `scaleFactor` | 8 / 1.2 → **10 / 1.15** | 스케일 피라미드를 촘촘히 → 카트 전진 시 스케일 변화 대응 |
| D | `ORBextractor.iniThFAST` / `minThFAST` | 20 / 7 → **12 / 5** | 실내 저대비 벽면에서 특징 확보 (`260826` 는 "dark" 세션이라 원래부터 문제였다) |
| E | `Camera.fps` | 30 → **15** | `mMaxFrames`(`Tracking.cc:617`) 를 반으로 → `c1a`(`Tracking.cc:3280`) 가 1 s 대신 0.5 s 마다 발화 → 키프레임 약 2배. **주의: fps 는 다른 곳에도 쓰이므로 단독으로 한 번 검증할 것** |
| F | `System.thFarPoints` | 없음 → **6.0** | 먼 맵포인트가 회전 추정을 오염시키는 것 차단 (`LocalMapping.cc:684`) |
| G | `Optimizer.PoseIterations` | 이 저장소 로컬 추가 (`Tracking.cc:55-81`) | 성능 커밋으로 줄여 놓은 값이면 정확도와 맞바꾼 것이므로 원복 실험 가치 있음 |

**4단계 (선택, 재컴파일) — 회전 중 키프레임을 늘린다.** `Tracking.cc:3251` 의 `nTrackedClose < 100 &&
nNonTrackedClose > 70` 을 `< 130 && > 50` 으로, `Tracking.cc:3254` 의 `thRefRatio = 0.75f` 를 `0.85f` 로.
E 와 목적이 같으므로 **E 를 먼저 해 보고 부족할 때만** 한다.

**하지 말 것: 루프 문턱값을 더 낮추는 것.** §1.5(d) 대로 이미 원본의 절반 수준으로 완화돼 있는데도 159 초에
1–2 번뿐이다. 더 낮추면 거짓 루프가 생기고, 거짓 루프의 대가는 Mac 실행의 **158.63 cm 보정**과
**mug 가 두 개체로 쪼개진 사고**로 이미 관측됐다.

### 2.4 검증 방법

각 설정마다 전체 159 초 실행 후:

```bash
/home/jucpark/anaconda3/envs/sam6d/bin/python objpose/pc/compare_runs.py <새 실행> <기준 실행> \
  --extrinsic-a objpose/rt/260910/X_slam_sam_260910.json \
  --extrinsic-b objpose/rt/260910/X_slam_sam_260910.json
```

합격 판정에 쓸 필드 (`compare_runs.py:108-123`):

1. `observation_spread_cm.median` ≤ **3.0**, `.p90` ≤ **7.0** (현재 4.31 / 9.54)
2. `live_vs_final_traj_cm.p95` ≤ **8**, `.max` ≤ **25** (현재 15.18 / 55.52)
3. `track_ms_p99` ≤ **25 ms**, `slam_dropped == 0`
4. `kf_max_correction_cm` ≤ **20** (현재 41.39)
5. `memory_kept == 8 && memory_classes == 8` (개체 분리 사고 없음)

그리고 LiDAR 기준 정렬 (`--extrinsic-a objpose/rt/260910/X_lidar_sam.json`):
`trajectory_final_B_aligned_to_A_cm.rmse` ≤ **4 cm** (현재 6.36),
`pairwise_object_distance_diff_cm.median` ≤ **1.5 cm** (현재 2.13).

**구간별 검증이 더 민감하다.** 총 중앙값은 정지 구간 때문에 희석되므로, 부록 B 의 5 초 구간 표를 다시 돌려
**60–80 s 창의 중앙값**을 주 지표로 삼는다 (현재 .3 14.7 / Mac 29.8 cm).

### 2.5 소요 / 위험

- 1단계 `--features` 스윕: **각 4분 실행 × 3 설정 ≈ 30분**, 코드 변경 0, 위험 없음.
- 2단계 `--param` 구현: **1–2시간** (코드 15줄 + 두 호스트 재빌드·배포). 위험: Mac 빌드는
  `$HOME/objpose/ORB_SLAM3_src` 사본을 쓰므로 동기화를 빠뜨리면 조용히 옛 바이너리가 돈다.
- 3단계 스윕 7종 × 4분 + 조합 실험: **반나절**.
- 4단계 재컴파일 튜닝: **1일**, 위험 중간 (상류와 갈라짐이 커짐).
- 공통 위험: 특징을 늘리면 p99 가 예산에 근접해 프레임 드롭(`slam_dropped`)이 생길 수 있다. `slam_stream.cc` 는
  늦은 프레임을 건너뛰므로(`slam_stream.cc:521-527`) 조용히 실효 프레임률이 떨어진다 — `slam_dropped` 를 반드시 확인한다.

---

## 3. 방향 2 — 루프 클로저 이후 관측 재추정

### 3.1 지금 코드가 하는 일

- 관측 시각의 기준 키프레임에 `T_kf_obj` 로 저장 (`objpose/pc/fusion.py:289-319`, 특히 앵커 선택
  `fusion.py:298 → anchor_at()` `fusion.py:229-236`, 상대 변환 `fusion.py:303`).
- `kf_update` 가 오면 키프레임 포즈만 갈아끼우고 (`fusion.py:190-192`, `fusion.py:84-107`),
  월드 좌표는 읽을 때 `T_w_kf(최신) · T_kf_obj` 로 재계산 (`fusion.py:283-287`).
- 객체 메모리는 키프레임이 움직일 때마다 **전 프레임을 처음부터 재생해 통째로 다시 만든다**
  (`objpose/pc/memory.py:108-132`). 실측 재구축 162회, 마지막 59.7 ms.
- 즉 **"루프 후 재추정"은 이미 하고 있다.** 문제는 그 재추정의 품질이다.

### 3.2 먼저 확인한 것: 앵커 선택을 바꾸는 것은 도움이 안 된다 (실측)

326개 관측 전체를 로그에서 오프라인 재구성해 네 가지 앵커 규칙의 객체별 흩어짐 중앙값을 비교했다
(각 관측 직전의 `kf_update` 스냅샷을 "관측 당시 지도", 마지막 `kf_update` 를 "최종 지도"로 삼음):

| 규칙 | ORB .3 중앙 / p90 | ORB Mac 중앙 / p90 | HDL 중앙 / p90 |
|---|---|---|---|
| **현재 (추적 기준 키프레임)** | **3.09 / 10.3** | **4.09 / 15.3** | 1.64 / 2.9 |
| 공간상 가장 가까운 키프레임으로 재앵커 | 4.29 / 10.1 | 5.20 / 14.5 | 1.81 / 3.1 |
| 시간상 가장 최근 키프레임으로 재앵커 | 4.56 / 10.0 | 5.64 / 14.3 | 1.63 / 2.9 |
| 가장 가까운 3개 키프레임 역거리 가중 | 4.36 / 10.2 | 5.03 / 14.3 | 1.71 / 3.1 |

**모든 대안이 현재보다 나쁘다.** 이유는 지렛대 길이다. 현재 방식의 `|T_kf_c|` (관측 프레임과 기준 키프레임
사이 거리)는 중앙 5.4 cm / p90 15.3 cm (45–70 s 구간에서도 8.2 / 19.3 cm), 회전 중앙 1.3° / p90 8.2° 로
**이미 매우 짧다**. 다른 키프레임으로 옮기면 지렛대만 길어진다.

**더 결정적인 반증**: LiDAR 실행은 `|T_kf_c|` 중앙이 **28.5 cm**(ORB 의 5배), 회전 12.3° 인데도 흩어짐은
1.6 cm 다. **즉 문제는 앵커링 기하가 아니라 ORB 지도 자체의 내부 일관성이다.** 앵커 규칙을 고치는 작업은
전부 취소한다.

### 3.3 그래서 무엇을 해야 하는가 — 관측 가중치 (실측으로 검증됨)

§1.3 의 상관계수(+0.69 / +0.54)가 가리키는 대로, **루프 전 관측에 가중치를 낮춰서** 융합하면 된다.
LiDAR 실행의 객체 위치를 기준(`compare_runs.py` 의 Umeyama 정렬로 ORB 지도에 옮김)으로 실측한 결과:

**ORB .3 — 객체별 LiDAR 대비 오차 (cm)**

| 객체 | n | 루프전 % | 현재(전체 중앙) | 루프전 **제외** | 루프전 가중 0.25 | 가중 0.05 |
|---|---|---|---|---|---|---|
| Bear | 30 | 57 | 7.29 | 8.09 | **6.52** | 6.52 |
| Dinosaur | 19 | 58 | 9.76 | 8.45 | 8.24 | **7.93** |
| Febreze_high | 23 | 52 | 8.53 | 8.90 | **7.80** | 8.13 |
| Mugcup_high | 29 | 48 | 3.55 | 5.30 | 3.07 | **2.95** |
| Sikhye_high | 63 | 29 | 3.83 | 5.18 | **3.07** | 3.38 |
| choco_hazelnut_high | 58 | 41 | 4.46 | 5.99 | **4.37** | 4.80 |
| milk | 52 | 33 | 2.96 | 5.42 | **2.66** | 2.98 |
| saffron | 49 | 31 | 6.36 | 5.97 | 5.12 | **4.94** |
| **중앙 / 평균** | | | **5.41 / 5.84** | 5.98 / 6.66 | **4.75 / 5.11** | 4.87 / 5.20 |

**ORB Mac — 같은 계산**: 현재 6.25 / 5.93 → 가중 0.25 에서 **4.88 / 4.69** (최대 8.93 → 7.56).

세 가지 결론:

1. **가중치 0.25 가 최선.** .3 중앙 5.41 → 4.75 cm (**−12 %**), Mac 6.25 → 4.88 cm (**−22 %**).
   Mac 처럼 드리프트가 클수록 이득이 크다.
2. **루프 전 관측을 버리는 것은 오히려 나쁘다** (5.98 / 6.77). 시점 다양성을 잃기 때문이다.
   → **버리지 말고 가중치만 낮춘다.** 이게 이 절의 핵심 발견이다.
3. 0.25 와 0.05 는 사실상 같다 → 정확한 값에 민감하지 않다. 견고한 설계다.

**가중치를 무엇으로 정할 것인가 (실시간에 쓸 수 있는 대리 지표).** "루프 전/후"는 사후적 개념이지만,
같은 정보를 실시간에 얻을 수 있다: **그 관측의 앵커 키프레임이 이후 받은 보정량**이다.
`fusion.py:92` 가 이미 `max_correction_m` 을 계산하고 있으니, 키프레임별로 누적 보정량을 들고 있으면 된다.

```
w(관측) = 1 / (1 + (d_kf / d0)^2),   d_kf = 그 관측의 앵커 키프레임이 지금까지 받은 누적 보정 거리
```

`d0 = 5 cm` 로 두면 보정 0 인 키프레임은 w=1, 41 cm 보정을 받은 키프레임은 w≈0.015 가 되어 위 실험의
0.25/0.05 영역을 자연스럽게 재현한다. LiDAR 는 최대 보정이 2.67 cm 라 전부 w≈0.78–1.0 으로 남는다 —
**LiDAR 실행의 결과를 건드리지 않는다**는 점이 중요하다.

### 3.4 구체 작업 단계

**1단계 — 키프레임별 누적 보정량을 추적한다.**
`objpose/pc/fusion.py` `KeyframeMap`:
- `__init__`(`fusion.py:70-78`)에 `self.correction: dict[int, float] = {}` 추가.
- `apply_update()`(`fusion.py:84-107`) 의 기존 최대 보정 루프(`fusion.py:90-92`)에서
  `self.correction[k] = self.correction.get(k, 0.0) + 이동거리` 로 누적.
- `resolve()`(`fusion.py:109-120`) 옆에 `def weight(self, kf, d0=0.05)` 를 추가해
  cull 리다이렉트를 따라간 뒤 `1/(1+(d/d0)**2)` 를 돌려준다.

**2단계 — 관측에 가중치를 실어 보낸다.**
- `Observation`(`fusion.py:247-253`)에는 이미 `score` 가 있다. `add_estimate()`(`fusion.py:289-319`)가
  만드는 `ob` 는 그대로 두고, 소비하는 쪽에서 `poses.kfs.weight(ob.kf)` 를 곱한다.
- 객체 메모리 경로가 실제 융합을 하므로 여기가 본체다: `objpose/pc/memory.py:95-106 _step()` 이
  `Sam6DDetection(..., score=score)` 를 만든다(`memory.py:97-99`). 여기서
  `score=score * self.fusion.poses.kfs.weight(f.kf)` 로 바꾸면, `StreamingObjectMemory` 의
  기존 점수 기반 융합·존재 필터가 그대로 가중치를 받는다.
  단, `objectmemory_ws/.../object_memory_runner.py:374` 의 `score_floor` 아래로 떨어지면 관측이 **버려진다** —
  §3.3 결론 2 에 따르면 버리면 안 되므로 `score_floor` 를 0 으로 낮추거나
  `score`(SAM-6D 신뢰도)와 `weight`(SLAM 신뢰도)를 분리해 넘겨야 한다. 후자가 옳다:
  `quality_weighting=True` + `QualityWeights` 경로(`object_memory_runner.py:307-308,424-427,461-464`)가 이미
  존재하므로 여기에 SLAM 가중치를 하나 더 얹는 것이 자연스럽다.
- 메모리는 `kf_update` 마다 통째로 재생되므로(`memory.py:108-132`), **가중치는 재구축 때마다 자동으로 최신
  보정값을 반영한다.** 추가 트리거가 필요 없다 — 이 설계가 이미 맞게 돼 있다.

**3단계 — 오프라인 "최종 지도" 재추정 도구를 만든다 (오늘 데이터로 바로 가능).**
필요한 정보가 이미 로그에 전부 있다:
- `slam_poses.jsonl`: `T_wc`(실시간), `ref_kf`, `T_w_kf`(그 시점 키프레임 포즈) — `slam_stream.cc:549-556` 이 기록.
- `kf_updates.jsonl`: 매 초 **전체** 키프레임 포즈 스냅샷 (`slam_stream.cc:454-483`; `n` 이 1→183 으로 단조 증가함을
  확인했다 — 증분이 아니라 전량이다. `fusion.py:93` 의 컬링 판정이 이에 의존한다).
- `sam6d_estimates.jsonl`: `T_w_obj`(관측 당시 월드), `anchor_kf`, `t_ns`, `R`, `t_mm`.

→ 관측 i 의 최종 위치 = `T_w_kf_final[k] · T_w_kf_snapshot(t_i)[k]⁻¹ · T_w_obj_live(i)`.
이게 부록 B 의 스크립트가 하는 계산이고, `compare_runs.py:36-39` 의 궤적 보정과 정확히 같은 식이다.
`objpose/pc/reestimate_offline.py` (신규) 로 정착시켜 `--weight none|kf-correction|drop-preloop` 를 받아
객체별 위치와 흩어짐을 뽑게 한다. **실시간 코드를 고치기 전에 이 도구로 가중 함수를 먼저 고르는 것이
올바른 순서다** — 실행 없이 기존 로그만으로 몇 초 만에 스윕할 수 있다.

**4단계 (선택) — 그래프가 안정된 뒤 한 번 더 재추정.**
지금은 `kf_update` 가 루프 후 +0.5 s / +3 s 에 온다(`slam_stream.cc:563-565`). 전역 BA 는 그보다 오래 걸릴 수 있으므로
+10 s 팔로우업을 하나 더 추가하고(같은 위치, 한 줄), 마지막 `end` 업데이트(`slam_stream.cc:571`) 시점의
메모리 상태를 `summary.json` 의 최종 답으로 삼는다. 이미 그렇게 동작하고 있으므로 비용은 사실상 0 이다.

**5단계 (연구용) — 하지 말아야 할 것.** §3.2 대로 앵커 재선택(공간/시간 최근접, 다중 키프레임 블렌딩)은
전부 측정상 손해다. 구현하지 않는다.

### 3.5 검증 방법

1. **오프라인 (즉시)**: `reestimate_offline.py` 로 세 기존 실행에 가중 함수를 적용하고,
   LiDAR 실행 객체 위치를 기준으로 중앙/평균 오차를 낸다. 목표는 §3.3 표의 재현 —
   .3 중앙 5.41 → **≤ 4.8 cm**, Mac 6.25 → **≤ 5.0 cm**, 그리고 LiDAR 실행 자체는 **변화 ≤ 0.2 cm**
   (가중치가 좋은 SLAM 을 망치지 않는지 확인하는 회귀 테스트).
2. **실시간**: 가중치를 켠 채 재실행하고 `compare_runs.py` 의
   `objects_B_in_A_map` 최대값이 8.42 → **≤ 6 cm**,
   `pairwise_object_distance_diff_cm.median` 2.13 → **≤ 1.5 cm** 인지 본다.
3. **회귀**: `memory_kept == 8 && memory_classes == 8`, `memory.last_rebuild_ms` ≤ 100 ms.

### 3.6 소요 / 위험

- 3단계 오프라인 도구: **2–3시간**, 위험 없음(읽기 전용).
- 1–2단계 실시간 가중치: **반나절**. 위험: `score` 와 `weight` 를 한 필드에 섞으면 `score_floor`
  (`object_memory_runner.py:374`)가 관측을 삭제해 버린다 — §3.3 결론 2 와 정면으로 어긋난다. 반드시 분리한다.
- 기대 효과 **−12 % (.3) / −22 % (Mac)** 는 크지 않다. 이게 정직한 상한이다. 왜냐하면
  **애초에 루프 전 드리프트를 줄이는 것(방향 1·3)이 근본 해결이고, 가중치는 사후 완화일 뿐이기 때문**이다.
  다만 비용이 매우 싸고 SLAM 백엔드와 무관하게 동작한다.

---

## 4. 방향 3 — IMU 융합 (ORB-SLAM3 RGB-D-Inertial)

### 4.1 가장 중요한 사실: **지금 데이터에는 IMU 가 없고, 소급해서 만들 수도 없다**

조사 결과 (경로 주의: `converting_launch/` 는 이 저장소가 아니라
`/home/jucpark/DeepLearning/old/CLI_environment/converting_launch/` 에 있다):

- `convert_all.py:79` 의 `STAGES = ("clock","convert","lidar","imu")` 중 `imu` 단계는 **토픽이 아니라 디렉터리**를
  찾는다. `xsens_dir()`(`convert_all.py:210-215`)가 원본 SLAM 폴더에서 `xsens*` 디렉터리를 찾고, 없으면
  `info["imu"] = None`(`convert_all.py:561-566`). RealSense IMU 토픽은 **아예 쳐다보지도 않는다**.
- `convert_recording.py:281` 이 출력 bag 의 토픽을 `[color, depth, color_info, depth_info]` 4개로 하드코딩한다.
  변환본에 IMU 가 들어갈 자리가 구조적으로 없다.
- 변환본 `info.json` 실측: `260826_*` 4개 세션, `260910_object`, `260910_no_object` — 전부 `"imu": null`.
  IMU 가 있는 것은 `260804_office` 2개와 `260807_chungbuk` 3개뿐이고, 그나마 **Xsens 외장 IMU**
  (`/xsens/imu/data`, 200 Hz)지 카메라 IMU 가 아니다.
- `260910_object` 의 `SLAM/`, `SAM/` bag 토픽은 정확히 4개 (color/depth image + camera_info), `imu/` 폴더 없음.
- **원본 bag 을 뒤진 결과가 결정적이다.** `260826` 원본 SDK bag 에는
  `/device_0/sensor_2/info = "Name=Motion Module"` 과 그 옵션 토픽들(`Gyro_Sensitivity` 등)이 **열거만** 돼 있고,
  실제 데이터 스트림인 `/device_0/sensor_2/Gyro_0/...` · `/Accel_0/...` 는 **없다**. 즉 모션 스트림을 켜지 않고
  녹화했다. `session_manifest.json` 의 `"xsens_messages": 0` 도 같은 이야기다.
- `260910` 원본 bag 은 아예 접근 불가 (변환본만 남음).
- **하드웨어는 IMU 를 갖고 있다**: `/device_0/info` = `Intel RealSense D455F, Imu Type=BMI085`.
  (일부 사이드카 JSON 이 `d435i` 라고 쓰지만 bag 의 장치 문자열이 우선이다.)
- **치명적 추가 문제**: `/mnt/synology/Dataset/data_slam/COLLECTION_PROTOCOL.md:130` —
  "이 데이터셋에서 IMU-카메라 extrinsic 이 없어 IMU 결합 SLAM 을 쓰지 못했다." `T_b_c1` 이 없다.

**결론: 기존 데이터로 IMU 실험은 불가능하다. 새로 찍어야 한다.**

### 4.2 이 데이터에서 IMU 가 실제로 도움이 될까 — 냉정한 평가

도움이 **될** 이유: 드리프트는 순수하게 회전에 붙어 자란다(§1.2, 45–70 s 회전율 11–17 °/s).
자이로는 정확히 그 구간의 단기 회전을 잡아 준다. RGB-D-Inertial 이 실제로 좋아지는 전형적 상황이다.

도움이 **안 될** 이유들, 그리고 이게 더 무겁다:

1. **처음 35 초가 완전 정지다.** ORB-SLAM3 의 IMU 초기화는 여기서 붕괴한다:
   - `LocalMapping.cc:1209-1211`: RGB-D 는 최소 **1.0 s** 시간폭 + 키프레임 10개(`:1192-1193`, `:1205-1206`).
   - `LocalMapping.cc:131-136`: `mTinit` 은 **움직일 때만**(연속 프레임 이동 > 0.05 m) 누적된다.
     정지 35 초 동안 `mTinit` 은 0 에 머문다.
   - **`LocalMapping.cc:137-146` 이 진짜 문제다**: VIBA2 이전에 `mTinit < 10 s && dist < 0.02 m` 이면
     `"Not enough motion for initializing. Reseting..."` 를 찍고 `mbResetRequestedActiveMap = true`,
     `mbBadImu = true` 로 **활성 지도를 통째로 날린다**. 정지 35 초 구간에서 이게 반복 발화한다.
   - `Tracking.cc:2471-2475`: 초기화 자체가 `(avgA_cur − avgA_last).norm() < 0.5` 이면 거부
     ("not enough acceleration").
   → **녹화 시작 직후 5–10 초 동안 8자 흔들기 같은 여기 동작을 넣지 않으면 IMU_RGBD 는 시작조차 못 한다.**
   이건 파라미터가 아니라 전부 컴파일 타임 상수다.
2. **VIBA 스케줄이 5 s / 15 s 에 걸려 있다** (`LocalMapping.cc:205-227`). 즉 IMU 가 제 성능을 내는 시점은
   움직임 개시 후 15 초 뒤인데, 이 데이터에서 드리프트 최대 구간은 45–70 s 로 겹치기는 한다. 다만
   `mTinit` 이 정지 구간 때문에 늦게 시작하므로 실제로는 60 s 이후에나 VIBA2 가 끝난다.
3. **`T_b_c1` 이 없으면 아무것도 못 한다.** `Settings.cc:414-432` 에서 `IMU.T_b_c1` 을 포함한 6개 키가
   IMU_RGBD 일 때 **필수**이고, 없으면 `exit(-1)` 이다.

### 4.3 구체 작업 단계

**A. 녹화 파이프라인 변경 (선행 조건, 이게 없으면 나머지 전부 무의미)**

1. 녹화기를 librealsense SDK GUI(`sync_record_gui_v3d`)에서 **realsense-ros** 로 바꾸거나, SDK 파이프라인에
   `RS2_STREAM_GYRO` / `RS2_STREAM_ACCEL` 를 추가한다. realsense-ros 라면
   `sensor_sync_ws/config/realsense_A.yaml:13-15` 가 이미 정답을 갖고 있다:
   `enable_gyro: true`, `enable_accel: true`, `unite_imu_method: 2` → 단일 `/camera/camera/imu` (~200 Hz).
   (참고: `_bmad-output/research-artifacts/orbslam3-d455f-imu-feasibility-research.md:133-134,166` 은
   `unite_imu_method` 의 보간이 VI-SLAM 에 부적절할 수 있으니 gyro/accel 를 따로 받아 직접 보간하라고 권한다.
   ORB-SLAM3 는 `IMU::Point(a, w, t)` 를 받으므로 어느 쪽이든 되지만, 따로 받는 쪽이 안전하다.)
2. **`T_imu_cam` 캘리브레이션을 반드시 같이 찍는다** (`COLLECTION_PROTOCOL.md:124-128` 의 `calib_imu_cam_<date>`
   절차). Kalibr 또는 이에 준하는 것. 이것 없이 찍은 IMU 데이터는 쓸모가 없다.
3. **녹화 프로토콜 변경**: 시작 직후 **5–10 초 동안 3축 여기 동작**(카트를 좌우로 흔들기 + 상하 진동), 그 다음에
   기존처럼 정지 → 주행. 이건 §4.2 의 하드코딩된 게이트들 때문에 타협 불가다.
4. `convert_recording.py:281` 의 토픽 목록에 IMU 를 추가하고, `convert_all.py` 의 `imu` 단계
   (`convert_all.py:561-571`)가 Xsens 디렉터리뿐 아니라 **카메라 IMU 토픽**도 받아들이게 확장.

**B. 스트리머 (`objpose/mac_slam/`) 변경 — 작업량 자체는 작다**

`raw_session.h` 에 필요한 기계장치가 전부 이미 있다:
- 토픽 상수는 `raw_session.h:227-229` (`CONV_COLOR/CONV_DEPTH/CONV_CINFO`) 옆에 `CONV_IMU` 를 추가.
- 토픽 존재 확인 `has_topic()` (`raw_session.h:259`) 으로 IMU 없는 bag 은 우아하게 무시.
- 메시지 순회는 `header_stamps()` (`raw_session.h:272`) 가 `SELECT m.id, substr(m.data,1,16) ...` 를 하고 있으니
  `substr` 대신 전체 `m.data` 를 뽑는 형태로 복제.
- CDR 역직렬화는 `CdrReader` (`raw_session.h:232`) 로 충분하다. `sensor_msgs/Imu` 는
  header(sec, nsec, frame_id) → `double[4]` orientation → `double[9]` cov → `double[3]` angular_velocity →
  `double[9]` cov → `double[3]` linear_acceleration → `double[9]` cov 순.
  `load_converted()` 의 camera_info 파싱(`raw_session.h:431-445`)이 그대로 본보기다.
- `Session`(`raw_session.h:202`)에 `std::vector<ImuSample> imu;` 추가.
→ **약 40–60줄.**

`slam_stream.cc` 쪽:
- `System` 생성(`slam_stream.cc:276`)의 `ORB_SLAM3::System::RGBD` 를 `IMU_RGBD` 로 (설정 yaml 에 IMU 블록이
  있을 때만 자동 선택하도록 분기).
- **`TrackRGBD` API 는 바꿀 필요가 없다.** `orbslam_ws/src/ORB_SLAM3/include/System.h:116` 이 이미
  `TrackRGBD(im, depthmap, timestamp, const vector<IMU::Point>& vImuMeas = {}, string filename="")` 이고,
  `System.cc:386-388` 이 `mSensor == IMU_RGBD` 일 때만 `GrabImuData()` 로 흘린다.
  즉 **직전 프레임 이후의 IMU 샘플 벡터를 채워 넘기기만 하면 된다.**
- `eSensor` 열거는 `include/System.h:87-94`, `IMU_RGBD = 5`.

**C. 설정 yaml**

`objpose/mac_slam/settings/` 에 IMU 블록을 넣은 새 파일을 만든다 (전부 필수, `Settings.cc:414-422`):
`IMU.NoiseGyro`, `IMU.NoiseAcc`, `IMU.GyroWalk`, `IMU.AccWalk`, `IMU.Frequency`, `IMU.T_b_c1` (4×4 cv::Mat).
BMI085 의 데이터시트 값으로 시작해 Allan variance 로 조정. `IMU.InsertKFsWhenLost` 는 선택(기본 true,
`Settings.cc:425-430`).

### 4.4 검증 방법

- 새 데이터 녹화 후 RGB-D 와 RGB-D-Inertial 을 같은 bag 으로 나란히 돌리고 `compare_runs.py` 로 비교.
- 1차 관문: `mac_slam.log` 에 `"Not enough motion for initializing"` 과 `"not enough acceleration"` 이
  **한 번도 안 나오고**, `slam_map_changes == 0` (지도 리셋 없음). 여기서 걸리면 여기 동작을 더 넣는다.
- 2차: 회전 구간(이 데이터 기준 40–80 s 에 해당하는 구간)의 `live_vs_final` 중앙값이 RGB-D 대비 **1/3 이하**.
- 3차: `observation_spread_cm.median` ≤ 2.5 cm (LiDAR 1.62 에 근접).

### 4.5 소요 / 위험 — **정직하게: 가장 비싸고 가장 불확실하다**

| 항목 | 소요 | 위험 |
|---|---|---|
| 녹화기 교체 + IMU 스트림 활성화 | 1–2일 | 중 (SDK GUI 를 버리면 기존 파이프라인 재검증 필요) |
| `T_imu_cam` 캘리브레이션 | 1일 | **높음** — 품질이 나쁘면 IMU 가 오히려 해가 된다 |
| 변환 파이프라인 확장 | 0.5일 | 낮음 |
| `raw_session.h` IMU 리더 | 0.5일 | 낮음 |
| `slam_stream.cc` + yaml | 0.5일 | 낮음 |
| 튜닝(노이즈 파라미터, 여기 프로토콜) | 2–5일 | **높음** — §4.2 의 하드코딩 게이트들 |
| **합계** | **1–2주** | |

추가 위험:
- **기존 데이터셋 전체(260826, 260910)에 소급 적용 불가.** IMU 개선은 새 데이터에서만 검증된다.
  즉 이 문서의 다른 두 방향과 달리 **오늘의 벤치마크로 비교할 수 없다.**
- 정지 35 초 시작 패턴을 프로토콜에서 바꿔야 하는데, 이는 SAM-6D 쪽 기존 실험 조건과 달라진다.
- 이미 LiDAR 가 1.62 cm 흩어짐을 내고 있다. **IMU 로 얻으려는 것을 LiDAR 가 이미 주고 있다.**
  이 사실이 우선순위를 결정한다 (§5).

---

## 5. 우선순위 제안

```
지금 (코드 0줄)  ──►  1주차          ──►  2주차            ──►  그 다음
①features 스윕      ②--param 구현     ④관측 가중치        ⑥(조건부) IMU
③오프라인 재추정 도구  + ThDepth 스윕     ⑤n_tracked_mp 로깅
```

**① `--features 3000/4000` 스윕 — 지금 당장 (§2.3 1단계).**
왜 먼저인가: 코드 변경이 0 이고, 연산 여유가 실측으로 확인됐고 (Mac 10.76 ms / 33 ms 예산),
4분이면 끝난다. 실패해도 잃는 게 없고, 성공하면 방향 1 의 나머지를 정당화한다.

**② 오프라인 재추정 도구 — 지금 당장, ①과 병렬 (§3.4 3단계).**
왜: **SLAM 을 한 번도 안 돌리고** 기존 로그 3개만으로 가중 함수를 스윕할 수 있다. §3.3 의 표가 그 증거다.
읽기 전용이라 위험이 0 이고, ④의 설계를 확정해 준다.

**③ `--param` 구현 + `Stereo.ThDepth` 스윕 — 1주차 (§2.3 2·3단계).**
왜 세 번째인가: `--slam-param` 이 도움말에 있는데 실행이 죽는다는 것 자체가 버그다(§1.5a). 고치는 김에
가장 유망한 knob(`Stereo.ThDepth` 40 → 55/70)을 연다. RTAB-Map 튜닝에서 `Vis/MaxDepth` 가 최대 효과를
낸 전례(README.md:283-284)가 근거다. 1–2시간 작업으로 7개 파라미터가 열린다.

**④ 관측 가중치 실시간 적용 — 2주차 (§3.4 1·2단계).**
왜 뒤인가: 효과가 −12 ~ −22 % 로 확실하지만 작고, ②의 오프라인 검증이 먼저 끝나야 안전하다.
`score_floor` 함정(§3.6)이 있어 서두르면 오히려 관측을 잃는다.

**⑤ `n_tracked_mp` 를 라이브 포즈에 추가 — 1줄, 아무 때나 (§1.5f).**
`slam_stream.cc:535-541` 의 `json pose` 에 `{"n_tracked_mp", r.n_tracked_mp}` 를 넣기만 하면 된다.
값은 이미 `slam_stream.cc:331` 에서 계산돼 있다. 그러면 "45–70 s 에 특징이 몇 개였나"라는, 방향 1 전체를
진단하는 데 필요한 관측치를 얻는다. ③의 재빌드에 묶어서 하면 추가 비용 0.

**⑥ IMU — 조건부로 마지막 (§4).**
왜 마지막인가, 그리고 왜 "조건부"인가:
- 기존 데이터에 IMU 가 **아예 없고**(§4.1), `T_imu_cam` 도 없다. 1–2주 + 새 녹화가 전제다.
- 정지 35 초 시작 패턴이 ORB-SLAM3 의 IMU 초기화 게이트를 정면으로 위반한다(§4.2). 녹화 프로토콜
  자체를 바꿔야 한다.
- **결정적으로: LiDAR hdl_graph_slam 이 이미 흩어짐 1.62 cm, 실시간-최종 오차 3.7 cm 최대를 내고 있다.**
  IMU 가 잘 돼도 그보다 나아질 이유가 없다. 카메라만으로 가야 하는 제품 제약이 명확히 있을 때에만
  착수한다. 그 전까지는 **LiDAR 를 기준선이자 실운용 후보로 두고**, ORB 는 ①③④로 4.3 → 3 cm 대를 노리는
  것이 비용 대비 효과가 훨씬 낫다.

한 문장 요약: **오늘 30분이면 ①②를 시작할 수 있고, 1주면 ③⑤로 파라미터 공간이 열리며,
IMU 는 "카메라 단독으로 LiDAR 급 정확도가 반드시 필요하다"가 확정되기 전에는 착수하지 않는다.**

---

## 부록 A — 첫 실험을 실행하는 정확한 명령

### A.1 특징 수 스윕 (코드 변경 없이 오늘 가능)

전제: 허브는 한 번에 하나만 뜬다 (`objpose/run_one.sh:84-86` 이 다른 `hub.py` 를 감지하면 종료 코드 3).

```bash
cd /home/jucpark/DeepLearning/CLI_environment
pgrep -af 'objpose/pc/(hub|launcher)\.py'      # 비어 있어야 한다

# 기준 재현 (f2000) — Mac
bash objpose/run_one.sh 260910_object orb-mac --exit-when-done

# f3000 — Mac (추적 시간 여유가 가장 크다: 현재 10.76 / p99 13.65 ms)
bash objpose/run_one.sh 260910_object orb-mac --features 3000 --exit-when-done

# f4000 — Mac
bash objpose/run_one.sh 260910_object orb-mac --features 4000 --exit-when-done

# f3000 — .3 (여유가 적다: 현재 15.54 / p99 20.37 ms. p99 가 25 ms 를 넘거나
#              slam_dropped > 0 이면 여기서 멈춘다)
bash objpose/run_one.sh 260910_object orb --features 3000 --exit-when-done
```

`--features` 가 재컴파일 없이 통하는 경로:
`hub.py:1030` → `hub.py:320` 이 `~/objpose/run_slam.sh ... --features N` 생성 →
`mac_slam/run_slam.sh:40` → `slam_stream.cc:189` → `slam_stream.cc:268 settings_override()` 가
`slam_stream.cc:145-151` 에서 `ORBextractor.nFeatures` 를 임시 yaml 에 다시 쓴다.

각 실행은 159 초 데이터를 1배속 재생하므로 기동 포함 4–5분. 출력은
`objpose/output/one_260910_object_orb-mac_<타임스탬프>/`.

### A.2 결과 비교

```bash
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
RT=/home/jucpark/DeepLearning/CLI_environment/objpose/rt/260910
OUT=/home/jucpark/DeepLearning/CLI_environment/objpose/output

# 새 f3000 실행 vs 기존 f2000 기준 (둘 다 카메라라 같은 외부 파라미터)
$PY objpose/pc/compare_runs.py $OUT/<f3000 실행> $OUT/one_orb-mac_260914_125210 \
    --extrinsic-a $RT/X_slam_sam_260910.json \
    --extrinsic-b $RT/X_slam_sam_260910.json

# 새 실행 vs LiDAR 기준선 — 외부 파라미터를 반드시 서로 다르게 준다.
# (생략하면 LiDAR 센서 좌표와 카메라 좌표 차이 때문에 RMSE 가 6.4 대신 33 cm 로 나온다)
$PY objpose/pc/compare_runs.py $OUT/one_hdl-mac_260914_123836 $OUT/<f3000 실행> \
    --extrinsic-a $RT/X_lidar_sam.json \
    --extrinsic-b $RT/X_slam_sam_260910.json
```

합격선 (§2.4): `observation_spread_cm.median ≤ 3.0`, `live_vs_final_traj_cm.p95 ≤ 8`,
`track_ms_p99 ≤ 25`, `slam_dropped == 0`, `kf_max_correction_cm ≤ 20`,
`memory_kept == 8 && memory_classes == 8`.

### A.3 `--param` 구현 후의 스윕 (③ 이후)

```bash
# 가장 유망: 근접점 깊이 지평을 2.98 m → 4.10 m
bash objpose/run_one.sh 260910_object orb-mac --slam-param Stereo.ThDepth=55 --exit-when-done
# → 5.22 m
bash objpose/run_one.sh 260910_object orb-mac --slam-param Stereo.ThDepth=70 --exit-when-done
# 스케일 피라미드를 촘촘히
bash objpose/run_one.sh 260910_object orb-mac \
     --slam-param ORBextractor.nLevels=10 --slam-param ORBextractor.scaleFactor=1.15 --exit-when-done
# 저대비 실내에서 특징 확보
bash objpose/run_one.sh 260910_object orb-mac \
     --slam-param ORBextractor.iniThFAST=12 --slam-param ORBextractor.minThFAST=5 --exit-when-done
# 먼 맵포인트 차단
bash objpose/run_one.sh 260910_object orb-mac --slam-param System.thFarPoints=6.0 --exit-when-done
# 키프레임 강제 삽입 주기 1 s → 0.5 s (fps 는 다른 곳에도 쓰이므로 반드시 단독 검증)
bash objpose/run_one.sh 260910_object orb-mac --slam-param Camera.fps=15 --exit-when-done
# 최선 조합
bash objpose/run_one.sh 260910_object orb-mac --features 3000 \
     --slam-param Stereo.ThDepth=55 --slam-param System.thFarPoints=6.0 --exit-when-done
```

`--param` 을 구현하기 **전에** 급히 하나만 시험하고 싶다면, 허브를 거치지 않고 SLAM 호스트의 기본 설정 파일을
직접 고치는 우회로가 있다 (Mac: `~/objpose/settings/orbslam3_260826_dark_nf2000.yaml`,
`.3`: `~/objpose/mac_slam/settings/orbslam3_260826_dark_nf2000.yaml` —
`mac_slam/run_slam.sh:38`, `linux_slam/run_slam.sh:28`). 원본을 반드시 백업하고, 실험 후 되돌린다.

### A.4 오프라인 재추정 (SLAM 실행 없이, 기존 로그만으로)

부록 B 의 스크립트를 `objpose/pc/reestimate_offline.py` 로 정착시킨 뒤:

```bash
$PY objpose/pc/reestimate_offline.py $OUT/one_orb_260914_124614 \
    --reference $OUT/one_hdl-mac_260914_123836 \
    --extrinsic $RT/X_slam_sam_260910.json --reference-extrinsic $RT/X_lidar_sam.json \
    --weight kf-correction --d0 0.05
```

---

## 부록 B — 이 문서의 수치를 만든 분석 (재현용)

전부 읽기 전용이며 `slam_poses.jsonl` / `kf_updates.jsonl` / `sam6d_estimates.jsonl` / `summary.json` 만 쓴다.

**B.1 구간별 실시간 vs 최종 궤적 (§1.2).**
`kf_updates.jsonl` 을 처음부터 재생해 최종 키프레임 포즈 집합과 컬링 리다이렉트를 만들고
(`fusion.py:84-107` 과 같은 로직), 각 포즈에 대해
`T_final = T_w_kf_final[ref_kf] · mat(T_w_kf)⁻¹ · mat(T_wc)` 를 계산해 `T_wc` 와의 거리를 잰다.
`compare_runs.py:36-39` 와 같은 식이고, 시간 구간으로 쪼갠 것만 다르다.

**B.2 앵커 규칙 비교 (§3.2).**
각 관측 시각 직전의 `kf_update` 스냅샷을 "관측 당시 지도"로 삼아
`p = T_final[k] · T_snapshot[k]⁻¹ · T_w_obj_live` 를 네 가지 `k` 선택 규칙으로 계산하고,
객체별 중앙값으로부터의 거리 중앙값을 비교한다.

**B.3 LiDAR 기준 객체 오차와 가중치 실험 (§1.3, §3.3).**
`compare_runs.load_run()` 을 그대로 import 해 두 실행을 각자의 외부 파라미터로 SAM 카메라 좌표에 올리고,
LiDAR 시각에 ORB 궤적을 보간해 Umeyama 정렬(`compare_runs.py:51-57`)한 뒤,
LiDAR 객체 위치를 ORB 지도로 역변환하여 각 가중 규칙의 오차를 낸다.

**B.4 지렛대 길이 (§3.2).**
포즈마다 `T_kf_c = mat(T_w_kf)⁻¹ · mat(T_wc)` 의 평행이동 크기와 회전각.

**B.5 `kf_update` 가 전량 스냅샷인지 확인.**
`n` 필드가 단조 증가하고 최댓값이 `kf_count` 와 일치하는지 본다 (.3 1→183, Mac 1→213, HDL 1→105, 모두 단조).
이것이 참이어야 `fusion.py:93` 의 "새 업데이트에 없는 키프레임 = 컬링됨" 판정이 성립한다.
증분 업데이트로 바뀌면 모든 키프레임이 컬링으로 오인된다 — 향후 스트리머를 고칠 때 반드시 지켜야 할 불변식이다.

---

## 부록 C — 이 문서를 쓰며 발견한 버그·개선 후보 (별건)

1. **`--slam-param` 이 ORB 레인에서 실행을 죽인다** (§1.5a). `hub.py:1032-1034` 의 도움말이
   `ORBextractor.nFeatures=1500` 을 예로 드는데 `slam_stream.cc:200` 이 종료 코드 2 를 낸다.
   지금은 아무도 안 써서 드러나지 않았다 (`summary.json` 의 `slam_params: []`).
2. **`n_tracked_mp` 가 라이브 로그에 없다** (§1.5f). `slam_stream.cc:331` 에서 계산되지만
   오프라인 CSV(`:364`)에만 쓰인다.
3. **`compare_runs.py` 에 외부 파라미터를 안 주면 조용히 틀린다**. LiDAR vs 카메라 비교에서
   RMSE 6.36 이 33.12 로 나온다. `--extrinsic-a/-b` 없이 서로 다른 센서의 실행을 비교하면
   경고를 찍게 하는 게 좋다 (`compare_runs.py:93-95`).
4. **저장된 비교 결과에 어떤 외부 파라미터를 썼는지 기록이 없다**.
   `objpose/output/compare_260910_hdlmac_vs_orb.json` 을 재현하려면 추측해야 했다.
   `out` dict(`compare_runs.py:97`)에 `extrinsic_a/b` 경로를 넣으면 된다.
5. **`run_one.sh:84` 의 단독 실행 가드가 오탐한다.** `pgrep -f 'objpose/pc/hub.py'` 는 **커맨드라인에 그 문자열이
   들어 있기만 하면** 아무 프로세스나 잡는다. 2026-09-14 14:20–14:31 에 허브를 기다리려고 돌린 폴링 루프
   (`while pgrep -f 'objpose/pc/hub.py' ...`)가 자기 자신을 매칭해, 실제로는 허브가 없는데도 런 3개
   (`one_260910_object_hdl-3_260914_142100`, `_142107`, `one_260910_object_orb-mac_260914_142938`)가
   `"다른 허브가 이미 실행 중입니다 ... 145004 /bin/bash -c source ..."` 로 죽었다. 세 디렉터리 모두
   `hub_stdout.log` 한 줄만 남은 빈 상태다.
   고칠 방법: `pgrep -f` 대신 PID 파일이나 포트 점유(`ss -ltn 'sport = :8766'`)로 판정하거나, 최소한
   `pgrep -f 'python.*objpose/pc/hub\.py'` 처럼 인터프리터까지 묶어 매칭한다.
   (`run_one.sh` 는 현재 다른 작업자가 편집 중이라 이 문서에서는 지적만 하고 고치지 않았다.)
