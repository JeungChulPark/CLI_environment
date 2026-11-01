# 새 주력 설정 `orb-gyro-fl-dp` (2026-11-01)

2026-11-01부터 "주력"은 PC 허브(192.168.219.200)의 런처 버튼 `orb-gyro-fl-dp`를 가리킨다.
이 설정은 수정본 ORB-SLAM3(`~/objpose/build_fl`)에 자이로 보조와 루프 수정 다섯 가지, 깊이보정 1035를 더한 것이다.
그 전의 주력은 `orbslam_ws/src/ORB_SLAM3`에 `--gyro`를 붙인 것(ours_gyro)이었다. 그 빌드에는 아래의 루프 수정이 없어서 이제는 주력이 아니다.

- 출처: PC 세션이 Mac에 남긴 `~/objpose/MAIN_CONFIG_261101.md`.
  PC 쪽 원문은 `objpose/docs/loop_fix_261101/README.md`와 `orb_slam3_261101.patch`다.
- 코드 변경점은 Mac의 `~/objpose/ORB_SLAM3_src`를 백업 `*.bak-261101`과 비교해 확인했다.
- 검증 수치(3절)는 PC에서 잰 것이다. Mac에서 다시 재지는 않았다.

## 1. 실행 명령

```bash
ORB_BUILD=/Users/user/objpose/build_fl ~/objpose/run_slam.sh \
  --gyro rig_1029_gyro.json \
  --param RGBD.DepthMapFactor=1035.0 \
  --param Loop.FalseMinCorrM=3.0 \
  --param Loop.FalseResidFrac=0.25 \
  --param Loop.MaxGapM=2.0 \
  --param Loop.PostLoopRelocM=0.5 \
  --param Loop.EarlyUnlock=1 \
  --slam orbslam3 --session <세션>/SLAM \
  --features 2000 --time-source frame --mode live \
  --connect 192.168.219.200:17004
```

- 1029 세션은 `rig_1029_gyro.json`, 1030 세션은 `rig_1030_gyro.json`을 쓴다(`~/objpose/settings/`).
- `run_slam.sh`에 `--settings`를 주지 않으면 `~/objpose/settings/orbslam3_260826_dark_nf2000.yaml`이 기본값이다.
  `--param`은 그 위에 값을 덮어쓴다. 어휘 사전은 `~/objpose/Vocabulary/ORBvoc.txt`다.
- `--mode live --time-source frame`: 프레임 타임스탬프에 맞춰 30 fps로 재생한다. 결과는 PC의 17004 포트로 보낸다.

### 관련 키워드

| 키워드 | 가리키는 것 |
|---|---|
| `orb-gyro-fl-dp` | 이 설정 전체(PC 허브 버튼 이름) |
| `build_fl` | 바이너리 `~/objpose/build_fl/slam_stream`, `libORB_SLAM3.dylib` (2026-11-01 07:57 빌드) |
| `*.bak-261101`, `b`, `c` | 수정 전 소스 백업(`~/objpose/ORB_SLAM3_src/{src,include}`) |
| `loop_fix_261101` | PC 쪽 정리 문서와 패치 |
| `dmf1035` | 깊이보정 1035에 맞춘 rig 파일 `X_slam_sam_<날짜>_dmf1035.json`(PC 쪽) |

`fl`과 `dp`의 원래 뜻은 어디에도 적혀 있지 않다. 각각 false-loop(루프 수정)와 depth(깊이보정)로 짐작될 뿐이다.

## 2. 설정 항목

| 파라미터 | 값 | 기본값 | 하는 일 |
|---|---|---|---|
| `Loop.EarlyUnlock` | 1 | 0 | 루프 보정에서 essential graph 최적화 전에 tracking 잠금을 푼다 |
| `Loop.PostLoopRelocM` | 0.5 | 0 (끔) | 루프 뒤 오래된 KF로 재위치를 시도하고, 0.5 m 넘게 어긋나면 그 위치로 점프한다 |
| `Loop.FalseMinCorrM` | 3.0 | 0 (끔) | 거짓 루프 거부: 보정량 기준(m) |
| `Loop.FalseResidFrac` | 0.25 | 0.25 | 거짓 루프 거부: 잔차 비율 기준 |
| `Loop.MaxGapM` | 2.0 | 0 (끔) | 보정 전 KF끼리 2 m보다 멀리 떨어진 루프는 미룬다 |
| `RGBD.DepthMapFactor` | 1035.0 | yaml 값 | 깊이 스케일 보정 |
| `--gyro` | rig json | 없음 | Xsens 자이로 보조(`Gyro.Use: 1`) |
| `--features` | 2000 | — | ORB 특징점 2000개, scaleFactor 1.2, nLevels 8, FAST 20/7 |

`PostLoopReloc`에는 명령에 나오지 않는 보조 파라미터 세 개가 있고, 모두 기본값으로 쓴다.
`Loop.PostLoopRelocFrames=300`(루프 뒤 약 10초 동안 검사), `Loop.PostLoopRelocStride=5`(5프레임마다 시도),
`Loop.PostLoopOldKF=100`(루프 KF보다 id가 100 이상 작은 KF만 후보로 씀)이다.

## 3. 각 수정이 고친 문제

### 3.1 EarlyUnlock: 루프 직후 tracking 정지

- **증상:** 1029 all live에서 시작점 루프 직후 tracking이 `mMutexLoopCorrection`을 3.4~11.7초 기다렸다.
  그 사이 102~351프레임이 버려졌다. PC 문서에 따르면 3~21초 동안 멈췄다가 위치가 1.6~2.5 m 무너지는 일이 약 3번에 1번 있었다.
- **수정(`LoopClosing::CorrectLoop`):** 현재 KF와 이웃 KF, 그 맵포인트의 보정과 fusion이 끝나면 잠금을 푼다.
  이 시점에는 LocalMapping이 멈춰 있어 새 KF가 들어오지 않는다. 그래프 결과는 `mMutexMapUpdate` 아래에서 쓰이므로
  최적화가 도는 동안에도 tracking을 계속할 수 있다.
- **로그:** `LOOP_TIMING|...|early_unlock=1|tracking_blocked_ms=...`. tracking 쪽은 100 ms 넘게 기다리면 `LOOPLOCK_WAIT_MS`를 찍는다.

### 3.2 PostLoopReloc: 루프 뒤 어긋난 위치

- **증상:** 루프 보정 뒤 현재 위치가 수 m 어긋났는데도, 방금 보정한 로컬 맵에 대해서는 tracking이 "OK"로 남았다.
  1029 all live에서는 tracking이 우연히 실패해 재위치하지 않는 한 70초 동안 1.6~2.1 m 어긋나 있었다.
  r5에서는 루프 0.17초 뒤에 맞았던 위치가 3초 뒤에 2 m 어긋났다.
- **수정(`Tracking::PostLoopRelocCheck`):** 루프가 끝나면 `CorrectLoop`이 tracking에 검사 시작을 알린다.
  그 뒤 300프레임 동안 5프레임마다 현재 프레임을 오래된 KF(첫 바퀴)에 재위치시키고, tracking 위치와 비교한다.
  - 차이가 0.5 m 이하이면 tracking 위치를 그대로 두고 검사를 이어 간다(`verdict=consistent`).
  - 차이가 0.5 m보다 크면 재위치한 쪽으로 점프하고 로컬 맵으로 다시 다듬는다(`verdict=JUMP`).
  - 다듬기가 실패하면 원래 위치를 유지한다(`localmap_fail_keep`). 재위치에 실패하면 `nomatch`다.
- `Relocalization()`에 후보 KF id 상한 인자(`maxCandKFId`)를 더했다. 기본값 −1은 기존 동작과 같다.
- **로그:** `POSTLOOP_ARM`, `POSTLOOP_RELOC|...|diff_m=...|verdict=...`

### 3.3 거짓 루프 거부 (`FalseMinCorrM`, `FalseResidFrac`)

- **증상:** 1029_etri_library_all의 거짓 루프는 보정량이 5.7~6.2 m였다. 보정 뒤 위치는 상대 KF에서 0.2~0.3 m 떨어져 있었고,
  보정 전 거리는 5.8~6.1 m였다.
- **판정:** 보정량 `corr > 3.0 m`이면서 보정 뒤 잔차 `resid < 0.25 × 보정 전 거리`이면 루프를 거부한다.
  즉 먼 KF를 통째로 끌어다 붙이는 루프만 막는다.
- **주의:** `FalseMinCorrM`을 1.0으로 낮추면 무너진 run을 되살리는 복구 루프까지 거부한다. 3.0을 유지해야 한다.
- **로그:** `FALSE_LOOP_CHECK|...|corr_m=...|resid_m=...|verdict=VETO|ok`

### 3.4 먼 루프 미루기 (`MaxGapM`)

- **증상:** 시작점을 다시 지날 때, 보정 전 3.2~4.4 m 떨어진 KF끼리 닫힌 루프는 0.3~1.6 m 어긋났다.
  0.2~1.1 m 떨어진 루프는 3 cm 안으로 맞았다.
- **수정:** 보정 전 KF 거리가 2.0 m보다 크면 그 루프는 받지 않는다. 더 가까워진 뒤 다시 잡히는 루프를 기다린다.
- **로그:** `LOOP_GAP_CHECK|...|gap_m=...|limit_m=2.000`

거짓 루프 거부와 먼 루프 미루기는 10-31에 들어간 기능이다. 261101에서 바뀐 것은 이 두 기능을 켜는 값이다.

### 3.5 깊이보정 (`DepthMapFactor=1035`)

- SLAM 카메라 깊이를 1035로 나눈다. 원래 1000이므로 깊이를 약 0.966배(1000/1035) 줄이는 셈이다.
- PC 쪽에서는 SAM depth ×0.96618, rig `X_slam_sam_<날짜>_dmf1035.json`과 짝으로 쓴다. 따로 바꾸면 SLAM과 SAM의 스케일이 어긋난다.

### 3.6 자이로 rig (`rig_1029_gyro.json`)

- 외부 Xsens 자이로를 쓴다. 카메라 시계 기준 IMU 시간차는 −2.0 ms, |ω| 상관은 0.9994다.
- R_cam_imu와 bias는 `1029_etri_library_middle_2` 하나만으로 맞췄다(표본 19393개). mapping 세션은 상관이 0.959로 낮아서 뺐다.
- SLAM 카메라 시리얼은 419222301870이다. 맞춘 방법은 `calib_gyro_261022.py`(|ω| 시간 이동, Kabsch, R/bias 공동 최소제곱)다.

## 4. 검증 (PC 측정, 2026-11-01)

| 세션 | 반복 | 결과 | ATE |
|---|---|---|---|
| 1029_etri_library_all | 40회 연속 | 모두 통과 | 14.9~19.3 cm |
| 1030_etri_library_all | 40회 연속 | 0회 실패 | 14.7~23.8 cm |

## 5. Mac에서 아직 맞지 않는 곳

- **웹 허브(`orbslam_ws/mac_harness`, 127.0.0.1:8080)의 "ORB-SLAM3 주력 (+gyro)" 버튼(`orbg`)은 예전 주력이다.**
  `orbslam_ws/output/mac_build/orb3/build/live_orbslam`에 `--gyro`만 붙여서 돌리므로 3.1~3.5절의 수정이 빠져 있다.
  새 주력을 웹에서 보려면 `build_fl` 기반 백엔드를 따로 붙여야 한다.
- 저장소 `CLI_environment/objpose/`는 `~/objpose`의 사본이라, 261101 소스 변경과 `build_fl`은 아직 들어오지 않았다.
- 2026-11-01 이전 주력(ours_gyro)으로 낸 결과는 새 주력 결과로 보면 안 된다.
