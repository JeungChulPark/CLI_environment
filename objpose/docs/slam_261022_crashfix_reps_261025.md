# 261022 세션: ORB-SLAM3 크래시 원인 수정, digut·kiuk 반복 실행, cbnu_s 데이터 복구 (2026-10-25)

10-24에 261022 세션 6개를 변형 6개로 한 번씩 돌린 결과(`rt/261024_map`)에는 세 가지 문제가 있었다.
digut과 kiuk는 변형에 따라 ATE가 4~128 cm로 튀었다. digut 주력+깊이보정은 세 번 모두 죽었다.
cbnu_s는 LiDAR bag을 읽지 못했다. 이 문서는 이 세 가지를 차례로 다룬다.

- 기준: KISS-ICP 궤적(`rt/261024_map/<세션>/kiss`). rig는 run마다 자기 optimized 궤적으로 맞추고, 카메라 높이는 −0.357 m로 고정한다.
  run1과 같은 방식이며, `rt/261017_new/build_map_data.py`의 `fit_eval`을 그대로 쓴다.
- 변형: 수정본 / +깊이보정(`RGBD.DepthMapFactor=925`, 깊이 ×1.081) / 업스트림(UZ-SLAMLab v1.0) / 업스트림+깊이 /
  수정본+자이로 / 주력+깊이(수정본+자이로+깊이). 모두 f2000, offline 모드다.
- 스크립트: `rt/261024_map/run_reps.sh`(반복 실행, `OUT`과 `VARS`로 대상 지정), `rt/261024_map/eval_reps.py`(채점 → `reps.json`)

## 1. 요약

1. **크래시 원인은 `LoopClosing::CorrectLoop()`의 쓰레드 레이스였다.** 원본 v1.0에도 있는 버그다.
   수정 전에는 digut 주력+깊이가 27회 중 9회 죽었고, 수정 후에는 16회 모두 정상 종료했다.
   반복 실행 32회에서도 크래시는 0회였다.
2. **레이스를 고치자 kiuk의 이상값이 사라졌다.** 수정 전 kiuk 수정본은 optimized ATE가 16.9~76.6 cm로 흩어졌다.
   수정 후에는 16.9~21.9 cm로 모였다. 주력+깊이도 4.0~62.6 cm에서 3.8~4.8 cm로 좁아졌다.
3. **digut에서는 자이로가 결정적이다.** 자이로가 없으면 잘못된 루프를 받아들여 optimized ATE가 34~94 cm까지 나온다.
   자이로를 켜면 18.7 cm, 깊이보정까지 더하면 13.1 cm다.
4. **cbnu_s는 bag 세 개가 모두 손상돼 있었다.** LiDAR는 앞쪽 148.6초, SLAM 카메라는 앞쪽 59.3초만 살릴 수 있었다(전체 188초).
   두 데이터가 겹치는 59초 구간으로만 채점했다.

## 2. 크래시 원인과 수정

### 증상

10-24 하루에만 수정본 빌드에서 크래시 리포트 6건, 업스트림 빌드에서 1건이 나왔다.
어느 빌드인지는 `~/Library/Logs/DiagnosticReports`의 dylib UUID로 구분했다. 죽는 위치는 세 가지였다.

- `MapPoint::ComputeDistinctiveDescriptors()`: `pKF->mDescriptors.row(leftIndex)`에서 cv::Mat 행 범위 assertion
- `Optimizer::OptimizeEssentialGraph()`: `addEdge`에서 SIGSEGV(주소 0x18). 존재하지 않는 정점에 엣지를 붙이려 했다.
- `MapPoint::MapPoint()`: `mutex::lock`에서 system_error (업스트림 빌드)

### 원인

`-O3 -g` 빌드(`~/objpose/build_dbg`)로 크래시를 재현하고 `atos`로 줄 번호를 확인했다.
그 결과 `ComputeDistinctiveDescriptors`가 **LoopClosing 쓰레드**에서 실행되고 있었다.
호출 경로는 `CorrectLoop()` → `mpLocalMapper->EmptyQueue()` → `ProcessNewKeyFrame()`이다.

`CorrectLoop()`는 `RequestStop()` 직후, LocalMapping이 실제로 멈추기를 기다리지 않고 `EmptyQueue()`를 부른다.
그 사이 LocalMapping 쓰레드는 아직 자기 키프레임을 처리하는 중이다(`CreateNewMapPoints`, `MapPointCulling` 등).
두 쓰레드가 멤버 변수 `mpCurrentKeyFrame`을 서로 덮어쓰면, 한 키프레임의 특징점 인덱스가 다른 키프레임의 맵포인트 관측으로 들어간다.
이것이 위 세 가지 증상으로 나타난다. `MergeLocal()`은 멈출 때까지 기다린 뒤에 `EmptyQueue()`를 부르는데, `CorrectLoop()`만 순서가 반대다.
수정본은 루프 임계값을 완화해서 루프 보정이 훨씬 자주 일어나므로 크래시도 그만큼 자주 났다.

**수정** (`src/LoopClosing.cc`): `EmptyQueue()`를 `while(!isStopped())` 대기 뒤로 옮겼다.
LocalMapping은 큐가 비어야만 멈추고 Tracking은 정지 요청 중에 키프레임을 만들지 않으므로, 대기 뒤의 `EmptyQueue()`는
정지 요청 직후에 들어온 키프레임만 처리한다.

### 함께 고친 것: `OptimizableTypes.cpp`의 `auto` 야코비안

ASan 빌드(`~/objpose/build_asan`)에서 `EdgeSE3ProjectXYZ::linearizeOplus()`의 stack-use-after-scope가 잡혔다.
`auto projectJac = -pCamera->projectJac(...)`가 Eigen 표현식 템플릿으로 남아, 이미 사라진 임시 행렬을 참조한다.
그래서 local BA 야코비안을 해제된 스택에서 읽는다. `Eigen::Matrix<double,2,3>`으로 타입을 명시해 고쳤다.
원본 v1.0에도 같은 줄이 있다. 다만 **이것은 크래시 원인이 아니었다.** 이 수정만 넣은 빌드도 6회 중 3회 죽었다.

### 크래시율 (digut 주력+깊이)

| 빌드 | 실행 | 크래시 |
|---|---|---|
| 수정 전 (`build`, 10-18 빌드) | 9 | 3 |
| 수정 전 + `-g` (`build_dbg`) | 12 | 3 |
| 야코비안만 수정 (`build_fix` 1차) | 6 | 3 |
| **레이스 + 야코비안 수정** (`build_fix` 2차) | 16 | **0** |
| 같은 빌드, 반복 실행 (`rt/261025_fix`) | 32 | **0** |

수정 전 합계는 27회 중 9회다. 크래시율이 1/3 그대로라면 16번 연속 성공할 확률은 (2/3)^16 ≈ 0.15 %다.

**반영 위치**
- `~/objpose/ORB_SLAM3_src`: 운영 소스
- `~/objpose/build`: 수정 빌드로 교체했다. 이전 빌드는 `~/objpose/build.prev_261025`에 백업했다.
- 저장소: `objpose/ORB_SLAM3_src`, `orbslam_ws/src/ORB_SLAM3`
- 업스트림 빌드(`build_upstream`)는 비교 기준이라 수정하지 않았다. 따라서 업스트림 결과에는 이 레이스가 그대로 남아 있다.

## 3. digut·kiuk 반복 실행 (optimized ATE cm, 평균과 범위)

| 세션 | 변형 | 수정 전 (`rt/261024_map`) | 수정 후 (`rt/261025_fix`) |
|---|---|---|---|
| digut | 수정본 | 42.1 (35.6~48.9), 4/4 | 58.5 (34.3~85.6), 4/4 |
| digut | +깊이보정 | 61.7 (43.4~71.7), 4/4 | 80.1 (65.3~93.5), 4/4 |
| digut | 업스트림 | 39.6 (23.9~55.8), 4/4 | — |
| digut | 업스트림+깊이 | 40.2 (15.5~95.3), 4/4 | — |
| digut | 수정본+자이로 | 18.4 (17.7~19.0), 4/4 | 18.7 (17.5~19.5), 4/4 |
| digut | 주력+깊이 | 6.2, 1/6 완주 | **13.1 (7.7~21.0), 4/4** |
| kiuk | 수정본 | 36.9 (16.9~76.6), 3/4 | 19.9 (16.9~21.9), 4/4 |
| kiuk | +깊이보정 | 7.6 (3.7~18.1), 4/4 | 8.2 (4.1~19.3), 4/4 |
| kiuk | 업스트림 | 16.9 (16.2~18.2), 4/4 | — |
| kiuk | 업스트림+깊이 | 4.4 (3.6~5.1), 4/4 | — |
| kiuk | 수정본+자이로 | 16.3 (15.2~17.2), 4/4 | 18.0 (16.7~20.1), 4/4 |
| kiuk | 주력+깊이 | 18.7 (4.0~62.6), 4/4 | **4.3 (3.8~4.8), 4/4** |

live ATE와 run별 값은 `reps.json`에 있다(`rt/261024_map/reps.json`, `rt/261025_fix/reps.json`).

**digut에서 자이로가 없을 때 틀어지는 이유**: 잘못된 루프를 받아들이기 때문이다.
수정 후 run 16개의 로그(`LOOP_GAP`, `GYRO_LOOP_CHECK`)를 보면 다음과 같다.

| 변형 | 보정된 루프 | 자이로가 거부한 루프 | 루프 보정이 키프레임을 옮긴 최대 거리 |
|---|---|---|---|
| 수정본 / +깊이보정 | 2~4 | — | 3.2~4.4 m |
| 수정본+자이로 / 주력+깊이 | 2~3 | 2~28 | 0.64~1.19 m |

digut에서 깊이보정은 자이로가 없으면 오히려 나빠지고(58.5 → 80.1 cm), 자이로가 있을 때는 좋아진다(18.7 → 13.1 cm).
잘못된 루프가 맵을 망가뜨린 상태에서는 깊이 스케일 보정의 효과가 드러나지 않는 것으로 보인다.

## 4. cbnu_s 데이터 복구

세 bag 모두 `pragma quick_check`에서 `btreeInitPage() error 11`이 났다. 세 파일 모두 10-21 20:04에 변환된 것이다.

| bag | 원본 | 복구 가능 구간 |
|---|---|---|
| lidar (`user-device_velodyne_..._0.db3`) | 188.0초, `/velodyne_points` 1865개 | 0~148.6초, 1474개 |
| SLAM 카메라 (`SLAM_0.db3`) | 컬러 5631장 | 0~59.3초, 1781장 |
| SAM 카메라 (`SAM_0.db3`) | — | 같은 방식으로 손상됨(복구 안 함) |

- 복구 방법은 `sqlite3 <원본> .recover | sqlite3 <새 파일>`로 새 파일을 만든 뒤 `lost_and_found` 테이블(쓰레기 행 33개)을 지우는 것이다.
- 테이블 leaf 페이지를 직접 훑어봐도 148.6초 이후의 메시지 행은 어디에도 없었다. 뒷부분은 파일 안에 아예 남아 있지 않다.
- 결과물은 `Dataset/261022_cbnu_s/lidar_recovered/`에 두었다(README 포함). 원본은 건드리지 않았다.
- SLAM 카메라 bag은 `slam_stream`이 깨진 부분 앞까지 그대로 읽으므로(1779프레임) 따로 복구본을 만들지 않았다.
- 녹화 PC(`user-device:converting_launch/input/cbnu_s`)에 원본이 남아 있다면 다시 가져오는 것이 가장 좋다.

**cbnu_s 채점** (수정 전 빌드, 1회씩, 59초 구간, optimized ATE cm)

| 수정본 | +깊이보정 | 업스트림 | 업스트림+깊이 | 수정본+자이로 | 주력+깊이 |
|---|---|---|---|---|---|
| 19.2 (2회: 17.7, 20.7) | 5.9 | 19.6 | 12.3 | 16.8 | 5.8 |

## 5. 남은 일

- 나머지 261022 세션(eight, long, long_obchange, kiuk_obchange)을 수정 빌드로 다시 돌리고, 3D 맵 페이지(`html/260918_slam_3d_map`)를 갱신하기
- 레이스가 들어 있던 10-18 빌드로 만든 기존 결과의 재검증. 260901 2×2, OpenLORIS, slam_bench가 대상이고, 특히 크게 튄 run을 먼저 본다.
- 업스트림에도 같은 레이스가 있다는 점을 논문에서 "업스트림 크래시 수정" 항목에 추가하기
