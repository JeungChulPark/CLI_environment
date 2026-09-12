# ORB-SLAM3 nFeatures 2000을 실시간(33.33 ms)으로 만들기

작성: 2026-09-13 · 이 노트북(i9-12900HK / RTX 3080 Ti Laptop / WSL2)에서 실측
데이터셋: `260826_etri_eightcircle_dark` (RealSense D455F, 640x480 RGB-D, 285 s, 7144 프레임)
빌드: CPU only (CUDA OpenCV 없음), `-O3 -march=native`, `REGISTER_TIMES` 활성

---

## 1. 문제

nFeatures를 2000으로 올리면 실시간이 깨진다.

| nFeatures | mean | p95 | **p99** | max | **마감 초과** |
|---|---|---|---|---|---|
| 800 | 12.50 | 14.94 | 16.78 | 29.81 | 0 |
| 1250 | 16.67 | 20.81 | 23.40 | 42.06 | 2 |
| **2000** | **24.68** | **31.17** | **35.64** ❌ | 43.64 | **170 / 7142 (2.4%)** |

P99가 마감(33.33 ms)을 넘고 7142프레임 중 170개가 초과한다.

이 머신에서 별도로 측정한 지터 한계선과도 일치한다 — 30 FPS 주기로 고정 연산을 돌리며
지연 분포를 잰 결과, **연산 예산 20 ms에서 P99 28.69 ms / 마감 초과 0%**, 22 ms부터
놓치기 시작하고 25 ms에서 명확히 실패한다. mean 24.68 ms는 그 실패 구간에 있다.

## 2. 진단 — 처음 세운 가설은 틀렸다

단계별 분해(`REGISTER_TIMES`)를 nFeatures별로 비교하면:

| 단계 | 800 | 1250 | 2000 | 800→2000 |
|---|---|---|---|---|
| ORB extraction | 6.68 | 7.25 | 8.46 | **+27%** |
| Pose prediction | 1.44 | 2.46 | 4.41 | **+206%** |
| LM track | 2.94 | 5.06 | 8.95 | **+204%** |

feature를 2.5배로 늘려도 **ORB extraction은 +27%에 그친다.** 피라미드 생성과 FAST 검출이
지배적이고 이들은 픽셀 수에만 의존하기 때문이다. 실제로 3배가 되는 건 매칭·최적화 쪽이다.

여기서 **"병목은 `Optimizer::PoseOptimization()`의 프레임당 40~120 LM iteration"** 이라는
가설을 세웠다. `Tracking.cc:2827`(또는 `2991`)과 `3053`(+`3059`)에서 프레임당 2~3회 호출되고,
각 호출이 `const int its[4]={10,10,10,10}`(`Optimizer.cc:1003`)으로 4라운드 x 10 iteration을
dense solver로 돈다.

**이 가설은 실측으로 반박됐다.** 반복 횟수를 40 → 25로 줄인 실험(B)이 사실상 아무 효과가
없었다(16.34 → 16.37 ms). 진짜 비용은 g2o 최적화가 아니라 **맵 포인트 수에 비례하는
부대비용**이었고, nFeatures 2000에서 맵 포인트가 12k → 20k로 늘자 그 부대비용이 함께
폭증한 것이다. 그래서 2000만 마감을 넘겼다.

## 3. 적용한 수정

### 3-1. 결과 보존 수정 (bit-identical) — 이득의 대부분

모두 프레임당 맵 포인트 수에 비례하던 낭비다. 별도 세션에서 만들어 둔
`perf_patches/orbslam3-perf-all.patch`와 `04-octree-node-reserve.patch`가 대부분을 담고 있고,
`isInFrustum` 건은 이번에 추가했다.

| # | 파일 : 함수 | 내용 |
|---|---|---|
| 1 | `Tracking.cc:2221` `Tracking::Track()` | viewer가 꺼져 있는데도 매 프레임 `FrameDrawer::Update(this)`가 **Frame 전체를 deep copy**했다 — descriptor clone + 64x48 그리드 셀 벡터 전부 + `mmProjectPoints`/`mmMatchedInImage` 맵 + 로컬 맵 포인트 벡터. `if(mpViewer)`로 감쌌다. `mpViewer`는 `System`이 `bUseViewer=true`로 생성될 때만 non-NULL이다(`System.cc:229-236`). |
| 2 | `Tracking.cc:3461` `Tracking::SearchLocalPoints()` | `mmProjectPoints`에 **가시 맵 포인트마다 RB-tree 삽입**. 유일한 소비자가 `FrameDrawer`(`FrameDrawer.cc:89`, `:393`)라 headless에선 순수 낭비. `if(mpViewer && pMP->mbTrackInView)`로 제한. |
| 3 | `Tracking.cc:3549,3572` `Tracking::UpdateLocalKeyFrames()` | 맵 포인트마다 `GetObservations()`가 **`std::map`을 값으로 반환** — red-black tree 전체 deep copy, 관측 하나당 노드 할당 하나. `MapPoint::AccumulateObservingKeyFrames()`로 대체해 같은 잠금 아래 제자리 누적, 할당 0. |
| 4 | `Frame.cc:657` `Frame::GetFeaturesInArea()` | `vIndices.reserve(N)`가 **매 호출 N*8 바이트 힙 할당**(실제 반환은 O(10)개). 그리고 `const vector<size_t> vCell = ...` 의 `&` 누락으로 **방문 셀마다 값복사**(호출당 9~25셀). |
| 5 | `ORBmatcher.cc:2053` `ORBmatcher::DescriptorDistance()` | SWAR 비트핵 popcount → `__builtin_popcountll`. 20개 호출 지점에서 초당 수백만 회 실행. 별도 마이크로벤치에서 **1.62~1.94배** 확인. |
| 6 | `Frame.cc:512` `Frame::isInFrustum()` **(이번 추가)** | `GetWorldPos` / `GetMaxDistanceInvariance` / `GetMinDistanceInvariance` / `GetNormal`이 **모두 같은 `mMutexPos`를 각각** 잡는다. 로컬 맵 포인트마다 프레임당 실행되므로 nFeatures 2000에서 프레임당 수천 번의 중복 잠금. `MapPoint::GetFrustumData()` 하나로 통합. |
| 7 | `ORBextractor.cc` `ExtractorNode::DivideNode()` | 자식 노드마다 부모 전체 크기를 `reserve`해 분할마다 4배 과할당. 평균적으로 자식은 부모의 1/4을 받으므로 `vKeys.size()/4 + 8`로 변경. 용량 힌트만 바뀌므로 keypoint는 동일. |
| 8 | `ORBextractor.cc` 레벨 루프 3곳 | **피라미드 레벨 단위 OpenMP 병렬화**. FAST+octree, orientation, blur+descriptor 루프를 `#pragma omp parallel for num_threads(ORBEXTRACTOR_OMP_THREADS)`(기본 4)로 실행. 레벨 간 의존이 없어 결과는 동일하며, descriptor를 최종 배열로 모으는 scatter만 직렬로 남긴다. **스레드 수를 4로 고정하는 것이 필수다**(6장 참조). |

6번의 동등성 근거: 네 접근자가 반환하는 것은 각각 `mWorldPos`, `mNormalVector`,
`0.8f*mfMinDistance`, `1.2f*mfMaxDistance`이고 모두 `mMutexPos` 아래에서만 쓰인다.
새 접근자는 같은 계수로 같은 멤버를 한 번의 잠금으로 읽는다. 사용 순서도 바꾸지 않았다.

### 3-2. 정확도에 영향 가능한 수정 (yaml로 opt-in, 기본 off)

`Optimizer.h` / `Optimizer.cc`에 두 개의 정적 튜너블을 추가하고 `Tracking` 생성자에서
설정 파일을 읽어 채운다(키가 없으면 원래 동작 그대로).

```yaml
Optimizer.PoseIterations: [10, 5, 5, 5]   # 라운드별 LM iteration (기본 10 10 10 10)
Optimizer.PoseEarlyExit: 1                # outlier 집합이 안정되면 조기 종료 (기본 0)
```

조기 종료 조건은 `it>=2 && nBad==nBadPrev`다. `it==2`에서 robust kernel이 제거되므로
(`Optimizer.cc`의 `if(it==2) e->setRobustKernel(0);`), 그 이후 inlier/outlier 분류가 더 이상
바뀌지 않으면 다음 라운드는 **이미 수렴한 자세를 동일한 엣지 집합으로 다시 최적화**하는
것에 불과하다.

## 4. 결과

| 구성 | mean | p95 | **p99** | max | **>33.33 ms** |
|---|---|---|---|---|---|
| 2000 baseline | 24.68 | 31.17 | **35.64** ❌ | 43.64 | **170** |
| **A** 결과보존 수정만 | **16.34** | 20.01 | **21.71** ✅ | 30.86 | **0** |
| B  A + 반복 10/5/5/5 | 16.37 | 20.25 | 22.12 | 39.51 | 1 |
| **C** B + 조기종료 | **15.38** | **18.80** | **20.34** ✅ | **27.03** | **0** |
| (참고) 1250 stock | 16.67 | 20.81 | 23.40 | 42.06 | 2 |

**P99 35.64 → 20.34 ms (-43%), 마감 초과 170 → 0.**
그리고 수정된 2000이 **수정 전의 1250보다 빠르다**(15.38 vs 16.67 ms).

단계별로는:

| 단계 | baseline | A(패치) | 변화 |
|---|---|---|---|
| ORB extraction | 8.46 | **3.63** | **-57%** |
| Pose prediction | 4.41 | 3.85 | -13% |
| LM track | 8.95 | 7.51 | -16% |

ORB extraction이 절반 이하가 된 주된 이유는 **패치가 피라미드 레벨 단위 OpenMP 병렬화를
함께 켜기 때문**이다. `CMakeLists.txt`에 `ORBEXTRACTOR_OMP_THREADS`(기본 4)가 추가되고,
`ORBextractor.cc`의 FAST+octree / orientation / descriptor 세 루프가
`#pragma omp parallel for num_threads(4)`로 돈다. A/B/C 빌드의 컴파일 플래그에
`-DORBEXTRACTOR_OMP_THREADS=4`가 실제로 들어가 있음을 확인했다.

레벨은 서로 독립이다 — 각 반복은 `mvImagePyramid[level]`만 읽고 `allKeypoints[level]`만
쓰며, `DistributeOctTree()`는 멤버 상태를 바꾸지 않는다. 8.46 → 3.63 ms(약 2.3배)는
4스레드 병렬화로 설명되는 크기다(가장 큰 레벨이 하한).

스레드 수를 4로 **명시**한 것이 중요하다. 이 머신에서 별도로 측정한 WSL2의 OpenMP
parallel region 진입 비용은 20스레드 3.7858 ms vs 4스레드 0.0015 ms로 **2500배** 차이가
난다. 기본값(논리 CPU 20개)에 맡겼다면 병렬화가 오히려 손해였을 것이다.

> 주의: 이 절의 A/B/C 수치는 "결과 보존 수정"과 "OpenMP 병렬화"가 **함께 적용된** 값이다.
> 둘을 분리하려면 `-DORBEXTRACTOR_OMP_THREADS=1`로 다시 빌드해 측정해야 한다. 아래
> "기여도 분해"의 A 항목도 같은 이유로 두 요인의 합이다.

### 기여도 분해

- **결과 보존 수정 + OpenMP 레벨 병렬화(A)**: 24.68 → 16.34 ms = **-8.34 ms, 전체 이득의 90%**
  (두 요인의 합. 분리 측정은 하지 않았다.)
- 반복 축소(B): 16.34 → 16.37 ms = **0 (효과 없음)**
- 조기 종료(C): 16.37 → 15.38 ms = **-0.99 ms, 이득의 10%**

## 5. 정확도 검증 — 손상 없음

8자 코스이므로 시작-끝 폐합 오차가 누적 드리프트의 직접 지표다.

| 구성 | 폐합 오차 | 경로 길이 | 키프레임 | 맵 포인트 |
|---|---|---|---|---|
| 2000 baseline | 26.6 cm | 36.16 m | 160 | 19,855 |
| A | 26.7 cm | 36.14 m | 170 | 20,715 |
| **C** | **26.5 cm** | 36.44 m | 164 | 20,255 |
| (참고) 1250 stock | 26.8 cm | 36.98 m | 152 | 12,154 |

폐합 오차가 26.5~26.7 cm로 전부 동일하다(차이 2 mm, 측정 잡음 수준).
**속도를 정확도와 맞바꾼 것이 아니다.**

부수적으로 확인된 사실: nFeatures 1250 → 2000은 맵 포인트를 12.1k → 20.2k(+67%)로
늘리지만 **폐합 오차는 26.8 → 26.5 cm로 3 mm밖에 개선하지 않는다.** 즉 feature 수를
올려 얻는 것은 **지도 밀도**이지 **궤적 정확도**가 아니다.

## 6. 권장 설정

```yaml
ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Optimizer.PoseEarlyExit: 1
# Optimizer.PoseIterations: [10, 5, 5, 5]   # 선택 — 아래 참고
```

`PoseIterations` 축소는 **B가 A보다 나아지지 않았으므로 넣지 않아도 된다.**
조기 종료는 "수렴 후 동일 엣지 집합 재최적화"만 건너뛰므로 반복 축소보다 안전하고,
실측 이득도 그쪽에서 나왔다. 반복 축소까지 쓸 경우 ATE 회귀를 함께 볼 것.

빌드 옵션 (기본값 4, 그대로 두면 된다):

```bash
-DORBEXTRACTOR_OMP_THREADS=4   # 피라미드 레벨 병렬 추출. 1이면 직렬
```

실행 환경도 함께 지킬 것 (별도 실측 근거):

```bash
export OMP_NUM_THREADS=6      # 기본값(논리 CPU 20)이면 parallel region 하나에 3.79 ms
```

WSL2에서 OpenMP parallel region 진입 비용은 20스레드 3.7858 ms vs 4스레드 0.0015 ms로
**2500배** 차이가 난다. 첫 region은 10.5 ms를 먹으므로 시작 시 워밍업도 권장한다.

## 7. 남은 한계

- **worst case는 여전히 보장되지 않는다.** C의 max는 27.03 ms로 마감 안이지만, 이는
  이 시퀀스에서의 관측값이다. `LoopClosing`의 Global BA가 맵 뮤텍스를 잡는 구조
  (`LoopClosing.cc:1858/1887` ↔ `Tracking.cc:1886`)는 그대로이므로 더 큰 맵에서는 재현될 수 있다.
  이 실행의 맵 규모(164 KF / 20k MP)에서는 GBA의 Map Update가 4.72 ms에 그쳤다.
- **입력이 25.05 Hz였다.** 녹화 자체의 RGB-D 페어링 성공률이 89.7%이고(녹화 도구의
  associations도 89.0%), 재생 브리지의 depth->color 정합이 프레임당 8.0 ms 든다.
  프레임당 tracking 비용은 영향받지 않지만 진짜 30 Hz에서는 백그라운드 경합이 조금 더 심해진다.
- **`REGISTER_TIMES`의 단계별 CSV 일부 행이 오염된다.** 단계별 벡터 길이가 서로 다른데
  총 프레임 수로 인덱싱해 일부 행이 stale memory를 읽는다(최대 1e18 같은 값). 위 단계별
  수치는 해당 행을 버리고 집계한 것이며, `Total` 열은 노드의 독립 측정과 정확히 일치한다.
- **GPU는 쓰지 않았다.** 두 conda 환경의 OpenCV가 CUDA 없이 빌드되어 `cv::cuda` 경로가
  없다. 위 결과는 전부 CPU-only이며, 그 상태로 nFeatures 2000이 실시간이다.

## 7-1. 재현 시 주의 — WSL2 벽시계 역행이 맵을 리셋시킨다

측정 자체를 무효로 만들 수 있는 함정을 하나 확인했다.

`scripts/rs_bag_bridge.py --stamp-now` 은 발행 시점의 **벽시계**(`Node.get_clock().now()`,
기본적으로 시스템 시계)로 프레임에 타임스탬프를 찍는다. WSL2의 시스템 시계는 Windows 호스트와
주기적으로 동기화되면서 **뒤로 점프할 수 있다.**

실제로 한 실행에서 이 일이 벌어졌다. `TrackPerFrame.csv`의 `stamp` 열을 검사한 결과:

| 실행 | 타임스탬프 역행 | 최대 역행 | 맵 생성 | 추적 프레임 |
|---|---|---|---|---|
| `nf2000_C` | 0회 | — | 1 | 7144 |
| `map_1.0x` | 0회 | — | 1 | 7144 |
| `nf2000_live` | **6회** | **-482.5 ms** | **7** | 7104 |

역행이 일어난 프레임(805, 2385, 3172, 3983, 4795, 5599)이 로그의
`ERROR: Frame with a timestamp older than previous frame detected!` 및 그 직후의
`Creation of new map with id: N` 과 **정확히 일치**한다. ORB-SLAM3는 비단조 타임스탬프를
보면 Atlas에 새 맵을 만들고 기존 맵을 보관한다 — 정상적인 방어 동작이지만, 측정 입장에서는
궤적이 조각나고 지연 꼬리가 오염된다(그 실행의 max는 76.8 ms였다).

**대책**: 재생 브리지가 벽시계 대신 **프레임 인덱스에서 유도한 단조 타임스탬프**를 찍게 할 것
(`stamp = t0 + frame_index / 30.0`). 재생 속도는 여전히 실시간으로 맞추되, 스탬프 자체는
구조적으로 단조가 된다. 측정 전에는 항상 `TrackPerFrame.csv`의 `stamp` 열이 단조인지,
로그에 `Creation of new map` 이 예상보다 많지 않은지 확인할 것.

본 문서의 4~5장 수치(baseline / A / B / C)는 모두 역행 0회 실행에서 얻은 것이다.

## 8. 재현 방법

```bash
cd orbslam_ws/src/ORB_SLAM3
git apply ../../perf_patches/orbslam3-perf-all.patch
git apply ../../perf_patches/04-octree-node-reserve.patch
# isInFrustum / PoseOptimization 수정은 현재 작업 트리에 직접 적용되어 있음 (git diff 참조)

cd ../..
conda activate orbslam3
export CC=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc
export CXX=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++
export MAKEFLAGS="-j6"
colcon build --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_C_COMPILER=$CC -DCMAKE_CXX_COMPILER=$CXX \
  -DCMAKE_CXX_FLAGS="-DREGISTER_TIMES" \
  -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
```

측정은 `scripts/rs_bag_bridge.py`로 bag을 재생하고
`output/<run>/TrackPerFrame.csv`(프레임별 end-to-end)와
`TrackingTimeStats.txt`(단계별)를 집계한다. 정확도는 `CameraTrajectory.txt`의
시작-끝 거리(폐합 오차)로 본다.

## 9. 교훈

1. **가설을 먼저 측정으로 검증할 것.** "40 LM iteration이 병목"이라는 그럴듯한 가설이
   실측에서 0의 효과를 냈다. 단계별 계측을 켜지 않았다면 반복 횟수를 줄이고
   정확도 리스크만 떠안은 채 "최적화했다"고 결론냈을 것이다.
2. **headless 실행에서 시각화 코드 경로를 반드시 확인할 것.** 가장 큰 단일 이득이
   "꺼져 있는 viewer를 위해 매 프레임 Frame 전체를 복사하던 것"의 제거였다.
3. **비용이 무엇에 비례하는지 볼 것.** nFeatures를 올렸을 때 터진 것은 feature 수에
   비례하는 항이 아니라 맵 포인트 수에 비례하는 항이었다.
4. **평균이 아니라 P99를 볼 것.** baseline의 mean 24.68 ms는 33.33 ms 안이지만
   P99 35.64 ms로 실패한다.
