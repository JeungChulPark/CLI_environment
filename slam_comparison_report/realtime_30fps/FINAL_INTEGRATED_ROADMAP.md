# ORB-SLAM3 vs RTAB-Map — 이 노트북에서 30 FPS 실시간 달성 통합 분석

작성: 2026-09-12 · Main Agent 통합본
입력: `orbslam_ws/perf_patches/AGENT1_ORBSLAM3_REALTIME_REPORT.md` (A1-A20),
      `rtabmap_ws/AGENT2_RTABMAP_REALTIME_REPORT.md` (B1-B20),
      `HW_MEASURED_BASELINE.md` (Main Agent 실측)

## 표기 규약

- **[M]** 이 세션에서 실제로 실행해 측정한 값
- **[V]** 저장소 소스에서 파일:줄 단위로 확인한 사실
- **[E]** 추정값. ORB-SLAM3도 RTAB-Map도 **이 머신에서 빌드된 적이 없어** 실행 측정이 불가능하다. 오차 ±40%로 본다.
- **[P]** 이 프로젝트의 과거 기록(`slam_comparison_report/`)

---

# 0. 전제 정정 — 요청서의 가정 4개가 틀렸다

| # | 요청서의 가정 | 실제 | 근거 |
|---|---|---|---|
| 1 | RTX 4090 | **RTX 3080 Ti Laptop** (58 SM, 137 W, 4090의 35~45%) | [M] `nvidia-smi` |
| 2 | 예산 33.33 ms, 목표 25~30 ms | **실질 예산 ≈20 ms**. 25 ms면 P99 34.60 ms로 실패 | [M] 스윕 |
| 3 | GPU matching으로 가속 (A8/B7) | **순손실.** 오프로드 바닥값이 실제 작업량보다 큼 | [M] |
| 4 | "RTAB-Map을 30 FPS로" | RTAB-Map은 **29/30 프레임을 매핑에서 버린다**. 문제가 둘로 쪼개짐 | [V] `Parameters.h:182` |

---

# 1. 실시간 성립 조건 (측정)

30 FPS 주기로 고정 연산을 돌리며 지연 분포 측정. 백그라운드 3스레드 부하.

| 연산 예산 | avg | P95 | P99 | max | 마감 초과 | 판정 |
|---|---|---|---|---|---|---|
| 16 ms | 18.09 | 21.08 | 22.70 | 25.30 | 0.00% | OK |
| 18 ms | 20.52 | 24.05 | 27.14 | 32.19 | 0.00% | OK |
| **20 ms** | **22.33** | 25.64 | **28.69** | 31.90 | **0.00%** | **한계선** |
| 22 ms | 24.95 | 29.24 | 32.56 | 34.99 | 1.00% | MARGINAL |
| 25 ms | 27.16 | 31.58 | 34.60 | 38.85 | 2.20% | FAIL |

**설계 규칙**
1. Tracking 연산 예산 상한 **20 ms**. avg→P99 배율 **1.3~1.5배**를 항상 계상.
2. WSL2 타이머 깨어남 지연 **P50 776 us / max 1624 us** — 연산 전에 이미 소모.
3. 총 스레드 수를 물리 코어 아래로. 16스레드 오버서브스크립션 시 **60% 마감 초과**.
4. `SCHED_FIFO`/`SCHED_RR`/`nice(-10)` 전부 **EPERM**. P/E 코어 구분 불가(`core_id` 0..9 평탄화).
   → **P99를 지키는 유일한 수단은 스레드 총량 관리**다.

---

# 2. GPU 판정 — 옮길 것은 하나뿐

| 연산 | CPU 실제 비용 | GPU 비용 | 판정 |
|---|---|---|---|
| **ORB/feature extraction** | 5~9 ms [E] | 0.64~0.89 ms [M] + octree 1.40 ms CPU [M] | ✅ **유일한 대상, 2~4배** |
| descriptor matching | 0.03~0.15 ms [M] | 오프로드 바닥값 0.108 ms [M] | ❌ **순손실** |
| PoseOptimization (g2o) | 3~7 ms [E] | 희소 LM, 전송 지배 | ❌ 가치 없음 |
| ICP (RTAB-Map) | 12~45 ms [E] | RTAB-Map에 **CUDA ICP 없음** [V] | ❌ 구현 부재 |

**GPU matching이 손해인 이유(양쪽 공통, 구조적)**: 두 시스템 모두 grid/window 제약 탐색을 쓴다
(`Frame::GetFeaturesInArea`, `RegistrationVis.cpp:1011-1016`의 radiusSearch 분기).
프레임당 실제 비교는 **10^4~10^5회**이지 10^6 이상이 아니다. 손익분기는 약 **1M 비교**.

**GPU ORB의 진짜 상한 — `DistributeOctTree`** [M, Agent 1 측정]:
2k corner 0.56 ms / 8k 1.40 ms / **64k 10.05 ms**. 비용이 *입력 corner 수*에 선형이고
출력 개수와 무관하다. 즉 **GPU FAST가 과검출하면 이득이 통째로 사라진다.**
현실적 예산: GPU 0.89 + CPU octree 1.40 = **2.4 ms vs 5~9 ms → 2~4배** (10배가 아님).
GPU 포팅 시 **per-cell FAST 임계값을 반드시 재현**해야 한다.

**BRIEF 호환성** [V, Agent 1]: `ORBextractor.cc:149-408`의 `bit_pattern_31_`이 OpenCV
`features2d/src/orb.cpp` 및 `cudafeatures2d`와 **바이트 단위로 동일**. WTA_K=2(`:448`),
7x7 σ=2 `BORDER_REFLECT_101` 사전 블러, IC angle 모두 일치.
→ **`ORBvoc.txt`가 GPU 포팅 후에도 유효하다.** 이것이 GPU ORB를 실현 가능하게 만드는 핵심 사실.
단 `cv::cuda::ORB`는 여전히 drop-in이 아니다 — 전역 response 정렬(`cull_gpu`)을 쓰고
ORB-SLAM3의 octree 균등화를 하지 않는다.

---

# 3. P99를 깨는 요인 — 시스템별

## ORB-SLAM3

| 요인 | 크기 | 위치 | 수정 가능? |
|---|---|---|---|
| 🔴 dense map 인라인 발행 | **150~600 ms** [E] | `rgbd_node.cpp:443-445` `imageCallback` [V] | ✅ patch 05 |
| 🔴 Global BA 맵 뮤텍스 | **100~800 ms** [E] | `LoopClosing.cc:1858/1887` ↔ `Tracking.cc:1886` [V] | ❌ **구조적** |
| LocalBA 쓰기 반영 | 2~10 ms/KF [E] | 동일 뮤텍스 | 부분적 |
| E-core 이주 | 수 ms | WSL2 | ❌ 불가 |

`Tracking::Track()`은 **본문 전체**에서 `pCurrentMap->mMutexMapUpdate`를 잡는다(`Tracking.cc:1886` [V]).
`LoopClosing`이 같은 뮤텍스를 스패닝 트리 보정 내내 잡는다. **설계상 배타적**이다.

## RTAB-Map

| 요인 | 크기 | 위치 | 수정 가능? |
|---|---|---|---|
| 🔴 TF 조회 블로킹 | **200 ms** [E] | `rtabmap.launch.py:439` `wait_for_transform=0.2` [V] | ✅ 0.05로 |
| Bayes filter O(N^2) | N=10000 시 400 MB/100M MAC | `BayesFilter.cpp:196,313` [V] | ✅ `TimeThr` |
| loop closure 지연 | **odometry에 전파 안 됨** | drop-oldest 큐 [V] | 해당 없음 |
| 🟡 조용한 프레임 손실 | — | `OdometryThread.cpp:182-188` [V] | 설계 선택 |

**결정적 구조 차이**: RTAB-Map의 두 큐는 **drop-oldest, 절대 블로킹하지 않는다**
(`ImageBufferSize=1`). loop closure가 아무리 밀려도 odometry를 멈추지 못한다.
**대신 프레임을 조용히 버린다** — ROS2 경로의 경고문은 주석 처리되어 있다(`OdometryROS.cpp:477` [V]).
즉 **"늦지 않음"을 "가끔 건너뜀"으로 산다.** 이것이 두 시스템의 가장 중요한 차이다.

---

# 4. 지연 예산 (640x480 RGB-D, headless, 이 노트북)

## ORB-SLAM3 (nFeatures=1250)

| | Stock | +기존 패치 | +Phase1-4 | +GPU ORB |
|---|---|---|---|---|
| Mean | 10~21 ms | 7~16 | 5~13 | 4~11 |
| P95 | ~22 ms | ~17 | ~13 | ~11 |
| P99 | **150~600 ms** | 변화 없음 | **~25 ms** (patch 05 후) | ~22 |
| Worst | **100~800 ms** | 변화 없음 | **변화 없음 (GBA)** | 변화 없음 |

## RTAB-Map (Vis-only, CPU)

| | 값 |
|---|---|
| Best | ~4.8 ms |
| Typical | ~10.5 ms |
| Worst | ~23 ms **+ 200 ms TF 꼬리** |
| ICP를 odometry 경로에 두면 | 12~45 ms → ❌ 즉시 실패 |

최대 항목은 GFTT detect (2.5~10 ms [E]). matching은 0.06~0.15 ms로 무시 가능 [M].

---

# 5. 정면 비교표

| 항목 | ORB-SLAM3 | RTAB-Map |
|---|---|---|
| **CPU 30FPS 가능성** | ✅ 여유 있음. 640x480/1250에서 10~21 ms [E], [P] 12.4 ms 실측(다른 PC) | ✅ 더 여유. F2M visual 6~12 ms [E]. 단 현 빌드는 local BA 불가 |
| **RTX3080Ti 활용 가능성** | 좁음. ORB extraction만, 2~4배. BRIEF 호환 확인됨 | 더 좁음. `.cu` 파일 0개, OpenCV CUDA 위임. OpenCV 재빌드 + Version.h 수정 선행 |
| **Tracking/Odometry latency** | 10~21 ms. ORB 5~9 + PoseOpt x2 3~7 ms | 4.8~23 ms. GFTT detect가 최대 항목 |
| **33.33ms 달성 난이도** | 평균 쉬움 / P99 어려움 / **worst 불가** | 평균 쉬움 / P99 중간 / worst는 "버려서" 회피 |
| **GPU ORB** | ✅ 가치 있음. `ORBvoc.txt` 유효 유지 | 🟡 `ORB/Gpu`만. `FAST/Gpu=true`는 `UFATAL` 지뢰 |
| **GPU Matching** | ❌ 순손실 [M] | ❌ 순손실 [M] |
| **Mapping 비용** | 버스티. LocalBA가 맵 뮤텍스로 2~10 ms 스톨 | **분리됨**. 별도 스레드 + 1 Hz + drop-oldest |
| **Loop Closure 비용** | 🔴 **100~800 ms 맵 동결** | ✅ **odometry에 전파 불가** (구조적) |
| **Long-term Mapping** | Atlas, 메모리 전용 | ✅ **압도적**. STM/WM/LTM + SQLite + 멀티세션 |
| **RGB-D** | 1급 | 1급 + ICP/occupancy grid |
| **Stereo** | 1급. L/R 이미 스레드 분리(`Frame.cc:122` [V]) | 1급 |
| **정확도** | ✅ 우수. [P] 평균 변위 0.067 m | 0.185 m. 단 **보정은 우수** (loop gap 0.168→0.008 m, χ² 200→0.47) |
| **구현 난이도** | CPU 중간 / GPU 높음 | 낮음. 파라미터 하나로 동작 변경 |
| **유지보수 난이도** | 높음. 29k줄 연구 모놀리스 | 중간. 단 **옵션 의존성이 조용히 실패** |
| **ROS2 통합** | 🔴 얇고 **현재 병목**. 1327줄 커스텀 노드 | ✅ 네이티브, 전 노드 composable (단 intra-process 미사용) |

---

# 6. 반드시 답할 질문 10개

**1. 어느 쪽이 30 FPS / 33.33 ms 달성이 쉬운가?**
→ **RTAB-Map.** 평균 ~10 ms로 3배 헤드룸이고, 매핑/loop closure가 **설계상** odometry를 막을 수 없다.
ORB-SLAM3는 평균은 이미 충족하지만 P99가 ROS2 래퍼 수정에 걸려 있고 worst case는 고칠 수 없다.

**2. Tracking/Odometry만 보면?**
→ **RTAB-Map이 약간 빠르다** (4.8~23 vs 10~21 ms). 단 현 빌드에서 local BA가 꺼져 있어
불공정한 비교다. 정확도를 맞추면 격차는 거의 사라진다.

**3. Mapping까지 포함하면?**
→ **RTAB-Map이 결정적으로 유리.** `Rtabmap/DetectionRate=1 Hz` + drop-oldest 큐로
매핑 비용이 tracking 예산에서 **구조적으로 분리**되어 있다. ORB-SLAM3는 같은 뮤텍스를 공유한다.

**4. RGB-D 사용 시?**
→ **RTAB-Map.** depth registration, occupancy grid, ICP 경로가 내장. 단 **ICP를 odometry 경로에
넣으면 안 된다** (12~45 ms).

**5. Stereo 사용 시?**
→ **ORB-SLAM3.** L/R ORB 추출이 이미 `std::thread`로 분리되어 있고(`Frame.cc:122-125` [V]),
stereo가 1급 시민이다. 단 실제 비용은 descriptor matching이 아니라
`ComputeStereoMatches`의 11-tap `cv::norm` SAD다.

**6. GPU 적극 활용 시 어느 architecture가 확장성이 좋은가?**
→ **ORB-SLAM3.** feature extraction이 `ORBextractor` 한 클래스에 응집되어 있고
BRIEF 패턴이 OpenCV와 동일해 vocabulary가 유지된다. RTAB-Map은 OpenCV CUDA 모듈에
전적으로 위임하고 자체 `.cu`가 없어 GPU 확장 여지가 적다.

**7. CPU-only라면?**
→ **양쪽 다 가능하지만 RTAB-Map이 현실적.** 이 머신엔 CUDA OpenCV가 없어
**오늘 당장 돌릴 수 있는 구성은 CPU-only뿐**이다. RTAB-Map은 그 상태로 헤드룸이 있다.

**8. 가장 먼저 GPU로 옮겨야 하는 연산은?**
→ **ORB/feature extraction 하나뿐.** 그 외에는 옮길 가치가 있는 것이 없다.
그리고 **`DistributeOctTree`가 CPU에 남는 한 이득은 2~4배가 상한**이다.

**9. 최종 추천은?**
→ 아래 7장. 1순위 **Option B(RTAB-Map)**, 2순위 **Option C(하이브리드)**, 3순위 **Option A**.

**10. 이유 (latency / accuracy / development cost)**
- **Latency**: RTAB-Map이 평균·P99 모두 우세. 결정적 이유는 속도가 아니라 **분리 구조**다.
- **Accuracy**: ORB-SLAM3가 tracking 2.8배 정확(0.067 vs 0.185 m)하지만,
  RTAB-Map이 loop closure로 **사후 보정**한다(0.168→0.008 m). 최종 지도 품질은 RTAB-Map이 낫다.
- **Development cost**: RTAB-Map 압승. ORB-SLAM3는 ROS2 래퍼를 직접 고쳐야 하고
  GPU 포팅 시 golden-keypoint 회귀 테스트가 필수다.

---

# 7. 최종 권장 Architecture 3안

## Option A — Optimized ORB-SLAM3 (3순위)

```
  Camera (RGB-D 30Hz)
        |
        v
  [ Tracking thread ]  <-- 목표 20 ms
    Frame ctor -> ExtractORB (GPU 선택) -> DistributeOctTree (CPU 1.4ms)
    -> TrackWithMotionModel -> PoseOptimization #1
    -> TrackLocalMap        -> PoseOptimization #2
    -> NeedNewKeyFrame
        |  ... mMutexMapUpdate (본문 전체 보유) ...
        v
  [ LocalMapping ]  LocalBA -- 같은 뮤텍스로 2~10ms 스톨
        |
  [ LoopClosing ]   GBA -- 같은 뮤텍스로 100~800ms 동결  <-- 🔴 수정 불가
        |
  [ ROS2 wrapper ]  dense map 발행이 콜백 인라인  <-- 🔴 patch 05로 수정
```
선택 조건: pose 정확도가 최우선이고 GBA 동결을 감수하거나, **prior map 기반
localization-only**(`Tracking.cc:2377`)로 운용해 GBA를 아예 돌리지 않을 때.

## Option B — Optimized RTAB-Map (1순위) ★

```
  Camera (RGB-D 30Hz)
        |
        v
  [ OdometryThread ]  drop-oldest, ImageBufferSize=1   <-- 33.33ms 지켜야 하는 유일한 곳
    Feature2D::generateKeypoints (ORB)
    -> RegistrationVis (radiusSearch + BFMatcher, 0.06~0.15ms)
    -> PnP / motion estimation
        |
        | drop-oldest 큐 (블로킹 없음)
        v
  [ RtabmapThread ]  DetectionRate = 1 Hz  <-- 1000 ms 예산
    Memory::update -> VWDictionary -> BayesFilter -> Optimizer(g2o)
        |
  [ SQLite LTM ]  STM/WM/LTM 전이, TimeThr로 제한
```
선택 조건: **대부분의 경우.** ROS2 네이티브, 매핑 분리, long-term mapping, 개발비 최저.
단 **프레임 손실을 계측하고 허용**해야 한다.

## Option C — ORB-SLAM3 + RTAB-Map Hybrid (2순위)

**이미 upstream에 존재한다.** `Odom/Strategy=5` → `OdometryORBSLAM3` (`Odometry.cpp:87-93` [V]).

```
  Camera
    |
    v
  [ OdometryORBSLAM3 ]  ORB-SLAM3 System::TrackRGBD / TrackStereo
    - ORB-SLAM3 loop closer는 꺼짐:  ofs << "loopClosing: " << 0   (:324-326) [V]
    - 따라서 GBA 맵 동결이 구조적으로 사라진다  <-- Option A의 유일한 치명상 제거
    |
    | SE3 증분 변환 + 공분산
    v
  [ RTAB-Map ]  loop closure / 전역 그래프 / LTM / 멀티세션
```

인터페이스는 `System` 메서드 **7개**뿐: `TrackRGBD`(`:475`), `TrackStereo`(`:460`),
`GetTrackedMapPoints`, `GetTrackedKeyPointsUn`, `isLost`, `Shutdown`, `Converter::toCvMat`.

**전제 조건 3개 (반드시 먼저)**
1. `Version.h` 수정 — 없으면 `RTABMAP_ORB_SLAM`이 꺼져 있어 `Odom/Strategy=5`가 **컴파일 타임에 죽는다**(`OdometryORBSLAM3.cpp:592-595` [V]).
2. `loopClosing:` yaml 키는 **stock ORB-SLAM3에 없다.** matlabbe 포크 필요.
3. 아래 결함 3개 수정.

**알려진 결함**
- 🔴 공분산이 `(baseline/8)^2` 하드코딩(`:520-533` [V]) — tracking 품질과 무관하게 모든 그래프 엣지가 동일 가중치. `mapPoints.size()`/inlier로 대체, **약 15줄, 최고 가치**.
- 증분 변환 반환(`t = previousPoseInv*p`, `:502`) — local BA 보정이 odometry 점프로 새어 나옴.
- `data.setFeatures()` 미호출 → `Memory.cpp:4871`에서 **feature 재추출**. 1 Hz면 무시 가능, 30 Hz면 초당 30회 중복.
- `ORB/ScaleFactor`/`NLevels` 기본값이 2/3 (ORB-SLAM3는 1.2/8) — 명시 설정 필요.
- `:458` 왼쪽 영상을 `rightMono`로 넣는 복붙 버그.

**중복 계산 제거법**: `OdometryORBSLAM3`에서 `GetTrackedKeyPointsUn()`/`GetTrackedMapPoints()`로
받은 keypoint를 `data.setFeatures()`로 넘기면 RTAB-Map 쪽 재추출이 사라진다.

---

# 8. 최종 구현 Roadmap

난이도: ★ 쉬움 / ★★ 보통 / ★★★ 어려움

## Phase 0 — 빌드 정합성 (양쪽 공통, 반드시 먼저)

| # | 파일 / 함수 | 변경 | 이득 | 난이도 | 정확도 영향 | 리스크 |
|---|---|---|---|---|---|---|
| 0.1 | `rtabmap/corelib/include/rtabmap/core/Version.h` | **삭제** (또는 `corelib/src/CMakeLists.txt:860` include 순서 교체) | ms 아님. g2o/GTSAM/libpointmatcher/nonfree/**ORB_SLAM** 전부 해금 | ★ | 개선 | 없음. 생성본이 대체 |
| 0.2 | `.gitignore` | 위 경로 추가 | 재발 방지 | ★ | — | 없음 |
| 0.3 | `orbslam_ws/perf_patches/02-*.patch` | **폐기** (`corrupt patch at line 77` [M] 확인) | — | ★ | — | 없음. `orbslam3-perf-all`이 대체 |

## Phase 1 — 프로파일링 (측정 없이는 아무것도 하지 말 것)

| # | 대상 | 변경 | 이득 | 난이도 |
|---|---|---|---|---|
| 1.1 | ORB-SLAM3 `REGISTER_TIMES` | 컴파일 플래그 활성화 (이미 존재, `Frame.cc:119` 등 [V]) | 기준선 확보 | ★ |
| 1.2 | ORB-SLAM3 `include/StageTimer.h` | 기존 패치가 `ORBSLAM3_PROFILE`을 **선언만 하고 구현 안 함** — 구현하거나 제거 | — | ★★ |
| 1.3 | RTAB-Map `Timing/*` 38개 키 | `Rtabmap/PublishStats` + `OdomInfo` 덤프. **신규 계측 거의 불필요** | 기준선 확보 | ★ |

> `gettimeofday` 15.71 ns/call [M] — `UTimer` 계측은 프레임당 0.07 ms. 걱정할 필요 없다.

## Phase 2 — 스레드 총량 관리 (P99에 가장 큰 영향, 양쪽 공통)

| # | 파일 / 함수 | 변경 | 이득 | 난이도 | 리스크 |
|---|---|---|---|---|---|
| 2.1 | ORB-SLAM3 `System::ConfigureThreadPools` (신규, `03-thread-governance.patch`) | `omp_set_num_threads(4)` + `cv::setNumThreads(4)` + 워밍업 region | **11.4 ms/frame 함정 회피 + 첫 프레임 10.5 ms 스파이크 제거** [M] | ★ | 없음 |
| 2.2 | RTAB-Map 진입점 | 동일. RTAB-Map은 **어디서도 스레드 수를 설정하지 않는다** [V] | 순수 Vis 경로 0 ms, ICP/lidar 경로 3.79 ms x region [M] | ★ | 없음 |
| 2.3 | 양쪽 | 총 스레드 수를 물리 코어 아래로 유지 | 오버서브스크립션 시 60% 마감 초과 회피 [M] | ★★ | 없음 |

## Phase 3 — P99 킬러 제거 (여기가 진짜 병목)

| # | 파일 / 함수 | 변경 | 이득 | 난이도 | 리스크 |
|---|---|---|---|---|---|
| 3.1 | **`orbslam3_ros2/src/rgbd_node.cpp:443` `RgbdNode::imageCallback`** | `publishDenseMapIfNeeded` / `publishMapPointsIfNeeded` / `accumulateDenseMapIfNeeded`를 **tracking 콜백 밖 별도 스레드로** (`05-ros2-latency.patch`) | 🔴 **P99 150~600 ms → ~25 ms** | ★★ | 발행 지연 증가 |
| 3.2 | `rtabmap.launch.py:439` `wait_for_transform` | 0.2 → 0.05 | 🔴 **200 ms 꼬리 제거** | ★ | TF 미도착 시 프레임 스킵 |
| 3.3 | `rtabmap.launch.py:417` `rtabmap_viz` | `true` → `false` | 시각화 부하 제거 | ★ | 디버깅 불편 |
| 3.4 | ORB-SLAM3 `System` 생성자 `bUseViewer` | `false` (headless) | Viewer/FrameDrawer 뮤텍스 경합 제거 | ★ | 동일 |

## Phase 4 — CPU 저비용 최적화 (정확도 영향 0)

| # | 파일 / 함수 | 변경 | 이득 | 난이도 | 정확도 |
|---|---|---|---|---|---|
| 4.1 | `orbslam3-perf-all.patch` 적용 (`git apply --check` **통과 확인** [M]) | `Frame::GetFeaturesInArea` 할당 제거, POPCNT 등 | **1.62~2.06배, 0.06~0.22 ms** [M] | ★ | **비트 동일** |
| 4.2 | `Frame::isInFrustum` (`Frame.cc:512`) | 잠금 접근자 3회 → 1회 통합 (프레임당 9k~45k 잠금쌍) | 0.15~0.4 ms [E] | ★★ | **비트 동일** |
| 4.3 | `Tracking::UpdateLocalKeyFrames` | 참조 전달 (Agent 1 패치) | **2.64배, 0.24~0.59 ms** [M] | ★ | 비트 동일 |
| 4.4 | `ORBextractor` octree 노드 `reserve` (`04-octree-node-reserve.patch`) | 할당 제거 | 소폭 | ★ | 비트 동일 |
| 4.5 | `RegistrationVis.cpp:1123,1275` | `cv::BFMatcher`를 **루프 밖으로 호이스팅** (현재 프레임당 ~1500회 생성) | 0.2~0.8 ms [E] | ★ | **동작 보존** |
| 4.6 | ❌ `UpdateLocalPoints` 참조 전달 | **하지 말 것** — 0.02 ms [M], 교착 위험만 추가 | — | — | — |
| 4.7 | 컴파일러 | `-O3 -march=native -mtune=native` + IPO. **AVX-512 없음**, `-ffast-math` 금지(g2o/Eigen NaN 검사) | 5~15% [E] | ★ | `-ffast-math`만 위험 |

## Phase 5 — 파라미터 (정확도 트레이드오프 시작)

| # | 대상 | 변경 | 이득 | 정확도 영향 |
|---|---|---|---|---|
| 5.1 | `Parameters.h:180` `Rtabmap/TimeThr` | **0 → 700 ms**. O(N^2) Bayes 증가의 **유일한 내장 제어기이며 기본이 꺼져 있다** | 맵 성장 시 지연 폭주 방지 | WM 축소 → loop 재인식률 소폭 하락 |
| 5.2 | `Vis/FeatureType` + `Kp/DetectorStrategy` | 8(GFTT/ORB) → 2(ORB), **반드시 함께**. 다르면 `Memory.cpp:833-849`가 `Mem/UseOdomFeatures`를 조용히 끈다 [V] | 1.5~7 ms/frame [E] | 중간 |
| 5.3 | ORB-SLAM3 `nFeatures` | 2000 → 1250 (Performance) / 1500 (Balanced) / 2000 (Accuracy) | 선형 | 저텍스처에서 열화 |
| 5.4 | `Odom/ImageDecimation`, `Mem/ImagePre/PostDecimation` | RTAB-Map은 **내장 데시메이션 4종 보유** — 고해상도 녹화 + 저해상도 SLAM을 외부 리사이즈 없이 | 해상도 제곱에 비례 | 중간 |

## Phase 6 — PoseOptimization (정확도 영향 있음, 게이트 필수)

| # | 파일 / 함수 | 변경 | 이득 | 리스크 |
|---|---|---|---|---|
| 6.1 | `Optimizer.cc:1003` `Optimizer::PoseOptimization` | `its[4]={10,10,10,10}` → `{10,5,5,5}` + `nBad` 안정 시 조기 종료 | **1.8~4.5 ms/frame** [E] (프레임당 **2~3회 호출** [V]: `Tracking.cc:2827` 또는 `2991`, 그리고 `3053`(+`3059`)) | 🔴 **yaml 키로 게이트하고 ATE 회귀 측정 필수** |

> 프레임당 총 **80~120 LM iteration**을 dense solver로 돌고 있다. CPU 측 최대 단일 표적이다.

## Phase 7 — GPU feature extraction (유일한 GPU 작업)

| # | 대상 | 변경 | 이득 | 난이도 | 리스크 |
|---|---|---|---|---|---|
| 7.1 | OpenCV 재빌드 | `-DWITH_CUDA=ON -DCUDA_ARCH_BIN=8.6` + contrib | 전제 조건 | ★★ | 빌드 시간 |
| 7.2 | ORB-SLAM3 `ORBextractorCUDA.{h,cu}` (신규) | pyramid/blur/FAST/BRIEF를 sm_86으로. **BRIEF 패턴은 OpenCV와 동일하므로 `ORBvoc.txt` 유지** [V] | 5~9 ms → **2.4 ms** (2~4배) | ★★★ | **per-cell FAST 임계 재현 필수**. 과검출 시 octree가 10 ms로 폭발 [M] |
| 7.3 | RTAB-Map `ORB/Gpu` | `true`. **`FAST/Gpu=true`는 금지** — `UFATAL("not implemented")` [V] | 유사 | ★ | OpenCV CUDA 필요 |
| 7.4 | GPU 동기화 | 프레임당 **5개 이하**. 1회 26 us [M] | — | ★★ | — |
| 7.5 | pinned memory | 쓰되 **1.2~1.5배만 기대** [M]. WSL2는 3~4배가 안 나온다 | 0.1~0.2 ms | ★ | — |

## Phase 8 — 하이브리드 (Option C 채택 시)

| # | 파일 / 함수 | 변경 | 이득 | 난이도 |
|---|---|---|---|---|
| 8.1 | Phase 0.1 완료 | `RTABMAP_ORB_SLAM` 해금 | `Odom/Strategy=5` 사용 가능 | ★ |
| 8.2 | ORB-SLAM3를 matlabbe 포크로 교체 | `loopClosing:` 키 지원 | GBA 동결 제거 | ★★ |
| 8.3 | `OdometryORBSLAM3.cpp:520-533` | 하드코딩 공분산 → `mapPoints.size()`/inlier 기반. **약 15줄** | 🔴 **그래프 최적화 품질 직결. 최고 가치** | ★★ |
| 8.4 | `OdometryORBSLAM3.cpp` | `data.setFeatures()` 호출 추가 | feature 재추출 제거 | ★★ |
| 8.5 | `OdometryORBSLAM3.cpp:458` | `rightMono` 복붙 버그 수정 | stereo 정합성 | ★ |
| 8.6 | `ORB/ScaleFactor`, `ORB/NLevels` | 2/3 → 1.2/8 명시 | ORB-SLAM3 기본값 정합 | ★ |

## Phase 9 — 검증

| # | 항목 |
|---|---|
| 9.1 | **Average가 아니라 P95/P99/worst를 본다.** 20 ms 예산에서 P99 28.69 ms가 기준선 [M] |
| 9.2 | **드롭된 프레임 수를 계측한다.** RTAB-Map은 조용히 버린다 — `OdometryROS.cpp:477`의 주석 처리된 경고를 되살릴 것 |
| 9.3 | GPU 포팅 시 **golden-keypoint 회귀 스위트** 필수 |
| 9.4 | Phase 6 전후 **ATE/RPE 비교** |

---

# 9. 최종 질문에 대한 답

> "RTX 4090이 장착된 시스템에서 ... 어떤 파일의 어떤 함수를 어떤 순서로 수정해야 하는가?"

장비는 4090이 아니라 **RTX 3080 Ti Laptop + i9-12900HK + WSL2**다. 그 위에서의 답:

1. `rtabmap/corelib/include/rtabmap/core/Version.h` — **삭제**
2. `orbslam3_ros2/src/rgbd_node.cpp:443` `RgbdNode::imageCallback` — dense map 발행을 **콜백 밖으로**
3. `rtabmap.launch.py:439` `wait_for_transform` — **0.2 → 0.05**
4. `System::ConfigureThreadPools` (신규) + RTAB-Map 진입점 — **OMP/OpenCV 스레드 4개로 고정 + 워밍업**
5. `orbslam3-perf-all.patch` 적용 (02번 패치는 폐기)
6. `Frame::isInFrustum`(`Frame.cc:512`), `Tracking::UpdateLocalKeyFrames`, `RegistrationVis.cpp:1123,1275`
7. `Parameters.h:180` `Rtabmap/TimeThr` — **0 → 700**
8. `Optimizer.cc:1003` `Optimizer::PoseOptimization` — `its[4]={10,5,5,5}`, **yaml 게이트 + ATE 회귀**
9. OpenCV CUDA 재빌드 → `ORBextractorCUDA.cu`
10. (하이브리드 시) `OdometryORBSLAM3.cpp:520-533` 공분산

**1~4번만으로 P99가 150~600 ms에서 ~25 ms로 떨어진다.** GPU는 9번에서야 등장하며,
그때도 2~4배이지 10배가 아니다.

## 실시간 요건 최종 평가

| 관점 | Option A (ORB-SLAM3) | Option B (RTAB-Map) | Option C (하이브리드) |
|---|---|---|---|
| Average < 33.33 ms | ✅ 이미 충족 | ✅ 여유 3배 | ✅ |
| P95 < 33.33 ms | ✅ Phase 3 후 | ✅ | ✅ |
| P99 < 33.33 ms | 🟡 ~25 ms, 여유 없음 | 🟡 가능하나 보장 불가 | 🟡 |
| **Worst case < 33.33 ms** | ❌ **불가 (GBA 100~800 ms)** | 🟡 **버려서 회피** | 🟡 GBA 제거됨 |

**"모든 프레임이 33.33 ms 이하"를 문자 그대로 요구한다면 세 안 모두 이 WSL2 노트북에서는 보장할 수 없다.**
`SCHED_FIFO`가 EPERM이고 P/E 코어를 구분할 수 없기 때문이다. 진짜 하드 실시간이 필요하면
**베어메탈 Linux**로 가야 한다. 그 전까지의 정직한 목표는
**"평균 10 ms, P99 25~29 ms, 드롭률을 계측하고 1% 이하로 유지"** 다.
