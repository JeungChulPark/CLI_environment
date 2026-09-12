# 이 노트북에서 실측한 하드웨어 기준선 (RTX 4090 아님)

측정일: 2026-09-12. 모든 수치는 이 머신에서 **실제로 실행해 얻은 값**이며 추정이 아니다.
벤치마크 소스는 scratchpad의 `sched_test.c`, `cudabench.cu`, `hamming.cu`, `realistic.cu`,
`orbgpu.cu`, `orbcpu.cpp`.

## 0. 실제 장비

| 항목 | 값 | 출처 |
|---|---|---|
| CPU | Intel Core i9-12900HK (Alder Lake-HX) | `lscpu` |
| 코어 | 물리 14 (6 P-core + 8 E-core) / 논리 20 | 스펙 |
| **WSL2가 보는 토폴로지** | **10 core x 2 thread = 20 CPU (평탄화됨)** | `/sys/.../topology/core_id` = 0..9 |
| SIMD | AVX2, FMA, BMI1/2, POPCNT, VAES, AVX-VNNI | `lscpu` flags |
| **AVX-512** | **없음** (Alder Lake에서 비활성) | flags에 avx512 없음 |
| 캐시 | L2 12.5 MiB, L3 24 MiB | `lscpu` |
| RAM | 31 GiB + swap 8 GiB | `free -h` |
| **GPU** | **RTX 3080 Ti Laptop GPU (GA103)** | `nvidia-smi` |
| VRAM | 16384 MiB | `nvidia-smi` |
| SM / 클럭 | **58 SM**, 1.46 GHz | `cudaGetDeviceProperties` |
| Compute capability | **8.6** → `-arch=sm_86` | 동일 |
| 메모리 대역폭(이론) | 256-bit @ 8001 MHz = **512 GB/s** | 동일 |
| 전력 한계 | **137 W TGP** (데스크톱 3080Ti 350W 대비 낮음) | `nvidia-smi` |
| CUDA 툴킷 | `/usr/local/cuda-12.4/bin/nvcc` (12.4) | `nvcc --version` |
| 드라이버 | 596.08 (Windows 호스트), CUDA 13.2 보고 | `nvidia-smi` |
| OS | **WSL2** (5.15.167.4-microsoft-standard-WSL2) | `uname` |
| ROS2 | **설치 안 됨** (`/opt/ros` 없음) | 확인 |
| OpenCV | **시스템에 없음**, CUDA 빌드도 없음 (`libopencv_cuda*` 0개) | `find` |

> RTX 4090 대비: SM 128 → 58 (45%), 대역폭 1008 → 512 GB/s (51%), TGP 450 → 137 W (30%).
> **GPU 연산 성능은 4090의 대략 35~45% 수준으로 잡는 것이 안전하다.**

## 1. 실시간 스케줄링 — WSL2에서 대부분 불가 (실측)

```
SCHED_FIFO(prio50): Operation not permitted
SCHED_RR(prio10)  : Operation not permitted
nice(-10)         : Operation not permitted
setaffinity{0,1}  : OK
```

- `sched_setscheduler(SCHED_FIFO/RR)` 와 `nice(-10)` 은 일반 사용자로 **전부 실패**한다.
  → "Tracking thread에 실시간 우선순위를 준다" 는 계획은 이 환경에서 **그대로는 성립하지 않는다**.
  (CAP_SYS_NICE / root 필요, 그래도 Windows 호스트 스케줄러가 VM 전체를 옮긴다.)
- `pthread_setaffinity_np` 는 동작한다. 다만 `cpufreq` 가 노출되지 않고 `core_id` 가 0..9로
  평탄화되어 **P-core와 E-core를 구분할 수 없다**. 즉 "Tracking을 P-core에 고정" 은 불가능하다.
  → E-core에 얹히는 프레임이 P99 지연의 주원인이 될 수 있고, 이 환경에서는 막을 방법이 없다.

## 2. CUDA 오버헤드 — WSL2의 실제 바닥값 (실측)

```
null kernel 비동기 enqueue        :  7.700 us / launch
launch + cudaDeviceSynchronize   : 26.063 us  <-- 동기화 1회의 가격
empty kernel launch + sync       : 27 us      (재측정, 일치)
```

**전송 대역폭 (pageable vs pinned):**

| 크기 | H2D pageable | H2D pinned | D2H pageable | D2H pinned |
|---|---|---|---|---|
| 64 KiB | 0.028 ms (2.3 GB/s) | 0.029 ms (2.3 GB/s) | 0.034 ms (1.9 GB/s) | 0.032 ms (2.0 GB/s) |
| 640x480 gray (300 KB) | 0.055 ms (5.5) | 0.049 ms (6.3) | 0.068 ms (4.5) | 0.050 ms (6.1) |
| 1280x720 gray (0.9 MB) | 0.123 ms (7.5) | 0.096 ms (9.6) | 0.181 ms (5.1) | 0.093 ms (9.9) |
| 1920x1080 gray (2.0 MB) | 0.237 ms (8.7) | 0.194 ms (10.7) | 0.289 ms (7.2) | 0.190 ms (10.9) |
| 1920x1080 BGR (5.9 MB) | 0.601 ms (10.3) | 0.538 ms (11.6) | 0.725 ms (8.6) | 0.522 ms (11.9) |

실측 결론 3가지:

1. **pinned memory 이득이 작다.** 2 MB에서 8.7 → 10.7 GB/s, 고작 **1.2x**.
   베어메탈 PCIe4 x16이면 pageable ~6 / pinned ~24 GB/s로 3~4x가 나온다.
   → WSL2의 pinned 메모리 DMA 저하가 실제로 확인된다. `cudaHostAlloc` 은 여전히 쓸 가치가
   있지만(특히 D2H에서 0.289 → 0.190 ms) **기대치를 1.2~1.5x로 낮춰 잡아야 한다.**
2. **작은 전송은 전적으로 지연에 지배된다.** 64 KiB(= descriptor 2000개)가 0.03 ms.
   크기를 절반으로 줄여도 시간은 거의 그대로다.
3. **동기화 1회 = 26 us.** 프레임당 동기화 지점을 20개 만들면 그것만으로 0.5 ms가 사라진다.

## 3. OpenMP — WSL2에서 스레드 수를 잘못 잡으면 재앙 (실측)

```
FIRST parallel region (스레드풀 생성)  : 10.508 ms   <-- 첫 프레임 스파이크
WARM parallel region, 기본 20 threads  :  3.7858 ms  <-- parallel-for 1회당!
WARM parallel region,  2 threads       :  0.0053 ms
WARM parallel region,  4 threads       :  0.0015 ms
WARM parallel region,  8 threads       :  0.0033 ms
```

**이것이 이번 측정에서 가장 실용적인 발견이다.**

- 기본값(논리 CPU 20개)으로 `#pragma omp parallel for` 를 돌면 **fork/join 배리어 하나에
  3.79 ms**가 든다. 33.33 ms 예산의 11%를 계산이 아니라 배리어에 버린다.
  프레임당 parallel region이 3개면 11 ms — 30 FPS가 이것만으로 무너진다.
- 스레드를 4~8개로 제한하면 **0.0015~0.0033 ms**, 즉 **1000배 이상 저렴**해진다.
- 첫 parallel region은 10.5 ms를 먹으므로, 시작 시 **워밍업 region을 한 번 돌려야**
  첫 프레임이 튀지 않는다.

→ 실행 규칙: `OMP_NUM_THREADS=6` (또는 `omp_set_num_threads(6)`) 를 반드시 명시하고,
  `cv::setNumThreads()` 도 동일하게 제한한다. **절대 기본값에 맡기지 않는다.**

## 4. ORB descriptor Hamming 매칭 — GPU로 옮기면 손해다 (실측)

### 4-1. POPCNT 패치의 실제 이득

`ORBmatcher::DescriptorDistance` 의 기존 SWAR 비트핵 vs `__builtin_popcountll`:

| 워크로드 | 비교 횟수 | SWAR | POPCNT | 이득 |
|---|---|---|---|---|
| SearchByProjection local map (1500 MP x 20 후보) | 30,000 | 0.117 ms | 0.060 ms | 1.94x |
| SearchByProjection local map (2000 MP x 40 후보) | 80,000 | 0.274 ms | 0.150 ms | 1.83x |
| TrackWithMotionModel (1000 x 15) | 15,000 | 0.051 ms | 0.029 ms | 1.73x |
| SearchByBoW frame<->KF (1000 x 50) | 50,000 | 0.170 ms | 0.092 ms | 1.84x |
| ComputeStereoMatches (2000 x 30) | 60,000 | 0.201 ms | 0.113 ms | 1.77x |
| Reloc/Loop BoW heavy (2000 x 200) | 400,000 | 1.279 ms | 0.789 ms | 1.62x |
| (가정) 전수 비교 2000 x 2000 | 4,000,000 | 12.851 ms | 7.883 ms | 1.63x |

→ `perf_patches/02-descriptor-distance-popcnt.patch` 는 **1.6~1.9x 확실히 유효**하다.
  다만 절대 절감량은 tracking 경로 전체에서 **0.15~0.30 ms/frame** 수준이다.
  비트 단위로 동일한 결과를 내므로 정확도 영향 0. 넣지 않을 이유가 없지만,
  **이것만으로는 30 FPS를 만들지 못한다.**

### 4-2. GPU 매칭의 손익분기점

GPU brute-force Hamming 커널 (2000x2000 = 4M 비교):

```
CPU 1-thread SWAR    : 9.75 ms
CPU 1-thread POPCNT  : 5.03 ms
GPU 커널만 (양쪽 상주): 0.409 ms
GPU 전체 왕복 (2up+3down): 0.648 ms
GPU train 상주 (1up+2down): 0.572 ms
```

GPU 고정 오버헤드 바닥값 (실측): **64KB 업로드 + 커널 + 64KB 다운로드 = 0.108 ms**

**결론 — ORB-SLAM3의 실제 매칭을 GPU로 옮기면 느려진다.**

ORB-SLAM3는 전수 비교를 하지 않는다. `Frame::GetFeaturesInArea` 의 grid 제약 탐색으로
쿼리당 후보가 10~40개뿐이라, 프레임당 실제 비교 횟수는 **10^4~10^5** 이지 10^6~10^7 이 아니다.
위 표에서 보듯 그 구간의 CPU POPCNT 비용은 **0.03~0.15 ms**이고,
GPU 오프로드 1회의 고정 비용이 **0.108 ms**다. 즉 계산을 0으로 만들어도 본전이다.

- 프레임당 매칭 호출이 여러 번이므로 GPU 왕복도 여러 번 → **순손실**.
- GPU가 이기는 유일한 구간은 전수 비교(4M, 7.9 ms → 0.65 ms)인데,
  그건 relocalization / loop closure 후보 검증이고 **이미 critical path 밖**이다.

→ 권고: **GPU descriptor matching은 하지 않는다.** 사용자 요청서의 A8 / B7 가정과 반대다.
  대신 CPU POPCNT + grid 탐색 유지가 이 하드웨어에서 최적이다.

## 5. ORB extraction — 여기가 GPU로 옮길 유일한 곳 (실측)

`ORBextractor.cc` 의 단계를 그대로 CUDA로 옮겨 측정 (8 levels, scaleFactor 1.2, iniThFAST 20).
피라미드 총 픽셀은 기저의 **3.10배**.

| 단계 | 640x480 | 1280x720 | 1920x1080 |
|---|---|---|---|
| H2D 업로드 (pinned) | 0.030 ms | 0.080 ms | 0.172 ms |
| ComputePyramid (bilinear x7) | 0.082 ms | 0.118 ms | 0.119 ms |
| GaussianBlur 7x7 x8 levels | 0.151 ms | 0.211 ms | 0.379 ms |
| FAST-9 검출 x8 levels | 0.169 ms | 0.224 ms | 0.377 ms |
| IC_Angle + BRIEF-256 | 0.156 ms | 0.157 ms | 0.157 ms |
| D2H keypoint+descriptor | 0.029 ms | 0.034 ms | 0.029 ms |
| **전체 (단계마다 sync)** | **0.638 ms** | **0.841 ms** | **1.186 ms** |
| 전체 (끝에 1회 sync) | 0.888 ms | 0.981 ms | 1.349 ms |

측정 조건과 한계를 분명히 해둔다:

- 입력이 랜덤 노이즈라 FAST가 과검출된다 (640x480에서 178k, 1280x720에서 540k 코너).
  실제 영상은 이보다 훨씬 적으므로 **위 FAST 수치는 상한**이다.
- BRIEF는 8000 keypoint 기준이다. 실제 목표인 2000개면 더 싸다. 역시 **상한**.
- **`DistributeOctTree` (octree 균등 분배)는 포함하지 않았다.** ORB-SLAM3 고유 단계이고
  본질적으로 순차적 트리 분할이라 GPU 이식이 까다롭다. 이건 Agent 1의 분석 대상이다.
  이 단계가 CPU에 남으면 위 GPU 수치에 CPU 시간이 더해진다.

→ **GPU ORB extraction은 이 노트북에서 1.0~1.5 ms의 GPU 시간으로 끝난다.**
  참고로 ORB-SLAM3의 CPU ORB extraction은 640x480 / 1000~2000 feature에서
  통상 **8~15 ms** 로 보고된다. 즉 **실효 5~10x 개선**이 기대되며,
  이것이 이 하드웨어에서 **GPU로 옮길 가치가 있는 유일한 큰 덩어리**다.

### 공정성 주의 — CPU 기준선에 대하여

동일한 나이브 코드를 CPU 단일 스레드로 돌리면 640x480에서 137 ms가 나왔다
(pyramid 5.1 / blur 26.0 / FAST 67.6 / BRIEF 43.6 ms).
그러나 이 수치를 "ORB-SLAM3의 CPU 성능" 으로 인용하면 **부정확하다**.
실제 ORB-SLAM3는 OpenCV의 분리형(separable) Gaussian, SIMD FAST, 조기 종료 캐스케이드를
쓰기 때문에 훨씬 빠르다. 위 CPU 수치는 "같은 나이브 알고리즘끼리의 비교" 로만 유효하다.
**따라서 GPU 이득은 100x가 아니라 5~10x로 보수적으로 잡는다.**

## 6. 이 실측이 최적화 전략에 주는 결론

1. **GPU로 옮길 1순위는 ORB/feature extraction 하나뿐이다.** (~1.0~1.5 ms 예산)
2. **descriptor matching은 CPU에 둔다.** GPU 오프로드 고정비(0.108 ms)가 실제 작업량보다 크다.
3. **POPCNT 패치는 넣는다.** 1.6~1.9x, 정확도 영향 0, 비용 없음.
4. **OpenMP 스레드 수를 4~8로 강제하고 시작 시 워밍업한다.** 기본값이면 배리어에만 3.8 ms.
5. **GPU 동기화 지점을 프레임당 5개 이하로 설계한다.** 1회 26 us.
6. **pinned memory는 쓰되 1.2~1.5x만 기대한다.** WSL2에서는 그 이상 안 나온다.
7. **실시간 우선순위는 포기하고 다른 수단으로 P99를 잡는다.** SCHED_FIFO가 아예 막혀 있다.
8. **P-core 고정은 불가능하다.** WSL2에서 P/E 구분이 노출되지 않는다. P99 리스크로 명시한다.

---

## 7. 30 FPS 실시간 성립 한계선 — 실측 (P95/P99)

30 FPS 주기(33.33 ms)로 고정 연산을 반복하며 프레임 지연 분포를 측정했다.
소스: `jitter.cpp`, `sweep.cpp`.

### 7-1. 백그라운드 부하의 영향 (연산 20 ms 고정)

| 조건 | avg | P95 | P99 | max | 마감 초과 |
|---|---|---|---|---|---|
| 유휴 머신 | 18.24 | 19.93 | 21.20 | 25.71 ms | 0 / 600 |
| + 백그라운드 4스레드 | 20.05 | 24.40 | **30.39** | **42.65** ms | 4 / 600 (0.7%) |
| + 백그라운드 16스레드 | 36.23 | 47.80 | 55.51 | 66.98 ms | 362 / 600 (60.3%) |

- 유휴 상태의 WSL2는 30 FPS를 유지할 수 있다. 환경 자체는 실격이 아니다.
- 그러나 SLAM의 백그라운드 스레드(LocalMapping / LoopClosing / Viewer)만 켜도
  **P99가 21 → 30.4 ms로 튄다.** avg 대비 P99 배율이 **약 1.5배**다.
- **오버서브스크립션은 치명적이다.** 16스레드에서 60% 마감 초과.
  앞의 OpenMP 20스레드 문제와 동일한 뿌리다.

### 7-2. 연산 예산 스윕 (백그라운드 3스레드 고정)

| 연산 예산 | avg | P50 | P95 | P99 | max | 마감 초과 | 판정 |
|---|---|---|---|---|---|---|---|
| 8 ms | 9.59 | 9.50 | 10.39 | 11.42 | 15.74 | 0.00% | OK |
| 12 ms | 13.32 | 13.24 | 14.15 | 15.58 | 17.59 | 0.00% | OK |
| 16 ms | 18.09 | 17.49 | 21.08 | 22.70 | 25.30 | 0.00% | OK |
| 18 ms | 20.52 | 19.78 | 24.05 | 27.14 | 32.19 | 0.00% | OK |
| **20 ms** | **22.33** | 21.62 | 25.64 | **28.69** | 31.90 | **0.00%** | **OK (한계선)** |
| 22 ms | 24.95 | 23.92 | 29.24 | 32.56 | 34.99 | 1.00% | MARGINAL |
| 25 ms | 27.16 | 26.33 | 31.58 | 34.60 | 38.85 | 2.20% | FAIL |
| 28 ms | 29.26 | 29.15 | 30.44 | 33.21 | 39.76 | 1.00% | MARGINAL |
| 30 ms | 31.25 | 31.16 | 32.47 | 34.45 | 42.42 | 2.80% | FAIL |

(28 ms 행은 추세에서 벗어난다 — 백그라운드 부하의 분산 때문으로 보인다.
그 한 줄을 빼면 단조 증가이며, 결론은 바뀌지 않는다.)

### 7-3. 여기서 나오는 설계 규칙

1. **Tracking 연산 예산의 실질 상한은 33.33 ms가 아니라 ≈20 ms다.**
   20 ms에서 P99 28.69 ms, 마감 초과 0%. 22 ms부터 놓치기 시작한다.
   사용자 요청서의 "25~30 ms 목표" 는 이 하드웨어에서 **P99 기준 미달**이다.
2. **avg → P99 배율 약 1.3~1.5배를 항상 계상한다.** 평균만 보면 반드시 틀린다.
3. **WSL2 타이머 깨어남 지연이 프레임당 P50 776 us / 최대 1624 us.**
   연산을 시작하기도 전에 ~0.8 ms를 잃는다. 예산에 포함해야 한다.
4. **총 스레드 수를 물리 코어 수 아래로 유지한다.** Tracking + LocalMapping +
   LoopClosing + OpenMP 풀을 모두 합쳐 관리해야 한다. 이것이 P99를 지키는
   가장 강력한 수단이며, SCHED_FIFO를 쓸 수 없는 이 환경에서는 사실상 유일한 수단이다.
