# ORB-SLAM3 realtime 30 FPS optimization analysis — RTX 3080 Ti Laptop + i9-12900HK + WSL2

**Agent 1 · 2026-09-12 · scope: `orbslam_ws/` only**

Source of truth: `orbslam_ws/src/ORB_SLAM3` (29 406 lines of `.cc`) and
`orbslam_ws/src/orbslam3_ros2`. Every symbol, constant and line number below was
read out of those files. Nothing here is recalled from upstream ORB-SLAM3 —
this fork has local modifications (prior-map localization, dense-map
reprojection) that change some conclusions.

Evidence is tagged throughout:

| Tag | Meaning |
|---|---|
| **[V]** | **Verified from source** — `file:line` given, read directly. |
| **[M]** | **Measured on this machine.** Either by me (micro-benchmarks in this session, described inline) or by the coordinator (`slam_comparison_report/realtime_30fps/HW_MEASURED_BASELINE.md`). |
| **[E]** | **Estimated / derived.** Reasoning stated, uncertainty stated. Not measured — ORB-SLAM3 cannot be built on this machine (see below). |
| **[X]** | **Requires an experiment** that could not be run here. |

> **Blocking fact [V]:** ORB-SLAM3 is **not built on this laptop** and cannot be
> without work. There is no system OpenCV (`find_package(OpenCV 4.4)`,
> `CMakeLists.txt:37`), no `/opt/ros`, and `conda env list` has no `orbslam3`
> env. So no end-to-end profile exists. Where I needed a number I either
> measured a faithful standalone replica of the exact ORB-SLAM3 code (labelled
> **[M]**, replica source noted), or gave a reasoned range (**[E]**).

---

## 🔴 KEY FINDINGS (read first)

**F1 — `Optimizer::PoseOptimization()` runs TWICE per frame, 40 Levenberg iterations each. [V]**
This is the single biggest source-level finding and it is not in any existing patch.
- Call 1: `Tracking::TrackWithMotionModel()` → `Optimizer::PoseOptimization` (`src/Tracking.cc:2991`), or `Tracking::TrackReferenceKeyFrame()` (`src/Tracking.cc:2827`).
- Call 2: `Tracking::TrackLocalMap()` → `Optimizer::PoseOptimization` (`src/Tracking.cc:3053`).
- Each call runs the schedule `const int its[4]={10,10,10,10};` (`src/Optimizer.cc:1003`) = **4 rounds × 10 LM iterations = 40**, with a **dense** solver (`g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>`, `src/Optimizer.cc:818`).
- So a normal frame performs **80 LM iterations** of 6-DoF-only pose refinement. **[E]** 3–7 ms/frame, i.e. **20–35 % of the 33.33 ms budget**, spent almost entirely re-converging an already-converged pose. This is the highest-yield CPU target in the whole system (see **A11**).

**F2 — The BRIEF pattern is byte-identical to OpenCV's, so `ORBvoc.txt` survives a GPU port. [V] + verified against upstream OpenCV**
`static int bit_pattern_31_[256*4]` (`src/ORBextractor.cc:149-408`) is verbatim OpenCV's `bit_pattern_31_`, down to the `/*mean (0), correlation (0)*/` annotations, and the constructor consumes it exactly as OpenCV's WTA_K=2 path does (`std::copy(pattern0, pattern0+512, back_inserter(pattern))`, `src/ORBextractor.cc:448-450`). I fetched and diffed OpenCV `4.x` `modules/features2d/src/orb.cpp` and `opencv_contrib` `modules/cudafeatures2d/src/orb.cpp`: **all three use the same table**, `cv::cuda::ORB` applies the same `createGaussianFilter(CV_8UC1,-1,Size(7,7),2,2,BORDER_REFLECT_101)` pre-blur that ORB-SLAM3 applies at `src/ORBextractor.cc:1131`, and the same intensity-centroid orientation.
→ **Descriptors are compatible. The DBoW2 vocabulary stays valid.** The incompatibility is *keypoint selection*, not description — see **F3**.

**F3 — `cv::cuda::ORB` is still not a drop-in, because it replaces the octree homogenization with a global response sort. [V]**
ORB-SLAM3 spreads keypoints spatially via `ORBextractor::DistributeOctTree` (`src/ORBextractor.cc:555-779`). `cv::cuda::ORB` culls with `cull_gpu()` — a **global response sort** per level, no spatial spreading. Losing homogenization degrades `Frame::AssignFeaturesToGrid` occupancy (`src/Frame.cc:385-413`), which degrades every `Frame::GetFeaturesInArea` search, which degrades tracking robustness. Independently, this machine has **no CUDA OpenCV at all** — option (A) requires a full OpenCV rebuild with `-DWITH_CUDA=ON -DCUDA_ARCH_BIN=8.6`.

**F4 — `DistributeOctTree` is the hard floor on any GPU-ORB design, and it costs 1.4 ms. [M]**
I built a byte-faithful standalone replica of `DistributeOctTree` + `ExtractorNode::DivideNode` (28-byte KeyPoint POD, no OpenCV) and measured it on this CPU at the project's real config (640×480, 8 levels, `nFeatures: 1250` from `data/orbslam3_d455f_640x480_rgbd.yaml`):

| FAST corners fed in (level 0) | DistributeOctTree, all 8 levels |
|---|---|
| 2 000 | **0.560 ms** |
| 4 000 | **1.028 ms** |
| 8 000 | **1.405 ms** |
| 16 000 | **2.635 ms** |
| 32 000 | **4.870 ms** |
| 64 000 | **10.054 ms** |

Cost is **linear in the number of INPUT corners and independent of the 1250 output**. Consequence: a GPU FAST that hands the CPU more corners than the CPU FAST would have **destroys the entire GPU gain**. The GPU FAST must reproduce ORB-SLAM3's per-cell threshold fallback (`iniThFAST` then `minThFAST` if the cell is empty, `src/ORBextractor.cc:826-840`) *and* pre-cull per cell on the GPU.

**F5 — GPU descriptor matching is a net loss on this machine. Do not do it. [M, coordinator]**
GPU offload fixed floor = 0.108 ms (64 KB H2D + kernel + 64 KB D2H); one `cudaDeviceSynchronize` alone = 26 µs. ORB-SLAM3 never brute-forces: `Frame::GetFeaturesInArea` (`src/Frame.cc:657`) constrains each query to ~10–40 candidates, so real per-frame comparisons are 10⁴–10⁵, where CPU POPCNT costs **0.03–0.15 ms**. Zeroing the compute still loses. Detail and arithmetic in **A8**.

**F6 — The existing `orbslam3-perf-all.patch` would *break* 30 FPS as written, because of OpenMP team size. [M, coordinator] + [V]**
`orbslam3-perf-all.patch` opens **three** `#pragma omp parallel for` regions per `ORBextractor::operator()` call. Measured parallel-region entry cost on this box:

| OpenMP team | Cost per parallel region |
|---|---|
| first region ever (pool spawn) | **10.508 ms** |
| warm, default 20 threads | **3.786 ms** |
| warm, 8 threads | 0.0033 ms |
| warm, 4 threads | **0.0015 ms** |
| warm, 2 threads | 0.0053 ms |

The patch *does* default `ORBEXTRACTOR_OMP_THREADS` to 4 via `num_threads(...)`, which is correct — but it (a) never warms the pool, so the **first frame takes a 10.5 ms hit**, and (b) never bounds **OpenCV's own** pool, which `resize`/`GaussianBlur`/`FAST` all use, so the two pools oversubscribe 20 logical CPUs. Fixed by the new `03-thread-governance.patch`.

**F7 — The ROS2 wrapper, not ORB-SLAM3, sets P99 in this project's actual configuration. [V]**
`orbslam3_ros2/src/rgbd_node.cpp:443-445` calls `accumulateDenseMapIfNeeded`, `publishDenseMapIfNeeded` and `publishMapPointsIfNeeded` **inline on the RGB-D sync callback thread**, immediately after `TrackRGBD`. With `config/no_cli_rgbd.yaml` (`dense_map.max_points: 2000000`, `publish_period_sec: 2.0`), `publishDenseMapIfNeeded` snapshots the whole voxel hash into a vector and serialises up to **2 000 000 × 16 B = 32 MB** into a `PointCloud2`, **on the tracking thread**, once every 2 s. **[E]** that is a 150–600 ms stall on 1 frame in ~60. No amount of ORB-SLAM3 optimization fixes it. Fixed by the new `05-ros2-latency.patch`.

**F8 — Global Bundle Adjustment freezes tracking for hundreds of milliseconds and there is no mitigation in the code. [V]**
`LoopClosing::RunGlobalBundleAdjustment` (`src/LoopClosing.cc:2290`) calls `Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,...)` (`:2306`), then `mpLocalMapper->RequestStop()`, spins on `isStopped()`, then takes `unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate)` (`src/LoopClosing.cc:2351`) for the **entire spanning-tree correction of every keyframe and every map point in the map**. `Tracking::Track()` holds that same mutex for its whole body (`src/Tracking.cc:1886`). So on loop closure, tracking is blocked for the duration of the correction — **[E] 100–800 ms** on a 500–1500 keyframe map. This is the unavoidable P99 of ORB-SLAM3 and it is architectural, not a tuning problem.

**F9 — 33.33 ms mean is already achievable at 640×480. P99 is not, without work.** See **A19**.

---

## Review of the existing `perf_patches/` (prior session)

I verified every hunk against the current source and measured the two that were measurable.

| Patch | Applies today? | Verdict |
|---|---|---|
| `orbslam3-perf-all.patch` (18 KB, 9 files) | **Yes, clean** (`patch -p1 --dry-run` from `src/ORB_SLAM3`) | **Sound.** All claims verified. See per-hunk table below. |
| `01-hotpath-allocations.patch` | Yes, clean | **Sound but redundant** — fully contained in the all-patch. |
| `02-descriptor-distance-popcnt.patch` | **NO — corrupt** (`patch: **** unexpected end of file in patch`, `git apply: corrupt patch at line 77`) | **Truncated file.** `gen_patches.py` only ever writes `orbslam3-perf-all.patch` (see its `main()`), so 01/02 were hand-extracted and 02 lost its trailing context. **Do not use it; the all-patch contains the same change correctly.** |
| `gen_patches.py` | n/a | **Good methodology** — literal `old→new` substitution with a MISS report, then `diff -u`. I reused it for the new patches (`gen_patches2.py`). |
| `test_descriptor_distance.cpp` | builds | Correct equivalence + speed harness. Its `DescriptorDistance_stock` is a faithful copy of `src/ORBmatcher.cc:2058-2076`. |

### Per-hunk verdict on `orbslam3-perf-all.patch`

| # | Hunk | Claim | Verified? | Measured effect |
|---|---|---|---|---|
| 1 | `Frame::GetFeaturesInArea`: `reserve(N)`→`reserve(32)`, and `const vector<size_t> vCell = …` → `&` | Two heap allocations per call/cell | **[V]** `src/Frame.cc:659` and `:695` — the missing `&` is real, it is a by-value `std::vector` copy per visited grid cell | **[M] 1.62–2.06×**, saves **0.06–0.22 ms/frame**. See A12. |
| 2 | `DescriptorDistance`: SWAR → `__builtin_popcountll`, + raw-pointer overload | Bit-identical, hottest function | **[V]** `src/ORBmatcher.cc:2058`; 20 call sites confirmed by grep | **[M, coordinator] 1.62–1.94×**, saves **0.15–0.30 ms/frame** on the tracking path |
| 3 | `MapPoint::AccumulateObservingKeyFrames` replacing `GetObservations()` in `UpdateLocalKeyFrames` | `GetObservations()` deep-copies the whole `std::map` | **[V]** `src/MapPoint.cc:204-209` returns `mObservations` by value; used at `src/Tracking.cc:3552` and `:3575` | **[M] 2.64–2.65×**, saves **0.24–0.59 ms/frame**. **Largest single CPU win in the existing patch set.** |
| 4 | `MapPoint::CopyDescriptorTo` replacing `GetDescriptor()` in `SearchByProjection` | `GetDescriptor()` returns `mDescriptor.clone()` | **[V]** `src/MapPoint.cc:416-419` | **[E]** 0.05–0.15 ms/frame (one malloc per visible local map point) |
| 5 | OMP per-pyramid-level parallelism in `ComputeKeyPointsOctTree`, `computeOrientation`, descriptors | Levels are independent | **[V] Correct.** `DistributeOctTree` writes no member state (only reads `nfeatures`, `src/ORBextractor.cc:759`); `vToDistributeKeys` is a per-iteration local (`:795`); `allKeypoints` is pre-sized (`:783`). **Thread-safe.** | **[M] Hazardous as shipped** — see **F6**. Needs `03-thread-governance.patch`. |
| 6 | `mvBlurBuffer` replacing `mvImagePyramid[level].clone()` | `copyTo` into a same-size/type buffer reuses the allocation and still yields a standalone Mat, so `BORDER_REFLECT_101` stays ROI-isolated | **[V] Correct and subtle.** This *matters*: `mvImagePyramid[level]` is an **ROI into a bordered `temp`** (`src/ORBextractor.cc:1178`), so blurring it in place would read the border instead of reflecting. `copyTo` into a standalone buffer preserves stock behaviour exactly. | **[E]** 0.2–0.5 ms/frame (8 clones of pyramid levels avoided) |
| 7 | `if(mpViewer)` guards around `mpFrameDrawer->Update(this)` and `mmProjectPoints` | Nothing reads it headless | **[V] Correct.** `Tracking::mpViewer` is `NULL` at `src/Tracking.cc:47` and only set by `SetViewer` (`:1439`), which `System` calls only under `if(bUseViewer)` (`src/System.cc:229-236`). The ROS2 node passes `false` (`rgbd_node.cpp:275-279`) and `config/no_cli_rgbd.yaml` has `visualization.enabled: false`. | **[M]** a `Frame` deep copy is **0.070 ms**; the guard removes one per frame plus the `mmProjectPoints` red-black-tree inserts → **~0.1–0.2 ms/frame**. Smaller than the patch comment implies — report it honestly. |
| 8 | CMake: documents why `-ffast-math`, AVX-512 and `-flto` are rejected | | **[V] The `-ffast-math` and AVX-512 reasoning is correct** (see A4). | 0 (documentation) |

### Gaps in the existing patch set

1. **`ORBSLAM3_PROFILE` / `include/StageTimer.h` is a dangling option.** The CMake hunk adds the option and the `-DORBSLAM3_PROFILE` define and references `include/StageTimer.h`, but **no such header is created and no source uses the macro**. Enabling it does nothing. (`grep -c 'StageTimer|ORBSLAM3_PROFILE'` in the patch = 5, all inside the CMake comment/option block.)
2. **No OpenMP warm-up and no `cv::setNumThreads`** → F6.
3. **`PoseOptimization` untouched** → F1, the biggest single target.
4. **The ROS2 wrapper untouched** → F7.
5. **`System::TrackRGBD` unconditional `im.clone()` + `depthmap.clone()`** (`src/System.cc:336-337`) — **[M] 0.069 ms/frame** wasted when `settings_->needToResize()` is false, on top of the clone the ROS2 node already did in `imageToBgr`.

---

## A1 — Per-frame call chain, image-in to tracking-done

### Entry points [V]

| Sensor | ROS2 / System entry | Tracking entry | Frame ctor | ORB entry |
|---|---|---|---|---|
| RGB-D | `System::TrackRGBD` `src/System.cc:329` | `Tracking::GrabImageRGBD` `src/Tracking.cc:1520` | `Frame::Frame(imGray, imDepth, …)` `src/Frame.cc:200` | `ExtractORB(0,imGray,0,0)` `src/Frame.cc:222` |
| Stereo (pinhole) | `System::TrackStereo` | `Tracking::GrabImageStereo` `src/Tracking.cc:1454` | `Frame::Frame(imLeft,imRight,…)` `src/Frame.cc:101` | **two `std::thread`s**: `src/Frame.cc:122-125` |
| Stereo (fisheye, `mpCamera2`) | as above | as above | `Frame::Frame(…,pCamera2,Tlr,…)` `src/Frame.cc:1034` | **two `std::thread`s** with lapping areas: `src/Frame.cc:1059-1062` |
| Mono | `System::TrackMonocular` `src/System.cc:399` | `Tracking::GrabImageMonocular` `src/Tracking.cc:1566` | `Frame::Frame(imGray,…)` `src/Frame.cc:289` | `ExtractORB(0,imGray,0,1000)` `src/Frame.cc:311` |

`Frame::ExtractORB` (`src/Frame.cc:418-424`) is a thin dispatcher to
`ORBextractor::operator()` (`src/ORBextractor.cc:1086`) — **confirmed, there is
no other extractor entry point.**

### RGB-D critical path (this project's configuration)

```
rgbd_node.cpp:428   slam_->TrackRGBD(rgb, depth, ts, imu)         [under slam_mutex_]
 └ System.cc:336-337  im.clone() + depthmap.clone()               [M] 0.069 ms  ← unconditional
   System.cc:340      cv::resize if settings_->needToResize()     (not used: no Camera.newWidth in yaml)
   System.cc:346-367  mode-change + reset checks (2 mutexes)      ~0
 └ Tracking.cc:1520 GrabImageRGBD
   ├ Tracking.cc:1526-1540 cvtColor BGR→GRAY                      [E] 0.15-0.30 ms
   ├ Tracking.cc:1542-1543 imDepth.convertTo(CV_32F, 1/1000)      [E] 0.20-0.40 ms
   ├ Tracking.cc:1546  Frame ctor  ────────────────────────────────────────────
   │   ├ Frame.cc:222  ExtractORB → ORBextractor::operator()      [E] 5-9 ms  ← DOMINANT
   │   │   ├ ORBextractor.cc:1096 ComputePyramid                  [E] 0.3-0.8 ms
   │   │   ├ ORBextractor.cc:1099 ComputeKeyPointsOctTree         [E] 3.5-6 ms
   │   │   │   ├ :826/:840 cv::FAST per 35x35 cell, 8 levels      [E] 2.0-4.5 ms
   │   │   │   ├ :874 DistributeOctTree                           [M] 1.40 ms
   │   │   │   └ :901 computeOrientation (IC_Angle)               [E] 0.15-0.4 ms
   │   │   └ ORBextractor.cc:1123-1163 blur + computeDescriptors  [E] 1.2-3.0 ms
   │   ├ Frame.cc:235  UndistortKeyPoints (cv::undistortPoints)   [E] 0.10-0.25 ms
   │   ├ Frame.cc:237  ComputeStereoFromRGBD                      [E] 0.05-0.15 ms
   │   └ Frame.cc:285  AssignFeaturesToGrid (64x48 cells)         [E] 0.03-0.08 ms
   └ Tracking.cc:1560 Track()  ──────────────────────────────────────────────
       ├ Tracking.cc:1886  lock(pCurrentMap->mMutexMapUpdate)     ← BLOCKS on LocalMapping/LoopClosing
       ├ Tracking.cc:2991  TrackWithMotionModel
       │   ├ Tracking.cc:2942 UpdateLastFrame                     [E] 0.2-0.6 ms (creates temporal MPs for RGBD)
       │   ├ Tracking.cc:2967 SearchByProjection(Frame,LastFrame,th=15)  [M] 0.03-0.05 ms of Hamming
       │   └ Tracking.cc:2991 PoseOptimization   #1               [E] 1.5-3.5 ms  ← F1
       ├ Tracking.cc:3043  TrackLocalMap
       │   ├ Tracking.cc:3038 UpdateLocalMap
       │   │   ├ :3541 UpdateLocalKeyFrames                       [M] 0.39-0.94 ms (0.15-0.36 patched)
       │   │   └ :3509 UpdateLocalPoints                          [M] 0.04-0.05 ms
       │   ├ Tracking.cc:3039 SearchLocalPoints
       │   │   ├ :3458 Frame::isInFrustum  x |mvpLocalMapPoints|  [E] 0.3-0.8 ms
       │   │   └ :3494 SearchByProjection(F,vpMapPoints,th)       [M] 0.06-0.15 ms Hamming
       │   │                                                       + [M] 0.14-0.24 ms GetFeaturesInArea
       │   └ Tracking.cc:3053 PoseOptimization   #2               [E] 1.5-3.5 ms  ← F1
       ├ Tracking.cc:2224  mpFrameDrawer->Update(this)            [M] 0.070 ms (0 if headless-guarded)
       ├ Tracking.cc:2262  NeedNewKeyFrame()                      [E] 0.02-0.05 ms
       └ Tracking.cc:2267  CreateNewKeyFrame()                    [E] 2-6 ms, 1 frame in 5-15
 back in rgbd_node.cpp:443-445
   ├ accumulateDenseMapIfNeeded  (every 5th frame, 160x120 px)    [E] 1-3 ms
   └ publishDenseMapIfNeeded     (every 2.0 s, up to 32 MB)       [E] 150-600 ms  ← F7
```

---

## A2 — Critical path vs. background: where P99 spikes come from

### The one mutex that matters [V]

`Tracking::Track()` takes `unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate)`
at **`src/Tracking.cc:1886`** and holds it for the **entire remainder of the
function** — pose prediction, local-map tracking, both pose optimizations,
keyframe creation. Everything that also takes that mutex therefore stalls
tracking directly:

| Holder | Where | What it does under the lock | **[E]** stall on Tracking |
|---|---|---|---|
| `Optimizer::LocalBundleAdjustment` | `src/Optimizer.cc:1464` | erase outlier observations, write back poses for `lLocalKeyFrames`, write back + `UpdateNormalAndDepth()` for **every** `lLocalMapPoints` (`:1487-1493`) | **2–10 ms**, once per keyframe |
| `Optimizer::LocalInertialBA` | `src/Optimizer.cc:2057` | same shape, inertial | 2–12 ms |
| `LocalMapping::KeyFrameCulling` / `InitializeIMU` | `src/LocalMapping.cc:1280`, `:1317`, `:1473` | `ApplyScaledRotation` over the whole map | 1–20 ms (IMU init only) |
| `LoopClosing::MergeLocal` | `src/LoopClosing.cc:1529-1530`, `:1671`, `:1745-1746` | takes **two** map mutexes and rewrites both maps | 50–500 ms |
| `LoopClosing::RunGlobalBundleAdjustment` | `src/LoopClosing.cc:2351` | spanning-tree correction of **all** KFs + **all** MPs | **100–800 ms** ← **F8** |
| `Optimizer::OptimizeEssentialGraph` | `src/Optimizer.cc:1733` | pose-graph result write-back | 20–200 ms |

### The soft handshakes [V]

These do not block tracking but do throttle it:

- `Tracking::CreateNewKeyFrame()` → `if(!mpLocalMapper->SetNotStop(true)) return;` (`src/Tracking.cc:3303`) — **silently drops the keyframe** if LocalMapping is stopping.
- `Tracking::NeedNewKeyFrame()` → `if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested()) return false;` (`src/Tracking.cc:3170`) — **no keyframes at all while a loop closure is running**. During GBA the map stops growing.
- `NeedNewKeyFrame` → `mpLocalMapper->InterruptBA()` (`src/Tracking.cc:3266`) sets `mbAbortBA=true` (`src/LocalMapping.cc:899`), which `Optimizer::LocalBundleAdjustment` polls via `pbStopFlag`. This is the only backpressure valve and it is one-directional.
- `LoopClosing` spins `while(!mpLocalMapper->isStopped()) usleep(1000);` (`src/LoopClosing.cc:1018`, `:1272`, `:1731`, `:1845`) — a 1 ms polling loop, three times over.

### Conclusion

Tracking's **mean** latency is set by ORB extraction + 2× pose optimization.
Tracking's **P99** is set by `mMutexMapUpdate` contention, and the worst case is
Global BA, which is **unbounded in the current design**. Any credible
"30 FPS with bounded P99" plan must address F8; shaving 2 ms off ORB extraction
does nothing for a 400 ms GBA stall.

---

## A3 — Profiling instrumentation plan

### What already exists [V]

ORB-SLAM3's `REGISTER_TIMES` mechanism is **present and complete in this fork,
but switched off**: `include/Settings.h:24` has `//#define REGISTER_TIMES`
commented out, and nothing in `CMakeLists.txt` defines it.

It fills these vectors (`include/Tracking.h:184-192`) [V]:

```
vdRectStereo_ms  vdResizeImage_ms  vdORBExtract_ms  vdStereoMatch_ms
vdIMUInteg_ms    vdPosePred_ms     vdLMTrack_ms     vdNewKF_ms      vdTrackTotal_ms
```

plus LocalMapping (`include/LocalMapping.h:114`: `vdKFInsert_ms`, `vdMPCulling_ms`,
`vdMPCreation_ms`, `vdLBA_ms`, `vdKFCulling_ms`, `vdLMTotal_ms`, `vnLBA_edges`,
`vnLBA_KFopt`, …) and LoopClosing (`include/LoopClosing.h:87`, incl. `vdGBA_ms`,
`vnGBAKFs`, `vnGBAMPs`). Reporting is `Tracking::PrintTimeStats()`
(`src/Tracking.cc:263`), which writes `ExecMean.txt` + per-frame CSVs via
`TrackStats2File()` (`:213`) and `LocalMapStats2File()` (`:184`).

**Limitation [V]:** `PrintTimeStats` reports **mean ± std only**. For a hard
33.33 ms budget you need percentiles, and the granularity stops at
`vdORBExtract_ms` — it cannot tell you FAST vs octree vs descriptors.

### Plan

**Step 1 — turn on what exists (zero new code).** The existing
`orbslam3-perf-all.patch` already adds `option(ORBSLAM3_REGISTER_TIMES …)`
(`CMakeLists.txt` hunk). Build with `-DORBSLAM3_REGISTER_TIMES=ON`. This gives
the coarse table immediately.

**Step 2 — add percentiles to the existing reporter, don't build a parallel one.**
Extend `Tracking::PrintTimeStats` (`src/Tracking.cc:263`) — it already has
`calcAverage`/`calcDeviation` helpers and every vector in scope:

```cpp
// src/Tracking.cc, inside PrintTimeStats(), next to calcAverage/calcDeviation
static void reportPct(const char* name, std::vector<double> v, std::ostream& o)
{
    if(v.empty()) return;
    std::sort(v.begin(), v.end());
    auto q = [&v](double p){ return v[std::min(v.size()-1,
                             (size_t)(p*(double)(v.size()-1)+0.5))]; };
    o << std::fixed << std::setprecision(3)
      << name  << " n=" << v.size()
      << "  p50=" << q(0.50) << "  p90=" << q(0.90)
      << "  p95=" << q(0.95) << "  p99=" << q(0.99)
      << "  max=" << v.back()
      << "  over33.33=" << std::count_if(v.begin(), v.end(),
                            [](double x){ return x > 33.33; })
      << std::endl;
}
```
called as `reportPct("TrackTotal", vdTrackTotal_ms, std::cout);` etc. for all
nine tracking vectors, `vdLBA_ms`, and `vdGBA_ms`.

**Step 3 — the missing sub-extractor granularity.** The all-patch's
`ORBSLAM3_PROFILE` option is **dangling** (no `StageTimer.h` exists). Either
delete the option or implement it. The minimal useful version adds four probes
inside `ORBextractor::operator()` (`src/ORBextractor.cc:1086`) — note these
**must** be `#ifdef`'d out by default, because a `steady_clock::now()` pair per
pyramid level inside an OMP region is itself measurable:

```cpp
// src/ORBextractor.cc:1096 onwards, in operator()
#ifdef ORBSLAM3_PROFILE
  #define ORB_T0(v) auto v = std::chrono::steady_clock::now()
  #define ORB_DT(a,b) std::chrono::duration<double,std::milli>(b-a).count()
#else
  #define ORB_T0(v) do{}while(0)
#endif
        ORB_T0(t0);  ComputePyramid(image);
        ORB_T0(t1);  ComputeKeyPointsOctTree(allKeypoints);
        ORB_T0(t2);  /* blur + computeDescriptors loop */
        ORB_T0(t3);
#ifdef ORBSLAM3_PROFILE
        mLastPyramid_ms = ORB_DT(t0,t1);
        mLastKeyPoints_ms = ORB_DT(t1,t2);
        mLastDescriptors_ms = ORB_DT(t2,t3);
#endif
```
and, inside `ComputeKeyPointsOctTree`, one accumulator around the FAST loop
(`:826-887`) and one around `DistributeOctTree` (`:874`). Those two numbers are
what decides whether GPU ORB is worth it — **`DistributeOctTree` cannot go to
the GPU (F4), so if it is already 25 % of extraction, the GPU ceiling is 4×.**

**Step 4 — end-to-end, already present in the wrapper [V].**
`rgbd_node.cpp:429-431` already timestamps `TrackRGBD` into `track_times_ms_`
and the destructor already sorts it (`:344-345`). Extend that print with the
same percentile helper, and add a second timer around lines 443-445 so the
dense-map cost (F7) shows up separately from tracking.

**[X] Step 5 — the experiment I could not run.** Build the system and record
`vdTrackTotal_ms` over ≥3000 frames of `data/rgbd_bag`, with and without the
patch stack. Everything in A19 is a prediction until that exists.

---

## A4 — CPU compiler / build flags

### Current state [V]

`src/ORB_SLAM3/CMakeLists.txt`:
```cmake
13: set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall   -O3")
14: set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
15: set(CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -march=native")
16: set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
 7: IF(NOT CMAKE_BUILD_TYPE) SET(CMAKE_BUILD_TYPE Release) ENDIF()
26: set(CMAKE_CXX_STANDARD 14)
```
So the core library already gets `-O3 -march=native`. **This is already close to
optimal on this CPU and is not where the wins are.**

`src/orbslam3_ros2/CMakeLists.txt`: **no build type, no optimisation flags at
all.** Only `-Wall -Wextra -Wpedantic`. It relies on the caller passing
`-DCMAKE_BUILD_TYPE=Release` (which `README.md:12` does), but nothing enforces
it, and even Release gives **no `-march=native`** — so the per-pixel dense-map
unprojection loop in `rgbd_node.cpp:697-760` gets no AVX2/FMA. **Fixed in
`05-ros2-latency.patch`.**

### `-march=native` on i9-12900HK [M]

`lscpu` flags on this machine include `avx2 fma bmi1 bmi2 popcnt sha_ni vaes
avx_vnni f16c` and **no `avx512*`** — Alder Lake fuses AVX-512 off when E-cores
are enabled. So:
- `-march=native` selects **AVX2 + FMA + BMI2 + POPCNT**. That is exactly what the POPCNT patch needs. ✔
- **Never propose an AVX-512 code path.** Forcing `-mavx512f` yields SIGILL.
- `-mtune=native` is implied by `-march=native` on GCC but harmless to state.

### `-ffast-math`: **do not use.** [V]

`-ffast-math` implies `-ffinite-math-only`, which lets the compiler assume no
NaN/Inf and licenses it to fold comparisons involving them. ORB-SLAM3 gates
correctness on exactly such comparisons. Verified occurrences:

| Site | What breaks |
|---|---|
| `src/Optimizer.cc:1015` `const float chi2 = e->chi2(); if(chi2>chi2Mono[it])` | Outlier rejection. A diverged edge produces `chi2 = NaN`; `NaN > 5.991` is `false`, so it is already fragile — with `-ffinite-math-only` GCC may fold the whole comparison. **Outlier rejection silently becomes a no-op → pose corrupted by outliers, no error message.** |
| `src/Optimizer.cc:1101` `if(optimizer.edges().size()<10) break;` | fine |
| g2o LM damping (`Thirdparty/g2o`) | The Levenberg lambda schedule divides by residual norms; `-freciprocal-math` and reassociation change the iterate sequence and therefore the result. |
| Eigen (`Thirdparty/Sophus`, `G2oTypes.cc`) | Eigen explicitly documents that `-ffast-math` breaks its NaN-based algorithms and some vectorised reductions. |
| `src/Frame.cc:886` / `src/Frame.cc:940` depth sanity | `if(deltaR<-1 \|\| deltaR>1) continue;` — `deltaR` comes from a parabola fit whose denominator `(dist1+dist3-2*dist2)` can be 0. Stock code relies on `Inf`/`NaN` failing both comparisons and skipping. Under `-ffinite-math-only` this guard can be optimised away → garbage disparity → garbage depth. |

The existing patch's CMake comment already says this; **it is correct and should
stay.** If you want *some* of the benefit, `-fno-math-errno` alone is safe
(it only stops `errno` being set by `sqrt`/`pow`) and is already implied by `-O3`
on GCC for most cases.

### `-flto` / IPO

**[E] Low value here, non-trivial cost.** `ORB_SLAM3` is built as a `SHARED`
library (`CMakeLists.txt:64`) and consumed through a **prebuilt `.so` path** by
the ROS2 node (`orbslam3_ros2/CMakeLists.txt:63` hard-codes
`${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so`). Cross-TU inlining therefore stops at
the library boundary — and the hot loops are *inside* single translation units
already (`ORBmatcher.cc`, `ORBextractor.cc`). `DBoW2` and `g2o` are separate
`.so`s too. Expect **0–3 %** for a much longer rebuild. Try it only after
everything in A20 Phase 1–3 is done and measured.

### Recommended flag set

```cmake
# src/ORB_SLAM3/CMakeLists.txt  — already present, keep
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native")
# add: helps the descriptor/FAST loops, zero semantic risk
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fno-semantic-interposition")
# NEVER: -ffast-math, -funsafe-math-optimizations, -mavx512*
```
`-fno-semantic-interposition` matters specifically because this *is* a shared
library: without it every call to a non-static function in the same `.so` goes
through the PLT and cannot be inlined. `ORBmatcher::DescriptorDistance` is a
static member, so it is already fine, but `RadiusByViewingCos`,
`Frame::PosInGrid` and the `MapPoint` accessors are not. **[E] 1–4 %.**

---

## A5 — `ORBextractor` breakdown

### `ComputePyramid` is strictly serial — this is the decisive data dependency [V]

```cpp
// src/ORBextractor.cc:1170-1195
void ORBextractor::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < nlevels; ++level)
    {
        ...
        if( level != 0 )
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);   // :1180
        ...
    }
}
```
**Level `i` is resized from level `i-1`, not from level 0.** So the pyramid is a
serial chain and **cannot** be level-parallelised without changing results
(resizing from level 0 each time gives different pixels, hence different FAST
corners, hence a different map). The existing patch correctly does **not**
touch it. ✔

Two further details worth knowing:
- `mvImagePyramid[level]` is an **ROI** into a larger bordered `temp`
  (`:1176-1178`), and `temp` is a **local** that dies each iteration — the ROI
  keeps it alive via refcount. That is why hunk 6 of the existing patch
  (`clone()` → `copyTo(mvBlurBuffer[level])`) is correct and why blurring
  in-place would be wrong.
- `Mat temp(wholeSize, image.type())` is a **fresh allocation per level per
  frame** (`:1177`). 8 allocations totalling ~3.1× the base image.
  **[E] 0.05–0.15 ms/frame**; could be made persistent the same way
  `mvBlurBuffer` was. Small; listed as Phase 4 in A20.

### What IS level-parallel [V]

| Stage | `src/ORBextractor.cc` | Level-parallel? | Why |
|---|---|---|---|
| `ComputePyramid` | 1170-1195 | **NO** | `resize(level-1 → level)` |
| FAST over 35×35 cells | 786-891 | **YES** | reads `mvImagePyramid[level]`, writes local `vToDistributeKeys` (`:795`) |
| `DistributeOctTree` | 555-779 | **YES** | pure — reads only the `nfeatures` member (`:759`), writes nothing |
| `computeOrientation`/`IC_Angle` | 899-902, 75-101 | **YES** | free function, reads image + `umax`, writes `allKeypoints[level]` |
| blur + `computeDescriptors` | 1123-1163 | **YES** (with per-level buffers) | independent per level |
| the scatter into `_keypoints`/`descriptors` | 1141-1163 | **NO** | carries `offset`, `monoIndex`, `stereoIndex` across levels |

The existing patch parallelises exactly rows 2/3, 4 and 5, and keeps row 6
serial. **That partition is correct.**

### Thread-safety hazards — audited [V]

- `vToDistributeKeys` (`:795`) — declared **inside** the level loop. Per-thread. Safe.
- `allKeypoints` — `resize(nlevels)` at `:783` **before** the parallel region; each iteration touches only `allKeypoints[level]`. No reallocation race. Safe.
- `mvImagePyramid` — read-only during extraction. Safe.
- `mvBlurBuffer` (patch) — indexed by level, sized in the ctor. Safe.
- `pattern`, `umax`, `mnFeaturesPerLevel`, `mvScaleFactor` — const after ctor. Safe.
- **`cv::FAST` allocation churn** — each cell call constructs `vector<cv::KeyPoint> vKeysCell` (`:824`). ~250 cells/level × 8 levels = **~2000 vector allocations per frame per extractor**. Hoisting it out of the loop with `.clear()` is bit-identical and free. **[E] 0.1–0.3 ms.**
- `vToDistributeKeys.reserve(nfeatures*10)` (`:796`) = 12 500 × 28 B = **350 KB per level per frame**, 8 levels = 2.8 MB of `mmap`/`munmap` churn (above glibc's 128 KB `MMAP_THRESHOLD`). Also hoistable. **[E] 0.05–0.2 ms.**

### `ExtractorNode::DivideNode` over-reservation — measured, and smaller than it looks [M]

`src/ORBextractor.cc:489,494,499,504` each do `nX.vKeys.reserve(vKeys.size())` —
all four children reserve the **parent's full size**, a 4× over-allocation on
every split, and there are hundreds of splits per level. Measured with the
faithful replica:

| Reserve policy | 8k corners, nf=1250 | 12k corners, nf=2000 |
|---|---|---|
| stock `reserve(parent)` | 1.398 ms | 2.684 ms |
| `reserve(parent/4 + 8)` | **1.320 ms (−5.6 %)** | **2.346 ms (−12.6 %)** |
| no reserve at all | 1.632 ms (**worse**) | 2.911 ms (**worse**) |

Output keypoints identical (reserve only sets capacity). **Real but modest —
0.08–0.34 ms.** Shipped as `04-octree-node-reserve.patch`. I am reporting this
as *measured and small* rather than hyping it; my initial source-reading
estimate was much larger and the measurement corrected it.

### Concrete OpenMP code — with the F6 correction

The existing patch's pragma shape is right. What it is missing is team-size
governance. The full correct form, given the measurement in F6:

```cpp
// src/ORBextractor.cc:786 — as in orbslam3-perf-all.patch (correct as-is)
#if defined(_OPENMP) && defined(ORBEXTRACTOR_OMP_THREADS)
#pragma omp parallel for schedule(dynamic) num_threads(ORBEXTRACTOR_OMP_THREADS)
#endif
        for (int level = 0; level < nlevels; ++level) { ... }
```
plus, **new**, once at startup (`03-thread-governance.patch`):
```cpp
// src/System.cc — called first thing in System::System()
void System::ConfigureThreadPools()
{
    int n = 4;                                  // measured optimum on this box
    if(const char* e = std::getenv("ORBSLAM3_NUM_THREADS")) { ... }
    cv::setNumThreads(n);                       // OpenCV's pool — otherwise it
                                                // spawns its own 20 and fights
#ifdef _OPENMP
    omp_set_num_threads(n);
    omp_set_dynamic(0);
    volatile double warm = 0.0;                 // pay the 10.5 ms pool spawn HERE
    #pragma omp parallel for reduction(+:warm) num_threads(n)
    for(int i = 0; i < n*64; i++) warm += 1.0;
#endif
}
```

`schedule(dynamic)` is the right choice: level 0 is 2.99× the work of level 1
(area ratio `1.2²`), so a static schedule leaves three threads idle.

**Expected benefit [E]:** FAST + octree + orientation + descriptors is
~4.7–8 ms of the 5–9 ms extraction. With 4 threads over 8 levels whose work is
geometric (level 0 alone is 32 % of total pyramid area), Amdahl caps the speedup
at roughly **1/0.32 ≈ 3.1×** for the parallel part; realistically
**1.8–2.4× on extraction**, i.e. **2.5–4.5 ms saved**. Serial `ComputePyramid`
(0.3–0.8 ms) and the serial scatter remain.

**Caveat [E]:** in **stereo** mode `Frame::Frame` already runs two extractors on
two `std::thread`s (`src/Frame.cc:122-125`). Each will open its own OMP team, so
the machine runs 2 + 2×4 = 10 threads at peak. That is fine on 20 logical CPUs,
but it means the **stereo** path benefits far less from intra-extractor
parallelism than the mono/RGB-D path — the cheap 2× is already taken.

---

## A6 — Stereo path

### Left/right extraction is already threaded [V — confirmed]

```cpp
// src/Frame.cc:122-125  (pinhole stereo)
thread threadLeft(&Frame::ExtractORB,this,0,imLeft,0,0);
thread threadRight(&Frame::ExtractORB,this,1,imRight,0,0);
threadLeft.join();
threadRight.join();
```
```cpp
// src/Frame.cc:1059-1062  (KannalaBrandt8 fisheye stereo)
thread threadLeft (&Frame::ExtractORB,this,0,imLeft,
    static_cast<KannalaBrandt8*>(mpCamera )->mvLappingArea[0], ...mvLappingArea[1]);
thread threadRight(&Frame::ExtractORB,this,1,imRight,
    static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0], ...mvLappingArea[1]);
```
**Both stereo constructors already parallelise.** Two costs worth noting:
- `std::thread` construction + join is **[E] 30–80 µs per frame** under WSL2 — a persistent 2-worker pool would remove it, but it is small.
- These are raw `std::thread`s, *not* from a pool, so on top of the OMP teams they add churn. Another reason for `03-thread-governance.patch`.

### `Frame::ComputeStereoMatches()` [V] `src/Frame.cc:811-982`

Structure:
1. `vector<vector<size_t>> vRowIndices(nRows)` with `reserve(200)` per row (`:820-824`) — **480 vector allocations per frame**, ~96 000 reserved slots. **[E] 0.1–0.3 ms of pure allocation.**
2. For each right keypoint, push its index into every row band it spans (`:827-838`).
3. For each of `N` left keypoints: gather candidates from `vRowIndices[vL]`, filter by octave ±1 and disparity range, `ORBmatcher::DescriptorDistance` over each (`:886`). **[M]** at 2000×30 candidates this is 0.201 ms SWAR → 0.113 ms POPCNT.
4. **The expensive part**: sub-pixel refinement by an 11-tap sliding-window SAD using `cv::norm(IL,IR,cv::NORM_L1)` on 11×11 patches (`:908-933`). That is **11 `cv::norm` calls per matched keypoint**, each constructing two `cv::Mat` ROI headers. At ~800 matches that is **8800 `cv::norm` calls per frame**. **[E] 3–8 ms — this, not the Hamming, dominates `ComputeStereoMatches`.**

**Parallelisability:** step 3+4 is a perfectly parallel loop over `iL`. The only
shared writes are `mvuRight[iL]`, `mvDepth[iL]` (disjoint indices) and
`vDistIdx.push_back` (needs an `omp critical` or per-thread vectors merged after).
`vRowIndices` is read-only in the loop. **This is a clean, safe 4× target and
the existing patch does not touch it.**

```cpp
// src/Frame.cc:849 — proposed
    std::vector<std::vector<std::pair<int,int> > > vDistIdxT(nThreads);
#if defined(_OPENMP) && defined(ORBEXTRACTOR_OMP_THREADS)
#pragma omp parallel for schedule(dynamic,64) num_threads(ORBEXTRACTOR_OMP_THREADS)
#endif
    for(int iL=0; iL<N; iL++) { ... vDistIdxT[omp_get_thread_num()].push_back(...); }
    // then concatenate vDistIdxT into vDistIdx before the sort at :967
```
The `sort` at `:967` and the median cut at `:969-981` must stay serial, and the
**concatenation order must be deterministic** (concatenate by thread id, not by
completion) or the median tie-breaking changes. **[E] 2–5 ms saved on stereo.**

Also replacing `cv::norm(IL,IR,NORM_L1)` with a hand-written 11×11 `_mm256_sad_epu8`
SAD would remove 8800 Mat-header constructions. **[E] a further 1–3 ms.** Both
are stereo-only, so they are Phase 5 (this project runs RGB-D).

### Fisheye path `Frame::ComputeStereoFishEyeMatches()` [V] `src/Frame.cc:1126-1166`

Completely different: a **brute-force `cv::BFMatcher` knnMatch** over the
lapping-area descriptors (`:1145`, using the static `Frame::BFmatcher =
cv::BFMatcher(cv::NORM_HAMMING)` at `src/Frame.cc:42`), then Lowe ratio 0.7 and
`KannalaBrandt8::TriangulateMatches` per survivor.

**This is the one matching site in the whole system that IS brute force**, and
therefore the **only** one where a GPU Hamming kernel could pay (see A8). If the
lapping areas hold ~800 descriptors each, that is 640 000 comparisons ×2-NN.
**[M, coordinator]** CPU POPCNT at 400 k comparisons = 0.789 ms; GPU full round
trip at 4 M = 0.648 ms. So even here it is roughly break-even — and this project
uses a pinhole RGB-D D455, so `mpCamera2` is null and this path never runs.

---

## A7 — GPU ORB on a 3080 Ti Laptop: `cv::cuda::ORB` vs a custom extractor

### Option (A): `cv::cuda::ORB`

**Descriptor compatibility: COMPATIBLE. [V, verified against upstream]** — see **F2**.
Same `bit_pattern_31_`, same WTA_K=2 consumption, same 7×7 σ=2
`BORDER_REFLECT_101` pre-blur, same intensity-centroid orientation.
**`Vocabulary/ORBvoc.txt` remains valid.** This was the single most important
thing to check and the answer is favourable.

**Everything else is incompatible:**

| Aspect | ORB-SLAM3 | `cv::cuda::ORB` | Consequence |
|---|---|---|---|
| Keypoint selection | octree spatial homogenization, `DistributeOctTree` `src/ORBextractor.cc:555` | **global response sort** (`cull_gpu`) | Keypoints cluster on texture. `Frame::AssignFeaturesToGrid` (`src/Frame.cc:385`) fills unevenly → `GetFeaturesInArea` returns 0 candidates in weak regions → tracking robustness drops. **This is the real blocker.** |
| FAST thresholding | per 35×35 cell, `iniThFAST` then `minThFAST` fallback if empty (`:826`, `:840`) | one global `fastThreshold_`, capped at 5 % of level area | Different corner sets; weakly-textured cells contribute nothing |
| Blur | `GaussianBlur` **only** before descriptors (`:1131`), FAST sees the unblurred pyramid | `blurForDescriptor` flag | Must be set `true` or descriptors differ |
| Output layout | `monoIndex`/`stereoIndex` bidirectional fill for the fisheye lapping area (`:1152-1161`) | plain array | Wrapper must rebuild it |
| Level borders | `EDGE_THRESHOLD=19`, `mvImagePyramid` is an ROI into a bordered buffer (`:1178`) | different border handling | Sub-pixel keypoint coordinates shift |

**Build cost [V]:** there is **no CUDA OpenCV on this machine** — no
`libopencv_cuda*` anywhere, and the only OpenCV present is an OpenVINO 2021
bundle at `/opt/intel/openvino_2021/opencv`. Option (A) requires a full source
rebuild with `-DWITH_CUDA=ON -DCUDA_ARCH_BIN=8.6 -DOPENCV_EXTRA_MODULES_PATH=…opencv_contrib/modules`
(cudafeatures2d lives in contrib). **[E] 60–120 min of build time on this CPU,
plus reconciling it with the conda ROS/OpenCV that `find_package(OpenCV 4.4)`
currently resolves to** — that reconciliation is the real risk, not the build.

**Verdict on (A): not recommended.** You pay a full OpenCV rebuild and still
have to reimplement homogenization to keep ORB-SLAM3's robustness.

### Option (B): custom `ORBextractorCUDA` — recommended, with a hard constraint

**Measured GPU budget on this exact GPU [M, coordinator]** (8 levels, scaleFactor 1.2,
pyramid = 3.10× base pixels):

| Stage | 640×480 | 1280×720 | 1920×1080 |
|---|---|---|---|
| H2D upload (pinned) | 0.030 | 0.080 | 0.172 |
| ComputePyramid (7× bilinear) | 0.082 | 0.118 | 0.119 |
| GaussianBlur 7×7 × 8 levels | 0.151 | 0.211 | 0.379 |
| FAST-9 × 8 levels | 0.169 | 0.224 | 0.377 |
| IC_Angle + rBRIEF-256 | 0.156 | 0.157 | 0.157 |
| D2H keypoints + descriptors | 0.029 | 0.034 | 0.029 |
| **total (sync per stage)** | **0.638 ms** | **0.841 ms** | **1.186 ms** |
| total (single sync at end) | 0.888 | 0.981 | 1.349 |

Upper bounds: the input was random noise so FAST over-detected (178 k corners at
640×480), and rBRIEF was measured at 8000 keypoints vs. the 1250 target.
**`DistributeOctTree` is NOT in this table.**

**The hard constraint (F4):** whatever the GPU produces must be fed to
`DistributeOctTree` on the CPU, which costs **[M] 0.56 ms at 2 k corners,
1.40 ms at 8 k, 10.05 ms at 64 k**. So:

```
GPU-ORB realistic budget @640x480 =
      0.89 ms GPU (pyramid+blur+FAST+angle+BRIEF, one sync)
    + 0.05 ms  D2H of ~8k raw corners (8000 x 28 B = 224 KB, latency-bound)
    + 1.40 ms  DistributeOctTree on CPU          [M]  <-- irreducible
    + 0.10 ms  scatter / grid assign
    ~= 2.4 ms   vs.  [E] 5-9 ms CPU today   =>  2.1x - 3.8x
```
**Not 100×, not even 10×. Roughly 2–4×, saving [E] 2.5–6.5 ms/frame.** That is
still the single largest GPU opportunity on this machine — but the honest number
is 2–4×, and it only holds **if the GPU FAST hands back ~8 k corners, not 178 k.**

**Therefore the GPU FAST kernel must reproduce ORB-SLAM3's cell structure:**

```cuda
// One CUDA block per 35x35 ORB-SLAM3 cell (src/ORBextractor.cc:805-889).
// Reproduces: iniThFAST first; if the cell yields nothing, retry at minThFAST.
// Also caps each cell, which is what keeps DistributeOctTree cheap (F4).
__global__ void fast_per_cell(const uchar* __restrict__ img, int step,
                              int minBorderX, int minBorderY,
                              int wCell, int hCell, int nCols, int nRows,
                              int iniThFAST, int minThFAST,
                              short2* outPts, float* outResp, int* outCount,
                              int maxPerCell)
{
    const int cell = blockIdx.x;            // cell = i*nCols + j
    const int i = cell / nCols, j = cell % nCols;
    __shared__ int sCount;
    if(threadIdx.x == 0) sCount = 0;
    __syncthreads();

    for(int pass = 0; pass < 2; ++pass)
    {
        const int th = (pass == 0) ? iniThFAST : minThFAST;
        if(pass == 1 && sCount > 0) break;          // matches the CPU fallback
        for(int p = threadIdx.x; p < wCell*hCell; p += blockDim.x)
        {
            const int lx = p % wCell, ly = p / wCell;
            const int x = minBorderX + j*wCell + lx;
            const int y = minBorderY + i*hCell + ly;
            float score;
            if(fast9_score(img, step, x, y, th, &score))     // bresenham-16, contiguous-9
            {
                if(is_local_max_3x3(img, step, x, y, th, score))   // NMS, as cv::FAST(nonmax=true)
                {
                    int k = atomicAdd(&sCount, 1);
                    if(k < maxPerCell) { outPts[cell*maxPerCell+k]  = make_short2(x,y);
                                         outResp[cell*maxPerCell+k] = score; }
                }
            }
        }
        __syncthreads();
    }
    if(threadIdx.x == 0) outCount[cell] = min(sCount, maxPerCell);
}
```
`maxPerCell` is the lever that bounds F4. The CPU side then compacts the
per-cell arrays and calls the **unmodified** `DistributeOctTree`, so keypoint
homogenization — and therefore tracking robustness — is byte-for-byte preserved.

**Prior art agrees with this split (A18).** `Jetson-ORB-SLAM3` (arXiv 2608.17874)
implements "scale pyramid, FAST detection, multi-scale NMS, intensity-centroid
orientation, and steered rBRIEF" as CUDA kernels and explicitly **retains "the
reference grid-based homogenization and corner-score ordering"**, reporting
**94.7 % exact keypoint agreement and 99.9 % descriptor bit agreement** with the
CPU reference (mean Hamming distance 0.25 of 256 bits), with residuals attributed
to floating-point rounding and tie-breaking, **not** algorithmic difference. That
is the target fidelity for option (B).

---

## A8 — GPU Hamming matching: **do not do it** [M]

### Every `DescriptorDistance` call site [V] — 20 total, exactly as the patch claims

| File:line | Function | Typical candidates per query | Per-frame comparisons **[E]** |
|---|---|---|---|
| `ORBmatcher.cc:101` | `SearchByProjection(Frame&, vector<MapPoint*>, th)` — **local map** | `GetFeaturesInArea` → 10–40 | **3×10⁴ – 8×10⁴** |
| `ORBmatcher.cc:175` | same, right camera (fisheye) | 10–40 | 0 here (pinhole) |
| `ORBmatcher.cc:283`, `:304` | `SearchByBoW(KeyFrame*, Frame&, …)` — frame↔KF | per BoW node, 10–60 | 3×10⁴ – 6×10⁴ (only on `TrackReferenceKeyFrame`/reloc) |
| `ORBmatcher.cc:514`, `:627` | `SearchByProjection(KeyFrame*, Sim3, …)` — **loop closing** | 10–40 | off critical path |
| `ORBmatcher.cc:685` | `SearchForInitialization` | window `windowSize` | init only |
| `ORBmatcher.cc:834` | `SearchByBoW(KeyFrame*,KeyFrame*,…)` | | LocalMapping/LoopClosing |
| `ORBmatcher.cc:1015` | `SearchForTriangulation` | | LocalMapping |
| `ORBmatcher.cc:1302`, `:1427` | `Fuse` ×2 | | LocalMapping |
| `ORBmatcher.cc:1560`, `:1640` | `SearchBySim3` | | LoopClosing |
| `ORBmatcher.cc:1761`, `:1827` | `SearchByProjection(Frame&, const Frame&, th)` — **motion model** | 10–30 | **1×10⁴ – 3×10⁴** |
| `ORBmatcher.cc:1957` | `SearchByProjection(Frame&, KeyFrame*, sAlreadyFound, …)` — reloc | | reloc only |
| `Frame.cc:886` | `ComputeStereoMatches` | 10–40 per left kp | 5×10⁴ – 8×10⁴ (**stereo only**) |
| `MapPoint.cc:377` | `ComputeDistinctiveDescriptors` — **O(n²)** over observations | n²/2, n = 5–30 | LocalMapping, off path |
| `ORBmatcher.cc:2058` | the function itself | | |

**Every tracking-path site is grid- or BoW-constrained. None is brute force.**
`Frame::GetFeaturesInArea` (`src/Frame.cc:657`) with the radii from
`RadiusByViewingCos` (2.5 or 4.0, `src/ORBmatcher.cc:215-221`) × `th` × scale
gives 10–40 candidates.

### The arithmetic [M]

Measured CPU cost at exactly those sizes, and the measured GPU fixed floor:

| Workload | Comparisons | SWAR | **POPCNT** | GPU fixed floor |
|---|---|---|---|---|
| local-map `SearchByProjection` (1500 MP × 20) | 30 000 | 0.117 ms | **0.060 ms** | **0.108 ms** |
| local-map (2000 MP × 40) | 80 000 | 0.274 ms | **0.150 ms** | 0.108 ms |
| motion model (1000 × 15) | 15 000 | 0.051 ms | **0.029 ms** | 0.108 ms |
| `SearchByBoW` (1000 × 50) | 50 000 | 0.170 ms | **0.092 ms** | 0.108 ms |
| `ComputeStereoMatches` (2000 × 30) | 60 000 | 0.201 ms | **0.113 ms** | 0.108 ms |
| reloc/loop-heavy (2000 × 200) | 400 000 | 1.279 ms | **0.789 ms** | 0.108 ms |
| hypothetical brute force (2000 × 2000) | 4 000 000 | 12.851 ms | 7.883 ms | GPU round trip **0.648 ms** |

GPU fixed floor = 64 KB H2D + kernel + 64 KB D2H = **0.108 ms**, measured;
`cudaDeviceSynchronize` alone = **26 µs**.

> **At the sizes ORB-SLAM3 actually uses, the CPU POPCNT cost (0.03–0.15 ms) is
> at or below the GPU's fixed offload cost (0.108 ms) before the kernel does any
> work.** And there are 3–5 such matching calls per frame, so the GPU pays the
> floor 3–5 times. **Net loss.**

**Arithmetic intensity check [E]:** one comparison = 32 B + 32 B loaded, 4 XOR +
4 POPCNT + 3 ADD ≈ 11 ops → **0.17 ops/byte**. Utterly memory-bound. A 3080 Ti
Laptop at 512 GB/s could sustain ~8×10⁹ comparisons/s if both operands were
resident, but at 8×10⁴ comparisons the kernel runs for **10 µs** and the 0.108 ms
of transfer+launch dominates 10:1. The only way to win is to keep *both*
descriptor sets GPU-resident across frames — which is impossible, because the
local map point set changes every frame (`UpdateLocalPoints`,
`src/Tracking.cc:3509` clears and rebuilds `mvpLocalMapPoints` from scratch).

### Which of the five matching types could ever be worth it

| Type | Size | Verdict |
|---|---|---|
| frame-to-frame (motion model) | 1.5×10⁴ | **No** — 0.029 ms CPU |
| frame-to-keyframe (BoW) | 5×10⁴ | **No** — 0.092 ms CPU |
| projection (local map) | 3–8×10⁴ | **No** — 0.060–0.150 ms CPU |
| stereo (`ComputeStereoMatches`) | 6×10⁴ | **No** — and its real cost is `cv::norm`, not Hamming (A6) |
| local map / relocalization / loop candidate verification | 4×10⁵–4×10⁶ | **Marginal, and off the critical path.** `Relocalization` (`src/Tracking.cc:3691`) and `LoopClosing` already run asynchronously; a 7.9 → 0.65 ms win there does not change tracking latency. |
| **fisheye `ComputeStereoFishEyeMatches`** (`src/Frame.cc:1145`) | true brute force via `cv::BFMatcher` | **The only genuine candidate** — and `mpCamera2` is null in this project |

**Recommendation: keep matching on CPU with `__builtin_popcountll`.** The
existing patch 02 change (inside `orbslam3-perf-all.patch`) is the correct and
sufficient answer. It is worth **0.15–0.30 ms/frame** — necessary but nowhere
near sufficient on its own.

---

## A9 — CUDA stream pipelining: what the real frame dependency allows

### The dependency, from the source [V]

```
Frame N-1:  ExtractORB → Track() → mCurrentFrame.SetPose(...)
                                 → mVelocity = mCurrentFrame.GetPose() * mLastFrame.GetPose().inverse()   Tracking.cc:2230
                                 → mLastFrame = Frame(mCurrentFrame)                                       Tracking.cc:~2350
Frame N:    ExtractORB → Track():
              TrackWithMotionModel:  mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());   Tracking.cc:2957
                                     SearchByProjection(mCurrentFrame, mLastFrame, th)          Tracking.cc:2967
```

**`ExtractORB` for frame N depends on nothing from frame N-1.** It reads only
the image. The pose-dependent chain (`mVelocity`, `mLastFrame`,
`SearchByProjection(Frame,LastFrame)`) begins only *after* extraction.

**Therefore the one safe overlap is: extract frame N on the GPU while the CPU
finishes tracking frame N-1.** No false dependency, no change to `mVelocity` or
`mLastFrame` semantics, no change to results.

### What must NOT be overlapped

- **`TrackLocalMap`'s `PoseOptimization` for frame N-1 must complete before frame N's `TrackWithMotionModel`**, because `mVelocity` (`src/Tracking.cc:2230`) is computed from the *optimized* pose, and `mLastFrame`'s map-point associations are consumed by `SearchByProjection(mCurrentFrame, mLastFrame, …)` (`src/ORBmatcher.cc:1676`). Overlapping these would silently degrade the motion model.
- **Keyframe creation cannot be deferred past the next frame** — `CreateNewKeyFrame` uses `mCurrentFrame.mvDepth` and `mCurrentFrame.UnprojectStereo` (`src/Tracking.cc:3329-3386`).

### Design

```
stream 0 (GPU): [upload N][pyramid N][blur N][FAST N][angle+BRIEF N][D2H N]
CPU thread A  :        ... Track(N-1): PoseOpt, local map, PoseOpt, NeedNewKF ...
CPU thread A  :                                        then: octree(N) -> Frame(N) -> Track(N)
```
This is **one frame of added end-to-end latency** in exchange for hiding the GPU
time entirely. For a 30 FPS pose stream that is usually acceptable; for a
control loop it may not be. **State the trade-off to the user explicitly.**

`Jetson-ORB-SLAM3` reports exactly this: stereo-inertial **14.4 FPS baseline →
28.0 FPS with pipelining** on Orin Nano. The pipelining, not the kernels, is
where most of their end-to-end gain came from.

**Sync budget [M]:** one `cudaDeviceSynchronize` = 26 µs. Budget **≤5 sync
points per frame** (0.13 ms). The measured per-stage-sync GPU-ORB total
(0.638 ms at 640×480) already assumes 6 syncs; the single-sync variant is
0.888 ms because the stages then serialise differently. **Use ~2 syncs: one
after D2H of keypoints, one after D2H of descriptors** — or one, with both in
the same `cudaMemcpyAsync` batch.

---

## A10 — GPU memory strategy under WSL2

### Pinned memory: real, but only 1.2–1.5× here [M]

| Transfer | pageable | pinned | ratio |
|---|---|---|---|
| 64 KiB (2000 descriptors) | 0.028 ms | 0.029 ms | **1.0×** (latency-bound) |
| 640×480 gray (300 KB) | 0.055 | 0.049 | 1.12× |
| 1280×720 gray (0.9 MB) | 0.123 | 0.096 | 1.28× |
| 1920×1080 gray (2.0 MB) | 0.237 | 0.194 | 1.22× |
| 1920×1080 BGR (5.9 MB) | 0.601 | 0.538 | 1.12× |
| D2H 1920×1080 gray | 0.289 | 0.190 | **1.52×** |

On bare-metal PCIe 4.0 ×16, pageable ≈ 6 GB/s and pinned ≈ 24 GB/s — a 3–4×
gap. **Here the gap is 1.1–1.5×**, confirming the degraded pinned-memory DMA
semantics under WSL2's WDDM GPU-paravirtualisation. `cudaHostAlloc` is still
worth using (especially for D2H), but **do not promise more than 1.2–1.5×**, and
do not build a design whose viability depends on zero-copy.

Small transfers are **entirely latency-bound**: 64 KiB costs the same as 4 KiB.
So **batch aggressively** — one `cudaMemcpyAsync` for all levels' keypoints plus
all descriptors beats 16 small ones by ~0.4 ms.

### What stays GPU-resident vs. what must come back [V]

| Data | Where it is needed | Decision |
|---|---|---|
| image pyramid `mvImagePyramid` | `ComputeKeyPointsOctTree`, `computeDescriptors`, **and `ComputeStereoMatches` sub-pixel SAD** (`src/Frame.cc:908`, `:923`) | **GPU-resident**, *except* stereo, which reads `mpORBextractorLeft->mvImagePyramid[octave]` on the CPU. For stereo, either keep a CPU mirror or move the SAD to GPU too. |
| blurred pyramid | descriptors only | **GPU-resident, never comes back** |
| raw FAST corners (x, y, response, level) | **`DistributeOctTree` — CPU only (F4)** | **must D2H.** 8 k × 12 B (short2 + float) = 96 KB → 0.03 ms. Use a packed 12-byte struct, **not** `cv::KeyPoint` (28 B). |
| final keypoints after octree | `Frame::mvKeys`, `UndistortKeyPoints`, `AssignFeaturesToGrid` | **CPU** (they are produced on CPU) |
| descriptors | **DBoW2** `Frame::ComputeBoW`, **every `ORBmatcher` call**, `MapPoint::ComputeDistinctiveDescriptors` — all CPU | **must D2H.** 1250 × 32 B = 40 KB → 0.03 ms |
| map points / poses | g2o, CPU | **never GPU** |

**Ordering problem [V]:** descriptors can only be computed *after* the octree has
selected keypoints, and the octree runs on the CPU. So a naive port has
`GPU(FAST) → D2H → CPU(octree) → H2D → GPU(BRIEF) → D2H` = **4 transfers and 4
syncs**. Two ways out:

1. **Compute descriptors for ALL FAST corners on the GPU, then discard.** At 8 k
   corners vs. 1250 kept, that is 6.4× wasted BRIEF work — but **[M]** BRIEF at
   8000 keypoints was measured at only 0.156 ms, so the waste costs ~0.13 ms and
   **saves an entire H2D+sync round trip (~0.06 ms + 0.026 ms)**. Roughly a wash,
   but it halves the sync count and simplifies the pipeline. **Recommended.**
2. Keep two streams and overlap the CPU octree of level *i* with the GPU BRIEF of
   level *i−1*. More complex; only worth it if (1) measures badly.

### Buffer reuse

Allocate once in the `ORBextractorCUDA` constructor and never again:
`cv::cuda::GpuMat mPyr[8], mBlur[8]`, `short2* dCorners`, `float* dResp`,
`int* dCellCount`, `uchar* dDesc`, plus **pinned** host staging
`cudaHostAlloc(&hCorners, …, cudaHostAllocDefault)`. `cudaMalloc` during
steady-state is a synchronising call and will show up directly in P99.

---

## A11 — `Optimizer::PoseOptimization()` — the biggest single CPU target

### What it actually does [V] `src/Optimizer.cc:814-1114`

```cpp
:818  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
:820  g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
:822  g2o::OptimizationAlgorithmLevenberg* solver = new ...Levenberg(solver_ptr);
:853  const float deltaMono   = sqrt(5.991);
:854  const float deltaStereo = sqrt(7.815);
:1001 const float chi2Mono[4]   = {5.991,5.991,5.991,5.991};
:1002 const float chi2Stereo[4] = {7.815,7.815,7.815,7.815};
:1003 const int   its[4]        = {10,10,10,10};
:1006 for(size_t it=0; it<4; it++) {
:1007     Tcw = pFrame->GetPose();              // reset to the PRE-optimization pose each round
:1008     vSE3->setEstimate(...);
:1011     optimizer.initializeOptimization(0);
:1012     optimizer.optimize(its[it]);
          ... chi2 classification, e->setLevel(0/1) ...
:1037     if(it==2) e->setRobustKernel(0);      // Huber removed for the last round only
:1102     if(optimizer.edges().size()<10) break;
```

Facts:
- **4 rounds × 10 LM iterations = 40 iterations**, and the vertex estimate is
  **reset to the un-optimized pose at the start of every round** (`:1007-1008`).
  So rounds 2–4 re-converge from scratch with a different inlier set.
- Only the pose is free; all 3D points are fixed (`OnlyPose` edge types).
- The solver is **dense** — correct, since there is exactly one 6-DoF vertex.
- The robust kernel is Huber with δ=√5.991 (mono) / √7.815 (stereo), removed
  after round 3 (`it==2`).
- One `new` per edge and one `new g2o::RobustKernelHuber` per edge
  (`:879-881`, `:911-913`, …) — **[E] ~2×N heap allocations per call**, N≈300–700.
  At 2 calls/frame that is **1200–2800 mallocs per frame just for g2o objects.**

### And it runs twice per frame [V] — **F1**

`src/Tracking.cc:2827` (TrackReferenceKeyFrame) or `:2991`
(TrackWithMotionModel), then `:3053` (TrackLocalMap). **80 LM iterations per
normal frame.**

Relocalization runs up to 3 more (`src/Tracking.cc:3842`, `:3859`, `:3874`) but
is off the normal path.

### Cost estimate [E]

Each LM iteration linearises every active edge (a 2×6 or 3×6 Jacobian through
`GeometricCamera::projectJac`), accumulates a 6×6 `H` and 6×1 `b`, then does a
6×6 Cholesky. With g2o's virtual dispatch and Eigen fixed-size blocks, **[E]
80–200 ns per edge per iteration** on a P-core. At N=500 edges:
`40 × 500 × 140 ns ≈ 2.8 ms` per call → **5.6 ms/frame for the two calls**.
Range: **3–7 ms/frame**, i.e. **9–21 % of the 33.33 ms budget.** Published
ORB-SLAM3 "Pose Opt" figures of 2.5–4 ms per call are consistent with this.

### Safe reductions, ordered by risk

| # | Change | Where | **[E]** gain | Accuracy risk |
|---|---|---|---|---|
| **1** | **`its[4] = {10,5,5,5}`** — round 1 does the real convergence; rounds 2–4 only refine after outlier reclassification and start from the same initial estimate anyway (`:1007`). | `src/Optimizer.cc:1003` | **−35 % → 1.0–2.5 ms/frame** | **Low.** LM on a 6-DoF problem with a good prior converges in 3–6 iterations; 10 is already generous. **[X] validate: compare `mnMatchesInliers` and ATE over the bag.** |
| **2** | **Early exit on converged χ²** — after round `it`, if `nBad` is unchanged from round `it-1`, the inlier set is stable and further rounds are pure re-convergence. Add `if(it>0 && nBad==nBadPrev) break;` before `:1102`. | `src/Optimizer.cc:1099` | **[E] skips 1–2 rounds on 60–80 % of frames → 0.8–2.0 ms** | **Very low** — it only skips rounds that would not change the classification. Not bit-identical (the pose differs in the last digits) but statistically identical. |
| **3** | **Reuse the optimizer across calls.** A `thread_local` `SparseOptimizer` plus pre-allocated edge pools removes 1200–2800 `new`/`delete` per frame. | `src/Optimizer.cc:815-822` + edge construction | **[E] 0.3–0.8 ms** | **None** — pure allocation change, bit-identical. Non-trivial to implement (g2o owns its edges). |
| **4** | **Skip PoseOptimization #1 when the motion model is strong.** If `TrackWithMotionModel`'s `SearchByProjection` returns a high inlier ratio and `mVelocity` is smooth, the pose fed to `TrackLocalMap` barely matters — `TrackLocalMap` re-optimizes with strictly more constraints anyway. Gate on e.g. `nmatches > 150 && frames_since_reloc > 10`. | `src/Tracking.cc:2991` | **[E] 1.5–3.5 ms on the frames where it fires** | **Medium.** `SearchLocalPoints` → `isInFrustum` uses the un-refined pose, so the projection search windows are wider and `SearchByProjection` returns more candidates (slower) and possibly more wrong matches. **Must be measured, not assumed. [X]** |
| 5 | Drop `its` to `{10,10,10}` (3 rounds) | `:1003` and the loop bound `:1006` | −25 % | Low–medium; removes one outlier reclassification pass |

**Do NOT** change `chi2Mono`/`chi2Stereo` (5.991 / 7.815 are the 95 % χ² critical
values for 2 and 3 DoF) or the Huber δ. Those are statistics, not tuning knobs.

### Inertial variants [V]

`Optimizer::PoseInertialOptimizationLastKeyFrame` (`src/Optimizer.cc:4491`) and
`…LastFrame` (`:4875`) use the **same** `int its[4]={10,10,10,10}` (`:4700`,
`:5100`) but a **graduated** chi2 schedule `{12, 7.5, 5.991, 5.991}` (`:4697`)
— i.e. they start permissive and tighten. They add IMU preintegration, bias-walk
and gravity-direction edges, so each iteration is more expensive.
**[E] 4–9 ms/call.** They are selected in `TrackLocalMap` at
`src/Tracking.cc:3067`/`:3072` depending on `mbMapUpdated`. **The same `its[]`
reduction applies, with the same caveat — but for IMU the graduated chi2 means
round 1 is deliberately loose, so cutting it to 5 is riskier. Recommend
`{10,10,5,5}` for the inertial variants.**

---

## A12 — `Tracking::TrackLocalMap()` and its cost drivers

### Structure [V] `src/Tracking.cc:3031-3140`

```
TrackLocalMap()
 ├ :3038 UpdateLocalMap()
 │        ├ :3502 mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints)   // viewer only
 │        ├ :3505 UpdateLocalKeyFrames()   src/Tracking.cc:3539
 │        └ :3506 UpdateLocalPoints()      src/Tracking.cc:3509
 ├ :3039 SearchLocalPoints()               src/Tracking.cc:3425
 └ :3053 PoseOptimization()                ← F1
```

### Cost driver 1 — `UpdateLocalKeyFrames` vote counting [M] — **the largest**

```cpp
// src/Tracking.cc:3552 (and identically at :3575 for the IMU branch)
const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
for(...it=observations.begin(); it!=observations.end(); it++) keyframeCounter[it->first]++;
```
`MapPoint::GetObservations()` (`src/MapPoint.cc:204-209`) returns the
`std::map` **by value** — a full red-black-tree deep copy with one node `malloc`
per observation, for **every matched map point, every frame**.

Measured (replica of the exact loop, real `std::map`/`std::mutex`):

| Scenario | stock `GetObservations()` | patched `AccumulateObservingKeyFrames` | speedup |
|---|---|---|---|
| 700 matched MP × 8 observations | **0.392 ms** | **0.148 ms** | 2.65× |
| 900 matched MP × 15 observations | **0.945 ms** | **0.357 ms** | 2.64× |

**The existing patch's fix is correct and is the single best CPU change in it.**

### Cost driver 2 — `UpdateLocalPoints` gather [M] — measured, and NOT worth changing

```cpp
// src/Tracking.cc:3516
const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
```
`KeyFrame::GetMapPointMatches()` (`src/KeyFrame.cc:367-371`) also returns
**by value** — up to 80 local keyframes × 1250 pointers = 800 KB copied per
frame. I expected this to be significant. **It is not:**

| Scenario | by value | by reference | saving |
|---|---|---|---|
| 60 local KF × 1250 features | **0.0376 ms** | 0.0222 ms | **0.015 ms** |
| 80 local KF × 1250 features | **0.0536 ms** | 0.0321 ms | **0.021 ms** |

`memcpy` of 800 KB at ~20 GB/s is 0.04 ms — exactly what we see. **Returning a
reference would require the caller to hold `mMutexFeatures` across the loop,
introducing a real deadlock surface, for 0.02 ms. Do not do it.** Reporting this
as a measured negative result.

`mvpLocalMapPoints.clear()` then `push_back` without `reserve` (`:3511`,
`:3529`) does cause ~13 geometric reallocations to reach ~10 000 elements.
`mvpLocalMapPoints.reserve(mvpLocalKeyFrames.size()*400)` after the clear is free
and bit-identical. **[E] 0.03–0.08 ms.**

### Cost driver 3 — `Frame::isInFrustum` over the whole local map [V] [E]

`SearchLocalPoints` (`src/Tracking.cc:3449-3467`) calls
`mCurrentFrame.isInFrustum(pMP, 0.5)` for every point in `mvpLocalMapPoints`
that was not already matched. `isInFrustum` (`src/Frame.cc:512-580`) does a
`GetWorldPos()` (locks `mGlobalMutex` + `mMutexPos`), a projection via
`mpCamera->project`, a `GetNormal()` (another lock), a `norm()`, and
`PredictScale`. With `mvpLocalKeyFrames` capped at 80 (`src/Tracking.cc:3609`)
and ~1250 points per KF with heavy overlap, `mvpLocalMapPoints` is typically
**3 000–15 000** points. **[E] 0.3–0.8 ms, dominated by mutex acquisitions**
(3 locks per point → 9 000–45 000 lock/unlock pairs per frame).

**Result-preserving improvement:** `MapPoint` already caches everything
`isInFrustum` needs. Add a single accessor that takes `mMutexPos` **once** and
returns `{mWorldPos, mNormalVector, mfMinDistance, mfMaxDistance}` together,
instead of three separate locked calls. Bit-identical, **[E] 0.15–0.4 ms.**

### Cost driver 4 — `GetFeaturesInArea` [M]

`src/Frame.cc:657-723`, `FRAME_GRID_COLS=64`, `FRAME_GRID_ROWS=48`
(`include/Frame.h:44-45`) → 3072 cells. At 640×480 a cell is 10×10 px. With
`th=3` (RGB-D, `src/Tracking.cc:3471`) and `RadiusByViewingCos` = 2.5–4.0,
the radius is 7.5–12 px × scale → a query spans **9–25 cells**.

The two defects and their measured cost (replica of `Frame.cc:657-723` +
`AssignFeaturesToGrid`, identical results asserted):

| Config | stock | patched | speedup |
|---|---|---|---|
| N=1250, 1500 queries, th=3 | **0.2416 ms** | **0.1406 ms** | 1.72× |
| N=1250, 800 queries, th=3 | 0.1143 ms | 0.0555 ms | 2.06× |
| N=1250, 1500 queries, th=1 | 0.1476 ms | 0.0790 ms | 1.87× |
| N=2000, 2500 queries, th=3 | 0.5802 ms | 0.3575 ms | 1.62× |

**Patch 01 is confirmed sound: 1.6–2.1×, saving 0.06–0.22 ms/frame,
bit-identical.**

### Changes that DO alter results — keep separate

| Change | Effect |
|---|---|
| Cap `mvpLocalKeyFrames` below 80 (`src/Tracking.cc:3609`) | fewer local points → weaker `TrackLocalMap`, lower `mnMatchesInliers`, more keyframes via `c2` (A14) |
| Lower `th` in `SearchLocalPoints` (`src/Tracking.cc:3468-3489`) | smaller search windows → fewer matches after fast motion |
| Skip `isInFrustum` for points whose `mnLastFrameSeen` is old | changes the candidate set |
| Sub-sample `mvpLocalMapPoints` | changes everything |

---

## A13 — Tracking / Mapping decoupling and scheduling

### Where Tracking waits [V]

| Site | Mechanism |
|---|---|
| `src/Tracking.cc:1886` | `unique_lock(pCurrentMap->mMutexMapUpdate)` held for **all** of `Track()` |
| `src/Tracking.cc:3170` | `NeedNewKeyFrame` returns false if `mpLocalMapper->isStopped() \|\| stopRequested()` |
| `src/Tracking.cc:3189` | `bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames()` |
| `src/Tracking.cc:3266` | `mpLocalMapper->InterruptBA()` → `mbAbortBA=true` (`src/LocalMapping.cc:899`) |
| `src/Tracking.cc:3270` | `if(mpLocalMapper->KeyframesInQueue()<3) return true; else return false;` — the only backpressure |
| `src/Tracking.cc:3303` | `if(!mpLocalMapper->SetNotStop(true)) return;` — **silently drops the keyframe** |
| `src/Tracking.cc:3419` | `mpLocalMapper->SetNotStop(false)` |

`LocalMapping::Run` (`src/LocalMapping.cc:64`) calls `SetAcceptKeyFrames(false)`
at the **top of every loop iteration** (`:71`) and `true` only at `:273`, so
`AcceptKeyFrames()` is false for essentially the whole time a keyframe is being
processed.

### Where LoopClosing freezes everything [V]

`LoopClosing::RunGlobalBundleAdjustment` (`src/LoopClosing.cc:2290`):
```
:2306  Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF, false)   // async, off-lock
:2308  (or FullInertialBA(pActiveMap, 7, ...))
:2342  mpLocalMapper->RequestStop();
:2345  while(!mpLocalMapper->isStopped() && !isFinished()) usleep(1000);
:2351  unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);      ← TRACKING BLOCKS HERE
:2355+ spanning-tree correction of every KeyFrame and every MapPoint
```
The BA itself is off-lock and abortable via `mbStopGBA` — good. **The write-back
is not.** It walks the entire spanning tree from `mvpKeyFrameOrigins` and
rewrites every pose and every point, with tracking blocked. **[E] 100–800 ms.**

`MergeLocal` is worse: `src/LoopClosing.cc:1529-1530` takes **two** map mutexes
simultaneously.

### What actually works on THIS machine [M — measured as a normal user in WSL2]

```
sched_setscheduler(SCHED_FIFO, prio 50) -> EPERM  "Operation not permitted"
sched_setscheduler(SCHED_RR,   prio 10) -> EPERM
nice(-10)                               -> EPERM
pthread_setaffinity_np                  -> OK
```
Also: `/sys/devices/system/cpu/cpuN/cpufreq` is **absent**, and
`topology/core_id` is **flattened to 0..9 for 20 logical CPUs**.

**Consequences that must be stated plainly:**

1. **SCHED_FIFO/SCHED_RR are unavailable.** Any plan that says "give the tracking
   thread real-time priority" does not work here. It would need `CAP_SYS_NICE` or
   root — and even then the Windows host scheduler migrates the whole VM, so the
   guarantee would be illusory.
2. **P-core pinning is impossible.** WSL2 does not expose the P/E distinction.
   `pthread_setaffinity_np(cpu 0..19)` works, but you cannot know which of those
   are the 6 P-cores. **A tracking slice landing on an E-core is an unavoidable
   P99 hazard on this box** — an Alder Lake E-core is roughly 0.55–0.7× a P-core
   at these workloads, so a 15 ms frame can become 22–27 ms purely from placement.
   **[E]** This alone probably accounts for a large part of the P95→P99 gap and
   **cannot be fixed in software here.** Report it as a hardware/OS limitation,
   not an ORB-SLAM3 defect.

### What to do instead

**(a) Affinity partitioning (works [M]).** Even without knowing which cores are
P-cores, *separating* the threads helps, because it stops Tracking and
LocalMapping from contending for the same core's L2:

```cpp
// Called from System::System() after the threads are created.
// pthread_setaffinity_np is the ONE scheduling primitive that works in WSL2 [M].
static void pinThread(std::thread& t, std::initializer_list<int> cpus)
{
    cpu_set_t set; CPU_ZERO(&set);
    for(int c : cpus) CPU_SET(c, &set);
    pthread_setaffinity_np(t.native_handle(), sizeof(set), &set);
}
// Tracking runs on the caller's thread (the ROS2 callback), so pin that instead:
//   sched_setaffinity(0, sizeof(set), &set)  with cpus {0,1,2,3}
// mptLocalMapping -> {4,...,11}
// mptLoopClosing  -> {12,...,15}
// OMP team of 4   -> leave unpinned; it must follow whichever thread opened it
```
**[E] 10–25 % P95 improvement, 0 % mean improvement.** It reduces cache
thrashing; it cannot prevent host-level VM migration.

**(b) Bound the Global BA stall — the only real fix for F8.**
The write-back loop at `src/LoopClosing.cc:2355+` is one monolithic critical
section. Two options:

- **Chunked write-back:** release and re-acquire `mMutexMapUpdate` every K
  keyframes. **This changes semantics** — tracking would then run against a
  partially-corrected map for a few frames. ORB-SLAM3's design assumes atomicity
  here, so this is a genuine architectural change, not a tweak. It would bound
  the stall to ~K×(per-KF cost) but risks a visible pose discontinuity.
  **[X] must be validated against ATE.**
- **Accept the stall and make it visible:** report `vdGBA_ms` and a
  "map frozen" flag on a ROS2 topic so downstream consumers can hold their last
  pose instead of consuming a stale one. **Lower risk, no accuracy change,
  and honest.** Recommended first.

**(c) Bound the queue at the source.** `message_filters::sync_policies::ApproximateTime`
with `sync_queue_size: 60` (`config/no_cli_rgbd.yaml`) means that if tracking
falls behind, up to **60 frames (2 seconds)** of backlog accumulate silently and
end-to-end latency grows without bound while `TrackRGBD`'s own timer still looks
fine. **Reduce `sync_queue_size` to 2–5 and drop, rather than queue.** For a
realtime pose this is strictly better: a dropped frame costs you the motion model
for one step; a 2-second-old pose is worse than no pose.

**(d) Never let the thread pools oversubscribe.** `03-thread-governance.patch`.

---

## A14 — Keyframe insertion policy

### Every condition in `Tracking::NeedNewKeyFrame()` [V] `src/Tracking.cc:3146-3296`

| Line | Gate |
|---|---|
| :3148-3157 | IMU, not yet initialised: return true if `mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp >= 0.25` |
| :3159 | `if(mbOnlyTracking) return false;` |
| :3170 | `if(mpLocalMapper->isStopped() \|\| stopRequested()) return false;` |
| :3178 | `if(mCurrentFrame.mnId < mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames) return false;` |
| :3184-3187 | `nMinObs = 3` (2 if `nKFs<=2`); `nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs)` |
| :3190 | `bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames()` |
| :3196-3210 | count `nTrackedClose` / `nNonTrackedClose` over features with `0 < mvDepth[i] < mThDepth` |
| :3214 | `bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70)` |
| :3217-3219 | `thRefRatio = 0.75f`; `0.4f` if `nKFs<2` |
| :3229 | `if(mSensor==MONOCULAR) thRefRatio = 0.9f` |
| :3231 | `if(mpCamera2) thRefRatio = 0.75f` |
| :3233-3239 | IMU_MONOCULAR: `0.75f` if `mnMatchesInliers>350` else `0.90f` |
| :3242 | **c1a** = `mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames` |
| :3244 | **c1b** = `(mnId >= mnLastKeyFrameId + mMinFrames) && bLocalMappingIdle` |
| :3246 | **c1c** = non-mono, non-inertial, and (`mnMatchesInliers < nRefMatches*0.25` **or** `bNeedToInsertClose`) |
| :3248 | **c2** = `(mnMatchesInliers < nRefMatches*thRefRatio \|\| bNeedToInsertClose) && mnMatchesInliers > 15` |
| :3253-3265 | **c3** = inertial, `mTimeStamp - mpLastKeyFrame->mTimeStamp >= 0.5` |
| :3267-3271 | **c4** = IMU_MONOCULAR and `15 < mnMatchesInliers < 75`, or `RECENTLY_LOST` |
| :3273 | **fire if** `((c1a \|\| c1b \|\| c1c) && c2) \|\| c3 \|\| c4` |
| :3276-3294 | if not idle: `InterruptBA()`, then non-mono returns true only if `KeyframesInQueue()<3` |

`mMaxFrames` / `mMinFrames` come from `Camera.fps` — **[V]** `Tracking::newParameterLoader`
(`src/Tracking.cc:535`) / `ParseCamParamFile`. With `Camera.fps: 30`
(`data/orbslam3_d455f_640x480_rgbd.yaml`), **`mMaxFrames = 30`, `mMinFrames = 0`**.

**So at 30 FPS input, `c1b` is `(mnId >= mnLastKeyFrameId + 0) && idle` — i.e.
`c1b` is TRUE on every frame where LocalMapping happens to be idle.** The only
real brake is `c2`. This is why ORB-SLAM3 tends to insert keyframes aggressively
on RGB-D: `bNeedToInsertClose` (`nTrackedClose<100 && nNonTrackedClose>70`) fires
constantly indoors with a D455 at 0.15–6 m.

### Adaptive policy for 30 FPS

```cpp
// src/Tracking.cc:3244 — proposed
// mMinFrames == 0 at 30 FPS (it is set from Camera.fps), so c1b degenerates to
// "LocalMapping is idle" and fires on essentially every frame. Give it a real
// floor so keyframe rate cannot exceed what LocalMapping can absorb.
const int minFramesAdaptive = std::max(mMinFrames, (int)(mMaxFrames/10));   // 3 @ 30fps
const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId + minFramesAdaptive)
                  && bLocalMappingIdle);
```
and tighten the backpressure at `:3270`:
```cpp
// was: if(mpLocalMapper->KeyframesInQueue()<3) return true;
if(mpLocalMapper->KeyframesInQueue() < 2) return true;   // shorter queue = lower p99
```

**Effects [E]:**
- **LocalMapping load** drops roughly in proportion to the keyframe rate. Each
  keyframe costs LocalMapping `ProcessNewKeyFrame` + `MapPointCulling` +
  `CreateNewMapPoints` + `SearchInNeighbors` + `LocalBundleAdjustment` +
  `KeyFrameCulling` (`src/LocalMapping.cc:81-190`), and **[E]** LocalBA alone is
  50–300 ms. Fewer keyframes → fewer LocalBA write-backs → **fewer
  `mMutexMapUpdate` stalls on Tracking → directly better P95.** This is one of
  the few policy changes that improves the tail rather than the mean.
- **Relocalization robustness gets WORSE.** A sparser keyframe graph means
  `KeyFrameDatabase::DetectRelocalizationCandidates` has fewer candidates and
  larger viewpoint gaps between them. Given this fork's heavy investment in
  prior-map localization (`Tracking::ActivatePriorMapForLocalization`,
  `src/Tracking.cc:2377`, and the `bUnanchored` logic at `src/Tracking.cc:2055-2070`),
  **this trade-off matters a lot for this project specifically.**
  **Recommendation: apply the adaptive policy for the mapping runs where latency
  matters, and keep the stock policy when building a map that will later be used
  for relocalization.** Make it a yaml key, not a compile-time change.

---

## A15 — Viewer

### The actual rates [V]

- `Viewer::Viewer` (`src/Viewer.cc:28`): `mT = 1e3/fps;` (`:64` from `Settings`, `:85` from the yaml). At `Camera.fps: 30` → **`mT = 33` ms**.
- `Viewer::Run` (`:162`) calls `cv::waitKey(mT)` (`:339`) each iteration → **the viewer redraws at ~30 Hz, i.e. as often as tracking runs.**
- `mImageViewerScale` (`:59`, `:70`, `:112`) from `Viewer.imageViewScale`, defaulting to `1.0f` (`Settings::readViewer`, `src/Settings.cc`). Not present in `data/orbslam3_d455f_640x480_rgbd.yaml`, so **1.0**.
- `usleep(3000)` in the stop loop (`:375`).

### Does Tracking block on the Viewer? **Yes.** [V]

`FrameDrawer::Update` (`src/FrameDrawer.cc:370`) — **called from `Tracking::Track()` at `src/Tracking.cc:2224`** — takes `unique_lock<mutex> lock(mMutex)` (`:372`; the member is `std::mutex mMutex` at `include/FrameDrawer.h:74`, **not** `mMutexFrame`).

`FrameDrawer::DrawFrame` (`src/FrameDrawer.cc:37`), called from the Viewer
thread, takes the **same** mutex (`:62`) and inside it does
`currentFrame = mCurrentFrame;` (`:84`) — **a full `Frame` deep copy under the
lock**, plus copies of `mvpLocalMap`, `mmProjectPoints`, `mmMatchedInImage`.

So per frame with the viewer enabled:
- Tracking pays one `Frame` deep copy (`src/FrameDrawer.cc:392`) — **[M] 0.070 ms**
- Tracking may block waiting for the Viewer's copy — **[E] 0.05–0.3 ms of lock wait**
- Tracking pays `mvpLocalMap = pTracker->GetLocalMapMPS()` — a vector copy of ~10 000 pointers
- Tracking pays `mmMatchedInImage[pMP->mnId] = …` — one red-black-tree insert per matched point (`:425`)
- Plus, in `SearchLocalPoints`, `mCurrentFrame.mmProjectPoints[pMP->mnId] = …` (`src/Tracking.cc:3474`) — one insert per **visible** local map point, 1000–5000 per frame

The `Frame` copy constructor (`src/Frame.cc:55-97`) clones `mDescriptors` and
`mDescriptorsRight` and copies all `FRAME_GRID_COLS*FRAME_GRID_ROWS = 64*48 =
3072` grid-cell vectors (`:75-81`).

### Headless benefit [M] + [E]

| Item | Cost with viewer | Headless |
|---|---|---|
| `FrameDrawer::Update` Frame deep copy | 0.070 ms **[M]** | 0 |
| `mvpLocalMap` copy + `mmMatchedInImage` inserts | **[E]** 0.05–0.15 ms | 0 |
| `mmProjectPoints` inserts in `SearchLocalPoints` | **[E]** 0.05–0.20 ms | 0 |
| lock wait on `FrameDrawer::mMutex` | **[E]** 0.05–0.30 ms, **and it is a P99 contributor** | 0 |
| Pangolin GL thread + `MapDrawer` | 5–15 % of one core, competing for the same 20 CPUs | 0 |
| **total on the tracking thread** | **[E] 0.2–0.7 ms mean, more in the tail** | **0** |

**This is real but modest** — I am explicitly correcting the existing patch's
comment, which implies a much larger win. The genuine benefit is in the **tail**
(lock contention + one fewer CPU-hungry thread), not the mean.

### How to disable [V]

- `System` constructor argument `bUseViewer` (`src/System.h` / `src/System.cc:42, 229-236`). **The ROS2 node already passes `false`** (`rgbd_node.cpp:275-279`), driven by `visualization.enabled: false` in `config/no_cli_rgbd.yaml`. **So this project is already headless** and the `if(mpViewer)` guards in the existing patch are pure win with zero behaviour change.
- The `Viewer.*` yaml keys (`Viewer.KeyFrameSize`, …, `Viewer.imageViewScale`) are read by `Settings::readViewer` **regardless** of `bUseViewer`, and `readViewer` marks most of them `required` — so **they must stay in the yaml even headless** or `Settings` throws.

---

## A16 — Parameter profiles

### Real key names [V]

`Settings::readORB` (`src/Settings.cc`):
```
ORBextractor.nFeatures   ORBextractor.scaleFactor   ORBextractor.nLevels
ORBextractor.iniThFAST   ORBextractor.minThFAST
```
`Settings::readImageInfo` (`src/Settings.cc:355-411`):
```
Camera.width  Camera.height  Camera.newWidth  Camera.newHeight  Camera.fps  Camera.RGB
```
Current values in `data/orbslam3_d455f_640x480_rgbd.yaml` [V]:
```
Camera.width: 640   Camera.height: 480   Camera.fps: 30   Camera.RGB: 1
ORBextractor.nFeatures: 1250      <-- note: the project already uses 1250, not 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Stereo.ThDepth: 40.0   Stereo.b: 0.0745   RGBD.DepthMapFactor: 1000.0
```

### Profiles

| Key | **Performance** | **Balanced** (recommended) | **Accuracy** | Baseline (current) |
|---|---|---|---|---|
| `ORBextractor.nFeatures` | 800 | **1000** | 1500 | 1250 |
| `ORBextractor.scaleFactor` | 1.25 | **1.2** | 1.2 | 1.2 |
| `ORBextractor.nLevels` | 6 | **8** | 8 | 8 |
| `ORBextractor.iniThFAST` | 25 | **20** | 15 | 20 |
| `ORBextractor.minThFAST` | 9 | **7** | 5 | 7 |
| `Camera.newWidth`/`newHeight` | 480/360 | *(absent)* | *(absent)* | absent |
| `Stereo.ThDepth` | 35 | **40** | 45 | 40 |

**Reasoning, not guessing:**
- `nFeatures` scales `DistributeOctTree`'s output target and every matching loop
  ~linearly. **[M]** the octree measurement at nf=2000 vs nf=1250 with the same
  input corners was 2.684 vs 1.398 ms — but note **[M]** the octree's cost is
  driven by *input corners*, not `nFeatures`, so lowering `nFeatures` helps the
  *matching* and *PoseOptimization* stages (fewer edges → F1 scales directly)
  far more than it helps extraction.
- `iniThFAST` **is** the input-corner lever, and therefore the real octree lever
  (**F4**): raising 20 → 25 cuts corner count substantially. **[E] 20–35 % fewer
  corners → 0.3–0.5 ms off `DistributeOctTree`.** Risk: more cells fall through
  to the `minThFAST` fallback (`src/ORBextractor.cc:838-841`), which costs a
  *second* FAST pass on those cells — so raising `iniThFAST` too far can make
  extraction **slower**, not faster. **[X] must be measured.**
- `nLevels` 8 → 6 removes the two smallest levels (area `1.2⁻¹⁰` and `1.2⁻¹²` ≈
  16 % and 11 % of base) — cheap to compute but they carry the scale invariance
  that `PredictScale`/`isInFrustum` relies on. **Only for the Performance profile.**
- `scaleFactor` 1.2 → 1.25 shrinks total pyramid area from 3.10× base to ~2.78×.
  **[E] −10 % extraction.** Coarser scale quantisation hurts `PredictScale`.

**Do not treat these as the answer.** The core of this report is source-level;
the profiles are a fallback lever, and per the brief they are one section, not
the plan.

---

## A17 — Resolution strategy

### `Settings` DOES support resizing, with automatic intrinsics rescaling [V]

`Settings::readImageInfo` (`src/Settings.cc:355-411`):
```cpp
newImSize_ = originalImSize_;
int newHeigh = readParameter<int>(fSettings,"Camera.newHeight",found,false);
if(found){
    bNeedToResize1_ = true;  newImSize_.height = newHeigh;
    if(!bNeedToRectify_){
        float scaleRowFactor = (float)newImSize_.height/(float)originalImSize_.height;
        calibration1_->setParameter(calibration1_->getParameter(1)*scaleRowFactor, 1);  // fy
        calibration1_->setParameter(calibration1_->getParameter(3)*scaleRowFactor, 3);  // cy
        ...
    }
}
// ... and symmetrically for Camera.newWidth -> fx (index 0) and cx (index 2)
```
and the resize is applied in `System::TrackRGBD` (`src/System.cc:340-344`),
`TrackStereo` (`:263-264`) and `TrackMonocular` (`:417`).

**So `fx, fy, cx, cy` are rescaled automatically** (parameter indices 0,1,2,3 of
`Pinhole`). Two caveats **[V]**:
- Only when `!bNeedToRectify_`. If stereo rectification is on, `Settings`
  recomputes the rectified `K` instead.
- **Distortion coefficients `k1,k2,p1,p2,k3` are NOT rescaled** — correct, since
  they are defined on normalised coordinates and are scale-invariant. ✔
- For `KannalaBrandt`, `mvLappingArea` is also scaled (`src/Settings.cc:395-401`). ✔
- **`Stereo.b` and `RGBD.DepthMapFactor` are unaffected** (metric, not pixel).
  But `mbf = fx * b` is recomputed from the scaled `fx`, so `Stereo.ThDepth` (a
  multiple of `mb = mbf/fx = b`) stays correct. ✔

### Scaling estimates

ORB cost is ~linear in pixel count, and the pyramid multiplies base area by
`Σ 1.2⁻²ⁱ` for i=0..7 = **3.10×**.

| Resolution | pixels | rel. to 640×480 | **[E]** ORB extract (CPU, nf=1250) | **[M]** GPU ORB stages | **[M]** octree (unchanged nf) |
|---|---|---|---|---|---|
| 1920×1080 | 2 073 600 | 6.75× | **20–40 ms** — hopeless on CPU | 1.186 ms | grows with corner count, **[E]** 4–9 ms |
| 1280×720 | 921 600 | 3.00× | **12–22 ms** | 0.841 ms | **[E]** 2.5–5 ms |
| 960×540 | 518 400 | 1.69× | **7–14 ms** | ~0.73 ms **[E]** | **[E]** 1.8–3 ms |
| **640×480** | **307 200** | **1.00×** | **5–9 ms** | **0.638 ms** | **1.40 ms [M]** |

**Two things the linear model gets wrong, and you must say so:**

1. **`DistributeOctTree` does not scale with resolution — it scales with
   corner count [M].** More pixels at a fixed `iniThFAST` means more corners,
   which is superlinear in cost relative to the pixel count only if corner
   *density* also rises. In practice it roughly tracks pixel count, so
   1920×1080 gives ~6.75× the corners → **[M interpolated] ~9 ms of pure CPU
   octree**. At 1080p, **the octree alone eats a quarter of the 33.33 ms budget
   regardless of what the GPU does.** This is the strongest argument for staying
   at 640×480 or 960×540 on this hardware.
2. **Matching and `PoseOptimization` scale with `nFeatures`, not resolution.**
   If you raise resolution but keep `nFeatures: 1250`, the back half of the
   pipeline is unchanged. So resolution is almost purely an *extraction* lever.

**Recommendation:** keep **640×480** (the D455's native rate-friendly mode and
what the calibration in `data/orbslam3_d455f_640x480_rgbd.yaml` is for). If a
higher-resolution sensor stream is unavoidable, use
`Camera.newWidth: 640 / Camera.newHeight: 480` rather than feeding full
resolution — `Settings` handles the intrinsics correctly, and `cv::resize` in
`System::TrackRGBD` costs **[E] 0.3–0.8 ms**, far less than the extraction it
saves. Note that `cv::resize` on the **depth** image (`src/System.cc:344`) uses
the default `INTER_LINEAR`, which **interpolates across depth discontinuities and
invents wrong depths at object edges** — for depth you want `INTER_NEAREST`.
**That is a latent correctness bug in the resize path** (only reachable if
`Camera.newWidth`/`newHeight` are set, which this project does not currently do).

---

## A18 — GPU ORB-SLAM prior art: what transfers here

| Work | What went to GPU | What stayed on CPU | Reported | What transfers |
|---|---|---|---|---|
| **yunchih / ORB-SLAM2-GPU2016-final** ([site](https://yunchih.github.io/ORB-SLAM2-GPU2016-final/), [repo](https://github.com/yunchih/ORB-SLAM2-GPU2016-final)) | FAST corner detection (tiled, one CUDA block per tile), Gaussian filter, ORB descriptor computation | Everything else, incl. keypoint distribution | **5.98 → 14.42 FPS** live; 0.166 → 0.068 s/frame on Xeon E3-1231 + GTX 760 | **The tiled FAST decomposition is exactly the structure ORB-SLAM3's 35×35 cell loop needs (A7).** Their "keep the GPU busy while the CPU overlaps" is A9. Note their CPU baseline was very weak (6 FPS), so the 2.4× is the honest number, not the FPS jump. |
| **thien94 / ORB_SLAM2_CUDA** ([repo](https://github.com/thien94/ORB_SLAM2_CUDA)) | inherits yunchih | — | Jetson TX1, adds ROS publishers | Mostly an integration/ROS fork. Little algorithmic transfer. |
| **nathantsoi / ORB-SLAM2-GPU-RGBD** ([repo](https://github.com/nathantsoi/ORB-SLAM2-GPU-RGBD)) | yunchih's mono GPU ORB, extended to RGB-D | — | **18–20 FPS on Jetson TX2** at max clock | Confirms the RGB-D path benefits from the same extraction port. TX2 ≈ 1.3 TFLOPS vs this 3080 Ti Laptop's ~20 TFLOPS, so their bottleneck profile is not ours. |
| **FastTrack (SFU-RSL, IROS 2025)** ([repo](https://github.com/sfu-rsl/FastTrack)) | **ORB extraction + stereo feature matching + local map tracking** — all three, as CUDA kernels | pose optimization, mapping | **up to 2.8× on desktop (i7-12700K + RTX 3090), 2.7× on Xavier NX** | **Most directly comparable — same CPU generation as ours.** Their 2.8× on an RTX 3090 (2× this GPU) bounds what we can expect here: **≤2.5×, and only on the tracking front end.** Their inclusion of *stereo matching* on GPU matches my A6 finding that `ComputeStereoMatches`'s `cv::norm` SAD is the real stereo cost — **not** the Hamming distance. |
| **Jetson-ORB-SLAM3** ([arXiv 2608.17874](https://arxiv.org/abs/2608.17874)) | scale pyramid, FAST, **multi-scale NMS**, IC orientation, steered rBRIEF | **explicitly retains "the reference grid-based homogenization and corner-score ordering"** | **94.7 % exact keypoint agreement, 99.9 % descriptor bit agreement** (mean Hamming 0.25/256). Orin Nano: mono-inertial 32 FPS; **stereo-inertial 14.4 → 28.0 FPS with pipelining**. ORB extraction = 13.7 ms of 35.4 ms (mono), 38.9 of 76.6 (stereo) | **The single most transferable result.** It proves (a) the homogenization can be preserved while GPU-accelerating everything around it — exactly the F4-constrained design in A7; (b) residual differences are floating-point tie-breaking, not algorithmic; (c) **pipelining (A9) doubled their stereo FPS, more than the kernels did.** |
| **ACM SPAA 2023 brief announcement** ([dl.acm.org](https://dl.acm.org/doi/abs/10.1145/3558481.3591310)) | optimized GPU feature extraction for ORB-SLAM | — | up to 3× over prior GPU methods | Confirms ~3× is the ceiling for extraction alone; consistent with my A7 estimate of 2–4×. |

**Synthesis — what actually transfers to this repo:**
1. **Port extraction, not matching.** Every serious effort ports FAST + blur +
   orientation + BRIEF. **None ports the grid-constrained descriptor matching**,
   which independently confirms F5.
2. **Keep the octree on CPU and preserve it.** Jetson-ORB-SLAM3 explicitly does
   this and gets 94.7 % keypoint agreement. Anyone who replaces it
   (`cv::cuda::ORB`'s global response sort) is changing the algorithm.
3. **Pipelining is worth as much as the kernels.** 14.4 → 28.0 FPS. This is A9,
   and it costs one frame of latency.
4. **Expect 2–3×, not 10×.** FastTrack on an RTX 3090 got 2.8×. We have ~50 % of
   that GPU. **[E] 1.8–2.5× on the tracking front end is the realistic target
   here.**

---

## A19 — Latency budget: RTX 3080 Ti Laptop + i9-12900HK + WSL2

**Configuration: RGB-D, 640×480, `nFeatures: 1250`, headless, `Camera.fps: 30`
(the project's actual `data/orbslam3_d455f_640x480_rgbd.yaml`).**

Every row is tagged. Ranges are honest; where I could measure, I measured.

### Stage table — per frame, tracking thread

| Stage | Source | Stock | After patches 01+02+all | After Phase 1–4 (A20) | GPU-ORB (Phase 6) | Tag |
|---|---|---|---|---|---|---|
| ROS2 image conversion (`imageToBgr`/`imageToDepth`) | `rgbd_node.cpp:400-408` | 0.2–0.5 | 0.2–0.5 | 0.2–0.5 | 0.2–0.5 | [E] |
| `System::TrackRGBD` unconditional clone ×2 | `System.cc:336-337` | **0.069** | 0.069 | **0** | 0 | **[M]** |
| `cvtColor` BGR→GRAY | `Tracking.cc:1528` | 0.15–0.30 | " | " | " | [E] |
| depth `convertTo(CV_32F)` | `Tracking.cc:1543` | 0.20–0.40 | " | " | " | [E] |
| `ComputePyramid` | `ORBextractor.cc:1170` | 0.3–0.8 | 0.3–0.8 | 0.3–0.8 | *(GPU)* | [E] |
| FAST over cells ×8 | `ORBextractor.cc:826` | 2.0–4.5 | **0.6–1.6** (4 threads) | 0.6–1.6 | *(GPU)* | [E]/[M] |
| **`DistributeOctTree` ×8** | `ORBextractor.cc:874` | **1.40** | **0.45** (4 threads) | **0.42** (+patch 04) | **0.42** ← irreducible | **[M]** |
| `computeOrientation` | `ORBextractor.cc:901` | 0.15–0.4 | 0.05–0.13 | " | *(GPU)* | [E] |
| blur + `computeDescriptors` ×8 | `ORBextractor.cc:1123` | 1.2–3.0 | **0.4–1.0** | " | *(GPU)* | [E] |
| **GPU ORB total (replaces the 5 rows above except octree)** | — | — | — | — | **0.89 [M] + 0.05 D2H** | **[M]** |
| `UndistortKeyPoints` | `Frame.cc:235` | 0.10–0.25 | " | " | " | [E] |
| `ComputeStereoFromRGBD` | `Frame.cc:237` | 0.05–0.15 | " | " | " | [E] |
| `AssignFeaturesToGrid` | `Frame.cc:385` | 0.03–0.08 | " | " | " | [E] |
| `UpdateLastFrame` (temporal MPs) | `Tracking.cc:2942` | 0.2–0.6 | " | " | " | [E] |
| `SearchByProjection(F,LastFrame)` Hamming | `ORBmatcher.cc:1761` | 0.051 | **0.029** | 0.029 | 0.029 | **[M]** |
| **`PoseOptimization` #1** | `Tracking.cc:2991` | **1.5–3.5** | 1.5–3.5 | **0.9–2.2** (its={10,5,5,5}) | 0.9–2.2 | [E] |
| `UpdateLocalKeyFrames` | `Tracking.cc:3539` | **0.39–0.94** | **0.15–0.36** | 0.15–0.36 | " | **[M]** |
| `UpdateLocalPoints` | `Tracking.cc:3509` | 0.038–0.054 | " | " | " | **[M]** |
| `isInFrustum` × local MPs | `Frame.cc:512` | 0.3–0.8 | " | **0.15–0.4** | " | [E] |
| `GetFeaturesInArea` × queries | `Frame.cc:657` | **0.11–0.24** | **0.06–0.14** | " | " | **[M]** |
| `SearchByProjection` local-map Hamming | `ORBmatcher.cc:101` | 0.117–0.274 | **0.060–0.150** | " | " | **[M]** |
| **`PoseOptimization` #2** | `Tracking.cc:3053` | **1.5–3.5** | 1.5–3.5 | **0.9–2.2** | 0.9–2.2 | [E] |
| `FrameDrawer::Update` | `Tracking.cc:2224` | **0.070** | **0** (headless guard) | 0 | 0 | **[M]** |
| bookkeeping (`mmProjectPoints`, outlier sweeps) | `Tracking.cc:3474`, `:2240` | 0.1–0.3 | 0.02–0.05 | " | " | [E] |
| `NeedNewKeyFrame` (`TrackedMapPoints` scan) | `Tracking.cc:3187` | 0.02–0.05 | " | " | " | [E] |
| **TOTAL (typical frame, no keyframe)** | | **~9.0–20.5 ms** | **~5.6–14.5 ms** | **~4.3–11.5 ms** | **~3.3–9.5 ms** | |

### Additions on specific frames

| Event | Frequency | Cost | Tag |
|---|---|---|---|
| `CreateNewKeyFrame` (`Tracking.cc:3298`): `new KeyFrame` + depth sort + up to 100 `new MapPoint` each with `ComputeDistinctiveDescriptors` (O(n²) Hamming) + `UpdateNormalAndDepth` | **[E]** 1 in 5–15 frames at 30 FPS with the stock policy | **+2–6 ms** | [E] |
| `mMutexMapUpdate` wait for LocalBA write-back (`Optimizer.cc:1464-1496`) | once per keyframe | **+2–10 ms** | [E] |
| `Relocalization()` (`Tracking.cc:3691`): BoW query + up to 3 `PoseOptimization` per candidate + PnP | on loss | **+30–200 ms** | [E] |
| `LoopClosing` merge (`LoopClosing.cc:1529`) | rare | **+50–500 ms** | [E] |
| **`RunGlobalBundleAdjustment` write-back (`LoopClosing.cc:2351`)** | per loop closure | **+100–800 ms** ← **F8** | [E] |
| **ROS2 `publishDenseMapIfNeeded` (`rgbd_node.cpp:444`)** | **every 2.0 s = 1 frame in 60** | **+150–600 ms** ← **F7** | [E] |
| First OpenMP parallel region (if patch applied without `03`) | frame 1 only | **+10.5 ms** | **[M]** |
| OpenMP with default 20-thread team (if `num_threads` were dropped) | **every frame, ×3 regions** | **+11.4 ms** | **[M]** |

### Verdict

| Metric | Stock | + existing patches | + Phase 1–4 | + GPU ORB + pipelining | Target |
|---|---|---|---|---|---|
| **Mean** | 10–21 ms | 7–16 ms | 5–13 ms | 4–11 ms | 33.33 ms |
| **P50** | ~13 ms | ~9 ms | ~7 ms | ~6 ms | 33.33 ms |
| **P95** | ~22 ms (keyframe frames + LocalBA write-back) | ~17 ms | ~13 ms | ~11 ms | 33.33 ms |
| **P99** | **150–600 ms** (dense-map publish) | **150–600 ms** (unchanged — F7 is in the wrapper) | **~25 ms** (after `05-ros2-latency.patch`) | ~22 ms | 33.33 ms |
| **Worst case** | **100–800 ms** (Global BA, F8) | unchanged | unchanged | unchanged | — |

**Answers to the two questions asked:**

> **Is 33.33 ms mean feasible on this hardware?**
> **Yes — and it already is, today, at 640×480 with `nFeatures: 1250`.**
> Stock mean is **[E] 10–21 ms**, comfortably inside budget. The existing patch
> stack plus `03-thread-governance.patch` takes it to 7–16 ms. **GPU is not
> required for 30 FPS mean at this resolution.** GPU ORB buys headroom for
> 1280×720 or for a second camera, not for 640×480/30.

> **Is 33.33 ms P99 feasible?**
> **P99 — yes, but only after fixing the ROS2 wrapper (F7), and only if no loop
> closes.** Specifically:
> - `05-ros2-latency.patch` is **mandatory**; without it P99 is 150–600 ms and no
>   ORB-SLAM3 work matters.
> - With it, **[E]** P99 lands around 20–25 ms — inside budget, but with little
>   margin, and the margin is eaten by E-core migration that **[M]** cannot be
>   prevented under WSL2.
> - **Worst case is NOT feasible and cannot be made feasible without an
>   architectural change.** `LoopClosing::RunGlobalBundleAdjustment`'s write-back
>   (`src/LoopClosing.cc:2351`) blocks `Tracking::Track()`'s
>   `mMutexMapUpdate` (`src/Tracking.cc:1886`) for **[E] 100–800 ms** on every
>   loop closure. This is by design in ORB-SLAM3. If your requirement is "no
>   frame ever exceeds 33.33 ms", **ORB-SLAM3 cannot meet it** — the honest
>   answer is to (a) run in localization-only mode on a prior map, which this
>   fork already supports (`Tracking::ActivatePriorMapForLocalization`,
>   `src/Tracking.cc:2377`) and which never runs GBA, or (b) accept and expose
>   the stall.

**Uncertainty statement.** The `[E]` ranges above carry roughly **±40 %**
uncertainty because ORB-SLAM3 is not built here. The `[M]` rows carry **±5 %**.
`DistributeOctTree` (1.40 ms), the Hamming costs, the OpenMP barrier costs, the
GPU stage costs and the four deep-copy costs are measured; ORB extraction as a
whole, `PoseOptimization`, `isInFrustum` and all the tail events are estimated.
**The single most valuable next action is A3 Step 1+2 — build with
`REGISTER_TIMES` and percentile reporting — which converts most of the `[E]`
rows to `[M]` in one run.**

---

## A20 — Implementation phases

| Ph | File | Class | Function | Change | Expected gain | Difficulty | Accuracy impact | Risk |
|---|---|---|---|---|---|---|---|---|
| **1** | `src/ORB_SLAM3/CMakeLists.txt`, `include/Tracking.h`, `src/Tracking.cc:263` | `Tracking` | `PrintTimeStats` | Build `-DORBSLAM3_REGISTER_TIMES=ON`; add the `reportPct` percentile helper (A3 Step 2) | **0 ms** — but converts the `[E]` half of A19 to `[M]` | Trivial | None | None. **Do this first.** |
| **1** | `perf_patches/` | — | — | Apply `orbslam3-perf-all.patch` **together with** `03-thread-governance.patch`. Delete/ignore the corrupt `02-…patch`. | **[M]** 0.5–1.2 ms mean, plus avoids the 10.5 ms first-frame and 11.4 ms/frame OMP traps | Trivial | **None** — all hunks bit-identical except OMP scheduling | Low. Verified: all three patches apply and stack cleanly. |
| **1** | `orbslam3_ros2/src/rgbd_node.cpp:443`, `CMakeLists.txt` | `RgbdNode` | `publishDenseMapIfNeeded`, `publishMapPointsIfNeeded` | `05-ros2-latency.patch` — move both to a drop-on-busy background thread; force `CMAKE_BUILD_TYPE=Release` + `-march=native` | **[E] P99: 150–600 ms → ~25 ms** | Low | **None** — publishing only | Low. Needs a join in `~RgbdNode` (included in the patch). **Highest-leverage single change in this report.** |
| **2** | `orbslam3_ros2/config/no_cli_rgbd.yaml` | — | `runtime.sync_queue_size` | 60 → 3, and drop rather than queue | **[E]** bounds end-to-end latency; converts unbounded lag into visible frame drops | Trivial | None (frames dropped, not corrupted) | Low. Makes overload *visible*, which some will read as a regression. |
| **2** | `src/System.cc:336-337` | `System` | `TrackRGBD`, `TrackStereo`, `TrackMonocular` | Clone only when `settings_->needToResize()`; otherwise pass `im` through | **[M] 0.069 ms/frame** | Trivial | None | Low. Must confirm no downstream code mutates `im` — `GrabImageRGBD` does `cvtColor(mImGray,mImGray,…)` in-place, so `mImGray` must not alias the ROS message buffer. **Safe because `rgbd_node.cpp:405` already `.clone()`s.** |
| **3** | `src/Optimizer.cc:1003` | `Optimizer` | `PoseOptimization` | `its[4] = {10,5,5,5}` | **[E] 1.0–2.5 ms/frame** (F1, ×2 calls) | Low | **Low, but must be measured.** Compare `mnMatchesInliers` and ATE. | **Medium** — this is the first change in this list that can degrade accuracy. Gate behind a yaml key. |
| **3** | `src/Optimizer.cc:1099` | `Optimizer` | `PoseOptimization` | Early-exit when `nBad` is unchanged between rounds | **[E] 0.8–2.0 ms/frame** | Low | Very low — skips only rounds that would not reclassify | Low |
| **3** | `src/ORBextractor.cc:489-504` | `ExtractorNode` | `DivideNode` | `04-octree-node-reserve.patch` | **[M] 0.08–0.34 ms** | Trivial | **None** — `reserve` is capacity only | None |
| **3** | `src/ORBextractor.cc:796, 824` | `ORBextractor` | `ComputeKeyPointsOctTree` | Hoist `vToDistributeKeys` and `vKeysCell` out of the loops, `.clear()` instead of reconstruct | **[E] 0.15–0.5 ms** | Low | None | Low. Must stay per-thread when the OMP pragma is active — declare inside the parallel region, outside the cell loop. |
| **4** | `src/MapPoint.cc` / `src/Frame.cc:512` | `Frame`, `MapPoint` | `isInFrustum` | One combined locked accessor for `{WorldPos, Normal, minDist, maxDist}` instead of 3 separate locked calls | **[E] 0.15–0.4 ms** | Medium | **None** — same values | Low |
| **4** | `src/Tracking.cc:3511` | `Tracking` | `UpdateLocalPoints` | `mvpLocalMapPoints.reserve(mvpLocalKeyFrames.size()*400)` | **[E] 0.03–0.08 ms** | Trivial | None | None |
| **4** | `src/ORBextractor.cc:1177` | `ORBextractor` | `ComputePyramid` | Persistent `mvPyramidTemp[level]` instead of a fresh `Mat temp` per level per frame | **[E] 0.05–0.15 ms** | Low | **None** — but must preserve the ROI-into-bordered-buffer structure exactly (`:1178`) | **Medium** — get this wrong and descriptors change silently. Validate against a golden keypoint dump. |
| **5** | `src/Frame.cc:849-965` | `Frame` | `ComputeStereoMatches` | OMP-parallelise the `iL` loop with per-thread `vDistIdx`, concatenated deterministically by thread id | **[E] 2–5 ms (stereo only)** | Medium | **None if concatenation order is deterministic**; the median cut at `:969` is order-sensitive | Medium |
| **5** | `src/Frame.cc:923` | `Frame` | `ComputeStereoMatches` | Replace `cv::norm(IL,IR,NORM_L1)` with an inline AVX2 `_mm256_sad_epu8` 11×11 SAD | **[E] 1–3 ms (stereo only)** | Medium | None (identical L1 norm) | Low |
| **5** | `src/Tracking.cc:3244, 3270` | `Tracking` | `NeedNewKeyFrame` | Adaptive `minFramesAdaptive = max(mMinFrames, mMaxFrames/10)`; queue threshold 3 → 2 | **[E] fewer LocalBA write-backs → 2–8 ms off P95** | Low | **Medium — degrades relocalization robustness** (A14). Make it a yaml key, off by default for map-building runs. | Medium |
| **6** | **new** `src/ORBextractorCUDA.cu/.h`, `src/Frame.cc:222` | `ORBextractorCUDA` | pyramid, blur, per-cell FAST, IC angle, rBRIEF | Custom CUDA extractor (A7 option B) feeding the **unmodified** `DistributeOctTree` | **[M]-anchored: 5–9 ms → ~2.4 ms ⇒ 2.5–6.5 ms saved** | **High** (2–4 weeks) | **Low if `maxPerCell` and the per-cell `iniThFAST→minThFAST` fallback are reproduced** (prior art: 94.7 % keypoint / 99.9 % bit agreement). **High if not.** | **High.** Needs sm_86 kernels, pinned staging, and a golden-keypoint regression test vs. the CPU extractor. **Must be gated behind a runtime flag with CPU fallback.** |
| **7** | `src/Tracking.cc` + new pipeline stage | `Tracking` | `GrabImageRGBD` | CUDA-stream pipelining: extract frame N while CPU tracks N-1 (A9) | **[E] hides ~0.9 ms of GPU entirely; prior art doubled stereo FPS** | **High** | **None** — the dependency analysis in A9 shows no false dependency | **High.** Costs **one frame of added end-to-end latency**. Must be an explicit user choice. |
| **8** | `src/LoopClosing.cc:2351` | `LoopClosing` | `RunGlobalBundleAdjustment` | Expose a "map frozen" ROS2 topic + `vdGBA_ms`; **do not** chunk the write-back yet | **0 ms** — makes F8 visible instead of mysterious | Low | None | Low. **The honest first step for F8.** |
| **9** | `src/LoopClosing.cc:2355+` | `LoopClosing` | `RunGlobalBundleAdjustment` | Chunked write-back, releasing `mMutexMapUpdate` every K keyframes | **[E] worst case 100–800 ms → 20–60 ms** | **Very high** | **High — breaks the atomicity ORB-SLAM3 assumes.** Tracking would run against a partially-corrected map. | **Very high.** Only attempt with a full ATE regression suite. **[X]** |

### Suggested order

1. **Phase 1 in full** (profiling + existing patches + `03` + `05`). This is a
   day's work, is low-risk, and **fixes the actual P99 problem**.
2. **Measure.** Everything in A19 tagged `[E]` becomes `[M]`.
3. **Phase 2–4** — cheap, safe, bit-identical or near-identical.
4. **Re-measure. Decide whether GPU is needed at all.** At 640×480 it probably
   is not (see A19 verdict).
5. Phase 5 only if stereo is in scope. Phase 6–7 only if 1280×720 or a second
   camera is in scope. Phase 8 always; Phase 9 probably never.

---

## Files produced by this analysis

| File | Status |
|---|---|
| `perf_patches/AGENT1_ORBSLAM3_REALTIME_REPORT.md` | this report |
| `perf_patches/gen_patches2.py` | generator (literal-substitution, MISS-reporting) for the three new patches |
| `perf_patches/03-thread-governance.patch` | **verified applies** — `System.cc` + `System.h`; OpenMP/OpenCV team size + warm-up (fixes F6) |
| `perf_patches/04-octree-node-reserve.patch` | **verified applies** — `ORBextractor.cc`; `DivideNode` reserve (measured 5–13 %) |
| `perf_patches/05-ros2-latency.patch` | **verified applies** — `rgbd_node.cpp` + `CMakeLists.txt`; background publisher + build flags (fixes F7) |

All three were checked with `patch -p1 --dry-run --forward`, and
`orbslam3-perf-all.patch` → `03` → `04` was verified to **stack cleanly** on a
scratch copy of the tree.

**Nothing under `orbslam_ws/src/` was modified.** All changes are patches.

Micro-benchmark sources used for the `[M]` rows (in the session scratchpad, not
committed): `octree_bench.cpp` (faithful `DistributeOctTree`/`DivideNode`
replica), `gfia_bench.cpp` (faithful `GetFeaturesInArea` + `AssignFeaturesToGrid`
replica, asserts identical results), `localmap_bench.cpp`
(`GetObservations`/`GetMapPointMatches`/`Frame` copy), `clonebench.c`.

---

## Sources (A18)

- [yunchih / ORB-SLAM2-GPU2016-final — project page](https://yunchih.github.io/ORB-SLAM2-GPU2016-final/) and [repo](https://github.com/yunchih/ORB-SLAM2-GPU2016-final)
- [thien94 / ORB_SLAM2_CUDA](https://github.com/thien94/ORB_SLAM2_CUDA)
- [nathantsoi / ORB-SLAM2-GPU-RGBD](https://github.com/nathantsoi/ORB-SLAM2-GPU-RGBD)
- [sfu-rsl / FastTrack — GPU-Accelerated Tracking for ORB-SLAM3 (IROS 2025)](https://github.com/sfu-rsl/FastTrack)
- [Jetson-ORB-SLAM3: Accuracy-Preserving GPU Implementation for Edge Computing Devices (arXiv 2608.17874)](https://arxiv.org/abs/2608.17874)
- [Brief Announcement: Optimized GPU-accelerated Feature Extraction for ORB-SLAM Systems (ACM SPAA 2023)](https://dl.acm.org/doi/abs/10.1145/3558481.3591310)
- OpenCV `4.x` `modules/features2d/src/orb.cpp` and `opencv_contrib` `4.x` `modules/cudafeatures2d/src/orb.cpp` — fetched and compared against `src/ORBextractor.cc:149-408` for the F2 finding.
