# Agent 2 — RTAB-Map Realtime 30 FPS Optimization Analysis

**Target machine (VERIFIED on this host, NOT an RTX 4090):**

| Item | Verified value | How verified |
|---|---|---|
| CPU | Intel Core i9-12900HK (Alder Lake-HX mobile) | `lscpu` → `Model name: 12th Gen Intel(R) Core(TM) i9-12900HK` |
| Cores | 20 logical (`nproc`=20); 6 P + 8 E = 14 physical. WSL2 reports `Core(s) per socket: 10`, `Thread(s) per core: 2` — a virtualized view, P/E cores are **indistinguishable** | `nproc`, `lscpu` |
| Cache | L2 12.5 MiB (10 inst.), L3 24 MiB | `lscpu` |
| AVX-512 | **ABSENT** — `grep -o "avx512[a-z0-9_]*" /proc/cpuinfo` returns nothing | verified |
| RAM | 31 GiB + 8 GiB swap | `free -g` |
| GPU | **NVIDIA GeForce RTX 3080 Ti Laptop GPU**, 16384 MiB, compute cap **8.6**, driver 596.08 | `nvidia-smi --query-gpu=...` |
| CUDA toolkit | 12.4 (`V12.4.131`) at `/usr/local/cuda-12.4/bin/nvcc` → `-arch=sm_86` | `nvcc --version` |
| OS | WSL2, kernel 5.15.167.4-microsoft-standard-WSL2 | system |
| ROS 2 | **none** — `/opt/ros` does not exist; no conda env on this host has `rclcpp`/`rtabmap` | `ls /opt/ros`, `ls ~/miniconda3/envs` (ETRIAvatar, ETRIHeadAvatar, FateAvatar, HRAvatar, MICA, gs, handmesh_etr, onnxtf, openvino, rta, test2022, tracker) |
| OpenCV | **no CUDA build anywhere**: `find / -name "libopencv_cuda*"` → empty. Only `/opt/intel/openvino_2021/opencv` (OpenCV 4.x CPU, OpenVINO 2021 bundle) and two Windows-side ZED SDK OpenCV 3.1.0 trees under `/mnt/c` | verified |
| RTAB-Map build | **not built on this machine** — `find ~ -name "librtabmap_core*"` → empty | verified |

**Source under analysis:** `rtabmap_ws/src/rtabmap` = **RTAB-Map 0.22.1** (`CMakeLists.txt:21-23` → `SET(RTABMAP_MAJOR_VERSION 0) / MINOR 22 / PATCH 1`), plus `rtabmap_ws/src/rtabmap_ros` (ROS 2).

> **Everything in this report was read from source.** Where a number is not from source it is explicitly tagged `[EST]` (estimate with stated reasoning) or `[EXP]` (needs experiment). Because RTAB-Map cannot be run on this host, **no section contains a measurement taken by me.** The only real measurements available come from this project's own prior artifacts and are cited as such.

---

## 🔴 KEY FINDINGS (read first)

### KF-1 — The "30 FPS" question is mis-framed: RTAB-Map's mapping loop is deliberately **1 Hz** by default, and the 30 FPS path is only the odometry node.

`corelib/include/rtabmap/core/Parameters.h:182`
```cpp
RTABMAP_PARAM(Rtabmap, DetectionRate,                float, 1,    "Detection rate (Hz). RTAB-Map will filter input images to satisfy this rate.");
```
This filter is applied at the **input of the mapping thread**, `corelib/src/RtabmapThread.cpp:501-508`:
```cpp
bool ignoreFrame = false;
if(_rate>0.0f)
{
    if((_previousStamp>=0.0 && odomEvent.data().stamp()>_previousStamp && odomEvent.data().stamp() - _previousStamp < 1.0f/_rate) ||
        ((_previousStamp<=0.0 || odomEvent.data().stamp()<=_previousStamp) && _frameRateTimer->getElapsedTime() < 1.0f/_rate))
    {
        ignoreFrame = true;
    }
}
```
and `RtabmapThread.cpp:536-539`:
```cpp
if(ignoreFrame && !_createIntermediateNodes)
{
    return;                      // frame silently discarded, never reaches Rtabmap::process()
}
```
So at a 30 FPS camera with stock settings, **29 of every 30 frames never enter `Rtabmap::process()` at all.** The only thing that must hold 33.33 ms is `Odometry::process()`. Loop closure / Bayes filter / graph optimisation / SQLite are on a 1 Hz budget of 1000 ms, not 33.33 ms.

**Consequence:** "make RTAB-Map 30 FPS" decomposes into two *different* problems with two *different* budgets. Optimising `Rtabmap::process()` for 33 ms is mostly wasted effort unless you deliberately raise `Rtabmap/DetectionRate`.

### KF-2 — Both inter-thread queues are **drop-oldest, never blocking**. A loop-closure stall can *never* backpressure the camera; it silently drops frames instead.

`corelib/src/OdometryThread.cpp:182-188` (odometry input queue, `Odom/ImageBufferSize` default **1**):
```cpp
_dataBuffer.push_back(data);
while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
{
    UDEBUG("Data buffer is full, the oldest data is removed to add the new one.");
    _dataBuffer.erase(_dataBuffer.begin());
    notify = false;
}
```
`corelib/src/RtabmapThread.cpp:571-579` (mapping input queue, `Rtabmap/ImageBufferSize` default **1**):
```cpp
while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
{
    if(_rate > 0.0f)
    {
        ULOGGER_WARN("Data buffer is full, the oldest data is removed to add the new one.");
    }
    _dataBuffer.pop_front();
    notify = false;
}
```
Parameters (`Parameters.h:460` and `:183`):
```cpp
RTABMAP_PARAM(Odom, ImageBufferSize,     unsigned int, 1, "Data buffer size (0 min inf).");
RTABMAP_PARAM(Rtabmap, ImageBufferSize,  unsigned int, 1, "Data buffer size (0 min inf).");
```
**The failure mode is therefore silent frame loss, not latency growth.** Your P99 metric must be *"fraction of frames dropped"*, not just "ms per processed frame" — a system that drops 40 % of frames can still report a beautiful 20 ms mean. Note `0` means **infinite** buffer, which converts frame-drop into unbounded memory growth and latency — never set these to 0 for realtime.

### KF-3 — The dominant super-linear cost as the map grows is a **dense N×N float matrix multiply** in the Bayes filter, where N = |Working Memory|.

`corelib/src/BayesFilter.cpp:313`:
```cpp
cv::Mat prediction = cv::Mat::zeros(ids.size(), ids.size(), CV_32FC1);
```
`corelib/src/BayesFilter.cpp:196`:
```cpp
prior = _prediction * posterior;     // (m,m) X (m,1)
```
This is O(N²) time and O(N²) **memory** per detection iteration. At N=2000 → 16 MB and 4 M MACs (trivial). At N=10000 → **400 MB** and 100 M MACs. This is precisely what `Rtabmap/MemoryThr` and `Rtabmap/TimeThr` exist to bound, and both default to **0 = unbounded**:
```
Parameters.h:180  RTABMAP_PARAM(Rtabmap, TimeThr,   float, 0,  "...(0 means infinity)...");
Parameters.h:181  RTABMAP_PARAM(Rtabmap, MemoryThr, int,   0,  "...(0 means infinity)...");
```
`Bayes/FullPredictionUpdate` (`Parameters.h:352`, default `false`) already avoids regenerating the matrix from scratch, but the **multiply itself is unconditional**. Timing key: `Timing/Posterior_computation/ms`.

### KF-4 — With `Odom/Strategy=5` (ORB-SLAM3), feature extraction is done **twice per mapped frame**, because `OdometryORBSLAM3` never populates `SensorData`'s features.

`Parameters.h:234`:
```cpp
RTABMAP_PARAM(Mem, UseOdomFeatures,             bool, true,     "Use odometry features instead of regenerating them.");
```
`Memory.cpp:4871-4875` — features are re-extracted whenever the odometry did not supply them:
```cpp
if(!_useOdometryFeatures ||
    data.keypoints().empty() ||
    (int)data.keypoints().size() != data.descriptors().rows ||
    (_feature2D->getType() == Feature2D::kFeatureOrbOctree && data.descriptors().empty()))
{
```
`data.setFeatures(...)` is called **only** by F2F and F2M:
```
corelib/src/odometry/OdometryF2F.cpp:302
corelib/src/odometry/OdometryF2M.cpp:338, 1264, 1299
corelib/src/Odometry.cpp:782            (decimation rescale path)
```
`corelib/src/odometry/OdometryORBSLAM3.cpp` contains **no `setFeatures` call** (confirmed by grep over `src/odometry/*.cpp`). It only fills `info->words` / `info->localMap` for visualisation (lines 537-588). So ORB-SLAM3 extracts ~1000 ORB inside `TrackRGBD`, then RTAB-Map's `Memory::createSignature` extracts another 500 (`Kp/MaxFeatures`) of its own. At the default `Rtabmap/DetectionRate=1` that costs one extra extraction per second (tolerable); if you raise DetectionRate to 30 it costs 30/s (not tolerable). **See §B-EXTRA.**

### KF-5 — GPU is, today, *architecturally* reachable but *practically* dead on this machine.

RTAB-Map 0.22.1 exposes 11 distinct GPU toggles (all verified in `Parameters.h`):

| Parameter | Line | Default | Gate |
|---|---|---|---|
| `SURF/GpuVersion` | 283 | `false` | OpenCV CUDA + nonfree |
| `SURF/GpuKeypointsRatio` | 284 | `0.01` | ditto |
| `SIFT/Gpu` | 292 | `false` | **CudaSift** dependency (not OpenCV) |
| `SIFT/GaussianThreshold` | 293 | `2.0` | CudaSift |
| `SIFT/Upscale` | 294 | `false` | CudaSift |
| `FAST/Gpu` | 300 | `false` | OpenCV CUDA |
| `FAST/GpuKeypointsRatio` | 301 | `0.05` | OpenCV CUDA |
| `GFTT/Gpu` | 313 | `false` | OpenCV ≥3 CUDA |
| `ORB/Gpu` | 322 | `false` | OpenCV CUDA |
| `SuperPoint/Cuda` | 344 | `true` | `RTABMAP_TORCH` (libtorch, not OpenCV) |
| `Stereo/Gpu` | 807 | `false` | OpenCV CUDA (`Stereo/OpticalFlow=true`) |
| `Vis/CorFlowGpu` | 729 | `false` | OpenCV CUDA |
| `Vis/CorNNType=4` (`kNNBruteForceGPU`) | 721 | default `1` | OpenCV CUDA |
| `Kp/NNStrategy=4` (`kNNBruteForceGPU`) | 241 | default `1` | OpenCV CUDA |
| `PyMatcher/Cuda` | 735 | `true` | `RTABMAP_PYTHON` (SuperGlue) |

**Every one of the OpenCV-gated entries is inert on this host**, because no `libopencv_cuda*` exists. The only GPU paths that do *not* need a CUDA OpenCV are SuperPoint (libtorch), CudaSift, and SuperGlue/PyMatcher (Python+torch) — i.e. the *deep* paths, which are heavier per-frame than the classical CPU ones they replace.

### KF-6 — There is **no prior RTAB-Map timing data in this project**, but there *is* prior ORB-SLAM3 timing data, on a *different and stronger* machine.

From this project's own artifacts:
- `slam_comparison_report/`, `rtabmap_ws/output/`, `localization_eval/` contain trajectory/node/loop/ATE data only. **No ms, no fps, no `elapsed=` for any RTAB-Map run.** `rtabmap_ws/scripts/run_rtabmap_four_bags.py` never records elapsed time (its ORB-SLAM3 counterpart does), and every `*.db` (which holds the `Statistics` table with the `Timing/*` keys) is gitignored.
- The RTAB-Map runs used essentially **stock defaults**: `run_rtabmap_four_bags.py:34` → `RTAB_ARGS = os.environ.get("RTAB_ARGS", "-d")`, `:35` → `RTAB_ODOM_ARGS` default `""`. So `Rtabmap/DetectionRate=1`, `RGBD/LinearUpdate=0.1`, `RGBD/AngularUpdate=0.1` were all in force.
- ORB-SLAM3 measured `TrackRGBD` **mean 12.4 ms** (RGB-D) / **13.7 ms** (IMU-RGBD) at 640×480@30 on the *original* PC (`_env_specs/README.md:63` → NVIDIA RTX PRO 6000 Blackwell Max-Q); a later run measured **18.7 ms mean / 24.5 ms p95**. ORB-SLAM3 tracking is pure CPU, so the GPU difference is irrelevant; the **CPU** difference is not, and is unknown.
- Dataset used throughout: **Intel RealSense D455f, 640×480 @ ~30 Hz aligned RGB-D** (derived: 1095 color frames / 36.63 s = 29.9 Hz; 4362 / 145.6 s = 30.0 Hz), intrinsics fx 386.92 fy 386.35 cx 322.78 cy 250.28.

This matters enormously for the rest of the report: **the target resolution for this project is already 640×480, not 1920×1080.** Section B11's "reduce resolution" lever is therefore mostly already spent.

### KF-7 — WSL2 removes the two tools you would normally reach for to fix a P99 problem.

- `SCHED_FIFO` / `sched_setscheduler` real-time priority is not usefully available inside WSL2: the entire VM is a set of normal Windows threads scheduled by the Windows kernel. Raising a Linux-side RT priority reorders threads *within* the VM but cannot stop the Windows scheduler from descheduling the whole VM (Windows timer resolution, host GPU/compositor work, Defender scans). **P99 is host-controlled and you cannot bound it from inside.**
- `pthread_setaffinity_np` works on the *virtual* CPU set, but the virtual CPUs are not stably mapped to P-cores vs E-cores, and the P/E distinction is not exposed (`lscpu` shows a flat 10×2 topology). **Pinning the odometry thread cannot guarantee a P-core.** An Alder Lake E-core is roughly 0.5–0.65× a P-core in IPC for this kind of integer/SIMD-light work `[EST]`, so an odometry step that runs 18 ms on a P-core can run 28–36 ms on an E-core — straddling the 33.33 ms line purely from scheduler luck. This is a genuine, unfixable-from-inside P99 hazard and is the single biggest reason I will not certify a hard 33.33 ms P99 on this machine.
- CUDA works through WDDM paravirtualisation (`/dev/dxg`), but **kernel launch and H2D/D2H latency are materially higher than bare metal, and `cudaHostAlloc` pinned memory does not give the usual zero-copy DMA benefit** because the transfer still crosses the VM boundary. Small, frequent transfers (which is exactly what a per-frame 640×480 grayscale upload is — 300 KB) are the worst case.
- ROS 2 DDS over WSL2 loopback and USB camera passthrough via `usbipd-win` both add latency and jitter and are integration risks in their own right.

### KF-8 🔥 — A **stale, git-tracked `Version.h` shadows the CMake-generated one**, permanently disabling g2o, GTSAM, Ceres, libpointmatcher, nonfree SIFT/SURF, SuperPoint/Torch, Python *and ORB-SLAM* — no matter what you install.

`corelib/include/rtabmap/core/Version.h` is a **committed build artifact** (`git ls-files --error-unmatch` succeeds; introduced in commit `a4aff56 "Initial commit: SLAM + SAM-6D ROS2 workspaces (code only)"`). Its contents:
```c
//#define RTABMAP_NONFREE
#define RTABMAP_TORO
//#define RTABMAP_G2O
//#define RTABMAP_GTSAM
//#define RTABMAP_CERES
#define RTABMAP_VERTIGO
//#define RTABMAP_CVSBA
//#define RTABMAP_POINTMATCHER
//#define RTABMAP_CCCORELIB
//#define RTABMAP_OPEN3D
//#define RTABMAP_CUDASIFT
//#define RTABMAP_ORB_SLAM
#define RTABMAP_ORB_OCTREE
//#define RTABMAP_TORCH
//#define RTABMAP_PYTHON
#define RTABMAP_MADGWICK
#define RTABMAP_OCTOMAP
```
CMake generates the *real* one elsewhere — `CMakeLists.txt:1129`:
```cmake
CONFIGURE_FILE(Version.h.in ${CMAKE_CURRENT_BINARY_DIR}/corelib/src/include/${PROJECT_PREFIX}/core/Version.h)
```
but the include search order puts the **source** tree first — `corelib/src/CMakeLists.txt:859-861`:
```cmake
target_include_directories(rtabmap_core PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include;${CMAKE_CURRENT_BINARY_DIR}/include;${PUBLIC_INCLUDE_DIRS};${INCLUDE_DIRS}>"
```
`${CMAKE_CURRENT_SOURCE_DIR}/../include` = `corelib/include` (the stale file) comes **before** `${CMAKE_CURRENT_BINARY_DIR}/include` (the generated one). Upstream RTAB-Map never ships this file in the source tree — it exists here only because the workspace was snapshotted from a built tree.

**Consequences, all verified against the code the defines gate:**

| Disabled define | What it silently costs you |
|---|---|
| `RTABMAP_G2O`, `RTABMAP_GTSAM`, `RTABMAP_CERES` | `Optimizer::create()` (`corelib/src/Optimizer.cpp:79-186`) falls all the way through to `default: optimizer = new OptimizerTORO(parameters);`. **Graph optimisation is TORO-only**, and `Parameters.h:424-426` gives the no-backend branch `Optimizer/Iterations = 100` (vs 20 for GTSAM/g2o). Setting `Optimizer/Strategy=1` or `2` does nothing but print a `UWARN`. |
| `RTABMAP_G2O` (again) | `Parameters.h:490-492` → `OdomF2M/BundleAdjustment` default **0 (disabled)**, and `Parameters.h:730-734` → `Vis/BundleAdjustment` default **0**. Local BA is simply not available. This *lowers* odometry latency but costs accuracy. |
| `RTABMAP_G2O`/`RTABMAP_GTSAM` | `Parameters.h:434-438` → `Optimizer/GravitySigma` default **0.0** — IMU gravity constraints are inert. |
| `RTABMAP_POINTMATCHER` | `Parameters.h:752-756, 763-767, 774-778` → `Icp/Strategy=0` (PCL), `Icp/MaxCorrespondenceDistance=0.05`, `Icp/PointToPlane=false`. `RegistrationIcp::parseParameters` (`RegistrationIcp.cpp:149-153`) demotes any request for strategy 1 with a warning. **ICP is PCL-only.** |
| `RTABMAP_NONFREE` | SIFT/SURF unavailable; `Feature2D::create` (`Features2d.cpp:635-644`) rewrites SURF→SIFT / SURF-variants→GFTT-ORB. |
| `RTABMAP_TORCH`, `RTABMAP_PYTHON`, `RTABMAP_CUDASIFT` | SuperPoint, PyDetector/SuperGlue and CudaSift all unavailable — i.e. **every GPU path that does *not* require a CUDA OpenCV is also off.** |
| **`RTABMAP_ORB_SLAM`** | **`Odom/Strategy=5` is unusable.** `OdometryORBSLAM3::computeTransform` compiles to `UERROR("RTAB-Map is not built with ORB_SLAM support! Select another visual odometry approach."); return t;` (`OdometryORBSLAM3.cpp:592-595`). The hybrid the user wants is compile-time dead in this tree. |

**Fix before anything else** (see `perf_patches/00-remove-stale-version-header.md`). This is Phase 0 of §B20.

---

## B1 — Full per-frame pipeline, file:function:line

### B1.1 Standalone (library/`RtabmapThread`) topology

```
SensorCaptureThread::mainLoop()            corelib/src/SensorCaptureThread.cpp:325
  └─ post(new SensorEvent(data, info))     corelib/src/SensorCaptureThread.cpp:538
       │  [UEventsManager fan-out]
       ▼
OdometryThread::handleEvent()              corelib/src/OdometryThread.cpp:57-68
  └─ OdometryThread::addData()             corelib/src/OdometryThread.cpp:132
       └─ _dataBuffer.push_back  + DROP-OLDEST (Odom/ImageBufferSize=1)   :182-188
       └─ _dataAdded.release()                                            :204
       ▼
OdometryThread::mainLoop()                 corelib/src/OdometryThread.cpp:104
  ├─ getData(data)   [_dataAdded.acquire()]                               :118 / :208-234
  ├─ Odometry::process(data, &info)                                       :122
  └─ post(new OdometryEvent(data, pose, info))                            :127
       ▼
RtabmapThread::handleEvent()               corelib/src/RtabmapThread.cpp:365
  └─ RtabmapThread::addData(OdometryEvent) corelib/src/RtabmapThread.cpp:494
       ├─ Rtabmap/DetectionRate FILTER  → `return` (frame dropped)        :501-539
       └─ _dataBuffer.push_back + DROP-OLDEST (Rtabmap/ImageBufferSize=1) :566-579
       ▼
RtabmapThread::mainLoop() → process()      corelib/src/RtabmapThread.cpp:180 / :465
  └─ Rtabmap::process(data, pose, cov, vel)                               :473
```

### B1.2 `Odometry::process` internals — `corelib/src/Odometry.cpp:303`

| Step | Line | Note |
|---|---|---|
| `UTimer time;` | `:651` | the master odometry timer |
| deskewing (lidar) | `:711` → `info->timeDeskewing = time.ticks();` | no-op for RGB-D |
| **`Odom/ImageDecimation` applied** | `:722-768` | `util2d::decimate(imageRaw, _imageDecimation)` `:744`; depth decimated independently `:726-745`; camera models rescaled `:749` / `:760` |
| `this->computeTransform(decimatedData, guess, info)` | **`:770`** (decimated) or **`:809`** (not decimated) | virtual → strategy subclass |
| keypoints rescaled back to full res | `:774-803` | `kpts[i].pt.x *= _imageDecimation` etc. |
| `data.setFeatures(kpts, keypoints3D, descriptors)` | `:782` | **only on the decimation path** |
| `info->timeEstimation = time.ticks();` | **`:819`** | ← the only end-to-end odometry timing field |
| Kalman/particle filtering | `:925` → `info->timeParticleFiltering = time.ticks();` | only if `Odom/FilteringStrategy != 0` (default 0) |

### B1.3 Strategy dispatch — `Odometry::create`, `corelib/src/Odometry.cpp:66-121`

```cpp
	switch(type)
	{
	case Odometry::kTypeF2M:      odometry = new OdometryF2M(parameters);     break;   // :72
	case Odometry::kTypeF2F:      odometry = new OdometryF2F(parameters);     break;   // :75
	case Odometry::kTypeFovis:    odometry = new OdometryFovis(parameters);   break;   // :78
	case Odometry::kTypeViso2:    odometry = new OdometryViso2(parameters);   break;   // :81
	case Odometry::kTypeDVO:      odometry = new OdometryDVO(parameters);     break;   // :84
	case Odometry::kTypeORBSLAM:                                                       // :87
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 2
		odometry = new OdometryORBSLAM2(parameters);
#else
		odometry = new OdometryORBSLAM3(parameters);
#endif
		break;
	case Odometry::kTypeOkvis:    ...  // :94
	case Odometry::kTypeLOAM:     ...  // :97
	case Odometry::kTypeFLOAM:    ...  // :100
	case Odometry::kTypeMSCKF:    ...  // :103
	case Odometry::kTypeVINS:     ...  // :106
	case Odometry::kTypeOpenVINS: ...  // :109
	case Odometry::kTypeOpen3D:   ...  // :112
	default:
		UERROR("Unknown odometry type %d, using F2M instead...", (int)type);
		odometry = new OdometryF2M(parameters);
		type = Odometry::kTypeF2M;
		break;
	}
```
`Odometry::Type` (`corelib/include/rtabmap/core/Odometry.h:45-60`) — verbatim integer mapping in §B6.

### B1.4 `RegistrationVis::computeTransformationImpl` — `corelib/src/RegistrationVis.cpp:305`

The RGB-D critical path, in execution order:

| # | Step | Line | Cost driver |
|---|---|---|---|
| 1 | `_detectorFrom->generateKeypoints(imageFrom, roi, maskFrom)` | `:455` | only when the "from" signature has no cached words |
| 2 | GPU grayscale + `SensorData::setImageRawGpu` | `:503-540` | `#ifdef HAVE_OPENCV_CUDAOPTFLOW` — **dead here** |
| 3 | **branch on `Vis/CorType`** | `:500` | `==1` optical flow, else feature matching |
| 3a | `_detectorFrom->generateKeypoints3D(fromSignature.sensorData(), kptsFrom)` | `:569` / `:921` | RGB-D → `util3d::generateKeypoints3DDepth` (`Features2d.cpp:1087-1095`) |
| 3b | `cv::calcOpticalFlowPyrLK(...)` | `:677` | `Vis/CorType=1` path only |
| 4 | `_detectorTo->generateKeypoints(...)` | `:821` | new frame detection |
| 5 | `_detectorFrom->generateDescriptors(imageFrom, kptsFrom)` | `:863` | |
| 6 | `_detectorTo->generateDescriptors(imageTo, kptsTo)` | `:888` | |
| 7 | `_detectorTo->generateKeypoints3D(toSignature.sensorData(), kptsTo)` | `:957` | |
| 8 | **guess-window matching** (the default, see below) | `:1011-1380` | |
| 9 | fallback global matching (BF / `VWDictionary` kNN) | `:1381-1500` | `VWDictionary dictionary(_featureParameters);` is constructed **per call** at `:1478` |
| 10 | `util3d::estimateMotion3DTo2D(...)` (PnP RANSAC) | `:1741`, `:1777` | `Vis/EstimationType=1` default |
| 11 | `util3d::estimateMotion3DTo3D(...)` | `:1839` | `Vis/EstimationType=0` |

**Step 8 is the single most important structural fact for the GPU-matching question.** When a motion guess exists (`Odom/GuessMotion=true`, default) and `Vis/CorGuessWinSize>0` (default **40** px), matching is **spatially constrained**, `RegistrationVis.cpp:1011-1016`:
```cpp
			// If guess is set, limit the search of matches using optical flow window size
			bool guessSet = !guess.isIdentity() && !guess.isNull();
			if(guessSet && _guessWinSize > 0 && kptsFrom3D.size() &&
					isCalibrated &&  // needed for projection
					_estimationType != 2) // To make sure we match all features for 2D->2D
			{
```
It projects the "from" 3-D points with `cv::projectPoints` (`:1038`), builds a **2-D rtflann kd-tree** on the projections (`:1079-1081`), does a `radiusSearch` with r = `_guessWinSize` px (`:1090` / `:1230`), then runs a `cv::BFMatcher` over **only the handful of candidates inside each 40 px window** (`:1123-1135`, `:1275-1295`).

### B1.5 `Rtabmap::process` internals — `corelib/src/Rtabmap.cpp:1209`

Timer-delimited stages (each `timer.ticks()`), with the `Statistics` key they publish:

| Stage | `timer.ticks()` line | Statistics key (emitted at) |
|---|---|---|
| `_memory->update(...)` | `:1490` `timeMemoryUpdate` | `kTimingMemory_update()` `:4169` |
| neighbour link refining | `:1657` | `kTimingNeighbor_link_refining()` `:4170` |
| proximity by time | `:1926` | `kTimingProximity_by_time()` `:4171` |
| **Bayes likelihood** | `:2100` | `kTimingLikelihood_computation()` `:4178` |
| **Bayes posterior** | `:2111` | `kTimingPosterior_computation()` `:4179` |
| hypotheses creation / validation | `:2136` / `:2189` | `:4180` / `:4181` |
| joining trash (waits on DB thread) | `:2221` | `kTimingJoining_trash()` `:4513` |
| reactivation (LTM→WM retrieval) | `:2607` | `kTimingReactivation()` `:4175` |
| proximity by space (search/visual/total) | `:2731` / `:2847` / `:3037` | `:4172` / `:4173` / `:4174` |
| add loop closure link | `:3109` | `kTimingAdd_loop_closure_link()` `:4176` |
| **graph optimisation** — `optimizeCurrentMap(...)` at **`:3802`** | `:3994` `timeMapOptimization` | `kTimingMap_optimization()` `:4177` |
| statistics creation | `:4246` | `kTimingStatistics_creation()` `:4510` |
| memory cleanup | `:4348` | `kTimingMemory_cleanup()` `:4515` |
| forgetting (`Rtabmap/TimeThr` transfer) | `:4501` | `kTimingForgetting()` `:4512` |
| **TOTAL** | | `kTimingTotal()` `:4511` |

### B1.6 `Memory::update` / `createSignature` — RGB-D specifics

`Memory::update` — `corelib/src/Memory.cpp:873`:
```
:890  this->preUpdate();                                    → kTimingMemPre_update       :892
:898  Signature * signature = this->createSignature(...)     → kTimingMemSignature_creation :910
:914  this->addSignatureToStm(signature, covariance);
:928  this->rehearsal(signature, stats);                     → kTimingMemRehearsal        :931
```

`Memory::createSignature` — `corelib/src/Memory.cpp:4535`:
```
:4610   PreUpdateThread preUpdateThread(_vwd);     // class at :4519 — VWDictionary::update() on a SEPARATE thread
:4758   → kTimingMemRectification
:4769   if(_parallelized && !isIntermediateNode)   // Kp/Parallelized, default TRUE → starts preUpdateThread
:4871   if(!_useOdometryFeatures || data.keypoints().empty() || ...)   ← KF-4 re-extraction gate
:4887-4930  Mem/ImagePreDecimation applied: util2d::decimate(imageRaw/depth, _imagePreDecimation)
:4931-4937  cvtColor(BGR2GRAY)
:4941-4975  Mem/DepthAsMask → depth used as feature mask (+ util3d::filterFloor)
:5022   keypoints  = _feature2D->generateKeypoints(imageMono, depthMask)  → kTimingMemKeypoints_detection   :5032
:5036   descriptors= _feature2D->generateDescriptors(imageMono, keypoints) → kTimingMemDescriptors_extraction :5038
        ... generateKeypoints3D → kTimingMemKeypoints_3D
        ... kTimingMemJoining_dictionary_update (joins preUpdateThread), kTimingMemAdd_new_words,
            kTimingMemCompressing_data, kTimingMemPost_decimation, kTimingMemOccupancy_grid
```
`Kp/Parallelized` (`Parameters.h:259`, default **true**) is genuine existing parallelism: the FLANN dictionary update overlaps feature extraction.

**RGB-D keypoint 3-D-ization** — `Feature2D::generateKeypoints3D`, `corelib/src/Features2d.cpp:888`:
- RGB-D branch `:1087-1095` → `util3d::generateKeypoints3DDepth(keypoints, depthOrRightRaw, cameraModels, _minDepth, _maxDepth)`
- Stereo single-camera `:895-1000` → `_stereo->computeCorrespondences(...)` (GPU overload at `:954-958` under `#ifdef HAVE_OPENCV_CUDEV`) then `util3d::generateKeypoints3DStereo`
- Stereo multi-camera `:1001-1085` → per-sub-image, same idea

Note `util3d::cloudFromDepth` is **not** on the odometry critical path — it is used for occupancy-grid / cloud export (`RGBD/CreateOccupancyGrid` default **false**, `Parameters.h:387`).

---

## B2 — Critical path vs background: verified from the threading code

### B2.1 The queue-bounding mechanism (the real key names)

| Key | Line | Default | Applies to |
|---|---|---|---|
| `Odom/ImageBufferSize` | `Parameters.h:460` | `1` | `OdometryThread::_dataBufferMaxSize` |
| `Rtabmap/ImageBufferSize` | `Parameters.h:183` | `1` | `RtabmapThread::_dataBufferMaxSize` (`RtabmapThread.cpp:47`) |
| `Rtabmap/DetectionRate` | `Parameters.h:182` | `1` Hz | `RtabmapThread::_rate` (`RtabmapThread.cpp:48`) |

**Overflow policy = DROP OLDEST, never block** — see KF-2 for the verbatim code at `OdometryThread.cpp:182-188` and `RtabmapThread.cpp:571-579`. `0` means *infinite*, not *zero*.

### B2.2 Is loop closure really off the odometry thread? — YES, three times over.

1. **Standalone**: `Rtabmap::process` (which contains every loop-closure and optimiser call) is invoked only from `RtabmapThread::process()`, `corelib/src/RtabmapThread.cpp:473`, inside `RtabmapThread::mainLoop()` (`:180`) — a `UThread` worker distinct from `OdometryThread`.
2. **The queue between them is drop-oldest**, so even an unbounded mapping stall cannot apply backpressure.
3. **ROS 2**: `rtabmap_odom` and `rtabmap_slam` are normally *separate processes*. And inside `rtabmap_slam`, `RtabmapThread` is **not used at all** — `CoreWrapper` runs `Rtabmap::process` from a self-rescheduling zero-period wall timer on its own `MutuallyExclusive` callback group (`rtabmap_slam/src/CoreWrapper.cpp:280-283`), with a try-lock frame-drop in the sync callback (`:1329-1348`). Same decoupling, different mechanism.

Likewise in ROS 2 the odometry itself is off the ROS callback thread: `OdometryROS : public rclcpp::Node, public UThread` (`rtabmap_odom/include/rtabmap_odom/OdometryROS.h:63`), worker started at `rtabmap_odom/src/OdometryROS.cpp:388`, and the callback hands off under a **try-lock with silent drop** — `rtabmap_odom/src/OdometryROS.cpp:461-483`:
```cpp
	if(dataMutex_.lockTry() == 0)
	{
		...
		dataToProcess_ = data;
		dataReady_.release();
		dataMutex_.unlock();
		++processedMsgs_;
	}
	else
	{
		//RCLCPP_WARN(get_logger(), "Dropping image/scan data");
		++droppedMsgs_;
	}
```
Effective queue depth toward VO is **one frame**. Note the warning is commented out — **drops are silent**. `droppedMsgs_` is the counter you must expose.

### B2.3 What *is* still synchronous inside `Rtabmap::process`

Graph optimisation (`:3802`), Bayes posterior (`:2111`), LTM retrieval (`:2607`) and the trash join (`:2221`) all run **inline on the mapping thread**. They do not touch odometry, but they do serialise against each other. The one genuinely asynchronous piece is the database: `class DBDriver : public UThreadNode` (`corelib/include/rtabmap/core/DBDriver.h:62`) with `asyncSave(Signature*)` / `asyncSave(VisualWord*)` (`:78-79`) and `emptyTrashes(bool async)` (`:80`). `Timing/Joining_trash/ms` and `Timing/Emptying_trash/ms` are precisely the points where the mapping thread *waits* for SQLite.

---

## B3 — Profiling: use what already exists

RTAB-Map's existing instrumentation is unusually complete. **Do not add probes until you have used these.**

### B3.1 `OdometryInfo` — `corelib/include/rtabmap/core/OdometryInfo.h`
```
:63  float timeDeskewing;
:64  float timeEstimation;          ← whole-frame VO time (set at Odometry.cpp:819)
:65  float timeParticleFiltering;
:56  float localBundleTime;
:51  int   localMapSize;   :53 int localKeyFrames;   :50 int features;
:61  bool  keyFrameAdded;  :67 double interval;      :75 int memoryUsage;
```
plus the embedded `RegistrationInfo reg` (`corelib/include/rtabmap/core/RegistrationInfo.h`):
```
:79  double totalTime;      :82 int inliers;    :87 int matches;
:83  float inliersRatio;    :84 float inliersMeanDistance;   :85 float inliersDistribution;
:94-100 icpInliersRatio, icpTranslation, icpRotation, icpStructuralComplexity,
        icpStructuralDistribution, icpCorrespondences, icpRMS
```

### B3.2 `Statistics` timing keys — `corelib/include/rtabmap/core/Statistics.h:158-197`

**21 `Timing/*` keys** (`Memory_update, Neighbor_link_refining, Proximity_by_time, Proximity_by_space_search, Proximity_by_space_visual, Proximity_by_space, Cleaning_neighbors, Reactivation, Add_loop_closure_link, Map_optimization, Likelihood_computation, Posterior_computation, Hypotheses_creation, Hypotheses_validation, Statistics_creation, Memory_cleanup, Total, Forgetting, Joining_trash, Emptying_trash, Finalizing_statistics, RAM_estimation`)

**17 `TimingMem/*` keys** (`Pre_update, Signature_creation, Rehearsal, Keypoints_detection, Subpixel, Stereo_correspondences, Descriptors_extraction, Rectification, Keypoints_3D, Keypoints_3D_motion, Joining_dictionary_update, Add_new_words, Compressing_data, Post_decimation, Scan_filtering, Occupancy_grid, Markers_detection`)

All are emitted as `"Timing/<name>/ms"` / `"TimingMem/<name>/ms"` (`Statistics.h:333` documents the format).

### B3.3 How to dump them

```bash
# 1) CSV log files next to the database (the cheapest, most complete option)
ros2 launch rtabmap_launch rtabmap.launch.py \
  args:="-d --Rtabmap/StatisticLogged true --Rtabmap/StatisticLoggedHeaders true \
         --Rtabmap/StatisticLogsBufferedInRAM true --Rtabmap/PublishStats true"
#   → writes LogF_*.txt / LogI_*.txt in Rtabmap/WorkingDirectory

# 2) Live, per-frame odometry timing (LITE topic — see the warning below)
ros2 topic echo /odom_info_lite --field time_estimation
ros2 topic hz  /odom_info_lite

# 3) All Timing/* keys live, from the mapping node
ros2 topic echo /rtabmap/info --field stats_keys
ros2 topic echo /rtabmap/info --field stats_values

# 4) Post-hoc from the database (survives the run; Statistics table)
rtabmap-report --stats <db>        # or open the .db and read the Statistics table
```
Verified keys: `Rtabmap/PublishStats` (`Parameters.h:173`, default **true**), `Rtabmap/StatisticLogged` (`:189`, default **false** ← must enable), `Rtabmap/StatisticLoggedHeaders` (`:190`, default true), `Rtabmap/StatisticLogsBufferedInRAM` (`:188`, default true — keep it, it avoids per-iteration disk writes).

`Info.msg` carries `string[] stats_keys` / `float32[] stats_values` (`rtabmap_msgs/msg/Info.msg:42-43`) — that is where every `Timing/*` key lands.

⚠️ **`/odom_info` (full) is itself a latency source.** `rtabmap_conversions::odomInfoToROS` (`rtabmap_conversions/src/MsgConversion.cpp:1823-1838`) runs per-keypoint and per-3D-point loops (`keypointsToROS`, `points2fToROS`, `points3fToROS`, plus `laserScanToPointCloud2`) whenever any subscriber exists on the full topic. `OdometryROS.cpp:1054-1078` picks the cheap path only when `odomInfoPub_->get_subscription_count()==0`. **Always profile through `/odom_info_lite`.**

### B3.4 The one real gap — and the only probes worth adding

`OdometryInfo` has a single whole-frame number (`timeEstimation`) and `RegistrationInfo` a single `totalTime`. There is **no breakdown inside `RegistrationVis::computeTransformationImpl`**, which is where 80–90 % of odometry time goes. `Memory::createSignature` has a full breakdown (`TimingMem/*`), but that path runs at `Rtabmap/DetectionRate` (1 Hz), not 30 Hz.

So the only justified new instrumentation is a per-stage split of `RegistrationVis` — see `perf_patches/01-registrationvis-stage-timers.diff`. Target table:

| Stage | Existing field | Proposed new field |
|---|---|---|
| decimation | — | `RegistrationInfo::timeDecimation` |
| keypoint detection (from) | `TimingMem/Keypoints_detection/ms` (1 Hz only) | `timeKeypointsFrom` |
| keypoint detection (to) | — | `timeKeypointsTo` |
| descriptor extraction | `TimingMem/Descriptors_extraction/ms` (1 Hz only) | `timeDescriptors` |
| keypoints 3-D | `TimingMem/Keypoints_3D/ms` (1 Hz only) | `timeKeypoints3D` |
| projection + kd-tree build | — | `timeGuessProjection` |
| descriptor matching | — | `timeMatching` |
| PnP RANSAC | — | `timeMotionEstimation` |
| local BA | `OdometryInfo::localBundleTime` ✅ | — |
| **frame total** | `OdometryInfo::timeEstimation` ✅ | — |

---

## B4 — Feature detectors actually available

### B4.1 The real enum — `corelib/include/rtabmap/core/Features2d.h:116-132`
```cpp
	enum Type {kFeatureUndef=-1,
		kFeatureSurf=0,
		kFeatureSift=1,
		kFeatureOrb=2,
		kFeatureFastFreak=3,
		kFeatureFastBrief=4,
		kFeatureGfttFreak=5,
		kFeatureGfttBrief=6,
		kFeatureBrisk=7,
		kFeatureGfttOrb=8,  //new 0.10.11
		kFeatureKaze=9,     //new 0.13.2
		kFeatureOrbOctree=10, //new 0.19.2
		kFeatureSuperPointTorch=11, //new 0.19.7
		kFeatureSurfFreak=12, //new 0.20.4
		kFeatureGfttDaisy=13, //new 0.20.6
		kFeatureSurfDaisy=14,  //new 0.20.6
		kFeaturePyDetector=15}; //new 0.20.8
```
Selected by `Kp/DetectorStrategy` (`Parameters.h:254`, default **8** = GFTT/ORB) and `Vis/FeatureType` (`Parameters.h:704`, default **8**). Both fall back to **6** (GFTT/BRIEF) on `MOBILE_BUILD` (`:256`, `:706`).

### B4.2 Availability on THIS build

| # | Type | Gate (`Features2d.cpp`) | Available here? | GPU member |
|---|---|---|---|---|
| 0 | SURF | `#ifndef RTABMAP_NONFREE` → SURF→SIFT (`:635-639`) | ❌ → rewritten to SIFT, then SIFT also unavailable → see note | `_gpuSurf` (`Features2d.h:283`) |
| 1 | SIFT | needs `RTABMAP_NONFREE` on OpenCV<4.4; OpenCV≥4.4 has SIFT in `features2d` | ⚠️ depends on OpenCV version at build time | `gpu_` + CudaSift (`:307-316`) — needs `RTABMAP_CUDASIFT` ❌ |
| 2 | ORB | none | ✅ | `_gpuOrb` (`:347`) — needs `HAVE_OPENCV_CUDAFEATURES2D` ❌ |
| 3 | FAST/FREAK | `HAVE_OPENCV_XFEATURES2D` (`:649-660`) | ❌ if no contrib → GFTT/ORB | `_gpuFast` (`:383`) ❌ |
| 4 | FAST/BRIEF | `HAVE_OPENCV_XFEATURES2D` | ❌ if no contrib → GFTT/ORB | `_gpuFast` ❌ |
| 5 | GFTT/FREAK | `HAVE_OPENCV_XFEATURES2D` | ❌ if no contrib → GFTT/ORB | `_gpuGftt` (`:448`) ❌ |
| 6 | GFTT/BRIEF | `HAVE_OPENCV_XFEATURES2D` | ❌ if no contrib → GFTT/ORB | `_gpuGftt` ❌ |
| 7 | BRISK | none | ✅ | **none** |
| **8** | **GFTT/ORB** | none — **this is the universal fallback** | ✅ **default** | `_gpuGftt` + embedded `ORB _orb` (`:528`) ❌ |
| 9 | KAZE | OpenCV ≥3 | ✅ | **none** |
| 10 | ORB-OCTREE | `#ifndef RTABMAP_ORB_OCTREE` → GFTT/ORB (`:680-686`) | ✅ (`RTABMAP_ORB_OCTREE` **is** defined) | **none** |
| 11 | SuperPoint | `#ifndef RTABMAP_TORCH` → GFTT/ORB (`:688-694`) | ❌ | libtorch CUDA (`:626`) ❌ |
| 12 | SURF/FREAK | nonfree + xfeatures2d | ❌ | `_gpuSurf` ❌ |
| 13 | GFTT/DAISY | xfeatures2d | ❌ if no contrib | `_gpuGftt` ❌ |
| 14 | SURF/DAISY | nonfree + xfeatures2d | ❌ | `_gpuSurf` ❌ |
| 15 | PyDetector | `RTABMAP_PYTHON` | ❌ | Python/torch (`PyDetector.h:37`) ❌ |

**Usable on this machine today: ORB (2), BRISK (7), GFTT/ORB (8), KAZE (9), ORB-OCTREE (10)** — plus SIFT (1) *if and only if* the OpenCV you build against is ≥4.4 (where SIFT left `xfeatures2d`). Everything else silently rewrites to GFTT/ORB with a `UWARN`.

### B4.3 Comparison for 30 FPS at 640×480

`[EST]` — reasoned from algorithm structure and the measured GPU-ORB stage breakdown; **not measured on this machine.**

| Detector | Detect cost | Descriptor | Match cost (256-bit Hamming / L2) | Odometry robustness | Loop-closure (BoW) robustness | Verdict for 30 FPS here |
|---|---|---|---|---|---|---|
| **ORB (2)** | FAST pyramid, cheapest classical | 256-bit binary, very cheap | Hamming + POPCNT — cheapest | good | good (binary BoW) | ✅ **best pure-speed choice** |
| **GFTT/ORB (8, default)** | `goodFeaturesToTrack` = Harris/min-eigen over the **whole image** + NMS + sort → the most expensive classical detector here | 256-bit binary | Hamming | very good — GFTT gives well-spread, sub-pixel-stable corners, which is why it is the default for *odometry* | weaker than ORB-OCTREE (GFTT corners are not scale-invariant) | ⚠️ works, but GFTT detect is the #1 CPU cost |
| **ORB-OCTREE (10)** | ORB-SLAM2's `ORBextractor`, octree-uniform distribution — more expensive than plain ORB, cheaper than GFTT | 256-bit binary | Hamming | very good (this is the ORB-SLAM recipe) | very good | ✅ strong candidate; **see the mask bug below** |
| BRISK (7) | moderate | 512-bit binary | Hamming, 2× ORB's descriptor bytes | moderate | moderate | ➖ no advantage over ORB |
| KAZE (9) | nonlinear scale space — **far more expensive than all of the above** | 64-float | L2 float | good | good | ❌ not viable at 30 FPS |
| SIFT (1) | DoG pyramid — expensive | 128-float | L2 float, 128 D | excellent | excellent | ❌ at 640×480/1000 feats, too slow for 33 ms with everything else `[EXP]` |
| SuperPoint (11) | CNN | 256-float | L2 | excellent | excellent | ❌ not compiled (`RTABMAP_TORCH` off) |

⚠️ **Bug found in ORB-OCTREE's mask handling** — `Features2d.cpp:2506`:
```cpp
maskRoi.at<unsigned char>(keypoints[i].pt.y+roi.y, keypoints[i].pt.x+roi.x)
```
`maskRoi` is already the ROI sub-matrix, but it is indexed with ROI-offset coordinates. With any non-default `Kp/RoiRatios` or `Kp/GridRows`/`GridCols`, this reads out of bounds. Safe only with the defaults (ROI = full image, 1×1 grid).

⚠️ **`FAST/Gpu=true` aborts the process** on OpenCV ≥3 builds that *do* have `cudafeatures2d` — `Features2d.cpp:1936-1940`:
```cpp
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
	UFATAL("not implemented");
#endif
#endif
```
Never set `FAST/Gpu=true` after a CUDA OpenCV rebuild.

---

## B5 — RTX 3080 Ti / CUDA in RTAB-Map: exactly what is GPU-capable

### B5.1 There is no `RTABMAP_CUDA` define. At all.

Grepping the whole tree for `RTABMAP_CUDA` returns only `RTABMAP_CUDASIFT` as a prefix match. **All OpenCV-CUDA gating is done through OpenCV's own `HAVE_OPENCV_*` macros**, which come from the installed `opencv2/opencv_modules.hpp` (included at `Features2d.cpp:41`) — RTAB-Map never defines them itself. The single OpenCV discovery line, `CMakeLists.txt:233`:
```cmake
FIND_PACKAGE(OpenCV REQUIRED QUIET COMPONENTS core calib3d imgproc highgui stitching photo video videoio OPTIONAL_COMPONENTS aruco objdetect xfeatures2d nonfree gpu cudafeatures2d cudaoptflow cudaimgproc)
```
Note `cudastereo`, `cudawarping` and `cudev` are **not requested** — yet `HAVE_OPENCV_CUDEV` is used in four places. That works only because the macro is OpenCV-owned; the component list controls *linking* only.

### B5.2 Complete GPU-capable code inventory

**Feature detection / description**
| Path | File:line | Macro gate |
|---|---|---|
| SURF CUDA (`cv::cuda::SURF_CUDA`) | `Features2d.cpp:1134-1148`, `:1178-1184`, `:1205-1221` | `RTABMAP_NONFREE` + `HAVE_OPENCV_XFEATURES2D` + CUDA |
| ORB CUDA (`cv::cuda::ORB`) | `Features2d.cpp:1549-1585`, `:1600-1620`, `:1640-1668` | `HAVE_OPENCV_CUDAFEATURES2D` |
| GFTT CUDA (`cv::cuda::createGoodFeaturesToTrackDetector`) | `Features2d.cpp:2085-2101`, `:2127-2140` | `HAVE_OPENCV_CUDAIMGPROC` |
| FAST CUDA | `Features2d.cpp:1926-1941` | **broken** — `UFATAL("not implemented")` on OpenCV≥3 |
| CudaSift SIFT | `Features2d.cpp:1284-1300`, `:1334-1433`, `:1452-1465` | `RTABMAP_CUDASIFT` (separate lib, not OpenCV) |
| SuperPoint (libtorch) | `superpoint_torch/SuperPoint.cc:129-134` | `RTABMAP_TORCH` |
| PyDetector (Python/torch) | `python/PyDetector.cpp`, `rtabmap_superpoint.py:28` | `RTABMAP_PYTHON` |

**Descriptor matching**
| Path | File:line | Gate |
|---|---|---|
| `cv::cuda::DescriptorMatcher::createBFMatcher` (dictionary build) | `VWDictionary.cpp:873-904` | `HAVE_OPENCV_CUDAFEATURES2D`; selected by `Kp/NNStrategy=4` |
| same (query) | `VWDictionary.cpp:1200-1231` | same |
| strategy validation / downgrade | `VWDictionary.cpp:295-318` — `UERROR` + demote to CPU BF when absent | |
| SuperGlue / OANet via Python | `python/PyMatcher.cpp:108-117`, `rtabmap_superglue.py:28` | `RTABMAP_PYTHON`; `Vis/CorNNType=6` |

**Image processing / visual registration**
| Path | File:line | Gate |
|---|---|---|
| `cv::cuda::cvtColor` + GPU frame cache (`SensorData::setImageRawGpu`) | `RegistrationVis.cpp:503-540` | `HAVE_OPENCV_CUDAOPTFLOW` + `HAVE_OPENCV_CUDEV` |
| `cv::cuda::SparsePyrLKOpticalFlow` (VO optical flow) | `RegistrationVis.cpp:646-671` | same; `Vis/CorType=1` + `Vis/CorFlowGpu=true` |
| GPU grayscale in `generateKeypoints3D` | `Features2d.cpp:902-930`, `:951-961`, `:1024-1034` | `HAVE_OPENCV_CUDEV` |
| GPU-resident frame buffers | `SensorData.h:367-372` (accessors), `:428-434` (`cv::cuda::GpuMat _imageRawGpu; _depthOrRightRawGpu;`) | `HAVE_OPENCV_CUDEV` |

**Stereo**
| Path | File:line | Gate |
|---|---|---|
| `cv::cuda::SparsePyrLKOpticalFlow` (sparse stereo correspondence) | `Stereo.cpp:182-219` | `HAVE_OPENCV_CUDAOPTFLOW`; `Stereo/OpticalFlow=true` + `Stereo/Gpu=true` |
| Dense stereo (`stereo/StereoBM.cpp`, `stereo/StereoSGBM.cpp`) | — | **NO GPU AT ALL.** `cv::cuda::StereoBM` / `StereoBeliefPropagation` are never used. |

**ICP**
> **There is zero CUDA in ICP.** No `cv::cuda`, no `GpuMat`, no CUDA symbol appears in `RegistrationIcp.cpp`, `util3d_registration.cpp`, `util3d_filtering.cpp` or any `optimizer/*`. ICP and graph optimisation are 100 % CPU in every configuration.

**Other**
- `rtflann` has a `FLANN_USE_CUDA` → `KDTreeCuda3dIndex` path (`rtflann/algorithms/all_indices.h:44-45, 163-166`) — **`FLANN_USE_CUDA` is never defined anywhere in the repo. Dead.**
- `find . -name "*.cu" -o -name "*.cuh"` → **zero.** RTAB-Map ships no CUDA kernels of its own and never invokes `nvcc`. Every GPU path delegates to a prebuilt external library.

### B5.3 Status today on this machine

**Every OpenCV-gated GPU path is inert** (no `libopencv_cuda*` exists), and **every non-OpenCV GPU path is also off** because `RTABMAP_CUDASIFT`, `RTABMAP_TORCH` and `RTABMAP_PYTHON` are all commented out in the stale `Version.h` (KF-8). Net: **RTAB-Map on this machine has exactly zero GPU code paths active.** The RTX 3080 Ti is idle.

### B5.4 What an OpenCV CUDA rebuild would unlock — and what it costs

```bash
# Prerequisites verified present: nvcc 12.4 at /usr/local/cuda-12.4, compute cap 8.6
git clone --branch 4.11.0 --depth 1 https://github.com/opencv/opencv.git
git clone --branch 4.11.0 --depth 1 https://github.com/opencv/opencv_contrib.git
cmake -S opencv -B opencv/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENCV_EXTRA_MODULES_PATH=$PWD/opencv_contrib/modules \
  -DWITH_CUDA=ON \
  -DCUDA_ARCH_BIN=8.6 -DCUDA_ARCH_PTX= \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.4 \
  -DWITH_CUDNN=OFF -DOPENCV_DNN_CUDA=OFF \
  -DBUILD_opencv_cudafeatures2d=ON -DBUILD_opencv_cudaoptflow=ON \
  -DBUILD_opencv_cudaimgproc=ON -DBUILD_opencv_cudawarping=ON -DBUILD_opencv_cudev=ON \
  -DOPENCV_ENABLE_NONFREE=ON \
  -DBUILD_LIST=core,imgproc,calib3d,features2d,highgui,video,videoio,photo,stitching,flann,xfeatures2d,cudev,cudafeatures2d,cudaoptflow,cudaimgproc,cudawarping,cudaarithm \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DWITH_TBB=ON     # or leave the default pthreads backend — see B13.4
```

Unlocked, with an honest verdict for each:

| Unlocked | Parameter | Worth it on this hardware? |
|---|---|---|
| `cv::cuda::ORB` | `ORB/Gpu=true` (`Parameters.h:322`) | ✅ **YES — the single best GPU offload.** MEASURED: a CUDA ORB-stage pipeline costs **0.64 ms total at 640×480** (upload 0.030 / pyramid 0.082 / blur 0.151 / FAST-9 0.169 / orientation+BRIEF 0.156 / D2H 0.029), **0.84 ms at 1280×720**, **1.19 ms at 1920×1080**. Conservative real-world speedup **5–10×** vs an optimised CPU extractor. |
| `cv::cuda::createGoodFeaturesToTrackDetector` | `GFTT/Gpu=true` (`:313`) | ✅ likely yes — GFTT is the default detector's most expensive stage and is a pure stencil+NMS+sort, ideal for GPU. `[EXP]` — not measured. |
| `cv::cuda::SURF_CUDA` | `SURF/GpuVersion=true` (`:283`) + `OPENCV_ENABLE_NONFREE` | ➖ SURF is not the right accuracy/speed point here |
| `cv::cuda::DescriptorMatcher` BF | `Kp/NNStrategy=4`, `Vis/CorNNType=4` (`:241`, `:721`) | ❌ **NO — measured net loss.** See B5.5. |
| `cv::cuda::SparsePyrLKOpticalFlow` | `Vis/CorFlowGpu=true` (`:729`), `Stereo/Gpu=true` (`:807`) | ⚠️ only if you switch to `Vis/CorType=1`; `[EXP]` |
| `cv::cuda::cvtColor` + `GpuMat` frame cache | automatic under `HAVE_OPENCV_CUDEV` | ✅ free once the frame is already on the GPU for ORB |
| `FAST/Gpu=true` | `:300` | ❌ **`UFATAL` — will kill the process.** (`Features2d.cpp:1938`) |

**Costs:** build time on 20 logical CPUs ≈ **60–120 min** wall clock for `CUDA_ARCH_BIN=8.6` only (a multi-arch build is several times that) `[EST]`; disk ≈ 8–15 GB of build tree. `OPENCV_ENABLE_NONFREE=ON` is what `CMakeLists.txt:923-931` looks for — it literally greps the installed `opencv_modules.hpp` for `#define OPENCV_ENABLE_NONFREE`. Licensing: SIFT is patent-free since 2020, SURF is not — do not ship nonfree in a product build.

**And note: this rebuild alone is not sufficient.** You must *also* fix KF-8, or `RTABMAP_NONFREE` stays off regardless of what OpenCV reports.

### B5.5 🔴 GPU descriptor matching is a **net loss** in RTAB-Map — and the code says exactly why

MEASURED GPU offload floor on this machine: **64 KB H2D + kernel + 64 KB D2H = 0.108 ms**; a bare `kernel + cudaDeviceSynchronize` = **0.027 ms**; async enqueue alone = **0.0077 ms**.

MEASURED CPU 256-bit Hamming with hardware POPCNT:
| Comparisons | CPU POPCNT |
|---|---|
| 30 k | 0.060 ms |
| 80 k | 0.150 ms |
| 400 k | 0.789 ms |
| 4 M (2000×2000 brute force) | 5.03–7.88 ms (GPU full round trip: 0.648 ms) |

→ GPU wins only above roughly **1 M comparisons per offload**.

**How many comparisons does RTAB-Map actually do per odometry frame?** From the source, with defaults:

`Vis/CorType=0` (Features Matching, default) + `Odom/GuessMotion=true` (default) + `Vis/CorGuessWinSize=40` (default) takes the **spatially-constrained** branch at `RegistrationVis.cpp:1011-1016`. It:
1. projects the local-map 3-D points with `cv::projectPoints` (`:1038`),
2. builds a **2-D** rtflann kd-tree over the projections (`:1079-1081`),
3. `radiusSearch` with r = 40 px (`:1090`, `:1230`),
4. runs `cv::BFMatcher` over **only the candidates inside each 40 px window** (`:1123-1135`, `:1275-1295`).

With `OdomF2M/MaxSize=2000` and typically 500–1500 map points projecting into the frame, and 5–40 candidates per 40 px window at 640×480 with ~1000 features, the per-frame comparison count is **≈ 1×10⁴ – 6×10⁴**. That is the 30 k / 80 k row above: **0.06–0.15 ms on CPU.**

**A single GPU round trip costs 0.108 ms.** Even reducing the matching computation to *zero* cannot pay for the transfer. And matching is invoked more than once per frame, so the GPU version is a strict loss.

> **Recommendation: do not set `Vis/CorNNType=4` or `Kp/NNStrategy=4`.** Keep matching on the CPU. This is the same structural conclusion the ORB-SLAM3 analysis reached, for the same reason: both systems use spatial constraints to avoid brute force, which keeps the workload two orders of magnitude below the GPU break-even.

The only place where a ≥1 M-comparison brute force actually happens is **loop-closure candidate verification / relocalisation** — `VWDictionary` global kNN over the whole vocabulary. That already runs on the mapping thread at 1 Hz, **off the odometry critical path**, so accelerating it does nothing for 30 FPS. It would help `Timing/Likelihood_computation/ms` on large maps `[EXP]`.

### B5.6 FLANN, not brute force — and its rebuild behaviour

`Vis/CorNNType` default is **1 = kNNFlannKdTree** (`Parameters.h:721`), `Kp/NNStrategy` default **1** (`:241`). The relevant cost is therefore the **FLANN index**, not raw comparisons:

- `VWDictionary::update()` (`corelib/src/VWDictionary.cpp:468`) maintains an *incremental* FLANN index — `Kp/IncrementalFlann` default **true** (`Parameters.h:243`). Per update it removes `_removedIndexedWords` (`:492-500`) and inserts `_notIndexedWords` (`:502-528`) point-by-point via `_flannIndex->addPoints`.
- A **full rebuild** is triggered only when the dictionary grows by `Kp/FlannRebalancingFactor` (`Parameters.h:244`, default **2.0**) — `_flannIndex->buildKDTreeIndex(descriptor, KDTREE_SIZE, useDistanceL1_, _rebalancingFactor)` (`:539`) or `buildLinearIndex` (`:535`).
- **Because rebuilds are geometric (every 2× growth), their amortised cost is O(1) per word — but the *instantaneous* cost is O(N log N) and lands entirely inside one `Rtabmap::process` call.** With `Rtabmap/DetectionRate=1` that is a 1 Hz spike, visible in `TimingMem/Joining_dictionary_update/ms`, not a 30 Hz problem. Setting `Kp/FlannRebalancingFactor` ≤ 1 disables rebuilding (index quality degrades over time); raising it to 3–4 makes spikes rarer but larger.
- ⚠️ `rtflann/algorithms/kdtree_index.h` has **all 20 of its `#pragma omp` directives commented out** (lines 384–639), unlike `nn_index.h` (20 active) and `lsh_index.h` (8 active). **The kd-tree search RTAB-Map actually uses is single-threaded.** That is good news here (see B13.4) — it means the default FLANN path does *not* pay the 3.79 ms OpenMP barrier.

⚠️ Separately, `RegistrationVis.cpp:1478` constructs a **whole `VWDictionary` per registration call** in the non-guess fallback path:
```cpp
	VWDictionary dictionary(_featureParameters);
```
That path is taken when there is no motion guess (loop-closure verification, odometry reset, first frame). It is a real allocation + index build per call — acceptable at 1 Hz, not at 30 Hz. It is one more reason `Odom/GuessMotion=true` must stay on.

---

## B6 — Odometry strategy comparison

### B6.1 The real mapping

`Parameters.h:456`:
```cpp
RTABMAP_PARAM(Odom, Strategy,               int, 0,       "0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D");
```
`Odometry::Type` (`corelib/include/rtabmap/core/Odometry.h:45-60`):
```cpp
	enum Type {
		kTypeUndef = -1,  kTypeF2M = 0,   kTypeF2F = 1,   kTypeFovis = 2,
		kTypeViso2 = 3,   kTypeDVO = 4,   kTypeORBSLAM = 5,  kTypeOkvis = 6,
		kTypeLOAM = 7,    kTypeMSCKF = 8, kTypeVINS = 9,  kTypeOpenVINS = 10,
		kTypeFLOAM = 11,  kTypeOpen3D = 12
	};
```

⚠️ **The parameter description is stale.** It says `5=ORB_SLAM2`, but `Odometry::create` (`Odometry.cpp:87-93`) dispatches to `OdometryORBSLAM3` unless `RTABMAP_ORB_SLAM == 2`. On a modern build, **`Odom/Strategy=5` means ORB-SLAM3**.

Availability on this build (all the exotic ones need defines that are off — KF-8): only **0 (F2M)** and **1 (F2F)** work. Every other value falls to `default:` → `UERROR("Unknown odometry type %d, using F2M instead...")` (`:114-119`).

### B6.2 F2M vs F2F

| | **F2M** (`Odom/Strategy=0`, default) | **F2F** (`Odom/Strategy=1`) |
|---|---|---|
| Reference | Persistent **local feature map** of up to `OdomF2M/MaxSize=2000` words (`Parameters.h:479`) | The **last keyframe** only |
| Impl | `corelib/src/odometry/OdometryF2M.cpp` (1577 lines) | `corelib/src/odometry/OdometryF2F.cpp` (330 lines) |
| Sets `data.setFeatures` | ✅ `:338`, `:1264`, `:1299` | ✅ `:302` |
| Local BA | `OdomF2M/BundleAdjustment` (`:489-492`) — **default 0 here** (no g2o, KF-8) | none |
| Keyframe policy | `Odom/KeyFrameThr=0.3` (`:471`), `Odom/VisKeyFrameThr=150` (`:472`) | same |
| Latency | Higher — matches against 2000 map words, plus map maintenance (`OdomF2M/MaxNewFeatures`, `OdomF2M/ValidDepthRatio`) | Lower — matches against one frame's worth of features |
| Drift | Lower (map acts as a short-horizon anchor) | Higher |
| Latency variance | Higher: keyframe-insertion frames do extra map merge/prune work | Lower and flatter — **better P99** |

### B6.3 Registration strategy — `Reg/Strategy` (`Parameters.h:677`)
```cpp
RTABMAP_PARAM(Reg, Strategy,                 int, 0,        "0=Vis, 1=Icp, 2=VisIcp");
```
This is the knob that decides whether ICP is on the critical path at all. Default **0 = Vis only.**

### B6.4 Ranking for 30 FPS RGB-D on this machine

1. **`Odom/Strategy=1` (F2F) + `Reg/Strategy=0`** — lowest and flattest latency. Best P99. Use when drift is acceptable or when a loop-closing map layer (which RTAB-Map provides anyway) absorbs the drift.
2. **`Odom/Strategy=0` (F2M) + `Reg/Strategy=0`** — the default; best accuracy/latency compromise. Recommended starting point.
3. `Odom/Strategy=5` (ORB-SLAM3) — best tracking quality, but requires a rebuild with ORB-SLAM3 (KF-8) and duplicates feature extraction (KF-4). See §B-EXTRA.
4. `Reg/Strategy=2` (Vis+ICP) — adds full ICP to every odometry frame. **Not viable at 30 FPS for dense RGB-D.** See B7.
5. `Reg/Strategy=1` (ICP only) — RGB-D ICP without visual init. Slowest and least robust for a hand-held camera.

---

## B7 — ICP: does it fit in 33.33 ms?

### B7.1 The parameters (with the `#ifdef`-dependent defaults that apply *here*)

From `Parameters.h:751-795`. Because `RTABMAP_POINTMATCHER` is **off** (KF-8), the effective defaults are the `#else` branches:
```cpp
752:#ifdef RTABMAP_POINTMATCHER
753:    RTABMAP_PARAM(Icp, Strategy,                  int, 1, "ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).");
755:    RTABMAP_PARAM(Icp, Strategy,                  int, 0, ...);   ← EFFECTIVE HERE
757:    RTABMAP_PARAM(Icp, MaxTranslation,            float, 0.2,   "Maximum ICP translation correction accepted (m).");
758:    RTABMAP_PARAM(Icp, MaxRotation,               float, 0.78,  "Maximum ICP rotation correction accepted (rad).");
759:    RTABMAP_PARAM(Icp, VoxelSize,                 float, 0.05,  "Uniform sampling voxel size (0=disabled).");
760:    RTABMAP_PARAM(Icp, DownsamplingStep,          int, 1,       "Downsampling step size (1=no sampling). This is done before uniform sampling.");
761:    RTABMAP_PARAM(Icp, RangeMin,                  float, 0,     "Minimum range filtering (0=disabled).");
762:    RTABMAP_PARAM(Icp, RangeMax,                  float, 0,     "Maximum range filtering (0=disabled).");
764:    RTABMAP_PARAM(Icp, MaxCorrespondenceDistance, float, 0.1,   ...);
766:    RTABMAP_PARAM(Icp, MaxCorrespondenceDistance, float, 0.05,  ...);   ← EFFECTIVE HERE
768:    RTABMAP_PARAM(Icp, ReciprocalCorrespondences, bool, true,   "...should be both their closest correspondence.");
769:    RTABMAP_PARAM(Icp, Iterations,                int, 30,      "Max iterations.");
770:    RTABMAP_PARAM(Icp, Epsilon,                   float, 0,     "...transformation epsilon...");
771:    RTABMAP_PARAM(Icp, CorrespondenceRatio,       float, 0.1,   "Ratio of matching correspondences to accept the transform.");
772:    RTABMAP_PARAM(Icp, Force4DoF,                 bool, false,  "Limit ICP to x, y, z and yaw DoF...");
773:    RTABMAP_PARAM(Icp, FiltersEnabled,            int, 3,       "1=\"from\" cloud only, 2=\"to\" cloud only, 3=both.");
775:    RTABMAP_PARAM(Icp, PointToPlane,              bool, true,   "Use point to plane ICP.");
777:    RTABMAP_PARAM(Icp, PointToPlane,              bool, false,  ...);   ← EFFECTIVE HERE
779:    RTABMAP_PARAM(Icp, PointToPlaneK,             int, 5,       "Number of neighbors to compute normals...");
780:    RTABMAP_PARAM(Icp, PointToPlaneRadius,        float, 0.0,   "Search radius to compute normals...");
781:    RTABMAP_PARAM(Icp, PointToPlaneGroundNormalsUp, float, 0.0, ...);
782:    RTABMAP_PARAM(Icp, PointToPlaneMinComplexity, float, 0.02,  ...);
783:    RTABMAP_PARAM(Icp, PointToPlaneLowComplexityStrategy, int, 1, ...);
784:    RTABMAP_PARAM(Icp, OutlierRatio,              float, 0.85,  ...);
789:    RTABMAP_PARAM(Icp, PMMatcherKnn,             int, 1,        "KDTreeMatcher/knn...");
793:    RTABMAP_PARAM(Icp, CCSamplingLimit,          unsigned int, 50000, ...);
795:    RTABMAP_PARAM(Icp, CCMaxFinalRMS,            float, 0.2,   "Maximum final RMS error.");
```
`RegistrationIcp::parseParameters` demotes strategy 1→0 with a `UWARN` at `RegistrationIcp.cpp:149-153`, and 2→1→0 at `:282-290`. **ICP is PCL-only here.** `corelib/src/icp/` holds only two header-only helpers (`cccorelib.h`, `libpointmatcher.h`), neither compiled.

### B7.2 The pipeline — `RegistrationIcp::computeTransformationImpl`, `RegistrationIcp.cpp:314-988`

```
:369 / :410   util3d::commonFiltering(scan, _downsamplingStep, _rangeMin, _rangeMax, _voxelSize,
                                       pointToPlane?_pointToPlaneK:0, ...)     ← the whole filter stage
:520-582      util3d::computeNormalsComplexity — structural complexity gate (Icp/PointToPlaneMinComplexity)
:593-660      POINT-TO-PLANE branch → util3d::icpPointToPlane (PCL)   [off by default here]
:664-852      POINT-TO-POINT branch → util3d::icp (PCL)               [the default here]
:851          util3d::computeVarianceAndCorrespondences(..., _reciprocalCorrespondences)
:862-937      accept/reject: MaxTranslation, MaxRotation, CorrespondenceRatio, covariance
```
`util3d::commonFiltering` (`util3d_filtering.cpp:74-333`) fuses step-downsampling + range filter (`:89-160`, directly on the `cv::Mat` scan) with voxelisation (`voxelize`, `:179`/`:212`/`:272`) and normal computation. `pcl::VoxelGrid` is used (`:739-752`) — **not** the OMP variant.

### B7.3 Cloud-size reasoning and the honest verdict

For a 640×480 RGB-D frame the raw organised cloud is **307 200 points**. With `Icp/VoxelSize=0.05` m in a typical indoor room (≈5×5×3 m occupied volume), the surviving count is on the order of **3 000 – 15 000 points** `[EST]`, and with `Icp/DownsamplingStep=1` you voxelise the *full* 307 k first.

Per ICP call, with `Icp/Iterations=30` and `Icp/ReciprocalCorrespondences=true`:
- voxelisation of 307 k points: `[EST]` **4–12 ms** (PCL `VoxelGrid` is single-threaded, hash-based, and touches all 307 k points)
- kd-tree build on the target cloud: `[EST]` **1–4 ms**
- 30 iterations × reciprocal NN search over ~5–15 k points: `[EST]` **6–25 ms** (reciprocal correspondence **doubles** the searches)
- `computeVarianceAndCorrespondences`: `[EST]` **1–3 ms**

**Total per ICP call: ~12–45 ms `[EST]`.** Against a 33.33 ms budget that already has to cover visual odometry.

> **Verdict: ICP does NOT fit inside 33.33 ms for dense RGB-D on this CPU, and `Reg/Strategy=2` (Vis+ICP) must not be used for 30 FPS RGB-D odometry.**
>
> The configuration that *can* work is the default: `Reg/Strategy=0` (Vis only) for odometry, with ICP reserved for loop-closure link refinement on the 1 Hz mapping thread. If you absolutely need ICP per-frame, you must (a) raise `Icp/VoxelSize` to 0.10–0.15 m, (b) set `Icp/DownsamplingStep` to 4–8 so the 307 k is cut *before* voxelisation, (c) cut `Icp/Iterations` to 5–10, (d) set `Icp/ReciprocalCorrespondences=false`, and (e) accept the accuracy loss. Even then it is `[EXP]`.

**ICP + OpenMP interaction — read B13.4.** `util3d::computeNormals` (`util3d_surface.cpp:2838`) selects `pcl::NormalEstimationOMP` only when `PCL_OMP` is defined (`CMakeLists.txt:276`). If it is, and `OMP_NUM_THREADS` is left at 20, **every point-to-plane ICP call pays a measured 3.79 ms OpenMP barrier on top of the numbers above.**

---

## B8 — Async design: what already exists, what is actually missing

### B8.1 Already asynchronous (do not re-invent)

| Concern | Mechanism | Evidence |
|---|---|---|
| Sensor capture | `SensorCaptureThread : UThread` | `SensorCaptureThread.cpp:325`, posts `SensorEvent` `:538` |
| Odometry | `OdometryThread : UThread` (standalone) / `OdometryROS : UThread` (ROS) | `OdometryThread.cpp:104`; `OdometryROS.cpp:388, 491` |
| Mapping + loop closure + optimiser | `RtabmapThread : UThread` / `CoreWrapper` processing timer on its own callback group | `RtabmapThread.cpp:180, 473`; `CoreWrapper.cpp:280-283` |
| Database writes | `DBDriver : UThreadNode` with `asyncSave` | `DBDriver.h:62, 78-80` |
| Dictionary update ‖ feature extraction | `PreUpdateThread`, gated by `Kp/Parallelized` (default **true**) | `Memory.cpp:4519, 4610, 4769` |
| Sensor-data compression | `Mem/CompressionParallelized` (`Parameters.h:229`, default **true**) | |
| IMU ingestion (ROS) | separate callback group | `OdometryROS.cpp:377-382` |

That is a genuinely well-threaded design. **Loop closure and graph optimisation are already off the odometry thread.** Proposals to "make loop closure async" are already satisfied.

### B8.2 🔴 Real-time priority is **measured impossible** here — revise any plan that depends on it

MEASURED on this machine, as a normal user in WSL2:
```
sched_setscheduler(SCHED_FIFO, prio 50) -> EPERM "Operation not permitted"
sched_setscheduler(SCHED_RR,   prio 10) -> EPERM
nice(-10)                               -> EPERM
pthread_setaffinity_np                  -> OK
```
Additionally `/sys/.../cpufreq` is absent and `topology/core_id` is flattened to 0..9 for 20 logical CPUs, so **P-cores and E-cores cannot be distinguished, let alone pinned.**

### B8.3 What actually works instead

1. **Bounded queues with an explicit, *counted* drop policy.** The mechanism exists (`Odom/ImageBufferSize=1`, `Rtabmap/ImageBufferSize=1`) but the drops are silent — `OdometryROS.cpp:477` has the warning commented out, and `OdometryThread.cpp:185` logs at `UDEBUG`. Expose `droppedMsgs_` / `processedMsgs_` as a published ratio. **This is the single most important missing observability feature.** See `perf_patches/03-expose-drop-counters.md`.
2. **Affinity partitioning** (works, per the measurement). Give the odometry thread a disjoint CPU set from the mapping thread and from every library thread pool, so a 1 Hz graph optimisation cannot steal the cores odometry is running on:
   ```bash
   # 20 logical CPUs. Reserve 0-7 for odometry, 8-15 for mapping/DB, 16-19 for ROS/DDS.
   taskset -c 0-7   ros2 run rtabmap_odom rgbd_odometry ...
   taskset -c 8-15  ros2 run rtabmap_slam rtabmap ...
   ```
   (Coarse but effective; `pthread_setaffinity_np` inside the process would be finer — but see the E-core caveat.)
3. **Cap every library thread pool** so background work cannot oversubscribe: `OMP_NUM_THREADS=6`, `cv::setNumThreads(6)`, and PCL's OpenMP inherits `OMP_NUM_THREADS`. See B13.4 — this is worth more than anything in this section.
4. **Warm up one OpenMP parallel region at startup** — MEASURED: the first region costs **10.508 ms** (thread-pool spawn), which would otherwise land on frame 1.
5. **Accept E-core migration as an unavoidable P99 risk** and design for it: a frame that lands on an E-core runs roughly 1.5–2× slower `[EST]`, and there is no way to prevent it from inside WSL2. Budget for it, or run on bare-metal Linux for anything with a hard deadline.

### B8.4 Genuinely missing (small, and not about threading)

- Drop-rate telemetry (above).
- No adaptive degradation: nothing reduces `Kp/MaxFeatures` or raises `Odom/ImageDecimation` when frames start dropping. A closed loop on `droppedMsgs_` would be a real improvement `[EXP]`.
- `Rtabmap/TimeThr` (`Parameters.h:180`) is the built-in adaptive mechanism for the *mapping* side and defaults to **0 = off**. Turning it on is the sanctioned way to bound `Timing/Total/ms`. See B9.

---

## B9 — Memory management: why latency grows with the map, and how to bound it

### B9.1 Verified parameters

| Key | Line | Default | Role |
|---|---|---|---|
| `Mem/STMSize` | `Parameters.h:213` | `10` | Short-Term Memory size. Nodes in STM are exempt from loop-closure detection (they are the "just seen" window) and from transfer. |
| `Rtabmap/TimeThr` | `:180` | **`0` = infinity** | Max map-update time (ms). Over budget → nodes move WM→LTM. **The built-in real-time governor, off by default.** |
| `Rtabmap/MemoryThr` | `:181` | **`0` = infinity** | Max nodes in Working Memory. Same transfer mechanism, size-based. |
| `Mem/IncrementalMemory` | `:214` | `true` | true = SLAM, false = localisation-only (map frozen). |
| `Mem/ReduceGraph` | `:216` | `false` | Merge nodes when loop closures are added. |
| `Mem/BinDataKept` | `:205` | `true` | Keep raw binary (images/scans) in the DB. Drives DB size and compression cost, not WM latency. |
| `Mem/ImageKept` | `:204` | `false` | Keep raw images in **RAM**. |
| `Mem/RehearsalSimilarity` | `:203` | `0.6` | Similarity above which a new node is merged into the previous one instead of added. |
| `Rtabmap/DetectionRate` | `:182` | `1` Hz | How often `Rtabmap::process` runs at all. |
| `Mem/RecentWmRatio` | `:217` | `0.2` | Fraction of WM after the last loop closure that is immune to transfer. |
| `RGBD/LinearUpdate` | `:362` | `0.1` m | Minimum motion before a node is created. |
| `RGBD/AngularUpdate` | `:363` | `0.1` rad | ditto, rotation. |
| `Bayes/FullPredictionUpdate` | `:352` | `false` | Regenerate the whole prediction matrix each iteration (keep false). |
| `Kp/IncrementalFlann` | `:243` | `true` | Incremental FLANN index (keep true). |
| `Kp/FlannRebalancingFactor` | `:244` | `2.0` | Growth factor that triggers a full index rebuild. |

### B9.2 Why latency grows — three distinct mechanisms, each with its citation

**(1) Bayes filter: O(N²) dense matrix multiply over |WM|.** The dominant super-linear term.
`corelib/src/BayesFilter.cpp:313`:
```cpp
	cv::Mat prediction = cv::Mat::zeros(ids.size(), ids.size(), CV_32FC1);
```
`corelib/src/BayesFilter.cpp:194-197`:
```cpp
	// Multiply prediction matrix with the last posterior
	// (m,m) X (m,1) = (m,1)
	prior = _prediction * posterior;
```
With N = |WM|: **N² floats of memory and N² MACs per detection iteration.** N=1000 → 4 MB / 1 M MAC (sub-ms). N=5000 → 100 MB / 25 M MAC. N=10000 → **400 MB / 100 M MAC**, and the multiply alone becomes memory-bandwidth-bound at tens of ms `[EST]`. `Bayes/FullPredictionUpdate=false` (the default) makes `generatePrediction` take the incremental path (`BayesFilter.cpp:282-284` → `updatePrediction`), which avoids *rebuilding* the matrix — but the multiply is unconditional.
→ Watch `Timing/Posterior_computation/ms`.

**(2) TF-IDF likelihood: O(words × places-per-word).** `corelib/src/Memory.cpp:1905` `Memory::computeLikelihood`, the `_tfIdfLikelihoodUsed` branch (`Kp/TfIdfLikelihoodUsed`, `Parameters.h:258`, default **true**) walks the inverted index:
```cpp
		for(std::list<int>::const_iterator i=wordIds.begin(); i!=wordIds.end(); ++i)
		{
				// "Inverted index" - Pour chaque endroit contenu dans chaque mot
				vw = _vwd->getWord(*i);
				const std::map<int, int> & refs = vw->getReferences();
				nw = refs.size();
```
Cost = Σ over the signature's words of (places referencing that word). As the map grows, common words accumulate references, so this grows **sub-linearly but unboundedly**. The `!_tfIdfLikelihoodUsed` branch is far worse — a literal `for` over every id calling `signature->compareTo(*sB)` (`Memory.cpp:1924-1935`), i.e. O(N) full signature comparisons. **Keep `Kp/TfIdfLikelihoodUsed=true`.**
→ Watch `Timing/Likelihood_computation/ms`.

**(3) FLANN index growth + periodic rebuild.** `VWDictionary::update()` (`corelib/src/VWDictionary.cpp:468`): incremental add/remove at `:492-528`, full rebuild at `:535`/`:539` when the dictionary grows by `Kp/FlannRebalancingFactor`. Amortised O(1) per word; instantaneous O(N log N) spike.
→ Watch `TimingMem/Joining_dictionary_update/ms`.

**(4) Database.** `DBDriverSqlite3` writes on its own thread (`DBDriver : UThreadNode`), but `Rtabmap::process` **waits** for it at two points — `Timing/Joining_trash/ms` (`Rtabmap.cpp:2221`) and `Timing/Emptying_trash/ms`. As the DB grows, these waits grow. `Mem/BinDataKept=true` (default) means full compressed RGB+depth per node goes to disk; on WSL2, `/mnt/c` paths are **9p filesystem and dramatically slower than the ext4 VHD** — always put `database_path` under the Linux filesystem (`~/…`), never under `/mnt/c/…`.

### B9.3 Parameters that hold 30 FPS

Remember (KF-1): the *mapping* side has a 1000 ms budget at `DetectionRate=1`, not 33.33 ms. The goal is to keep `Timing/Total/ms` comfortably under `1000/DetectionRate` so the drop-oldest queue never fires.

```bash
# --- Bound the mapping thread so map growth cannot cause frame drops ---
--Rtabmap/TimeThr 700            # ms. THE key change. 0 (default) = unbounded.
                                 #   Transfers WM->LTM whenever an update exceeds this.
                                 #   700 gives 30% headroom under a 1 Hz DetectionRate.
--Rtabmap/MemoryThr 0            # leave 0; let TimeThr govern (it is adaptive, MemoryThr is not)
--Mem/STMSize 30                 # default 10. Larger STM => a longer "recently seen" window
                                 #   exempt from loop detection; reduces spurious close-in-time loops.
--Rtabmap/DetectionRate 1        # default. Raise only if you genuinely need denser nodes.
--RGBD/LinearUpdate 0.1          # default. Raising to 0.2 halves node creation rate in a walk.
--RGBD/AngularUpdate 0.1         # default
--Bayes/FullPredictionUpdate false   # default; NEVER set true for realtime
--Kp/TfIdfLikelihoodUsed true    # default; NEVER set false
--Kp/IncrementalFlann true       # default
--Kp/FlannRebalancingFactor 2.0  # default; raise to 4.0 for fewer/larger spikes
--Mem/BinDataKept true           # keep unless DB size is a problem; it does not affect WM latency
--Mem/ImageKept false            # default; true would balloon RAM
--Mem/ReduceGraph false          # default; true trades map fidelity for a smaller graph
```
`Rtabmap/TimeThr` is the single highest-value mapping-side change and it is **off by default**. In this project's own runs it was left at the default (`run_rtabmap_four_bags.py:34` → `RTAB_ARGS="-d"`), and the robot config explicitly set `Rtabmap/TimeThr: "0"` — i.e. **no run in this project has ever had a map-update time budget.**

For long sessions, also consider `--Mem/IncrementalMemory false` once the map is built (localisation mode): WM is then frozen, the Bayes matrix stops growing, and `Timing/Total/ms` becomes flat.

---

## B10 — Graph optimiser

### B10.1 What is actually present

`corelib/src/optimizer/`: `OptimizerTORO.cpp` (19.5 KB), `OptimizerG2O.cpp` (89.6 KB), `OptimizerGTSAM.cpp` (43.2 KB), `OptimizerCeres.cpp` (21.0 KB), `OptimizerCVSBA.cpp` (7.2 KB), plus vendored `g2o/`, `gtsam/`, `ceres/`, `toro3d/`, `vertigo/` subdirectories.

`corelib/include/rtabmap/core/Optimizer.h:64-71`:
```cpp
	enum Type {
		kTypeUndef = -1,
		kTypeTORO = 0,
		kTypeG2O = 1,
		kTypeGTSAM = 2,
		kTypeCeres = 3,
		kTypeCVSBA = 4
	};
```
`Optimizer/Strategy` (`Parameters.h:409-429`) — default depends on what is compiled: **2 (GTSAM)** if `RTABMAP_GTSAM`, else **1 (g2o)**, else **3 (Ceres)**, else **0 (TORO) with `Optimizer/Iterations=100`**.

**`Optimizer.cpp` contains zero `#ifdef`s.** Dispatch is runtime, via each backend's `available()` static (each of which is itself `#ifdef`-guarded inside its own `.cpp`: `RTABMAP_TORO`, `RTABMAP_G2O`, `RTABMAP_GTSAM`, `RTABMAP_CERES`, `RTABMAP_CVSBA`). `Optimizer::create` (`Optimizer.cpp:79-186`) has an explicit fallback chain:

| Requested (unavailable) | 1st | 2nd | 3rd |
|---|---|---|---|
| TORO | GTSAM | g2o | Ceres |
| g2o | GTSAM | TORO | Ceres |
| GTSAM | g2o | TORO | Ceres |
| CVSBA | g2o | — | — |
| Ceres | GTSAM | g2o | TORO |

and the `switch`'s `default:` is `new OptimizerTORO(parameters)`.

**On this build (KF-8): TORO only, 100 iterations.** `Optimizer/Strategy=1/2/3` are silently demoted.

### B10.2 Confirmation that optimisation is off the odometry critical path

`Rtabmap::optimizeCurrentMap` is called from 10 sites, but only **one** is on the per-frame mapping path: **`corelib/src/Rtabmap.cpp:3802`**, inside `Rtabmap::process` (which begins at `:1209`). Its cost is captured at `:3994` (`timeMapOptimization = timer.ticks();`) and published as `Statistics::kTimingMap_optimization()` at `:4177`.

The direct `_graphOptimizer->optimize(...)` sites are `:3305`, `:3455` (localisation mode, inside `Rtabmap::process`), `:5275`, `:5284` (inside `Rtabmap::optimizeGraph`, `:5210`), `:5795`, `:5937` (`detectMoreLoopClosures` — post-processing only), and `:6345` (`addLink`).

`Rtabmap::process` runs on `RtabmapThread` (`RtabmapThread.cpp:473`) in the standalone build, and on `CoreWrapper`'s dedicated processing callback group in ROS 2 (`CoreWrapper.cpp:280-283, 1969, 2293`). **Either way, never on the odometry thread.** ✅ Confirmed.

`Rtabmap::optimizeGraph` short-circuits entirely when `_graphOptimizer->iterations() == 0` (`Rtabmap.cpp:5257-5265`) — setting `--Optimizer/Iterations 0` is the emergency "turn optimisation off" switch.

### B10.3 Recommendation

1. **Fix KF-8 and install g2o or GTSAM.** TORO at 100 iterations is both the slowest available backend and the least accurate. With GTSAM you get `Optimizer/Iterations=20` and `GTSAM/Incremental=true` (iSAM2, `Parameters.h:451`) which makes loop-closure optimisation incremental rather than batch — exactly what you want for bounded latency on a growing map.
2. Keep `Optimizer/Robust=false` (default) — Vertigo robust optimisation is incompatible with `RGBD/OptimizeMaxError` (`Parameters.h:431`), and `RGBD/OptimizeMaxError=3.0` (`:369`) is the cheaper loop-rejection mechanism.
3. If you must stay on TORO, drop `--Optimizer/Iterations 30`. 100 is a no-backend fallback value, not a tuned one.

---

## B11 — Resolution, and the "record high, SLAM low" idea

### B11.1 There are FOUR real decimation parameters — all verified, all built in

| Key | Line | Default | Where it acts | Does it change what's saved? |
|---|---|---|---|---|
| `Odom/ImageDecimation` | `Parameters.h:474` | `1` | `Odometry::process`, `corelib/src/Odometry.cpp:722-768` — decimates **only the copy handed to `computeTransform`**; keypoints are scaled back up at `:774-803` | **No.** The full-resolution `SensorData` continues downstream. |
| `Mem/ImagePreDecimation` | `Parameters.h:227` | `1` | `Memory::createSignature`, `corelib/src/Memory.cpp:4887-4930` — decimates before **visual feature detection** for the loop-closure vocabulary | No (only the detection input) |
| `Mem/ImagePostDecimation` | `Parameters.h:228` | `1` | `Memory::createSignature` — decimates before **saving to the database** | **Yes** — this is the one that shrinks the DB |
| `decimation` (ROS, `rgbd_sync`) | `rtabmap_sync/src/nodelets/rgbd_sync.cpp:77` | `1` | at the sync node, before anything else | Yes, for everything downstream |

Plus `Vis/DepthAsMask` (`Parameters.h:712`, default **true**) and `Mem/DepthAsMask` (`:224`, default **true**), which use depth as a feature-extraction mask — cheap and worth keeping.

**So the "high-res record, low-res SLAM" idea is natively supported and needs no external resizing:**
```bash
--Odom/ImageDecimation 2       # VO runs at half resolution
--Mem/ImagePreDecimation 2     # loop-closure features at half resolution
--Mem/ImagePostDecimation 1    # but FULL resolution RGB+depth is stored in the database
```
`Odom/ImageDecimation` is the cleanest 30 FPS lever in the whole system: it halves the pixel count seen by the detector while leaving the recorded map data untouched, and RTAB-Map handles the keypoint rescaling itself. Note the depth image is decimated independently so it never exceeds the decimated RGB size (`Odometry.cpp:726-745`).

### B11.2 Resolution scaling

⚠️ **Reality check for this project: the dataset is already 640×480.** Derived from this project's own bag info (`localization_eval/topics_matching.txt`): 1095 color frames / 36.632 s = **29.9 Hz**; 4362 / 145.614 s = **30.0 Hz**. Intrinsics `fx 386.92, fy 386.35, cx 322.78, cy 250.28`, RealSense D455f, aligned depth. **The 1920×1080 → 640×480 reduction has already been spent.** Anything below 640×480 starts to hurt depth quality and feature count badly.

For completeness, and using the MEASURED GPU-ORB pyramid numbers as a proxy for how work scales with pixel count:

| Resolution | Pixels | Relative | Detector CPU cost (relative) `[EST]` | Features at constant density | Notes |
|---|---|---|---|---|---|
| 1920×1080 | 2.07 M | 6.75× | ~6.75× (detector is O(pixels)) | very high | never viable on CPU at 30 FPS |
| 1280×720 | 0.92 M | 3.0× | ~3.0× | high | borderline CPU-only `[EXP]` |
| 960×540 | 0.52 M | 1.69× | ~1.69× | moderate | plausible CPU-only |
| **640×480** | **0.31 M** | **1.0×** | **1.0×** | baseline | **this project's actual data** |
| 640×480 + `Odom/ImageDecimation 2` | 0.077 M | 0.25× | ~0.25× | ~4× fewer | the last big lever available here |

Note MEASURED GPU ORB does *not* scale linearly with resolution (0.64 → 0.84 → 1.19 ms for 0.31 → 0.92 → 2.07 MP) because it is launch-latency- and transfer-bound at small sizes. **On the GPU, resolution is nearly free; on the CPU it is the dominant term.** That asymmetry is the real argument for the OpenCV CUDA rebuild if you ever want 1280×720.

---

## B12 — Parameter profiles, with every key verified

### B12.1 Verdict on the user's guessed keys — **all nine exist**

| Guessed key | Exists? | Line | Real default | Note |
|---|---|---|---|---|
| `Kp/MaxFeatures` | ✅ | `Parameters.h:248` | `500` | `<0` means *no extraction at all* |
| `Vis/MaxFeatures` | ✅ | `:708` | `1000` | `0` = no limit |
| `Vis/MinInliers` | ✅ | `:697` | `20` | |
| `Vis/CorType` | ✅ | `:720` | `0` (Features Matching) | `1` = Optical Flow |
| `Odom/Strategy` | ✅ | `:456` | `0` (F2M) | doc string says `5=ORB_SLAM2`; it is **ORB-SLAM3** on a modern build |
| `OdomF2M/MaxSize` | ✅ | `:479` | `2000` | |
| `OdomF2M/BundleAdjustment` | ✅ | `:489`/`:491` | **`1` with g2o, `0` without** | **`0` here** (KF-8) |
| `Mem/STMSize` | ✅ | `:213` | `10` | `unsigned int` |
| `Rtabmap/DetectionRate` | ✅ | `:182` | `1` Hz | |

No invented keys. Two corrections worth flagging: `OdomF2M/BundleAdjustment` and `Optimizer/Strategy` have **build-dependent defaults**, so quoting a single number for them is wrong without stating the build.

### B12.2 Three profiles (640×480 RGB-D, this machine, CPU-only)

**⚠️ Every profile must be preceded by the environment setup in B13.4 — `OMP_NUM_THREADS` alone is worth more than any parameter below.**

#### PERFORMANCE — target: hold 30 FPS odometry with headroom
```bash
# odometry node
--Odom/Strategy 1                  # F2F (default 0=F2M). Flattest latency, best P99.
--Odom/ImageDecimation 2           # default 1. Halve VO resolution; recording unaffected.
--Odom/GuessMotion true            # default. MUST stay on — enables the 40px window matching.
--Odom/KeyFrameThr 0.3             # default
--Odom/ImageBufferSize 1           # default. Never 0.
--Odom/FilteringStrategy 0         # default (no Kalman/particle filter)
--Reg/Strategy 0                   # default: Vis only, NO ICP
--Vis/FeatureType 2                # ORB (default 8 = GFTT/ORB). ORB detect is much cheaper than GFTT.
--Vis/MaxFeatures 400              # default 1000
--Vis/MinInliers 15                # default 20
--Vis/CorType 0                    # default
--Vis/CorNNType 1                  # default FLANN kd-tree. NOT 4 (GPU) — see B5.5.
--Vis/CorGuessWinSize 30           # default 40. Smaller window = fewer BF comparisons.
--Vis/Iterations 150               # default 300 (RANSAC)
--Vis/DepthAsMask true             # default
--Vis/SubPixIterations 0           # default (sub-pixel refinement already off)
--Vis/BundleAdjustment 0           # forced 0 here anyway
--OdomF2M/MaxSize 1000             # default 2000 (only relevant if you keep Strategy 0)

# mapping node
--Rtabmap/DetectionRate 1          # default
--Rtabmap/TimeThr 600              # default 0 (!). Bound the map update.
--Mem/ImagePreDecimation 2         # default 1
--Mem/ImagePostDecimation 1        # default — keep full-res data in the DB
--Kp/MaxFeatures 300               # default 500
--Kp/DetectorStrategy 2            # ORB, to match Vis/FeatureType (required by Mem/UseOdomFeatures)
--Mem/STMSize 30                   # default 10
--RGBD/CreateOccupancyGrid false   # default
--RGBD/ProximityBySpace true       # default
--Rtabmap/PublishPdf false         # default true — the PDF array is |WM| floats per update
--Rtabmap/PublishLikelihood false  # default true — same
```
> `Kp/DetectorStrategy` **must equal** `Vis/FeatureType` or `Mem/UseOdomFeatures` silently turns itself off and you pay for a second feature extraction — `Memory.cpp:833-849`:
> ```cpp
> if(visFeatureType != kpDetectorStrategy)
> {
>     UWARN("%s is enabled, but %s and %s parameters are not the same! Disabling %s...", ...);
>     _useOdometryFeatures = false;
> ```

#### BALANCED — the recommended starting point
```bash
--Odom/Strategy 0                  # F2M (default)
--Odom/ImageDecimation 1           # default — full 640x480
--Reg/Strategy 0                   # default
--Vis/FeatureType 8                # GFTT/ORB (default)
--Vis/MaxFeatures 800              # default 1000
--Vis/MinInliers 20                # default
--Vis/CorGuessWinSize 40           # default
--OdomF2M/MaxSize 2000             # default
--Rtabmap/DetectionRate 1          # default
--Rtabmap/TimeThr 700              # default 0 (!)
--Kp/MaxFeatures 500               # default
--Kp/DetectorStrategy 8            # match Vis/FeatureType
--Mem/STMSize 20                   # default 10
```

#### ACCURACY — offline / bag replay slower than 1.0×, NOT realtime
```bash
--Odom/Strategy 0
--Reg/Strategy 0                   # still Vis; ICP only for loop-closure refinement
--RGBD/NeighborLinkRefining true   # default false — refines odometry links with Reg/Strategy
--Vis/FeatureType 1                # SIFT — REQUIRES OpenCV>=4.4 and the KF-8 fix
--Vis/MaxFeatures 1500
--Vis/MinInliers 25
--Kp/DetectorStrategy 1            # SIFT, must match
--Kp/MaxFeatures 1500
--SIFT/RootSIFT true               # Parameters.h:291, default false
--Rtabmap/DetectionRate 3          # denser nodes
--Rtabmap/TimeThr 0                # no budget
--RGBD/LinearUpdate 0.05
--RGBD/AngularUpdate 0.05
--RGBD/ProximityBySpace true
--RGBD/ProximityPathMaxNeighbors 10   # default 0 (one-to-many proximity DISABLED by default)
--Optimizer/Strategy 2             # GTSAM — REQUIRES the KF-8 fix + GTSAM installed
--Optimizer/Iterations 20
--Optimizer/Robust false
--RGBD/OptimizeMaxError 3.0        # default
--OdomF2M/BundleAdjustment 1       # g2o — REQUIRES the KF-8 fix + g2o installed
```

---

## B13 — Compiler / build

### B13.1 Current flags — verified, and thinner than you'd expect

`CMakeLists.txt:36-41` is the **only** thing that sets an optimisation level, and it does so indirectly:
```cmake
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
IF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    IF("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    ENDIF("${CMAKE_BUILD_TYPE}" STREQUAL "")
ENDIF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")
```
**RTAB-Map sets no `-O`, no `-march`, no `-mtune`, no `-flto`, and no `INTERPROCEDURAL_OPTIMIZATION` anywhere.** The only `-O3`/`-march` strings in the entire tree are in `cmake_modules/android.toolchain.cmake:1399-1400` and `:1259-1270`, which are irrelevant here. Optimisation comes purely from CMake's built-in `CMAKE_CXX_FLAGS_RELEASE` (`-O3 -DNDEBUG` on GCC).

⚠️ **That default only fires for Makefile generators with an empty build type.** With Ninja, or with colcon not passing `--cmake-args -DCMAKE_BUILD_TYPE=Release`, **you can silently get an unoptimised build**. This project's own notes do pass it (`BUILD_CONDA_NOTES.md`: `-DCMAKE_BUILD_TYPE=Release`), but verify at configure time — `CMakeLists.txt:1348` prints `CMAKE_CXX_FLAGS` for exactly this reason.

Complete list of `CMAKE_CXX_FLAGS` mutations:
```cmake
 65:  ADD_DEFINITIONS( "-Wall -Wtype-limits" )
 66:  ADD_DEFINITIONS("-Wno-unknown-pragmas")     # <-- masks unrecognized #pragma omp
101:  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
273:  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
854:  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")   # if Qt6 / g2o-cpp11 / Torch / MRPT
865:  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")   # if PCL > 1.9.1 / g2o / CCCoreLib / Open3D  <-- what applies here
878:  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
```
Line 66 matters: with `-Wno-unknown-pragmas`, if OpenMP is not found **every `#pragma omp` compiles to a silent no-op with no warning.**

`corelib/src/CMakeLists.txt` has **no per-target compile flags at all** — no `target_compile_options`, no `set_source_files_properties`. Everything is global.

ROS 2 packages: only `-Wall -Wextra -Wpedantic` plus feature defines. ⚠️ `rtabmap_slam/CMakeLists.txt:48` and `rtabmap_sync/CMakeLists.txt:47` do `add_compile_options(-bigobj)` **unconditionally** — that is an MSVC flag; GCC's spelling is `-Wa,-mbig-obj`. Harmless in practice but wrong.

### B13.2 `-march=native` — RTAB-Map deliberately refuses it, and it is right to

`CMakeLists.txt:241-252` warns explicitly:
```cmake
    MESSAGE(WARNING "PCL compile options contain \"-march=native\", make sure all libraries using Eigen are also compiled with that flag to avoid some segmentation faults (with gdb referring to some Eigen functions).")
```
**This is a real ABI hazard, not paranoia.** Eigen's `alignas` and fixed-size vectorisable types change size/alignment with the active ISA. If `rtabmap_core` is built with `-march=native` (AVX2, 32-byte alignment) while the PCL/OpenCV/g2o binaries it links against were built for a baseline ISA (16-byte), every `Eigen::Matrix4f` crossing that boundary is misaligned → segfaults inside Eigen, with stack traces that look like compiler bugs.

**Recommendation:** do **not** add `-march=native` to RTAB-Map alone. Either (a) leave it off, or (b) rebuild the *entire* Eigen-touching stack (PCL, OpenCV, g2o/GTSAM, RTAB-Map) with the identical flag. Given that AVX2 is the ceiling here (no AVX-512), the upside is modest anyway: `[EST]` **3–10 %** on the float-heavy paths, not the 2× people imagine.

If you want ISA gains safely, target a specific *baseline* consistently rather than `native`:
```cmake
-DCMAKE_CXX_FLAGS="-O3 -mavx2 -mfma -mbmi2 -mpopcnt"   # applied to EVERY component, or none
```
**No AVX-512** — Alder Lake fuses it off, confirmed by `grep -o "avx512" /proc/cpuinfo` returning nothing.

### B13.3 `-ffast-math` is genuinely dangerous here

`-ffast-math` implies `-ffinite-math-only`, which makes `isnan`/`isinf` **unconditionally false**. RTAB-Map's correctness depends on NaN/Inf checks in exactly the places that matter:
- `uIsFinite(...)` is used all over the depth and keypoint-3D path (e.g. `RegistrationVis.cpp:1046`, `:1255`).
- `Memory::computeLikelihood` / `BayesFilter` divide by sums that can be zero.
- Invalid depth is represented as `0` or `NaN` and filtered by finiteness tests in `util3d::generateKeypoints3DDepth`.
- `RegistrationIcp.cpp` removes NaN normals explicitly before point-to-plane.

With `-ffast-math`, invalid depth silently becomes valid 3-D points, RANSAC gets poisoned, and covariances go non-finite. It also breaks `Odometry`'s `covariance.at<double>(0,0)>=9999` reset sentinel (`RtabmapThread.cpp:511`). **Do not use `-ffast-math`, `-Ofast`, or `-funsafe-math-optimizations` anywhere in this stack.**

`-flto` / IPO is safe in principle but must be applied consistently across the whole link; with ROS 2 + conda prebuilt binaries that is impractical. Expected gain `[EST]` 2–5 %. Low priority.

### B13.4 🔴 OpenMP thread count — the highest-value, lowest-effort change on this machine

**MEASURED on this machine:**
```
FIRST parallel region (thread-pool spawn) : 10.508 ms     <- one-time startup spike
WARM parallel region, default 20 threads  :  3.7858 ms    <- PER PARALLEL REGION
WARM parallel region,  2 threads          :  0.0053 ms
WARM parallel region,  4 threads          :  0.0015 ms
WARM parallel region,  8 threads          :  0.0033 ms
```
A single `#pragma omp parallel for` fork/join barrier costs **3.79 ms** with the default 20 logical CPUs — **11 % of the entire 33.33 ms budget, spent on a barrier rather than computation.** Three parallel regions per frame = 11 ms = 30 FPS is dead on arrival. Capping at 4–8 threads makes it **~1000× cheaper**.

**Where does RTAB-Map actually enter OpenMP regions?** I verified this rather than assuming. Complete inventory of active `#pragma omp` in `corelib`:

| File:line | Loop | On the RGB-D VO critical path? |
|---|---|---|
| `util3d_surface.cpp:3584, 3625, 3701, 3767, 3847` | `adjustNormalsToViewPoint*` — normal orientation | ❌ laser-scan / cloud paths only |
| `util3d_filtering.cpp:1465` | `proportionalRadiusFilteringImpl` per-point radius search | ❌ not on ICP's `commonFiltering` path |
| `clams/discrete_depth_distortion_model.cpp:285, 299` | per-row depth undistortion | ❌ CLAMS calibration only |
| `icp/libpointmatcher.h:512` | `KDTreeMatcherIntensity::findClosests` | ❌ not compiled (`RTABMAP_POINTMATCHER` off) |
| `camera/CameraRealSense.cpp:161` | depth/color align | ❌ not compiled |
| `rtflann/algorithms/nn_index.h` (20 pragmas), `lsh_index.h` (8) | knn/radius search over queries | ❌ **kd-tree is what's used, and `kdtree_index.h`'s 20 pragmas are ALL commented out** (lines 384–639) |

And I explicitly checked the VO path:
```
$ grep -n "#pragma omp" util3d_features.cpp util3d_motion_estimation.cpp util2d.cpp util3d.cpp \
      util3d_correspondences.cpp util3d_registration.cpp Features2d.cpp RegistrationVis.cpp \
      Memory.cpp VWDictionary.cpp FlannIndex.cpp
(no output)
```
**→ RTAB-Map's own code enters ZERO OpenMP regions on the RGB-D visual-odometry critical path.** That is genuinely good news and it is a *measured-negative* finding worth stating plainly.

**But the risk is real in three other places, and you must cap threads anyway:**

1. **ICP (`Reg/Strategy` 1 or 2, or `icp_odometry`).** `util3d::commonFiltering` → `util3d::computeNormals` → `util3d_surface.cpp:2838`:
   ```cpp
   #ifdef PCL_OMP
   	pcl::NormalEstimationOMP<PointT, pcl::Normal> n;
   #else
   	pcl::NormalEstimation<PointT, pcl::Normal> n;
   #endif
   ```
   `PCL_OMP` is added by `CMakeLists.txt:276` whenever OpenMP is found and the `PCL_OMP` option is ON (**default ON**, `:226`). So **every point-to-plane ICP call pays +3.79 ms of pure barrier at 20 threads.** Add the 3-D lidar `adjustNormalsToViewPoint*` regions and you can pay it several times per frame.
2. **PCL's own internal OpenMP** in filters and estimators RTAB-Map calls indirectly — same runtime, same barrier cost.
3. **OpenCV's parallel backend.** On this machine the only OpenCV present (OpenVINO 2021 bundle, 4.5.3) reports `Parallel framework: pthreads` — verified via `strings libopencv_core.so`. So OpenCV here does *not* use OpenMP. **But the conda/robostack OpenCV 4.11 used on the original build PC may differ**, and either way OpenCV's pthread pool also defaults to all 20 CPUs. `cv::setNumThreads()` is the control, and **RTAB-Map never calls it** — verified:
   ```
   $ grep -rn "cv::setNumThreads\|omp_set_num_threads\|OMP_NUM_THREADS" --include=*.cpp --include=*.h rtabmap_ws/src/rtabmap/
   (no output)
   ```
   **RTAB-Map leaves every library thread pool at its default. That is the bug.**

**The fix — do this before touching any SLAM parameter:**
```bash
# Environment (simplest, works for both standalone and ROS 2 nodes)
export OMP_NUM_THREADS=6
export OMP_WAIT_POLICY=PASSIVE     # don't spin-burn cores between regions
export OMP_PROC_BIND=close
export OPENBLAS_NUM_THREADS=1      # Eigen/BLAS backends: keep them out of the way
export MKL_NUM_THREADS=1
export OPENCV_FOR_THREADS_NUM=6    # OpenCV's env equivalent of cv::setNumThreads
```
plus, in-process, a startup warm-up + explicit caps — see `perf_patches/04-thread-pool-caps.cpp` in this directory. Expected gain: **`0` if you never touch ICP/lidar (the VO path has no OpenMP), up to `3.8 ms × regions-per-frame` if you do** — and it removes a 10.5 ms first-frame spike unconditionally.

### B13.5 A measured non-issue, for the record

`UTimer` uses `gettimeofday` on Unix (`utilite/src/UTimer.cpp:83`). `RegistrationVis.cpp:1244-1272` calls `bruteForceTimer.restart()` / `.ticks()` / `.elapsed()` **inside the per-keypoint matching loop** — roughly 3 clock reads × ~1500 keypoints = 4500 calls/frame. I was concerned WSL2 might fall back from vDSO to a syscall, which would make this cost milliseconds.

**MEASURED by me on this machine:**
```
gettimeofday   : 15.71 ns/call
clock_gettime  : 15.05 ns/call
```
vDSO works normally under WSL2. 4500 × 15.7 ns = **0.07 ms/frame**. **Not a problem — no action needed.** (Reported so nobody else spends time on it.)

### B13.6 A real, cheap CPU win that *is* worth fixing

`RegistrationVis.cpp:1275` constructs a `cv::BFMatcher` **inside** the per-keypoint loop:
```cpp
	for(unsigned int i = 0; i < cornersProjectedMat.rows; ++i)
	{
		...
			cv::BFMatcher matcher(descriptors.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR, _nnType==5);
```
(and again at `:1123`). That is ~1000–1500 matcher constructions per frame, each with its own allocation, plus `descriptors.resize()` and per-row `copyTo` inside the loop. Hoisting the matcher out of the loop is behaviour-preserving. `[EST]` **0.2–0.8 ms/frame** — see `perf_patches/05-hoist-bfmatcher.diff`.

---

## B14 — ROS 2 overhead

### B14.1 Composability — the enabler exists, and is never used

Every node in the pipeline is a registered component:
- `rtabmap_sync`: `rgbd_sync.cpp:290`, `stereo_sync.cpp:222`, `rgb_sync.cpp:201`, `rgbdx_sync.cpp:322`
- `rtabmap_odom`: `rgbd_odometry.cpp:918`, `stereo_odometry.cpp:1081`, `icp_odometry.cpp:842`
- `rtabmap_slam`: `CoreWrapper.cpp:5096`
- `rtabmap_util`: 12 more

**But `use_intra_process_comms` appears ZERO times in the repository**, and there is **no `ComposableNodeContainer` in any launch file**. Every hop is a full DDS serialize/deserialize of RGB+depth.

The publishers that would benefit already use the move-friendly form — `rgbd_sync.cpp:174, 266` and `stereo_sync.cpp:160, 198` do `publish(std::move(msg))` with a `UniquePtr`. ⚠️ **`RGBDXSync` does not** (`rgbdx_sync.cpp:185` publishes by const-ref, after N full struct copies at `:183-184`) — that one cannot be zero-copied as written.

⚠️ Caveat that limits the payoff: the pipeline uses `image_transport::SubscriberFilter` everywhere, and **`image_transport` subscriptions do not participate in intra-process comms in Humble.** So only the `rgbd_image` (`rtabmap_msgs::msg::RGBDImage`) hop and the `odom`/`odom_info` hops can actually be zero-copied. Enabling composition still helps — it removes process boundaries and lets you control thread pools centrally — but do not expect it to eliminate all copies.

⚠️ And: use `component_container_mt`, not `component_container`. `rgbd_sync` standalone is `rclcpp::spin()` (single-threaded, `RGBDSyncNode.cpp:35`), whereas `rgbd_odometry` and `rtabmap` use `MultiThreadedExecutor` (`RGBDOdometryNode.cpp:80-82`, `CoreNode.cpp:88-91`) and rely on separate callback groups. Composing them into a single-threaded container collapses that design.

### B14.2 Sync policies and queue defaults — verified

| Node | Policy default | `topic_queue_size` | `sync_queue_size` | `approx_sync` | `approx_sync_max_interval` |
|---|---|---|---|---|---|
| `rgbd_sync` (`rgbd_sync.cpp:57-78`) | ApproximateTime | 10 | 10 | **true** | 0.0 (unbounded) |
| `stereo_sync` (`stereo_sync.cpp:53-73`) | **ExactTime** | 10 | 10 | **false** | 0.0 |
| `rgb_sync` (`rgb_sync.cpp:59-74`) | ApproximateTime | 10 | 10 | true | 0.0 |
| `rgbdx_sync` (`rgbdx_sync.cpp:45-65`) | ApproximateTime | 10 | 10 | true | 0.0 |
| `CommonDataSubscriber` (used by `rtabmap`) (`CommonDataSubscriber.cpp:33-49, 384-402`) | context-dependent (`:510-523`: false for scan-only and for stereo, true otherwise) | 10 | 10 | context | 0.0 |
| `rgbd_odometry` (`rgbd_odometry.cpp:95-107`) | ApproximateTime | 10 | **5** (≠ 10!) | true | 0.0 |
| `rtabmap.launch.py` (`:436-452`) | — | 10 | 10 | **'false'** | 0.0 |

Two traps: `rgbd_odometry`'s `sync_queue_size` default is **5**, not 10; and `rtabmap.launch.py` sets `approx_sync:='false'` at launch level, overriding `CommonDataSubscriber`'s `true`. **`approx_sync_max_interval=0.0` means "any interval is acceptable"** — for a 30 Hz camera set it to ≤ 0.01 s so badly-paired frames are rejected rather than fed to VO. This project's own scripts already do: `run_rtabmap_four_bags.py:81-93` uses `approx_sync:=true approx_sync_max_interval:=0.05` — 50 ms is **1.5 frame periods at 30 Hz**, too loose. Tighten to 0.01.

### B14.3 The real per-frame costs, ranked

1. **No intra-process comms** (B14.1) — every hop is a full serialize/deserialize.
2. 🔴 **An unconditional full-image `copyTo` into a "mosaic", even for a single camera.** `rtabmap_odom/src/nodelets/rgbd_odometry.cpp:539-546`:
   ```cpp
   	if(rgb.empty())
   	{
   		rgb = cv::Mat(imageHeight, imageWidth*cameraCount, ptrImage->image.type());
   	}
   	...
   	ptrImage->image.copyTo(cv::Mat(rgb, cv::Rect(i*imageWidth, 0, imageWidth, imageHeight)));
   ```
   and the same for depth at `:553`; the identical pattern is repeated in `rtabmap_conversions/src/MsgConversion.cpp:2221-2248`. At 640×480 that is 0.9 MB RGB + 0.6 MB depth memcpy **per frame, per node** — `[EST]` 0.1–0.3 ms each, so 0.2–0.6 ms across odom + slam. Pure overhead when `rgbd_cameras=1`.
3. **`cv_bridge::cvtColor` per frame** whenever the encoding is not already `bgr8`/`mono8` (`rgbd_odometry.cpp:517-525`, `MsgConversion.cpp:2213-2218`). **Fix at the source: publish `bgr8` from the camera driver.**
4. 🔴 **Blocking TF lookups with `wait_for_transform=0.2`** (`rtabmap.launch.py:439`), once per camera per frame, on the callback thread (`rgbd_odometry.cpp:489`, `MsgConversion.cpp:2177`). If TF is ever late this is a **200 ms stall** — a catastrophic P99 contributor. Lower it to 0.05 for a 30 FPS pipeline and make sure `tf_static` is published before the camera starts.
5. **`/odom_info` (full)** per-point loops (see B3.3). Use `/odom_info_lite`.
6. **`rtabmap_viz:=true` is the launch default** (`rtabmap.launch.py:417`). The GUI is a heavy `MapData` + image consumer that adds real publish cost to the `rtabmap` node. **Set `rtabmap_viz:=false` for any performance measurement.**
7. **`compressed:=true` is a trap** (`rtabmap.launch.py:474`). It does not set in-node transport hints — it spawns separate `image_transport/republish` processes (`:71-84`, `:107-120`) that decode and then **republish full-size raw** on `*_relay` topics (`:60-63`). That is strictly worse than in-node decode.
8. `odom_sensor_sync:=true` adds a second `getMovingTransform` TF lookup per camera per frame (`MsgConversion.cpp:2187-2194`).
9. `always_check_imu_tf` defaults **true** (`rtabmap.launch.py:501`) → a TF lookup per IMU message (`OdometryROS.cpp:540-545`). At 200 Hz IMU that is 200 TF lookups/s for a rigidly-mounted sensor. **Set `false`.**

### B14.4 Compressed transport is NOT on the critical path by default

Every transport hint defaults to `"raw"`: `rgbd_sync.cpp:112-113` (`rgb_image_transport`/`depth_image_transport`), `rgbd_odometry.cpp:357-358` (`rgb_transport`/`depth_transport`), and the plain `image_transport::TransportHints(this)` ctor elsewhere. Compressed decode only enters via `rtabmap_msgs::msg::RGBDImage`'s `rgb_compressed`/`depth_compressed` fields, handled in `rtabmap_conversions::toCvShare` (`MsgConversion.cpp:200, 216-224`) — i.e. only if you subscribe to `rgbd_image/compressed`. **Don't.**

### B14.5 QoS

Defaults are `RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT` = 0 = Reliable (`OdometryROS.cpp:83, 109`; `CommonDataSubscriber.cpp:389`). **For a 30 FPS live camera, Reliable QoS causes retransmit stalls under load.** Use `qos:=2` (Best Effort) — this project's own scripts already do (`run_rtabmap_four_bags.py:81-93`: `qos:=2 qos_image:=2 qos_camera_info:=2`). All odom output publishers are hard-coded `rclcpp::QoS(1)` (`OdometryROS.cpp:112-121`), which is correct for realtime.

### B14.6 Recommended ROS 2 launch delta

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_viz:=false rviz:=false \
  approx_sync:=true approx_sync_max_interval:=0.01 \
  qos:=2 qos_image:=2 qos_camera_info:=2 \
  topic_queue_size:=5 sync_queue_size:=5 \
  wait_for_transform:=0.05 \
  always_check_imu_tf:=false \
  compressed:=false \
  rgbd_sync:=false            # one fewer process hop unless you need the merged topic
```
⚠️ **None of this can be validated on this machine — there is no ROS 2 here.** These are source-derived recommendations, not measurements. `[EXP]`

---

## B15 — GPU memory transfer strategy under the WSL2 pinned-memory caveat

**MEASURED transfer bandwidth on this machine** (pageable vs pinned):

| Size | H2D pageable | H2D pinned | D2H pageable | D2H pinned |
|---|---|---|---|---|
| 64 KiB | 0.028 ms (2.3 GB/s) | 0.029 ms (2.3 GB/s) | 0.034 ms (1.9) | 0.032 ms (2.0) |
| 640×480 gray (300 KB) | 0.055 ms (5.5) | 0.049 ms (6.3) | 0.068 ms (4.5) | 0.050 ms (6.1) |
| 1280×720 gray (0.9 MB) | 0.123 ms (7.5) | 0.096 ms (9.6) | 0.181 ms (5.1) | 0.093 ms (9.9) |
| 1920×1080 gray (2.0 MB) | 0.237 ms (8.7) | 0.194 ms (10.7) | 0.289 ms (7.2) | 0.190 ms (10.9) |
| 1920×1080 BGR (5.9 MB) | 0.601 ms (10.3) | 0.538 ms (11.6) | 0.725 ms (8.6) | 0.522 ms (11.9) |

Plus: `kernel launch + cudaDeviceSynchronize` = **0.027 ms**; async enqueue alone = **0.0077 ms**; the composite 64 KB-up + kernel + 64 KB-down floor = **0.108 ms**.

### Three rules that follow from the measurements

**1. Pinned memory gives only 1.2–1.5×, not 3–4×. Do not over-promise `cudaHostAlloc`.**
At 2 MB: 8.7 → 10.7 GB/s H2D. On bare-metal PCIe 4.0 ×16 the same comparison is ~6 → ~24 GB/s. WSL2's WDDM paravirtualisation degrades pinned DMA exactly as expected. It is still worth using — the **D2H** case improves most (0.289 → 0.190 ms at 2 MB, 1.5×) — but budget 1.2–1.5×, not more.

**2. Small transfers are pure latency. Halving the size buys nothing.**
64 KiB (≈ 2000 × 256-bit ORB descriptors) costs 0.028 ms up and 0.034 ms down — 2.3 GB/s, i.e. ~4× below the large-transfer rate. **This is the arithmetic that kills GPU descriptor matching** (B5.5): the data you would ship is exactly in the latency-dominated regime.

**3. Budget at most ~5 synchronisation points per frame.** Each costs 0.027 ms; twenty of them is 0.54 ms of pure stall.

### What this means for RTAB-Map specifically

RTAB-Map already has the right architecture for GPU residency: `SensorData` carries `cv::cuda::GpuMat _imageRawGpu` / `_depthOrRightRawGpu` with accessors `imageRawGpu()`, `setImageRawGpu()`, `depthOrRightRawGpu()`, `setDepthOrRightRawGpu()` (`corelib/include/rtabmap/core/SensorData.h:367-372, 428-434`, gated on `HAVE_OPENCV_CUDEV`). `RegistrationVis.cpp:503-540` uploads once, converts to grayscale on the GPU with `cv::cuda::cvtColor`, and **caches the result back into the `SensorData`** so downstream consumers reuse it. `Features2d.cpp:902-930` and `:951-961` then read `data.imageRawGpu()` instead of re-uploading.

**So the correct GPU plan for this machine is: one upload per frame, feed GPU ORB (or GPU GFTT) from the resident `GpuMat`, download only keypoints + descriptors, keep everything else on the CPU.**

Budget for that plan at 640×480 (MEASURED components):
```
H2D upload, pinned, 300 KB gray          0.030 ms
GPU ORB pyramid + blur + FAST + BRIEF    0.58  ms   (0.082 + 0.151 + 0.169 + 0.156, measured)
D2H keypoints + descriptors (~64 KB)     0.029 ms
sync points (≤ 3 × 0.027)                0.08  ms
-----------------------------------------------
TOTAL GPU round trip                     ~0.72 ms   (measured whole-pipeline figure: 0.638 ms
                                                     with per-stage sync, 0.888 ms with one sync)
```
Against a CPU ORB extractor at `[EST]` 4–10 ms for the same work, that is a **5–10× improvement on the largest single term in the odometry budget**, for ~0.7 ms of GPU time and one round trip. It is the only GPU offload that clears the 0.108 ms floor by a wide margin.

**What NOT to do:**
- ❌ Do not upload the depth image unless something on the GPU consumes it. An RGB+depth pair at 1280×720 is ~2.7 MB ≈ 0.3–0.5 ms just to upload.
- ❌ Do not offload descriptor matching (B5.5) — the workload is 10⁴–10⁵ comparisons, the floor is 0.108 ms, CPU POPCNT does it in 0.06–0.15 ms.
- ❌ Do not offload ICP. There is no CUDA ICP in RTAB-Map at all, and writing one would require moving 3–15 k points each way per iteration.
- ❌ Do not offload graph optimisation. It is off the critical path (B10.2) and TORO/g2o/GTSAM are sparse-matrix workloads with poor GPU fit at these sizes.

---

## B16 — Pipeline parallelism: only the genuinely safe overlaps

Real data dependencies inside one odometry frame:
```
image ──► grayscale ──► detect ──► describe ──┐
   │                        │                 ├──► match ──► PnP RANSAC ──► pose
   └──► depth ──────────────┴──► keypoints3D ─┘
```
`describe` needs `detect`'s keypoints; `keypoints3D` needs `detect`'s keypoints **and** depth; `match` needs both descriptor sets; PnP needs matches + 3-D points. **The chain is fundamentally serial within a frame.** The only intra-frame overlap that is safe is `describe` ‖ `keypoints3D` — both consume the keypoints, neither writes shared state. `[EST]` gain 0.1–0.4 ms; **not worth the complexity.**

Safe overlaps that already exist and should be preserved:
- `Kp/Parallelized=true` (`Parameters.h:259`) — `PreUpdateThread` runs `VWDictionary::update()` concurrently with feature extraction in `Memory::createSignature` (`Memory.cpp:4519, 4610, 4769`).
- `Mem/CompressionParallelized=true` (`:229`) — sensor-data compression is multi-threaded.
- The three-stage thread pipeline itself (sensor ‖ odometry ‖ mapping) — this is where the real parallelism lives.
- `DBDriver`'s async save thread.

Safe overlaps worth **adding**:
1. **GPU ORB extraction ‖ CPU depth work.** With a CUDA OpenCV, launch the GPU extraction asynchronously, do `generateKeypoints3D`'s depth lookups on the CPU meanwhile, then sync once. Saves roughly `min(GPU 0.6 ms, CPU depth 0.1 ms)` — marginal, and it adds a sync point. `[EXP]`
2. **Frame N+1 upload ‖ frame N compute** (double-buffered `GpuMat` + two CUDA streams). Hides the 0.030 ms upload entirely. Only meaningful once GPU extraction exists.

Overlaps that look attractive and are **NOT safe**:
- ❌ Running `Memory::createSignature`'s feature extraction concurrently with odometry's — they share the `Feature2D` instance and, under `Mem/UseOdomFeatures=true`, the *point* is that they are the same features (KF-4).
- ❌ Overlapping `Rtabmap::process` iterations — it mutates `Memory`, `VWDictionary` and `_optimizedPoses` without internal locking; it is single-threaded by design.
- ❌ Parallelising the RANSAC loop in `util3d::estimateMotion3DTo2D` — OpenCV's `solvePnPRansac` already handles its own parallelism, and at `Vis/Iterations=300` over ~100 correspondences the work is too small to amortise a fork/join (which, at default thread counts on this box, would cost 3.79 ms — see B13.4).

---

## B17 — Latency budget: RTX 3080 Ti Laptop + i9-12900HK + WSL2

**Scope:** one odometry frame, 640×480 RGB-D (this project's real data), `Reg/Strategy=0` (Vis only, no ICP), F2M, GFTT/ORB, `Kp/MaxFeatures=500` / `Vis/MaxFeatures=1000`.

### B17.1 Evidence classes

- **[M]** measured on this machine (by the coordinator, or by me where noted)
- **[P]** measured earlier in this project, on a *different* machine
- **[EST]** my estimate with stated reasoning
- **[EXP]** needs an experiment

### B17.2 Component budget — CPU-only (today's reality)

| Stage | Best | Typical | Worst | Class | Reasoning |
|---|---|---|---|---|---|
| ROS 2 sync + cv_bridge + mosaic copy | 0.2 | 0.4 | 1.2 | [EST] | 1.5 MB memcpy ×2 nodes (B14.3 #2); worst case includes a `cvtColor` |
| TF lookup | 0.05 | 0.2 | **200** | [EST] | `wait_for_transform=0.2` blocks (B14.3 #4) — the dominant tail risk |
| BGR→GRAY | 0.10 | 0.20 | 0.40 | [EST] | 307 k px, cache-resident |
| **GFTT detect (default)** | **2.5** | **5.0** | **10.0** | [EST] | Harris/min-eigen response over all 307 k px + NMS + sort. The largest single CPU term. ORB instead: 1.0–3.0 ms |
| ORB describe (1000 kp) | 0.8 | 1.5 | 3.0 | [EST] | 8-level pyramid + BRIEF-256 |
| `generateKeypoints3D` (RGB-D) | 0.05 | 0.10 | 0.25 | [EST] | 1000 depth lookups + reproject |
| Project + 2-D kd-tree build | 0.10 | 0.25 | 0.60 | [EST] | `cv::projectPoints` on ≤2000 pts + rtflann 2-D index (`RegistrationVis.cpp:1038, 1079-1081`) |
| **Descriptor matching** | **0.06** | **0.15** | **0.79** | **[M]** | 10⁴–6×10⁴ Hamming comparisons (derived from source, B5.5) mapped onto the measured POPCNT table. Worst = the 400 k row (no guess → wider search) |
| BFMatcher construction overhead | 0.2 | 0.5 | 0.8 | [EST] | ~1500 constructions/frame (B13.6) |
| PnP RANSAC (`Vis/Iterations=300`) | 0.5 | 1.5 | 4.0 | [EST] | 300 iterations × EPnP/Iterative on ~100 corr. |
| Local BA | — | — | — | — | **0 here** — `OdomF2M/BundleAdjustment=0` (KF-8) |
| F2M local-map maintenance | 0.1 | 0.4 | 2.0 | [EST] | worst = keyframe-insertion frames (prune to `OdomF2M/MaxSize`) |
| UTimer instrumentation | 0.07 | 0.07 | 0.07 | **[M]** | 4500 × 15.7 ns — measured by me, negligible |
| OpenMP barriers on VO path | **0** | **0** | **0** | **[M]+verified** | zero `#pragma omp` on the RGB-D VO path (B13.4) |
| **TOTAL (CPU-only, no ICP)** | **~4.8** | **~10.5** | **~23** | | excluding the TF stall |
| **+ 200 ms TF stall** | | | **~223** | | catastrophic, but preventable |

### B17.3 Verdict table

| Question | Verdict | Confidence |
|---|---|---|
| **Mean < 33.33 ms, CPU-only, 640×480 RGB-D, Vis-only?** | ✅ **YES, comfortably.** ~10 ms typical leaves ~3× headroom. This is consistent with [P]: ORB-SLAM3 — a structurally *heavier* per-frame pipeline (local BA + local-map tracking, which RTAB-Map does not do here) — measured **12.4 ms mean** on the same 640×480 data, on a stronger CPU. | **High** |
| **P95 < 33.33 ms?** | ✅ **Probably yes**, ~15–20 ms, *provided* `wait_for_transform` is lowered and thread pools are capped. | Medium |
| **P99 < 33.33 ms?** | ⚠️ **NOT GUARANTEEABLE on this machine.** Three reasons, two of them unfixable from inside: (1) **[M]** `SCHED_FIFO`/`SCHED_RR`/`nice(-10)` all return **EPERM** in WSL2 — you cannot protect the odometry thread; (2) **[M]** P/E cores are indistinguishable (`core_id` flattened to 0..9) so an odometry frame can land on an E-core and run ~1.5–2× slower `[EST]`; (3) the Windows host schedules the whole VM. **The P99 tail is host-controlled.** | **High confidence in the negative** |
| **With ICP on the odometry path (`Reg/Strategy=2`)?** | ❌ **NO.** ICP alone is `[EST]` 12–45 ms for dense 640×480 RGB-D (B7.3). | High |
| **At 1280×720 CPU-only?** | ⚠️ `[EXP]` — detector cost ×3 pushes typical to ~25–30 ms. Mean might hold; P95 would not. | Low |
| **At 1280×720 with GPU ORB (after OpenCV CUDA rebuild)?** | ✅ Likely — **[M]** GPU ORB at 1280×720 is 0.84 ms, replacing `[EST]` 12–30 ms of CPU detect+describe. | Medium |
| **Mapping thread `Timing/Total/ms` < 1000 ms at `DetectionRate=1`?** | ✅ Yes for small maps; ⚠️ degrades as O(\|WM\|²) via the Bayes multiply (KF-3). **Set `Rtabmap/TimeThr=700`** to bound it. | High |

### B17.4 The honest bottom line

> On **this** laptop, in **WSL2**, RTAB-Map RGB-D visual odometry at **640×480 with no ICP** will hold a 30 FPS *average* with real headroom, and will very likely hold P95. **It will not give you a certified P99 under 33.33 ms**, and no amount of RTAB-Map tuning changes that, because the two standard mitigations — real-time scheduling and P-core affinity — are **measured unavailable** in this environment.
>
> If a hard 33.33 ms P99 is a requirement, the answer is not a parameter: it is **bare-metal Linux** (which restores `SCHED_FIFO` and P/E-core topology), or accepting a documented frame-drop rate and instrumenting it (B8.3 #1).
>
> Note also the measurement gap: **this project has never measured a single RTAB-Map timing number.** `run_rtabmap_four_bags.py` records no elapsed time (its ORB-SLAM3 counterpart does), and every `*.db` — which holds the `Statistics` table with all 38 `Timing/*` keys — is gitignored. **Before optimising anything, run B3.3 step 1.**

---

## B18 — CPU-only 30 FPS: the exact recipe ⭐

**This is the most important section for this machine**, because OpenCV here has no CUDA build, no `libopencv_cuda*` exists, and the stale `Version.h` (KF-8) disables every non-OpenCV GPU path as well. **CPU-only is the only configuration that can run here today.**

### B18.1 Exact conditions under which CPU-only 30 FPS holds

| Condition | Required setting | Why |
|---|---|---|
| **Resolution** | **640×480** (native), or 848×480. Add `--Odom/ImageDecimation 2` for margin. | Detector cost is O(pixels); 640×480 is this project's actual data |
| **Detector** | `--Vis/FeatureType 2` + `--Kp/DetectorStrategy 2` (**ORB**) — or keep `8` (GFTT/ORB) if you have headroom | GFTT's whole-image corner response is the biggest single CPU term. **They must be equal** or `Mem/UseOdomFeatures` silently disables itself (`Memory.cpp:833-849`) |
| **Feature count** | `--Vis/MaxFeatures 400-800`, `--Kp/MaxFeatures 300-500` | descriptor + matching cost is linear in this |
| **Odometry mode** | `--Odom/Strategy 0` (F2M) for accuracy, `1` (F2F) for the flattest P99 | |
| **ICP** | `--Reg/Strategy 0` — **OFF**. Never `1` or `2` on the odometry path. | 12–45 ms (B7.3) |
| **Motion guess** | `--Odom/GuessMotion true` (default) + `--Vis/CorGuessWinSize 30-40` | **This is what keeps matching at 10⁴–10⁵ comparisons instead of 10⁶.** Without it, `RegistrationVis.cpp:1478` builds a whole `VWDictionary` per call. |
| **Local BA** | `--OdomF2M/BundleAdjustment 0` (already forced, KF-8) | |
| **Memory** | `--Rtabmap/DetectionRate 1`, `--Rtabmap/TimeThr 700`, `--Mem/STMSize 20-30` | bounds the 1 Hz mapping thread |
| **Queues** | `--Odom/ImageBufferSize 1`, `--Rtabmap/ImageBufferSize 1` (defaults) | never 0 |
| **Thread pools** | `OMP_NUM_THREADS=6`, `cv::setNumThreads(6)` | **[M]** 3.79 ms/barrier at 20 threads vs 0.003 ms at 8 |
| **Build** | `-DCMAKE_BUILD_TYPE=Release` **verified at configure time** | B13.1 — it is not guaranteed |
| **ROS 2** | `rtabmap_viz:=false`, `qos:=2`, `wait_for_transform:=0.05`, `approx_sync_max_interval:=0.01`, `/odom_info_lite` only | B14.6 |
| **Filesystem** | `database_path` on the **Linux** filesystem, never `/mnt/c/...` | 9p is dramatically slower than the ext4 VHD |

### B18.2 The complete runnable command

```bash
# ---- 1. Thread pool caps. Do this FIRST; it is worth more than any SLAM parameter. ----
export OMP_NUM_THREADS=6
export OMP_WAIT_POLICY=PASSIVE
export OMP_PROC_BIND=close
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OPENCV_FOR_THREADS_NUM=6

# ---- 2. Keep odometry and mapping off each other's cores (affinity WORKS; SCHED_FIFO does not) ----
# taskset -c 0-7  <odometry node>
# taskset -c 8-15 <mapping node>

# ---- 3. Launch ----
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_viz:=false rviz:=false \
  frame_id:=camera_color_optical_frame \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  approx_sync:=true approx_sync_max_interval:=0.01 \
  qos:=2 qos_image:=2 qos_camera_info:=2 \
  topic_queue_size:=5 sync_queue_size:=5 \
  wait_for_transform:=0.05 always_check_imu_tf:=false compressed:=false \
  database_path:=$HOME/rtabmap_perf.db \
  odom_args:="--Odom/Strategy 0 \
              --Odom/ImageDecimation 1 \
              --Odom/GuessMotion true \
              --Odom/ImageBufferSize 1 \
              --Odom/FilteringStrategy 0 \
              --Reg/Strategy 0 \
              --Vis/FeatureType 2 \
              --Vis/MaxFeatures 600 \
              --Vis/MinInliers 15 \
              --Vis/CorType 0 \
              --Vis/CorNNType 1 \
              --Vis/CorGuessWinSize 35 \
              --Vis/Iterations 200 \
              --Vis/DepthAsMask true \
              --OdomF2M/MaxSize 1500 \
              --OdomF2M/BundleAdjustment 0" \
  args:="-d \
         --Rtabmap/DetectionRate 1 \
         --Rtabmap/TimeThr 700 \
         --Rtabmap/ImageBufferSize 1 \
         --Rtabmap/PublishPdf false \
         --Rtabmap/PublishLikelihood false \
         --Rtabmap/StatisticLogged true \
         --Kp/DetectorStrategy 2 \
         --Kp/MaxFeatures 400 \
         --Kp/TfIdfLikelihoodUsed true \
         --Kp/IncrementalFlann true \
         --Kp/Parallelized true \
         --Mem/STMSize 25 \
         --Mem/UseOdomFeatures true \
         --Mem/ImagePreDecimation 1 \
         --Mem/ImagePostDecimation 1 \
         --Bayes/FullPredictionUpdate false \
         --RGBD/CreateOccupancyGrid false \
         --RGBD/LinearUpdate 0.1 \
         --RGBD/AngularUpdate 0.1"

# ---- 4. Measure (do this BEFORE tuning anything further) ----
ros2 topic hz /odom
ros2 topic echo /odom_info_lite --field time_estimation
ros2 topic echo /rtabmap/info --field stats_values      # all Timing/* keys
```

### B18.3 Expected result

`[EST]`, grounded in [P] (ORB-SLAM3 at 12.4 ms on the same data, structurally heavier pipeline) and the [M] component numbers:
- `time_estimation` **mean 6–12 ms**, P95 **12–20 ms**
- `/odom` at a stable 30 Hz with **< 1 %** drops
- `Timing/Total/ms` on the mapping thread **50–300 ms** at 1 Hz, bounded by `TimeThr=700`
- **P99: expect occasional 35–60 ms frames from E-core migration and host scheduling. This is expected and cannot be eliminated in WSL2.**

### B18.4 Escalation order if 30 FPS is not met

1. `--Odom/ImageDecimation 2` (biggest single lever; recording unaffected — B11.1)
2. `--Vis/MaxFeatures 400 --Kp/MaxFeatures 300`
3. `--Odom/Strategy 1` (F2F)
4. `--Vis/Iterations 100`
5. `--Vis/CorGuessWinSize 25`
6. Apply `perf_patches/05-hoist-bfmatcher.diff` (`[EST]` 0.2–0.8 ms)
7. Only then consider the OpenCV CUDA rebuild (§B19)

---

## B19 — Recommended architecture for THIS GPU

**Honest answer: mostly CPU, with exactly one GPU offload — and that offload requires a prerequisite that is not currently met.**

### B19.1 Why "mostly CPU"

Given the measurements:
- GPU offload floor is **0.108 ms** per round trip; **0.027 ms** per sync.
- Descriptor matching in RTAB-Map is **10⁴–10⁵ comparisons** (proven from `RegistrationVis.cpp:1011-1016, 1090, 1275`), which CPU POPCNT does in **0.06–0.15 ms**. GPU loses.
- There is **no CUDA ICP** and no CUDA graph optimiser in RTAB-Map.
- Dense stereo has **no GPU path at all** (`stereo/StereoBM.cpp`, `stereo/StereoSGBM.cpp`).
- The 3080 Ti Laptop is 58 SM / 512 GB/s / **137 W TGP** — roughly **35–45 %** of a desktop 4090. Literature GPU speedups must be scaled down accordingly.

### B19.2 The one thing worth moving to the GPU

**Feature extraction.** MEASURED: **0.64 ms @ 640×480, 0.84 ms @ 1280×720, 1.19 ms @ 1920×1080** for the full ORB pipeline (upload → pyramid → blur → FAST-9 → orientation+BRIEF → download). Conservative **5–10×** vs an optimised CPU extractor. It is the largest single term in the odometry budget and it clears the 0.108 ms floor by 6×.

In RTAB-Map this is reached through `ORB/Gpu=true` (`Parameters.h:322` → `Features2d.cpp:1549-1585, 1600-1620, 1640-1668`, `cv::cuda::ORB`) and/or `GFTT/Gpu=true` (`:313` → `Features2d.cpp:2085-2101, 2127-2140`, `cv::cuda::createGoodFeaturesToTrackDetector`). Both are gated on `HAVE_OPENCV_CUDAFEATURES2D` / `HAVE_OPENCV_CUDAIMGPROC`.

### B19.3 The two prerequisites, in order

1. **Fix KF-8** — delete/regenerate `corelib/include/rtabmap/core/Version.h`. Without this, `RTABMAP_NONFREE`, `RTABMAP_G2O`, `RTABMAP_ORB_SLAM` etc. stay off no matter what you install. *(The OpenCV-CUDA macros are OpenCV-owned and unaffected, but everything else you'd want is.)*
2. **Rebuild OpenCV with CUDA** (exact cmake line in B5.4): `-DWITH_CUDA=ON -DCUDA_ARCH_BIN=8.6 -DOPENCV_EXTRA_MODULES_PATH=.../opencv_contrib/modules -DBUILD_opencv_cudafeatures2d=ON -DBUILD_opencv_cudaimgproc=ON -DBUILD_opencv_cudev=ON`. **60–120 min** build `[EST]`, 8–15 GB disk. Then rebuild RTAB-Map against it.

### B19.4 Target architecture

```
┌─ CPU core set A (odometry) ──────────────────────────────────────┐
│  cv_bridge (bgr8 direct, no cvtColor)                            │
│  ONE H2D upload (pinned) ──────────────────────┐                 │
│  ... CPU depth work in parallel ...            │                 │
│  match (POPCNT, 40px window)  ◄── descriptors ─┤                 │
│  PnP RANSAC                                    │                 │
│  pose out                                      │                 │
└────────────────────────────────────────────────┼─────────────────┘
                                                 ▼
                                 ┌─ GPU (RTX 3080 Ti, 1 stream) ──┐
                                 │  cuda::cvtColor  (cached GpuMat)│
                                 │  cuda::ORB / cuda::GFTT         │
                                 │  ≈0.64 ms @ 640x480  [MEASURED] │
                                 │  ONE D2H: keypoints+descriptors │
                                 └─────────────────────────────────┘
┌─ CPU core set B (mapping, 1 Hz) ─────────────────────────────────┐
│  Memory::update  →  VWDictionary (FLANN, CPU)                    │
│  BayesFilter (dense NxN — bound with Rtabmap/TimeThr)            │
│  TORO/g2o/GTSAM optimiser (CPU, sparse)                          │
│  DBDriver async SQLite thread                                    │
└──────────────────────────────────────────────────────────────────┘
  OMP_NUM_THREADS=6, cv::setNumThreads(6), warm-up region at startup
```

Settings once both prerequisites are met:
```bash
--Vis/FeatureType 2 --Kp/DetectorStrategy 2 --ORB/Gpu true
# or: --Vis/FeatureType 8 --Kp/DetectorStrategy 8 --GFTT/Gpu true --ORB/Gpu true
--Vis/CorNNType 1        # NOT 4 — CPU matching wins (B5.5)
--Kp/NNStrategy 1        # NOT 4
--FAST/Gpu false         # NEVER true — UFATAL at Features2d.cpp:1938
--Stereo/Gpu false       # RGB-D; irrelevant
--Vis/CorFlowGpu false   # only if you switch to Vis/CorType 1
```

### B19.5 Is the rebuild worth it?

- **If 640×480 CPU-only already meets your needs (B18): NO.** The budget is ~10 ms typical against 33.33 ms; you would spend 2 hours of build time and take on a custom OpenCV to reclaim ~4–8 ms you do not need.
- **If you want 1280×720 or higher, or you want to run SAM-6D / other GPU work concurrently and need CPU headroom: YES.** GPU extraction is nearly resolution-independent (0.64 → 1.19 ms across a 6.75× pixel range), while CPU extraction is linear. That asymmetry is the whole argument.

---

## B20 — Phase table

| Phase | File | Class / Function | Parameter | Change | Expected gain | Difficulty | Accuracy impact | Risk |
|---|---|---|---|---|---|---|---|---|
| **0** 🔴 | `corelib/include/rtabmap/core/Version.h` | — (stale generated header, git-tracked) | — | Delete it; let `CMakeLists.txt:1129` regenerate. Or reorder `corelib/src/CMakeLists.txt:859-861` to put `${CMAKE_CURRENT_BINARY_DIR}/include` first. | **Unblocks g2o/GTSAM/libpointmatcher/nonfree/Torch/Python/ORB-SLAM entirely.** No direct ms. | Low | none (enabling) | Rebuild required; new deps may not be installed — verify the regenerated header |
| **1** 🔴 | *environment* | OpenMP / OpenCV thread pools | `OMP_NUM_THREADS`, `cv::setNumThreads` | Set to 6; `OMP_WAIT_POLICY=PASSIVE`; warm up one parallel region at startup | **[M]** 3.79 → 0.003 ms **per parallel region**; removes a 10.5 ms first-frame spike. 0 ms on the pure-Vis path (no OpenMP there — verified), **3.8 ms × regions** on any ICP/lidar path | **Trivial** | none | none |
| **2** | *build* | CMake | `CMAKE_BUILD_TYPE` | Verify `Release` actually reached the compiler (`CMakeLists.txt:1348` prints the flags) | up to **10×** if it was accidentally unoptimised | Trivial | none | none |
| **3** | *launch* | `rtabmap.launch.py` | `wait_for_transform` | 0.2 → **0.05** | removes a **200 ms** P99 stall | Trivial | none | TF must be ready; test with a cold start |
| **4** | *launch* | `rtabmap.launch.py:417` | `rtabmap_viz` | `true` → **false** | `[EST]` 1–5 ms/frame on the mapping node | Trivial | none | lose the GUI |
| **5** | *launch* | `rtabmap.launch.py` | `qos`, `approx_sync_max_interval` | `qos:=2` (Best Effort), interval `0.05` → **0.01** | removes retransmit stalls; rejects mis-paired frames | Trivial | slightly fewer frames accepted | none |
| **6** | `Parameters.h:180` | `Rtabmap::process` | `Rtabmap/TimeThr` | **0 → 700** ms | bounds `Timing/Total/ms` as the map grows (KF-3); prevents the 1 Hz queue from overflowing | Trivial | older nodes move to LTM sooner → slightly fewer loop candidates | map coverage shrinks if set too low |
| **7** | `Parameters.h:704/254` | `Feature2D::create` | `Vis/FeatureType` + `Kp/DetectorStrategy` | `8` (GFTT/ORB) → **`2` (ORB)**, both together | `[EST]` **1.5–7 ms/frame** — GFTT's whole-image response is the biggest CPU term | Low | ORB corners are less evenly spread than GFTT → slightly noisier odometry | **They must be equal** or `Mem/UseOdomFeatures` self-disables (`Memory.cpp:833-849`) |
| **8** | `Parameters.h:474` | `Odometry::process` `:722-768` | `Odom/ImageDecimation` | **1 → 2** | `[EST]` **~60 %** of detector+descriptor time | Trivial | fewer/coarser features; recorded data unaffected | fewer inliers in low-texture scenes |
| **9** | `Parameters.h:708/248` | — | `Vis/MaxFeatures`, `Kp/MaxFeatures` | `1000/500` → **600/400** | `[EST]` 1–3 ms/frame | Trivial | fewer inliers | raise `Vis/MinInliers` risk |
| **10** | `corelib/src/RegistrationVis.cpp:1123, 1275` | `RegistrationVis::computeTransformationImpl` | — (code) | Hoist `cv::BFMatcher` out of the per-keypoint loop (`perf_patches/05-hoist-bfmatcher.diff`) | `[EST]` **0.2–0.8 ms/frame** | Low | **none** (behaviour-preserving) | must keep the `_nnType==5` cross-check flag correct |
| **11** | `corelib/src/RegistrationVis.cpp` + `RegistrationInfo.h` | stage timers | — (code) | Add the 8 stage timers from B3.4 (`perf_patches/01-registrationvis-stage-timers.diff`) | 0 ms — **enables everything else** | Low | none | ~0.1 ms of `gettimeofday` (measured 15.7 ns/call) |
| **12** | `rtabmap_odom/src/OdometryROS.cpp:461-483` | `OdometryROS::processData` | — (code) | Publish `droppedMsgs_`/`processedMsgs_` (currently the warning is commented out) | 0 ms — **the missing P99 metric** | Low | none | none |
| **13** | *build* | OpenCV | `-DWITH_CUDA=ON -DCUDA_ARCH_BIN=8.6` + contrib | full rebuild (B5.4) | unlocks Phase 14 | **High** (60–120 min build, custom OpenCV to maintain) | none | version skew with conda/ROS OpenCV; must rebuild RTAB-Map too |
| **14** | `Parameters.h:322` / `:313` | `ORB::generateKeypointsImpl` `Features2d.cpp:1600`, `GFTT` `:2127` | `ORB/Gpu`, `GFTT/Gpu` | `false` → **true** | **[M]** GPU ORB **0.64 ms** @640×480 / 0.84 @720p / 1.19 @1080p; replaces `[EST]` 4–10 ms CPU. **5–10×** on the largest term | Medium (after Phase 13) | none (same algorithm) | **Never set `FAST/Gpu=true`** — `UFATAL` at `Features2d.cpp:1938` |
| **15** | `Parameters.h:410` | `Optimizer::create` `Optimizer.cpp:79` | `Optimizer/Strategy` | TORO(0)/100 iters → **GTSAM(2)/20 iters + `GTSAM/Incremental true`** | `[EST]` 2–10× on `Timing/Map_optimization/ms` for large graphs; off the 30 FPS path but fixes mapping-thread spikes | Medium (needs Phase 0 + GTSAM installed) | **better** (iSAM2 > TORO) | new dependency |
| **—** ❌ | `Parameters.h:721/241` | `VWDictionary.cpp:873, 1200` | `Vis/CorNNType`, `Kp/NNStrategy` | ~~`1` → `4` (GPU BF)~~ | **[M] NET LOSS.** 10⁴–10⁵ comparisons = 0.06–0.15 ms CPU vs a 0.108 ms GPU floor | — | — | **Do not do this** |
| **—** ❌ | `Parameters.h:677` | `RegistrationIcp` | `Reg/Strategy` | ~~`0` → `2` (Vis+ICP)~~ | **`[EST]` +12–45 ms.** Breaks 30 FPS outright | — | better local alignment | **Do not do this on the odometry path** |

---

## B-EXTRA — How RTAB-Map already wraps ORB-SLAM3 as an odometry source

**File: `corelib/src/odometry/OdometryORBSLAM3.cpp` (598 lines).** This is the direct answer to "ORB-SLAM3 tracking + RTAB-Map mapping hybrid": **it already exists, upstream, and is selected by one parameter.**

### E.1 Selection

`Parameters.h:456` — `Odom/Strategy = 5`. The doc string says `5=ORB_SLAM2`, but `Odometry::create` (`corelib/src/Odometry.cpp:87-93`) dispatches on a *version* macro:
```cpp
	case Odometry::kTypeORBSLAM:
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 2
		odometry = new OdometryORBSLAM2(parameters);
#else
		odometry = new OdometryORBSLAM3(parameters);
#endif
		break;
```
So on any build with `RTABMAP_ORB_SLAM` ≠ 2, **`Odom/Strategy=5` is ORB-SLAM3**.

🔴 **On this tree it is compile-time dead.** `Version.h` has `//#define RTABMAP_ORB_SLAM` (KF-8), so the whole file reduces to `OdometryORBSLAM3.cpp:592-595`:
```cpp
#else
	UERROR("RTAB-Map is not built with ORB_SLAM support! Select another visual odometry approach.");
#endif
	return t;
```

### E.2 Which ORB-SLAM3 API it calls — the complete surface

| ORB-SLAM3 symbol | Call site | Purpose |
|---|---|---|
| `ORB_SLAM3::System(vocab, configPath, sensorType, false)` | `OdometryORBSLAM3.cpp:343-350` | construction; `sensorType` ∈ `{IMU_STEREO, STEREO, IMU_RGBD, RGBD}` chosen at `:346-349`; last arg `false` = no viewer |
| `System::TrackRGBD(rgb, depth, stamp, imus)` | **`:475`** | **the RGB-D tracking entry point** |
| `System::TrackStereo(leftMono, rightMono, stamp, imus)` | **`:460`** | stereo entry point |
| `System::GetTrackedMapPoints()` | `:480` | `std::vector<ORB_SLAM3::MapPoint*>` of the current frame |
| `System::GetTrackedKeyPointsUn()` | `:547` | undistorted keypoints, only when `Odom/FillInfoData=true` |
| `System::isLost()` | `:481` | tracking state |
| `System::Shutdown()` | `:72`, `:84` | destructor and `reset()` |
| `ORB_SLAM3::Converter::toCvMat(Converter::toSE3Quat(Tcw))` | `:487` | `Sophus::SE3f` → `cv::Mat` 4×4 |
| `ORB_SLAM3::IMU::Point(ax,ay,az,gx,gy,gz,t)` | `:376-383` | IMU sample struct |
| `MapPoint::mnId`, `MapPoint::GetWorldPos()` | `:558`, `:582` | for `info->words` / `info->localMap` |

**That is the entire interface. Seven `System` methods.** It is a remarkably thin, clean wrapper.

### E.3 How ORB-SLAM3 is configured — a generated YAML, written at runtime

`OdometryORBSLAM3::init` (`:106-356`) **writes `<workingDir>/rtabmap_orbslam.yaml` from RTAB-Map's own parameters** and hands the path to `ORB_SLAM3::System`. Mapped parameters:

| RTAB-Map key | Line | → ORB-SLAM3 YAML key |
|---|---|---|
| `OdomORBSLAM/VocPath` | `:110` | (constructor arg) |
| camera model `fx/fy/cx/cy`, `D` | `:146-173` | `Camera1.fx/fy/cx/cy`, `.k1/.k2/.p1/.p2/.k3` |
| `OdomORBSLAM/Bf` (`Parameters.h:553`, default 0.076) | `:184-185, 200` | `Camera.bf` = `fx * baseline` |
| stereo baseline | `:190-198` | `Stereo.T_c1_c2` |
| image size | `:202-203` | `Camera.width/height` |
| `OdomORBSLAM/Fps` (`:555`, default 0 = auto) | `:211-221` | `Camera.fps` |
| `OdomORBSLAM/ThDepth` (`:554`, default 40.0) | `:225-228` | `Stereo.ThDepth`, `Stereo.b` |
| — | `:232` | `RGBD.DepthMapFactor: 1.0` |
| `OdomORBSLAM/{GyroNoise,AccNoise,GyroWalk,AccWalk,SamplingRate}` (`:559-563`) | `:257-283` | `IMU.NoiseGyro/NoiseAcc/GyroWalk/AccWalk/Frequency`, `IMU.T_b_c1` |
| **`OdomORBSLAM/MaxFeatures`** (`:556`, default **1000**) | `:292-294` | `ORBextractor.nFeatures` |
| `ORB/ScaleFactor` (`Parameters.h:315`, default 2) | `:298-300` | `ORBextractor.scaleFactor` |
| `ORB/NLevels` (`:316`, default 3) | `:304-306` | `ORBextractor.nLevels` |
| `FAST/Threshold` (`:298`, default 20), `FAST/MinThreshold` (`:302`, default 7) | `:313-318` | `ORBextractor.iniThFAST/minThFAST` |
| `OdomORBSLAM/Inertial` (`:558`, default false) | `:63`, `:346-349` | selects the `IMU_*` sensor type |

🔑 **`OdometryORBSLAM3.cpp:324-326`:**
```cpp
	//# Disable loop closure detection
	ofs << "loopClosing: " << 0 << std::endl;
	ofs << std::endl;
```
**This is the architectural heart of the hybrid.** `loopClosing` is *not* a stock ORB-SLAM3 YAML key — it requires the patched fork (`https://github.com/matlabbe/ORB_SLAM3`, the one `cmake_modules/FindORB_SLAM.cmake` looks for). ORB-SLAM3 is deliberately reduced to **tracking + local mapping**, and **RTAB-Map owns loop closure, the global graph, and the persistent map.** That is precisely the division of labour the hybrid question is asking about, and upstream already made it.

Note `ORB/ScaleFactor` default is **2** and `ORB/NLevels` default is **3** (`Parameters.h:315-316`) — ORB-SLAM3's own defaults are 1.2 and 8. If you use this path, set `--ORB/ScaleFactor 1.2 --ORB/NLevels 8` explicitly or you will get a much coarser pyramid than ORB-SLAM3 expects.

### E.4 What it hands back to RTAB-Map — and the two things that matter most

**(a) An incremental transform, not a global pose.** `OdometryORBSLAM3.cpp:479-504`:
```cpp
	Transform previousPoseInv = previousPose_.inverse();
	...
		cv::Mat TcwMat = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(Tcw)).clone();
		Transform p = Transform(cv::Mat(TcwMat, cv::Range(0,3), cv::Range(0,4)));
		if(!p.isNull())
		{
			if(!localTransform.isNull())
			{
				if(originLocalTransform_.isNull())
				{
					originLocalTransform_ = localTransform;
				}
				// transform in base frame
				p = originLocalTransform_ * p.inverse() * localTransform.inverse();
			}
			t = previousPoseInv*p;          // <-- DELTA, not absolute
		}
		previousPose_ = p;
```
RTAB-Map consumes `t` as an odometry *increment*. **Consequence:** any correction ORB-SLAM3's local BA applies to the current pose shows up as a one-frame odometry jump. Since `loopClosing: 0` disables its global loop corrections, those jumps stay small — but local-BA-sized discontinuities do leak into RTAB-Map's odometry links.

**(b) 🔴 A hard-coded, heuristic covariance.** `OdometryORBSLAM3.cpp:514-534`:
```cpp
			float baseline = data.cameraModels().size()==1?0.0f:data.stereoCameraModels()[0].baseline();
			if(baseline <= 0.0f)
			{
				baseline = rtabmap::Parameters::defaultOdomORBSLAMBf();
				rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMBf(), baseline);
			}
			double linearVar = 0.0001;
			if(baseline > 0.0f)
			{
				linearVar = baseline/8.0;
				linearVar *= linearVar;
			}

			covariance = cv::Mat::eye(6,6, CV_64FC1);
			covariance.at<double>(0,0) = linearVar;
			covariance.at<double>(1,1) = linearVar;
			covariance.at<double>(2,2) = linearVar;
			covariance.at<double>(3,3) = 0.0001;
			covariance.at<double>(4,4) = 0.0001;
			covariance.at<double>(5,5) = 0.0001;
```
`(baseline/8)²` is a **constant**, identical for a well-tracked frame and a barely-tracked one. It carries no information from ORB-SLAM3's actual tracking quality (inlier count, chi², local-map size). Because RTAB-Map builds the graph's information matrices from exactly this covariance (`Optimizer/VarianceIgnored=false`, `Parameters.h:430`), **every odometry edge in the pose graph gets the same weight.** For the RGB-D case (`cameraModels().size()==1`) it falls back to `OdomORBSLAM/Bf=0.076` → `linearVar = (0.0095)² = 9.03e-5`.

Only the failure cases are distinguished, with `9999` sentinels: `:483` (lost or no map points) and `:509` (first frame after recovery). RTAB-Map treats `≥9999` as "odometry reset → new map" (`RtabmapThread.cpp:509-523`).

> **This is the first thing to improve in any hybrid work.** Deriving `linearVar` from `mapPoints.size()` / the inlier ratio (both already available at `:480` and `:574`) would let RTAB-Map's optimiser actually down-weight shaky tracking. It is a ~15-line change.

**(c) Info fields, only when `Odom/FillInfoData=true`** (`Parameters.h:459`, default **true**) — `:545-587`: `info->words` (keypoint ↔ `MapPoint::mnId`), `info->reg.inliers`/`matches` (both set to the same count of non-null map points, `:574-575`), `info->localMap` (world-frame 3-D points, transformed at `:577-586`), `info->localMapSize = mapPoints.size()` (`:542`). Note `info->localKeyFrames = 0` (`:543`) — **always zero, never populated.**

### E.5 🔴 Is feature extraction duplicated? **Yes — verified.**

`Mem/UseOdomFeatures` (`Parameters.h:234`, default **true**) exists precisely to avoid this. `Memory::createSignature` re-extracts only when the odometry failed to supply features — `Memory.cpp:4871-4875`:
```cpp
	if(!_useOdometryFeatures ||
		data.keypoints().empty() ||
		(int)data.keypoints().size() != data.descriptors().rows ||
		(_feature2D->getType() == Feature2D::kFeatureOrbOctree && data.descriptors().empty()))
	{
```
And `data.setFeatures(...)` is called by **only** F2F, F2M, and the decimation path:
```
corelib/src/odometry/OdometryF2F.cpp:302
corelib/src/odometry/OdometryF2M.cpp:338, 1264, 1299
corelib/src/Odometry.cpp:782
```
`OdometryORBSLAM3.cpp` contains **no `setFeatures` call anywhere** — it populates only `info->words` (`:564`), which is a *visualisation* channel that `Memory` never reads.

**Therefore, with `Odom/Strategy=5`:**
1. ORB-SLAM3's `ORBextractor` extracts `OdomORBSLAM/MaxFeatures = 1000` ORB features inside `TrackRGBD` — **every frame, 30×/s.**
2. RTAB-Map's `Memory::createSignature` extracts `Kp/MaxFeatures = 500` features of its own with `Kp/DetectorStrategy` — **on mapped frames only, i.e. `Rtabmap/DetectionRate = 1×/s` by default.**

**Cost: one extra extraction per second at defaults** (`[EST]` 4–10 ms once per second — negligible). **But it becomes 30 extra extractions per second if you raise `Rtabmap/DetectionRate` to 30**, which is exactly what someone chasing "30 FPS mapping" would do. Flag this loudly.

There is no clean fix without patching: RTAB-Map's BoW vocabulary and ORB-SLAM3's DBoW2 vocabulary are different objects with different descriptors, so the features genuinely cannot be shared as-is. The minimal patch is to call `data.setFeatures(kptsUn, keypoints3D, descriptors)` in `OdometryORBSLAM3::computeTransform` using `GetTrackedKeyPointsUn()` plus the descriptors (which the wrapper does not currently retrieve — ORB-SLAM3's `System` exposes no descriptor getter, so this needs a fork change). Sketch in `perf_patches/06-orbslam3-share-features.md`.

### E.6 Other findings on this path

- **Input is the color image, unconverted.** `:475` passes `data.imageRaw()` straight into `TrackRGBD`, with `Camera.RGB: 0` (BGR) written at `:208`. ORB-SLAM3 converts internally. For stereo, RTAB-Map converts to mono itself (`:450-459`) — and there is a copy-paste bug at `:458`: `cv::cvtColor(data.imageRaw(), rightMono, CV_BGR2GRAY)` uses **`imageRaw()` (the left image)** where it should use `data.rightRaw()`.
- **Depth is converted when needed**: `CV_16UC1` → `util2d::cvtDepthToFloat` (`:471-474`). A full-image conversion per frame for the common RealSense 16UC1 case — `[EST]` 0.2–0.5 ms at 640×480.
- **`OdomORBSLAM/MapSize`** (`Parameters.h:557`, default 3000) is parsed at `:321-322` and then **never used** — the doc string even says "Only supported with ORB_SLAM2".
- **Two frames of warm-up** are consumed before `init()`: `:432-436` returns an empty transform on the first frame just to estimate the camera FPS.
- **Bootstrapping writes to disk**: `rtabmap_orbslam.yaml` in `Rtabmap/WorkingDirectory`, plus loading the ORB vocabulary (`:119` warns "This could take a while") — a multi-second startup cost, once.

### E.7 Verdict on the hybrid

The hybrid the user is imagining **is already implemented upstream and is architecturally sound**: ORB-SLAM3 does tracking + local mapping with its own loop closer explicitly disabled, RTAB-Map does appearance-based loop closure, graph optimisation, memory management and the persistent database. To use it on this machine you need, in order:
1. **Fix KF-8** (the stale `Version.h`).
2. Build the **matlabbe ORB-SLAM3 fork** (the stock upstream does not honour `loopClosing: 0`) and configure `-DWITH_ORB_SLAM=ON`.
3. Set `--Odom/Strategy 5 --OdomORBSLAM/VocPath <ORBvoc.txt> --ORB/ScaleFactor 1.2 --ORB/NLevels 8`.
4. Keep `--Rtabmap/DetectionRate 1` to keep the duplicate extraction at 1 Hz.
5. Consider the covariance fix (E.4b) before trusting the optimised graph.

**Expected odometry latency:** [P] this project measured ORB-SLAM3 `TrackRGBD` at **12.4 ms mean** (RGB-D) / **13.7 ms** (IMU-RGBD) / and in a later session **18.7 ms mean, 24.5 ms P95**, at 640×480 on a *stronger* CPU. On this i9-12900HK under WSL2, expect `[EST]` **15–25 ms mean**, which fits 33.33 ms but with less margin than RTAB-Map's own F2M (`[EST]` 6–12 ms) — because ORB-SLAM3 additionally runs motion-only BA and local-map tracking every frame, which RTAB-Map's F2M does not (local BA is disabled here, KF-8).

**Trade:** you buy ORB-SLAM3's markedly better tracking accuracy ([P]: Method-B mean displacement **0.067 m** vs RTAB-Map's 0.185 m on the same bag) at roughly **2× the odometry latency** and a much harder build.

---

## Appendix A — Evidence ledger

### A.1 MEASURED on this machine (by the coordinator)
- OpenMP: first region 10.508 ms; warm region 20 thr **3.7858 ms**, 2 thr 0.0053, 4 thr 0.0015, 8 thr 0.0033
- CUDA: null-kernel async enqueue 7.7 µs; launch+sync 26.06 µs; 64 KB up + kernel + 64 KB down **0.108 ms**
- Transfer bandwidth table (pageable vs pinned, 64 KiB → 5.9 MB) — pinned gain only **1.2–1.5×**
- CPU 256-bit Hamming POPCNT: 30 k → 0.060 ms, 80 k → 0.150 ms, 400 k → 0.789 ms, 4 M → 5.03–7.88 ms
- GPU brute-force Hamming 4 M: kernel 0.409 ms, full round trip 0.648 ms
- GPU ORB stages: **0.638 ms @640×480**, 0.841 @1280×720, 1.186 @1920×1080 (per-stage sync)
- `sched_setscheduler(SCHED_FIFO/RR)` → **EPERM**; `nice(-10)` → **EPERM**; `pthread_setaffinity_np` → **OK**
- GPU: 58 SM @ 1.46 GHz, 512 GB/s, 16 GB, 137 W TGP

### A.2 MEASURED on this machine (by me, this session)
- `nproc`=20; `lscpu` → i9-12900HK, 10×2 flattened, L2 12.5 MiB, L3 24 MiB; **no AVX-512**
- `nvidia-smi` → RTX 3080 Ti Laptop, 16384 MiB, cc 8.6, driver 596.08
- `nvcc` → CUDA 12.4.131
- `find / -name "libopencv_cuda*"` → **empty**; only `/opt/intel/openvino_2021/opencv` (4.5.3) + Windows ZED 3.1.0
- `strings libopencv_core.so` → **`Parallel framework: pthreads`** (that OpenCV does not use OpenMP)
- `ls /opt/ros` → **absent**; no conda env with `rclcpp`/`rtabmap`; `find ~ -name "librtabmap_core*"` → empty
- `gettimeofday` **15.71 ns/call**, `clock_gettime` **15.05 ns/call** (vDSO works in WSL2 → `UTimer` instrumentation is a non-issue)

### A.3 MEASURED earlier in this project, on a DIFFERENT machine
Original PC = NVIDIA RTX PRO 6000 Blackwell Max-Q (`_env_specs/README.md:63`). CPU unspecified.
- ORB-SLAM3 `TrackRGBD` mean **12.4 ms** (RGB-D) / **13.7 ms** (IMU-RGBD), 4313 tracked frames (~94 %), 0 track failures at 1.0× replay
- A later session: track mean **18.7 ms, P95 24.5 ms** (2004 frames, 189 KF, 23853 map points)
- Dataset: RealSense D455f, **640×480 @ 29.9–30.0 Hz**, aligned depth, fx 386.92 / fy 386.35 / cx 322.78 / cy 250.28
- Accuracy (same bag): ORB-SLAM3 Method-B mean displacement 0.067 m vs RTAB-Map 0.185 m; loops fired 1 vs 166; RTAB-Map loop gap 0.168 m pre-opt → **0.008 m post-opt**, χ² 200 → 0.47
- **No RTAB-Map ms/fps measurement exists anywhere in this project.**

### A.4 Everything else is [EST] or [EXP] and is tagged inline.

## Appendix B — Files I did not modify

Per the brief, nothing under `rtabmap_ws/src/` was touched. Proposed code changes live as diffs/notes in `rtabmap_ws/perf_patches/`:
- `00-remove-stale-version-header.md` — Phase 0 (KF-8)
- `01-registrationvis-stage-timers.diff` — Phase 11
- `03-expose-drop-counters.md` — Phase 12
- `04-thread-pool-caps.cpp` — Phase 1
- `05-hoist-bfmatcher.diff` — Phase 10
- `06-orbslam3-share-features.md` — B-EXTRA E.5
