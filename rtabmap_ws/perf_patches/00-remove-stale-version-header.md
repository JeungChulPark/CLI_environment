# Phase 0 (CRITICAL) — remove the stale, git-tracked `Version.h`

## Problem

`rtabmap_ws/src/rtabmap/corelib/include/rtabmap/core/Version.h` is a **committed build
artifact**, not source. It was captured from some earlier machine's build tree.

```
$ git ls-files --error-unmatch rtabmap_ws/src/rtabmap/corelib/include/rtabmap/core/Version.h
rtabmap_ws/src/rtabmap/corelib/include/rtabmap/core/Version.h        # tracked
$ git log --oneline -1 -- .../Version.h
a4aff56 Initial commit: SLAM + SAM-6D ROS2 workspaces (code only)
```

CMake generates the real one somewhere else — `CMakeLists.txt:1129`:

```cmake
CONFIGURE_FILE(Version.h.in ${CMAKE_CURRENT_BINARY_DIR}/corelib/src/include/${PROJECT_PREFIX}/core/Version.h)
```

but the include search order puts the **source** tree first — `corelib/src/CMakeLists.txt:859-861`:

```cmake
target_include_directories(rtabmap_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include;${CMAKE_CURRENT_BINARY_DIR}/include;${PUBLIC_INCLUDE_DIRS};${INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR};${PUBLIC_INCLUDE_DIRS}>")
```

`${CMAKE_CURRENT_SOURCE_DIR}/../include` == `corelib/include` (stale) precedes
`${CMAKE_CURRENT_BINARY_DIR}/include` (generated). **The stale file wins.**

## Impact

Every optional backend is permanently off, regardless of what CMake finds and links:

```
//#define RTABMAP_NONFREE        -> no SIFT/SURF
//#define RTABMAP_G2O            -> no g2o; OdomF2M/BundleAdjustment and Vis/BundleAdjustment default to 0
//#define RTABMAP_GTSAM          -> no GTSAM / iSAM2
//#define RTABMAP_CERES          -> no Ceres
//#define RTABMAP_POINTMATCHER   -> ICP is PCL-only; Icp/PointToPlane defaults false
//#define RTABMAP_CCCORELIB
//#define RTABMAP_OPEN3D
//#define RTABMAP_CUDASIFT
//#define RTABMAP_TORCH          -> no SuperPoint
//#define RTABMAP_PYTHON         -> no PyDetector / SuperGlue
//#define RTABMAP_ORB_SLAM       -> Odom/Strategy=5 is DEAD (OdometryORBSLAM3.cpp:592-595)
#define RTABMAP_TORO             -> Optimizer::create() default: falls through to TORO,
                                    Optimizer/Iterations defaults to 100 (Parameters.h:424-426)
```

You can `apt install libg2o-dev ros-humble-libpointmatcher` etc., watch CMake report
"Found g2o", and still get a binary with g2o disabled. This is silent and very hard to debug.

## Fix — option A (preferred): delete the file

```bash
cd /home/jucpark/DeepLearning/CLI_environment/rtabmap_ws/src/rtabmap
git rm --cached corelib/include/rtabmap/core/Version.h
rm corelib/include/rtabmap/core/Version.h
printf 'corelib/include/rtabmap/core/Version.h\n' >> .gitignore
```

CMake then generates it into the binary dir, and the only copy on the include path is
the correct one. This matches upstream RTAB-Map, which never ships this file in source.

## Fix — option B: reorder the include dirs (leaves the file in place)

```diff
--- a/corelib/src/CMakeLists.txt
+++ b/corelib/src/CMakeLists.txt
@@
 target_include_directories(rtabmap_core PUBLIC 
-  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include;${CMAKE_CURRENT_BINARY_DIR}/include;${PUBLIC_INCLUDE_DIRS};${INCLUDE_DIRS}>"
+  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include;${CMAKE_CURRENT_SOURCE_DIR}/../include;${PUBLIC_INCLUDE_DIRS};${INCLUDE_DIRS}>"
   "$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR};${PUBLIC_INCLUDE_DIRS}>")
```

Option A is safer — option B leaves a misleading file in the tree that will confuse the
next reader and can still be picked up by other targets or by downstream consumers of
the exported `INSTALL_INTERFACE`.

## Verify after rebuilding

```bash
# the generated header should reflect what CMake actually found
grep -E "^(//)?#define RTABMAP_(G2O|GTSAM|POINTMATCHER|NONFREE|ORB_SLAM)" \
  <build>/corelib/src/include/rtabmap/core/Version.h

# and at runtime:
rtabmap --version      # prints "With g2o: true/false", "With CudaSift: ..." (Parameters.cpp:699-703)
```
