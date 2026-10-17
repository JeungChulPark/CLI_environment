#!/usr/bin/env bash
# Build ORB-SLAM3 + rt_harness natively on an Apple-silicon Mac (no ROS, no conda).
#
#   orbslam_ws/mac_harness/build_mac.sh <workdir>            # patched tree (configs A/B/C)
#   STOCK=1 orbslam_ws/mac_harness/build_mac.sh <workdir>    # f912e95^ tree (baseline)
#
# Nothing is written into the repository: sources are copied into <workdir>, where two
# portability fixes are applied (g2o's std::tr1 -> std, stdint-gcc.h -> stdint.h).
# Homebrew's current eigen (5) and opencv (5) are too new for this tree, hence the @3/@4 kegs.
set -euo pipefail

W=$(mkdir -p "$1" && cd "$1" && pwd)
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)

brew install cmake eigen@3 opencv@4 boost libomp openssl@3 ffmpeg

if [ ! -f "$W/deps/lib/cmake/Pangolin/PangolinConfig.cmake" ]; then
  git clone --depth 1 --branch v0.9.2 https://github.com/stevenlovegrove/Pangolin.git "$W/Pangolin"
  cmake -S "$W/Pangolin" -B "$W/Pangolin/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$W/deps" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_TESTS=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF -DBUILD_PANGOLIN_REALSENSE2=OFF -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DEigen3_DIR="$(brew --prefix eigen@3)/share/eigen3/cmake" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build "$W/Pangolin/build" -j"$(sysctl -n hw.ncpu)"
  cmake --install "$W/Pangolin/build"
fi

if [ ! -f "$W/ORBvoc.txt" ]; then
  curl -sL https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz | tar xz -C "$W"
fi

SRC="$W/orb3${STOCK:+_stock}"
rm -rf "$SRC" && mkdir -p "$SRC"
cp -R "$REPO/orbslam_ws/src/ORB_SLAM3/Thirdparty" "$SRC/"
if [ -n "${STOCK:-}" ]; then
  git -C "$REPO" archive f912e95^ orbslam_ws/src/ORB_SLAM3/src orbslam_ws/src/ORB_SLAM3/include \
    | tar -x -C "$SRC" --strip-components=3
else
  cp -R "$REPO/orbslam_ws/src/ORB_SLAM3/src" "$REPO/orbslam_ws/src/ORB_SLAM3/include" "$SRC/"
fi
rm -rf "$SRC/Thirdparty/DBoW2/lib" "$SRC/Thirdparty/g2o/lib"
for f in hyper_graph.h sparse_block_matrix_ccs.h estimate_propagator.h robust_kernel.h marginal_covariance_cholesky.h; do
  sed -i '' -e 's#<tr1/unordered_map>#<unordered_map>#; s#<tr1/memory>#<memory>#; s#std::tr1::#std::#g' \
    "$SRC/Thirdparty/g2o/g2o/core/$f"
done
sed -i '' 's#stdint-gcc.h#stdint.h#' "$SRC/Thirdparty/DBoW2/DBoW2/FORB.cpp" "$SRC/src/ORBmatcher.cc"
cp "$HERE/CMakeLists.txt" "$SRC/CMakeLists.txt"

OMP=$(brew --prefix libomp)
cmake -S "$SRC" -B "$SRC/build" -Wno-dev -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DORBSLAM3_REGISTER_TIMES=ON \
  -DHARNESS_DIR="$HERE" \
  -DOpenCV_DIR="$(brew --prefix opencv@4)/lib/cmake/opencv4" \
  -DEigen3_DIR="$(brew --prefix eigen@3)/share/eigen3/cmake" \
  -DPangolin_DIR="$W/deps/lib/cmake/Pangolin" \
  -DOPENSSL_ROOT_DIR="$(brew --prefix openssl@3)" \
  "-DOpenMP_CXX_FLAGS=-Xpreprocessor -fopenmp -I$OMP/include" -DOpenMP_CXX_LIB_NAMES=omp \
  -DOpenMP_omp_LIBRARY="$OMP/lib/libomp.dylib"
cmake --build "$SRC/build" -j"$(sysctl -n hw.ncpu)"

# live_rtabmap: Homebrew's rtabmap (PCL, Eigen 5) in its own CMake project, apart from the Eigen 3 tree above
brew install rtabmap
cmake -S "$HERE/rtabmap" -B "$W/rtab" -Wno-dev -DRTABMap_DIR="$(brew --prefix rtabmap)/lib/rtabmap-0.23" \
  -DOpenCV_DIR="$(brew --prefix opencv@4)/lib/cmake/opencv4" -DOPENSSL_ROOT_DIR="$(brew --prefix openssl@3)"
cmake --build "$W/rtab" -j"$(sysctl -n hw.ncpu)"

cat <<EOF
built $SRC/build/rt_harness, $SRC/build/live_orbslam, $W/rtab/live_rtabmap

run (30 Hz, KEEP_LAST 5):
  mkdir -p run && cd run && OMP_NUM_THREADS=6 $SRC/build/rt_harness $W/ORBvoc.txt \\
    $REPO/orbslam_ws/data/nf2000_C_early.yaml <bag>.db3 . 30 0 5 > run.log 2>&1
video for the live page (frame i == trajectory row i):
  $SRC/build/rt_harness - - <bag>.db3 seq.mp4 video
live SLAM with the browser viewer (ORB-SLAM3 or RTAB-Map chosen on the page; video + 3D map + pose):
  BUILD=$W $HERE/run_live.sh [<bag>.db3]     # http://127.0.0.1:8080/
EOF
