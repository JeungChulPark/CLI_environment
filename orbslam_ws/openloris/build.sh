#!/usr/bin/env bash
# Build the OpenLORIS runner against two ORB-SLAM3 trees, the same way (flags, deps, OpenMP) for both:
#   upstream  UZ-SLAMLab/ORB_SLAM3 @ v1.0-release, only the macOS portability fixes below
#   ours      orbslam_ws/src/ORB_SLAM3 (this repo), WITH_GYRO
#
#   orbslam_ws/openloris/build.sh [WORKDIR]      (default orbslam_ws/output/openloris)
#
# Pangolin and ORBvoc.txt come from orbslam_ws/output/mac_build (mac_harness/build_mac.sh).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
W=$(mkdir -p "${1:-$REPO/orbslam_ws/output/openloris}" && cd "${1:-$REPO/orbslam_ws/output/openloris}" && pwd)
MB=$REPO/orbslam_ws/output/mac_build
[ -f "$MB/deps/lib/cmake/Pangolin/PangolinConfig.cmake" ] || { echo "run orbslam_ws/mac_harness/build_mac.sh first (Pangolin)"; exit 1; }

if [ ! -d "$W/upstream_src" ]; then
  git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git "$W/upstream_src"
fi
git -C "$W/upstream_src" checkout -q v1.0-release

prep() {   # $1 = destination; copies src/include/Thirdparty from $2 and applies the portability fixes
  local dst=$1 from=$2
  rm -rf "$dst" && mkdir -p "$dst"
  cp -R "$from/src" "$from/include" "$from/Thirdparty" "$dst/"
  rm -rf "$dst/Thirdparty/DBoW2/lib" "$dst/Thirdparty/g2o/lib" "$dst/Thirdparty/DBoW2/build" "$dst/Thirdparty/g2o/build"
  for f in hyper_graph.h sparse_block_matrix_ccs.h estimate_propagator.h robust_kernel.h marginal_covariance_cholesky.h; do
    sed -i '' -e 's#<tr1/unordered_map>#<unordered_map>#; s#<tr1/memory>#<memory>#; s#std::tr1::#std::#g' \
      "$dst/Thirdparty/g2o/g2o/core/$f"
  done
  sed -i '' 's#stdint-gcc.h#stdint.h#' "$dst/Thirdparty/DBoW2/DBoW2/FORB.cpp" "$dst/src/ORBmatcher.cc"
}
prep "$W/src_upstream" "$W/upstream_src"
prep "$W/src_ours" "$REPO/orbslam_ws/src/ORB_SLAM3"

OMP=$(brew --prefix libomp)
build() {   # $1 = name, $2 = source tree, $3 = WITH_GYRO
  cmake -S "$HERE" -B "$W/build_$1" -Wno-dev -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DORB_SRC="$2" -DWITH_GYRO="$3" \
    -DOpenCV_DIR="$(brew --prefix opencv@4)/lib/cmake/opencv4" \
    -DEigen3_DIR="$(brew --prefix eigen@3)/share/eigen3/cmake" \
    -DPangolin_DIR="$MB/deps/lib/cmake/Pangolin" \
    -DOPENSSL_ROOT_DIR="$(brew --prefix openssl@3)" \
    "-DOpenMP_CXX_FLAGS=-Xpreprocessor -fopenmp -I$OMP/include" -DOpenMP_CXX_LIB_NAMES=omp \
    -DOpenMP_omp_LIBRARY="$OMP/lib/libomp.dylib"
  cmake --build "$W/build_$1" -j"$(sysctl -n hw.ncpu)"
}
build upstream "$W/src_upstream" OFF
build ours "$W/src_ours" ON
echo "built $W/build_upstream/openloris_rgbd and $W/build_ours/openloris_rgbd"
