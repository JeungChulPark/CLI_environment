#!/bin/bash
# build_rtab_mamba.sh — build rtab_stream (RTAB-Map backend of run_slam.sh) on a Linux SLAM host without root,
# e.g. the Samsung laptop (Ubuntu 24.04, no system compiler). OpenCV/PCL/g2o and the toolchain come from the micromamba
# env that also runs hdl_graph_slam (robostack has no rtabmap package), the RTAB-Map core (0.23.11, the version of this
# PC's ~/DeepLearning/rtabmap_compare_ws) is built from deps/rtabmap into deps/rtabmap_install; rtab_stream gets an RPATH into both,
# so run_slam.sh needs no LD_LIBRARY_PATH for it.
#   rsync -a ~/DeepLearning/rtabmap_compare_ws/src/rtabmap/ <host>:objpose/deps/rtabmap/     (once, from the PC)
#   ~/objpose/linux_slam/build_rtab_mamba.sh           # -> ~/objpose/build_rtab/rtab_stream
set -euo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
cd "$ROOT"
ENV="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}/envs/${RTAB_MAMBA_ENV:-hdl_graph_slam_jazzy}"
CORE="$ROOT/deps/rtabmap_install"
[ -x "$ENV/bin/x86_64-conda-linux-gnu-g++" ] || { echo "env $ENV has no compiler" >&2; exit 1; }
unset CONDA_PREFIX
export PATH="$ENV/bin:/usr/local/bin:/usr/bin:/bin"
export CC="$ENV/bin/x86_64-conda-linux-gnu-gcc" CXX="$ENV/bin/x86_64-conda-linux-gnu-g++"
export CMAKE_PREFIX_PATH="$CORE:$ENV"
export LDFLAGS="-L$ENV/lib -Wl,-rpath,$CORE/lib -Wl,-rpath,$ENV/lib -Wl,-rpath-link,$ENV/lib"
J="${J:-$(nproc)}"
if [ ! -f "$CORE/lib/librtabmap_core.so" ]; then
  echo "== RTAB-Map core deps/rtabmap -> $CORE"
  cmake -S deps/rtabmap -B deps/rtabmap/build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CORE" \
    -DBUILD_APP=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DWITH_QT=OFF -DWITH_G2O=ON -DWITH_GTSAM=OFF -DWITH_CERES=OFF \
    -DWITH_OPENGV=OFF -DWITH_POINTMATCHER=OFF -DWITH_OCTOMAP=OFF -DWITH_PDAL=OFF -DWITH_MRPT=OFF -DWITH_REALSENSE2=OFF \
    -DWITH_PYTHON=OFF -DWITH_TORCH=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_RPATH="$ENV/lib" > build_rtab_core_cmake.log 2>&1 || { tail -40 build_rtab_core_cmake.log; exit 1; }
  cmake --build deps/rtabmap/build -j"$J" > build_rtab_core.log 2>&1 || { grep -E ' error' build_rtab_core.log | head -20; tail -20 build_rtab_core.log; exit 1; }
  cmake --install deps/rtabmap/build > /dev/null
fi
rm -rf build_rtab
cmake -S linux_slam/rtabmap -B build_rtab -G Ninja -DCMAKE_BUILD_TYPE=Release > build_rtab_cmake.log 2>&1 || { tail -40 build_rtab_cmake.log; exit 1; }
cmake --build build_rtab -j"$J" > build_rtab_ninja.log 2>&1 || { grep -E 'error' build_rtab_ninja.log | head -20; tail -20 build_rtab_ninja.log; exit 1; }
if ldd build_rtab/rtab_stream | grep -q "not found"; then ldd build_rtab/rtab_stream | grep "not found"; exit 1; fi
echo "ok: $ROOT/build_rtab/rtab_stream"
