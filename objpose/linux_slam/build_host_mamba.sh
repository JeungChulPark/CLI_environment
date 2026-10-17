#!/bin/bash
# build_host_mamba.sh — build_host.sh for a Linux SLAM host WITHOUT root (no sudo/apt): the Samsung laptop
# 192.168.20.55 (Ubuntu 24.04, 2026-09-22). Toolchain + libraries come from a micromamba env instead of apt
# (build_host.sh refuses conda because on 192.168.20.3 conda libs shadowed the system OpenCV; here there IS no
# system OpenCV/compiler, so the whole build is done inside the env and the binaries get an RPATH into it).
#   env:   ~/micromamba/envs/objpose  (cxx-compiler cmake ninja libopencv=4 libboost-devel eigen glew epoxy
#          nlohmann_json openssl libsqlite libgl-devel libegl-devel libglx-devel xorg-libx11 xorg-libxext)
#   run:   ~/objpose/linux_slam/build_host_mamba.sh        # Pangolin once, ORB patch once, libORB_SLAM3 + slam_stream
#          ~/objpose/linux_slam/build_host_mamba.sh --check
# Same ~/objpose layout as build_host.sh, so hub.py launches ~/objpose/run_slam.sh unchanged.
set -euo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
cd "$ROOT"
MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
ENV_NAME="${OBJPOSE_MAMBA_ENV:-objpose}"
ENV="$MAMBA_ROOT_PREFIX/envs/$ENV_NAME"

if [ "${1:-}" = "--check" ]; then
  for f in build/slam_stream build/libORB_SLAM3.so deps/install/lib/cmake/Pangolin/PangolinConfig.cmake Vocabulary/ORBvoc.txt run_slam.sh; do
    [ -e "$f" ] || { echo "missing: $ROOT/$f" >&2; exit 1; }
  done
  echo "ok: $ROOT ($(nproc) cpus, $(lscpu | sed -n 's/Model name: *//p'), env $ENV)"; exit 0
fi

echo "== 1. micromamba env $ENV =="
[ -x "$ENV/bin/cmake" ] && [ -x "$ENV/bin/x86_64-conda-linux-gnu-g++" ] || {
  echo "env missing; create it first (see header)" >&2; exit 1; }
# no `micromamba activate` (its compiler activate.d hooks do not run in a non-interactive script): set the toolchain
# by hand. linux_slam/CMakeLists.txt puts $CONDA_PREFIX on CMAKE_IGNORE_PATH (for the apt hosts) -> keep it unset.
unset CONDA_PREFIX
export PATH="$ENV/bin:/usr/local/bin:/usr/bin:/bin"
export CC="$ENV/bin/x86_64-conda-linux-gnu-gcc" CXX="$ENV/bin/x86_64-conda-linux-gnu-g++"
export CMAKE_PREFIX_PATH="$ENV"
export CPATH="$ENV/include" LIBRARY_PATH="$ENV/lib"
export LDFLAGS="-L$ENV/lib -Wl,-rpath,$ENV/lib -Wl,-rpath-link,$ENV/lib"
echo "   $($CXX --version | head -1)"

echo "== 2. Pangolin -> deps/install =="
if [ ! -f deps/install/lib/cmake/Pangolin/PangolinConfig.cmake ]; then
  cmake -S deps/Pangolin -B deps/Pangolin/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$ROOT/deps/install" -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
        -DBUILD_PANGOLIN_FFMPEG=OFF -DBUILD_PANGOLIN_LIBREALSENSE=OFF -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
        -DBUILD_PANGOLIN_OPENNI=OFF -DBUILD_PANGOLIN_OPENNI2=OFF -DBUILD_PANGOLIN_V4L=OFF \
        -DCMAKE_CXX_FLAGS="-march=native" > deps/pangolin_cmake.log 2>&1 || { tail -30 deps/pangolin_cmake.log; exit 1; }
  cmake --build deps/Pangolin/build -j"$(nproc)" > deps/pangolin_build.log 2>&1 || { grep -E "error" deps/pangolin_build.log | head; tail -30 deps/pangolin_build.log; exit 1; }
  cmake --install deps/Pangolin/build > /dev/null
  echo "   built"
else
  echo "   present"
fi

echo "== 3. ORB-SLAM3 source patch (mac_slam/patches) =="
for p in mac_slam/patches/*.patch; do
  if patch -p1 -d ORB_SLAM3_src --dry-run --forward -s < "$p" >/dev/null 2>&1; then
    patch -p1 -d ORB_SLAM3_src --forward -s < "$p" && echo "   applied $(basename "$p")"
  else
    echo "   already applied $(basename "$p")"
  fi
done

echo "== 4. libORB_SLAM3 + slam_stream (-march=native, $(nproc) jobs) =="
cmake -S linux_slam -B build -G Ninja -DCMAKE_BUILD_TYPE=Release > build_cmake.log 2>&1 || { tail -40 build_cmake.log; exit 1; }
cmake --build build -j"$(nproc)" > build_ninja.log 2>&1 || { grep -E 'error|Error' build_ninja.log | head -20; tail -20 build_ninja.log; exit 1; }
ln -sfn linux_slam/run_slam.sh run_slam.sh
chmod +x linux_slam/*.sh

echo "== 5. smoke =="
LD_LIBRARY_PATH="$ROOT/build:$ROOT/deps/install/lib" ./build/slam_stream 2>&1 | head -3 || true
if ldd build/slam_stream | grep -q "not found"; then ldd build/slam_stream | grep "not found"; exit 1; fi
"$0" --check
