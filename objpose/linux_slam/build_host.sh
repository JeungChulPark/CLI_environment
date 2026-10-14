#!/bin/bash
# build_host.sh — run ON the Linux SLAM host (192.168.20.3) after linux_slam/deploy.sh has put the
# bundle in ~/objpose. Idempotent: builds Pangolin once, applies the ORB-SLAM3 patch once, then
# (re)builds libORB_SLAM3 + slam_stream with -march=native for THIS machine's CPU.
#   ~/objpose/linux_slam/build_host.sh            # full build
#   ~/objpose/linux_slam/build_host.sh --check    # only verify that the build products exist
set -euo pipefail
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
cd "$ROOT"

if [ "${1:-}" = "--check" ]; then
  for f in build/slam_stream build/libORB_SLAM3.so deps/install/lib/cmake/Pangolin/PangolinConfig.cmake Vocabulary/ORBvoc.txt run_slam.sh; do
    [ -e "$f" ] || { echo "missing: $ROOT/$f" >&2; exit 1; }
  done
  echo "ok: $ROOT ($(nproc) cpus, $(lscpu | sed -n 's/Model name: *//p'))"; exit 0
fi

echo "== 1. apt packages =="
PKGS=(build-essential cmake ninja-build git patch libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev
      libsqlite3-dev nlohmann-json3-dev libglew-dev libgl1-mesa-dev libepoxy-dev)
missing=()
for p in "${PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
# A host without sudo (.41): the dev files of libsqlite3-dev / nlohmann-json3-dev can be staged by hand under
# deps/install (include/sqlite3.h, lib/libsqlite3.so -> the system .so.0, include/nlohmann + lib/cmake/nlohmann_json);
# CMake finds them through CMAKE_PREFIX_PATH below, so they are not missing packages then.
[ -f deps/install/include/sqlite3.h ] && [ -e deps/install/lib/libsqlite3.so ] && missing=("${missing[@]/libsqlite3-dev}")
[ -f deps/install/include/nlohmann/json.hpp ] && missing=("${missing[@]/nlohmann-json3-dev}")
missing=(${missing[@]})   # drop the empty slots left by the removals
if [ ${#missing[@]} -gt 0 ]; then
  echo "   installing: ${missing[*]}"
  if ! sudo -n apt-get install -y "${missing[@]}" 2>/dev/null; then
    echo "   sudo needs a password here. Run once on this host, then re-run build_host.sh:" >&2
    echo "     sudo apt-get install -y ${missing[*]}" >&2
    exit 1
  fi
else
  echo "   all present"
fi

echo "== 2. Pangolin v0.9.2 -> deps/install =="
if [ ! -f deps/install/lib/cmake/Pangolin/PangolinConfig.cmake ]; then
  cmake -S deps/Pangolin -B deps/Pangolin/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$ROOT/deps/install" -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
        -DCMAKE_CXX_FLAGS="-march=native" > deps/pangolin_cmake.log 2>&1 || { tail -30 deps/pangolin_cmake.log; exit 1; }
  cmake --build deps/Pangolin/build -j"$(nproc)" > deps/pangolin_build.log 2>&1 || { tail -30 deps/pangolin_build.log; exit 1; }
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
# a stale cache that once found conda libraries keeps them; start clean in that case
if [ -f build/CMakeCache.txt ] && grep -q '/anaconda3/\|/miniconda3/\|/conda/' build/CMakeCache.txt; then
  echo "   build/CMakeCache.txt references a conda prefix; reconfiguring from scratch"; rm -rf build
fi
cmake -S linux_slam -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$ROOT/deps/install" > build_cmake.log 2>&1 || { tail -40 build_cmake.log; exit 1; }
if grep -q '/anaconda3/\|/miniconda3/\|/conda/' build/CMakeCache.txt; then
  echo "   a conda library got into the build (see build/CMakeCache.txt); run build_host.sh from a shell without conda on PATH" >&2
  grep '/anaconda3/\|/miniconda3/\|/conda/' build/CMakeCache.txt | head -5 >&2; exit 1
fi
cmake --build build -j"$(nproc)" > build_ninja.log 2>&1 || { grep -E 'error|Error' build_ninja.log | head -20; tail -20 build_ninja.log; exit 1; }
ln -sfn linux_slam/run_slam.sh run_slam.sh
chmod +x linux_slam/*.sh

echo "== 5. smoke =="
LD_LIBRARY_PATH="$ROOT/build:$ROOT/deps/install/lib" ./build/slam_stream 2>&1 | head -3 || true
"$0" --check
