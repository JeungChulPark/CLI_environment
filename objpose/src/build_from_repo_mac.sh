#!/bin/bash
# build_from_repo_mac.sh — run ON the Mac (192.168.20.55), inside ITS OWN CLI_environment checkout
# (~/Documents/DefenseMeta/CLI_environment). Rebuilds the runtime the hub launches (~/objpose/run_slam.sh ->
# ~/objpose/build/slam_stream) from the sources of THIS checkout:
#     <repo>/orbslam_ws/src/ORB_SLAM3   -> ~/objpose/ORB_SLAM3_src   (+ patches/, applied idempotently)
#     <repo>/objpose/src                -> ~/objpose/src              (slam_stream.cc, raw_session.h, gyro_aid.h, CMakeLists.txt)
#     <repo>/objpose/src/settings       -> ~/objpose/settings         (what run_slam.sh reads by default)
# The previous build is kept once as ~/objpose/build.prev_<date>.
#   bash objpose/src/build_from_repo_mac.sh
set -euo pipefail
export PATH="/opt/homebrew/bin:$PATH"   # non-interactive ssh shells lack the Homebrew cmake
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="$HOME/objpose"
[ -d "$REPO/orbslam_ws/src/ORB_SLAM3/src" ] || { echo "no ORB-SLAM3 source under $REPO/orbslam_ws/src" >&2; exit 1; }
[ -d "$DST/build" ] || { echo "$DST/build missing: this script only rebuilds an existing Mac bundle" >&2; exit 1; }

if [ -d "$DST/build" ]; then
  B="$DST/build.prev_$(date +%y%m%d)"
  [ -e "$B" ] || { cp -a "$DST/build" "$B"; echo "== previous build kept at $B"; }
fi
echo "== $REPO -> $DST =="
rsync -a --exclude lib --exclude build --exclude 'build_*' --exclude Vocabulary --exclude '*.log' --exclude '*.pdf' \
      --exclude '*.bak*' "$REPO/orbslam_ws/src/ORB_SLAM3/" "$DST/ORB_SLAM3_src/"
rsync -a --exclude build --exclude __pycache__ "$SRC/" "$DST/src/"
mkdir -p "$DST/settings" && rsync -a "$SRC/settings/" "$DST/settings/"
for p in "$DST"/src/patches/*.patch; do
  if patch -p1 -d "$DST/ORB_SLAM3_src" --dry-run --forward -s < "$p" >/dev/null 2>&1; then
    patch -p1 -d "$DST/ORB_SLAM3_src" --forward -s < "$p" && echo "   applied $(basename "$p")"
  else
    echo "   already applied $(basename "$p")"
  fi
done
echo "== build (cmake, $(sysctl -n hw.ncpu) jobs) =="
cmake -S "$DST/src" -B "$DST/build" -DCMAKE_BUILD_TYPE=Release > "$DST/build_cmake.log" 2>&1 || { tail -30 "$DST/build_cmake.log"; exit 1; }
cmake --build "$DST/build" -j"$(sysctl -n hw.ncpu)" > "$DST/build_make.log" 2>&1 || { grep -E "error" "$DST/build_make.log" | head -20; tail -20 "$DST/build_make.log"; exit 1; }
echo "== smoke =="
"$DST/build/slam_stream" 2>&1 | head -2 || true
ls -la "$DST/build/slam_stream"
