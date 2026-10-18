#!/bin/bash
# build_upstream_mac.sh — run ON the Mac. Builds slam_stream against UNMODIFIED ORB-SLAM3 V1.0
# (UZ-SLAMLab/ORB_SLAM3 4452a3c4 = v1.0-release source, none of patches/; one macOS stack fix below) next to the modified runtime, which it leaves alone:
#     ~/objpose/ORB_SLAM3_upstream   upstream source tree (copied there from the PC beforehand)
#     ~/objpose/src_upstream         copy of this directory (CMakeLists.txt, compat/, slam_stream.cc, headers)
#     ~/objpose/build_upstream       slam_stream compiled with -DOBJPOSE_UPSTREAM_ORB (no gyro hooks; --gyro refused)
#     ~/objpose/run_slam_upstream.sh run_slam.sh pointed at build_upstream (hub.py --mac-runner)
#   bash ~/objpose/src_upstream/build_upstream_mac.sh
set -euo pipefail
export PATH="/opt/homebrew/bin:$PATH"
DST="$HOME/objpose"; UP="$DST/ORB_SLAM3_upstream"; SRC="$DST/src_upstream"; B="$DST/build_upstream"
[ -d "$UP/src" ] || { echo "no upstream source at $UP" >&2; exit 1; }
# The one source edit, as orbslam_ws/openloris/build.sh does for its upstream build: MapPoint::
# ComputeDistinctiveDescriptors keeps an N x N float array on the stack (N = observations); macOS gives secondary
# threads 512 KB, so LocalMapping dies with SIGBUS on long runs. Same values, on the heap. (tr1 and stdint-gcc.h are
# covered by compat/.)
sed -i '' -e 's#float Distances\[N\]\[N\];#std::vector<std::vector<float> > Distances(N, std::vector<float>(N));#' \
    -e 's#vDists(Distances\[i\],Distances\[i\]+N)#vDists(Distances[i].begin(),Distances[i].end())#' "$UP/src/MapPoint.cc"
cmake -S "$SRC" -B "$B" -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_SRC="$UP" -DCMAKE_CXX_FLAGS=-DOBJPOSE_UPSTREAM_ORB \
      > "$B.cmake.log" 2>&1 || { tail -30 "$B.cmake.log"; exit 1; }
cmake --build "$B" -j"$(sysctl -n hw.ncpu)" > "$B.make.log" 2>&1 || { grep -E "error" "$B.make.log" | head -30; exit 1; }
sed -e 's#\$ROOT/build:#$ROOT/build_upstream:#' -e 's#"\$ROOT/build/slam_stream"#"$ROOT/build_upstream/slam_stream"#' \
    "$DST/run_slam.sh" > "$DST/run_slam_upstream.sh"
chmod +x "$DST/run_slam_upstream.sh"
grep -n build_upstream "$DST/run_slam_upstream.sh"
otool -L "$B/slam_stream" | grep -i orb
