#!/bin/bash
# Wrapper for the SLAM streamers on the Mac. Same CLI for both backends:
#   --slam orbslam3 (default) -> slam_stream   (ORB-SLAM3; defaults: SLAM-camera NF2000 settings, ORB vocabulary)
#   --slam rtabmap            -> rtab_stream   (RTAB-Map; --settings/--vocab accepted and ignored)
# Examples:
#   ~/objpose/run_slam.sh --session <SLAM dir> --features 2000 --mode live --connect 127.0.0.1:17001
#   ~/objpose/run_slam.sh --slam rtabmap --session <SLAM dir> --features 2000 --mode live --connect 127.0.0.1:17001
#   ~/objpose/run_slam.sh --session <SAM dir> --settings ~/objpose/settings/sam_camera_nf2000.yaml \
#       --offset-ns -18225662057 --features 2000 --mode offline --out sam_traj_tum.txt
set -eu
ROOT="$HOME/objpose"

backend=orbslam3
args=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--slam" ]; then
    backend="${2:?--slam needs orbslam3|rtabmap}"
    shift 2
  else
    args+=("$1")
    shift
  fi
done

case "$backend" in
  rtabmap)
    export DYLD_LIBRARY_PATH="/opt/homebrew/opt/rtabmap/lib:/opt/homebrew/opt/opencv@4/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
    exec "$ROOT/build_rtab/rtab_stream" "${args[@]}"
    ;;
  orbslam3)
    export DYLD_LIBRARY_PATH="$ROOT/build:$ROOT/deps/install/lib:/opt/homebrew/opt/opencv@4/lib:/opt/homebrew/opt/libomp/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
    has_settings=0; has_vocab=0
    for arg in "${args[@]}"; do
      [ "$arg" = "--settings" ] && has_settings=1
      [ "$arg" = "--vocab" ] && has_vocab=1
    done
    extra=()
    [ $has_settings = 0 ] && extra+=(--settings "$ROOT/settings/orbslam3_260826_dark_nf2000.yaml")
    [ $has_vocab = 0 ] && extra+=(--vocab "$ROOT/Vocabulary/ORBvoc.txt")
    exec "$ROOT/build/slam_stream" "${extra[@]}" "${args[@]}"
    ;;
  *)
    echo "unknown --slam $backend (orbslam3|rtabmap)" >&2
    exit 2
    ;;
esac
