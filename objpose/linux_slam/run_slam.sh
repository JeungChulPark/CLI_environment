#!/bin/bash
# run_slam.sh — Linux SLAM host wrapper (same CLI as mac_slam/run_slam.sh; hub.py calls it as
# ~/objpose/run_slam.sh, a symlink to this file made by build_host.sh).
#   ~/objpose/run_slam.sh --slam orbslam3 --session <raw SLAM dir> --features 2000 \
#       --time-source frame --mode live --connect 192.168.20.5:17001 --rate 1.0
set -eu
ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"

backend=orbslam3
args=()
while [ $# -gt 0 ]; do
  if [ "$1" = "--slam" ]; then
    backend="${2:?--slam needs orbslam3}"; shift 2
  else
    args+=("$1"); shift
  fi
done
[ "$backend" = orbslam3 ] || { echo "run_slam.sh (linux): only --slam orbslam3 is built here (got $backend)" >&2; exit 2; }

export LD_LIBRARY_PATH="$ROOT/build:$ROOT/deps/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
has_settings=0; has_vocab=0
for arg in "${args[@]}"; do
  [ "$arg" = "--settings" ] && has_settings=1
  [ "$arg" = "--vocab" ] && has_vocab=1
done
extra=()
[ $has_settings = 0 ] && extra+=(--settings "$ROOT/mac_slam/settings/orbslam3_260826_dark_nf2000.yaml")
[ $has_vocab = 0 ] && extra+=(--vocab "$ROOT/Vocabulary/ORBvoc.txt")
exec "$ROOT/build/slam_stream" "${extra[@]}" "${args[@]}"
