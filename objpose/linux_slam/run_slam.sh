#!/bin/bash
# run_slam.sh — Linux SLAM host wrapper (same CLI as mac_slam/run_slam.sh; hub.py calls it as
# ~/objpose/run_slam.sh, a symlink to this file made by build_host.sh).
#   ~/objpose/run_slam.sh --slam orbslam3 --session <raw SLAM dir> --features 2000 \
#       --time-source frame --mode live --connect 192.168.20.5:17001 --rate 1.0
#   --session live: this host's D455f instead of a recording (linux_slam/rs_capture.py | slam_stream)
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
if [ "$backend" = rtabmap ]; then
  # build_rtab_mamba.sh links rtab_stream with an RPATH into the conda env that holds the RTAB-Map core
  [ -x "$ROOT/build_rtab/rtab_stream" ] || { echo "run_slam.sh (linux): $ROOT/build_rtab/rtab_stream not built (linux_slam/build_rtab_mamba.sh)" >&2; exit 2; }
  exec "$ROOT/build_rtab/rtab_stream" "${args[@]}"
fi
[ "$backend" = orbslam3 ] || { echo "run_slam.sh (linux): --slam orbslam3|rtabmap (got $backend)" >&2; exit 2; }

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
live=0
for ((i = 0; i + 1 < ${#args[@]}; i++)); do
  [ "${args[i]}" = "--session" ] && [ "${args[i+1]}" = "live" ] && live=1
done
if [ $live = 1 ]; then
  # this host's own D455f: rs_capture.py (PyPI pyrealsense2, V4L2 backend, no root) pipes frames to slam_stream.
  # RS_PY needs numpy; RS_PYPATH holds `pip install --target ... pyrealsense2`.
  RS_PY="${RS_PY:-$HOME/micromamba/envs/hdl_graph_slam_jazzy/bin/python}"
  RS_PYPATH="${RS_PYPATH:-$ROOT/deps/pyrs}"
  PYTHONPATH="$RS_PYPATH" "$RS_PY" -u "$ROOT/linux_slam/rs_capture.py" ${RS_SERIAL:+--serial "$RS_SERIAL"} \
    | "$ROOT/build/slam_stream" "${extra[@]}" "${args[@]}"
  exit "${PIPESTATUS[1]}"
fi
exec "$ROOT/build/slam_stream" "${extra[@]}" "${args[@]}"
