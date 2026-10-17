#!/bin/bash
# run_lidar_stream.sh — KISS-ICP streamer (lidar_stream.py) on a Linux SLAM host whose kiss_icp + rclpy live in a
# conda env instead of ~/objpose/lidar/venv (laptop etri-rog: env hdl_graph_slam_humble, 2026-09-17). Same CLI as
# lidar_stream.py; `--session live` also needs the ROS 2 velodyne driver from that env.
#   ~/objpose/lidar/run_lidar_stream.sh --session live --mode live --connect <hub ip>:17007 --rate 1.0
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${KISS_ENV:-}" ]; then
  for cand in hdl_graph_slam_jazzy hdl_graph_slam_humble kiss_icp; do
    for root in "$HOME/anaconda3" "$HOME/miniconda3"; do
      [ -d "$root/envs/$cand" ] && "$root/envs/$cand/bin/python" -c "import kiss_icp" 2>/dev/null && KISS_ENV="$cand" && break 2
    done
  done
fi
[ -n "${KISS_ENV:-}" ] || { echo "no conda env with kiss_icp (set KISS_ENV)" >&2; exit 2; }
set +u
CONDA_SH="${CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
[ -f "$CONDA_SH" ] || CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
source "$CONDA_SH"
conda activate "$KISS_ENV"
set -u
exec python -u "$HERE/lidar_stream.py" "$@"
