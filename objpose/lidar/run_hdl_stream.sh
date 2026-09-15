#!/bin/bash
# run_hdl_stream.sh — hdl_graph_slam LiDAR streamer for the object-pose hub (same CLI as lidar_stream.py).
# Runs on the SLAM host: activates the `hdl_graph_slam_jazzy` conda env (robostack ROS 2 Jazzy, PCL, g2o),
# sources the hdlgraphslam_ws overlay and starts hdl_stream.py, which launches the hdl_graph_slam nodes itself.
#   ~/objpose/lidar/run_hdl_stream.sh --session <lidar dir> --mode live --connect <hub ip>:17001 --rate 1.0
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HDL_ENV="${HDL_ENV:-hdl_graph_slam_jazzy}"
# the workspace: next to objpose/ (CLI_environment/hdlgraphslam_ws), or a deployed copy inside ~/objpose (Mac)
for cand in "${HDL_WS:-}" "$HERE/../../hdlgraphslam_ws" "$HERE/../hdlgraphslam_ws" "$HERE/hdlgraphslam_ws"; do
  [ -n "$cand" ] && [ -d "$cand/install" ] && HDL_WS="$cand" && break
done
[ -d "${HDL_WS:-}/install" ] || { echo "hdlgraphslam_ws install not found (HDL_WS=$HDL_WS)" >&2; exit 2; }
set +u
if [ -x "$HOME/micromamba/bin/micromamba" ]; then            # Mac: micromamba root ~/micromamba
  export MAMBA_ROOT_PREFIX="$HOME/micromamba"
  eval "$("$HOME/micromamba/bin/micromamba" shell hook -s bash)"
  micromamba activate "$HDL_ENV"
else                                                          # Linux workstation: anaconda
  CONDA_SH="${CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
  [ -f "$CONDA_SH" ] || CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
  source "$CONDA_SH"
  conda activate "$HDL_ENV"
fi
source "$HDL_WS/install/setup.bash"
set -u
exec python -u "$HERE/hdl_stream.py" "$@"
