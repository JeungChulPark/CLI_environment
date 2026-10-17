#!/bin/bash
# run_hdl_stream.sh — hdl_graph_slam LiDAR streamer for the object-pose hub (same CLI as lidar_stream.py).
# Runs on the SLAM host: activates the `hdl_graph_slam_jazzy` conda env (robostack ROS 2 Jazzy, PCL, g2o),
# sources the hdlgraphslam_ws overlay and starts hdl_stream.py, which launches the hdl_graph_slam nodes itself.
#   ~/objpose/lidar/run_hdl_stream.sh --session <lidar dir> --mode live --connect <hub ip>:17001 --rate 1.0
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# conda env: HDL_ENV, else the first that exists of hdl_graph_slam_jazzy / hdl_graph_slam_humble
# (the laptop etri-rog builds the same sources with ROS 2 Humble, 2026-09-17)
if [ -z "${HDL_ENV:-}" ]; then
  for cand in hdl_graph_slam_jazzy hdl_graph_slam_humble; do
    for root in "$HOME/anaconda3" "$HOME/miniconda3" "$HOME/micromamba"; do
      [ -d "$root/envs/$cand" ] && HDL_ENV="$cand" && break 2
    done
  done
fi
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
# --attach 1 (shared hdl of the Nav2 stack, nav/real/real_up.sh): same RMW and shared-memory profile as that stack
case " $* " in *" --attach 1 "*)
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  [ -f "$HERE/../nav/real/fastdds_big_shm.xml" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../nav/real/fastdds_big_shm.xml" ;;
esac
exec python -u "$HERE/hdl_stream.py" "$@"
