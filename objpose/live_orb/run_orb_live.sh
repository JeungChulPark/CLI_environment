#!/bin/bash
# run_orb_live.sh — LIVE ORB-SLAM3 (RGB-D, this host's D455f) streamer for the object-pose hub; runs on the SLAM host
# (laptop etri-rog, deployed at ~/objpose/live_orb/). The hub calls it over ssh with its orbslam3 lane arguments:
#   ~/objpose/live_orb/run_orb_live.sh --slam orbslam3 --session live --features 2000 --mode live --connect <hub>:17001 --rate 1.0
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV="${ORB_BRIDGE_ENV:-orbslam3}"                       # rclpy (ROS 2 Jazzy) for the bridge itself
set +u
CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"; [ -f "$CONDA_SH" ] || CONDA_SH="$HOME/anaconda3/etc/profile.d/conda.sh"
source "$CONDA_SH"
conda activate "$ENV"
set -u
exec python -u "$HERE/orb_live_stream.py" "$@"
