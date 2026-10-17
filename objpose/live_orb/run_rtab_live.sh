#!/bin/bash
# run_rtab_live.sh — LIVE RTAB-Map (RGB-D, this host's D455f) streamer for the object-pose hub; runs on the SLAM host
# (laptop etri-rog, ~/objpose/live_orb/). Bridge + RTAB-Map use system ROS 2 Humble and ~/CLI_environment/rtabmap_ws.
#   ~/objpose/live_orb/run_rtab_live.sh --slam rtabmap --session live --mode live --connect <hub>:17008 --rate 1.0
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${RTAB_WS:-$HOME/CLI_environment/rtabmap_ws}"
# a conda env left active by the ssh login shell would shadow the system Humble Python
while [ -n "${CONDA_PREFIX:-}" ] && command -v conda >/dev/null 2>&1; do conda deactivate 2>/dev/null || break; done
set +u
source /opt/ros/humble/setup.bash
# Fast DDS transport for the driver, RTAB-Map and the bridge (all Humble, Fast DDS 2.6), measured on the laptop at
# 640x480x30, 2026-09-17: default SHM (512 kB segments) 1-3 Hz odometry, UDP-only 6-9 Hz, SHM with 64 MB segments 27.8 Hz.
# FASTDDS_PROFILE=udp switches back to fastdds_udp_only.xml.
case "${FASTDDS_PROFILE:-shm}" in
  udp) export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_udp_only.xml" ;;
  *)   export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_shm_big.xml" ;;
esac
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
source "$WS/install/setup.bash" 2>/dev/null
exec /usr/bin/python3 -u "$HERE/rtab_live_stream.py" --ws "$WS" "$@"
