#!/bin/bash
# run_pc3.sh — the LAN profile (run_lan.sh) with the Ubuntu box 192.168.20.3 as the SLAM host instead
# of the Mac. Everything else is the existing pipeline: ORB-SLAM3 (f2000) replays the raw SLAM session
# on .3 and streams poses to this PC's 192.168.20.5:17001, SAM-6D + fusion run here, viewer at
# http://192.168.20.5:8765.
#
#   once:  bash objpose/linux_slam/deploy.sh          # bundle + dataset -> .3, build there (needs ssh key on .3)
#   run:   bash objpose/run_pc3.sh [hub.py options]   # LINK=auto: direct if ssh works, else manual
#          LINK=manual bash objpose/run_pc3.sh        # then on .3:  ~/objpose/run_slam.sh ... (printed by the hub)
set -euo pipefail
export SLAM_HOST="${SLAM_HOST:-192.168.20.3}"
export MAC_SSH="${MAC_SSH:-slam3}"                        # ~/.ssh/config: HostName 192.168.20.3, Port 10022, User jucpark
export MAC_DATASET="${MAC_DATASET:-~/Dataset/260826_etri_eightcircle_dark}"
export OUT="${OUT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/output/pc3_$(date +%y%m%d_%H%M%S)}"
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_lan.sh" "$@"
