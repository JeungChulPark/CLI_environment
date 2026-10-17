#!/usr/bin/env bash
# Live SLAM on a 30 Hz bag replay, watched in the browser: ORB-SLAM3 or RTAB-Map, chosen on the page.
#
#   orbslam_ws/mac_harness/run_live.sh [BAG.db3] [live_hub.py flags...]
#
# Expects build_mac.sh to have been run into orbslam_ws/output/mac_build (BUILD=... to override).
# The page (http://127.0.0.1:8080/) starts and stops the backends; Ctrl+C here stops everything.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/.." && pwd)
BAG=${1:-/Users/user/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM/recording_20260826_164423.db3}
shift $(( $# < 1 ? $# : 1 ))
PORT=${PORT:-8080}

[ -z "${NO_OPEN:-}" ] && (sleep 1 && open "http://127.0.0.1:$PORT/") &
exec python3 "$HERE/live_hub.py" --port "$PORT" --bag "$BAG" --build "${BUILD:-$WS/output/mac_build}" "$@"
