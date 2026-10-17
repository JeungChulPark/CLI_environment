#!/bin/bash
# run_offline_orb.sh — recorded converted session, ORB-SLAM3 on the laptop's orbslam3_ros2 (bag mode of
# live_orb/orb_live_stream.py), SAM-6D + object memory + viewer on this PC. Same SLAM code path as the live button,
# so feature counts can be compared on identical input.
#   FEATURES=3000 bash objpose/run_offline_orb.sh [session=260915_inlong_middle]      viewer http://192.168.240.2:${HTTP_PORT:-8767}
#   laptop needs ~/sam6d_object_memory/data_slam_converted/<session>/{SLAM,info.json}
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
SESSION="${1:-260915_inlong_middle}"; [ $# -gt 0 ] && shift     # the rest goes to hub.py (e.g. --depth-max-speed 0.6)
FEATURES="${FEATURES:-2000}"
PC_DATA="${PC_DATA:-$HOME/DeepLearning/Datasets/data_slam_converted/260915_six_routes/$SESSION}"
LAPTOP_DATA="${LAPTOP_DATA:-/home/etri/sam6d_object_memory/data_slam_converted/$SESSION}"
LAPTOP_SSH="${LAPTOP_SSH:-laptop240}"; LAPTOP_HOST="${LAPTOP_HOST:-192.168.240.1}"; PC_HOST="${PC_HOST:-192.168.240.2}"
OUT="${OUT:-$REPO/objpose/output/offline_orb_f${FEATURES}_${SESSION}_$(date +%y%m%d_%H%M%S)}"
mkdir -p "$OUT"
exec "$PY" -u "$REPO/objpose/pc/hub.py" --slam orbslam3 --link direct --slam-port 17001 \
  --extrinsic "$REPO/objpose/rt/260915/X_slam_sam_260915.json" \
  --mac-runner "~/objpose/live_orb/run_orb_live.sh" --mac-slam-session "$LAPTOP_DATA/SLAM" --mac-host "$LAPTOP_SSH" \
  --slam-host "$LAPTOP_HOST" --pc-host "$PC_HOST" --features "$FEATURES" \
  --sam-session "$PC_DATA/SAM" --http-host 0.0.0.0 --http-port "${HTTP_PORT:-8767}" --rate 1.0 --exit-when-done --out "$OUT" "$@"
