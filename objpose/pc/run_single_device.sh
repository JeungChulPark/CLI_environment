#!/bin/bash
# Baseline for the paper: SLAM and recognition on ONE computer (this PC).
# Same hub, dataset and settings as the two-device run, but the SLAM streamer runs locally
# (~/objpose_pc, Linux build of mac_slam/slam_stream.cc) and competes with SAM-6D for CPU/GPU.
#   bash objpose/pc/run_single_device.sh <out_name> [--no-sam6d]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
DS=/home/jucpark/DeepLearning/Dataset/260915_eightcircle
OUT="$REPO/objpose/output/$1"; shift
pkill -f "objpose/pc/hub.py" ; sleep 3
"$PY" -u "$REPO/objpose/pc/hub.py" --slam orbslam3 --extrinsic "$REPO/objpose/rt/260915/X_slam_sam_260915.json" \
  --sam-session "$DS/SAM" --slam-session "$DS/SLAM" --mac-slam-session "$DS/SLAM" \
  --out "$OUT" --features 2000 --no-mac "$@" > "$OUT.log" 2>&1 &
HUB=$!
for i in $(seq 1 120); do grep -q "waiting for SLAM stream" "$OUT.log" 2>/dev/null && break; sleep 2; done
sleep 20   # let SAM-6D finish loading before the replay starts
~/objpose_pc/run_slam.sh --session "$DS/SLAM" --settings "$REPO/objpose/mac_slam/settings/orbslam3_260915_slam_nf2000.yaml" \
  --features 2000 --time-source frame --mode live --connect 127.0.0.1:17001 --rate 1.0 > "$OUT/pc_slam.log" 2>&1 &
for i in $(seq 1 400); do grep -q "summary written" "$OUT.log" 2>/dev/null && break; sleep 3; done
sleep 5; kill $HUB 2>/dev/null; pkill -f "objpose/pc/hub.py"; pkill -f slam_stream
echo "done: $OUT"
