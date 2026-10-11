#!/bin/bash
# Live YCB-V playback: hub (object map, display) + SAM-6D on this GPU + ORB-SLAM3 (local Linux build of
# slam_stream) on the same video at 1x. One run per scene; outputs objpose/output/ycbv_live_<tag>_<scene>.
#   bash run_live.sh <tag> [scenes...]       e.g. bash run_live.sh v1 53
# Env overrides: YCBV_BAGS, YCBV_ISM (object list), SLAM_RUN (slam_stream wrapper), SAM6D_PY,
#   HUB_EXTRA (extra hub options, e.g. --map-prior).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}
BAGS=${YCBV_BAGS:-$HOME/DeepLearning/Dataset/bop/ycbv_bags}
ISM=${YCBV_ISM:-$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml}
SLAM_RUN=${SLAM_RUN:-$HOME/objpose_pc/run_slam.sh}
SET="$REPO/objpose/mac_slam/settings/orbslam3_ycbv_nf2000.yaml"
[ -f "$SET" ] || sed -e 's/^Camera.fps: .*/Camera.fps: 30/' "$REPO/objpose/mac_slam/settings/orbslam3_260915_slam_nf2000.yaml" > "$SET"
TAG=$1; shift
SCENES=${*:-48 49 50 51 52 53 54 55 56 57 58 59}
for s in $SCENES; do
  sc=$(printf %06d $s); OUT="$REPO/objpose/output/ycbv_live_${TAG}_${sc}"; mkdir -p "$OUT"
  pkill -f "objpose/pc/hub.py"; pkill -f slam_stream; sleep 3
  "$PY" -u "$REPO/objpose/pc/hub.py" --slam orbslam3 --extrinsic "$REPO/objpose/rt/ycbv/X_identity.json" \
     --sam-session "$BAGS/ycbv_$sc/SAM" --slam-session "$BAGS/ycbv_$sc/SLAM" --mac-slam-session "$BAGS/ycbv_$sc/SLAM" \
     --out "$OUT" --features 2000 --no-mac --ism-config "$ISM" --cluster-occupancy 0.3 --start-s 0.0 --tail-s 4 ${HUB_EXTRA:-} \
     > "$OUT.log" 2>&1 &
  HUB=$!
  for i in $(seq 1 150); do grep -q "waiting for SLAM stream" "$OUT.log" 2>/dev/null && break; sleep 2; done
  for i in $(seq 1 150); do [ -f "$OUT/sam6d/READY" ] && break; sleep 2; done
  "$SLAM_RUN" --session "$BAGS/ycbv_$sc/SLAM" --settings "$SET" --features 2000 --time-source frame \
     --mode live --connect 127.0.0.1:17001 --rate 1.0 > "$OUT/pc_slam.log" 2>&1 &
  for i in $(seq 1 600); do grep -q "summary written" "$OUT.log" 2>/dev/null && break; sleep 3; done
  sleep 3; kill $HUB 2>/dev/null; pkill -f "objpose/pc/hub.py"; pkill -f slam_stream
  echo "done $sc"
done
