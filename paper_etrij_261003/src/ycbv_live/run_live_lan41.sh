#!/bin/bash
# run_live_lan41.sh — run_live.sh 의 서버-클라이언트판 (2026-10-08).
#   서버(이 PC .41): hub.py = SAM-6D + 객체 지도. SAM/SLAM 세션은 로컬 bags.
#   클라이언트(.192, ssh 별칭 slam192): hub 가 ssh -R 역터널로 ~/objpose/run_slam.sh 를 띄워
#   같은 장면의 SLAM bag(.192 로컬 사본)을 1배속 재생. 로컬 slam_stream 실행 구간만 바꿨고
#   나머지(장면 목록, 옵션, 출력 위치)는 run_live.sh 그대로.
#   bash run_live_lan41.sh <tag> [scenes...]
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}
BAGS=${YCBV_BAGS:-$HOME/DeepLearning/Dataset/bop/ycbv_bags}
ISM=${YCBV_ISM:-$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml}
SLAM_SSH=${SLAM_SSH:-slam192}
SLAM_BAGS=${SLAM_BAGS:-/home/jucpark/DeepLearning/Dataset/bop/ycbv_bags}
SLAM_SET=${SLAM_SET:-/home/jucpark/DeepLearning/CLI_environment/objpose/mac_slam/settings/orbslam3_ycbv_nf2000.yaml}
TAG=$1; shift
SCENES=${*:-48 49 50 51 52 53 54 55 56 57 58 59}
for s in $SCENES; do
  sc=$(printf %06d $s); OUT="$REPO/objpose/output/ycbv_live_${TAG}_${sc}"; mkdir -p "$OUT"
  pkill -f "objpose/pc/hub.py"; pkill -f "sam6d_infe[r]"
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$SLAM_SSH" "pkill -f slam_strea[m]" 2>/dev/null
  sleep 3
  "$PY" -u "$REPO/objpose/pc/hub.py" --slam orbslam3 --extrinsic "$REPO/objpose/rt/ycbv/X_identity.json" \
     --sam-session "$BAGS/ycbv_$sc/SAM" --slam-session "$BAGS/ycbv_$sc/SLAM" \
     --mac-host "$SLAM_SSH" --mac-runner '~/objpose/run_slam.sh' \
     --mac-slam-session "$SLAM_BAGS/ycbv_$sc/SLAM" "--slam-arg=--settings" "--slam-arg=$SLAM_SET" \
     --out "$OUT" --features 2000 --http-port 8766 --ism-config "$ISM" --cluster-occupancy 0.3 --start-s 0.0 --tail-s 4 ${HUB_EXTRA:-} \
     > "$OUT.log" 2>&1 &
  HUB=$!
  for i in $(seq 1 600); do grep -q "summary written" "$OUT.log" 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
  sleep 3; kill $HUB 2>/dev/null; pkill -f "objpose/pc/hub.py"; pkill -f "sam6d_infe[r]"
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$SLAM_SSH" "pkill -f slam_strea[m]" 2>/dev/null
  echo "done $sc"
done
