#!/bin/bash
# run_cbnu_samsung.sh — one 260901_cbnu session, SLAM on the Samsung laptop (ssh samsung, 192.168.2.1, direct LAN),
# SAM-6D + object memory + viewer here (192.168.2.2). Called by pc/launcher.py (HTTP_PORT, OUT from the env), or by hand:
#   bash objpose/run_cbnu_samsung.sh <eightcircle|longcircle|digut|bigeightcircle> <orb|hdl|kiss>
# Clocks (2026-09-29, objpose/tools/overlay_error_decompose.py): the pose lookup offset is lane tau (extrinsic json
# sam_tau_s) minus --sam-tau-s and must be 0. eightcircle's converted SAM bag is ~208 ms late -> SAM tau -0.212 and lane
# tau -0.212 in both jsons; the other three sessions were converted with the genlock clock -> 0 / 0.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
SESSION="${1:?session: eightcircle|longcircle|digut|bigeightcircle}"; BACKEND="${2:?backend: orb|hdl|kiss}"
case "$SESSION" in eightcircle|longcircle|digut|bigeightcircle) ;; *) echo "unknown session $SESSION" >&2; exit 2 ;; esac
NAME="260901_cbnu_$SESSION"
SAM="/home/jucpark/DeepLearning/Datasets/data_slam_converted/$NAME/SAM"
RDIR="/home/user/DeepLearning/Dataset/$NAME"          # on the laptop
RT="$REPO/objpose/rt/260901_cbnu"
if [ "$SESSION" = eightcircle ]; then
  SAM_TAU=-0.212; X_CAM="$RT/X_slam_sam_260901_cbnu_eight.json"; X_LIDAR="$RT/X_lidar_sam_260901.json"
else
  SAM_TAU=0;      X_CAM="$RT/X_slam_sam_260901_cbnu.json";       X_LIDAR="$RT/X_lidar_sam_260901_tau0.json"
fi
case "$BACKEND" in
  orb)  LANE=(--slam orbslam3 --slam-port 17001 --features "${FEATURES:-3000}" --extrinsic "$X_CAM" --mac-slam-session "$RDIR/SLAM") ;;
  hdl)  LANE=(--slam hdl --slam-port 17005 --extrinsic "$X_LIDAR" --mac-slam-session "$RDIR/lidar") ;;
  kiss) LANE=(--slam lidar --slam-port 17007 --extrinsic "$X_LIDAR" --mac-slam-session "$RDIR/lidar"
              --lidar-runner "~/objpose/lidar/venv/bin/python -u ~/objpose/lidar/lidar_stream.py") ;;
  *) echo "backend: orb|hdl|kiss (got $BACKEND)" >&2; exit 2 ;;
esac
[ -d "$SAM" ] || { echo "SAM bag missing: $SAM" >&2; exit 1; }
OUT="${OUT:-$REPO/objpose/output/cbnu_${SESSION}_${BACKEND}_$(date +%y%m%d_%H%M%S)}"
mkdir -p "$OUT"
cd "$REPO"
exec "$PY" -u objpose/pc/hub.py "${LANE[@]}" --link direct --slam-host 192.168.2.1 --mac-host samsung --pc-host 192.168.2.2 \
  --http-host 0.0.0.0 --http-port "${HTTP_PORT:-8765}" --sam-session "$SAM" --sam-tau-s "$SAM_TAU" --rate 1.0 --out "$OUT"
