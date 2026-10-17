#!/bin/bash
# run_one.sh — run ONE SLAM backend at a time with the object-pose hub, never two.
#   bash objpose/run_one.sh <dataset> <backend> [hub.py options]
#   bash objpose/run_one.sh <backend>            # old 2-arg form = dataset 260910_object
#
# dataset 260910_object                  (NAS data_slam_converted/260910_object, camera + VLP-16)
#   orb      ORB-SLAM3 f2000 on the Ubuntu box 192.168.20.3 (ssh slam3, --link direct), pose port 17001
#   orb-mac  ORB-SLAM3 f2000 on the Mac 192.168.20.55 (ssh mac, --link direct), pose port 17004
#   hdl      hdl_graph_slam (VLP-16) on this PC (conda hdl_graph_slam_jazzy, --link local), pose port 17002
#   hdl-3    hdl_graph_slam on 192.168.20.3, pose port 17005
#   hdl-mac  hdl_graph_slam on the Mac, pose port 17003
# dataset 260826_etri_eightcircle_dark   (NAS .../260826_etri_eightcircle_dark, camera only — no lidar folder)
#   orb      ORB-SLAM3 f2000 on 192.168.20.3, pose port 17001   (= the run_pc3.sh / run_lan.sh recipe)
#   orb-mac  ORB-SLAM3 f2000 on the Mac, pose port 17004        (= the run_lan.sh default recipe)
#
# SAM-6D, fusion, object memory and the viewer always run on this PC.
# Refuses to start while another hub.py is running (shared memory /sam6d_* and the pose ports are single-user).
# Env: HTTP_PORT (default 8765; the launcher uses 8766 and proxies), OUT, NAS, PC3_DATASET, LINK,
#      extra hub.py options after the backend.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python

USAGE="usage: run_one.sh <dataset> <backend> [hub.py options]   (datasets: live | 260910_object | 260915_dense | 260915_distance | 260915_eightcircle | 260915_inlong_fast | 260915_inlong_middle | 260915_outlong_fast | 260915_outlong_middle | 260915_height | 260826_etri_eightcircle_dark)"
# dataset "live" = the sensors themselves (laptop VLP-16 + this PC's D455f), backend hdl | kiss -> objpose/run_live.sh
if [ "${1:-}" = live ]; then
  shift; exec bash "$REPO/objpose/run_live.sh" "$@"
fi
# first argument may be the dataset; without it we keep the old behaviour (260910_object)
case "${1:-}" in
  260910_object|260915_dense|260915_distance|260915_eightcircle|260915_inlong_fast|260915_inlong_middle|260915_outlong_fast|260915_outlong_middle|260915_height|260826_etri_eightcircle_dark) DATASET="$1"; shift ;;
  *) DATASET="260910_object" ;;
esac
BACKEND="${1:?$USAGE}"; shift

PC_HOST="${PC_HOST:-192.168.20.5}"
HTTP_PORT="${HTTP_PORT:-8765}"
PC3_SSH="${PC3_SSH:-slam3}"
PC3_HOST="${PC3_HOST:-192.168.20.3}"
MAC_HOST="${MAC_HOST:-192.168.20.55}"
MAC_SSH="${MAC_SSH:-mac}"
LINK="${LINK:-direct}"
OUT="${OUT:-$REPO/objpose/output/one_${DATASET}_${BACKEND}_$(date +%y%m%d_%H%M%S)}"
COMMON=()   # per-dataset hub options that are not part of the lane (clock offset ...)

case "$DATASET" in
  260910_object)
    NAS="${NAS:-/mnt/synology/Dataset/data_slam_converted/260910_object}"
    PC3_DATASET="${PC3_DATASET:-/home/jucpark/DeepLearning/Dataset/data_slam_converted/260910_object}"  # on .3
    MAC_DATASET="${MAC_DATASET:-~/Documents/DefenseMeta/Dataset/260910_object}"                          # Mac shell expands ~
    X_CAM="$REPO/objpose/rt/260910/X_slam_sam_260910.json"
    X_LIDAR="$REPO/objpose/rt/260910/X_lidar_sam.json"
    # SAM tau comes from the extrinsic json for this dataset — do not pass --sam-tau-s
    case "$BACKEND" in
      # ORB feature counts from the 2026-09-14 sweep (docs/orb_improvement_plan_260914.md §2): Mac f4000 (track p99 20.6 ms),
      # .3 f3000 (f4000 hits the 33 ms budget there); override with FEATURES=<n>
      orb) LANE=(--slam orbslam3 --link "$LINK" --slam-port 17001 --extrinsic "$X_CAM"
                 --mac-slam-session "$PC3_DATASET/SLAM" --slam-host "$PC3_HOST" --mac-host "$PC3_SSH"); FEATURES="${FEATURES:-3000}" ;;
      hdl) LANE=(--slam hdl --link local --slam-port 17002 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$NAS/lidar" --slam-host "$PC_HOST" --mac-host local) ;;
      orb-mac) LANE=(--slam orbslam3 --link direct --slam-port 17004 --extrinsic "$X_CAM"
                 --mac-slam-session "$MAC_DATASET/SLAM" --slam-host "$MAC_HOST" --mac-host "$MAC_SSH"); FEATURES="${FEATURES:-4000}" ;;
      kiss-3) LANE=(--slam lidar --link direct --slam-port 17007 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$PC3_DATASET/lidar" --slam-host "$PC3_HOST" --mac-host "$PC3_SSH") ;;
      kiss-mac) LANE=(--slam lidar --link direct --slam-port 17006 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$MAC_DATASET/lidar" --slam-host "$MAC_HOST" --mac-host "$MAC_SSH") ;;
      hdl-3) LANE=(--slam hdl --link direct --slam-port 17005 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$PC3_DATASET/lidar" --slam-host "$PC3_HOST" --mac-host "$PC3_SSH") ;;
      hdl-mac) LANE=(--slam hdl --link direct --slam-port 17003 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$MAC_DATASET/lidar" --slam-host "$MAC_HOST" --mac-host "$MAC_SSH") ;;
      *) echo "backend for 260910_object: orb|orb-mac|hdl|hdl-3|hdl-mac|kiss-3|kiss-mac (got $BACKEND)" >&2; exit 2 ;;
    esac ;;
  260915_dense|260915_distance|260915_eightcircle|260915_inlong_fast|260915_inlong_middle|260915_outlong_fast|260915_outlong_middle|260915_height)
    # 2026-09-15 recordings (dense / distance / eightcircle), converted bags (header stamps on the shared clock, SAM offset from the session manifest);
    # RT re-estimated on 260915_eightcircle (docs/rig_rt_260915.md): the SLAM camera moved ~5 cm / 1.7 deg on the
    # rig since 260910, the SAM camera and the VLP-16 did not (LiDAR->SAM RT within 0.3 cm / 0.6 deg). sam tau from
    # the json = 0. SLAM host: .3 has the converted copy under ~/DeepLearning/Dataset/data_slam_converted/260915_dense.
    NAS="${NAS:-/mnt/synology/Dataset/data_slam_converted/$DATASET}"
    PC3_DATASET="${PC3_DATASET:-/home/jucpark/DeepLearning/Dataset/data_slam_converted/$DATASET}"   # on .3
    MAC_DATASET="${MAC_DATASET:-~/Documents/DefenseMeta/Dataset/$DATASET}"                                  # Mac copy (if present)
    X_CAM="$REPO/objpose/rt/260915/X_slam_sam_260915.json"
    X_LIDAR="$REPO/objpose/rt/260915/X_lidar_sam_260915.json"
    case "$BACKEND" in
      orb) LANE=(--slam orbslam3 --link "$LINK" --slam-port 17001 --extrinsic "$X_CAM"
                 --mac-slam-session "$PC3_DATASET/SLAM" --slam-host "$PC3_HOST" --mac-host "$PC3_SSH"); FEATURES="${FEATURES:-3000}" ;;
      orb-mac) LANE=(--slam orbslam3 --link direct --slam-port 17004 --extrinsic "$X_CAM"
                 --mac-slam-session "$MAC_DATASET/SLAM" --slam-host "$MAC_HOST" --mac-host "$MAC_SSH"); FEATURES="${FEATURES:-4000}" ;;
      kiss-3) LANE=(--slam lidar --link direct --slam-port 17007 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$PC3_DATASET/lidar" --slam-host "$PC3_HOST" --mac-host "$PC3_SSH") ;;
      hdl-3) LANE=(--slam hdl --link direct --slam-port 17005 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$PC3_DATASET/lidar" --slam-host "$PC3_HOST" --mac-host "$PC3_SSH") ;;
      kiss-mac) LANE=(--slam lidar --link direct --slam-port 17006 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$MAC_DATASET/lidar" --slam-host "$MAC_HOST" --mac-host "$MAC_SSH") ;;
      hdl-mac) LANE=(--slam hdl --link direct --slam-port 17003 --extrinsic "$X_LIDAR"
                 --mac-slam-session "$MAC_DATASET/lidar" --slam-host "$MAC_HOST" --mac-host "$MAC_SSH") ;;
      *) echo "backend for $DATASET: orb|orb-mac|kiss-3|hdl-3|kiss-mac|hdl-mac (got $BACKEND)" >&2; exit 2 ;;
    esac ;;
  260826_etri_eightcircle_dark)
    # same inputs as objpose/run_pc3.sh + run_lan.sh: converted SAM/SLAM on the NAS, the raw SDK SLAM
    # session on the SLAM host, camera RT objpose/rt/X_slam_sam.json and the converted-bag SAM tau
    # (-48 ms frame-clock offset + 0.553 ms conversion shift = -47.4 ms; see run_lan.sh).
    NAS="${NAS:-/mnt/synology/Dataset/data_slam_converted/260826_etri_eightcircle_dark}"
    PC3_DATASET="${PC3_DATASET:-~/Dataset/260826_etri_eightcircle_dark}"                                 # on .3, shell expands ~
    MAC_DATASET="${MAC_DATASET:-~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark}"
    X_CAM="$REPO/objpose/rt/X_slam_sam.json"
    COMMON=(--sam-tau-s "${SAM_TAU_S:--0.0474}")
    case "$BACKEND" in
      orb) LANE=(--slam orbslam3 --link "$LINK" --slam-port 17001 --extrinsic "$X_CAM"
                 --mac-slam-session "$PC3_DATASET/SLAM" --slam-host "$PC3_HOST" --mac-host "$PC3_SSH") ;;
      orb-mac) LANE=(--slam orbslam3 --link direct --slam-port 17004 --extrinsic "$X_CAM"
                 --mac-slam-session "$MAC_DATASET/SLAM" --slam-host "$MAC_HOST" --mac-host "$MAC_SSH") ;;
      *) echo "backend for 260826_etri_eightcircle_dark: orb|orb-mac (no lidar folder in this dataset; got $BACKEND)" >&2; exit 2 ;;
    esac ;;
  *) echo "dataset: 260910_object|260826_etri_eightcircle_dark (got $DATASET)" >&2; exit 2 ;;
esac

[ -d "$NAS/SAM" ] || { echo "SAM 세션이 없습니다: $NAS/SAM" >&2; exit 1; }
[ -f "$X_CAM" ] || { echo "카메라 RT 가 없습니다: $X_CAM" >&2; exit 1; }
# match only a real hub process (python ... objpose/pc/hub.py), not a shell whose command line merely mentions it
if pgrep -f '^[^ ]*python[^ ]* -u [^ ]*objpose/pc/hub\.py' >/dev/null; then
  echo "다른 허브가 이미 실행 중입니다 (한 번에 하나만): $(pgrep -af 'objpose/pc/hub.py' | cut -c1-120)" >&2; exit 3
fi
echo "[run_one] dataset=$DATASET backend=$BACKEND  viewer http://$PC_HOST:$HTTP_PORT  out=$OUT" >&2
# ORB-SLAM3 depth threshold (Stereo.ThDepth, x Stereo.b 0.0745 m): THDEPTH=40 -> 3.0 m (yaml default), 70 -> 5.2 m.
# Reaches the streamer as --param (slam_stream.cc, 2026-09-16); ORB-SLAM3 requires a real number -> ".0".
if [ -n "${THDEPTH:-}" ]; then
  case " ${LANE[*]} " in *" orbslam3 "*) COMMON+=(--slam-param "Stereo.ThDepth=${THDEPTH%.*}.0") ;; esac
fi
exec "$PY" -u "$REPO/objpose/pc/hub.py" "${LANE[@]}" "${COMMON[@]+"${COMMON[@]}"}" --pc-host "$PC_HOST" \
  --http-host 0.0.0.0 --http-port "$HTTP_PORT" \
  --sam-session "$NAS/SAM" --slam-session "$NAS/SLAM" --features "${FEATURES:-2000}" --rate 1.0 --out "$OUT" "$@"
