#!/bin/bash
# run_lidar_orb_260910.sh — 260910_object, two SLAM lanes in one hub / one viewer (2026-09-14):
#
#   lane orb   ORB-SLAM3 f2000 on the Mac 192.168.20.55 (ssh direct), SLAM camera bag
#              Mac input  ~/Documents/DefenseMeta/Dataset/260910_object/SLAM      -> 192.168.20.5:17001
#   lane hdl   hdl_graph_slam (VLP-16) on THIS PC (conda hdl_graph_slam_jazzy + hdlgraphslam_ws)
#              PC input   /mnt/synology/Dataset/data_slam_converted/260910_object/lidar   -> 127.0.0.1:17002
#   SAM-6D     this PC, SAM bag /mnt/synology/Dataset/data_slam_converted/260910_object/SAM
#   viewer     http://192.168.20.5:8765  — one video per lane, one 3D map (reference = LiDAR, z-up),
#              ORB world rigidly aligned to the LiDAR world by the SAM-camera trajectory
#
#   bash objpose/run_lidar_orb_260910.sh                    # whole session (~159 s), Ctrl+C to exit
#   bash objpose/run_lidar_orb_260910.sh --duration-s 60    # hub.py options pass through
#   LANES=hdl bash objpose/run_lidar_orb_260910.sh          # LiDAR only, hdl_graph_slam on this PC
#   LANES=hdl LIDAR_HOST=mac bash objpose/run_lidar_orb_260910.sh   # LiDAR only, hdl_graph_slam on the Mac (ssh direct)
#   LANES=orb bash objpose/run_lidar_orb_260910.sh          # ORB only
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
PC_HOST="${PC_HOST:-192.168.20.5}"
MAC_SSH="${MAC_SSH:-mac}"
SLAM_HOST="${SLAM_HOST:-192.168.20.55}"
NAS="${NAS:-/mnt/synology/Dataset/data_slam_converted/260910_object}"
MAC_DATASET="${MAC_DATASET:-~/Documents/DefenseMeta/Dataset/260910_object}"
LANES="${LANES:-orb,hdl}"
LINK="${LINK:-auto}"                      # ORB lane: auto -> direct if ssh works, else manual
X_ORB="$REPO/objpose/rt/260910/X_slam_sam_260910.json"
X_LIDAR="$REPO/objpose/rt/260910/X_lidar_sam.json"
OUT="${OUT:-$REPO/objpose/output/lidar_orb_260910_$(date +%y%m%d_%H%M%S)}"
[ -d "$NAS/SAM" ] || { echo "SAM 세션이 없습니다: $NAS/SAM" >&2; exit 1; }
[ -d "$NAS/lidar" ] || { echo "lidar 세션이 없습니다: $NAS/lidar" >&2; exit 1; }

if [ "$LINK" = auto ]; then
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "$MAC_SSH" true 2>/dev/null; then LINK=direct; else
    LINK=manual; echo "[run] ssh $MAC_SSH 키 인증이 안 되어 ORB 레인은 manual 모드 (Mac 에서 출력된 명령 실행)" >&2; fi
fi
LIDAR_HOST="${LIDAR_HOST:-local}"         # local = hdl_graph_slam on this PC; mac = on the Mac over ssh (direct)
if [ "$LIDAR_HOST" = local ]; then
  HDL_LANE="name=hdl,slam=hdl,port=17002,ssh=local,session=$NAS/lidar,extrinsic=$X_LIDAR,label=LiDAR hdl_graph_slam"
  HDL0=(--slam hdl --link local --slam-port 17002 --extrinsic "$X_LIDAR" --mac-slam-session "$NAS/lidar")
else
  HDL_LANE="name=hdl,slam=hdl,port=17003,ssh=$MAC_SSH,link=direct,host=$SLAM_HOST,session=$MAC_DATASET/lidar,extrinsic=$X_LIDAR,label=LiDAR hdl_graph_slam (Mac)"
  HDL0=(--slam hdl --link direct --slam-port 17003 --extrinsic "$X_LIDAR" --mac-slam-session "$MAC_DATASET/lidar")
fi
case "$LANES" in
  orb,hdl|hdl,orb)
    LANE0=(--slam orbslam3 --link "$LINK" --slam-port 17001 --extrinsic "$X_ORB" --mac-slam-session "$MAC_DATASET/SLAM")
    EXTRA=(--lane "$HDL_LANE") ;;
  orb)
    LANE0=(--slam orbslam3 --link "$LINK" --slam-port 17001 --extrinsic "$X_ORB" --mac-slam-session "$MAC_DATASET/SLAM"); EXTRA=() ;;
  hdl)
    LANE0=("${HDL0[@]}"); EXTRA=() ;;
  *) echo "LANES=$LANES ?" >&2; exit 1 ;;
esac
echo "[run] lanes=$LANES  ORB link=$LINK  SLAM host=$SLAM_HOST  PC=$PC_HOST  out=$OUT" >&2
echo "[run] 뷰어: http://$PC_HOST:8765" >&2
exec "$PY" -u "$REPO/objpose/pc/hub.py" \
  "${LANE0[@]}" "${EXTRA[@]}" \
  --slam-host "$SLAM_HOST" --pc-host "$PC_HOST" --mac-host "$MAC_SSH" \
  --http-host 0.0.0.0 --http-port 8765 \
  --sam-session "$NAS/SAM" --slam-session "$NAS/SLAM" \
  --features 2000 --rate 1.0 --out "$OUT" "$@"
