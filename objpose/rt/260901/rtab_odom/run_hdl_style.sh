#!/bin/bash
# run_hdl_style.sh — hdl_graph_slam's loop-closure recipe, expressed in RTAB-Map's parameters.
#
# hdl_graph_slam closes loops three ways differently from the ICP lane run so far
# (hdlgraphslam_ws/src/hdl_graph_slam/include/hdl_graph_slam/loop_detector.hpp and
#  launch/hdl_graph_slam_imu.launch):
#
#   candidates   by DISTANCE, not appearance: at least accum_distance_thresh = 3 m apart along the
#                travelled path, and within distance_thresh = 1 m in the xy plane. No bag of words.
#   registration GICP — plane to plane. The ICP lane here ran point-to-point, because this build
#                defaults Icp/PointToPlane to false, which is the variant that slides along flat
#                surfaces instead of locking onto them.
#   acceptance   fitness_score_thresh = 0.5 on the registration residual, rather than a cap on how
#                far the link may move (Icp/MaxTranslation), which the sweep showed was acting as a
#                filter against wrong links rather than a limit on good ones.
#
# RTAB-Map cannot be made identical — it has point-to-plane, not GICP's plane-to-plane with both
# clouds' covariances — so these are approximations, taken one at a time so the cause of any change
# is attributable:
#
#   hdl1   Icp/PointToPlane=true                     the registration change alone
#   hdl2   + RGBD/LocalRadius=1.0                    hdl's 1 m proximity radius (default is 10 m)
#   hdl3   + Rtabmap/LoopThr=1.0                     appearance loops off; only proximity links,
#                                                    which is hdl's scheme end to end
#
#   bash run_hdl_style.sh [session ...]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REF="$HERE/../bigeight/Userui-MacBookPro"
DATA=/Users/user/Documents/DefenseMeta/Dataset
RUN="${RUN:-$HOME/objpose/run_slam.sh}"
SESSIONS=${*:-"bigeightcircle eightcircle longcircle digut"}

for s in $SESSIONS; do
  d="$DATA/260901_cbnu_$s"
  rig="$HERE/rigs/rig_$s.json"
  kiss="$REF/260901_cbnu_$s/kiss/kiss_tum.txt"
  out="$HERE/hdl/$s"; mkdir -p "$out"
  [ -f "$rig" ] && [ -f "$kiss" ] || { echo "== $s: 입력 없음"; continue; }
  echo "== [$(date +%T)] $s"
  common=(--slam rtabmap --session "$d/SLAM" --time-source frame --mode offline
          --rig "$rig" --ext-odom "$kiss" --lidar "$d/lidar" --lidar-z -0.45)

  ( $RUN "${common[@]}" --param Icp/PointToPlane=true \
         --out "$out/hdl1_tum.txt" > "$out/hdl1.log" 2>&1 ) &
  ( $RUN "${common[@]}" --param Icp/PointToPlane=true --param RGBD/LocalRadius=1.0 \
         --out "$out/hdl2_tum.txt" > "$out/hdl2.log" 2>&1 ) &
  ( $RUN "${common[@]}" --param Icp/PointToPlane=true --param RGBD/LocalRadius=1.0 \
         --param Rtabmap/LoopThr=1.0 \
         --out "$out/hdl3_tum.txt" > "$out/hdl3.log" 2>&1 ) &
  wait
  for v in hdl1 hdl2 hdl3; do
    printf "   %-5s " $v; grep -h "done:" "$out/$v.log" | cut -c15-120
  done
  rm -f "$out"/*.rtabmap.db "$out"/*.frames.csv
done
echo "== [$(date +%T)] 완료"
