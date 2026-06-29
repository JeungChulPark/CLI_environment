#!/bin/bash
# RGB-D-only localization reproducibility eval orchestration.
# Phase 1: ORB3 (easyLC build) x 4 datasets x 3 runs, rate=1.0
# Phase 2: RTAB-Map x 4 datasets x 3 runs, rate=1.0
# Phase 3: reference selection + per-condition eval + viz (numpy/matplotlib env)
# New runs -> localization_eval/{ds}/{orb3_slam|rtabmap}/new_runs/run{1-3}/trajectory.txt
source /home/ldh9501/miniconda3/etc/profile.d/conda.sh
cd /home/ldh9501/temp_ws/CLI_environment
DS="SLAM_one_lap SLAM_one_lap_back_and_forth SLAM_forward_backward_repeat SLAM_three_laps"
N=3
EVAL=slam_comparison_report/localization_eval
LE=slam_comparison_report/loc_eval
PYV=/home/ldh9501/miniconda3/envs/rtabmap/bin/python3

mkdir -p $EVAL

echo "===== Phase 0: bag topic info $(date) ====="
conda activate rtabmap
{
  echo "# RGB-D topic matching (RGB-D-only; no odom/tf/GT input)"
  echo "RGB topic:        /camera/camera/color/image_raw"
  echo "Depth topic:      /camera/camera/aligned_depth_to_color/image_raw"
  echo "CameraInfo topic: /camera/camera/color/camera_info"
  echo ""
} > $EVAL/topics_matching.txt
for ds in $DS; do
  echo "## $ds" >> $EVAL/topics_matching.txt
  ros2 bag info data_slam/$ds 2>/dev/null | grep -iE "Topic:|Duration|Messages:" >> $EVAL/topics_matching.txt || echo "  (ros2 bag info unavailable)" >> $EVAL/topics_matching.txt
  echo "" >> $EVAL/topics_matching.txt
done

echo "===== Phase 1: ORB3 easyLC (rate1.0) $(date) ====="
conda activate orbslam3; source orbslam_ws/install/setup.bash
for ds in $DS; do
  log=$EVAL/$ds/orb3_slam/run_log.txt; mkdir -p $(dirname $log)
  echo "# ORB3-SLAM RGB-D-only run log | $ds | $(date)" > $log
  for k in $(seq 1 $N); do
    nd=$EVAL/$ds/orb3_slam/new_runs/run$k; mkdir -p $nd
    cmd="ORB_OUTPUT_SUBDIR=loc_tmp_orb ORB_BAG_RATE=1.0 ORB_DISABLE_DENSE=1 python3 orbslam_ws/scripts/run_orbslam_four_bags.py $ds"
    echo "[run$k] $cmd" >> $log
    ORB_OUTPUT_SUBDIR=loc_tmp_orb ORB_BAG_RATE=1.0 ORB_DISABLE_DENSE=1 \
      python3 orbslam_ws/scripts/run_orbslam_four_bags.py $ds >> $log 2>&1
    cp orbslam_ws/output/loc_tmp_orb/$ds/trajectory.txt $nd/trajectory.txt 2>/dev/null
    np=$( [ -f $nd/trajectory.txt ] && wc -l < $nd/trajectory.txt || echo 0)
    echo "[run$k] -> $np poses" >> $log
    echo "  ORB3 $ds run$k ($np poses)"
  done
done

echo "===== Phase 2: RTAB-Map (rate1.0) $(date) ====="
conda activate rtabmap
for ds in $DS; do
  log=$EVAL/$ds/rtabmap/run_log.txt; mkdir -p $(dirname $log)
  echo "# RTAB-Map RGB-D-only run log | $ds | $(date)" > $log
  for k in $(seq 1 $N); do
    nd=$EVAL/$ds/rtabmap/new_runs/run$k; mkdir -p $nd
    cmd="RTAB_OUTPUT_SUBDIR=loc_tmp_rtab RTAB_BAG_RATE=1.0 python3 rtabmap_ws/scripts/run_rtabmap_four_bags.py $ds"
    echo "[run$k] $cmd" >> $log
    RTAB_OUTPUT_SUBDIR=loc_tmp_rtab RTAB_BAG_RATE=1.0 \
      python3 rtabmap_ws/scripts/run_rtabmap_four_bags.py $ds >> $log 2>&1
    cp rtabmap_ws/output/loc_tmp_rtab/$ds/trajectory.txt $nd/trajectory.txt 2>/dev/null
    np=$( [ -f $nd/trajectory.txt ] && wc -l < $nd/trajectory.txt || echo 0)
    echo "[run$k] -> $np poses" >> $log
    echo "  RTAB $ds run$k ($np poses)"
  done
done

echo "===== Phase 3: reference selection + eval $(date) ====="
cd $LE
$PYV select_reference.py >/dev/null 2>&1
cd /home/ldh9501/temp_ws/CLI_environment
for ds in $DS; do
  $PYV $LE/eval_condition.py orb3 $ds $EVAL/$ds/orb3_slam/new_runs/run*
  $PYV $LE/eval_condition.py rtab $ds $EVAL/$ds/rtabmap/new_runs/run*
done

echo "===== cleanup temp $(date) ====="
rm -rf orbslam_ws/output/loc_tmp_orb rtabmap_ws/output/loc_tmp_rtab
echo "===== ALL DONE $(date) ====="
