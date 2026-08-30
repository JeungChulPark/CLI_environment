#!/usr/bin/env bash
set -u
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"; cd "$ROOT"
for BAG in SAM_circle SAM_loop1 SAM_loop2 SAM_occlusion; do
  echo "=================== $BAG : $(date) ==================="
  rm -rf "outputs/pem_inputs/$BAG" "outputs/rgbd_imu_sdk_bag/$BAG"
  bash tools/run_imu_e2e.sh "$BAG" 10
done
echo "=================== ALL DONE : $(date) ==================="
