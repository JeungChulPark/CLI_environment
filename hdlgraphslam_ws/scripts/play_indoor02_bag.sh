#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="${HDL_BAG_PATH:-/home/etri/convert_ros1bag_to_ros2bag/output/indoor02_sauna_normal_2022-02-21-19-05-17_ros2}"

if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "Missing ROS2 bag metadata: $BAG_PATH/metadata.yaml" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

exec ros2 bag play "$BAG_PATH" --clock "$@"
