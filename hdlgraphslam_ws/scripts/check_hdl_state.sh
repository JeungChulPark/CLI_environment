#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROCESS_PATTERN='ros2|rosbag2|hdl_graph_slam|rtabmap|graph_slam_node|scan_matching_odometry'

echo "ROS2/SLAM processes:"
if ps -eo pid=,ppid=,comm=,args= | awk -v self_pid="$$" -v pattern="$PROCESS_PATTERN" '
  $1 == self_pid { next }
  /codex-linux-sandbox/ { next }
  /check_hdl_state[.]sh/ { next }
  $0 ~ pattern {
    print
    found = 1
  }
  END {
    exit found ? 0 : 1
  }
'; then
  :
else
  echo "none"
fi

echo
echo "Output directory:"
ls -lh "$ROOT_DIR/output"

echo
echo "Run summary:"
cat "$ROOT_DIR/output/run_summary.txt"
