#!/usr/bin/env bash
# Run nav_test.py (ground truth from Gazebo). usage: nav_test.sh [world] x1,y1 x2,y2 ...
here=$(cd "$(dirname "$0")" && pwd)
source "$here/sim_env.sh"
world=warehouse; [[ ${1:-} =~ ^[a-z_]+$ ]] && { world=$1; shift; }
out="$here/logs/nav_test_$(date +%H%M%S).csv"
python3 "$here/nav_test.py" "$out" "$world" "$@"
echo "csv: $out"
