#!/bin/bash
# run_bench.sh — run every SLAM system on every real session that has a LiDAR bag, for scoring against the
# LiDAR pseudo-GT (objpose/rt/gt/out/<session>/pgo_tum.txt) with score_bench.py.
#
#   bash objpose/rt/bench/run_bench.sh [session ...]     (default: every Dataset/26* session with lidar/)
#
# Per session -> out/<session>/<system>/run<REP>/:
#   orb        ORB-SLAM3 (~/objpose/build, slam_stream), NF2000, real-time paced (--pace 1.0)
#   orb_gyro   same + xsens gyro aid (only sessions with imu/; rig = rt/260901/rtab_odom/rigs/rig_<name>.json)
#   rtab       RTAB-Map, its own RGB-D visual odometry (rtab_stream defaults), offline unpaced
#   hdl        hdl_graph_slam (ROS 2 Jazzy, fast_gicp odometry + loop closure + floor), real-time at bag rate,
#              recorded by hub_record.py
#   kiss       KISS-ICP odometry, not run again: it is step 1 of the pseudo-GT (odom_tum.txt), so its score is
#              NOT independent of the reference
# Camera settings: ~/objpose/settings/orbslam3_<260901|260915>_slam_nf2000.yaml with the bag's own intrinsics.
# Env: REP (default 1), SYSTEMS (default "orb orb_gyro rtab hdl"), PORT (default 17101)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
DATA="${DATA:-$HOME/Documents/DefenseMeta/Dataset}"
RUN="$HOME/objpose/run_slam.sh"
HDL="$HOME/objpose/lidar/run_hdl_stream.sh"
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
REP="${REP:-1}"; SYSTEMS="${SYSTEMS:-orb orb_gyro rtab hdl}"; PORT="${PORT:-17101}"
if [ $# -gt 0 ]; then S=("$@"); else S=(); for d in "$DATA"/26*/lidar; do S+=("$(basename "$(dirname "$d")")"); done; fi

for s in "${S[@]}"; do
  d="$DATA/$s"; out="$HERE/out/$s"; mkdir -p "$out"
  case "$s" in 260901_*) base="$HOME/objpose/settings/orbslam3_260901_slam_nf2000.yaml" ;;
                *)        base="$HOME/objpose/settings/orbslam3_260915_slam_nf2000.yaml" ;; esac
  settings=$("$PY" - "$d/SLAM" "$base" "$out" <<'PY'
import sys
sys.path.insert(0, "/Users/user/Documents/DefenseMeta/CLI_environment/orbslam_ws/mac_harness")
from pathlib import Path
import live_hub
bag = next(Path(sys.argv[1]).glob("*.db3"))
print(live_hub.settings_for(str(bag), sys.argv[2], Path(sys.argv[3])))
PY
)
  echo "== [$(date +%T)] $s  (settings $settings)"
  for sys in $SYSTEMS; do
    r="$out/$sys/run$REP"
    [ -s "$r/live_tum.txt" ] && { echo "   $sys: done"; continue; }
    mkdir -p "$r"
    case $sys in
      orb)
        "$RUN" --slam orbslam3 --session "$d/SLAM" --settings "$settings" --features 2000 --time-source frame \
          --mode offline --pace 1.0 --out "$r/live_tum.txt" > "$r/run.log" 2>&1 ;;
      orb_gyro)
        name=${s#260901_cbnu_}; rig="$REPO/objpose/rt/260901/rtab_odom/rigs/rig_$name.json"
        [ -d "$d/imu" ] && [ -f "$rig" ] || { rmdir "$r" "$out/$sys" 2>/dev/null; continue; }
        "$RUN" --slam orbslam3 --session "$d/SLAM" --settings "$settings" --features 2000 --time-source frame \
          --mode offline --pace 1.0 --gyro "$rig" --imu "$d/imu" --out "$r/live_tum.txt" > "$r/run.log" 2>&1 ;;
      rtab)
        "$RUN" --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
          --out "$r/live_tum.txt" > "$r/run.log" 2>&1 ;;
      hdl)
        "$PY" "$HERE/hub_record.py" --port "$PORT" --out "$r" > "$r/hub.log" 2>&1 &
        hub=$!; sleep 1
        "$HDL" --session "$d/lidar" --mode live --connect "127.0.0.1:$PORT" --rate 1.0 > "$r/run.log" 2>&1
        wait $hub ;;
    esac
    # slam_stream / rtab_stream write the end-of-run optimised trajectory next to --out
    [ -f "$r/live_tum.txt.optimized.txt" ] && mv "$r/live_tum.txt.optimized.txt" "$r/final_tum.txt"
    echo "   $sys: $(wc -l < "$r/live_tum.txt" 2>/dev/null || echo 0) live poses, $(wc -l < "$r/final_tum.txt" 2>/dev/null || echo 0) final  [$(date +%T)]"
  done
done
