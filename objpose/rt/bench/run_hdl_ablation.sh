#!/bin/bash
# run_hdl_ablation.sh — why is hdl_graph_slam off the LiDAR pseudo-GT? Two checks on the 260901 sessions:
#   floor   hdl with floor detection + floor constraints on (as in run_bench.sh) vs off (--floor false);
#           the floor constraint is the suspect for the 1.2-1.5 deg tilt
#   kf      every run also dumps the optimised graph (--dump-dir -> keyframes_tum.txt), so the keyframe poses alone
#           can be scored apart from the per-scan live poses (live_tum.txt) and the re-anchored ones (final_tum.txt):
#           error in the graph itself vs error from odometry between keyframes
# Per session -> out/<session>/hdl_floor_<on|off>/run<REP>/
#   bash objpose/rt/bench/run_hdl_ablation.sh [session ...]      (default: the four 260901 sessions)
# Env: REP (default 1), FLOORS (default "on off"), PORT (default 17101)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="${DATA:-$HOME/Documents/DefenseMeta/Dataset}"
HDL="$HOME/objpose/lidar/run_hdl_stream.sh"
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
REP="${REP:-1}"; FLOORS="${FLOORS:-on off}"; PORT="${PORT:-17101}"
if [ $# -gt 0 ]; then S=("$@"); else
  S=(260901_cbnu_bigeightcircle 260901_cbnu_eightcircle 260901_cbnu_digut 260901_cbnu_longcircle); fi

for s in "${S[@]}"; do
  for fl in $FLOORS; do
    r="$HERE/out/$s/hdl_floor_$fl/run$REP"; mkdir -p "$r"
    [ "$fl" = on ] && flag=true || flag=false
    echo "== $s floor=$fl  [$(date +%T)]"
    "$PY" "$HERE/hub_record.py" --port "$PORT" --out "$r" > "$r/hub.log" 2>&1 &
    hub=$!; sleep 1
    "$HDL" --session "$DATA/$s/lidar" --mode live --connect "127.0.0.1:$PORT" --rate 1.0 --floor "$flag" \
      --dump-dir "$r/graph" > "$r/run.log" 2>&1
    wait $hub
    [ -f "$r/graph/keyframes_tum.txt" ] && cp "$r/graph/keyframes_tum.txt" "$r/kf_tum.txt"
    echo "   $(wc -l < "$r/live_tum.txt" 2>/dev/null || echo 0) live, $(wc -l < "$r/final_tum.txt" 2>/dev/null || echo 0) final, $(wc -l < "$r/kf_tum.txt" 2>/dev/null || echo 0) keyframes  [$(date +%T)]"
  done
done
