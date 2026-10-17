#!/usr/bin/env bash
# run_six_obchange.sh — ObChange 에 여섯 구성(six_rt.py)을 실시간 재생으로 돌리고, 구성 · 방마다 지금 방식의 객체 지도(proposed_maps.py)를 만든다. SmallRoom 을 먼저 여섯 구성 모두 돌린 뒤 나머지 방으로 간다.
#   결과 objpose/output/six_obchange_261011/c<구성>/{<방문>/…, maps/proposed/<방문>.json}, 기록 log.txt (SmallRoom 이 끝나면 SMALLROOM_DONE, 전부 끝나면 ALL_DONE). 끝난 것은 건너뛴다.
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/six_obchange_261011; mkdir -p $O; exec >> $O/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; OB=$HOME/DeepLearning/Datasets/ObChange; export PYTHONPATH=$REPO/objpose/tools/shims_nvml
log() { echo "=== $(date '+%F %T') $*"; }
one() { c=$1; r=$2; sc=""; for n in 2 3 4 5 6; do sc="$sc ${r}_s$n"; done
  [ -f $O/c$c/maps/proposed/${r}_s6.json ] && return
  log "c$c $r"; LIVE_SIX=$c timeout 3600 $PY objpose/tools/obchange/six_rt.py --out $O/c$c --scenes $sc > $O/c${c}_$r.run.log 2>&1; echo "exit=$? $(grep -a -c '^\[six_rt\] c' $O/c${c}_$r.run.log) 방문"
  BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 timeout 1800 $PY objpose/tools/obchange/proposed_maps.py --run $O/c$c --out $O/c$c --tag proposed --room $r --no-online > $O/c${c}_$r.maps.log 2>&1; echo "지도 $(ls $O/c$c/maps/proposed 2>/dev/null | grep -c "^${r}_")개"; }
for c in 1 2 3 4 5 6; do one $c smallroom; done; log "SMALLROOM_DONE"
for r in office livingarea kitchen; do for c in 1 2 3 4 5 6; do one $c $r; done; done; log "ALL_DONE"
