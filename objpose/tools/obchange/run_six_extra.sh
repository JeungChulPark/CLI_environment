#!/usr/bin/env bash
# run_six_extra.sh — 구성 7(제안 방법, 완화 설정) · 8(제안 방법, 배포 설정)을 추가로 돌린다. run_six_obchange.sh 가 끝난 뒤(ALL_DONE) ObChange 네 방(SmallRoom 먼저),
# 이어서 직접 수집한 영상 8개(실제 환경, 노트북 연결 필요)를 돌린다. 기록은 각 실험의 log.txt (ObChange: EXTRA_OB_DONE, 수집 영상: LIVE78_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/six_obchange_261011; PY=$HOME/anaconda3/envs/sam6d/bin/python; OB=$HOME/DeepLearning/Datasets/ObChange
until grep -aq ALL_DONE $O/log.txt 2>/dev/null; do sleep 20; done
exec >> $O/log.txt 2>&1; export PYTHONPATH=$REPO/objpose/tools/shims_nvml
log() { echo "=== $(date '+%F %T') $*"; }
one() { c=$1; r=$2; sc=""; for n in 2 3 4 5 6; do sc="$sc ${r}_s$n"; done
  [ -f $O/c$c/maps/proposed/${r}_s6.json ] && return
  log "c$c $r"; LIVE_SIX=$c timeout 3600 $PY objpose/tools/obchange/six_rt.py --out $O/c$c --scenes $sc > $O/c${c}_$r.run.log 2>&1; echo "exit=$? $(grep -a -c '^\[six_rt\] c' $O/c${c}_$r.run.log) 방문"
  BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 timeout 1800 $PY objpose/tools/obchange/proposed_maps.py --run $O/c$c --out $O/c$c --tag proposed --room $r --no-online > $O/c${c}_$r.maps.log 2>&1; echo "지도 $(ls $O/c$c/maps/proposed 2>/dev/null | grep -c "^${r}_")개"; }
for r in smallroom office livingarea kitchen; do for c in 7 8; do one $c $r; done; done; log "EXTRA_OB_DONE"
unset PYTHONPATH
if timeout 10 ssh -o BatchMode=yes -o ConnectTimeout=6 samsung true 2>/dev/null; then CONFIGS="7 8" DONE_MARK=LIVE78_DONE bash objpose/tools/lab/run_live_six.sh; else echo "=== $(date '+%F %T') 노트북 연결 안 됨: 직접 수집한 영상은 돌리지 못함 (LIVE78_SKIPPED)" >> $REPO/objpose/output/live_six_261010/log.txt; fi
