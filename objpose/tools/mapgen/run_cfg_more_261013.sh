#!/usr/bin/env bash
# run_cfg_more_261013.sh — 2026-10-13: 화면 전체 분할 구성 두 가지(①, LIVE_SIX=2 · ②, LIVE_SIX=9)의 추가 실행. 모두 실시간 재생, 지도는 실행 중 생성.
#   1) YCB-Video 3회씩 → cfg_ycbv/c<구성>/rep<번호>/{c<구성>/, designs/, scores.json}   (구성을 번갈아 돌린다)
#   2) ObChange 네 방 3회째 → cfg/c<구성>/rep3/
#   기록 objpose/output/mapgen_261012/log.txt (MORE 로 시작하는 줄), 끝나면 MORE_ALL_DONE
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); B=$REPO/objpose/output/mapgen_261012; L=$B/log.txt
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; SH=$REPO/objpose/tools/shims_nvml; D=$HOME/DeepLearning/Datasets
log() { echo "=== $(date '+%F %T') $*" >> $L; }
ROOT=$D/paper_benchmarks/YCB_Video; TABLE=$REPO/objpose/rt/bop/ycbv_objects.json
for rep in 1 2 3; do for c in 2 9; do O=$B/cfg_ycbv/c$c/rep$rep; [ -f $O/scores.json ] && continue; mkdir -p $O
  sc=""; for s in $(ls $ROOT/test); do [ -f $O/designs/c${c}_$s.json ] || sc="$sc $s"; done
  [ -n "$sc" ] && PYTHONPATH=$SH LIVE_SIX=$c timeout 2400 $PY objpose/tools/mapgen/rt_bop.py --table $TABLE --out $O/c$c --scenes $sc --online-map > $O/c$c.run.log 2>&1
  BENCH_ROOT=$ROOT BENCH_TABLE=$TABLE timeout 1800 $DPY objpose/tools/mapgen/score_ycb.py --root $O --out $O/scores.json > $O/score.log 2>&1
  log "MORE c$c ycbv rep$rep 지도 $(ls $O/designs 2>/dev/null | wc -l), 채점 $(ls $O/scores.json 2>/dev/null | wc -l)"
done; done
log "MORE_YCBV_DONE"
for c in 2 9; do O=$B/cfg/c$c/rep3; mkdir -p $O
  for r in smallroom office livingarea kitchen; do sc=""; for n in 2 3 4 5 6; do [ -f $O/designs/c${c}_${r}_s$n.json ] || sc="$sc ${r}_s$n"; done
    [ -z "$sc" ] && continue
    PYTHONPATH=$SH LIVE_SIX=$c timeout 3600 $PY objpose/tools/obchange/six_rt.py --out $O/c$c --scenes $sc --online-map > $O/c${c}_$r.run.log 2>&1
  done
  rm -f $O/scores.json; timeout 1800 $DPY objpose/tools/obchange/score_designs.py --root $O > $O/score.log 2>&1
  log "MORE c$c obchange rep3 지도 $(ls $O/designs 2>/dev/null | wc -l), 채점 $(ls $O/scores.json 2>/dev/null | wc -l)"
done
log "MORE_ALL_DONE"
