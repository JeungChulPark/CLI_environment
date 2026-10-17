#!/usr/bin/env bash
# run_cfg_ycbm_261014.sh — 2026-10-14: 표 1 의 YCB-M 열. ①(LIVE_SIX=2), ②(9), ⑤(6) 을 YCB-M 32장면에 1회씩 실시간 재생, 지도는 실행 중 생성, 채점은 score_ycb.py(③ 의 rep*/ycbm 과 같은 규칙)
#   결과 objpose/output/mapgen_261012/cfg_ycbm/c<구성>/rep1/{c<구성>/, designs/, scores.json}, 기록 log.txt (YCBM_CFG 로 시작), 끝나면 YCBM_CFG_DONE
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); B=$REPO/objpose/output/mapgen_261012; L=$B/log.txt
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; SH=$REPO/objpose/tools/shims_nvml
ROOT=$HOME/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop; TABLE=$REPO/objpose/rt/bop/ycbm_objects.json
log() { echo "=== $(date '+%F %T') $*" >> $L; }
for c in 2 6 9; do O=$B/cfg_ycbm/c$c/rep1; [ -f $O/scores.json ] && continue; mkdir -p $O
  sc=""; for s in $(ls $ROOT/test); do [ -f $O/designs/c${c}_$s.json ] || sc="$sc $s"; done
  log "YCBM_CFG c$c start"
  [ -n "$sc" ] && PYTHONPATH=$SH LIVE_SIX=$c timeout 3600 $PY objpose/tools/mapgen/rt_bop.py --table $TABLE --out $O/c$c --scenes $sc --online-map > $O/c$c.run.log 2>&1
  BENCH_ROOT=$ROOT BENCH_TABLE=$TABLE timeout 1800 $DPY objpose/tools/mapgen/score_ycb.py --root $O --out $O/scores.json > $O/score.log 2>&1
  log "YCBM_CFG c$c 지도 $(ls $O/designs 2>/dev/null | wc -l), 채점 $(ls $O/scores.json 2>/dev/null | wc -l)"
done
log "YCBM_CFG_DONE"
