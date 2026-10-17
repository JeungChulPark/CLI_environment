#!/usr/bin/env bash
# run_mapgen_ycb.sh <반복 번호 …> — "같은 인식 결과, 다른 지도 생성" 실험의 YCB-Video(12장면, 30 fps) · YCB-M(32장면, 6 fps) 부분. 카메라 자세는 데이터셋의 정답 자세.
#   반복마다 데이터셋마다: 1) 제안 방법의 인식(LIVE_SIX=8, 물체 21종 전부)을 실시간 재생 + 지도 M0~M3 실행 중 생성(rt_bop.py)  2) 저장한 인식 결과로 FM-Fusion · DualMap 지도(ext_maps.sh)  3) 채점(score_ycb.py)
#   결과 objpose/output/mapgen_261012/rep<번호>/{ycbv,ycbm}/{c8/, designs/, scores.json}, 기록 log.txt (YCB_REP<번호>_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); B=$REPO/objpose/output/mapgen_261012; mkdir -p $B; exec >> $B/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; export PYTHONPATH=$REPO/objpose/tools/shims_nvml; D=$HOME/DeepLearning/Datasets; F=objpose/tools/baselines/fmfusion
log() { echo "=== $(date '+%F %T') $*"; }
for rep in "$@"; do for ds in ${DATASETS:-ycbv ycbm}; do O=$B/rep$rep/$ds; mkdir -p $O; TABLE=$REPO/objpose/rt/bop/${ds}_objects.json
  if [ $ds = ycbv ]; then ROOT=$D/paper_benchmarks/YCB_Video; else ROOT=$D/paper_benchmarks/YCB_M/xtion_bop; fi
  ALL=$(ls $ROOT/test | tr '\n' ' '); sc=""; for s in $ALL; do [ -f $O/designs/c8_$s.json ] || sc="$sc $s"; done
  if [ -n "$sc" ]; then log "rep$rep $ds 인식 + 지도 M0~M3"; LIVE_SIX=8 timeout 5400 $PY objpose/tools/mapgen/rt_bop.py --table $TABLE --out $O/c8 --scenes $sc --online-map --save-pred > $O/c8.run.log 2>&1; echo "exit=$? $(grep -a -c '^\[six_rt\] c' $O/c8.run.log) 장면"; fi
  [ -f $O/scores.json ] && continue
  log "rep$rep $ds 지도 M4 · M5"; PYTHONPATH= bash objpose/tools/mapgen/ext_maps.sh $O/c8 $TABLE $D/fmfusion_${ds}_gtpose/scans $F/${ds}_small.yaml $ALL 2>&1 | grep -a "FM-Fusion\|objects\|FAILED" | cut -c1-110
  log "rep$rep $ds 채점"; BENCH_ROOT=$ROOT BENCH_TABLE=$TABLE PYTHONPATH= timeout 3600 $HOME/anaconda3/envs/dualmap/bin/python objpose/tools/mapgen/score_ycb.py --root $O --out $O/scores.json > $O/score.log 2>&1; tail -1 $O/score.log | cut -c1-160
done; log "YCB_REP${rep}_DONE"; done
