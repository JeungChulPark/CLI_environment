#!/usr/bin/env bash
# run_mapgen_lab.sh <반복 번호 …> — "같은 인식 결과, 다른 지도 생성" 실험의 직접 수집한 영상 8개 부분(실제 환경: 노트북 ORB-SLAM3 실시간 + 이 PC 인식 LIVE_SIX=8).
#   영상마다: 1) 실행(run_live_one.sh, 지도 M0~M3 실행 중 생성 + 마스크 저장)  2) 그 실행의 인식 결과 · 카메라 자세를 꺼내(lab_ext.py) FM-Fusion · DualMap 지도(ext_maps.sh). 꺼낸 영상은 지도를 만든 뒤 지운다.
#   결과 objpose/output/mapgen_261012/rep<번호>/lab/c8/<세션>/{online_designs.json, ext_in/run/ext/…}, 기록 log.txt (LAB_REP<번호>_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); B=$REPO/objpose/output/mapgen_261012; mkdir -p $B; exec >> $B/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; export OBJPOSE_ONLINE_DESIGNS=1 OBJPOSE_SAVE_MASKS=1
SES="260901_cbnu_bigeightcircle 260901_cbnu_digut 260901_cbnu_eightcircle 260901_cbnu_longcircle 260826_etri_differentObject 260826_etri_eightcircle_dark 260826_etri_longcircle_dark 260826_etri_similarObject"
log() { echo "=== $(date '+%F %T') $*"; }
for rep in "$@"; do O=$B/rep$rep/lab/c8; mkdir -p $O
  for s in $SES; do R=$O/$s
    ok() { [ -f $R/summary.json ] && [ -f $R/online_designs.json ]; }
    ok || { log "rep$rep lab $s"; bash objpose/tools/lab/run_live_one.sh $s 8 $R; }
    ok || { echo "재시도 $s"; sleep 5; bash objpose/tools/lab/run_live_one.sh $s 8 $R; }
    ok || { echo "실패 $s"; continue; }
    [ -f $R/ext_in/run/ext/done ] && continue
    PYTHONPATH=$REPO/objpose/tools/shims_nvml timeout 1800 $PY objpose/tools/mapgen/lab_ext.py $R --session $s --out $R/ext_in 2>&1 | tail -1 | cut -c1-120
    DM_PROMPTS=$REPO/objpose/tools/lab/plain_prompts_lab.json bash objpose/tools/mapgen/ext_maps.sh $R/ext_in/run $R/ext_in/table.json $R/ext_in/scans $R/ext_in/fmf_$s.yaml $s 2>&1 | grep -a "FM-Fusion\|objects\|FAILED" | cut -c1-110
    rm -rf $R/ext_in/bop/test/$s/rgb $R/ext_in/bop/test/$s/depth $R/ext_in/run/ext/dualmap/*_in; touch $R/ext_in/run/ext/done
  done; log "LAB_REP${rep}_DONE"; done
