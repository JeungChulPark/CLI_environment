#!/usr/bin/env bash
# run_mapgen_obchange.sh <반복 번호 …> — "같은 인식 결과, 다른 지도 생성" 실험의 ObChange 부분. 반복마다:
#   1) 제안 방법의 인식(LIVE_SIX=8)을 4개 방 방문 20회에 실시간 재생으로 돌린다. 지도 M0(단순 누적) · M1(제안) · M2(합의) · M3(깊이 다듬기)는 실행 중에 만들고, 통과한 물체의 마스크 · 이름을 저장한다.
#   2) 저장한 인식 결과를 FM-Fusion(M4) · DualMap(M5)의 원래 지도 생성 프로그램에 넣는다(ext_maps_obchange.sh).
#   3) 같은 규칙으로 채점한다(score_designs.py, score_base_same.py).
#   결과 objpose/output/mapgen_261012/rep<번호>/obchange/{c8/, designs/, scores.json, scores_ext.json}, 기록 objpose/output/mapgen_261012/log.txt (반복이 끝나면 OB_REP<번호>_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); B=$REPO/objpose/output/mapgen_261012; mkdir -p $B; exec >> $B/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; export PYTHONPATH=$REPO/objpose/tools/shims_nvml
log() { echo "=== $(date '+%F %T') $*"; }
for rep in "$@"; do O=$B/rep$rep/obchange; mkdir -p $O; ALL=""
  for r in smallroom office livingarea kitchen; do sc=""; for n in 2 3 4 5 6; do ALL="$ALL ${r}_s$n"; [ -f $O/designs/c8_${r}_s$n.json ] || sc="$sc ${r}_s$n"; done
    [ -z "$sc" ] && continue
    log "rep$rep 인식 + 지도 M0~M3 $r"; LIVE_SIX=8 timeout 3600 $PY objpose/tools/obchange/six_rt.py --out $O/c8 --scenes $sc --online-map --save-pred > $O/c8_$r.run.log 2>&1; echo "exit=$? $(grep -a -c '^\[six_rt\] c' $O/c8_$r.run.log) 방문"
  done
  log "rep$rep 지도 M4 · M5"; [ -f $O/scores_ext.json ] || PYTHONPATH= bash objpose/tools/mapgen/ext_maps_obchange.sh $O/c8 $ALL 2>&1 | grep -a "FM-Fusion\|objects\|FAILED" | cut -c1-120
  log "rep$rep 채점"
  [ -f $O/scores.json ] || PYTHONPATH= timeout 1800 $DPY objpose/tools/obchange/score_designs.py --root $O > $O/score.log 2>&1
  [ -f $O/scores_ext.json ] || PYTHONPATH= PROPOSED_MAPS=$O/c8/maps/proposed FMF_DIR=$O/c8/ext/fmfusion DUALMAP_MAPS=$O/c8/ext/dualmap/maps/$(cat $O/c8/ext/tag.txt) timeout 1800 $DPY objpose/tools/obchange/score_base_same.py --scenes $ALL --out $O/scores_ext.json > $O/score_ext.log 2>&1
  log "OB_REP${rep}_DONE $(ls $O/designs | wc -l) 지도, 채점 $(ls $O/scores.json $O/scores_ext.json 2>/dev/null | wc -l)/2"
done
