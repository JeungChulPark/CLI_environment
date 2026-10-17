#!/usr/bin/env bash
# run_mapgen_rest.sh — 2026-10-11 축소안(5시간 안): ObChange 3회 반복(run_mapgen_obchange.sh)이 끝나면
#   1) 총점 판정(LIVE_SIX=6, 확정 구성 ⑤)을 ObChange SmallRoom 에 2회 더 돌린다 → 제안 관문(③, 위 3회의 M1)과 반복 실행끼리 비교. 결과 objpose/output/mapgen_261012/gate/rep<번호>/{c6/, designs/, scores.json}
#   2) YCB-Video 3회(run_mapgen_ycb.sh, DATASETS=ycbv) → 지도에 저장된 6D 자세의 오차
#   수집 영상과 YCB-M 은 뺐다(run_mapgen_lab.sh 는 쓰지 않음).
cd "$(dirname "$0")/../../.."; REPO=$(pwd); B=$REPO/objpose/output/mapgen_261012; L=$B/log.txt
until grep -q "OB_REP3_DONE" $L 2>/dev/null; do sleep 30; done
for rep in 2 3; do O=$B/gate/rep$rep; mkdir -p $O
  [ -f $O/scores.json ] && continue
  echo "=== $(date '+%F %T') 총점 판정 SmallRoom rep$rep" >> $L
  PYTHONPATH=$REPO/objpose/tools/shims_nvml LIVE_SIX=6 timeout 3600 $HOME/anaconda3/envs/sam6d/bin/python objpose/tools/obchange/six_rt.py --out $O/c6 --scenes smallroom_s2 smallroom_s3 smallroom_s4 smallroom_s5 smallroom_s6 --online-map > $O/c6.run.log 2>&1
  timeout 1800 $HOME/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/score_designs.py --root $O > $O/score.log 2>&1
  echo "=== $(date '+%F %T') GATE_REP${rep}_DONE $(ls $O/designs 2>/dev/null | wc -l) 지도" >> $L
done
DATASETS=ycbv bash objpose/tools/mapgen/run_mapgen_ycb.sh 1 2 3; echo "=== $(date '+%F %T') MAPGEN_ALL_DONE" >> $L
