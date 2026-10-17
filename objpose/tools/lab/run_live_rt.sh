#!/usr/bin/env bash
# run_live_rt.sh — 8개 영상 × 여덟 구성을 실제 환경(노트북 ORB-SLAM3 실시간 + 이 PC 인식)으로 돌리되, 지도 설계 조합 11가지도 실행 중에 계산한다(OBJPOSE_ONLINE_DESIGNS=1, objpose/pc/online_designs.py).
#   결과 objpose/output/live_rt_261011/c<구성>/<세션>/ (hub 실행 폴더 + online_designs.json, online_timeline.jsonl), 설계 지도 designs/c<구성>_<세션>.json, 기록 log.txt (끝나면 LIVE_RT_DONE). 끝난 실행은 건너뛴다.
set -u
cd "$(dirname "$0")/../../.."; O=$(pwd)/objpose/output/live_rt_261011; mkdir -p $O/designs; exec >> $O/log.txt 2>&1; export OBJPOSE_ONLINE_DESIGNS=1
A="260901_cbnu_bigeightcircle 260901_cbnu_digut 260901_cbnu_eightcircle 260901_cbnu_longcircle"; B="260826_etri_differentObject 260826_etri_eightcircle_dark 260826_etri_longcircle_dark 260826_etri_similarObject"
ok() { [ -f $O/c$1/$2/summary.json ] && [ -f $O/c$1/$2/online_designs.json ]; }
for c in ${CONFIGS:-1 2 3 4 5 6 7 8}; do for s in $A $B; do
  ok $c $s || { echo "=== $(date '+%F %T') c$c $s"; bash objpose/tools/lab/run_live_one.sh $s $c $O/c$c/$s; }
  ok $c $s || { echo "재시도 c$c $s"; sleep 5; bash objpose/tools/lab/run_live_one.sh $s $c $O/c$c/$s; }
  ok $c $s && cp $O/c$c/$s/online_designs.json $O/designs/c${c}_$s.json && echo "지도 갱신 $(grep -a -o 'online map designs: [0-9]* passes.*' $O/c$c/$s.log | tail -1 | cut -c1-200)"
done; done
echo "=== $(date "+%F %T") LIVE_RT_DONE"
