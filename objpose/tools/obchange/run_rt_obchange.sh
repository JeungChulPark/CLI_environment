#!/usr/bin/env bash
# run_rt_obchange.sh <방 …> — ObChange 에 여덟 구성을 실시간 재생으로 돌리고, 객체 지도(지금 방식 + 설계 조합 11가지)도 실행 중에 만든다(six_rt.py --online-map).
#   결과 objpose/output/rt_obchange_261011/c<구성>/{<방문>/…, maps/proposed/<방문>.json}, designs/c<구성>_<방문>.json, 기록 log.txt (방 하나가 끝나면 <방>_DONE, 전부 끝나면 RT_OB_DONE). 끝난 것은 건너뛴다.
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/rt_obchange_261011; mkdir -p $O; exec >> $O/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; export PYTHONPATH=$REPO/objpose/tools/shims_nvml
log() { echo "=== $(date '+%F %T') $*"; }
for r in "$@"; do for c in ${CONFIGS:-1 2 3 4 5 6 7 8}; do sc=""; for n in 2 3 4 5 6; do [ -f $O/designs/c${c}_${r}_s$n.json ] || sc="$sc ${r}_s$n"; done
  [ -z "$sc" ] && continue
  log "c$c $r"; LIVE_SIX=$c timeout 3600 $PY objpose/tools/obchange/six_rt.py --out $O/c$c --scenes $sc --online-map > $O/c${c}_$r.run.log 2>&1; echo "exit=$? $(grep -a '지도 갱신' $O/c${c}_$r.run.log | tail -1 | cut -c1-200)"
done; log "${r}_DONE"; done; log "RT_OB_DONE"
