#!/usr/bin/env bash
# run_posecmp_261014.sh — 인식 구성 네 가지(① LIVE_SIX=2 전체 분할+총점, ② 9 전체 분할+단계별 기준, ⑤ 6 문구 상자+총점, ③ 8 제안)의 6D 포즈 정확도.
#   YCB-Video BOP19 시험 프레임 900장, YCB-M 925장(estcmp_260927 과 같은 프레임)을 정지 영상으로 처리 → objpose/output/posecmp_261014/<데이터셋>/c<구성>/
#   기록 objpose/output/posecmp_261014/log.txt, 끝나면 POSECMP_DONE. 채점 score_still.py → scores_<데이터셋>.json
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/posecmp_261014; L=$O/log.txt
PY=$HOME/anaconda3/envs/sam6d/bin/python; export PYTHONPATH=$REPO/objpose/tools/shims_nvml
log() { echo "=== $(date '+%F %T') $*" >> $L; }
for ds in ycbv ycbm; do for c in 8 6 2 9; do
  log "start $ds c$c"; LIVE_SIX=$c timeout 7200 $PY objpose/tools/mapgen/still_bop.py --table objpose/rt/bop/${ds}_objects.json --frames $O/frames_$ds.json --out $O/$ds/c$c >> $O/$ds.c$c.log 2>&1
  log "end $ds c$c exit=$? scenes=$(ls $O/$ds/c$c/*/run.json 2>/dev/null | wc -l)"
done
  $PY objpose/tools/mapgen/score_still.py $ds ①=$O/$ds/c2 ②=$O/$ds/c9 ⑤=$O/$ds/c6 ③=$O/$ds/c8 --out $O/scores_$ds.json >> $L 2>&1
done
log "POSECMP_DONE"
