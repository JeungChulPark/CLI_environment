#!/usr/bin/env bash
# run_night_261012.sh — 2026-10-12 밤 실험(07:00 전에 끝낸다). 모두 실시간 재생, 지도는 실행 중 생성.
#   1) 총점 판정(⑤, LIVE_SIX=6) ObChange: 기존 gate/rep2 · rep3(SmallRoom 만 있음)에 나머지 세 방을 더하고, gate/rep4 는 네 방 모두 → 네 방 3회씩
#   2) 총점 판정(⑤) YCB-Video 3회 → gate_ycbv/rep<번호>/{c6/, designs/, scores.json}
#   3) 원본(①, LIVE_SIX=2) · 화면 전체 분할 + 관문(②, LIVE_SIX=9) ObChange 네 방 2회씩 → cfg/c<구성>/rep<번호>/
#   4) YCB-M 지도 방식 M0~M5 3회(run_mapgen_ycb.sh, DATASETS=ycbm)
#   단계마다 "지금 + 예상 소요"가 마감(DEADLINE, 기본 06:50)을 넘으면 건너뛴다. 기록 objpose/output/mapgen_261012/log.txt (NIGHT_ 로 시작하는 줄), 끝나면 NIGHT_ALL_DONE
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); B=$REPO/objpose/output/mapgen_261012; L=$B/log.txt
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; SH=$REPO/objpose/tools/shims_nvml; D=$HOME/DeepLearning/Datasets
END=$(date -d "today ${DEADLINE:-06:50}" +%s)
log() { echo "=== $(date '+%F %T') $*" >> $L; }
fits() { [ $(( $(date +%s) + $1 * 60 )) -le $END ]; }      # fits <분>

ob() {   # ob <LIVE_SIX> <결과 폴더> <방 …> : 방마다 빠진 방문만 돌리고 다시 채점한다
  local c=$1 O=$2; shift 2; mkdir -p $O; local ran=0
  for r in "$@"; do local sc=""; for n in 2 3 4 5 6; do [ -f $O/designs/c${c}_${r}_s$n.json ] || sc="$sc ${r}_s$n"; done
    [ -z "$sc" ] && continue; ran=1
    PYTHONPATH=$SH LIVE_SIX=$c timeout 3600 $PY objpose/tools/obchange/six_rt.py --out $O/c$c --scenes $sc --online-map > $O/c${c}_$r.run.log 2>&1
  done
  if [ $ran = 1 ] || [ ! -f $O/scores.json ]; then rm -f $O/scores.json; timeout 1800 $DPY objpose/tools/obchange/score_designs.py --root $O > $O/score.log 2>&1; fi
  log "NIGHT c$c $(basename $(dirname $O))/$(basename $O) 지도 $(ls $O/designs 2>/dev/null | wc -l), 채점 $(ls $O/scores.json 2>/dev/null | wc -l)"
}

# 1) ⑤ ObChange
for rep in 2 3; do fits 20 && ob 6 $B/gate/rep$rep smallroom office livingarea kitchen || log "NIGHT 건너뜀 gate rep$rep"; done
fits 45 && ob 6 $B/gate/rep4 smallroom office livingarea kitchen || log "NIGHT 건너뜀 gate rep4"
log "NIGHT_STEP1_DONE"

# 2) ⑤ YCB-Video
ROOT=$D/paper_benchmarks/YCB_Video; TABLE=$REPO/objpose/rt/bop/ycbv_objects.json
for rep in 1 2 3; do O=$B/gate_ycbv/rep$rep; [ -f $O/scores.json ] && continue; fits 16 || { log "NIGHT 건너뜀 gate_ycbv rep$rep"; continue; }; mkdir -p $O
  sc=""; for s in $(ls $ROOT/test); do [ -f $O/designs/c6_$s.json ] || sc="$sc $s"; done
  [ -n "$sc" ] && PYTHONPATH=$SH LIVE_SIX=6 timeout 2400 $PY objpose/tools/mapgen/rt_bop.py --table $TABLE --out $O/c6 --scenes $sc --online-map > $O/c6.run.log 2>&1
  BENCH_ROOT=$ROOT BENCH_TABLE=$TABLE timeout 1800 $DPY objpose/tools/mapgen/score_ycb.py --root $O --out $O/scores.json > $O/score.log 2>&1
  log "NIGHT c6 ycbv rep$rep 지도 $(ls $O/designs 2>/dev/null | wc -l), 채점 $(ls $O/scores.json 2>/dev/null | wc -l)"
done
log "NIGHT_STEP2_DONE"

# 3) ① · ② ObChange 네 방 2회씩 (구성을 번갈아 돌려, 중간에 끊겨도 두 구성의 횟수가 같게)
for rep in 1 2; do for c in 2 9; do fits 45 && ob $c $B/cfg/c$c/rep$rep smallroom office livingarea kitchen || log "NIGHT 건너뜀 c$c rep$rep"; done; done
log "NIGHT_STEP3_DONE"

# 4) YCB-M M0~M5
for rep in 1 2 3; do fits 32 && DATASETS=ycbm bash objpose/tools/mapgen/run_mapgen_ycb.sh $rep || log "NIGHT 건너뜀 ycbm rep$rep"; done
log "NIGHT_ALL_DONE"
