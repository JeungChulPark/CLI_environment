#!/bin/bash
# 2026-10-19: 방 체인의 제안 방법(run_ours.py) 실행 중 자이로 스크립트의 pkill 에 죽은 것들을 다시 돈다 (GPU 1, 포트 8795/17031).
#   Office: all(방문 1~6) · 방문 3 · 5,  BigRoom: all(1~7) · 방문 2~7  — summary.json 이 없는 장면만.
#   GPU 0 체인(자이로·미사용 녹화)이 끝난 뒤(REDO_DONE) 시작해 CPU 경합을 피하고, 각 방의 체인이 끝나면(DONE <방>) 채점을 다시 한다.
set -u
source $HOME/Dataset/bop/ycbv_work/obchange_rooms_env_41.sh
export CUDA_VISIBLE_DEVICES=1 OBJMAP_GT=$S/gt30 OBJMAP_OUT=$S/out30 OBJMAP_OBCHANGE_FRAMES=$W30/frames SEQ_BAGS=$W30/bags \
       FMFUSION_DATA=$W30/fmfusion DUALMAP_DATA=$W30/dualmap SEQ_BAGS_DELETE=1
PY=$SAM6D_PY; PORTS="--http-port 8795 --slam-port 17031"
cd $S
until grep -q REDO_DONE $W/logs/gyro_lan41.log 2>/dev/null; do sleep 300; done
say "REPAIR start (GPU 1)"
missing(){ ds=$1; shift; for s in "$@"; do [ -f out30/ours_runs/${ds}_$(printf %06d $s)/summary.json ] || echo -n "$s "; done; }
for RL in "Office office 6" "LivingArea livingarea 6" "SmallRoom smallroom 6" "BigRoom bigroom 7"; do
  set -- $RL; ROOM=$1; r=$2; LAST=$3; VIS=$(seq -s ' ' 2 $LAST)
  m_all=$(missing obchange_${r}_all $LAST); m_vis=$(missing obchange_$r $VIS)
  [ -z "$m_all$m_vis" ] && { say "REPAIR $ROOM: nothing missing"; continue; }
  say "REPAIR $ROOM: all=[$m_all] visits=[$m_vis]"
  { [ -n "$m_all" ] && $PY run_ours.py run --dataset obchange_${r}_all --scenes $m_all $PORTS && $PY run_ours.py export --dataset obchange_${r}_all --scenes $LAST
    [ -n "$m_vis" ] && $PY run_ours.py run --dataset obchange_$r --scenes $m_vis $PORTS && $PY run_ours.py export --dataset obchange_$r --scenes $VIS
  } > out30/logs_$r/ours_repair.log 2>&1
  say "REPAIR $ROOM ours done: still missing all=[$(missing obchange_${r}_all $LAST)] visits=[$(missing obchange_$r $VIS)]"
done
for RL in "Office office" "LivingArea livingarea" "SmallRoom smallroom" "BigRoom bigroom"; do
  set -- $RL; ROOM=$1; r=$2
  until grep -qE "DONE $ROOM" $LOG 2>/dev/null; do sleep 300; done
  $PY obchange_score.py --room $ROOM > out30/logs_$r/score_repair.log 2>&1; say "RESCORE $ROOM rc=$?"; tail -25 out30/logs_$r/score_repair.log | tee -a $LOG
done
say "REPAIR_ALL_DONE"
