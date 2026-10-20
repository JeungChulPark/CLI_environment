#!/bin/bash
# 2026-10-20 밤 실험, R 줄 — ObChange 네 방 실시간 30 fps 의 제안 방법(연속 + 방문별)을 한 번 더(2 · 3회차).
#   정답 궤적을 흘려 넣으므로 SLAM(.192)은 필요 없음. 결과는 out30_r<REP>/ (gt30 · 프레임은 1회차와 공유), bag 은 bags_r<REP>/ 에
#   실행마다 쓰고 지움(SEQ_BAGS_DELETE). 끝난 장면(summary.json)은 건너뜀. 마지막에 방마다 obchange_score.py(기준 방법 지도가 없으면 0).
#   사용: REP=2 GPU=1 HTTP=8795 SLAMP=17031 SHM=_g1 WAIT_REPAIR2=0 bash run_obchange_ours_rep_41.sh   (taskset 으로 코어 고정해서 띄움)
set -u
: ${REP:?} ${GPU:?} ${HTTP:?} ${SLAMP:?} ${SHM:?} ${WAIT_REPAIR2:=0}
source $HOME/Dataset/bop/ycbv_work/obchange_rooms_env_41.sh
LOG=$W/logs/overnight_261020.log; say(){ echo "[$(date +%T)] R$REP $*" | tee -a $LOG; }
W30=$HOME/DeepLearning/Datasets/objmap_obchange30
export CUDA_VISIBLE_DEVICES=$GPU SAM6D_SHM_SUFFIX=$SHM OBJMAP_GT=$S/gt30 OBJMAP_OUT=$S/out30_r$REP OBJMAP_OBCHANGE_FRAMES=$W30/frames \
       SEQ_BAGS=$W30/bags_r$REP SEQ_BAGS_DELETE=1 FMFUSION_DATA=$W30/fmfusion DUALMAP_DATA=$W30/dualmap
PY=$SAM6D_PY; cd $S; mkdir -p out30_r$REP
if [ "$WAIT_REPAIR2" = 1 ]; then until grep -q REPAIR2_ALL_DONE $W/logs/obchange_rooms_41.log 2>/dev/null; do sleep 120; done; fi
say "LANE R$REP start (GPU $GPU, ports $HTTP/$SLAMP, shm $SHM, out30_r$REP)"
run_scene(){ ds=$1; s=$2; L=$3
  if [ -f out30_r$REP/ours_runs/${ds}_$(printf %06d $s)/summary.json ]; then echo "skip ${ds}_$s (summary exists)" >> $L/ours.log; return; fi
  $PY run_ours.py run --dataset $ds --scenes $s --http-port $HTTP --slam-port $SLAMP >> $L/ours.log 2>&1; }
for RL in "Office 6" "LivingArea 6" "SmallRoom 6" "BigRoom 7"; do
  set -- $RL; ROOM=$1; LAST=$2; r=$(echo $ROOM | tr A-Z a-z); D=obchange_$r; VIS=$(seq -s ' ' 2 $LAST); L=out30_r$REP/logs_$r; mkdir -p $L
  say "START $ROOM"
  run_scene ${D}_all $LAST $L; $PY run_ours.py export --dataset ${D}_all --scenes $LAST >> $L/ours.log 2>&1
  for s in $VIS; do run_scene $D $s $L; done; $PY run_ours.py export --dataset $D --scenes $VIS >> $L/ours.log 2>&1
  $PY obchange_score.py --room $ROOM > $L/score.log 2>&1; say "DONE $ROOM score rc=$?"
  grep -E "^ours_(obs|cad) +(per-visit|continuous) +ctr5cm" $L/score.log | sed "s/^/[R$REP $ROOM] /" | tee -a $LOG
done
say "LANE_R${REP}_DONE"
