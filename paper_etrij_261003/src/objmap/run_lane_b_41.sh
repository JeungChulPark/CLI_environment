#!/bin/bash
# 2026-10-20 밤 실험, B 줄(GPU 3, REPAIR2 가 끝난 뒤) — 기준 방법.
#   B1. DualMap 두 판(공개 · BGR)을 YCB-V 12 장면 + ObChange Kitchen(방문별 · 연속)에서 Blackwell 로(27 §10 "아직 없는 것").
#       기본 설정(stride 10, 시간 제약 없음). 결과 out_41dm/, 실행 dualmap_runs_41dm/. evaluate.py 네 가지 + Kitchen 채점.
#   B2. 네 방 실시간 30 fps 기준 방법 2회차: FM-Fusion(실시간 검출 → 융합, 패치된 빌드), DualMap 두 판(실시간 훅, 한 번에 하나).
#       FMFUSION_DATA=fmfusion_r2(새 검출), DualMap 입력은 공유(dualmap/), 실행은 dualmap_runs_r2/, 지도는 out30_r2/ (R2 줄의 제안 결과와 같은 곳)
#       → 방마다 obchange_score.py 로 2회차 전체 표. 순서 Office → LivingArea → SmallRoom → BigRoom.
#   머신 전체에서 DualMap 은 이 줄만 돌린다(31 §4.8 메모리 부족).
set -u
source $HOME/Dataset/bop/ycbv_work/obchange_rooms_env_41.sh
LOG=$W/logs/overnight_261020.log; say(){ echo "[$(date +%T)] B $*" | tee -a $LOG; }
W30=$HOME/DeepLearning/Datasets/objmap_obchange30; PY=$SAM6D_PY; cd $S
export CUDA_VISIBLE_DEVICES=3 SAM6D_SHM_SUFFIX=_g3
until grep -q REPAIR2_ALL_DONE $W/logs/obchange_rooms_41.log 2>/dev/null; do sleep 120; done
say "LANE B start (GPU 3)"
# ---- B1 ----
( export OBJMAP_OUT=$S/out_41dm OBJMAP_GT=$S/gt DUALMAP_DATA=$W/dualmap; unset DM_RT_INTERVAL DUALMAP_EXTRA; mkdir -p out_41dm/logs
  for m in dualmap dualmap_bgr; do export DUALMAP_RUNS=$W/dualmap_runs_41dm/$m
    DUALMAP_METHOD=$m DUALMAP_DATASET=ycbv bash dualmap_run.sh $(seq -s ' ' 48 59) > out_41dm/logs/${m}_ycbv.log 2>&1; say "B1 $m ycbv rc=$?"
    DUALMAP_METHOD=$m DUALMAP_DATASET=obchange_kitchen bash dualmap_run.sh 2 3 4 5 6 > out_41dm/logs/${m}_kitchen_v.log 2>&1
    DUALMAP_METHOD=$m DUALMAP_DATASET=obchange_kitchen_all bash dualmap_run.sh 2 3 4 5 6 > out_41dm/logs/${m}_kitchen_a.log 2>&1; say "B1 $m kitchen rc=$?"
    $PY evaluate.py --method $m > out_41dm/logs/eval_${m}_iou.log 2>&1; $PY evaluate.py --method $m --clean > out_41dm/logs/eval_${m}_iou_clean.log 2>&1
    $PY evaluate.py --method $m --match center > out_41dm/logs/eval_${m}_center.log 2>&1; $PY evaluate.py --method $m --match center --clean > out_41dm/logs/eval_${m}_center_clean.log 2>&1
  done
  $PY obchange_score.py --room Kitchen > out_41dm/logs/score_kitchen.log 2>&1; say "B1 DONE (out_41dm)" )
# ---- B2 ----
export OBJMAP_GT=$S/gt30 OBJMAP_OUT=$S/out30_r2 OBJMAP_OBCHANGE_FRAMES=$W30/frames SEQ_BAGS=$W30/bags_r2 \
       FMFUSION_DATA=$W30/fmfusion_r2 DUALMAP_DATA=$W30/dualmap DM_RT_INTERVAL=0.0333333 DUALMAP_EXTRA="stride=1" DUALMAP_TIMEOUT=28800
mkdir -p $FMFUSION_DATA out30_r2; cp -n $W30/fmfusion/likelihood_*.txt $W30/fmfusion/config_obchange_*.yaml $FMFUSION_DATA/ 2>/dev/null; [ -d $FMFUSION_DATA/splits ] || cp -r $W30/fmfusion/splits $FMFUSION_DATA/
for RL in "Office 6" "LivingArea 6" "SmallRoom 6" "BigRoom 7"; do
  set -- $RL; ROOM=$1; LAST=$2; r=$(echo $ROOM | tr A-Z a-z); D=obchange_$r; VIS=$(seq -s ' ' 2 $LAST); L=out30_r2/logs_$r; mkdir -p $L
  say "B2 START $ROOM"
  { $PY fmfusion_prep.py --dataset ${D}_all --scenes $VIS; $PY fmfusion_prep.py --dataset $D --scenes $VIS; } > $L/fmf_prep.log 2>&1
  $FMF fmfusion_detect.py --dataset ${D}_all --scenes $LAST --realtime-fps 30 > $L/fmf_detect.log 2>&1; say "B2 $ROOM fmf_detect rc=$?"
  ( $PY fmfusion_export.py --dataset ${D}_all --scenes $VIS --frame_gap 1 --max_frames 1000000
    $PY fmfusion_export.py --dataset $D --scenes $VIS --frame_gap 1 --max_frames 1000000; echo FMF_DONE ) > $L/fmfusion.log 2>&1 &
  FMFPID=$!
  for m in dualmap dualmap_bgr; do export DUALMAP_RUNS=$W30/dualmap_runs_r2/$m
    DUALMAP_METHOD=$m DUALMAP_DATASET=$D bash dualmap_run.sh $VIS; DUALMAP_METHOD=$m DUALMAP_DATASET=${D}_all bash dualmap_run.sh $VIS
  done > $L/dualmap.log 2>&1
  wait $FMFPID
  $PY obchange_score.py --room $ROOM > $L/score_full.log 2>&1; say "B2 DONE $ROOM score rc=$?"
  grep -E "ctr5cm" $L/score_full.log | sed "s/^/[B2 $ROOM] /" | tee -a $LOG
done
say "LANE_B_DONE"
