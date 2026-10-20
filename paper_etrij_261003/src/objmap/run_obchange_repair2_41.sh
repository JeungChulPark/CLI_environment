#!/bin/bash
# 2026-10-20 05:xx: BigRoom · SmallRoom 의 깨진 baseline 결과를 하나씩 다시 만든다 (GPU 3, 다른 체인이 끝난 뒤 혼자).
#   FM-Fusion: .41 의 IntegrateInstanceMap 에 프레임 상한(20,000 → 1,000,000)·볼록껍질 예외 패치가 없어 연속 지도(BigRoom 98k,
#              SmallRoom 50k 프레임)가 출력 없이 끝났고 BigRoom 방문 4~7 은 죽었다 → .192 의 패치 소스로 다시 빌드(05:10)한 뒤 융합만 다시.
#   DualMap:   연속 지도(방문 1~6, 1~7)가 SmallRoom 체인과 동시에 돌다 메모리 부족(SIGKILL)으로 죽음 → 한 번에 하나씩 다시.
#   끝나면 두 방을 다시 채점한다. 결과 폴더는 체인과 같은 out30/.
set -u
source $HOME/Dataset/bop/ycbv_work/obchange_rooms_env_41.sh
W30=$HOME/DeepLearning/Datasets/objmap_obchange30
export CUDA_VISIBLE_DEVICES=3 SAM6D_SHM_SUFFIX=_g3 OBJMAP_GT=$S/gt30 OBJMAP_OUT=$S/out30 OBJMAP_OBCHANGE_FRAMES=$W30/frames \
       SEQ_BAGS=$W30/bags FMFUSION_DATA=$W30/fmfusion DUALMAP_DATA=$W30/dualmap DM_RT_INTERVAL=0.0333333 DUALMAP_EXTRA="stride=1" DUALMAP_TIMEOUT=28800
PY=$SAM6D_PY
cd $S
until grep -q ROOMS_GPU2_DONE $LOG 2>/dev/null; do sleep 120; done
say "REPAIR2 start (GPU 3, alone)"
missing_dm(){ m=$1; ds=$2; shift 2; for s in "$@"; do [ -f out30/$m/${ds}/$(printf %06d $s)/map.json ] || echo -n "$s "; done; }
missing_fmf(){ ds=$1; shift; for s in "$@"; do [ -f out30/fmfusion/${ds}/$(printf %06d $s)/map.json ] || echo -n "$s "; done; }
for RL in "BigRoom bigroom 7" "SmallRoom smallroom 6"; do
  set -- $RL; ROOM=$1; r=$2; LAST=$3; VIS=$(seq -s ' ' 2 $LAST); D=obchange_$r; L=out30/logs_$r
  # FM-Fusion (CPU): continuous + visits that have no map
  fa=$(missing_fmf ${D}_all $VIS); fv=$(missing_fmf $D $VIS)
  say "REPAIR2 $ROOM FM-Fusion missing all=[$fa] visits=[$fv]"
  [ -n "$fa" ] && $PY fmfusion_export.py --dataset ${D}_all --scenes $fa --frame_gap 1 --max_frames 1000000 > $L/fmfusion_repair.log 2>&1
  [ -n "$fv" ] && $PY fmfusion_export.py --dataset $D --scenes $fv --frame_gap 1 --max_frames 1000000 >> $L/fmfusion_repair.log 2>&1
  say "REPAIR2 $ROOM FM-Fusion done, still missing all=[$(missing_fmf ${D}_all $VIS)] visits=[$(missing_fmf $D $VIS)]"
  # DualMap, one scene at a time
  for m in dualmap dualmap_bgr; do
    export DUALMAP_RUNS=$W30/dualmap_runs/$m
    da=$(missing_dm $m ${D}_all $VIS); dv=$(missing_dm $m $D $VIS)
    say "REPAIR2 $ROOM $m missing all=[$da] visits=[$dv]"
    [ -n "$dv" ] && DUALMAP_METHOD=$m DUALMAP_DATASET=$D bash dualmap_run.sh $dv >> $L/dualmap_repair.log 2>&1
    [ -n "$da" ] && DUALMAP_METHOD=$m DUALMAP_DATASET=${D}_all bash dualmap_run.sh $da >> $L/dualmap_repair.log 2>&1
    say "REPAIR2 $ROOM $m done, still missing all=[$(missing_dm $m ${D}_all $VIS)] visits=[$(missing_dm $m $D $VIS)]"
  done
  $PY obchange_score.py --room $ROOM > $L/score_repair.log 2>&1; say "RESCORE $ROOM rc=$?"; grep -E "ctr5cm" $L/score_repair.log | tee -a $LOG
done
say "REPAIR2_ALL_DONE"
