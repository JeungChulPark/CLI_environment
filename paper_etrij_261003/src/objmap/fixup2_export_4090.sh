#!/bin/bash
# The ours_obs export needs the BOP depth of every frame; on this PC only scenes 49/53/55 of the full test set
# were kept. Extract the depth of the other 9 scenes (1.8 GB) from ycbv_test_all.zip, then export/evaluate/compare.
set -u
cd "$(dirname "$0")"
source $HOME/Dataset/bop/ycbv_work/env.sh
export OBJMAP_OUT=$PWD/out_4090 SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
PY=$SAM6D_PY; L=$OBJMAP_OUT/logs
say(){ echo "[$(date '+%F %T')] $*" | tee -a $L/chain.log; }
Y=$HOME/DeepLearning/Dataset/bop/ycbv
pat=""; for s in 48 50 51 52 54 56 57 58 59; do pat="$pat test/0000$s/depth/*"; done
say "FIXUP2 unzip depth start"
unzip -q -n $Y/ycbv_test_all.zip $pat -d $Y/full/ > $L/unzip_depth.log 2>&1; say "FIXUP2 unzip rc=$?"
for s in 48 49 50 51 52 53 54 55 56 57 58 59; do echo -n "$s:$(ls $Y/full/test/0000$s/depth | wc -l) "; done | tee -a $L/chain.log; echo
$PY run_ours.py export --scenes 48 49 50 51 52 53 54 55 56 57 58 59 > $L/ours_ycbv_export.log 2>&1; say "FIXUP2 export rc=$?"
for m in ours_cad ours_obs; do
  $PY evaluate.py --method $m                 > $L/eval_${m}_iou.log 2>&1
  $PY evaluate.py --method $m --clean         > $L/eval_${m}_iou_clean.log 2>&1
  $PY evaluate.py --method $m --match center  > $L/eval_${m}_center.log 2>&1
  $PY evaluate.py --method $m --match center --clean > $L/eval_${m}_center_clean.log 2>&1
done
say "FIXUP2 eval done"
$PY compare_4090.py > $OBJMAP_OUT/compare_4090.md 2>&1; say "FIXUP2 compare rc=$?"
say "FIXUP2 ALL DONE"
