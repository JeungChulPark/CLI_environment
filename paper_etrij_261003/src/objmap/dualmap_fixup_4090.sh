#!/bin/bash
# 2026-10-12: re-run the DualMap scenes that failed in chain_ours_4090.sh before the empty-YOLO-frame fix
# (baselines/DualMap/utils/object_detector.py, doc 26 §7), then re-score. Run after the GPU0 sequence is idle.
set -u; cd "$(dirname "$0")"; source $HOME/Dataset/bop/ycbv_work/env.sh
export CUDA_VISIBLE_DEVICES=0 OBJMAP_OUT=$PWD/out_4090; PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}; DMO=$HOME/DeepLearning/baselines/DualMap/output; L=$OBJMAP_OUT/logs
say(){ echo "[$(date '+%F %T')] $*" | tee -a $L/chain.log; }
say "START dualmap fixup"
for m in dualmap dualmap_bgr; do
  miss=""; for s in 48 49 50 51 52 53 54 55 56 57 58 59; do [ -f $OBJMAP_OUT/$m/ycbv/0000$s/map.json ] || miss="$miss $s"; done
  [ -n "$miss" ] && { say "$m ycbv missing:$miss"; DUALMAP_METHOD=$m DUALMAP_RUNS=$DMO/ycbv_runs_${m}_4090 bash dualmap_run.sh $miss >> $L/dualmap_${m}_ycbv.log 2>&1; }
  for ds in obchange_kitchen:obkv obchange_kitchen_all:obka; do d=${ds%%:*}; pf=${ds##*:}; miss=""; for s in 2 3 4 5 6; do [ -f $OBJMAP_OUT/$m/$d/00000$s/map.json ] || miss="$miss $s"; done
    [ -n "$miss" ] && { say "$m $d missing:$miss"; DUALMAP_METHOD=$m DUALMAP_DATASET=$d DUALMAP_RUNS=$DMO/${pf}_runs_${m}_4090 bash dualmap_run.sh $miss >> $L/dualmap_${m}_${pf}.log 2>&1; }; done
  $PY evaluate.py --method $m > $L/eval_${m}_iou.log 2>&1; $PY evaluate.py --method $m --clean > $L/eval_${m}_iou_clean.log 2>&1
  $PY evaluate.py --method $m --match center > $L/eval_${m}_center.log 2>&1; $PY evaluate.py --method $m --match center --clean > $L/eval_${m}_center_clean.log 2>&1
done
$PY obchange_score.py --methods ours_cad ours_obs fmfusion dualmap dualmap_bgr > $L/obchange_score_all.log 2>&1; say "DUALMAP FIXUP DONE"
