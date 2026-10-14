#!/bin/bash
# Re-run the YCB-V scenes whose live run left no summary.json (scene 57: GT pose stream closed after 13 s,
# cause not found in the logs), then export + evaluate all 12 scenes again. Runs after chain_ours_4090.sh.
set -u
cd "$(dirname "$0")"
source $HOME/Dataset/bop/ycbv_work/env.sh
export CUDA_VISIBLE_DEVICES=0 OBJMAP_OUT=$PWD/out_4090 SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
PY=$SAM6D_PY; L=$OBJMAP_OUT/logs
say(){ echo "[$(date '+%F %T')] $*" | tee -a $L/chain.log; }
# (chain_ours_4090.sh died silently after scene 58 at 08:00:55; this runs on its own as a user unit)
say "FIXUP start"
for try in 1 2 3; do
  miss=""; for s in 48 49 50 51 52 53 54 55 56 57 58 59; do [ -f $OBJMAP_OUT/ours_runs/0000$s/summary.json ] || miss="$miss $s"; done
  [ -z "$miss" ] && break
  say "FIXUP try $try missing:$miss"
  for s in $miss; do mv $OBJMAP_OUT/ours_runs/0000$s $OBJMAP_OUT/ours_runs/0000${s}_fail$try 2>/dev/null; mv $OBJMAP_OUT/ours_runs/0000$s.log $OBJMAP_OUT/ours_runs/0000${s}_fail$try.log 2>/dev/null; done
  $PY run_ours.py run --scenes $miss >> $L/ours_ycbv_fixup.log 2>&1
done
$PY run_ours.py export --scenes 48 49 50 51 52 53 54 55 56 57 58 59 > $L/ours_ycbv_export.log 2>&1; say "FIXUP export rc=$?"
for m in ours_cad ours_obs; do
  $PY evaluate.py --method $m                 > $L/eval_${m}_iou.log 2>&1
  $PY evaluate.py --method $m --clean         > $L/eval_${m}_iou_clean.log 2>&1
  $PY evaluate.py --method $m --match center  > $L/eval_${m}_center.log 2>&1
  $PY evaluate.py --method $m --match center --clean > $L/eval_${m}_center_clean.log 2>&1
done
say "FIXUP eval done"
$PY compare_4090.py > $OBJMAP_OUT/compare_4090.md 2>&1; say "FIXUP compare rc=$?"
say "FIXUP ALL DONE"
