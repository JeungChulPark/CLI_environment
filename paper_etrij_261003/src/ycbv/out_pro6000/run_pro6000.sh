#!/bin/bash
# YCB-V 900 single-image: original SAM-6D (run_orig.py) and ours (개선안 3 + 군집 먼저) on .41 GPU2 (RTX PRO 6000 Blackwell), 2026-10-08
set -u
W=$HOME/Dataset/bop/ycbv_work; source $W/env.sh
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$YCBV_OUT/chain_pro6000.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
cd $OBJPOSE_REPO/paper_etrij_261003/src/ycbv
say "start orig (run_orig.py, 900)"
$PY run_orig.py --out $YCBV_OUT/pred_sam6d.json > $YCBV_OUT/orig_900.log 2>&1 && say "done orig rc=0" || say "FAIL orig rc=$?"
say "start ours (hybS05_cf, 900)"
$PY run_ours.py --mode text --warmup 3 --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full --verify-cluster-first --out $YCBV_OUT/pred_hybS05_cf_900.json > $YCBV_OUT/hybS05_cf_900.log 2>&1 && say "done ours rc=0" || say "FAIL ours rc=$?"
ln -sf pred_hybS05_cf_900.json $YCBV_OUT/pred_ladder_hybS05cf.json
say "evaluate"
CUDA_VISIBLE_DEVICES="" $PY evaluate.py --skip_bop --methods sam6d,ladder_hybS05cf   # (pred_ladder_hybS05cf.json -> pred_hybS05_cf_900.json; evaluate.py only takes ours_/ladder_ keys) --results $YCBV_OUT/results_pro6000.json > $YCBV_OUT/evaluate_pro6000.log 2>&1 && say "eval ok" || say "FAIL eval rc=$?"
say "ALL DONE"
