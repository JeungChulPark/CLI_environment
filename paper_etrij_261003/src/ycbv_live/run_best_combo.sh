#!/bin/bash
# best-combination study: 36-image checks, then live replays
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
cd ycbv
export YCBV_SUBSET=25 YCBV_OUT=$PWD/out/quick
# 1) ViT-S + higher thresholds
$PY ladder/run_ladder.py --level L1 --thresh 0.6 --warmup 2 || echo "FAIL S06"
$PY ladder/run_ladder.py --level L1 --thresh 0.7 --warmup 2 || echo "FAIL S07"
# 2) original ViT-L 0.5 without / with our pose verification
$PY ladder/run_ladder.py --level L0 --thresh 0.5 --warmup 2 || echo "FAIL L0_05"
$PY run_ours.py --mode text --warmup 2 --orig-ism 0.5 --out $YCBV_OUT/pred_combo_L05_verify.json || echo "FAIL L05v"
$PY run_ours.py --mode text --warmup 2 --orig-ism 0.5 --orig-desc dinov2_vits14 --out $YCBV_OUT/pred_combo_S05_verify.json || echo "FAIL S05v"
echo QUICK_COMBO_DONE
unset YCBV_SUBSET YCBV_OUT
cd ../ycbv_live
REC="--recognizer $PWD/orig_live_core.py"
for th in 0.6 0.7; do OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=$th HUB_EXTRA="$REC" bash run_live.sh origS_th${th/./}; done
YCBV_ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects_g13p.yaml bash run_live.sh g13p
R=../../../objpose/output
for t in origS_th06 origS_th07 g13p; do $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out results_$t.json > eval_$t.log 2>&1; done
$PY eval_penalized.py --tags v2,m5,m15,orig,hz025,hz05,orig_th03,orig_th04,orig_th05,orig_scene,origS_th04,origS_th05,origS_th06,origS_th07,g13p > eval_penalized.log 2>&1
$PY eval_breakdown.py orig orig_th05 origS_th05 origS_th06 origS_th07 v2 m15 g13p > eval_breakdown.log 2>&1
$PY eval_strict_found.py orig orig_th03 orig_th04 orig_th05 orig_scene origS_th04 origS_th05 origS_th06 origS_th07 v2 m5 m15 hz05 hz025 g13p > eval_strict_all.log 2>&1
echo COMBO_DONE
