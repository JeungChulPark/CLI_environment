#!/bin/bash
# One BOP dataset end to end on the .41 host, one GPU job at a time:
# wait for templates -> build_assets -> smoke (--limit 2) -> L0 -> ours -> evaluate (+BOP AR) -> same-object ADD-S
# usage: run_ds_host.sh lmo|tudl|icbin [GPU]
ds=$1; GPU=${2:-1}
SRC=$HOME/DeepLearning/CLI_env_paper/paper_etrij_261003/src/bop_more
export BOP_DATASET=$ds; source $SRC/env_bop_more_host.sh; mkdir -p $BOP_OUT; cd $SRC
LOG=$BOP_OUT/chain.log; say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
OBJS=$($SAM6D_PY -c "import paths as P; print(' '.join(map(str, P.OIDS)))"); LAST=$(echo $OBJS | awk '{print $NF}')
say "dataset $ds objs: $OBJS gpu $GPU"
until [ -f $BOP_WORK/templates/obj_$(printf %06d $LAST)/templates/xyz_41.npy ] && ! pgrep -f "render_templates_bproc.py.*--models $HOME/Dataset/bop/$ds/" >/dev/null; do sleep 30; done
say "templates ready"
export CUDA_VISIBLE_DEVICES=$GPU
$SAM6D_PY build_assets.py > $BOP_OUT/build_assets.log 2>&1 || { say "FAIL build_assets"; exit 1; }
say "assets built"
OURS="run_ours.py --mode text --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full --verify-cluster-first"
$SAM6D_PY run_orig.py --limit 2 --warmup 1 --out $BOP_OUT/smoke_L0.json > $BOP_OUT/smoke_L0.log 2>&1 || { say "FAIL smoke L0"; exit 1; }
$SAM6D_PY $OURS --limit 2 --warmup 1 --out $BOP_OUT/smoke_hybS05_cf.json > $BOP_OUT/smoke_hybS05_cf.log 2>&1 || { say "FAIL smoke ours"; exit 1; }
say "smoke ok: $(tail -1 $BOP_OUT/smoke_L0.log | cut -c1-100) | $(tail -1 $BOP_OUT/smoke_hybS05_cf.log | cut -c1-100)"
$SAM6D_PY run_orig.py --out $BOP_OUT/pred_ladder_L0.json > $BOP_OUT/L0.log 2>&1 && say "L0 done $(tail -1 $BOP_OUT/L0.log | cut -c1-150)" || say "FAIL L0"
$SAM6D_PY $OURS --out $BOP_OUT/pred_hybS05_cf.json > $BOP_OUT/hybS05_cf.log 2>&1 && say "ours done $(tail -1 $BOP_OUT/hybS05_cf.log | cut -c1-150)" || say "FAIL ours"
ln -sf pred_hybS05_cf.json $BOP_OUT/pred_ladder_hybS05cf.json
export CUDA_VISIBLE_DEVICES=""
$SAM6D_PY evaluate.py --methods ladder_L0,ladder_hybS05cf --results $BOP_OUT/results_${ds}_pro6000.json > $BOP_OUT/evaluate.log 2>&1 && say "evaluate done" || say "FAIL evaluate"
$SAM6D_PY ladder/same_object_adds.py --preds pred_ladder_L0.json,pred_hybS05_cf.json --names L0_원본,개선안3_cf --out $BOP_OUT/same_object_${ds}_pro6000.json > $BOP_OUT/same_object.log 2>&1 && say "same_object done" || say "FAIL same_object"
say "DS_DONE $ds"
