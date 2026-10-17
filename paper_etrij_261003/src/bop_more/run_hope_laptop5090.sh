#!/bin/bash
# HOPE-Video end to end on the RTX 5090 Laptop (doc 27), the run_ds_host.sh chain with laptop paths:
# templates (CPU) -> build_assets -> smoke -> L0 -> ours -> evaluate (no BOP AR: no toolkit here) -> same-object ADD-S
cd "$(dirname "$0")"; SRC=$PWD
source env_bop_more_laptop5090.sh; mkdir -p $BOP_OUT $BOP_WORK
LOG=$BOP_OUT/chain.log; say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
[ -f $BOP_DS_DIR/test_targets_bop19.json ] || $SAM6D_PY make_hope_targets.py --root $BOP_DS_DIR | tee -a $LOG
OBJS=$($SAM6D_PY -c "import paths as P; print(','.join(map(str, P.OIDS)))"); LAST=${OBJS##*,}
say "dataset $BOP_DATASET objs: $OBJS"
if [ ! -f $BOP_WORK/templates/obj_$(printf %06d $LAST)/templates/xyz_41.npy ]; then
  $BPROC run render_templates_bproc.py --blender-install-path $BLENDER_DIR --models $BOP_DS_DIR/models \
    --out $BOP_WORK/templates --objs $OBJS --threads ${THREADS:-12} > $BOP_OUT/render.log 2>&1 || { say "FAIL render"; exit 1; }
fi
say "templates ready"
$SAM6D_PY build_assets.py > $BOP_OUT/build_assets.log 2>&1 || { say "FAIL build_assets"; exit 1; }
say "assets built"
OURS="run_ours.py --mode text --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full --verify-cluster-first"
$SAM6D_PY run_orig.py --limit 2 --warmup 1 --out $BOP_OUT/smoke_L0.json > $BOP_OUT/smoke_L0.log 2>&1 || { say "FAIL smoke L0"; exit 1; }
$SAM6D_PY $OURS --limit 2 --warmup 1 --out $BOP_OUT/smoke_hybS05_cf.json > $BOP_OUT/smoke_hybS05_cf.log 2>&1 || { say "FAIL smoke ours"; exit 1; }
say "smoke ok"
$SAM6D_PY run_orig.py --out $BOP_OUT/pred_ladder_L0.json > $BOP_OUT/L0.log 2>&1 && say "L0 done $(tail -1 $BOP_OUT/L0.log | cut -c1-150)" || say "FAIL L0"
$SAM6D_PY $OURS --out $BOP_OUT/pred_hybS05_cf.json > $BOP_OUT/hybS05_cf.log 2>&1 && say "ours done $(tail -1 $BOP_OUT/hybS05_cf.log | cut -c1-150)" || say "FAIL ours"
ln -sf pred_hybS05_cf.json $BOP_OUT/pred_ladder_hybS05cf.json
$SAM6D_PY evaluate.py --skip_bop --methods ladder_L0,ladder_hybS05cf --results $BOP_OUT/results_hope_5090.json > $BOP_OUT/evaluate.log 2>&1 && say "evaluate done" || say "FAIL evaluate"
$SAM6D_PY ladder/same_object_adds.py --preds pred_ladder_L0.json,pred_hybS05_cf.json --names L0_원본,개선안3_cf --out $BOP_OUT/same_object_hope_5090.json > $BOP_OUT/same_object.log 2>&1 && say "same_object done" || say "FAIL same_object"
say "DS_DONE $BOP_DATASET"
