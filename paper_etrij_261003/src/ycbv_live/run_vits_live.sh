#!/bin/bash
# live: original SAM-6D decision rule with the ViT-S descriptor (ladder L1) and ISM threshold 0.5 / 0.4
cd "$(dirname "$0")"
REC="--recognizer $PWD/orig_live_core.py"
for th in 0.5 0.4; do
  OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=$th HUB_EXTRA="$REC" bash run_live.sh origS_th${th/./}
done
PY=$HOME/anaconda3/envs/sam6d/bin/python; R=../../../objpose/output
for t in origS_th05 origS_th04; do $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out results_$t.json > eval_$t.log 2>&1; done
$PY eval_penalized.py --tags v2,m5,m15,orig,hz025,hz05,orig_th03,orig_th04,orig_th05,orig_scene,origS_th05,origS_th04 > eval_penalized.log 2>&1
$PY eval_breakdown.py orig orig_th05 origS_th05 origS_th04 v2 m15 > eval_breakdown.log 2>&1
$PY eval_strict_found.py origS_th05 origS_th04 > eval_strict_vits.log 2>&1
echo VITS_DONE
