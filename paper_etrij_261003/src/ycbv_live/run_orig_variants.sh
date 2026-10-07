#!/bin/bash
# supplementary live baselines: original SAM-6D with a higher ISM threshold, and with the scene's objects only
cd "$(dirname "$0")"
REC="--recognizer $PWD/orig_live_core.py"
for th in 0.3 0.4 0.5; do
  tag=orig_th${th/./}
  OBJPOSE_ORIG_THRESH=$th HUB_EXTRA="$REC" bash run_live.sh $tag
done
FULL=$HOME/DeepLearning/Dataset/bop/ycbv/full/test
for s in 48 49 50 51 52 53 54 55 56 57 58 59; do
  ids=$(python3 -c "import json;g=json.load(open('$FULL/$(printf %06d $s)/scene_gt.json'));print(','.join(map(str,sorted({o['obj_id'] for v in g.values() for o in v}))))")
  OBJPOSE_ORIG_OIDS=$ids HUB_EXTRA="$REC" bash run_live.sh orig_scene $s
done
PY=$HOME/anaconda3/envs/sam6d/bin/python; R=../../../objpose/output
for t in orig_th03 orig_th04 orig_th05 orig_scene; do $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out results_$t.json > eval_$t.log 2>&1; done
$PY eval_penalized.py --tags v2,m5,m15,orig,hz025,hz05,orig_th03,orig_th04,orig_th05,orig_scene > eval_penalized.log 2>&1
echo VARIANTS_DONE
