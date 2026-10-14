#!/bin/bash
# Three-run repeats on this laptop (RTX 3080 Ti Laptop), same protocol as doc 15 §4-4 on the 4090:
#   original 0.5 (FastSAM-x > ViT-L > 0.5 > PEM geometry-best)          tag l3080_orig05_r<k>
#   improvement 3 + cluster-first (FastSAM-x > ViT-S > 0.5 > our PEM,
#     pose verification on, cluster-first with fallback)                 tag l3080_hyb3cf_r<k>
#   bash run_repeat_3080.sh [runs...]     default: 1 2 3
set -u
cd "$(dirname "$0")"; PY=$HOME/anaconda3/envs/sam6d/bin/python
R=../../../objpose/output; OUTJ=results_3080; mkdir -p $OUTJ
ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
ORIG="--recognizer $PWD/orig_live_core.py"
HYB="--recognizer $(cd ../realdata && pwd)/live_cores.py --verify-cluster-first"
for k in ${*:-1 2 3}; do
  t=l3080_orig05_r$k
  OBJPOSE_ORIG_THRESH=0.5 HUB_EXTRA="$ORIG" bash run_live.sh $t
  $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out $OUTJ/results_$t.json > $OUTJ/eval_$t.log 2>&1
  t=l3080_hyb3cf_r$k
  env OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 YCBV_ISM=$ISM \
      HUB_EXTRA="$HYB" bash run_live.sh $t
  $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out $OUTJ/results_$t.json > $OUTJ/eval_$t.log 2>&1
  echo "RUN_DONE $k"
done
TAGS=$(ls $OUTJ/results_l3080_*.json | sed 's|.*/results_||;s|\.json||' | paste -sd,)
$PY eval_penalized.py --tags $TAGS --out $OUTJ/results_penalized_3080.json > $OUTJ/eval_penalized_3080.log 2>&1
echo REPEAT_3080_DONE
