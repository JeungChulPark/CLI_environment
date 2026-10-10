#!/bin/bash
# Three-run repeats on 260901_cbnu_bigeightcircle on this laptop (RTX 3080 Ti Laptop + Mac ORB-SLAM3 f2000),
# same configurations as the 4090 repeats (doc 15 §4-3, results_real5_repeats_4090.json):
#   original 0.5                    real5_260901_l3080_origL05_r<k>
#   improvement 3 + cluster-first   real5_260901_l3080_hyb3cf_r<k>   (consensus = real-data default 0.5)
# Runs alternate between the two configurations. Waits until the ObChange baselines have freed the GPU.
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd)
WAIT=${WAIT_FOR:-$REPO/paper_etrij_261003/src/objmap/out/obchange_logs/chain_baselines.log}
until grep -q BASELINES_DONE "$WAIT" 2>/dev/null; do sleep 60; done
until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
ssh -o BatchMode=yes mac 'pgrep -q caffeinate || (nohup caffeinate -dimsu -t 10800 >/dev/null 2>&1 &)'
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
REC="--recognizer $HERE/live_cores.py"
run() {   # tag, extra hub args, env assignments...
  tag=$1; extra=$2; shift 2
  OUT=$REPO/objpose/output/real5_260901_$tag; rm -rf $OUT; mkdir -p $OUT
  for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done; sleep 3
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $REC $extra > $OUT.log 2>&1 &
  for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done
  echo "done $tag $(date +%T) summary=$(grep -c 'summary written' $OUT.log)"
}
TAGS=""
for k in 1 2 3; do
  run l3080_origL05_r$k "" OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5
  run l3080_hyb3cf_r$k "--verify-cluster-first" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
  TAGS="$TAGS real5_260901_l3080_origL05_r$k real5_260901_l3080_hyb3cf_r$k"
done
$PY eval_real.py --runs $TAGS --out results_real5_repeats_3080.json > eval_real5_repeats_3080.log 2>&1
echo REAL_REPEAT_3080_DONE
