#!/bin/bash
# Re-run of improvement 3 + cluster-first, run 2: real5_260901_l3080_hyb3cf_r2 got a 3.10 m wrong loop closure from
# ORB-SLAM3 (others 9-26 cm), so it is discarded as on the 4090 (doc 15 §2) and replaced by ..._hyb3cf_r2b.
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd)
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
OUT=$REPO/objpose/output/real5_260901_l3080_hyb3cf_r2b; rm -rf $OUT; mkdir -p $OUT
OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 \
  $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT --recognizer $HERE/live_cores.py --verify-cluster-first > $OUT.log 2>&1 &
for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
sleep 3; for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done
echo "done hyb3cf_r2b $(date +%T) $(grep -o 'max correction so far [0-9.]* cm' $OUT.log | tail -1)"
$PY eval_real.py --runs real5_260901_l3080_origL05_r1 real5_260901_l3080_hyb3cf_r1 real5_260901_l3080_origL05_r2 \
  real5_260901_l3080_hyb3cf_r2 real5_260901_l3080_hyb3cf_r2b real5_260901_l3080_origL05_r3 real5_260901_l3080_hyb3cf_r3 \
  --out results_real5_repeats_3080.json > eval_real5_repeats_3080.log 2>&1
echo REDO_DONE
