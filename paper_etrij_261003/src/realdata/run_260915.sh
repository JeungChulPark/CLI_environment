#!/bin/bash
# 원본 0.5 / 개선안 / 개선안 3 on the 128 s recording 260915_eightcircle (Mac ORB-SLAM3 f2000), 2026-10-06
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); R=$REPO/objpose/output; SR=$REPO/sam6d_realtime
REC="--recognizer $HERE/live_cores.py"
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
until grep -q R2_DONE ../ycbv_live/hybS05_r2.log 2>/dev/null; do sleep 20; done      # GPU free first
until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260915_eightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
run() {   # tag "extra hub args" env...
  tag=$1; extra=$2; shift 2
  OUT=$R/real_260915_$tag; rm -rf $OUT; mkdir -p $OUT
  pkill -f "objpose/pc/hub.py"; sleep 3
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $extra > $OUT.log 2>&1 &
  for i in $(seq 1 300); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; pkill -f "objpose/pc/hub.py"; echo "done real $tag $(grep -c 'summary written' $OUT.log)"
}
run origL05 "$REC" OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5
run g13 "--ism-config $SR/configs/yolo_ism_objects_g13.yaml" X=1
run hybS05 "$REC" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
EVAL_REAL_SESSION=260915 $PY eval_real.py --runs real_260915_origL05 real_260915_g13 real_260915_hybS05 live_260915_eightcircle_orbslam3 --out results_real_260915.json > eval_260915.log 2>&1
echo R260915_DONE
