#!/bin/bash
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); R=$REPO/objpose/output; SR=$REPO/sam6d_realtime
REC="--recognizer $HERE/live_cores.py"
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
run() {   # session outname "extra hub args" env...
  ses=$1; name=$2; extra=$3; shift 3
  until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
  BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('$ses','orbslam3',2000)))" | sed 's/--out [^ ]*//')
  OUT=$R/$name; rm -rf $OUT; mkdir -p $OUT
  pkill -f "objpose/pc/hub.py"; sleep 3
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $extra > $OUT.log 2>&1 &
  HP=$!
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HP 2>/dev/null || break; sleep 3; done
  sleep 3; pkill -f "objpose/pc/hub.py"; echo "done real $name $(grep -c 'summary written' $OUT.log)"
}
# 260826_etri_eightcircle_dark redo: the first attempt found an empty stray SAM_0.db3 next to the recording (moved aside)
until grep -q ALL_SESSIONS_DONE all_sessions.log 2>/dev/null; do sleep 15; done
ses=260826_etri_eightcircle_dark
run $ses real_${ses}_origL05 "$REC" OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5
run $ses real_${ses}_g13 "--ism-config $SR/configs/yolo_ism_objects_g13.yaml" X=1
run $ses real_${ses}_hybS05 "$REC" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
EVAL_REAL_SESSION=$ses $PY eval_real.py --runs real_${ses}_origL05 real_${ses}_g13 real_${ses}_hybS05 --out results_real_$ses.json > eval_$ses.log 2>&1
echo "SCORED $ses"
echo DARK_DONE
