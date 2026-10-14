#!/bin/bash
# re-run of 개선안 2 (origS05): the first real5 run had a SLAM loop misassociation (kf correction 305 cm)
#   origL05 원본 0.5 | g13 개선안 | origS05 개선안 2 | hybS05 개선안 3 | hybS05_nover 개선안 4
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); R=$REPO/objpose/output; SR=$REPO/sam6d_realtime
REC="--recognizer $HERE/live_cores.py"
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
run() {   # tag "extra hub args" env...
  tag=$1; extra=$2; shift 2
  OUT=$R/real5_260901_$tag; rm -rf $OUT; mkdir -p $OUT
  pkill -f "objpose/pc/hub.py"; sleep 3
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $extra > $OUT.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; pkill -f "objpose/pc/hub.py"; echo "done real $tag $(grep -c 'summary written' $OUT.log)"
}
run origS05 "$REC" OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
$PY eval_real.py --runs real5_260901_origL05 real5_260901_g13 real5_260901_origS05 real5_260901_hybS05 real5_260901_hybS05_nover real5_260901_origS05_slambad --out results_real5.json > eval_real5.log 2>&1
echo REDO_DONE
