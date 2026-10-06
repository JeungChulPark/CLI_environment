#!/bin/bash
# remaining real-data runs (hybrid import-order fix 2026-10-06; FastSAM-x from ycbv_work because /mnt/d is offline)
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd)
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
until timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=5 mac true 2>/dev/null; do sleep 60; done
echo "mac reachable $(date)"
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
run() {
  tag=$1; shift
  OUT=$REPO/objpose/output/real_260901_$tag; rm -rf $OUT; mkdir -p $OUT
  pkill -f "objpose/pc/hub.py"; sleep 3
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT --recognizer $HERE/live_cores.py > $OUT.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; pkill -f "objpose/pc/hub.py"; echo "done $tag $(grep -c 'summary written' $OUT.log)"
}
#run hybL05 OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5
run hybS05 OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
run hybS06 OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.6
$PY eval_real.py --runs real_260901_ours real_260901_g13 real_260901_origL05 real_260901_hybL05 real_260901_hybS05 real_260901_hybS06 > eval_all.log 2>&1
echo REST_DONE
