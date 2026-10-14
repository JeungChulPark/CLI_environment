#!/bin/bash
# real-data live comparison on 260901_cbnu_bigeightcircle (Mac ORB-SLAM3 f2000 + this PC), after the YCB chain
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); SR=$REPO/sam6d_realtime
until grep -q COMBO_DONE ../ycbv_live/combo.log; do sleep 30; done
$PY smoke.py orig dinov2_vitl14 > smoke_orig.log 2>&1; $PY smoke.py hybrid dinov2_vits14 > smoke_hyb.log 2>&1
grep -h "^OK" smoke_orig.log smoke_hyb.log; grep -q "^OK" smoke_orig.log && grep -q "^OK" smoke_hyb.log || { echo SMOKE_FAIL; exit 1; }
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
run() {   # tag, extra hub args, env assignments...
  tag=$1; extra=$2; shift 2
  OUT=$REPO/objpose/output/real_260901_$tag; mkdir -p $OUT
  pkill -f "objpose/pc/hub.py"; sleep 3
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $extra > $OUT.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; pkill -f "objpose/pc/hub.py"; echo "done $tag"
}
REC="--recognizer $HERE/live_cores.py"
run ours ""
run g13 "--ism-config $SR/configs/yolo_ism_objects_g13.yaml"
run origL05 "$REC" OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5
run hybL05 "$REC" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5
run hybS05 "$REC" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
run hybS06 "$REC" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.6
echo REAL_DONE
