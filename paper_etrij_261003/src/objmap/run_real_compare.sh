#!/bin/bash
# Object-map comparison on the real recordings (common.REAL_SESSIONS), 2026-10-10, RTX 3080 Ti Laptop.
#  1. ours live (Mac ORB-SLAM3 f2000 + improvement 3 + cluster-first, as realdata/run_real_repeat_3080.sh) on the
#     recordings 2..5; recording 1 (260901_cbnu_bigeightcircle) reuses real5_260901_l3080_hyb3cf_r2b
#  2. real_prep.py: frames + the keyframe-corrected trajectory of that run, for every recording
#  3. FM-Fusion and DualMap (as published / BGR input) on the same frames and trajectory
#  4. real_score.py -> out/real_scores.json
set -u
cd "$(dirname "$0")"; HERE=$PWD
PY=$HOME/anaconda3/envs/sam6d/bin/python; FMF=$HOME/anaconda3/envs/fmfusion/bin/python
REPO=$(cd ../../.. && pwd); RD=$REPO/paper_etrij_261003/src/realdata
L=out/real_logs; mkdir -p $L
SCENES=${SCENES:-"1 2 3 4 5"}
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
live() {   # session outname
  ses=$1; name=$2
  until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
  ssh -o BatchMode=yes mac 'pgrep -q caffeinate || (nohup caffeinate -dimsu -t 14400 >/dev/null 2>&1 &)'
  BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('$ses','orbslam3',2000)))" | sed 's/--out [^ ]*//')
  OUT=$REPO/objpose/output/$name; rm -rf $OUT; mkdir -p $OUT
  for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done; sleep 3
  env OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 \
    $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT --recognizer $RD/live_cores.py --verify-cluster-first > $OUT.log 2>&1 &
  HP=$!
  for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HP 2>/dev/null || break; sleep 3; done
  sleep 3; for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done
  echo "live $name $(date +%T) summary=$(grep -c 'summary written' $OUT.log)"
}
for s in $SCENES; do
  [ $s = 1 ] && continue
  read ses run < <($PY -c "from common import REAL_SESSIONS as R;print(*R[$s])")
  [ -f $REPO/objpose/output/$run/summary.json ] || live $ses $run
done
echo LIVE_DONE
for s in $SCENES; do
  $PY real_prep.py --scene $s || echo "PREP_FAILED $s"
done
$PY fmfusion_prep.py --dataset real --scenes $SCENES > $L/fmf_prep.log 2>&1
$PY dualmap_prep.py --dataset real --scenes $SCENES > $L/dm_prep.log 2>&1
echo PREP_DONE
(
  $FMF fmfusion_detect.py --dataset real --scenes $SCENES --frame-gap 10
  $PY fmfusion_export.py --dataset real --scenes $SCENES
  echo FMF_DONE
) > $L/fmfusion.log 2>&1 &
(
  for m in dualmap dualmap_bgr; do DUALMAP_METHOD=$m DUALMAP_DATASET=real bash dualmap_run.sh $SCENES; done
  echo DM_DONE
) > $L/dualmap.log 2>&1 &
wait
$PY real_score.py --scenes $SCENES > $L/score.log 2>&1
cat $L/score.log
echo REAL_COMPARE_DONE
