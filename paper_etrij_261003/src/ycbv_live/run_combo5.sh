#!/bin/bash
# technique-combination study (2026-10-06), five recognisers on YCB-V live and the real recording 260901:
#   1 original 0.5 (ViT-L)              orig_th05 / real origL05      existing
#   2 improved, verification on         g13p / real g13               existing
#   3 FastSAM-x > ViT-S > 0.5 > upstream PEM (geometry-best)          origS_th05 existing / real origS05 NEW
#   4 FastSAM-x > ViT-S > 0.5 > our PEM + verification on             hybS05 existing
#   5 FastSAM-x > ViT-S > 0.5 > our PEM, verification off             hybS05_nover NEW (both)
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); R=$REPO/objpose/output
REC="--recognizer $(cd ../realdata && pwd)/live_cores.py"
ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
# 1) YCB-V live, #5
H="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 YCBV_ISM=$ISM"
env $H HUB_EXTRA="$REC --no-verify" bash run_live.sh hybS05_nover 48
L=$R/ycbv_live_hybS05_nover_000048/sam6d_infer.log
if grep -q "mode=hybrid" $L && [ "$(grep -c 'PEM 건너뜀' $L)" -lt 5 ]; then
  env $H HUB_EXTRA="$REC --no-verify" bash run_live.sh hybS05_nover 49 50 51 52 53 54 55 56 57 58 59
  $PY eval_live.py --runs "$R/ycbv_live_hybS05_nover_0000??" --out results_hybS05_nover.json > eval_hybS05_nover.log 2>&1
  $PY eval_penalized.py --tags orig_th05,g13p,origS_th05,hybS05,hybS05_nover --out results_penalized_combo5.json > eval_penalized_combo5.log 2>&1
  $PY eval_breakdown.py orig orig_th05 origS_th05 origS_th06 origS_th07 v2 m15 g13p g13p_nover hybL05 hybS05 hybS05_nover > eval_breakdown.log 2>&1
else echo "SMOKE_FAIL hybS05_nover"; fi
echo LIVE_DONE
# 2) real data: #3 and #5
cd ../realdata
until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
run() {   # tag "extra hub args" env...
  tag=$1; extra=$2; shift 2
  OUT=$R/real_260901_$tag; rm -rf $OUT; mkdir -p $OUT
  pkill -f "objpose/pc/hub.py"; sleep 3
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $REC $extra > $OUT.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; pkill -f "objpose/pc/hub.py"; echo "done real $tag $(grep -c 'summary written' $OUT.log)"
}
run origS05 "" OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
run hybS05_nover "--no-verify" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
$PY eval_real.py --runs real_260901_origL05 real_260901_g13 real_260901_origS05 real_260901_hybS05 real_260901_hybS05_nover real_260901_g13_nover real_260901_hybL05 --out results_real_combo5.json > eval_combo5.log 2>&1
echo COMBO5_DONE
