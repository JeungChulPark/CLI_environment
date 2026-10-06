#!/bin/bash
# option A live (FastSAM-x boxes -> our ISM gates + exclusive assignment -> PEM -> verification), 2026-10-06 evening:
# YCB-V live (12 videos) and the five real recordings, scored next to 원본 0.5 / 개선안 / 개선안 3
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); R=$REPO/objpose/output; RD=$(cd ../realdata && pwd)
REC="--recognizer $RD/live_cores.py"
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
# 1) YCB-V live
A="OBJPOSE_LIVE_MODE=gateA YCBV_ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml"
env $A HUB_EXTRA="$REC" bash run_live.sh gateA 48
L=$R/ycbv_live_gateA_000048/sam6d_infer.log
if grep -q "mode=gateA" $L && [ "$(grep -cE '^#[0-9]+ ' $L)" -gt 3 ]; then
  env $A HUB_EXTRA="$REC" bash run_live.sh gateA 49 50 51 52 53 54 55 56 57 58 59
  $PY eval_live.py --runs "$R/ycbv_live_gateA_0000??" --out results_gateA.json > eval_gateA.log 2>&1
  $PY eval_penalized.py --tags orig_th05,g13p,origS_th05,hybS05,hybS05_r2,hybS05_nover,gateA --out results_penalized_combo5.json > eval_penalized_combo5.log 2>&1
  $PY eval_breakdown.py orig orig_th05 origS_th05 origS_th06 origS_th07 v2 m15 g13p g13p_nover hybL05 hybS05 hybS05_nover hybS05_r2 gateA > eval_breakdown.log 2>&1
else echo "SMOKE_FAIL gateA"; fi
echo YCB_DONE
# 2) real recordings
cd $RD
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
run 260901_cbnu_bigeightcircle real5_260901_gateA "$REC" OBJPOSE_LIVE_MODE=gateA
$PY eval_real.py --runs real5_260901_origL05 real5_260901_g13 real5_260901_origS05 real5_260901_hybS05 real5_260901_hybS05_nover real5_260901_gateA --out results_real5.json > eval_real5.log 2>&1
for ses in 260826_etri_eightcircle_dark 260901_cbnu_eightcircle 260910_object 260915_eightcircle; do
  run $ses real_${ses}_gateA "$REC" OBJPOSE_LIVE_MODE=gateA
done
for ses in 260826_etri_eightcircle_dark 260901_cbnu_eightcircle 260910_object; do
  EVAL_REAL_SESSION=$ses $PY eval_real.py --runs real_${ses}_origL05 real_${ses}_g13 real_${ses}_hybS05 real_${ses}_gateA --out results_real_$ses.json > eval_$ses.log 2>&1
done
EVAL_REAL_SESSION=260915 $PY eval_real.py --runs real_260915_origL05 real_260915_g13 real_260915_hybS05 real_260915_eightcircle_gateA --out results_real_260915.json > eval_260915.log 2>&1
echo GATEA_DONE
