#!/bin/bash
# "find every object by the end" study on the YCB-V live replays (12 videos, 55 scene objects):
# the original SAM-6D front end (finds 100 % by the end, but with wrong copies) feeding OUR pose
# verification (keeps wrong entries out), against the improved recogniser g13p (89.1 %).
cd "$(dirname "$0")"; PY=$HOME/anaconda3/envs/sam6d/bin/python
REC="--recognizer $(cd ../realdata && pwd)/live_cores.py"
ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml
until grep -q K50_LIVE_DONE k50_live.log 2>/dev/null; do sleep 20; done
# the D: drive copy run_orig defaults to is not mounted; the official Ultralytics FastSAM-x (as run_real_rest.sh)
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
hyb() {   # tag desc thresh [extra hub args]
  tag=$1; desc=$2; th=$3; extra=${4:-}
  # one scene first: the hybrid core must load and PEM must not fail on every frame
  OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=$desc OBJPOSE_ORIG_THRESH=$th YCBV_ISM=$ISM HUB_EXTRA="$REC $extra" bash run_live.sh $tag 48
  L=../../../objpose/output/ycbv_live_${tag}_000048/sam6d_infer.log
  grep -q "mode=hybrid" $L && [ "$(grep -c 'PEM 건너뜀' $L)" -lt 5 ] || { echo "SMOKE_FAIL $tag"; return 1; }
  OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=$desc OBJPOSE_ORIG_THRESH=$th YCBV_ISM=$ISM HUB_EXTRA="$REC $extra" bash run_live.sh $tag 49 50 51 52 53 54 55 56 57 58 59
}
hyb hybS05 dinov2_vits14 0.5
hyb hybS05_k50 dinov2_vits14 0.5 "--verify-candidates 50"
hyb hybL05 dinov2_vitl14 0.5
R=../../../objpose/output; T="orig orig_th03 orig_th04 orig_th05 orig_scene origS_th04 origS_th05 origS_th06 origS_th07 v2 m5 m15 hz05 hz025 g13p g13p_k50"
for t in hybS05 hybS05_k50 hybL05; do $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out results_$t.json > eval_$t.log 2>&1; T="$T $t"; done
$PY eval_penalized.py --tags $(echo $T | tr ' ' ,) --out results_penalized_findall.json > eval_penalized_findall.log 2>&1
$PY eval_strict_found.py $T > eval_strict_all.log 2>&1
for t in v2 g13p g13p_k50 hybS05 hybS05_k50 hybL05; do $PY eval_missed_live.py $t > missed_$t.log 2>&1; done
echo FINDALL_DONE
