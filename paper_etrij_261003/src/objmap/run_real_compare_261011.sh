#!/bin/bash
# Doc 24: FM-Fusion and DualMap on the trajectories of the 2026-10-11 live re-runs of ours (recording 1 = _r2),
# in separate work/output dirs so the doc 23 maps (out/, gt/, ycbv_work/fmfusion, ycbv_work/dualmap) stay as
# they are. Same steps and settings as run_real_compare.sh (frame gap 10, published / BGR DualMap).
# Recognition-camera frames (real_work/<session>) depend only on the recording and are shared; FM-Fusion's
# per-frame detections of doc 23 are hard-linked in for frames with the same id (same image, same prompts),
# so only frames new to this trajectory are detected again.
set -u
cd "$(dirname "$0")"
PY=$HOME/anaconda3/envs/sam6d/bin/python; FMF=$HOME/anaconda3/envs/fmfusion/bin/python
W=$HOME/DeepLearning/Dataset/bop/ycbv_work
TAG=${TAG:-261011}   # 2026-10-12: overridable (TAG=4090 with OBJMAP_REAL_RUNS = the 4090 live runs)
[ -n "${OBJMAP_REAL_RUNS:-}" ] || export OBJMAP_REAL_RUNS="1=live_261011_260901_cbnu_bigeightcircle_l3080_hyb3cf_r2,2=live_261011_260826_etri_eightcircle_dark_l3080_hyb3cf,3=live_261011_260901_cbnu_eightcircle_l3080_hyb3cf,4=live_261011_260910_object_l3080_hyb3cf,5=live_261011_260915_eightcircle_l3080_hyb3cf"
export OBJMAP_OUT=$PWD/out_$TAG OBJMAP_GT=$PWD/gt_$TAG FMFUSION_DATA=$W/fmfusion_$TAG DUALMAP_DATA=$W/dualmap_$TAG
# frames + poses laid out by real_prep.py: ids depend on the trajectory, so another tag gets its own dir (laptop tag keeps real_work)
export OBJMAP_REAL_WORK=${OBJMAP_REAL_WORK:-$HOME/DeepLearning/Dataset/real_work$([ "$TAG" = 261011 ] || echo _$TAG)}
SCENES="1 2 3 4 5"
L=$OBJMAP_OUT/real_logs; mkdir -p $L $FMFUSION_DATA $DUALMAP_DATA
cp -n $W/dualmap/real_prompts_colors.json $W/dualmap/real_prompts_id_colors.json $DUALMAP_DATA/
for s in $SCENES; do $PY real_prep.py --scene $s || echo "PREP_FAILED $s"; done
$PY fmfusion_prep.py --dataset real --scenes $SCENES > $L/fmf_prep.log 2>&1
$PY dualmap_prep.py --dataset real --scenes $SCENES > $L/dm_prep.log 2>&1
for s in $SCENES; do
  src=$W/fmfusion/scans/real_$(printf %06d $s)/prediction_no_augment
  dst=$FMFUSION_DATA/scans/real_$(printf %06d $s)/prediction_no_augment
  for f in $dst/../color/frame-*.png; do
    n=$(basename $f .png)
    for x in $src/${n}_*; do [ -e "$x" ] && ln -f "$x" $dst/; done
  done
done
echo "PREP_DONE $(date +%T)"
(
  $FMF fmfusion_detect.py --dataset real --scenes $SCENES --frame-gap 10
  $PY fmfusion_export.py --dataset real --scenes $SCENES
  echo "FMF_DONE $(date +%T)"
) > $L/fmfusion.log 2>&1 &
(
  for m in dualmap dualmap_bgr; do
    DUALMAP_METHOD=$m DUALMAP_DATASET=real DUALMAP_RUNS=$HOME/DeepLearning/baselines/DualMap/output/real_runs_${m}_$TAG bash dualmap_run.sh $SCENES
  done
  echo "DM_DONE $(date +%T)"
) > $L/dualmap.log 2>&1 &
wait
grep -h "DONE" $L/fmfusion.log $L/dualmap.log
$PY real_score.py --scenes $SCENES > $L/score.log 2>&1
cat $L/score.log
echo REAL_COMPARE_DONE
