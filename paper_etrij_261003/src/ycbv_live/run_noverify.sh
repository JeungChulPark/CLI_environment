#!/bin/bash
# the improved recogniser with pose verification OFF (ablation), on the same three evaluations:
# YCB-V 36 images (G123 + method-1 prompts), YCB-V live (g13p), real data 260901 (g13)
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); SR=$REPO/sam6d_realtime; R=$REPO/objpose/output
# 1) 36 images
cd ../ycbv
YCBV_SUBSET=25 YCBV_OUT=$PWD/out/quick $PY run_ours.py --mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem 0.25 \
  --gate-appe 0.605 --gate-hsv 0.06 --proposer text:yolov8m-worldv2.pt:$PWD/det_study/out/prompts_best_m.json \
  --no-verify --out $PWD/out/quick/pred_gate_G123_p_noverify.json > out/quick/noverify.log 2>&1 || echo "FAIL quick"
echo QUICK_DONE
# 2) YCB-V live
cd $HERE
YCBV_ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects_g13p.yaml HUB_EXTRA="--no-verify" bash run_live.sh g13p_nover
$PY eval_live.py --runs "$R/ycbv_live_g13p_nover_0000??" --out results_g13p_nover.json > eval_g13p_nover.log 2>&1
$PY eval_penalized.py --tags g13p,g13p_nover,orig_th05 --out results_penalized_nover.json > eval_penalized_nover.log 2>&1
$PY eval_breakdown.py orig orig_th05 origS_th05 origS_th06 origS_th07 v2 m15 g13p g13p_nover > eval_breakdown.log 2>&1
echo LIVE_DONE
# 3) real data (Mac ORB-SLAM3 f2000)
cd ../realdata
until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
OUT=$R/real_260901_g13_nover; mkdir -p $OUT
$PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT --ism-config $SR/configs/yolo_ism_objects_g13.yaml --no-verify > $OUT.log 2>&1 &
for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
sleep 3; kill %1 2>/dev/null; echo "done real $(grep -c 'summary written' $OUT.log)"
$PY eval_real.py --runs real_260901_ours real_260901_g13 real_260901_g13_nover real_260901_g13_k50 real_260901_origL05 real_260901_hybL05 real_260901_hybS05 real_260901_hybS06 --out results_real_nover.json > eval_nover.log 2>&1
echo NOVER_DONE
