#!/bin/bash
# 개선안 3 (FastSAM-x > ViT-S > 0.5 > our PEM + verification) re-run on YCB-V live, 2026-10-06 evening
set -u
cd "$(dirname "$0")"; PY=$HOME/anaconda3/envs/sam6d/bin/python
R=../../../objpose/output
REC="--recognizer $(cd ../realdata && pwd)/live_cores.py"
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 \
  YCBV_ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml HUB_EXTRA="$REC" \
  bash run_live.sh hybS05_r2 48 49 50 51 52 53 54 55 56 57 58 59
$PY eval_live.py --runs "$R/ycbv_live_hybS05_r2_0000??" --out results_hybS05_r2.json > eval_hybS05_r2.log 2>&1
$PY eval_penalized.py --tags orig_th05,g13p,origS_th05,hybS05,hybS05_r2,hybS05_nover --out results_penalized_combo5.json > eval_penalized_combo5.log 2>&1
$PY eval_breakdown.py orig orig_th05 origS_th05 origS_th06 origS_th07 v2 m15 g13p g13p_nover hybL05 hybS05 hybS05_nover hybS05_r2 > eval_breakdown.log 2>&1
echo R2_DONE
