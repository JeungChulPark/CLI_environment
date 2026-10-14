#!/bin/bash
# 실데이터 객체 지도 비교 (23·24 문서)를 .41 Blackwell 궤적으로 (2026-10-13).
#   제안 방법은 오늘 real08 체인의 Blackwell 실행(real_<ses>_hybS05, real5_260901_hybS05)을 궤적으로 사용 → SLAM 재실행 불필요.
#   real_prep(오늘 궤적으로 RGB-D 시퀀스) → FM-Fusion·DualMap 재실행 → real_score 채점.
#   주말 결과(out/, gt/) 보존: 별도 out_41r / gt_41r / fmfusion_41r / dualmap_41r 사용.
set -u
C=/home/jucpark/DeepLearning/CLI_environment; S=$C/paper_etrij_261003/src/objmap
PY=$HOME/anaconda3/envs/sam6d/bin/python; FMF=$HOME/anaconda3/envs/fmfusion/bin/python
W=$HOME/DeepLearning/Dataset/bop/ycbv_work
LOG=$HOME/Dataset/bop/ycbv_work/logs/objmap_real_41.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
cd $S
export OBJPOSE_REPO=$C SAM6D_ORIG=$C/sam6d_ws/sam6d_master/SAM-6D
export FASTSAM_X=$W/weights_dl/FastSAM-x.pt
export FMFUSION_ROOT=$C/../baselines/FM-Fusion GSAM_FMFUSION=$HOME/DeepLearning/baselines/gsam_fmfusion DUALMAP_REPO=$HOME/DeepLearning/baselines/DualMap
export LD_LIBRARY_PATH=$HOME/anaconda3/envs/fmfbuild/lib:$HOME/DeepLearning/baselines/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib
# 5개 세션 → 오늘 Blackwell 실행 매핑 (1은 bigeightcircle)
export OBJMAP_REAL_RUNS="1=real5_260901_hybS05,2=real_260826_etri_eightcircle_dark_hybS05,3=real_260901_cbnu_eightcircle_hybS05,4=real_260910_object_hybS05,5=real_260915_eightcircle_hybS05"
export OBJMAP_OUT=$S/out_41r OBJMAP_GT=$S/gt_41r FMFUSION_DATA=$W/fmfusion_41r DUALMAP_DATA=$W/dualmap_41r
SCENES="1 2 3 4 5"
L=$OBJMAP_OUT/real_logs; mkdir -p $L $FMFUSION_DATA $DUALMAP_DATA
cp -n $W/dualmap/real_prompts*colors.json $DUALMAP_DATA/ 2>/dev/null || true

say "real_prep (오늘 궤적으로 RGB-D)"
for s in $SCENES; do $PY real_prep.py --scene $s >> $LOG 2>&1 || say "PREP_FAIL $s"; done
say "fmfusion_prep / dualmap_prep"
$PY fmfusion_prep.py --dataset real --scenes $SCENES > $L/fmf_prep.log 2>&1 || say "FAIL fmf_prep"
$PY dualmap_prep.py  --dataset real --scenes $SCENES > $L/dm_prep.log  2>&1 || say "FAIL dm_prep"
say "FM-Fusion 검출+export, DualMap 실행 (병렬)"
( $FMF fmfusion_detect.py --dataset real --scenes $SCENES --frame-gap 10 && $PY fmfusion_export.py --dataset real --scenes $SCENES && echo FMF_DONE ) > $L/fmfusion.log 2>&1 &
( for m in dualmap dualmap_bgr; do DUALMAP_METHOD=$m DUALMAP_DATASET=real bash dualmap_run.sh $SCENES; done; echo DM_DONE ) > $L/dualmap.log 2>&1 &
wait
say "채점 real_score"
$PY real_score.py --scenes $SCENES > $L/score.log 2>&1 && say "score ok" || say "FAIL score"
say "OBJMAP REAL DONE"
