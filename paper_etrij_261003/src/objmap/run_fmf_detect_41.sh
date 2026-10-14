#!/bin/bash
# FM-Fusion 2D 검출(GroundingDINO+SAM)을 .41 Blackwell 에서 다시 돌리고 융합·채점 (2026-10-14).
#   .192(4090) 의 chain_ours_4090.sh fmf_detect 단계 이식 — .192 는 이 검출만 띄우면 드라이버가 멈춰(10-13, 10-14 ×2) 못 끝냄.
#   YCB-V 48~59 + ObChange-Kitchen(방문 2~6 은 all_000006 의 검출 폴더 하나를 공유), 10프레임마다 검출.
#   scans 는 fmfusion_41det/ 에 color/depth/pose 링크만 두고 검출 폴더를 새로 만들어 노트북 검출(fmfusion/scans)은 그대로 둔다.
#   결과는 out_41det/ (제안·DualMap 지도는 out/ 로 링크해 같은 채점기로 함께 채점). 20·21 문서의 out/ 는 건드리지 않는다.
set -u
C=/home/jucpark/DeepLearning/CLI_environment; S=$C/paper_etrij_261003/src/objmap
PY=$HOME/anaconda3/envs/sam6d/bin/python; FMF=$HOME/anaconda3/envs/fmfusion/bin/python
W=$HOME/DeepLearning/Dataset/bop/ycbv_work
LOG=$HOME/Dataset/bop/ycbv_work/logs/fmf_detect_41.log
say(){ echo "[$(date '+%F %T')] $*" | tee -a $LOG; }
cd $S
export OBJPOSE_REPO=$C SAM6D_ORIG=$C/sam6d_ws/sam6d_master/SAM-6D
export YCBV_DIR=$HOME/DeepLearning/Dataset/bop/ycbv YCBV_WORK=$W
export FMFUSION_ROOT=$C/../baselines/FM-Fusion GSAM_FMFUSION=$HOME/DeepLearning/baselines/gsam_fmfusion
export LD_LIBRARY_PATH=$HOME/anaconda3/envs/fmfbuild/lib:$HOME/DeepLearning/baselines/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib
export FMFUSION_DATA=$W/fmfusion_41det OBJMAP_OUT=$S/out_41det
export CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-2}
SC=$FMFUSION_DATA/scans; mkdir -p $SC $FMFUSION_DATA/output $OBJMAP_OUT/logs
# IntegrateInstanceMap 이 읽는 config_*.yaml·likelihood_*.txt·splits 는 노트북 작업 폴더에서 복사 (10-14 첫 실행에서 빠져 융합이 전부 실패했음)
[ -e $FMFUSION_DATA/config_ycbv.yaml ] || { cp $W/fmfusion/config_*.yaml $W/fmfusion/likelihood_*.txt $FMFUSION_DATA/; cp -r $W/fmfusion/splits $FMFUSION_DATA/; }
for s in 0000{48..59} obchange_kitchen_00000{2..6} obchange_kitchen_all_00000{2..6}; do
  [ -d $SC/$s ] || { mkdir $SC/$s; for k in color depth pose; do ln -s $W/fmfusion/scans/$s/$k $SC/$s/$k; done; }
  case $s in
    obchange_kitchen_all_000006) mkdir -p $SC/$s/prediction_no_augment;;
    obchange_*) [ -e $SC/$s/prediction_no_augment ] || ln -s $SC/obchange_kitchen_all_000006/prediction_no_augment $SC/$s/prediction_no_augment;;
  esac
done
for m in ours_cad ours_obs ours_runs dualmap dualmap_bgr; do [ -e $OBJMAP_OUT/$m ] || ln -s $S/out/$m $OBJMAP_OUT/$m; done
L=$OBJMAP_OUT/logs
say "START fmf_detect_41 (GPU $CUDA_VISIBLE_DEVICES)"
$FMF fmfusion_detect.py --scenes 48 49 50 51 52 53 54 55 56 57 58 59 --frame-gap 10 > $L/fmfusion_detect_ycbv.log 2>&1; rc=$?; say "detect ycbv rc=$rc"; [ $rc = 0 ] || { say "ABORT (ycbv detect failed)"; exit 1; }
$FMF fmfusion_detect.py --dataset obchange_kitchen_all --scenes 6 --frame-gap 10 > $L/fmfusion_detect_obchange.log 2>&1; rc=$?; say "detect obchange rc=$rc"; [ $rc = 0 ] || { say "ABORT (obchange detect failed)"; exit 1; }
$PY fmfusion_export.py --scenes 48 49 50 51 52 53 54 55 56 57 58 59 > $L/fmfusion_ycbv.log 2>&1; say "fusion ycbv rc=$?"
$PY fmfusion_export.py --dataset obchange_kitchen_all --scenes 2 3 4 5 6 > $L/fmfusion_obchange.log 2>&1; say "fusion obchange_all rc=$?"
$PY fmfusion_export.py --dataset obchange_kitchen --scenes 2 3 4 5 6 >> $L/fmfusion_obchange.log 2>&1; say "fusion obchange visits rc=$?"
for v in "" "--clean" "--match center" "--match center --clean"; do CUDA_VISIBLE_DEVICES="" $PY evaluate.py --method fmfusion $v > "$L/eval_fmfusion_${v// /_}.log" 2>&1; say "eval fmfusion $v rc=$?"; done
CUDA_VISIBLE_DEVICES="" $PY obchange_score.py --methods ours_cad ours_obs fmfusion dualmap dualmap_bgr > $L/obchange_score_all.log 2>&1; say "obchange score rc=$?"
say "FMF DETECT 41 ALL DONE"
