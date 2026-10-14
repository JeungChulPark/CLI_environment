#!/bin/bash
# FM-Fusion 2D 검출(GroundingDINO+SAM)을 .192(4090, 드라이버 580.178.04 로 올린 뒤)에서 처음부터 다시 돌리고 융합·채점 (2026-10-14 11:05).
#   .41 의 run_fmf_detect_41.sh 와 같은 구성. 10:38 의 chain fmf_detect 는 YCB-V 검출을 옛 드라이버(575) 때 쌓인 prediction 폴더가 있어 전부 건너뛰었으므로
#   새 작업 폴더 fmfusion_4090det2/ 에 color/depth/pose 링크만 두고 검출을 전부 새로 한다. 결과 out_4090det2/ (제안·DualMap 지도는 out_4090/ 링크).
#   msda=cuda(컴파일된 _C 커널, 노트북·.41 과 같은 원래 구성) — 드라이버 수정이 됐는지 원래 구성으로 확인. 디스플레이 GPU(bus E1, nvidia-smi 3번)는 피해 bus 01(0번) 사용.
set -u
C=$HOME/DeepLearning/server/CLI_environment; S=$C/paper_etrij_261003/src/objmap
source $HOME/Dataset/bop/ycbv_work/env.sh
PY=$HOME/anaconda3/envs/sam6d/bin/python; FMF=$HOME/anaconda3/envs/fmfusion/bin/python
W=$HOME/DeepLearning/Dataset/bop/ycbv_work
LOG=$HOME/Dataset/bop/ycbv_work/logs/fmf_detect_192.log
say(){ echo "[$(date "+%F %T")] $*" | tee -a $LOG; }
cd $S
export CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-0}
export FMFUSION_DATA=$W/fmfusion_4090det2 OBJMAP_OUT=$S/out_4090det2
SC=$FMFUSION_DATA/scans; mkdir -p $SC $FMFUSION_DATA/output $OBJMAP_OUT/logs
[ -e $FMFUSION_DATA/config_ycbv.yaml ] || { cp $W/fmfusion/config_*.yaml $W/fmfusion/likelihood_*.txt $FMFUSION_DATA/; cp -r $W/fmfusion/splits $FMFUSION_DATA/; }
for s in 0000{48..59} obchange_kitchen_00000{2..6} obchange_kitchen_all_00000{2..6}; do
  [ -d $SC/$s ] || { mkdir $SC/$s; for k in color depth pose; do ln -s $W/fmfusion/scans/$s/$k $SC/$s/$k; done; }
  case $s in
    obchange_kitchen_all_000006) mkdir -p $SC/$s/prediction_no_augment;;
    obchange_*) [ -e $SC/$s/prediction_no_augment ] || ln -s $SC/obchange_kitchen_all_000006/prediction_no_augment $SC/$s/prediction_no_augment;;
  esac
done
for m in ours_cad ours_obs ours_runs dualmap dualmap_bgr; do [ -e $OBJMAP_OUT/$m ] || ln -s $S/out_4090/$m $OBJMAP_OUT/$m; done
L=$OBJMAP_OUT/logs; MSDA=${OBJMAP_MSDA:-cuda}
say "START fmf_detect_192 (msda=$MSDA, GPU $CUDA_VISIBLE_DEVICES PCI order, driver $(cut -d" " -f8 /proc/driver/nvidia/version))"
$FMF fmfusion_detect.py --msda $MSDA --scenes 48 49 50 51 52 53 54 55 56 57 58 59 --frame-gap 10 > $L/fmfusion_detect_ycbv.log 2>&1; rc=$?; say "detect ycbv rc=$rc"; [ $rc = 0 ] || { say "ABORT (ycbv detect failed)"; exit 1; }
$FMF fmfusion_detect.py --msda $MSDA --dataset obchange_kitchen_all --scenes 6 --frame-gap 10 > $L/fmfusion_detect_obchange.log 2>&1; rc=$?; say "detect obchange rc=$rc"; [ $rc = 0 ] || { say "ABORT (obchange detect failed)"; exit 1; }
$PY fmfusion_export.py --scenes 48 49 50 51 52 53 54 55 56 57 58 59 > $L/fmfusion_ycbv.log 2>&1; say "fusion ycbv rc=$?"
$PY fmfusion_export.py --dataset obchange_kitchen_all --scenes 2 3 4 5 6 > $L/fmfusion_obchange.log 2>&1; say "fusion obchange_all rc=$?"
$PY fmfusion_export.py --dataset obchange_kitchen --scenes 2 3 4 5 6 >> $L/fmfusion_obchange.log 2>&1; say "fusion obchange visits rc=$?"
for v in "" "--clean" "--match center" "--match center --clean"; do CUDA_VISIBLE_DEVICES="" $PY evaluate.py --method fmfusion $v > "$L/eval_fmfusion_${v// /_}.log" 2>&1; say "eval fmfusion $v rc=$?"; done
CUDA_VISIBLE_DEVICES="" $PY obchange_score.py --methods ours_cad ours_obs fmfusion dualmap dualmap_bgr > $L/obchange_score_all.log 2>&1; say "obchange score rc=$?"
say "FMF DETECT 192 ALL DONE"
