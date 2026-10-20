#!/bin/bash
# 2026-10-19 밤: 29 문서 §5 재현 — ObChange 네 방(Office · LivingArea · SmallRoom · BigRoom)을 모든 방법 같은 실시간 30 fps 로
# .41 에서. 방마다 변환(convert_room_41.sh / convert_bigroom_41.sh 의 CONVERT_DONE rc=0)을 기다린 뒤
# src/objmap/run_obchange_rt.sh ROOM LAST 를 돈다(제안 → FM-Fusion 실시간 검출 → FM-Fusion 융합 · DualMap 두 판 → 채점).
#   GPU 2 (GPU 0 은 자이로 실험 hub 가 씀), hub 포트 8775/17011 (자이로 hub 8766/17001 과 다름).
#   결과: src/objmap/out30/, gt30/, 중간물 ~/DeepLearning/Datasets/objmap_obchange30 (= Dataset 링크, / 디스크).
set -u
C=/home/jucpark/DeepLearning/CLI_environment; S=$C/paper_etrij_261003/src/objmap; W=$HOME/Dataset/bop/ycbv_work
LOG=$W/logs/obchange_rooms_41.log; mkdir -p $W/logs
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
export OBJPOSE_REPO=$C SAM6D_ORIG=$C/sam6d_ws/sam6d_master/SAM-6D SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
export YCBV_DIR=$HOME/DeepLearning/Dataset/bop/ycbv YCBV_WORK=$W FASTSAM_X=$W/weights_dl/FastSAM-x.pt
export FMFUSION_ROOT=$HOME/DeepLearning/baselines/FM-Fusion GSAM_FMFUSION=$HOME/DeepLearning/baselines/gsam_fmfusion
export DUALMAP_REPO=$HOME/DeepLearning/baselines/DualMap BASELINE_WEIGHTS=$HOME/DeepLearning/baselines/weights
export LD_LIBRARY_PATH=$HOME/anaconda3/envs/fmfbuild/lib:$HOME/DeepLearning/baselines/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib
export FMF=$HOME/anaconda3/envs/fmfusion/bin/python          # run_obchange_rt.sh 의 $FMF (노트북 fmf_python.sh 대신)
export CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-2}
[ -e $HOME/DeepLearning/Datasets ] || ln -s $HOME/DeepLearning/Dataset $HOME/DeepLearning/Datasets   # run_obchange_rt.sh 의 W30 경로
W30=$HOME/DeepLearning/Datasets/objmap_obchange30; mkdir -p $W30/fmfusion $W30/dualmap $W30/bags
[ -e $W30/fmfusion/likelihood_ycbv.txt ] || cp $W/fmfusion/likelihood_*.txt $W30/fmfusion/
[ -d $W30/fmfusion/splits ] || cp -r $W/fmfusion/splits $W30/fmfusion/
cd $S
for RL in "Office 6" "LivingArea 6" "SmallRoom 6" "BigRoom 7"; do
  set -- $RL; ROOM=$1; LAST=$2; r=$(echo $ROOM | tr A-Z a-z)
  until grep -q "CONVERT_DONE $ROOM rc=0" out30/logs_$r/convert.log 2>/dev/null; do
    grep -qE "MISMATCH|CONVERT_DONE $ROOM rc=[1-9]" out30/logs_$r/convert.log 2>/dev/null && { say "SKIP $ROOM: conversion failed"; continue 2; }
    sleep 120; done
  say "START $ROOM (last visit $LAST, GPU $CUDA_VISIBLE_DEVICES)"
  bash run_obchange_rt.sh $ROOM $LAST > out30/logs_$r/rt_chain.log 2>&1; rc=$?
  say "DONE $ROOM rc=$rc"; grep -E "^== " out30/logs_$r/rt_chain.log | tail -12 | tee -a $LOG
  tail -25 out30/logs_$r/score.log 2>/dev/null | tee -a $LOG
done
say "ROOMS_ALL_DONE"
