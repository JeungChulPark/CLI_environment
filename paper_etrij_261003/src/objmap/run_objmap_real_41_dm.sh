#!/bin/bash
# 실데이터 객체 지도 비교(23·24 문서) — DualMap만 재실행 (.41, 2026-10-14).
#   10-13 run_objmap_real_41.sh 의 DualMap 10회가 전부 실패한 원인: dualmap env 에 mobileclip 모듈이 없어
#   Detector 초기화가 CLIP 단계에서 return → YOLO/SAM/FastSAM 미로드. .192 의 mobileclip 0.1.0 을 복사해 설치한 뒤 재실행.
#   제안/FM-Fusion 결과(out_41r)는 그대로 두고, DualMap 공개판(GPU0)·BGR판(GPU2)을 병렬로 돌린 뒤 real_score 재채점.
set -u
C=/home/jucpark/DeepLearning/CLI_environment; S=$C/paper_etrij_261003/src/objmap
PY=$HOME/anaconda3/envs/sam6d/bin/python
W=$HOME/DeepLearning/Dataset/bop/ycbv_work
DM=$HOME/DeepLearning/baselines/DualMap
LOG=$HOME/Dataset/bop/ycbv_work/logs/objmap_real_41_dm.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
cd $S
export OBJPOSE_REPO=$C SAM6D_ORIG=$C/sam6d_ws/sam6d_master/SAM-6D
export FASTSAM_X=$W/weights_dl/FastSAM-x.pt
export FMFUSION_ROOT=$C/../baselines/FM-Fusion GSAM_FMFUSION=$HOME/DeepLearning/baselines/gsam_fmfusion DUALMAP_REPO=$DM
export LD_LIBRARY_PATH=$HOME/anaconda3/envs/fmfbuild/lib:$HOME/DeepLearning/baselines/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib
export OBJMAP_REAL_RUNS="1=real5_260901_hybS05,2=real_260826_etri_eightcircle_dark_hybS05,3=real_260901_cbnu_eightcircle_hybS05,4=real_260910_object_hybS05,5=real_260915_eightcircle_hybS05"
export OBJMAP_OUT=$S/out_41r OBJMAP_GT=$S/gt_41r FMFUSION_DATA=$W/fmfusion_41r DUALMAP_DATA=$W/dualmap_41r
SCENES="1 2 3 4 5"
# 2차(08:5x): 1차에서 녹화 2·3 이 첫 프레임 교착(FastSAM 스레드 + YOLO/SAM 동시 첫 호출)으로 25분 timeout → DualMap 에 워밍업 패치 후
#   (워밍업은 원인이 아니었음: 실제 원인은 .41 DualMap 에 .192 의 26 문서 ② 패치(YOLO 검출 0개 프레임 건너뛰기)가 없어
#   녹화 2 첫 프레임에서 AttributeError → 작업 스레드만 남아 멈춤. .192 의 utils/object_detector.py 를 그대로 복사해 해결.)
#   끝난 녹화는 건너뛰고 남은 것만: SCENES_DM="2 3 4 5" SCENES_BGR="2 4 5" (export 가 out_41r/<method>/real/<scene> 에 있으면 완료).
L=$OBJMAP_OUT/real_logs; mkdir -p $L
say "DualMap 재실행 시작 (dualmap→GPU0, dualmap_bgr→GPU2 병렬, 각 5세션)"
( CUDA_VISIBLE_DEVICES=0 DUALMAP_METHOD=dualmap     DUALMAP_DATASET=real DUALMAP_RUNS=$DM/output/real_runs_dualmap_41r     bash dualmap_run.sh ${SCENES_DM:-$SCENES}; echo DM_DONE ) >> $L/dualmap_rerun.log 2>&1 &
( CUDA_VISIBLE_DEVICES=2 DUALMAP_METHOD=dualmap_bgr DUALMAP_DATASET=real DUALMAP_RUNS=$DM/output/real_runs_dualmap_bgr_41r bash dualmap_run.sh ${SCENES_BGR:-$SCENES}; echo DM_DONE ) >> $L/dualmap_bgr_rerun.log 2>&1 &
wait
say "DualMap 끝: $(grep -c '^== ' $L/dualmap_rerun.log)+$(grep -c '^== ' $L/dualmap_bgr_rerun.log) 회 실행, 실패 $(grep -c FAILED $L/dualmap_rerun.log $L/dualmap_bgr_rerun.log | awk -F: '{s+=$2} END{print s}') 회"
say "채점 real_score"
$PY real_score.py --scenes $SCENES > $L/score.log 2>&1 && say "score ok" || say "FAIL score"
say "OBJMAP REAL DM DONE"
