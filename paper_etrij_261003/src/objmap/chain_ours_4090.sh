#!/bin/bash
# Proposed method (improvement 3 + cluster-first) for the F-bundle object-map comparison, on this PC
# (RTX 4090, GPU0 only, one live hub at a time: the frame channel /sam6d_frame is a fixed name).
# Outputs go to out_4090/ (OBJMAP_OUT); the laptop maps in out/ are untouched. GT in gt/ is reused
# (ycbv_gt.py / obchange_gt.py are deterministic and the ObChange source data is not on this PC).
set -u
cd "$(dirname "$0")"
source $HOME/Dataset/bop/ycbv_work/env.sh
export CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-0} OBJMAP_OUT=$PWD/out_4090 SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
# OBJMAP_GPU: nvidia-smi(PCI 버스 순) 번호. 디스플레이가 붙은 GPU(nvidia-smi 에서 Xorg 가 수백 MB 쓰는 것, 부팅마다 바뀜)는 피할 것 — 10-07/08/13/14 드라이버 멈춤 재발.
PY=$SAM6D_PY; L=$OBJMAP_OUT/logs; mkdir -p $L
say(){ echo "[$(date '+%F %T')] $*" | tee -a $L/chain.log; }

# ---------- stage: FM-Fusion detection on this PC (added 2026-10-13) ----------
# `bash chain_ours_4090.sh fmf_detect`: re-run GroundingDINO+SAM on the 4090 (frame gap 10, as on the laptop) for
# YCB-V 48-59 and the ObChange kitchen (all visits share one prediction folder, scans/obchange_kitchen_all_000006),
# then fusion -> export -> evaluate -> obchange_score. Scan folders become real dirs with color/depth/pose linked to
# the laptop scans so the laptop detections stay untouched. The fusion-only results that reused the laptop
# detections are kept as out_4090/fmfusion_laptopdet/, out_4090/obchange_scores_laptopdet.json, fmfusion_4090/output_laptopdet/.
if [ "${1:-}" = fmf_detect ]; then
  W=$HOME/DeepLearning/Dataset/bop/ycbv_work; export FMFUSION_DATA=$W/fmfusion_4090; FMF=$HOME/anaconda3/envs/fmfusion/bin/python
  S=$FMFUSION_DATA/scans; mkdir -p $S $FMFUSION_DATA/output
  for s in 0000{48..59} obchange_kitchen_00000{2..6} obchange_kitchen_all_00000{2..6}; do
    if [ -L $S/$s ] || [ ! -d $S/$s ]; then rm -f $S/$s; mkdir $S/$s; for k in color depth pose; do ln -s $W/fmfusion/scans/$s/$k $S/$s/$k; done; fi
    case $s in
      obchange_kitchen_all_000006) mkdir -p $S/$s/prediction_no_augment;;
      obchange_*) [ -e $S/$s/prediction_no_augment ] || ln -s $S/obchange_kitchen_all_000006/prediction_no_augment $S/$s/prediction_no_augment;;
    esac
  done
  if [ -d $OBJMAP_OUT/fmfusion/ycbv ] && [ ! -d $OBJMAP_OUT/fmfusion_laptopdet ]; then
    mkdir $OBJMAP_OUT/fmfusion_laptopdet $FMFUSION_DATA/output_laptopdet
    mv $OBJMAP_OUT/fmfusion/ycbv $OBJMAP_OUT/fmfusion/obchange_kitchen $OBJMAP_OUT/fmfusion/obchange_kitchen_all $OBJMAP_OUT/fmfusion/metrics_* $OBJMAP_OUT/fmfusion_laptopdet/
    cp $OBJMAP_OUT/obchange_scores.json $OBJMAP_OUT/obchange_scores_laptopdet.json
    mv $FMFUSION_DATA/output/0000* $FMFUSION_DATA/output/obchange_* $FMFUSION_DATA/output_laptopdet/
    say "fusion-only results moved to fmfusion_laptopdet"
  fi
  MSDA=${OBJMAP_MSDA:-pytorch}   # 10-14: the compiled _C deformable-attention kernel is the prime freeze suspect; default to the PyTorch path
  say "START fmf_detect (msda=$MSDA, CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES)"
  $FMF fmfusion_detect.py --msda $MSDA --scenes 48 49 50 51 52 53 54 55 56 57 58 59 --frame-gap 10 > $L/fmfusion_detect_ycbv.log 2>&1; rc=$?; say "fmfusion detect ycbv rc=$rc"; [ $rc = 0 ] || { say "fmf_detect ABORT (ycbv detect failed; fusion not run)"; exit 1; }
  $FMF fmfusion_detect.py --msda $MSDA --dataset obchange_kitchen_all --scenes 6 --frame-gap 10 > $L/fmfusion_detect_obchange.log 2>&1; rc=$?; say "fmfusion detect obchange rc=$rc"; [ $rc = 0 ] || { say "fmf_detect ABORT (obchange detect failed; fusion not run)"; exit 1; }
  $PY fmfusion_export.py --scenes 48 49 50 51 52 53 54 55 56 57 58 59 > $L/fmfusion_ycbv_4090det.log 2>&1; say "fmfusion ycbv rc=$?"
  $PY fmfusion_export.py --dataset obchange_kitchen_all --scenes 2 3 4 5 6 > $L/fmfusion_obchange_4090det.log 2>&1; say "fmfusion obchange_all rc=$?"
  $PY fmfusion_export.py --dataset obchange_kitchen --scenes 2 3 4 5 6 >> $L/fmfusion_obchange_4090det.log 2>&1; say "fmfusion obchange visits rc=$?"
  m=fmfusion
  $PY evaluate.py --method $m                 > $L/eval_${m}_iou.log 2>&1
  $PY evaluate.py --method $m --clean         > $L/eval_${m}_iou_clean.log 2>&1
  $PY evaluate.py --method $m --match center  > $L/eval_${m}_center.log 2>&1
  $PY evaluate.py --method $m --match center --clean > $L/eval_${m}_center_clean.log 2>&1
  say "fmfusion eval done"
  $PY obchange_score.py --methods ours_cad ours_obs fmfusion dualmap dualmap_bgr > $L/obchange_score_all.log 2>&1; say "obchange score (all methods) rc=$?"
  say "FMF DETECT 4090 ALL DONE"; exit 0
fi
say "START obchange"
$PY run_ours.py run    --dataset obchange_kitchen_all --scenes 6 > $L/ours_obchange_all.log 2>&1;  say "obchange_all run rc=$?"
$PY run_ours.py export --dataset obchange_kitchen_all --scenes 6 >> $L/ours_obchange_all.log 2>&1; say "obchange_all export rc=$?"
$PY run_ours.py run    --dataset obchange_kitchen --scenes 2 3 4 5 6 > $L/ours_obchange_visits.log 2>&1;  say "obchange visits run rc=$?"
$PY run_ours.py export --dataset obchange_kitchen --scenes 2 3 4 5 6 >> $L/ours_obchange_visits.log 2>&1; say "obchange visits export rc=$?"
$PY obchange_score.py --methods ours_cad ours_obs > $L/obchange_score.log 2>&1; say "obchange score rc=$?"
say "START ycbv"
$PY run_ours.py run    --scenes 48 49 50 51 52 53 54 55 56 57 58 59 > $L/ours_ycbv.log 2>&1;  say "ycbv run rc=$?"
$PY run_ours.py export --scenes 48 49 50 51 52 53 54 55 56 57 58 59 >> $L/ours_ycbv.log 2>&1; say "ycbv export rc=$?"
for m in ours_cad ours_obs; do
  $PY evaluate.py --method $m                 > $L/eval_${m}_iou.log 2>&1
  $PY evaluate.py --method $m --clean         > $L/eval_${m}_iou_clean.log 2>&1
  $PY evaluate.py --method $m --match center  > $L/eval_${m}_center.log 2>&1
  $PY evaluate.py --method $m --match center --clean > $L/eval_${m}_center_clean.log 2>&1
done
say "ycbv eval done"
say "OBJMAP OURS ALL DONE"

# ---------- baselines on this PC (added 2026-10-12) ----------
# FM-Fusion: CPU only (IntegrateInstanceMap); the laptop's GroundingDINO+SAM detections in ycbv_work/fmfusion/scans are
# reused (same GT trajectories → same frames), raw maps go to fmfusion_4090/output so the laptop's stay untouched.
# DualMap: GPU0 (runs after the proposed method, one GPU job at a time on this PC — the driver hung twice when jobs overlapped).
W=$HOME/DeepLearning/Dataset/bop/ycbv_work; DMO=$HOME/DeepLearning/baselines/DualMap/output
export FMFUSION_DATA=$W/fmfusion_4090
if [ ! -d $FMFUSION_DATA ]; then mkdir -p $FMFUSION_DATA; ln -s $W/fmfusion/scans $FMFUSION_DATA/scans
  cp $W/fmfusion/config_*.yaml $W/fmfusion/likelihood_*.txt $FMFUSION_DATA/; cp -r $W/fmfusion/splits $FMFUSION_DATA/; fi
say "START baselines"
$PY fmfusion_export.py --scenes 48 49 50 51 52 53 54 55 56 57 58 59 > $L/fmfusion_ycbv.log 2>&1; say "fmfusion ycbv rc=$?"
$PY fmfusion_export.py --dataset obchange_kitchen_all --scenes 2 3 4 5 6 > $L/fmfusion_obchange.log 2>&1; say "fmfusion obchange_all rc=$?"
$PY fmfusion_export.py --dataset obchange_kitchen --scenes 2 3 4 5 6 >> $L/fmfusion_obchange.log 2>&1; say "fmfusion obchange visits rc=$?"
for m in dualmap dualmap_bgr; do
  DUALMAP_METHOD=$m DUALMAP_RUNS=$DMO/ycbv_runs_${m}_4090 bash dualmap_run.sh 48 49 50 51 52 53 54 55 56 57 58 59 > $L/dualmap_${m}_ycbv.log 2>&1; say "$m ycbv rc=$?"
  DUALMAP_METHOD=$m DUALMAP_DATASET=obchange_kitchen     DUALMAP_RUNS=$DMO/obkv_runs_${m}_4090 bash dualmap_run.sh 2 3 4 5 6 > $L/dualmap_${m}_obkv.log 2>&1; say "$m obchange visits rc=$?"
  DUALMAP_METHOD=$m DUALMAP_DATASET=obchange_kitchen_all DUALMAP_RUNS=$DMO/obka_runs_${m}_4090 bash dualmap_run.sh 2 3 4 5 6 > $L/dualmap_${m}_obka.log 2>&1; say "$m obchange_all rc=$?"
done
for m in fmfusion dualmap dualmap_bgr; do
  $PY evaluate.py --method $m                 > $L/eval_${m}_iou.log 2>&1
  $PY evaluate.py --method $m --clean         > $L/eval_${m}_iou_clean.log 2>&1
  $PY evaluate.py --method $m --match center  > $L/eval_${m}_center.log 2>&1
  $PY evaluate.py --method $m --match center --clean > $L/eval_${m}_center_clean.log 2>&1
done
$PY obchange_score.py --methods ours_cad ours_obs fmfusion dualmap dualmap_bgr > $L/obchange_score_all.log 2>&1; say "obchange score (all methods) rc=$?"
say "OBJMAP BASELINES ALL DONE"
