#!/bin/bash
# ObChange baselines, after the proposed method has released the GPU (run_obchange.sh prints OURS_DONE).
#   FM-Fusion: GroundingDINO + SAM once on the whole sequence (Kitchen 30 fps: every 10th frame, integrate gap 2;
#              the other rooms are 10 fps BOP frames: every 3rd frame, integrate gap 1), then IntegrateInstanceMap
#              per sequence (each visit alone; visits 1..k for k = 2..LAST)
#   DualMap:   as published and with BGR input, per sequence (it keeps no per-visit snapshot, so the
#              continuous map at the end of visit k is a run over visits 1..k)
# Usage: bash run_obchange_baselines.sh [ROOM=Kitchen] [LAST=6]
set -u
cd "$(dirname "$0")"
ROOM=${1:-Kitchen}; LAST=${2:-6}
source ./obchange_env.sh
D=obchange_$R
VIS=$(seq -s ' ' 2 "$LAST")
# MAXF = IntegrateInstanceMap --max_frames (largest global frame id; BigRoom runs to ~32.8k). The non-Kitchen rooms are
# longer (SmallRoom ~16.6k, BigRoom ~32.8k frames), so DualMap's per-run timeout is raised unless set.
if [ "$R" = kitchen ]; then DET_GAP=10; INT_GAP=2; MAXF=20000
else DET_GAP=3; INT_GAP=1; MAXF=100000; export DUALMAP_TIMEOUT=${DUALMAP_TIMEOUT:-14400}; fi
until grep -q OURS_DONE $L/chain_ours.log 2>/dev/null; do sleep 30; done
until grep -q "ob${R:0:1}v$(printf %06d "$LAST")" $L/dm_prep.log 2>/dev/null; do sleep 10; done
FMF=${FMF:-$HOME/anaconda3/envs/fmfusion/bin/python}; PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}
(
  $FMF fmfusion_detect.py --dataset ${D}_all --scenes $LAST --frame-gap $DET_GAP
  $PY fmfusion_export.py --dataset ${D}_all --scenes $VIS --frame_gap $INT_GAP --max_frames $MAXF
  $PY fmfusion_export.py --dataset $D --scenes $VIS --frame_gap $INT_GAP --max_frames $MAXF
  echo FMF_DONE
) > $L/fmfusion.log 2>&1 &
(
  for m in dualmap dualmap_bgr; do
    DUALMAP_METHOD=$m DUALMAP_DATASET=$D bash dualmap_run.sh $VIS
    DUALMAP_METHOD=$m DUALMAP_DATASET=${D}_all bash dualmap_run.sh $VIS
  done
  echo DM_DONE
) > $L/dualmap.log 2>&1 &
wait
echo BASELINES_DONE
