#!/bin/bash
# ObChange-Kitchen baselines, after the proposed method has released the GPU (run_obchange.sh prints OURS_DONE).
#   FM-Fusion: GroundingDINO + SAM once on the whole sequence (every 10th frame), then IntegrateInstanceMap
#              per sequence (each visit alone; visits 1..k for k = 2..6)
#   DualMap:   as published and with BGR input, per sequence (it keeps no per-visit snapshot, so the
#              continuous map at the end of visit k is a run over visits 1..k)
set -u
cd "$(dirname "$0")"
L=out/obchange_logs
until grep -q OURS_DONE $L/chain_ours.log 2>/dev/null; do sleep 30; done
until grep -q "obkv000006" $L/dm_prep.log 2>/dev/null; do sleep 10; done
FMF=$HOME/anaconda3/envs/fmfusion/bin/python; PY=$HOME/anaconda3/envs/sam6d/bin/python
(
  $FMF fmfusion_detect.py --dataset obchange_kitchen_all --scenes 6 --frame-gap 10
  $PY fmfusion_export.py --dataset obchange_kitchen_all --scenes 2 3 4 5 6
  $PY fmfusion_export.py --dataset obchange_kitchen --scenes 2 3 4 5 6
  echo FMF_DONE
) > $L/fmfusion.log 2>&1 &
(
  for m in dualmap dualmap_bgr; do
    DUALMAP_METHOD=$m DUALMAP_DATASET=obchange_kitchen bash dualmap_run.sh 2 3 4 5 6
    DUALMAP_METHOD=$m DUALMAP_DATASET=obchange_kitchen_all bash dualmap_run.sh 2 3 4 5 6
  done
  echo DM_DONE
) > $L/dualmap.log 2>&1 &
wait
echo BASELINES_DONE
