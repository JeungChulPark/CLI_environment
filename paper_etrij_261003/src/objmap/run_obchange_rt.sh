#!/bin/bash
# ObChange room, every method under the same real-time condition (doc 29): the 30 fps frames of the raw bags
# (obchange_raw_seq.py) arrive on the wall clock and each method takes the newest frame whenever it is free.
#   ours        live hub replay at 1x (as before; 30 fps bags written per run and deleted after, SEQ_BAGS_DELETE)
#   FM-Fusion   fmfusion_detect.py --realtime-fps 30 on the continuous sequence (GroundingDINO + SAM on the newest
#               frame), then IntegrateInstanceMap on the frames it took (per-frame integration time is logged in
#               time_records.txt so it can be checked against the detection interval)
#   DualMap     stride=1, DM_RT_INTERVAL=1/30 (runner_dataset.py drops frames that are already stale), as published
#               and with BGR input
# Everything goes to separate trees (gt30/, out30/, ~/DeepLearning/Datasets/objmap_obchange30/), so the 10 fps
# results stay as they are. The GPU runs one method at a time.
# Usage: bash run_obchange_rt.sh ROOM LAST      e.g. bash run_obchange_rt.sh Office 6
set -u
cd "$(dirname "$0")"
ROOM=${1:?ROOM}; LAST=${2:?LAST}
W30=$HOME/DeepLearning/Datasets/objmap_obchange30
export OBJMAP_GT=$PWD/gt30 OBJMAP_OUT=$PWD/out30 OBJMAP_OBCHANGE_FRAMES=$W30/frames SEQ_BAGS=$W30/bags \
       FMFUSION_DATA=$W30/fmfusion DUALMAP_DATA=$W30/dualmap SEQ_BAGS_DELETE=1
source ./obchange_env.sh
L=out30/logs_$R
PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}
D=obchange_$R
VIS=$(seq -s ' ' 2 "$LAST")
mkdir -p $L
step() { echo "== $* $(date +%T)"; }
if [ -z "${SKIP_PREP:-}" ]; then
step dm_prep;  { $PY dualmap_prep.py --dataset ${D}_all --scenes $VIS; $PY dualmap_prep.py --dataset $D --scenes $VIS; } > $L/dm_prep.log 2>&1
step fmf_prep; { $PY fmfusion_prep.py --dataset ${D}_all --scenes $VIS; $PY fmfusion_prep.py --dataset $D --scenes $VIS; } > $L/fmf_prep.log 2>&1
fi
step ours
{ $PY run_ours.py run --dataset ${D}_all --scenes $LAST; $PY run_ours.py export --dataset ${D}_all --scenes $LAST
  $PY run_ours.py run --dataset $D --scenes $VIS; $PY run_ours.py export --dataset $D --scenes $VIS; } > $L/ours.log 2>&1
step fmf_detect
$FMF fmfusion_detect.py --dataset ${D}_all --scenes $LAST --realtime-fps 30 > $L/fmf_detect.log 2>&1
step fmf_integrate_bg     # CPU, alongside DualMap
( $PY fmfusion_export.py --dataset ${D}_all --scenes $VIS --frame_gap 1 --max_frames 100000
  $PY fmfusion_export.py --dataset $D --scenes $VIS --frame_gap 1 --max_frames 100000; echo FMF_DONE ) > $L/fmfusion.log 2>&1 &
FMFPID=$!
step dualmap
export DM_RT_INTERVAL=0.0333333 DUALMAP_EXTRA="stride=1" DUALMAP_TIMEOUT=${DUALMAP_TIMEOUT:-14400}
for m in dualmap dualmap_bgr; do
  export DUALMAP_RUNS=$W30/dualmap_runs/$m
  DUALMAP_METHOD=$m DUALMAP_DATASET=$D bash dualmap_run.sh $VIS
  DUALMAP_METHOD=$m DUALMAP_DATASET=${D}_all bash dualmap_run.sh $VIS
done > $L/dualmap.log 2>&1
step wait_fmf; wait $FMFPID
step score;    $PY obchange_score.py --room "$ROOM" > $L/score.log 2>&1
step done;     tail -25 $L/score.log
