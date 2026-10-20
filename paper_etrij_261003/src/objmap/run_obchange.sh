#!/bin/bash
# ObChange, three methods, two map modes (see common.seq_cameras):
#   obchange_<room>      each visit k = 2..LAST mapped alone
#   obchange_<room>_all  visits 1..k in a row; the map at the end of visit k
# Order: the proposed method first and alone on the GPU (it replays in real time), then the baselines.
# Usage: bash run_obchange.sh [ROOM=Kitchen] [LAST=6]     (BigRoom 7, the other rooms 6)
set -u
cd "$(dirname "$0")"
ROOM=${1:-Kitchen}; LAST=${2:-6}
source ./obchange_env.sh
PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}
D=obchange_$R
VIS=$(seq -s ' ' 2 "$LAST")
mkdir -p $L
# seq_bags.py output: one bag per visit (2..LAST) + the continuous one
until [ "$(grep -c '^wrote\|^exists' $L/bags.log)" -ge "$LAST" ]; do sleep 20; done
# 1) proposed: one continuous run with per-visit snapshots, then each visit alone
$PY run_ours.py run --dataset ${D}_all --scenes $LAST > $L/ours_all.log 2>&1
$PY run_ours.py export --dataset ${D}_all --scenes $LAST >> $L/ours_all.log 2>&1
$PY run_ours.py run --dataset $D --scenes $VIS > $L/ours_visits.log 2>&1
$PY run_ours.py export --dataset $D --scenes $VIS >> $L/ours_visits.log 2>&1
echo OURS_DONE
