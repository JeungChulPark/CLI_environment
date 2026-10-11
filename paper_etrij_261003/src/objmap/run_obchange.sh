#!/bin/bash
# ObChange-Kitchen, three methods, two map modes (see common.seq_cameras):
#   obchange_kitchen      each visit k = 2..6 mapped alone
#   obchange_kitchen_all  visits 1..k in a row; the map at the end of visit k
# Order: the proposed method first and alone on the GPU (it replays in real time), then the baselines.
set -u
cd "$(dirname "$0")"
PY=$HOME/anaconda3/envs/sam6d/bin/python
L=out/obchange_logs; mkdir -p $L
until [ "$(grep -c '^wrote\|^exists' $L/bags.log)" -ge 6 ]; do sleep 20; done
# 1) proposed: one continuous run with per-visit snapshots, then each visit alone
$PY run_ours.py run --dataset obchange_kitchen_all --scenes 6 > $L/ours_all.log 2>&1
$PY run_ours.py export --dataset obchange_kitchen_all --scenes 6 >> $L/ours_all.log 2>&1
$PY run_ours.py run --dataset obchange_kitchen --scenes 2 3 4 5 6 > $L/ours_visits.log 2>&1
$PY run_ours.py export --dataset obchange_kitchen --scenes 2 3 4 5 6 >> $L/ours_visits.log 2>&1
echo OURS_DONE
