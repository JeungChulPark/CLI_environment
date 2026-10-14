#!/bin/bash
# real-data live run of the improved recogniser (g13) with pose verification limited to the 50 best
# geometry candidates; same as the earlier real_260901_g13 run otherwise (Mac ORB-SLAM3 f2000 + this PC)
set -u
cd "$(dirname "$0")"; HERE=$PWD; PY=$HOME/anaconda3/envs/sam6d/bin/python
REPO=$(cd ../../.. && pwd); SR=$REPO/sam6d_realtime
until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do sleep 60; done
echo "mac reachable $(date +%T)"
BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260901_cbnu_bigeightcircle','orbslam3',2000)))" | sed 's/--out [^ ]*//')
tag=g13_k50; OUT=$REPO/objpose/output/real_260901_$tag; mkdir -p $OUT
pkill -f "objpose/pc/hub.py"; sleep 3
$PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT --ism-config $SR/configs/yolo_ism_objects_g13.yaml --verify-candidates 50 > $OUT.log 2>&1 &
for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
sleep 3; pkill -f "objpose/pc/hub.py"; echo "done $tag $(grep -c 'summary written' $OUT.log)"
$PY eval_real.py --runs real_260901_ours real_260901_g13 real_260901_g13_k50 --out results_real_k50.json > eval_k50.log 2>&1
echo REAL_K50_DONE
