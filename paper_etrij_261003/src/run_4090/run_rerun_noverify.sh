#!/bin/bash
# g13_noverify 실제 영상 재실행 (첫 실행은 ORB-SLAM3 루프 보정 3.0 m 로 지도가 깨짐) — brief 체인 완료 후, 최대 2회 시도
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/brief_chain.log; O=$R/objpose/output
say(){ echo "[$(date +%T)] $*" >> $LOG; }
until grep -q "ALL DONE" $LOG; do sleep 30; done
source $W/env.sh; export OBJPOSE_REPO=$R
NAS=/mnt/synology/Dataset/data_slam_converted/260901_cbnu_bigeightcircle; SET=$R/objpose/mac_slam/settings/orbslam3_260901_slam_nf2000.yaml
G13=$R/sam6d_realtime/configs/yolo_ism_objects_g13.yaml
mv $O/real_260901_g13_noverify $O/real_260901_g13_noverify_slamfail1; mv $O/real_260901_g13_noverify.log $O/real_260901_g13_noverify_slamfail1.log
for attempt in 1 2; do
  OUT=$O/real_260901_g13_noverify; mkdir -p $OUT; say "rerun g13_noverify attempt $attempt"
  pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream; sleep 3
  $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $R/objpose/rt/260901/X_slam_sam_260901.json \
     --sam-session $NAS/SAM --slam-session $NAS/SLAM --mac-slam-session $NAS/SLAM --out $OUT --features 2000 --no-mac --ism-config $G13 --no-verify > $OUT.log 2>&1 &
  HUB=$!
  for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
  $HOME/objpose/run_slam.sh --slam orbslam3 --session $NAS/SLAM --settings $SET --features 2000 --time-source frame --mode live --connect 127.0.0.1:17001 --rate 1.0 > $OUT/pc_slam.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; kill $HUB 2>/dev/null; pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream
  corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]* cm"); say "done g13_noverify attempt $attempt, max loop correction $corr"
  big=$(echo "${corr%% cm}" | awk '{print ($1>60)?1:0}'); [ "$big" = "1" ] && { mv $OUT $O/real_260901_g13_noverify_slamfail$((attempt+1)); mv $OUT.log $O/real_260901_g13_noverify_slamfail$((attempt+1)).log; continue; }
  break
done
cd $S/realdata; $PY eval_real.py --runs real_260901_g13 real_260901_g13_noverify real_260901_origL05 --out results_real_4090.json > $W/logs/eval_real_4090.log 2>&1 || say "FAIL eval real (rerun)"
say "RERUN DONE"
