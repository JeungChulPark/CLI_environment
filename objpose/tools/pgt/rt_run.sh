#!/usr/bin/env bash
# fresh real-time run on the recorded 260915_inlong_middle session: laptop ORB-SLAM3 (wired) + this PC's SAM-6D + Object Pose Map,
# then the re-association replay, the video and the numbers for the slide
set -eu
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
SES=260915_inlong_middle
SAM=/home/jucpark/DeepLearning/Datasets/data_slam_converted/260915_six_routes/$SES/SAM
X=objpose/rt/260915/X_slam_sam_260915.json
D=objpose/output/estcmp_260927/rt_run; mkdir -p $D
OUT=objpose/output/live_${SES}_f3000_$(date +%y%m%d_%H%M%S)
echo "$OUT" > $D/run_dir.txt
$PY -u objpose/pc/hub.py --slam orbslam3 --link direct --slam-port 17001 --extrinsic $X \
  --mac-runner "~/objpose/run_slam.sh" --mac-slam-session /home/user/DeepLearning/Dataset/$SES/SLAM --mac-host samsung \
  --slam-host 192.168.2.1 --pc-host 192.168.2.2 --features 3000 --sam-session $SAM \
  --http-host 0.0.0.0 --http-port 8775 --rate 1.0 --exit-when-done --out $OUT > $D/hub.log 2>&1
$PY objpose/tools/obj_anchor/eval_reassoc_real.py $OUT --x $X --session $SAM --tau-s 0 --out $D/eval > $D/eval.log 2>&1
$PY objpose/tools/obj_anchor/render_reassoc_real.py $D/eval --out $D/rt_inlong.mp4 --speed 2 > $D/render.log 2>&1
$PY objpose/tools/pgt/rt_stats.py $OUT $D > $D/stats.log 2>&1
echo RT DONE
