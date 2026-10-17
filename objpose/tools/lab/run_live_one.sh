#!/bin/bash
# run_live_one.sh <세션> <구성 1~6> <out> — 실제 환경으로 한 번 돌린다: 노트북(ssh samsung)에서 ORB-SLAM3 실시간, 이 PC 에서 인식(live_six.py) + 객체 지도. 끝나면 저절로 끝난다.
#   260901_cbnu_*            인식용 · SLAM용 두 카메라, 리그 RT 와 시계는 run_cbnu_samsung.sh 와 같음
#   260826_etri_*circle_dark 두 카메라, 기본 리그 RT(objpose/rt/X_slam_sam.json)
#   260826_etri_*Object      카메라 하나(SLAM용 녹화만 있음) → 같은 영상을 SLAM 과 인식에 함께 쓴다(항등 변환)
set -u
NAME=${1:?}; MODE=${2:?}; OUT=${3:?}; REPO=/home/jucpark/DeepLearning/CLI_environment_server; cd $REPO; PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
D=/home/jucpark/DeepLearning/Datasets/data_slam_converted/$NAME; RDIR=/home/user/DeepLearning/Dataset/$NAME; RT=$REPO/objpose/rt
case "$NAME" in
  260901_cbnu_eightcircle) SAM=$D/SAM; EXTRA="--extrinsic $RT/260901_cbnu/X_slam_sam_260901_cbnu_eight.json --sam-tau-s -0.212" ;;
  260901_cbnu_*)           SAM=$D/SAM; EXTRA="--extrinsic $RT/260901_cbnu/X_slam_sam_260901_cbnu.json --sam-tau-s 0" ;;
  260826_etri_*Object)     SAM=$D/SLAM; EXTRA="--extrinsic $RT/single_cam/X_identity.json --sam-tau-s 0" ;;
  260826_etri_*)           SAM=$D/SAM; EXTRA="--extrinsic $RT/X_slam_sam.json" ;;
  *) echo "unknown session $NAME"; exit 2 ;;
esac
export PYTHONPATH=$REPO/objpose/tools/shims_nvml${PYTHONPATH:+:$PYTHONPATH}     # 드라이버 버전 불일치 동안의 우회(재부팅 뒤에는 없어도 됨)
export OBJPOSE_RECOGNIZER=$REPO/objpose/tools/lab/live_six.py LIVE_SIX=$MODE
ssh -o BatchMode=yes -o ConnectTimeout=8 samsung "pkill -f slam_stream; pkill -f run_slam.sh" >/dev/null 2>&1; sleep 1
rm -rf $OUT; mkdir -p $OUT
timeout 900 $PY -u objpose/pc/hub.py --slam orbslam3 --slam-port 17001 --features 3000 $EXTRA --mac-slam-session $RDIR/SLAM \
  --link direct --slam-host 192.168.2.1 --mac-host samsung --pc-host 192.168.2.2 --http-host 127.0.0.1 --http-port 8791 --sam-session $SAM --slam-session $D/SLAM --rate 1.0 --out $OUT --exit-when-done > $OUT.log 2>&1
echo "exit=$? $(grep -a -o 'fed SAM-6D [0-9]* frames' $OUT.log | tail -1) $(grep -a -o "SLAM end: .*" $OUT.log | tail -1 | cut -c1-70) lm=$(python3 -c "import json;print(json.load(open('$OUT/summary.json'))['memory']['landmarks'])" 2>/dev/null)"
