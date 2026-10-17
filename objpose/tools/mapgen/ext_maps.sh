#!/usr/bin/env bash
# ext_maps.sh <실행 폴더> <물체표> <FM-Fusion scans 폴더> <FM-Fusion 설정> <장면 …> — ext_maps_obchange.sh 의 일반형(YCB-Video, YCB-M).
#   같은 인식 결과(실행 폴더의 pred/)를 FM-Fusion(프레임 간격 1) 과 DualMap(--ext-pred, 정답 카메라 자세)의 원래 지도 생성 프로그램에 넣는다.
#   결과 <실행 폴더>/ext/{fmfusion/<장면>/, dualmap/maps/<tag>/<장면>.json, tag.txt}
set -u
RUN=$(realpath $1); TABLE=$(realpath $2); SCANS=$(realpath $3); CFG=$(realpath $4); shift 4; cd "$(dirname "$0")/../../.."; REPO=$(pwd)
DPY=$HOME/anaconda3/envs/dualmap/bin/python; TAG=ext_$(echo $RUN | md5sum | cut -c1-8); PN=prediction_$TAG; mkdir -p $RUN/ext/fmfusion; echo $TAG > $RUN/ext/tag.txt
BOP=$(python3 -c "import json;print(json.load(open('$TABLE'))['scenes_dir'])"); FPS=$(python3 -c "import json;print(json.load(open('$TABLE'))['fps'])")
for sc in "$@"; do
  [ -d $RUN/$sc/pred ] || mkdir -p $RUN/$sc/pred
  rm -f $SCANS/$sc/$PN; ln -s $RUN/$sc/pred $SCANS/$sc/$PN
  env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib timeout 1800 \
    $HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap --config $CFG --root $SCANS/$sc --prediction $PN --frame_gap 1 --output $RUN/ext/fmfusion --verbose 2 > $RUN/ext/fmfusion/$sc.log 2>&1
  rm -f $SCANS/$sc/$PN
  echo "FM-Fusion $sc $(grep -o 'Save [0-9]* semantic instances' $RUN/ext/fmfusion/$sc.log) / 입력 프레임 $(ls $RUN/$sc/pred | grep -c label)"
done
rm -rf $HOME/DeepLearning/third_party/DualMap/output/$TAG
timeout 10800 $DPY objpose/tools/baselines/dualmap/run_dualmap_bop.py ${DM_PROMPTS:+--prompts $DM_PROMPTS} --bop-test $BOP --table $TABLE --poses gt --fps $FPS --out $RUN/ext/dualmap --tag $TAG --ext-pred $RUN --timeout 1500 --scenes "$@" 2>&1 | grep -v -i warn | cut -c1-160
