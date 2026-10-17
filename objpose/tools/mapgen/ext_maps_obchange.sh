#!/usr/bin/env bash
# ext_maps_obchange.sh <실행 폴더(six_rt.py --save-pred 의 --out)> <방문 …> — 같은 인식 결과(실행 폴더의 pred/: 처리한 프레임의 마스크 + 이름)를 FM-Fusion 과 DualMap 의 지도 생성 프로그램에 넣어 객체 지도를 만든다.
#   FM-Fusion  원래 통합 프로그램(IntegrateInstanceMap). pred 폴더를 prediction 으로 주고 프레임 간격 1(인식이 처리한 프레임 전부)
#   DualMap    원래 지도 생성 단계. 검출만 저장된 결과로 바꿔 끼운다(run_dualmap_bop.py --ext-pred, DM_EXT_PRED). FastSAM 의 이름 없는 조각은 끈다
#   결과 <실행 폴더>/ext/{fmfusion/<방문>/, dualmap/maps/<tag>/<방문>.json}. 인식은 다시 돌리지 않는다(실시간 실행이 끝난 뒤 처리한 프레임을 순서대로 넣는다).
set -u
RUN=$(realpath $1); shift; cd "$(dirname "$0")/../../.."; REPO=$(pwd); OB=$HOME/DeepLearning/Datasets/ObChange; FD=$HOME/DeepLearning/Datasets/fmfusion_obchange_261005
DPY=$HOME/anaconda3/envs/dualmap/bin/python; TAG=ext_$(echo $RUN | md5sum | cut -c1-8); PN=prediction_$TAG; mkdir -p $RUN/ext/fmfusion
echo $TAG > $RUN/ext/tag.txt
for sc in "$@"; do
  [ -d $RUN/$sc/pred ] || mkdir -p $RUN/$sc/pred
  rm -f $FD/scans/$sc/$PN; ln -s $RUN/$sc/pred $FD/scans/$sc/$PN
  env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib timeout 1800 \
    $HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap --config $REPO/objpose/tools/obchange/fmf_obchange_small.yaml --root $FD/scans/$sc \
    --prediction $PN --frame_gap 1 --output $RUN/ext/fmfusion --verbose 2 > $RUN/ext/fmfusion/$sc.log 2>&1
  rm -f $FD/scans/$sc/$PN
  echo "FM-Fusion $sc $(grep -o 'Save [0-9]* semantic instances' $RUN/ext/fmfusion/$sc.log) / 입력 프레임 $(ls $RUN/$sc/pred | grep -c label)"
done
rm -rf $HOME/DeepLearning/third_party/DualMap/output/$TAG
[ "${SKIP_DUALMAP:-0}" = 1 ] || timeout 7200 $DPY objpose/tools/baselines/dualmap/run_dualmap_bop.py --bop-test $OB/bop/test --table objpose/rt/bop/obchange_objects.json --poses gt --fps 10 --out $RUN/ext/dualmap --tag $TAG --ext-pred $RUN --timeout 1500 --scenes "$@" 2>&1 | grep -v -i warn | cut -c1-160
