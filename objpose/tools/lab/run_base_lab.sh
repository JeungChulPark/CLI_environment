#!/usr/bin/env bash
# run_base_lab.sh — 직접 녹화한 8개 세션에 FM-Fusion 과 DualMap 을 실시간 재생으로 돌리고, 두 지도의 객체에 CAD 템플릿 검증으로 이름을 다시 붙인다(ObChange 의 run_base_rt_obchange.sh 와 같은 구성).
#   FM-Fusion  문구 = 실시간 뷰어의 물체 문구(yolo_ism_objects.yaml), 3장마다 한 번 검출하되 검출이 밀리면 최신 프레임만. 지도 설정은 ObChange 때와 같고 카메라 값만 세션에 맞춤
#   DualMap    문구 = 색 없는 일반 이름(plain_prompts_lab.json), 검출 기준 0.05, 초당 5장
# run_lab_combo.sh 가 끝난 뒤(COMBO_DONE) 시작한다. 결과 objpose/output/lab_combo_261010/{fmfusion_rt,dualmap_rt,cad_rt}, 기록 base_log.txt (끝나면 BASE_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/lab_combo_261010; mkdir -p $O; exec >> $O/base_log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; GPY=$HOME/DeepLearning/third_party/gsam_venv/bin/python
ROOT=$HOME/DeepLearning/Datasets/lab_sessions_bop; FD=$HOME/DeepLearning/Datasets/fmfusion_lab_261010; TAG=lab_rt_261010; SES=$(ls $ROOT/test | tr '\n' ' ')
log() { echo "=== $(date '+%F %T') $*"; }
until grep -aq COMBO_DONE $O/log.txt 2>/dev/null; do sleep 20; done
export BENCH_ROOT=$ROOT BENCH_TABLE=$REPO/objpose/rt/bop/lab8_objects.json BENCH_FPS=10 BENCH_SLAM=$O/slam

log "FM-Fusion 입력"
$PY objpose/tools/baselines/fmfusion/make_ycbv_scans.py --out $FD --scenes $SES 2>&1 | grep -v -i warn | cut -c1-120
log "FM-Fusion 검출 (실시간)"
for sc in $SES; do rm -rf $FD/scans/$sc/prediction_gsam_rt; done
timeout 7200 $GPY objpose/tools/baselines/fmfusion/detect_gsam_prompts.py --data $FD/scans --gap 3 --all-objects --realtime 10 --out-name prediction_gsam_rt --ism-config $REPO/sam6d_realtime/configs/yolo_ism_objects.yaml --scenes $SES 2>&1 | grep -E "^26[0-9]+_|Traceback|Error" | cut -c1-130
log "FM-Fusion 지도"
mkdir -p $O/fmfusion_rt
for sc in $SES; do
  python3 - "$ROOT/test/$sc" "$O/fmfusion_rt/$sc.yaml" <<'P'
import json, sys
c = json.load(open(sys.argv[1] + "/scene_camera.json")); K = c[sorted(c, key=int)[0]]["cam_K"]; rep = {"camera_fx": K[0], "camera_fy": K[4], "camera_cx": K[2], "camera_cy": K[5]}
t = open("objpose/tools/obchange/fmf_obchange_small.yaml").read().splitlines()
open(sys.argv[2], "w").write("\n".join(f"{l.split(':')[0]}: {rep[l.split(':')[0]]}" if l.split(":")[0] in rep else l for l in t) + "\n")
P
  env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib timeout 2400 \
    $HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap --config $O/fmfusion_rt/$sc.yaml --root $FD/scans/$sc \
    --prediction prediction_gsam_rt --frame_gap 3 --output $O/fmfusion_rt --verbose 2 > $O/fmfusion_rt/$sc.log 2>&1
  echo "$sc $(grep -o 'Save [0-9]* semantic instances' $O/fmfusion_rt/$sc.log) / 검출 프레임 $(python3 -c "import json;j=json.load(open('$FD/scans/$sc/prediction_gsam_rt/timing.json'));print(j['frames'],'/',j['offered'])")"
done

log "DualMap (실시간)"
DM_YOLO_CONF=0.05 timeout 9000 $DPY objpose/tools/baselines/dualmap/run_dualmap_bop.py --bop-test $ROOT/test --table objpose/rt/bop/lab8_objects.json --poses $O/slam --fps 10 --stride 2 \
  --out $O/dualmap_rt --tag $TAG --prompts $REPO/objpose/tools/lab/plain_prompts_lab.json --timeout 1500 --realtime --scenes $SES 2>&1 | grep -v -i warn | cut -c1-160
for sc in $SES; do echo "$sc $(grep -a -o 'REALTIME processed.*' $O/dualmap_rt/${TAG}_in/$sc.log | tail -1)"; done

log "CAD 이름 (실시간)"
unset BENCH_ROOT BENCH_TABLE BENCH_FPS BENCH_SLAM
timeout 9000 $PY objpose/tools/analysis/cad_relabel.py --data lab --out $O/cad_rt --gap FM-Fusion=3 DualMap=2 --poses $O/slam --fmf-dir $O/fmfusion_rt --dualmap-maps $O/dualmap_rt/maps/$TAG --scenes $SES 2>&1 | grep -E "^26[0-9]+_|Traceback|Error" -A3 | cut -c1-260
log "BASE_DONE"
