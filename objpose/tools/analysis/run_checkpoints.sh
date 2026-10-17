#!/usr/bin/env bash
# run_checkpoints.sh — YCB-Video 에서 FM-Fusion · DualMap 을 영상의 앞 10 % ~ 90 % 까지만 넣어 중간 시점의 지도를 만든다(100 % 는 기존 결과).
# 결과: objpose/output/checkpoints_261006/fmfusion/<scene>_fNN/ , objpose/output/checkpoints_261006/dualmap/maps/plain_c05_fNN/<scene>.json
cd "$(dirname "$0")/../../.."
V=objpose/output/fmf_cmp_260928; C=$(pwd)/objpose/output/checkpoints_261006; D=$HOME/DeepLearning/Datasets/fmfusion_ycbv_260928/scans; TR=$HOME/DeepLearning/Datasets/fmfusion_ycbv_260928/trunc
P=$(pwd)/objpose/output/dualmap_plain_261005; BIN=$HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap; CFG=$(pwd)/objpose/tools/baselines/fmfusion/ycbv_small.yaml
mkdir -p $C/fmfusion $TR; SCENES=$(ls $V/maps/dualmap | sed 's/.json//')
echo "=== $(date '+%T') FM-Fusion"
for s in $SCENES; do N=$(ls $D/$s/color | wc -l)
  for pp in 10 20 30 40 50 60 70 80 90; do K=$((N * pp / 100)); R=$TR/${s}_f$pp
    [ -f $C/fmfusion/${s}_f$pp/instance_info.txt ] && continue
    rm -rf $R; mkdir -p $R/color $R/depth $R/pose $R/prediction_gsam
    python3 - "$D/$s" "$R" "$K" <<'PY'
import os, sys, glob
src, dst, K = sys.argv[1], sys.argv[2], int(sys.argv[3])
for sub in ("color", "depth", "pose", "prediction_gsam"):
    for f in os.listdir(f"{src}/{sub}"):
        if not f.startswith("frame-") or int(f.split("-")[1][:6]) < K: os.symlink(os.path.realpath(f"{src}/{sub}/{f}"), f"{dst}/{sub}/{f}")   # 프레임 번호가 없는 파일은 그대로 넣는다
PY
    timeout 600 env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib "$BIN" --config "$CFG" --root "$R" --prediction prediction_gsam --frame_gap 10 --output $C/fmfusion --verbose 2 > $C/fmfusion/${s}_f$pp.log 2>&1
    echo "$s f$pp $(grep -o 'Save [0-9]* semantic instances' $C/fmfusion/${s}_f$pp.log)"
  done; done
echo "=== $(date '+%T') DualMap"
for pp in 10 20 30 40 50 60 70 80 90; do
  DM_YOLO_CONF=0.05 timeout 2400 $HOME/anaconda3/envs/dualmap/bin/python objpose/tools/baselines/dualmap/run_dualmap_bop.py --bop-test $HOME/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test --table objpose/rt/bop/ycbv_objects.json \
    --poses $V/slam --fps 30 --stride 10 --out $C/dualmap --tag plain_c05_f$pp --prompts $P/plain_prompts_ycb21.json --max-frac 0.$pp --timeout 600 --scenes $SCENES 2>&1 | grep -v -i warn | tail -12 | cut -c1-90
  echo "--- $(date '+%T') f$pp done"
done
echo "=== $(date '+%T') CKPT_DONE"
