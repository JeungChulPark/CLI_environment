#!/usr/bin/env bash
# run_slam_lab.sh — extract_lab_bop.py 가 꺼낸 세션마다 이 PC 에서 ORB-SLAM3(RGB-D, 3000 특징)를 돌려 카메라 자세를 만든다. 인식에 쓰는 것과 같은 카메라 영상(10 fps)을 쓴다.
#   결과 <out>/<세션>_frames.txt (시각 x y z qx qy qz qw, 카메라→월드), <세션>.yaml (그 세션 카메라의 내부 파라미터), <세션>.log
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); ROOT=${LAB_ROOT:-$HOME/DeepLearning/Datasets/lab_sessions_bop}; OUT=${1:-$REPO/objpose/output/lab_combo_261010/slam}; mkdir -p $OUT
BIN=$REPO/objpose/output/obchange_261005/ycbv_rgbd; VOC=$HOME/DeepLearning/CLI_environment_server/orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt
for d in $ROOT/test/*/; do s=$(basename $d); [ -s $OUT/${s}_frames.txt ] && continue; [ -f $d/scene_camera.json ] || continue
  python3 - "$d" "$OUT/$s.yaml" <<'P'
import json, sys
c = json.load(open(sys.argv[1] + "/scene_camera.json")); K = c[sorted(c, key=int)[0]]["cam_K"]
t = open("objpose/tools/obchange/orb_obchange_f3000.yaml").read().splitlines(); rep = {"Camera1.fx": K[0], "Camera1.fy": K[4], "Camera1.cx": K[2], "Camera1.cy": K[5], "Stereo.b": 0.05}
open(sys.argv[2], "w").write("\n".join(f"{l.split(':')[0]}: {rep[l.split(':')[0]]}" if l.split(":")[0] in rep else l for l in t) + "\n")
P
  echo $s
done | xargs -r -P 4 -I{} bash -c "n=\$(ls $ROOT/test/{}/rgb | wc -l); OMP_NUM_THREADS=2 $BIN $VOC $OUT/{}.yaml $ROOT/test/{} \$n $OUT/{} 10 > $OUT/{}.log 2>&1"
for d in $ROOT/test/*/; do s=$(basename $d); echo "$s $(wc -l < $OUT/${s}_frames.txt 2>/dev/null)/$(ls $d/rgb | wc -l) tracked"; done
