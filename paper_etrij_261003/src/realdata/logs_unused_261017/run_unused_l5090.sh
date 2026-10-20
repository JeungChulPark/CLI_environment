#!/bin/bash
# 2026-10-17 (doc 27): the real recordings in ~/DeepLearning/Datasets that docs 00-26 never ran, on the
# RTX 5090 Laptop + Mac ORB-SLAM3 f2000, three runs each:
#   ref_<ses>_ours_l5090                  deployed recogniser (the reference positions, as ref_260910_object_ours)
#   live_261017_<ses>_l5090_origL05       original 0.5 (doc 22)
#   live_261017_<ses>_l5090_hyb3cf        improvement 3 + cluster-first (doc 24)
# This laptop reads its data through ~/DeepLearning/Dataset (symlinks into Datasets/data_slam_converted) and
# needs two workarounds: env.sh for the checkpoint/template paths, and an ssh without LD_LIBRARY_PATH (conda cv2
# puts its own OpenSSL first, and the hub's ssh then fails with "OpenSSL version mismatch").
#   bash run_unused_l5090.sh [session ...]
set -u
cd "$(dirname "$0")"; HERE=$PWD; RD=$(cd .. && pwd); REPO=$(cd ../../../.. && pwd)
source $HOME/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper/env.sh; unset YCBV_OUT YCBV_RUN_LABEL
mkdir -p $HERE/bin; printf '#!/bin/sh\nexec env -u LD_LIBRARY_PATH /usr/bin/ssh "$@"\n' > $HERE/bin/ssh; chmod +x $HERE/bin/ssh
export PATH=$HERE/bin:$PATH
SESSIONS=${@:-260901_cbnu_digut 260915_inlong_fast 260915_inlong_middle 260915_outlong_fast 260915_outlong_middle
  260915_longcircle 260915_height 260915_dense 260915_distance 260826_etri_longcircle_dark 260915_eight_fast 260915_eight_middle}
ssh -o BatchMode=yes mac 'pgrep -q caffeinate || (nohup caffeinate -dimsu -t 14400 >/dev/null 2>&1 &)'
run() {   # session outname "extra hub args" env...
  ses=$1; name=$2; extra=$3; shift 3
  until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do echo "mac unreachable $(date +%T)"; sleep 60; done
  BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('$ses','orbslam3',2000)))" | sed 's/--out [^ ]*//')
  OUT=$REPO/objpose/output/$name; rm -rf $OUT $OUT.log; mkdir -p $OUT
  for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done; sleep 3
  echo "start $name $(date +%T)"
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $extra > $OUT.log 2>&1 &
  HP=$!
  for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HP 2>/dev/null || break; sleep 3; done
  sleep 3; for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done
  echo "done $name $(date +%T) summary=$(grep -c 'summary written' $OUT.log) $(grep -o 'max correction so far [0-9.]* cm' $OUT.log | tail -1)"
}
REC="--recognizer $RD/live_cores.py"
for ses in $SESSIONS; do
  run $ses ref_${ses}_ours_l5090 "" X=1
  run $ses live_261017_${ses}_l5090_origL05 "$REC" OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5
  run $ses live_261017_${ses}_l5090_hyb3cf "$REC --verify-cluster-first" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
done
echo ALL_DONE
