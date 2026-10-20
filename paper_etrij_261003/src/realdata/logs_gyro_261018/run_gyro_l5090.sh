#!/bin/bash
# 2026-10-18 (doc 30): object positions on the real recordings that have gyro data (the 260901 cbnu sessions: Xsens
# imu/ on the Mac + rig_260901_gyro.json), RTX 5090 Laptop + Mac ORB-SLAM3 f2000, three runs each:
#   live_261018_<ses>_l5090_origL05_orb       original 0.5, ORB-SLAM3
#   live_261018_<ses>_l5090_hyb3cf_orb        improvement 3 + cluster-first, ORB-SLAM3
#   live_261018_<ses>_l5090_hyb3cf_orbgyro    improvement 3 + cluster-first, ORB-SLAM3 + gyro aid (catalog backend
#                                             orbslam3_imu: slam_stream --gyro rig --imu <ses>/imu)
# plus ref_260901_cbnu_longcircle_ours_l5090 (its earlier reference run is not on this laptop). Same workarounds
# as ../logs_unused_261017/run_unused_l5090.sh.
#   bash run_gyro_l5090.sh [session ...]
#   SUF=_r2 HYB_ONLY=1 bash run_gyro_l5090.sh   repeat of the two hyb3cf runs (names get the suffix)
set -u
cd "$(dirname "$0")"; HERE=$PWD; RD=$(cd .. && pwd); REPO=$(cd ../../../.. && pwd)
source $HOME/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper/env.sh; unset YCBV_OUT YCBV_RUN_LABEL
mkdir -p $HERE/bin; printf '#!/bin/sh\nexec env -u LD_LIBRARY_PATH /usr/bin/ssh "$@"\n' > $HERE/bin/ssh; chmod +x $HERE/bin/ssh
export PATH=$HERE/bin:$PATH
SESSIONS=${@:-260901_cbnu_eightcircle 260901_cbnu_bigeightcircle 260901_cbnu_digut 260901_cbnu_longcircle}
ssh -o BatchMode=yes mac 'pkill caffeinate; (nohup caffeinate -dimsu -t 21600 >/dev/null 2>&1 &)'
run() {   # session backend outname "extra hub args" env...
  ses=$1; be=$2; name=$3; extra=$4; shift 4
  until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do echo "mac unreachable $(date +%T)"; sleep 60; done
  BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('$ses','$be',2000)))" | sed 's/--out [^ ]*//')
  OUT=$REPO/objpose/output/$name; rm -rf $OUT $OUT.log; mkdir -p $OUT
  for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done; sleep 3
  echo "start $name $(date +%T)"
  env "$@" $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT $extra > $OUT.log 2>&1 &
  HP=$!
  for i in $(seq 1 600); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HP 2>/dev/null || break; sleep 3; done
  sleep 3; for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done
  echo "done $name $(date +%T) summary=$(grep -c 'summary written' $OUT.log) $(grep -o 'max correction so far [0-9.]* cm' $OUT.log | tail -1)"
}
REC="--recognizer $RD/live_cores.py"
ORIG="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5"
HYB="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
for ses in $SESSIONS; do
  S=${SUF:-}
  if [ -z "${HYB_ONLY:-}" ]; then
    [ $ses = 260901_cbnu_longcircle ] && run $ses orbslam3 ref_${ses}_ours_l5090 "" X=1
    run $ses orbslam3   live_261018_${ses}_l5090_origL05_orb$S    "$REC"                       $ORIG
  fi
  run $ses orbslam3     live_261018_${ses}_l5090_hyb3cf_orb$S     "$REC --verify-cluster-first" $HYB
  run $ses orbslam3_imu live_261018_${ses}_l5090_hyb3cf_orbgyro$S "$REC --verify-cluster-first" $HYB
done
echo ALL_DONE
