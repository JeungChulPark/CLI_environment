#!/bin/bash
# run_anchor_view.sh <cbnu|inlong> — one real-time run with the object-anchored correction (map registered at the
# corrected pose, 0.5 s blend, depth ICP 10 Hz, SLAM pose jumps absorbed over 3 s): hub_anchor + the recorded ORB-SLAM3 stream replayed at 1x (no SLAM host needed).
# Started by objpose/pc/launcher.py (env HTTP_PORT, OUT) like run_one.sh; the hub is exec'd, so the launcher's
# SIGINT reaches it and it stops SAM-6D itself. The replay exits with it.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
D=/home/jucpark/DeepLearning/Datasets/data_slam_converted
case "$1" in
  cbnu)   SESS=$D/260901_cbnu_eightcircle; X=objpose/rt/260901_cbnu/X_slam_sam_260901_cbnu.json; TAU=-0.212
          REC=objpose/output/rep_cbnu_eight_r1_260922_233141 ;;
  inlong) SESS=$D/260915_six_routes/260915_inlong_middle; X=objpose/rt/260915/X_slam_sam_260915.json; TAU=0.0
          REC=objpose/output/one_260915_inlong_middle_orb_f2000_260916_090051 ;;
  outlong) SESS=$D/260915_six_routes/260915_outlong_middle; X=objpose/rt/260915/X_slam_sam_260915.json; TAU=0.0
          REC=objpose/output/one_260915_outlong_middle_orb_f2000_260916_091857 ;;
  *) echo "usage: $0 cbnu|inlong|outlong" >&2; exit 2 ;;
esac
HTTP_PORT="${HTTP_PORT:-8775}"
OUT="${OUT:-$REPO/objpose/output/anchor_view_$1_$(date +%y%m%d_%H%M%S)}"; mkdir -p "$OUT"
# the replay connects once the hub listens (retries), and stops when the hub goes away
$PY -u objpose/tools/obj_anchor/replay_slam_log.py --run "$REC" --connect 127.0.0.1:17011 > "$OUT/replay_slam.log" 2>&1 &
exec $PY -u objpose/pc/hub_anchor.py --anchor-map slamcorr --anchor-set use_rotation=false --anchor-set min_obs=5 \
  --anchor-set sig_obs_t0=0.03 --anchor-set deadband_m=0.03 --anchor-set max_rot_sigma=2.0 --anchor-set blend_s=0.5 \
  --anchor-set loop_blend_s=3 --anchor-set icp_sig_m=0.03 --anchor-set max_rot_step_deg=0.2 --anchor-icp-hz 10 \
  --link manual --no-mac --slam-port 17011 --http-host 0.0.0.0 --http-port "$HTTP_PORT" \
  --sam-session "$SESS/SAM" --slam-session "$SESS/SLAM" --extrinsic "$X" --sam-tau-s "$TAU" \
  --slam orbslam3 --features 2000 --rate 1.0 --out "$OUT"
