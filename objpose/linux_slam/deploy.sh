#!/bin/bash
# deploy.sh — run on the PC (192.168.20.5). Assembles the SLAM-host bundle and puts it at
# <host>:~/objpose (+ the raw SLAM session at <host>:~/Dataset/<session>/SLAM), then builds there.
#   bash objpose/linux_slam/deploy.sh               # HOST=slam3 (ssh alias, 192.168.20.3:10022)
#   HOST=local bash objpose/linux_slam/deploy.sh    # stage + build in ~/objpose on THIS PC (self-test)
#   NO_BUILD=1 / NO_DATA=1                          # skip the remote build / the 12 GB dataset copy
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${HOST:-slam3}"
SESSION="${SESSION:-260826_etri_eightcircle_dark}"
RAW_SLAM="${RAW_SLAM:-$HOME/DeepLearning/dataset/$SESSION/SLAM}"        # RAW SDK session (db3 + associations)
PANGOLIN_SRC="${PANGOLIN_SRC:-$HOME/DeepLearning/old/CLI_environment/orbslam_ws/standalone/third/Pangolin}"
ORB_SRC="$REPO/orbslam_ws/src/ORB_SLAM3"
VOCAB="${VOCAB:-$HOME/DeepLearning/sam6d-object-memory/orbslam_ws/src/ORB_SLAM3/Vocabulary/ORBvoc.txt}"

for d in "$RAW_SLAM" "$PANGOLIN_SRC" "$ORB_SRC"; do [ -d "$d" ] || { echo "missing: $d" >&2; exit 1; }; done
[ -f "$VOCAB" ] || { echo "missing: $VOCAB" >&2; exit 1; }
[ -f "$RAW_SLAM/rgbd_timestamp_associations.json" ] || { echo "not a raw SDK session: $RAW_SLAM" >&2; exit 1; }

if [ "$HOST" = local ]; then
  DST="$HOME/objpose"; DATA="$HOME/Dataset/$SESSION/SLAM"; RS=(rsync -a); RUN=(bash)
else
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$HOST" true || { echo "ssh $HOST: key not accepted (register ~/.ssh/id_ed25519.pub on the host first)" >&2; exit 1; }
  DST="$HOST:objpose"; DATA="$HOST:Dataset/$SESSION/SLAM"; RS=(rsync -a --info=progress2 -e ssh); RUN=(ssh "$HOST" bash)
  ssh "$HOST" "mkdir -p objpose/deps Dataset/$SESSION/SLAM"
fi
[ "$HOST" = local ] && mkdir -p "$DST/deps" "$DATA"

echo "== bundle -> $DST =="
"${RS[@]}" --delete --exclude build --exclude __pycache__ "$REPO/objpose/mac_slam" "$REPO/objpose/linux_slam" "$DST/"
"${RS[@]}" --exclude lib --exclude build --exclude 'build_*' --exclude Vocabulary --exclude '*.log' --exclude '*.pdf' \
           --exclude '*.bak*' "$ORB_SRC/" "$DST/ORB_SLAM3_src/"
"${RS[@]}" --exclude build "$PANGOLIN_SRC/" "$DST/deps/Pangolin/"
"${RS[@]}" "$VOCAB" "$DST/Vocabulary/"
if [ -z "${NO_DATA:-}" ]; then
  echo "== raw SLAM session ($(du -sh "$RAW_SLAM" | cut -f1)) -> $DATA =="
  "${RS[@]}" "$RAW_SLAM/" "$DATA/"
fi
if [ -z "${NO_BUILD:-}" ]; then
  echo "== build on $HOST =="
  # remote: ssh joins its arguments with spaces, so pass ONE word: the script path (never "bash -c bash path")
  "${RUN[@]}" '~/objpose/linux_slam/build_host.sh' </dev/null
fi
