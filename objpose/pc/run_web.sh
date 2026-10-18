#!/bin/bash
# run_web.sh — the viewer (http://<this PC>:8765) on a real recording, with the defaults of 2026-10-18:
#   recogniser  improvement 3 + cluster-first (paper_etrij_261003/src/realdata/live_cores.py, hybrid, ViT-S)
#   SLAM (Mac)  catalog.DEFAULT_BACKENDS: modified ORB-SLAM3 + gyro aid where the Mac has <dataset>/imu,
#               else modified ORB-SLAM3 (paper_etrij_261003 doc 30)
# Other recordings and backends are then picked in the page; the recogniser carries over (hub.exec_restart).
# Output: objpose/output/live_<dataset>_<backend>_web<time>.
#   bash objpose/pc/run_web.sh [dataset] [backend]     (default 260901_cbnu_eightcircle, its default backend)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
source $HOME/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper/env.sh; unset YCBV_OUT YCBV_RUN_LABEL
# conda's LD_LIBRARY_PATH breaks /usr/bin/ssh's OpenSSL on this laptop
B=${XDG_RUNTIME_DIR:-/tmp}/objpose_bin; mkdir -p $B
printf '#!/bin/sh\nexec env -u LD_LIBRARY_PATH /usr/bin/ssh "$@"\n' > $B/ssh; chmod +x $B/ssh; export PATH=$B:$PATH
DS=${1:-260901_cbnu_eightcircle}
BE=${2:-$(cd $HERE && $PY -c "import catalog as c; print(c.default_backend('$DS', c.mac_datasets()) or '')")}
[ -n "$BE" ] || { echo "$DS: neither default backend can run (see the viewer's tab reasons)" >&2; exit 1; }
ARGS=$(cd $HERE && $PY -c "import catalog as c, time; print(' '.join(c.hub_args('$DS', '$BE', run_tag=time.strftime('_web%y%m%d%H%M%S'))))")
echo "== $DS / $BE"
export OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5
exec $PY -u $HERE/hub.py $ARGS --compare-glob "live_${DS}_*" \
     --recognizer $REPO/paper_etrij_261003/src/realdata/live_cores.py --verify-cluster-first
