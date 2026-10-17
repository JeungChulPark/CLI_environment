#!/usr/bin/env bash
# run_bop.sh — replay a BOP scene (YCB-V, LM-O, ...) through the live pipeline and record the viewer.
#
# The scene's ground-truth camera poses take the place of the SLAM stream: they are converted to
# a TUM trajectory on the same synthetic clock the images get (objpose/pc/bop_session.py), and
# streamed to the hub with the same wire protocol ORB-SLAM3/hdl_graph_slam use — from the laptop
# over the LAN by default, so the run exercises the real two-host path.
#
#   PC   192.168.240.2  hub.py: BOP replay -> SAM-6D -> fusion -> object memory -> viewer :8765
#   note 192.168.240.1  fake_slam_stream.py: GT camera poses -> TCP 17001 (1x, 30 fps)
#
# Datasets whose table says `"world": "camera"` (LM, LM-O: per-image ground truth, no global
# world frame) get a fixed identity pose stream and the object memory is switched off — there
# the run scores per-frame 6D pose only, which is what the BOP protocol asks of them.
#
#   bash objpose/run_bop.sh                                   # YCB-V test/000048, whole scene, recorded
#   bash objpose/run_bop.sh --scene <...>/lmo/test/000002     # table guessed from the path
#   bash objpose/run_bop.sh --scene <...>/test/000052 --duration-s 60
#   SLAM_HOST=local bash objpose/run_bop.sh                   # stream the poses on this PC instead
#   ALL_OBJECTS=1 bash objpose/run_bop.sh                     # open set: every object in the table
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
PY_REC=/home/jucpark/anaconda3/bin/python          # has websocket-client, for the recorder

YCBV="${YCBV:-/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video}"
SCENE="${SCENE:-$YCBV/test/000048}"
TABLE="${TABLE:-}"
SLAM_HOST="${SLAM_HOST:-laptop240}"                # ssh alias of the pose streamer host, or "local"
PC_HOST="${PC_HOST:-192.168.240.2}"
PORT="${PORT:-17001}"
HTTP_PORT="${HTTP_PORT:-8765}"
DURATION=0
RECORD=1
HUB_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --scene) SCENE="$2"; shift 2 ;;
    --table) TABLE="$2"; shift 2 ;;
    --duration-s) DURATION="$2"; HUB_ARGS+=(--duration-s "$2"); shift 2 ;;
    --no-record) RECORD=0; shift ;;
    *) HUB_ARGS+=("$1"); shift ;;
  esac
done
SC="$(basename "$SCENE")"
[ -d "$SCENE" ] || { echo "no scene: $SCENE" >&2; exit 1; }
if [ -z "$TABLE" ]; then
  case "$SCENE" in
    */lmo/*) TABLE="$REPO/objpose/rt/bop/lmo_objects.json" ;;
    *)       TABLE="$REPO/objpose/rt/bop/ycbv_objects.json" ;;
  esac
fi
read -r DATASET WORLD < <("$PY" -c "
import json,sys; d=json.load(open(sys.argv[1])); print(d['dataset'], d.get('world','gt_camera'))" "$TABLE")

STAMP="$(date +%y%m%d_%H%M%S)"
OUT="${OUT:-$REPO/objpose/output/${DATASET}_${SC}_$STAMP}"
TUM="$REPO/objpose/rt/bop/gt_${DATASET}_${SC}_tum.txt"
mkdir -p "$OUT"

[ -f "$TUM" ] || "$PY" "$REPO/objpose/tools/bop_to_tum.py" --scene "$SCENE" --out "$TUM" || exit 1
echo "[run_bop] dataset=$DATASET scene=$SCENE world=$WORLD out=$OUT slam_host=$SLAM_HOST"

# A scene with no world frame cannot have a world-frame object memory. It also cannot carry an
# estimate forward between frames: the overlay's "interp" boxes assume the camera pose explains
# the motion, and with a fixed identity pose they just pile up stale boxes. So only fresh
# SAM-6D results are drawn there.
MEM_ARGS=()
[ "$WORLD" = camera ] && MEM_ARGS=(--no-memory --overlay-max-age 0.6)

# BOP states the objects to estimate per scene (test_targets_bop19.json), so the recogniser is
# told the scene's target list instead of hunting the whole set. ALL_OBJECTS=1 runs the open set.
OBJ_ARGS=()
if [ "${ALL_OBJECTS:-0}" != 1 ]; then
  mapfile -t TARGETS < <("$PY" - "$SCENE" "$TABLE" <<'PY'
import json, sys
gt = json.load(open(sys.argv[1] + "/scene_gt.json"))
name = {o["obj_id"]: o["name"] for o in json.load(open(sys.argv[2]))["objects"]}
ids = sorted({r["obj_id"] for v in gt.values() for r in v})
print("\n".join(name[i] for i in ids if i in name))
PY
)
  OBJ_ARGS=(--ism-objects "${TARGETS[@]}")
  echo "[run_bop] scene targets: ${TARGETS[*]}"
fi

# ── hub ──────────────────────────────────────────────────────────────────────
"$PY" -u "$REPO/objpose/pc/hub.py" \
  --sam-session "$SCENE" --extrinsic "$REPO/objpose/rt/bop/X_identity.json" \
  --ism-config "configs/yolo_ism_${DATASET}.yaml" "${OBJ_ARGS[@]}" "${MEM_ARGS[@]}" \
  --link manual --pc-host "$PC_HOST" --slam-host "${SLAM_HOST}" --slam-port "$PORT" \
  --http-host 0.0.0.0 --http-port "$HTTP_PORT" \
  --sam-tau-s 0 --rate 1.0 --start-s 0 --exit-when-done \
  --out "$OUT" "${HUB_ARGS[@]}" > "$OUT/hub.log" 2>&1 &
HUB=$!
echo "[run_bop] hub pid=$HUB  log=$OUT/hub.log  viewer=http://$PC_HOST:$HTTP_PORT"

cleanup() {
  kill -INT "$HUB" 2>/dev/null
  [ -n "${STREAM:-}" ] && kill "$STREAM" 2>/dev/null
  wait "$HUB" 2>/dev/null
}
trap cleanup EXIT INT TERM

# the hub loads SAM-6D (~1 min) before it says "start"; the streamer waits, so start it now
if [ "$SLAM_HOST" = local ]; then
  python3 -u "$REPO/objpose/tools/fake_slam_stream.py" --connect "127.0.0.1:$PORT" \
      --tum "$TUM" --session "$DATASET:$SC" > "$OUT/slam_stream.log" 2>&1 &
  STREAM=$!
else
  scp -q "$TUM" "$REPO/objpose/tools/fake_slam_stream.py" "$SLAM_HOST:/tmp/" || {
    echo "[run_bop] scp to $SLAM_HOST failed" >&2; exit 1; }
  ssh "$SLAM_HOST" "python3 -u /tmp/fake_slam_stream.py --connect $PC_HOST:$PORT \
      --tum /tmp/$(basename "$TUM") --session $DATASET:$SC" > "$OUT/slam_stream.log" 2>&1 &
  STREAM=$!
fi
echo "[run_bop] pose streamer started on $SLAM_HOST (pid=$STREAM)"

# ── recording ────────────────────────────────────────────────────────────────
if [ "$RECORD" = 1 ]; then
  # wait until the hub actually starts replaying, so the video does not open on a loading page
  for _ in $(seq 1 240); do grep -q "START t0_ns" "$OUT/hub.log" 2>/dev/null && break; sleep 1; done
  SECS="$DURATION"
  [ "$SECS" = 0 ] && SECS=$(python3 -c "print(len(open('$TUM').read().split(chr(10)))/30.0+15)")
  echo "[run_bop] recording the viewer for ${SECS}s -> $OUT/viewer.mp4"
  "$PY_REC" "$REPO/objpose/tools/record_viewer.py" --url "http://127.0.0.1:$HTTP_PORT" \
      --out "$OUT/viewer.mp4" --seconds "$SECS" >> "$OUT/record.log" 2>&1
fi

wait "$HUB"
echo "[run_bop] done -> $OUT"
