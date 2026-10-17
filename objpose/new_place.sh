#!/bin/bash
# new_place.sh — arriving at a new place: build the global map while driving by hand, find how the lidar sits on the
# robot from that drive, re-register the lidar to the object-pose camera, then switch to driving on the saved map with
# object mapping. Run on this PC; the laptop on the robot is reached over ssh.
#
#   bash objpose/new_place.sh start <map name>   laptop: real_up.sh (hdl_graph_slam mapping) + mount_yaw.py; PC: the viewer
#        ... drive the robot by hand through the whole place; include a few STRAIGHT runs of 1 m or more ...
#   bash objpose/new_place.sh status             straight runs seen so far and the lidar turn they give
#        ... stop the robot where the camera sees structure 1-4 m away (walls, shelves; not an empty hall) ...
#   bash objpose/new_place.sh finish             save the map -> stop -> capture lidar + camera standing still ->
#                                                calibrate -> write nav/real/mount.env -> real_loc_up.sh <map> + viewer
#   bash objpose/new_place.sh calib              only the standing-still calibration, with everything stopped
#
# What is found, and from what:
#   lidar yaw on the robot (LIDAR_YAW)   straight driving: the wheels say "straight", so the course of the lidar pose
#                                        must equal its heading (nav/real/mount_yaw.py)
#   lidar -> camera (X_LIDAR_SAM)        lidar points on the camera's depth image, robot standing (rt/calib_mount.py),
#                                        started from the turn the drive gave
#   clocks (laptop - PC)                 measured by run_live.sh at every start, nothing to do here
# Decision at "finish": camera and drive agree within AGREE_DEG -> LIDAR_YAW from the camera fit (finer, and it keeps
# the camera where it was on the robot); they disagree -> the camera itself was moved, LIDAR_YAW from the drive; no
# straight runs -> camera only, with a warning. A bad fit changes nothing.
# env: LAPTOP_SSH (samsung), VIEWER (1), AGREE_DEG (3), DRY (1 = "calib" writes no mount.env)
set -uo pipefail
REPO=/home/jucpark/DeepLearning/CLI_environment_server; O=$REPO/objpose; PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
L="${LAPTOP_SSH:-samsung}"; R='~/objpose/nav/real'; STATE=$O/nav/real/.new_place; MOUNT=$O/nav/real/mount.env
AGREE_DEG="${AGREE_DEG:-3}"
say() { echo "[new_place] $*"; }
die() { echo "[new_place] 중단: $*" >&2; exit 1; }
getv() { sed -n "s/^$1=//p" "$MOUNT"; }
rssh() { ssh -o ConnectTimeout=8 "$L" "$@"; }

deploy() {    # the laptop runs its own copies
  scp -q "$O"/nav/real/{mount_yaw.py,mount_yaw_up.sh,vlp_dump.py,real_up.sh,real_loc_up.sh,mount.env,nav_bridge.py,road.py} "$L:objpose/nav/real/" || die "노트북으로 파일 복사 실패"
}

capture_and_fit() {   # $1 = output dir, $2 = hint in deg or ""   -> $1/X_lidar_sam.json, prints the fit JSON line
  local dir=$1 hint=$2; mkdir -p "$dir"
  rssh 'pgrep -f "[v]lp16_publisher|[m]ap_localizer|[g]raph_slam_node" >/dev/null' && die "노트북에서 LiDAR를 쓰는 스택이 아직 돌고 있습니다 (real_down.sh 먼저)"
  say "LiDAR 수집 (정지 상태, 4초)"
  # the lidar needs about half a minute after its adapter comes up before packets arrive
  for i in 1 2 3 4; do rssh "python3 $R/vlp_dump.py 3000 /tmp/new_place_vlp.pkl 2>/dev/null" && break
    [ $i = 4 ] && die "LiDAR 패킷이 오지 않습니다 (USB 랜 어댑터·LiDAR 전원 확인)"; say "LiDAR 패킷 대기 ($i/3)"; sleep 10; done
  scp -q "$L:/tmp/new_place_vlp.pkl" "$dir/vlp_static.pkl" || die "LiDAR 수집 파일 복사 실패"
  say "카메라 수집 (깊이 45장 중앙값)"
  ( set +u; source /home/jucpark/anaconda3/etc/profile.d/conda.sh && conda activate realsense && export ROS_DOMAIN_ID=72 &&
    ( setsid ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true initial_reset:=false \
        rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 > "$dir/realsense.log" 2>&1 & echo $! > "$dir/cam.pid" ) &&
    sleep 8 && python "$O/rt/261013_husky/cam_cap.py" "$dir/cam_static.npz" 2>&1 | tail -1 )
  [ -f "$dir/cam.pid" ] && { kill -INT -- -"$(cat "$dir/cam.pid")" 2>/dev/null; sleep 3; pkill -INT -x realsense2_camera_node 2>/dev/null; rm -f "$dir/cam.pid"; }
  [ -s "$dir/cam_static.npz" ] || die "카메라 수집 실패 ($dir/realsense.log)"
  say "LiDAR -> 카메라 맞춤 (1~2분)"
  "$PY" "$O/rt/calib_mount.py" --cam "$dir/cam_static.npz" --vlp "$dir/vlp_static.pkl" --ref "$O/$(getv X_LIDAR_SAM)" \
        --out "$dir/X_lidar_sam.json" --yaw-ref "$(getv LIDAR_YAW)" ${hint:+--hint-deg "$hint"} --note "$(basename "$dir")" 2>/dev/null | tail -1 > "$dir/fit.json"
  [ -s "$dir/fit.json" ] || die "맞춤 계산 실패"
}

decide_and_write() {  # $1 = calibration dir, $2 = drive JSON file or ""; writes mount.env unless DRY=1. Prints the decision.
  "$PY" - "$1" "${2:-}" "$MOUNT" "$AGREE_DEG" "${DRY:-0}" "$O" <<'PYEOF'
import json, math, os, shutil, sys, time
d, drive_f, mount, agree, dry, O = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), sys.argv[5] == "1", sys.argv[6]
fit = json.load(open(d + "/fit.json")); cur = dict(l.strip().split("=", 1) for l in open(mount) if "=" in l and not l.startswith("#"))
yaw_old = float(cur["LIDAR_YAW"]); wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi; deg = math.degrees
drive = json.load(open(drive_f)) if drive_f and os.path.exists(drive_f) else None
print(f"  사용 중: LIDAR_YAW {yaw_old:.4f} rad ({deg(yaw_old):.1f} deg), {cur['X_LIDAR_SAM']}")
print(f"  카메라 맞춤: LiDAR가 {fit['turn_free_fit_deg']:+.1f} deg 돌아감, 깊이 면 위 점 {fit['share_within_3cm']*100:.0f}%, 중앙값 {fit['residual_median_cm']} cm"
      f" -> LIDAR_YAW {fit['lidar_yaw_from_camera']:.4f} ({deg(fit['lidar_yaw_from_camera']):.1f} deg) | {'양호' if fit['ok'] else '불량'}")
if drive and drive.get("n"):
    print(f"  주행 추정: 직진 구간 {drive['n']}개, LiDAR가 {drive['d_deg']:+.1f} deg 돌아감 (흩어짐 {drive['mad_deg']} deg)"
          f" -> LIDAR_YAW {drive['lidar_yaw_estimated']:.4f} ({deg(drive['lidar_yaw_estimated']):.1f} deg) | {'양호' if drive['ok'] else '구간 부족·불일치'}")
else:
    print("  주행 추정: 직진 구간 없음")
if not fit["ok"]:
    print("결정: 카메라 맞춤이 불량이라 아무것도 바꾸지 않습니다. 카메라가 1~4 m 거리의 벽·선반을 보는 곳에 세우고 다시 하세요 (new_place.sh calib)"); sys.exit(3)
drive_ok = bool(drive and drive.get("ok"))
if drive_ok and abs(deg(wrap(drive["lidar_yaw_estimated"] - fit["lidar_yaw_from_camera"]))) > agree:
    yaw, why = drive["lidar_yaw_estimated"], (f"주행과 카메라가 {abs(deg(wrap(drive['lidar_yaw_estimated'] - fit['lidar_yaw_from_camera']))):.1f} deg 다릅니다 -> "
                                             "카메라 자체가 로봇 위에서 움직인 것으로 보고 주행 값을 씁니다")
elif drive_ok:
    yaw, why = fit["lidar_yaw_from_camera"], "주행과 카메라가 일치 -> 더 정밀한 카메라 값을 씁니다"
else:
    yaw, why = fit["lidar_yaw_from_camera"], "주의: 주행으로 확인되지 않은 값입니다 (카메라가 로봇 위에서 그대로라고 가정)"
rel = os.path.relpath(d + "/X_lidar_sam.json", O)
print(f"결정: LIDAR_YAW {yaw:.4f} rad ({deg(yaw):.1f} deg), X_LIDAR_SAM {rel}\n  {why}")
x = json.load(open(d + "/X_lidar_sam.json")); x["decision"] = {"lidar_yaw": yaw, "why": why, "drive": drive and {k: drive[k] for k in ("n", "d_deg", "mad_deg", "lidar_yaw_estimated", "ok")}}
json.dump(x, open(d + "/X_lidar_sam.json", "w"), indent=1)
if dry:
    print("  (DRY=1: mount.env 는 바꾸지 않았습니다)"); sys.exit(0)
shutil.copy(mount, mount + time.strftime(".bak-%y%m%d_%H%M%S"))
head = [l for l in open(mount) if l.startswith("#")]
open(mount, "w").write("".join(head) + f"LIDAR_YAW={yaw:.4f}\nX_LIDAR_SAM={rel}\n")
PYEOF
}

case "${1:-}" in
start)
  NAME="${2:?usage: new_place.sh start <map name>}"
  rssh true || die "노트북($L)에 접속할 수 없습니다 (랜선·전원 확인)"
  rssh "[ ! -e $R/maps/$NAME.pcd ]" || die "지도 이름 '$NAME' 이 이미 있습니다. 다른 이름을 쓰세요"
  rssh 'pgrep -f "[m]ap_localizer|[g]raph_slam_node|[c]ontroller_server" >/dev/null' && die "노트북에 주행 스택이 이미 떠 있습니다 (real_down.sh 먼저)"
  rssh 'ip -br addr | grep -q "192.168.1.77"' || die "LiDAR용 USB 랜 어댑터가 꺼져 있습니다"
  deploy
  say "지도 작성 모드 시작 (hdl_graph_slam)"; rssh "bash $R/real_up.sh" | tail -1
  rssh "bash $R/mount_yaw_up.sh" | tail -1
  echo "NAME=$NAME" > "$STATE"
  if [ "${VIEWER:-1}" = 1 ]; then
    # --no-sam6d: map building needs no objects, and with the SLAM pose still being optimised (height / tilt drift, keyframe
    # corrections) the boxes sat off the objects anyway (2026-10-14); recognition starts with the saved map at "finish"
    say "뷰어 시작 (http://192.168.2.2:8765, 지도 작성 중에는 객체 인식 끔)"; OUTV=$O/output/live_hdl-nav_map_${NAME}_$(date +%y%m%d_%H%M%S)
    ( cd "$REPO" && OUT=$OUTV LAPTOP_SSH=$L setsid nohup bash objpose/run_live.sh hdl-nav --no-sam6d > "$OUTV.log" 2>&1 < /dev/null & ) > /dev/null 2>&1   # the waiting wrapper must not hold the caller's pipe
  fi
  cat <<EOF
[new_place] 지도 작성 중입니다. 이제 로봇을 손으로(조이스틱) 몰아 공간 전체를 도세요.
  - 1 m 이상 똑바로 가는 구간을 3번 이상 넣으세요 (LiDAR 방향을 여기서 구합니다). 후진 직선도 됩니다.
  - 지도 작성 중에는 객체 인식을 하지 않습니다 (finish 뒤 저장 지도 위에서 시작).
  - 지금은 LiDAR 방향이 틀렸을 수 있으니 뷰어의 '목표 지정'·'순환'은 쓰지 마세요.
  - 진행 확인: bash objpose/new_place.sh status
  - 다 돌았으면 카메라가 1~4 m 거리의 벽·선반을 보는 곳에 세우고: bash objpose/new_place.sh finish
EOF
  ;;
status)
  T=$(mktemp); rssh "cat $R/logs/mount_yaw.json 2>/dev/null" > "$T"
  "$PY" - "$T" <<'PYEOF'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    print("mount_yaw 결과가 아직 없습니다 (start 를 실행했는지 확인)"); sys.exit()
print(f"바퀴 메시지 {d['wheel_msgs']}, 위치 메시지 {d['pose_msgs']} ({d['pose_source']})")
if not d["n"]:
    print("직진 구간 0개: 1 m 이상 똑바로 가는 구간이 필요합니다")
else:
    print(f"직진 구간 {d['n']}개: LiDAR 회전 {d['d_deg']:+.1f} deg (흩어짐 {d['mad_deg']} deg) -> LIDAR_YAW {d['lidar_yaw_estimated']} "
          f"(사용 중 {d['lidar_yaw_assumed']}) | {'충분' if d['ok'] else '더 필요'}")
for g in d["segments"][-8:]:
    print(f"   {g['dir']} {g['dist_m']} m, {g['dur_s']} s: {g['d_deg']:+.1f} deg")
PYEOF
  rm -f "$T"
  ;;
finish)
  [ -f "$STATE" ] || die "start 를 먼저 실행하세요"; NAME=$(sed -n 's/^NAME=//p' "$STATE"); DIR=$O/rt/$(date +%y%m%d)_$NAME; mkdir -p "$DIR"
  rssh 'pgrep -x graph_slam_node >/dev/null' || die "노트북에 지도 작성(hdl_graph_slam)이 떠 있지 않습니다"
  rssh "cat $R/logs/mount_yaw.json" > "$DIR/drive.json" 2>/dev/null || true
  HINT=$("$PY" -c "import json;d=json.load(open('$DIR/drive.json'));print(d['d_deg'] if d.get('ok') else '')" 2>/dev/null || true)
  say "1/5 지도 저장: $NAME"; rssh "bash $R/freeze_map.sh $NAME" | tail -2
  rssh "[ -s $R/maps/$NAME.pcd ] && [ -s $R/maps/$NAME.yaml ] && [ -s $R/maps/$NAME.pgm ]" || die "지도 파일이 만들어지지 않았습니다 (노트북 $R/maps)"
  # the path driven by hand = the road of this map: goals near it are driven along it (nav_bridge.py + road.py)
  rssh "cp $R/logs/track.json $R/maps/${NAME}_track.json" 2>/dev/null && say "도로(손으로 몬 경로) 저장: maps/${NAME}_track.json" \
    || say "손으로 몬 경로 기록이 없습니다 (도로 없이 진행)"
  scp -q "$L:objpose/nav/real/maps/$NAME.yaml" "$L:objpose/nav/real/maps/$NAME.pgm" "$O/nav/real/maps/" || die "지도 파일을 PC로 복사하지 못했습니다"
  say "2/5 지도 작성 모드 종료"; bash "$O/stop_live.sh" > /dev/null 2>&1; rssh "bash $R/real_down.sh" | tail -1
  say "3/5 정지 보정 (로봇을 움직이지 마세요)"; capture_and_fit "$DIR" "$HINT"
  say "4/5 결과"; decide_and_write "$DIR" "$DIR/drive.json"; RC=$?
  [ $RC = 0 ] || say "보정값은 그대로 두고 새 지도로 올립니다 (보정은 new_place.sh calib 로 다시)"
  deploy
  say "5/5 저장 지도 주행 + 객체 인식 시작 (지도 $NAME)"; rssh "bash $R/real_loc_up.sh $NAME" | tail -1
  for i in $(seq 1 20); do rssh "grep -q initialized $R/logs/localizer.log" && break; sleep 3; done
  rssh "grep initialized $R/logs/localizer.log | tail -1 | sed 's/.*initialized/위치 추정:/'" || say "위치 추정이 아직 잡히지 않았습니다 (노트북 $R/logs/localizer.log)"
  OUTV=$O/output/live_loc-nav_${NAME}_$(date +%y%m%d_%H%M%S)
  ( cd "$REPO" && OUT=$OUTV LAPTOP_SSH=$L setsid nohup bash objpose/run_live.sh loc-nav > "$OUTV.log" 2>&1 < /dev/null & ) > /dev/null 2>&1   # the waiting wrapper must not hold the caller's pipe
  rm -f "$STATE"
  say "끝. 뷰어 http://192.168.2.2:8765 , 기록 $OUTV , 보정 $DIR , 다음부터: real_loc_up.sh $NAME + run_live.sh loc-nav"
  ;;
calib)
  DIR=$O/rt/$(date +%y%m%d_%H%M)_calib
  capture_and_fit "$DIR" "${2:-}"; decide_and_write "$DIR" ""; RC=$?
  [ $RC = 0 ] && [ "${DRY:-0}" != 1 ] && deploy
  say "보정 폴더 $DIR (겹쳐 그린 그림: X_lidar_sam.overlay.jpg)"
  ;;
*) sed -n 2,22p "$0"; exit 1 ;;
esac
