#!/bin/bash
# calib_sessions.sh — a per-session rig json for every 260901_cbnu session.
#
# The four sessions share one T_lidar_cam and one R_cam_imu (checked 2026-09-18: the differences,
# 0.687 deg and 21 mm, are the size of each fit's own residual), but their TIME OFFSETS genuinely
# differ from recording to recording — lidar_minus_cam spreads over 5.7 ms with correlations all
# above 0.995. At the few-cm level this comparison works at, that is not noise, so each session gets
# its own rig json rather than reusing bigeightcircle's.
#
# calib_rig.py needs the IMU as an npz (t_hdr_ns / gyro / acc), which only exists on the hub, so it
# is rebuilt here from the session's own bag.
#
# The camera trajectory is the bootstrap: the ORB-SLAM3 run made WITHOUT the gyro, so nothing
# circular enters. --static-s must match where each session starts moving (default 25 s only suits
# bigeightcircle; longcircle stands still for just 3.7 s, so its gyro bias z is the least certain).
#
#   bash calib_sessions.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIG="$HERE/../bigeight"
REF="$BIG/Userui-MacBookPro"
DATA=/Users/user/Documents/DefenseMeta/Dataset
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
OUT="$HERE/rigs"; mkdir -p "$OUT"

calib () {                      # session  static_s
  local s=$1 st=$2
  local d="$DATA/260901_cbnu_$s"
  local cam="$REF/260901_cbnu_$s/base_run1_tum.txt.optimized.txt"
  local kiss="$REF/260901_cbnu_$s/kiss/kiss_tum.txt"
  [ -f "$cam" ] && [ -f "$kiss" ] || { echo "  $s: 입력 없음"; return; }
  echo "== $s (static ${st}s)"
  "$PY" - "$d/imu" "$OUT/imu_$s.npz" <<'EOF'
import sys, glob, sqlite3, struct, numpy as np
db = glob.glob(sys.argv[1] + "/*.db3")[0]
c = sqlite3.connect("file:" + db + "?immutable=1", uri=True)
t, g, a = [], [], []
for (b,) in c.execute("select m.data from messages m join topics t on t.id=m.topic_id"
                      " where t.name='/xsens/imu/data' order by m.timestamp"):
    o = 4
    sec = struct.unpack_from("<i", b, o)[0]; o += 4
    ns = struct.unpack_from("<I", b, o)[0]; o += 4
    ln = struct.unpack_from("<I", b, o)[0]; o += 4 + ln
    o = 4 + ((o - 4 + 7) // 8) * 8
    o += 32 + 72                                  # orientation + its covariance
    g.append(struct.unpack_from("<3d", b, o)); o += 24 + 72
    a.append(struct.unpack_from("<3d", b, o))
    t.append(sec * 1000000000 + ns)
np.savez(sys.argv[2], t_hdr_ns=np.array(t, dtype=np.int64),
         gyro=np.array(g), acc=np.array(a))
print("   imu.npz: %d samples" % len(t))
EOF
  ( cd "$BIG" && "$PY" calib_rig.py --cam "$cam" --lidar "$kiss" --imu "$OUT/imu_$s.npz" \
      --static-s "$st" --out "$OUT/rig_$s.json" 2>&1 | grep -E "imu_minus_cam_s|lidar_minus_cam_s|closure_s|corr_c" | sed 's/^/   /' )
}

calib bigeightcircle 25
calib eightcircle    13
calib digut          13
calib longcircle      4

echo
echo "== 세션별 시간 오프셋"
"$PY" - "$OUT" <<'EOF'
import json, sys, glob, os
print("%-16s %14s %14s %12s %8s" % ("세션", "imu-cam ms", "lidar-cam ms", "closure ms", "corr"))
for f in sorted(glob.glob(sys.argv[1] + "/rig_*.json")):
    r = json.load(open(f)); t = r["time"]
    print("%-16s %14.3f %14.3f %12.3f %8.4f" % (
        os.path.basename(f)[4:-5], t["imu_minus_cam_s"] * 1e3,
        t["lidar_minus_cam_s"] * 1e3, t["closure_s"] * 1e3, t["corr_ci"]))
EOF
