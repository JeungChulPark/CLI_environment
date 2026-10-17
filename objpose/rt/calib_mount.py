#!/usr/bin/env python3
"""calib_mount.py — lidar -> SAM camera after the lidar turned on its mount. Robot standing still.

Inputs: one camera capture (rt/261013_husky/cam_cap.py: median depth + colour + intrinsics), raw VLP-16 packets of the
same moment (vlp_dump.py), and the calibration in use. Hypothesis first: "the rig is as calibrated and only the lidar was
turned about its own axis by d" — every d is scored (share of in-view lidar points on the depth surface), then a free
6-DoF fit starts from the best one. --hint-deg (the turn measured while driving, nav/real/mount_yaw.py) limits the
search to hint +- window, which keeps a look-alike direction of the room from winning.

  calib_mount.py --cam cam.npz --vlp vlp.pkl --ref X_old.json --out X_new.json [--hint-deg 142 --hint-window 10]
                 [--yaw-ref 1.2171]    LIDAR_YAW that belongs to --ref: the result then also carries the camera-based
                                        LIDAR_YAW (camera assumed fixed on the robot)
Prints one line of JSON (also stored under "fit" in --out).
"""
import argparse, json, math, pickle, sys
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "lidar"))
from estimate_lidar_cam_rt import DepthFrame, residuals, se3    # noqa: E402
from vlp16_decode import decode_scan                             # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--cam", required=True); ap.add_argument("--vlp", required=True); ap.add_argument("--ref", required=True)
ap.add_argument("--out", required=True); ap.add_argument("--hint-deg", type=float); ap.add_argument("--hint-window", type=float, default=10.0)
ap.add_argument("--yaw-ref", type=float); ap.add_argument("--note", default="")
a = ap.parse_args()

cam = np.load(a.cam); K, D = cam["K"], cam["D"]
revs, cur, last = [], [], None
for p in pickle.load(open(a.vlp, "rb"))["packets"]:
    az = p[2] | (p[3] << 8)
    if last is not None and az < last and len(cur) > 10: revs.append(cur); cur = []
    cur.append(p); last = az
revs = revs[1:]                                                   # the first one is partial
pts = []
for r in revs:
    s = decode_scan(r); ok = s["valid"]
    pts.append(np.column_stack([s["x"][ok], s["y"][ok], s["z"][ok]]).astype(np.float64))
L = np.vstack(pts); rr = np.linalg.norm(L, axis=1); L = L[(rr > 0.5) & (rr < 6.0)]
F = DepthFrame(cam["depth_mm"], K, D, stride=2, zmax=4.5)
pairs = [{"L": L, "F": F}]


def run(T, schedule, cap, seed=1):
    rng = np.random.default_rng(seed)
    for max_d in schedule:
        corr = residuals(T, pairs, max_d, per_pair_cap=cap, rng=rng)
        if not corr or len(corr[0][0]) < 30: return T
        Q, S_, N = corr[0]; Lc = (T[:3, :3].T @ (Q - T[:3, 3]).T).T
        f = lambda xi: np.sum((((T @ se3(xi))[:3, :3] @ Lc.T).T + (T @ se3(xi))[:3, 3] - S_) * N, axis=1)
        T = T @ se3(least_squares(f, np.zeros(6), loss="huber", f_scale=0.02).x)
    return T


def score(T, cap=20000):
    """lidar points that land in the image, and how many of them lie on the depth surface"""
    import cv2
    Q = (T[:3, :3] @ L.T).T + T[:3, 3]; Q = Q[Q[:, 2] > 0.4]
    uv = cv2.projectPoints(Q, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    Q = Q[(uv[:, 0] > 5) & (uv[:, 0] < F.w - 5) & (uv[:, 1] > 5) & (uv[:, 1] < F.h - 5)]
    if len(Q) < 50: return 0, 0, 9.9
    if len(Q) > cap: Q = Q[np.random.default_rng(0).choice(len(Q), cap, replace=False)]
    d, idx = F.tree.query(Q, distance_upper_bound=0.3); ok = np.isfinite(d)
    res = np.abs(np.sum((Q[ok] - F.P[idx[ok]]) * F.N[idx[ok]], axis=1))
    return len(Q), int((res < 0.03).sum()), float(np.median(res)) if ok.sum() else 9.9


def rz(d):
    T = np.eye(4); T[:3, :3] = Rot.from_euler("z", d, degrees=True).as_matrix(); return T


wrap = lambda d: (d + 180.0) % 360.0 - 180.0
old = json.load(open(a.ref)); ref = np.linalg.inv(np.array(old["T_slam_sam"]))     # T_cam_lidar of the calibration in use
rows = []
for d in np.arange(-180, 180, 2.0):
    n, inl, med = score(ref @ np.linalg.inv(rz(d)), cap=30000); rows.append((inl / max(n, 1), float(d), n, inl, med))
best_all = max(rows)
if a.hint_deg is not None:
    near = [r for r in rows if abs(wrap(r[1] - a.hint_deg)) <= a.hint_window]
    d0 = max(near)[1]
else:
    d0 = best_all[1]
fine = max((score(ref @ np.linalg.inv(rz(d)), cap=60000)[1] / 60000.0, float(d)) for d in np.arange(d0 - 3, d0 + 3.01, 0.5))
db = wrap(fine[1]); Tb = ref @ np.linalg.inv(rz(db))
res = [run(Tb, (0.12, 0.08, 0.06, 0.05, 0.04, 0.04, 0.04, 0.04), 25000, seed) for seed in range(1, 7)]
T = res[0]; n, inl, med = score(T, cap=60000); Tl = np.linalg.inv(T)
P = np.array([np.linalg.inv(x)[:3, 3] for x in res])
fwd = lambda M: math.degrees(math.atan2(M[1, 2], M[0, 2]))                          # camera forward (optical z) in the lidar frame
turn_fit = wrap(fwd(Tl) - fwd(np.array(old["T_slam_sam"])))                         # turn of the lidar implied by the free fit
fit = {"turn_scan_best_deg": best_all[1], "turn_scan_best_share": round(best_all[0], 3), "turn_hint_deg": a.hint_deg,
       "turn_1dof_deg": db, "turn_free_fit_deg": round(turn_fit, 2), "lidar_points_in_view": n, "within_3cm": inl,
       "share_within_3cm": round(inl / max(n, 1), 3), "residual_median_cm": round(med * 100, 2),
       "position_std_cm": [round(float(v), 2) for v in P.std(0) * 100], "revolutions": len(revs),
       "moved_vs_ref_cm": round(float(np.linalg.norm((np.linalg.inv(ref @ np.linalg.inv(rz(turn_fit))) - Tl)[:3, 3])) * 100, 1)}
if a.yaw_ref is not None:
    y = a.yaw_ref - math.radians(turn_fit); fit["lidar_yaw_from_camera"] = round((y + math.pi) % (2 * math.pi) - math.pi, 4)
# good = a clear majority of the in-view lidar points on the depth surface; the 10/12 and 10/13 fits had 42 %, a wrong pose ~20 %
fit["ok"] = bool(fit["share_within_3cm"] >= 0.30 and med < 0.035)
out = {"T_slam_sam": Tl.tolist(), "slam_sensor": "lidar", "sam_tau_s": 0.0, "clock": old.get("clock", "live: the hub measures laptop - PC at every start"),
       "source": f"static LiDAR-to-depth registration, robot not moving (objpose/rt/calib_mount.py): start = {Path(a.ref).name} with the lidar "
                 f"turned by {db:+.1f} deg about its own axis" + (f" (driving said {a.hint_deg:+.1f})" if a.hint_deg is not None else "")
                 + f", then a free 6-DoF fit. {a.note}".rstrip(),
       "fit": fit}
json.dump(out, open(a.out, "w"), indent=1)
try:                                                              # overlay for a look: lidar points coloured by range on the image
    import cv2
    Q = np.ascontiguousarray(((T[:3, :3] @ L.T).T + T[:3, 3])[::6]); Q = np.ascontiguousarray(Q[Q[:, 2] > 0.4])
    uv = cv2.projectPoints(Q, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2).round().astype(int)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480); img = cv2.resize(cam["bgr"], None, fx=2, fy=2)
    col = cv2.applyColorMap((np.clip(Q[ok, 2] / 5.0, 0, 1) * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
    for (u, v), c in zip(uv[ok], col): cv2.circle(img, (int(u) * 2, int(v) * 2), 2, c.tolist(), -1)
    cv2.imwrite(str(Path(a.out).with_suffix(".overlay.jpg")), img)
except Exception as e:                                            # noqa: BLE001
    print("overlay skipped:", e, file=sys.stderr)
print(json.dumps(fit))
