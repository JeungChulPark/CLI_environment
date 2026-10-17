# hypothesis test: the camera rig is as calibrated on 2026-10-06 and only the lidar was turned about its own axis by d.
# Scores every d (no fitting), then fits freely from the best one.
import sys, json, numpy as np
exec(open(sys.argv[1]).read().split("cands = []")[0])
ref = np.linalg.inv(np.array(json.load(open(sys.argv[2]))["T_slam_sam"]))      # T_cam_lidar of the reference calibration
def rz(d):
    T = np.eye(4); T[:3, :3] = Rot.from_euler("z", d, degrees=True).as_matrix(); return T
rows = []
for d in np.arange(-180, 180, 2.0):
    T = ref @ np.linalg.inv(rz(d)); n, inl, med, _ = score(T, cap=30000); rows.append((inl / max(n, 1), d, n, inl, med))
rows.sort(reverse=True)
print("best turns (share of in-view lidar points within 3 cm of the depth surface):")
for r in rows[:6]: print(f"   d {r[1]:+6.1f} deg  {r[0]*100:5.1f}%  ({r[3]}/{r[2]})  median {r[4]*100:.2f} cm")
d0 = rows[0][1]; fine = []
for d in np.arange(d0 - 3, d0 + 3.01, 0.5):
    T = ref @ np.linalg.inv(rz(d)); n, inl, med, _ = score(T, cap=60000); fine.append((inl / max(n, 1), d))
fine.sort(reverse=True); db = fine[0][1]; print(f"refined turn d = {db:+.1f} deg ({fine[0][0]*100:.1f}%)")
Tb = ref @ np.linalg.inv(rz(db)); res = []
for seed in range(1, 7):
    T, _ = run(Tb, (0.12, 0.08, 0.06, 0.05, 0.04, 0.04, 0.04, 0.04), 25000, seed); res.append(T)
def desc(T):
    Tlc = np.linalg.inv(T); f = Tlc[:3, 2]
    return f"cam in lidar {np.round(Tlc[:3,3],3)} fwd yaw {np.degrees(np.arctan2(f[1],f[0])):7.2f} down {np.degrees(np.arcsin(-f[2])):5.2f} roll {np.degrees(np.arctan2(Tlc[2,0], -Tlc[2,1])):6.2f}"
n, inl, med, _ = score(res[0], cap=60000)
P = np.array([np.linalg.inv(T)[:3, 3] for T in res])
print("1-DoF guess:", desc(Tb)); print(f"free fit   : {desc(res[0])} | inliers {inl}/{n} ({inl/max(n,1)*100:.1f}%) median {med*100:.2f} cm | position std over 6 subsamples cm {(P.std(0)*100).round(2)}")
np.save(C + "/" + sys.argv[3] + "_free.npy", np.stack(res)); np.save(C + "/" + sys.argv[3] + "_1dof.npy", Tb)
import cv2
T = res[0]; bgr = cam["bgr"]
Q = np.ascontiguousarray(((T[:3, :3] @ L.T).T + T[:3, 3])[::6]); Q = np.ascontiguousarray(Q[Q[:, 2] > 0.4])
uv = cv2.projectPoints(Q, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2).round().astype(int)
ok = (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480); img = cv2.resize(bgr, None, fx=2, fy=2)
col = cv2.applyColorMap((np.clip(Q[ok, 2] / 5.0, 0, 1) * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
for (u, v), c in zip(uv[ok], col): cv2.circle(img, (int(u) * 2, int(v) * 2), 2, c.tolist(), -1)
cv2.imwrite(C + "/overlay_" + sys.argv[3] + ".jpg", img)
