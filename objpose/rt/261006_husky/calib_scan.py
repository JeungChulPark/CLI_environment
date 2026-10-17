import sys, json, numpy as np, cv2
exec(open(sys.argv[1]).read().split("cands = []")[0])
old = np.linalg.inv(np.array(json.load(open(R0 + "/rt/260915/X_lidar_sam_260915.json"))["T_slam_sam"]))
def rz(d):
    T = np.eye(4); T[:3, :3] = Rot.from_euler("z", d, degrees=True).as_matrix(); return T
print("old rig with the lidar turned by d about its own axis, no fitting:")
best = None
for d in np.arange(78, 96.1, 1.0):
    T = old @ np.linalg.inv(rz(d)); n, inl, med, _ = score(T, cap=60000)
    if best is None or inl > best[1]: best = (d, inl, T)
    print(f"  d {d:5.1f}  inliers {inl:6d}/{n:6d} ({inl/max(n,1)*100:4.1f}%) median {med*100:5.2f} cm")
Tb = best[2]
# free 6-DoF fit from the best 1-DoF guess, several subsamples -> spread = how well each direction is determined
res = []
for seed in range(1, 7):
    T, _ = run(Tb, (0.12, 0.08, 0.06, 0.05, 0.04, 0.04, 0.04, 0.04), 25000, seed); res.append(T)
P = np.array([np.linalg.inv(T)[:3, 3] for T in res]); E = np.array([Rot.from_matrix(np.linalg.inv(T)[:3, :3]).as_euler("zyx", degrees=True) for T in res])
print("free fit, 6 subsamples: cam position in lidar mean", P.mean(0).round(3), "std cm", (P.std(0) * 100).round(2), "| euler zyx std deg", E.std(0).round(2))
# sensitivity: shift the best free fit along camera x / y / z and watch the inlier count
T = res[0]; n0, i0, m0, _ = score(T, cap=60000); print(f"free fit: inliers {i0}/{n0} median {m0*100:.2f} cm")
for ax, nm in ((0, "cam x (lateral)"), (1, "cam y (vertical)"), (2, "cam z (depth)")):
    row = []
    for dlt in (-0.10, -0.05, -0.02, 0.02, 0.05, 0.10):
        S2 = np.eye(4); S2[ax, 3] = dlt; n, inl, med, _ = score(S2 @ T, cap=60000); row.append(f"{dlt*100:+.0f}cm:{inl/max(i0,1)*100:.0f}%")
    print(f"  shift along {nm:16s}", " ".join(row))
np.save(C + "/T_cam_lidar_free.npy", np.stack(res)); np.save(C + "/T_cam_lidar_1dof.npy", Tb); print("best d", best[0])
bgr = cam["bgr"]
for name, T in (("free", res[0]), ("1dof", Tb)):
    Q = np.ascontiguousarray(((T[:3, :3] @ L.T).T + T[:3, 3])[::6]); Q = np.ascontiguousarray(Q[Q[:, 2] > 0.4])
    uv = cv2.projectPoints(Q, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2).round().astype(int)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)
    img = cv2.resize(bgr, None, fx=2, fy=2)
    col = cv2.applyColorMap((np.clip(Q[ok, 2] / 5.0, 0, 1) * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
    for (u, v), c in zip(uv[ok], col): cv2.circle(img, (int(u) * 2, int(v) * 2), 2, c.tolist(), -1)
    cv2.imwrite(C + f"/overlay_{name}.jpg", img)
dm = cam["depth_mm"].astype(np.float32) / 1000
cv2.imwrite(C + "/depth.jpg", cv2.applyColorMap((np.clip(dm / 5.0, 0, 1) * 255).astype(np.uint8), cv2.COLORMAP_TURBO))
