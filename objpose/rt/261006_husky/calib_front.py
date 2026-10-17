import sys, json, numpy as np, cv2
exec(open(sys.argv[1]).read().split("cands = []")[0])
cands = []
for psi in np.radians(np.arange(0, 360, 15)):
    for pitch in np.radians([0, 20, 40]):
        for h in (-0.2, -0.5, -0.8):
            T, _ = run(init(psi, pitch, h), (0.5, 0.3, 0.15), 1500); n, inl, med, _ = score(T); cands.append((inl, n, med, T))
cands.sort(key=lambda c: -c[0]); fin = []
for c in cands[:8]:
    T = c[3]
    for seed in (1, 2): T, _ = run(T, (0.20, 0.12, 0.08, 0.06, 0.05, 0.04, 0.04, 0.04), 25000, seed)
    n, inl, med, _ = score(T, cap=60000); fin.append((inl, n, med, T))
fin.sort(key=lambda c: -c[0])
def desc(T):
    Tlc = np.linalg.inv(T); f = Tlc[:3, 2]
    return f"cam in lidar {np.round(Tlc[:3,3],3)} fwd yaw {np.degrees(np.arctan2(f[1],f[0])):7.2f} down {np.degrees(np.arcsin(-f[2])):5.2f} roll {np.degrees(np.arctan2(Tlc[2,0], -Tlc[2,1])):6.2f}"
for inl, n, med, T in fin: print(f"inliers {inl:6d}/{n:6d} ({inl/max(n,1)*100:4.1f}%) median {med*100:5.2f} cm | {desc(T)}")
res = []
for seed in range(1, 7):
    T, _ = run(fin[0][3], (0.08, 0.06, 0.05, 0.04, 0.04, 0.04, 0.04), 25000, seed); res.append(T)
Y = [np.degrees(np.arctan2(np.linalg.inv(T)[1, 2], np.linalg.inv(T)[0, 2])) for T in res]
print("best, 6 subsamples: fwd yaw mean %.2f std %.2f" % (np.mean(Y), np.std(Y)), "|", desc(res[0]))
np.save(C + "/T_frontcam_lidar_free.npy", np.stack(res))
old = json.load(open(R0 + "/rt/260915/T_slamcam_lidar_eightcircle_final.json"))["T_cam_lidar"]; old = np.array(old)
print("old 260915 laptop camera:", desc(old)); n, inl, med, _ = score(old, cap=60000); print(f"  old as is on today's data: inliers {inl}/{n} ({inl/max(n,1)*100:.1f}%)")
T = res[0]; bgr = cam["bgr"]
Q = np.ascontiguousarray(((T[:3, :3] @ L.T).T + T[:3, 3])[::6]); Q = np.ascontiguousarray(Q[Q[:, 2] > 0.4])
uv = cv2.projectPoints(Q, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2).round().astype(int)
ok = (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480); img = cv2.resize(bgr, None, fx=2, fy=2)
col = cv2.applyColorMap((np.clip(Q[ok, 2] / 5.0, 0, 1) * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
for (u, v), c in zip(uv[ok], col): cv2.circle(img, (int(u) * 2, int(v) * 2), 2, c.tolist(), -1)
cv2.imwrite(C + "/overlay_front.jpg", img)
