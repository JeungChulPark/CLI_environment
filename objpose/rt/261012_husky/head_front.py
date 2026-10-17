import sys, pickle, json, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
R0 = "/home/jucpark/DeepLearning/CLI_environment_server/objpose"
sys.path.insert(0, R0 + "/lidar")
from estimate_lidar_cam_rt import DepthFrame, residuals, se3
from vlp16_decode import decode_scan
C = R0 + "/rt/261012_husky"
cam = np.load(C + "/cam_front.npz"); K, D = cam["K"], cam["D"]
pk = pickle.load(open(C + "/vlp_front.pkl", "rb"))["packets"]
revs, cur, last = [], [], None
for p in pk:
    az = p[2] | (p[3] << 8)
    if last is not None and az < last and len(cur) > 10: revs.append(cur); cur = []
    cur.append(p); last = az
revs = revs[1:]                               # the first one is partial
pts = []
for r in revs:
    s = decode_scan(r); ok = s["valid"]
    pts.append(np.column_stack([s["x"][ok], s["y"][ok], s["z"][ok]]).astype(np.float64))
L = np.vstack(pts); rr = np.linalg.norm(L, axis=1); L = L[(rr > 0.5) & (rr < 6.0)]
F = DepthFrame(cam["depth_mm"], K, D, stride=2, zmax=4.5)
pairs = [{"L": L, "F": F}]
print(f"revolutions {len(revs)}, lidar points 0.5-6 m: {len(L)}, depth surface points {len(F.P)}")

def init(psi, pitch, h):
    z = np.array([np.cos(psi), np.sin(psi), 0]); y = np.array([0, 0, -1.0]); x = np.cross(y, z)
    Rlc = np.column_stack([x, y, z]) @ Rot.from_euler("x", -pitch).as_matrix()     # pitch > 0: looking down
    T = np.eye(4); T[:3, :3] = Rlc; T[:3, 3] = [0, 0, h]
    return np.linalg.inv(T)

def run(T, schedule, cap, seed=1):
    rng = np.random.default_rng(seed); st = None
    for max_d in schedule:
        corr = residuals(T, pairs, max_d, per_pair_cap=cap, rng=rng)
        if not corr or len(corr[0][0]) < 30: return T, None
        Q, S_, N = corr[0]; Lc = (T[:3, :3].T @ (Q - T[:3, 3]).T).T
        f = lambda xi: np.sum((((T @ se3(xi))[:3, :3] @ Lc.T).T + (T @ se3(xi))[:3, 3] - S_) * N, axis=1)
        r = least_squares(f, np.zeros(6), loss="huber", f_scale=0.02); T = T @ se3(r.x)
    return T, st

def score(T, cap=20000):
    """lidar points that land in the image, and how many of them lie on the depth surface"""
    import cv2
    Q = (T[:3, :3] @ L.T).T + T[:3, 3]; Q = Q[Q[:, 2] > 0.4]
    uv = cv2.projectPoints(Q, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    Q = Q[(uv[:, 0] > 5) & (uv[:, 0] < F.w - 5) & (uv[:, 1] > 5) & (uv[:, 1] < F.h - 5)]
    if len(Q) < 50: return 0, 0, 9.9, None
    if len(Q) > cap: Q = Q[np.random.default_rng(0).choice(len(Q), cap, replace=False)]
    d, idx = F.tree.query(Q, distance_upper_bound=0.3); ok = np.isfinite(d)
    res = np.abs(np.sum((Q[ok] - F.P[idx[ok]]) * F.N[idx[ok]], axis=1))
    return len(Q), int((res < 0.03).sum()), float(np.median(res)) if ok.sum() else 9.9, (Q[ok], res)

cands = []
for psi in np.radians(np.arange(0, 360, 15)):
    for pitch in np.radians([0, 20, 40]):
        for h in (-0.2, -0.5, -0.8):
            T, _ = run(init(psi, pitch, h), (0.5, 0.3, 0.15), 1500)
            n, inl, med, _ = score(T)
            cands.append((inl, n, med, np.degrees(psi), np.degrees(pitch), h, T))
cands.sort(key=lambda c: -c[0])
print("top coarse candidates (inliers<3cm, in-view, median cm, init yaw/pitch/h):")
for c in cands[:8]: print(f"  {c[0]:6d} {c[1]:6d} {c[2]*100:5.2f}  yaw {c[3]:5.0f} pitch {c[4]:3.0f} h {c[5]}")
finals = []
for c in cands[:8]:
    T, _ = run(c[6], (0.25, 0.15, 0.10, 0.07, 0.05, 0.05, 0.05, 0.05), 6000)
    n, inl, med, _ = score(T); finals.append((inl, n, med, T))
finals.sort(key=lambda c: -c[0])
print("refined:")
for inl, n, med, T in finals:
    Tlc = np.linalg.inv(T); fwd = Tlc[:3, 2]
    print(f"  inliers {inl:6d}/{n:6d} median {med*100:5.2f} cm | cam in lidar xyz {np.round(Tlc[:3,3],3)} fwd yaw {np.degrees(np.arctan2(fwd[1],fwd[0])):7.2f} pitch-down {np.degrees(np.arcsin(-fwd[2])):6.2f}")
np.save(C + "/T_cam_lidar_candidates.npy", np.stack([f[3] for f in finals]))
np.save(C + "/lidar_points.npy", L)
