#!/usr/bin/env python3
"""pseudo_gt.py — offline (non-causal) LiDAR reference trajectory for our own sessions ("pseudo-GT").

The reference used so far (rt/260901/bigeight/kiss_gt.py) is plain KISS-ICP odometry: causal, no loop
closure, so its error grows with distance (bigeightcircle: 15 cm start-to-end, weak pitch/roll). This script
keeps the same front end and adds what an offline reference can afford:

  1. odometry   KISS-ICP 1.3, same settings as kiss_gt.py (0.5-30 m, voxel 0.10, two deskew passes), or
                `--deskew imu`: each sweep is deskewed with the xsens gyro rotation (+ constant-velocity
                translation) before registration. Every deskewed scan is kept (voxel 0.10) for stage 2.
  2. loops      keyframes every 1 m / 10 deg; each keyframe gets a submap (+-1 s of scans, odometry-stitched).
                Candidates: keyframes >= 15 s apart and < 4 m apart by odometry, at most one partner per
                re-visit. Verified by point-to-point ICP (KISS registration, coarse -> fine) in BOTH directions;
                accepted when inlier ratio, inlier RMS, forward/backward agreement and the size of the
                correction are all within gates.
  3. graph      pose graph over keyframes (odometry edges + loop edges), Gauss-Newton via scipy least_squares
                on increments, loop edges re-weighted (Cauchy) and gated. Every scan is then re-placed by
                interpolating the keyframe corrections. Optional `--gravity`: accelerometer gravity priors.
  4. checks     5-fold held-out loop edges (optimise without them, predict them), start/end closure held out,
                cross-pass map plane residual, tilt vs. accelerometer gravity, KISS vs. pseudo-GT difference.

Output (same convention as kiss_gt.py: T_world_velodyne, world = first scan, velodyne header stamp):
  <out>/pgo_tum.txt  odom_tum.txt  loops.json  summary.json

    nice -n 10 ~/objpose/lidar/venv/bin/python objpose/rt/gt/pseudo_gt.py --session <dataset>/<session> --out <dir>
"""
from __future__ import annotations
import argparse, json, sqlite3, struct, sys, time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose" / "lidar"))
sys.path.insert(0, str(REPO / "objpose" / "rt" / "260901" / "bigeight"))
from lidar_stream import header_stamp, parse_pc2  # noqa: E402
from trajlib import inv, se3  # noqa: E402

RIG_DEFAULT = REPO / "objpose" / "rt" / "260901" / "bigeight" / "rig_260901_bigeight.json"


# ----------------------------------------------------------------------------- small SE(3) helpers
def rv(R):
    return Rotation.from_matrix(R).as_rotvec()


def log6(T):
    """[rotvec, translation] of a 4x4 (or [N,4,4]) — translation not the true SE3 log, fine for small errors"""
    return np.concatenate([rv(T[..., :3, :3]), T[..., :3, 3]], axis=-1)


def write_tum(path, t_ns, T):
    q = Rotation.from_matrix(T[:, :3, :3]).as_quat()
    with open(path, "w") as f:
        for k in range(len(t_ns)):
            f.write(f"{t_ns[k]/1e9:.9f} {T[k,0,3]:.6f} {T[k,1,3]:.6f} {T[k,2,3]:.6f} "
                    f"{q[k,0]:.9f} {q[k,1]:.9f} {q[k,2]:.9f} {q[k,3]:.9f}\n")


def voxel(p, size):
    if len(p) == 0:
        return p
    _, i = np.unique(np.floor(p / size).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(i)]


# ----------------------------------------------------------------------------- data
def open_ro(db):
    try:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        con.execute("select count(*) from topics").fetchone()
        return con
    except sqlite3.Error:
        return sqlite3.connect(f"file:{db}?immutable=1", uri=True)


def lidar_index(lidar_dir):
    db = next(Path(lidar_dir).glob("*.db3"))
    con = open_ro(db)
    tid = con.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
    rows = con.execute("select id, substr(data, 1, 16) from messages where topic_id=? order by timestamp", (tid,)).fetchall()
    uniq = {}
    for mid, h in rows:
        uniq.setdefault(header_stamp(h), mid)     # 260915 bags repeat messages: one per header stamp
    stamps = sorted(uniq)
    return con, stamps, uniq, len(rows)


def read_scan(con, mid):
    pts = parse_pc2(con.execute("select data from messages where id=?", (mid,)).fetchone()[0])
    xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
    r = np.linalg.norm(xyz, axis=1)
    ok = np.isfinite(r) & (r > 0.1)
    return xyz[ok], pts["time"][ok].astype(np.float64)   # time: seconds relative to the header stamp (-0.1 .. 0)


def read_imu(imu_dir):
    """xsens sensor_msgs/Imu -> t_ns [N], gyro [N,3] (rad/s), acc [N,3] (m/s^2), imu frame"""
    db = next(Path(imu_dir).glob("*.db3"))
    con = open_ro(db)
    tid = con.execute("select id from topics where name='/xsens/imu/data'").fetchone()[0]
    T, G, A = [], [], []
    for (blob,) in con.execute("select data from messages where topic_id=? order by timestamp", (tid,)):
        sec, nsec = struct.unpack_from("<iI", blob, 4)
        n = struct.unpack_from("<I", blob, 12)[0]
        o = 16 + n
        o = 4 + ((o - 4 + 7) // 8) * 8            # CDR alignment counts from after the 4-byte encapsulation
        v = struct.unpack_from("<" + "d" * 37, blob, o)   # orientation 4, cov 9, gyro 3, cov 9, acc 3, cov 9
        T.append(sec * 1_000_000_000 + nsec); G.append(v[13:16]); A.append(v[25:28])
    t = np.array(T, np.int64); o = np.argsort(t, kind="stable")
    t, G, A = t[o], np.array(G)[o], np.array(A)[o]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], G[keep], A[keep]


class GyroDeskew:
    """per-point rotation inside a sweep from the integrated gyro, expressed in the LiDAR frame"""

    def __init__(self, t_ns, gyro, R_li, tau_li_s):
        self.t = t_ns.astype(np.float64) * 1e-9
        g = gyro - np.median(gyro[: min(len(gyro), 2000)], axis=0)   # bias: irrelevant over 0.1 s, removed anyway
        dt = np.diff(self.t)
        dR = Rotation.from_rotvec(0.5 * (g[:-1] + g[1:]) * dt[:, None])
        R = np.empty((len(self.t), 4)); R[0] = [0, 0, 0, 1]
        cur = Rotation.identity()
        for k in range(len(dR)):
            cur = cur * dR[k]
            R[k + 1] = cur.as_quat()
        self.slerp = Slerp(self.t, Rotation.from_quat(R))
        self.R_li = R_li
        self.tau = tau_li_s

    def deskew(self, xyz, s, t_hdr_ns, v_lidar):
        """points stamped t_hdr + s (s <= 0) -> frame of the sensor at t_hdr (end of the sweep)"""
        t_end = t_hdr_ns * 1e-9 + self.tau
        bins = np.round(s / 0.005).astype(int)            # 5 ms bins: < 0.1 deg of rotation per bin
        ub = np.unique(bins)
        tq = np.clip(t_end + ub * 0.005, self.t[0], self.t[-1])
        Ri = self.slerp(np.concatenate([[np.clip(t_end, self.t[0], self.t[-1])], tq])).as_matrix()
        R_end_s_imu = np.einsum("ji,njk->nik", Ri[0], Ri[1:])             # R_i(end)^T R_i(s)
        R_end_s = np.einsum("ij,njk,lk->nil", self.R_li, R_end_s_imu, self.R_li)
        idx = np.searchsorted(ub, bins)
        out = np.einsum("nij,nj->ni", R_end_s[idx], xyz) + s[:, None] * v_lidar[None, :]
        return out


# ----------------------------------------------------------------------------- stage 1: odometry
def kiss(voxel_size, deskew, threads):
    from kiss_icp.config import load_config
    from kiss_icp.kiss_icp import KissICP
    cfg = load_config(None)
    cfg.data.min_range, cfg.data.max_range, cfg.data.deskew = 0.5, 30.0, deskew
    cfg.mapping.voxel_size = voxel_size
    cfg.registration.max_num_threads = threads
    return KissICP(cfg), cfg


def odometry(a, out):
    con, stamps, uniq, n_msgs = lidar_index(Path(a.session) / "lidar")
    use_imu = a.deskew == "imu"
    odom, _ = kiss(0.10, not use_imu, a.threads)
    gd = None
    if use_imu:
        rig = json.loads(Path(a.rig).read_text())
        R_li = np.array(rig["T_lidar_cam"])[:3, :3] @ np.array(rig["R_cam_imu"])
        ti, gy, _ = read_imu(Path(a.session) / "imu")
        gd = GyroDeskew(ti, gy, R_li, rig["time"]["imu_minus_lidar_s"])
    poses, clouds = [], []
    t0 = time.perf_counter()
    for k, t_ns in enumerate(stamps):
        xyz, s = read_scan(con, uniq[t_ns])
        u = (s - s.min()) / max(s.max() - s.min(), 1e-9)
        if use_imu:
            v = odom.last_delta[:3, 3] / 0.1                 # constant-velocity translation (lidar frame)
            d = gd.deskew(xyz, s, t_ns, v)
            odom.register_frame(d, u)
            f = odom.preprocessor.preprocess(d, u, np.eye(4))
        else:
            guess = odom.last_pose @ odom.last_delta
            delta, pose = odom.last_delta, guess
            sigma = odom.adaptive_threshold.get_threshold()
            for _ in range(2):                              # = lidar_stream.register(..., passes=2)
                f = odom.preprocessor.preprocess(xyz, u, delta)
                source, frame_down = odom.voxelize(f)
                pose = odom.registration.align_points_to_map(points=source, voxel_map=odom.local_map, initial_guess=pose,
                                                             max_correspondance_distance=3 * sigma, kernel=sigma)
                delta = np.linalg.inv(odom.last_pose) @ pose
            odom.adaptive_threshold.update_model_deviation(np.linalg.inv(guess) @ pose)
            odom.local_map.update(frame_down, pose)
            odom.last_delta = delta
            odom.last_pose = pose
        poses.append(np.array(odom.last_pose))
        clouds.append(voxel(f, 0.10).astype(np.float32))
        if k % 500 == 0:
            print(f"  odom {k}/{len(stamps)}", flush=True)
    P = np.array(poses); t_ns = np.array(stamps, np.int64)
    write_tum(out / "odom_tum.txt", t_ns, P)
    off = np.cumsum([0] + [len(c) for c in clouds])
    np.savez(out / "scans_cache.npz", t_ns=t_ns, P=P, pts=np.concatenate(clouds), off=off)
    return {"messages": n_msgs, "scans": len(stamps), "deskew": a.deskew,
            "ms_per_scan": round((time.perf_counter() - t0) * 1e3 / len(stamps), 2)}


# ----------------------------------------------------------------------------- stage 2: loops
def keyframes(P, dist=1.0, ang_deg=10.0):
    kf = [0]
    for i in range(1, len(P)):
        D = inv(P[kf[-1]]) @ P[i]
        if np.linalg.norm(D[:3, 3]) >= dist or np.degrees(np.linalg.norm(rv(D[:3, :3]))) >= ang_deg:
            kf.append(i)
    if kf[-1] != len(P) - 1:
        kf.append(len(P) - 1)
    return np.array(kf)


def submap(P, pts, off, i, w):
    Ti = inv(P[i])
    parts = []
    for j in range(max(0, i - w), min(len(P), i + w + 1)):
        T = Ti @ P[j]
        q = pts[off[j]:off[j + 1]].astype(np.float64)
        parts.append(q @ T[:3, :3].T + T[:3, 3])
    return voxel(np.concatenate(parts), 0.10)


class Matcher:
    def __init__(self, threads):
        from kiss_icp.mapping import VoxelHashMap
        from kiss_icp.registration import Registration
        self.VHM = VoxelHashMap
        self.reg = Registration(max_num_iterations=500, convergence_criterion=1e-4, max_num_threads=threads)

    def align(self, src, tgt, tgt_tree, T0):
        m = self.VHM(voxel_size=0.25, max_distance=1e9, max_points_per_voxel=20)
        m.add_points(tgt)
        s = voxel(src, 0.25)
        T = T0
        for corr, ker in ((1.5, 0.5), (0.6, 0.15), (0.3, 0.05)):
            T = self.reg.align_points_to_map(points=s, voxel_map=m, initial_guess=T, max_correspondance_distance=corr, kernel=ker)
        q = src @ T[:3, :3].T + T[:3, 3]
        d, _ = tgt_tree.query(q, distance_upper_bound=0.5)
        inl = d < 0.15
        return T, float(inl.mean()), float(np.sqrt((d[inl] ** 2).mean())) if inl.any() else 9.0


def find_loops(P, t_s, pts, off, kf, a):
    path = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P[:, :3, 3], axis=0), axis=1))])
    X = P[kf, :3, 3]
    tree = cKDTree(X[:, :2])
    pairs = []
    for ia, i in enumerate(kf):
        cand = [ib for ib in tree.query_ball_point(X[ia, :2], a.loop_radius) if ib > ia
                and t_s[kf[ib]] - t_s[i] > a.loop_gap_s and path[kf[ib]] - path[i] > 5.0]
        if not cand:
            continue
        cand = np.array(sorted(cand))
        runs = np.split(cand, np.where(np.diff(cand) > 3)[0] + 1)     # one partner per re-visit
        for r in runs:
            dd = np.linalg.norm(X[r] - X[ia], axis=1)
            pairs.append((ia, int(r[np.argmin(dd)])))
    print(f"  loop candidates: {len(pairs)}", flush=True)
    M = Matcher(a.threads)
    cache = {}

    def sm(ia):
        if ia not in cache:
            p = submap(P, pts, off, kf[ia], a.submap_w)
            cache[ia] = (p, cKDTree(p))
        return cache[ia]

    loops = []
    for n, (ia, ib) in enumerate(pairs):
        (pa, ta), (pb, tb) = sm(ia), sm(ib)
        T0 = inv(P[kf[ia]]) @ P[kf[ib]]                      # odometry guess for T_a_b
        Tab, fa, ra = M.align(pb, pa, ta, T0)                # b's points into a's frame
        Tba, fb, rb = M.align(pa, pb, tb, inv(T0))
        fwbw = inv(Tab) @ inv(Tba)
        e_fb_t = float(np.linalg.norm(fwbw[:3, 3])); e_fb_r = float(np.degrees(np.linalg.norm(rv(fwbw[:3, :3]))))
        corr = inv(T0) @ Tab
        c_t = float(np.linalg.norm(corr[:3, 3])); c_r = float(np.degrees(np.linalg.norm(rv(corr[:3, :3]))))
        ok = (min(fa, fb) >= a.min_inlier and max(ra, rb) <= a.max_rms and e_fb_t <= 0.05 and e_fb_r <= 0.5
              and c_t <= 2.0 and c_r <= 15.0)
        loops.append({"a": int(ia), "b": int(ib), "dt_s": float(t_s[kf[ib]] - t_s[kf[ia]]),
                      "inlier": [round(fa, 3), round(fb, 3)], "rms_m": [round(ra, 4), round(rb, 4)],
                      "fwbw_cm": round(e_fb_t * 100, 2), "fwbw_deg": round(e_fb_r, 3),
                      "corr_cm": round(c_t * 100, 1), "corr_deg": round(c_r, 2), "accepted": bool(ok),
                      "T_ab": Tab.tolist()})
        if len(cache) > 400:
            cache.clear()
        if n % 50 == 0:
            print(f"  loop {n}/{len(pairs)} accepted so far {sum(l['accepted'] for l in loops)}", flush=True)
    return loops


# ----------------------------------------------------------------------------- stage 3: pose graph
def optimise(P0, edges, fixed=0, iters=4, cauchy_c=3.0, verbose=True, gravity=None):
    """P0 [N,4,4] initial keyframe poses; edges: list of (a, b, Z_ab, sig_r rad, sig_t m, is_loop).
    Increment parametrisation: R = R0 exp(d_phi), t = t0 + d_t. Loop edges IRLS re-weighted (Cauchy)."""
    N = len(P0)
    E = len(edges)
    A = np.array([e[0] for e in edges]); B = np.array([e[1] for e in edges])
    Z = np.array([e[2] for e in edges]); Zi = inv(Z)
    sr = np.array([e[3] for e in edges]); st = np.array([e[4] for e in edges]); loop = np.array([e[5] for e in edges])
    w = np.ones(E)
    R0 = P0[:, :3, :3]; t0 = P0[:, :3, 3]
    nG = 0 if gravity is None else len(gravity[0])

    def poses(x):
        d = x.reshape(N, 6)
        R = np.einsum("nij,njk->nik", R0, Rotation.from_rotvec(d[:, :3]).as_matrix())
        T = np.tile(np.eye(4), (N, 1, 1)); T[:, :3, :3] = R; T[:, :3, 3] = t0 + d[:, 3:]
        return T

    def res(x):
        T = poses(x)
        Er = inv(T[A]) @ T[B]
        err = Zi @ Er
        r = np.concatenate([rv(err[:, :3, :3]) / sr[:, None], err[:, :3, 3] / st[:, None]], axis=1) * np.sqrt(w)[:, None]
        out = [r.ravel(), x.reshape(N, 6)[fixed] * 1e4]
        if nG:
            idx, g_meas, g_w, sg = gravity
            pred = np.einsum("nji,j->ni", T[idx, :3, :3], g_w)        # world gravity in the keyframe frame
            out.append(((pred - g_meas) / sg).ravel())
        return np.concatenate(out)

    S = lil_matrix((E * 6 + 6 + nG * 3, N * 6), dtype=int)
    for k in range(E):
        S[k * 6:(k + 1) * 6, A[k] * 6:(A[k] + 1) * 6] = 1
        S[k * 6:(k + 1) * 6, B[k] * 6:(B[k] + 1) * 6] = 1
    S[E * 6:E * 6 + 6, fixed * 6:fixed * 6 + 6] = 1
    for g in range(nG):
        S[E * 6 + 6 + g * 3:E * 6 + 9 + g * 3, gravity[0][g] * 6:gravity[0][g] * 6 + 3] = 1
    x = np.zeros(N * 6)
    for it in range(iters):
        sol = least_squares(res, x, jac_sparsity=S, method="trf", tr_solver="lsmr", x_scale="jac", max_nfev=60, verbose=0)
        x = sol.x
        T = poses(x)
        err = Zi @ (inv(T[A]) @ T[B])
        nrm = np.sqrt(((rv(err[:, :3, :3]) / sr[:, None]) ** 2).sum(1) + ((err[:, :3, 3] / st[:, None]) ** 2).sum(1))
        w = np.where(loop, 1.0 / (1.0 + (nrm / cauchy_c) ** 2), 1.0)
        if verbose:
            print(f"  GN round {it}: cost {sol.cost:.1f}  loop edges w<0.1: {int((loop & (w < 0.1)).sum())}/{int(loop.sum())}", flush=True)
    return poses(x), w


def edge_list(P, kf, loops, use=None):
    edges = []
    for k in range(len(kf) - 1):
        Z = inv(P[kf[k]]) @ P[kf[k + 1]]
        d = np.linalg.norm(Z[:3, 3]); ang = np.linalg.norm(rv(Z[:3, :3]))
        edges.append((k, k + 1, Z, 0.0017 + 0.005 * ang + 0.0005 * d, 0.005 + 0.01 * d, False))
    for n, l in enumerate(loops):
        if l["accepted"] and (use is None or use[n]):
            edges.append((l["a"], l["b"], np.array(l["T_ab"]), np.radians(0.15), 0.02, True))
    return edges


def spread(P_odom, kf, Xkf):
    """all scans from keyframe corrections: C = X_kf P_kf^-1, interpolated between keyframes"""
    C = Xkf @ inv(P_odom[kf])
    N = len(P_odom)
    sl = Slerp(kf.astype(float), Rotation.from_matrix(C[:, :3, :3]))
    i = np.arange(N, dtype=float)
    Rc = sl(np.clip(i, kf[0], kf[-1])).as_matrix()
    tc = np.column_stack([np.interp(i, kf, C[:, j, 3]) for j in range(3)])
    Cn = np.tile(np.eye(4), (N, 1, 1)); Cn[:, :3, :3] = Rc; Cn[:, :3, 3] = tc
    return Cn @ P_odom


# ----------------------------------------------------------------------------- stage 4: checks
def predict(T, l):
    return inv(T[l["a"]]) @ T[l["b"]]


def held_out(P, kf, loops, a, gravity):
    acc = [n for n, l in enumerate(loops) if l["accepted"]]
    if len(acc) < 10:
        return {}
    rng = np.random.default_rng(0)
    perm = rng.permutation(acc)
    folds = np.array_split(perm, 5)
    et, er = [], []
    for f in folds:
        use = np.ones(len(loops), bool); use[f] = False
        X, _ = optimise(P[kf], edge_list(P, kf, loops, use), verbose=False, gravity=gravity)
        for n in f:
            e = inv(np.array(loops[n]["T_ab"])) @ predict(X, loops[n])
            et.append(np.linalg.norm(e[:3, 3])); er.append(np.degrees(np.linalg.norm(rv(e[:3, :3]))))
    X0 = P[kf]                                   # same edges predicted by plain odometry
    eo_t, eo_r = [], []
    for n in acc:
        e = inv(np.array(loops[n]["T_ab"])) @ predict(X0, loops[n])
        eo_t.append(np.linalg.norm(e[:3, 3])); eo_r.append(np.degrees(np.linalg.norm(rv(e[:3, :3]))))
    st = lambda v, s: {"n": len(v), "median": round(float(np.median(v)) * s, 2), "rms": round(float(np.sqrt(np.mean(np.square(v)))) * s, 2),
                       "p90": round(float(np.percentile(v, 90)) * s, 2), "max": round(float(np.max(v)) * s, 2)}
    return {"pgo_heldout_cm": st(et, 100), "pgo_heldout_deg": st(er, 1), "odom_cm": st(eo_t, 100), "odom_deg": st(eo_r, 1)}


def cross_pass_planes(T, t_s, pts, off, stride=2, n_query=20000, gap_s=15.0, seed=0):
    """point-to-plane distance of a point to a plane fitted from points recorded >= gap_s apart (other passes)"""
    P_, Ts_ = [], []
    for k in range(0, len(T), stride):
        q = pts[off[k]:off[k + 1]].astype(np.float64)
        q = q[np.linalg.norm(q, axis=1) < 15.0]
        P_.append(q @ T[k, :3, :3].T + T[k, :3, 3]); Ts_.append(np.full(len(q), t_s[k]))
    X = np.concatenate(P_); tt = np.concatenate(Ts_)
    tree = cKDTree(X)
    rng = np.random.default_rng(seed)
    qi = rng.choice(len(X), min(n_query, len(X)), replace=False)
    dist_o, dist_s = [], []
    nb = tree.query_ball_point(X[qi], 0.35)
    for q, idx in zip(qi, nb):
        idx = np.array(idx)
        for other, sink in ((np.abs(tt[idx] - tt[q]) >= gap_s, dist_o), ((np.abs(tt[idx] - tt[q]) < 1.0) & (idx != q), dist_s)):
            j = idx[other]
            if len(j) < 10:
                continue
            Y = X[j]; c = Y.mean(0)
            ev, V = np.linalg.eigh(np.cov((Y - c).T))
            if ev[0] > 0.05 * ev[1]:            # not planar enough
                continue
            sink.append(abs((X[q] - c) @ V[:, 0]))
    f = lambda v: {"n": len(v), "median_cm": round(float(np.median(v)) * 100, 2), "p90_cm": round(float(np.percentile(v, 90)) * 100, 2)} if v else {}
    return {"other_pass": f(dist_o), "same_pass": f(dist_s)}


def gravity_data(a, t_lidar_ns, kf, P):
    """accelerometer gravity direction at each keyframe (lidar frame), centripetal/linear accel removed from
    the trajectory itself, averaged over +-0.5 s; returns idx, g_meas [K,3] (unit), g_world (unit)"""
    rig = json.loads(Path(a.rig).read_text())
    R_li = np.array(rig["T_lidar_cam"])[:3, :3] @ np.array(rig["R_cam_imu"])
    tau = rig["time"]["imu_minus_lidar_s"]
    ti, _, acc = read_imu(Path(a.session) / "imu")
    ti = ti * 1e-9 - tau                         # imu stamps on the lidar clock
    tl = t_lidar_ns * 1e-9
    # linear acceleration of the lidar in the world from the trajectory (smoothed), into the lidar frame
    pos = P[:, :3, 3]
    k = 5
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    ps = np.column_stack([np.convolve(np.pad(pos[:, j], k, mode="edge"), ker, mode="valid") for j in range(3)])
    acc_w = np.gradient(np.gradient(ps, tl, axis=0), tl, axis=0)
    g_meas, idx = [], []
    for n, i in enumerate(kf):
        m = np.abs(ti - tl[i]) < 0.5
        if m.sum() < 50:
            continue
        f_i = acc[m].mean(0)                     # specific force (imu frame) = R^T (a - g); at rest +9.8 up
        f_l = R_li @ f_i
        a_l = P[i, :3, :3].T @ acc_w[i]
        up_l = f_l - a_l                         # = -g in lidar frame
        g_meas.append(-up_l / np.linalg.norm(up_l)); idx.append(n)
    g_meas = np.array(g_meas)
    g_w = np.einsum("nij,nj->ni", P[np.array(kf)[idx], :3, :3], g_meas).mean(0)   # world gravity from the odometry
    return np.array(idx), g_meas, g_w / np.linalg.norm(g_w)


def fit_mount(T_kf, gravity):
    """one constant tilt of the accelerometer in the LiDAR frame (rotation about the LiDAR x / y axes only:
    mounting error of R_lidar_imu, accelerometer bias) + the world gravity direction. Only 2 DOF: a free
    rotation about the vertical would let the fit re-shuffle tilt noise between axes."""
    idx, g_meas, g_w = gravity[:3]
    R = T_kf[idx, :3, :3]

    def ang(x):
        gw = x[2:] / np.linalg.norm(x[2:])
        pred = np.einsum("nji,j->ni", R, gw)
        m = g_meas @ Rotation.from_rotvec([x[0], x[1], 0.0]).as_matrix().T
        return np.degrees(np.arccos(np.clip((pred * m).sum(1), -1, 1)))

    sol = least_squares(ang, np.concatenate([[0.0, 0.0], g_w]))
    Rc = Rotation.from_rotvec([sol.x[0], sol.x[1], 0.0]).as_matrix()
    return Rc, sol.x[2:] / np.linalg.norm(sol.x[2:]), ang(np.concatenate([[0.0, 0.0], g_w])), ang(sol.x)


def tilt_vs_gravity(T_kf, gravity):
    """angle between the trajectory's gravity (world gravity seen in each keyframe) and the accelerometer's,
    raw and after fitting a constant 2-DOF mounting tilt (fit_mount)"""
    Rc, _, raw, cal = fit_mount(T_kf, gravity)
    f = lambda v: {"median_deg": round(float(np.median(v)), 3), "p90_deg": round(float(np.percentile(v, 90)), 3),
                   "max_deg": round(float(v.max()), 3)}
    return {"raw": f(raw), "cal": f(cal), "mount_corr_deg": round(float(np.degrees(np.linalg.norm(rv(Rc)))), 3)}


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, help="dataset session dir with lidar/ (and imu/ for --deskew imu / --gravity)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--rig", default=str(RIG_DEFAULT))
    ap.add_argument("--deskew", default="cv", choices=["cv", "imu"])
    ap.add_argument("--gravity", action="store_true", help="add accelerometer gravity priors (needs imu/)")
    ap.add_argument("--gravity-sigma-deg", type=float, default=0.5)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--submap-w", type=int, default=10, help="scans on each side of a keyframe in its submap")
    ap.add_argument("--loop-radius", type=float, default=4.0)
    ap.add_argument("--loop-gap-s", type=float, default=15.0)
    ap.add_argument("--min-inlier", type=float, default=0.5)
    ap.add_argument("--max-rms", type=float, default=0.08)
    ap.add_argument("--redo-odom", action="store_true")
    ap.add_argument("--no-checks", action="store_true")
    a = ap.parse_args()
    out = Path(a.out).expanduser(); out.mkdir(parents=True, exist_ok=True)
    summary = {"session": str(a.session), "args": vars(a)}
    t_start = time.perf_counter()

    if a.redo_odom or not (out / "scans_cache.npz").exists():
        summary["odometry"] = odometry(a, out)
    elif (out / "summary.json").exists():
        summary["odometry"] = json.loads((out / "summary.json").read_text()).get("odometry", {})
    C = np.load(out / "scans_cache.npz")
    t_ns, P, pts, off = C["t_ns"], C["P"], C["pts"], C["off"]
    t_s = (t_ns - t_ns[0]) * 1e-9
    path = float(np.linalg.norm(np.diff(P[:, :3, 3], axis=0), axis=1).sum())
    kf = keyframes(P)
    summary["path_m"] = round(path, 2); summary["duration_s"] = round(float(t_s[-1]), 1); summary["keyframes"] = int(len(kf))
    print(f"[pseudo_gt] {len(P)} scans, {len(kf)} keyframes, path {path:.1f} m", flush=True)

    if (out / "loops.json").exists() and not a.redo_odom:
        loops = json.loads((out / "loops.json").read_text())
    else:
        loops = find_loops(P, t_s, pts, off, kf, a)
        (out / "loops.json").write_text(json.dumps(loops))
    acc = [l for l in loops if l["accepted"]]
    summary["loops"] = {"candidates": len(loops), "accepted": len(acc),
                        "fwbw_cm_median": round(float(np.median([l["fwbw_cm"] for l in acc])), 2) if acc else None,
                        "fwbw_deg_median": round(float(np.median([l["fwbw_deg"] for l in acc])), 3) if acc else None,
                        "corr_cm_median": round(float(np.median([l["corr_cm"] for l in acc])), 1) if acc else None,
                        "corr_cm_max": round(float(np.max([l["corr_cm"] for l in acc])), 1) if acc else None}
    print(f"[pseudo_gt] loops {summary['loops']}", flush=True)

    gravity = gravity_data(a, t_ns, kf, P) if (Path(a.session) / "imu").exists() else None
    gprior = None
    if a.gravity and gravity is not None:
        # first pass without gravity, mounting tilt fitted on it, then gravity priors with the corrected accelerometer
        X1, _ = optimise(P[kf], edge_list(P, kf, loops), verbose=False)
        Rc, g_w, _, _ = fit_mount(X1, gravity)
        gprior = (gravity[0], gravity[1] @ Rc.T, g_w, np.sin(np.radians(a.gravity_sigma_deg)))
    Xkf, w = optimise(P[kf], edge_list(P, kf, loops), gravity=gprior)
    nloop = sum(1 for l in loops if l["accepted"])
    summary["loops"]["downweighted"] = int((w[len(kf) - 1:] < 0.1).sum()) if nloop else 0
    T = spread(P, kf, Xkf)
    write_tum(out / "pgo_tum.txt", t_ns, T)
    diff = np.linalg.norm(T[:, :3, 3] - P[:, :3, 3], axis=1)
    summary["pgo_minus_odom_cm"] = {"median": round(float(np.median(diff)) * 100, 1), "max": round(float(diff.max()) * 100, 1),
                                    "end": round(float(diff[-1]) * 100, 1)}
    summary["start_end_gap_cm"] = {"odom": round(float(np.linalg.norm(P[-1, :3, 3] - P[0, :3, 3])) * 100, 1),
                                   "pgo": round(float(np.linalg.norm(T[-1, :3, 3] - T[0, :3, 3])) * 100, 1)}

    if not a.no_checks:
        print("[pseudo_gt] checks: held-out loops", flush=True)
        summary["heldout_loops"] = held_out(P, kf, loops, a, gprior)
        # start/end: loops between the first and the last 20 s, held out together
        se = [n for n, l in enumerate(loops) if l["accepted"] and t_s[kf[l["a"]]] < 20 and t_s[-1] - t_s[kf[l["b"]]] < 20]
        if se:
            use = np.ones(len(loops), bool); use[se] = False
            Xh, _ = optimise(P[kf], edge_list(P, kf, loops, use), verbose=False, gravity=gprior)
            ev = []
            for n in se:
                e = inv(np.array(loops[n]["T_ab"])) @ predict(Xh, loops[n]); eo = inv(np.array(loops[n]["T_ab"])) @ predict(P[kf], loops[n])
                ev.append((np.linalg.norm(e[:3, 3]) * 100, np.linalg.norm(eo[:3, 3]) * 100))
            ev = np.array(ev)
            summary["start_end_heldout_cm"] = {"edges": len(se), "pgo_median": round(float(np.median(ev[:, 0])), 2),
                                               "odom_median": round(float(np.median(ev[:, 1])), 2)}
        print("[pseudo_gt] checks: cross-pass planes", flush=True)
        summary["planes_odom"] = cross_pass_planes(P, t_s, pts, off)
        summary["planes_pgo"] = cross_pass_planes(T, t_s, pts, off)
        if gravity is not None:
            summary["tilt_vs_gravity"] = {"odom": tilt_vs_gravity(P[kf], gravity), "pgo": tilt_vs_gravity(Xkf, gravity),
                                          "fitted": bool(a.gravity)}
    summary["wall_s"] = round(time.perf_counter() - t_start, 1)
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps({k: v for k, v in summary.items() if k != "args"}, indent=1), flush=True)


if __name__ == "__main__":
    main()
