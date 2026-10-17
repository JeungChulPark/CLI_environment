# localizer_core.py — scan-to-map localization against a fixed hdl_graph_slam map (small_gicp GICP).
# Shared by the live node (map_localizer.py) and the offline bag test (offline_loc_test.py).
#  - map: the hdl save_map PCD (map frame, z = 0 at base_link of the first keyframe)
#  - scan: raw VLP-16 points in the velodyne frame -> base_link with the lidar mount, distance filter
#  - track(): register from several initial guesses (odometry prediction, previous pose, constant velocity), keep the
#    best fit; fitness = share of scan points within FIT_D of the map after alignment (GICP's own inlier count uses
#    the 1 m correspondence radius and was ~1.0 for right and wrong poses alike)
#  - Tracker.step(): init / track / lost -> local search -> global search, shared by the live node and the bag test
#  - relocalize(): global search over free cells of the 2D grid x yaw, coarse GICP, refine the best candidates
import math, numpy as np, small_gicp
from scipy.spatial import cKDTree
FIT_D = 0.25


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            line = f.readline().decode('ascii').strip(); k, _, v = line.partition(' '); hdr[k] = v.split()
            if k == 'DATA': break
        fields, sizes, types, n = hdr['FIELDS'], list(map(int, hdr['SIZE'])), hdr['TYPE'], int(hdr['POINTS'][0])
        dt = np.dtype([(fn, {'F': 'f', 'U': 'u', 'I': 'i'}[t] + str(s)) for fn, s, t in zip(fields, sizes, types)])
        if hdr['DATA'][0] == 'binary':
            a = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
            return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
        return np.loadtxt(f, max_rows=n)[:, :3].astype(np.float64)


def read_grid(yaml_path):
    """nav2 map_server YAML + PGM -> (free mask [row 0 = min y], resolution, origin x, origin y)"""
    import os
    meta = {}
    for line in open(yaml_path):
        if ':' in line: k, v = line.split(':', 1); meta[k.strip()] = v.strip()
    res = float(meta['resolution']); ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(pgm, 'rb') as f:
        f.readline(); w, h = map(int, f.readline().split()); f.readline()
        a = np.frombuffer(f.read(w * h), np.uint8).reshape(h, w)[::-1]
    return a >= 250, res, ox, oy


def se2(x, y, yaw):
    T = np.eye(4); c, s = math.cos(yaw), math.sin(yaw); T[:2, :2] = [[c, -s], [s, c]]; T[0, 3], T[1, 3] = x, y; return T


def yaw_of(T): return math.atan2(T[1, 0], T[0, 0])


class MapLocalizer:
    def __init__(self, pcd_path, grid_yaml=None, lidar_xyz_yaw=(0.0, 0.0, 1.215, 1.2171), near=1.0, far=20.0,
                 threads=2):
        self.threads = threads; self.near, self.far = near, far
        lx, ly, lz, lyaw = lidar_xyz_yaw
        self.T_base_lidar = se2(lx, ly, lyaw); self.T_base_lidar[2, 3] = lz
        pts = read_pcd_xyz(pcd_path)
        self.map_pts = pts
        self.target, self.tree = small_gicp.preprocess_points(pts, downsampling_resolution=0.15, num_threads=threads)
        self.grid = read_grid(grid_yaml) if grid_yaml else None
        fit_pts = small_gicp.voxelgrid_sampling(pts, 0.05).points()[:, :3]
        self.kd = cKDTree(fit_pts)

    def scan_to_base(self, lidar_pts):
        """Nx3 velodyne-frame points -> base_link, distance-filtered (same near/far as the hdl prefilter)"""
        d = np.linalg.norm(lidar_pts[:, :3], axis=1)
        p = lidar_pts[(d > self.near) & (d < self.far), :3].astype(np.float64)
        R, t = self.T_base_lidar[:3, :3], self.T_base_lidar[:3, 3]
        return p @ R.T + t

    def fitness(self, src_pts, T):
        p = src_pts @ T[:3, :3].T + T[:3, 3]
        d, _ = self.kd.query(p, k=1, distance_upper_bound=FIT_D)
        return float(np.mean(np.isfinite(d)))

    def register(self, base_pts, T_init, res=0.25, max_corr=1.0, iters=20, src=None):
        if src is None:
            src, _ = small_gicp.preprocess_points(base_pts, downsampling_resolution=res, num_threads=self.threads)
        r = small_gicp.align(self.target, src, self.tree, init_T_target_source=T_init, max_correspondence_distance=max_corr,
                             num_threads=self.threads, max_iterations=iters)
        fit = self.fitness(src.points()[:, :3], r.T_target_source)
        return r.T_target_source, fit, r.error / max(r.num_inliers, 1), r.converged

    def track(self, base_pts, guesses, min_fit=0.6, strong_fit=0.8, gate=(0.4, math.radians(10)), max_jump=(1.5, math.radians(25))):
        """register from each initial guess (first = odometry prediction), keep the best fitness.
        accept: fit >= strong_fit within max_jump of the prediction, or fit >= min_fit within the tight gate."""
        src, _ = small_gicp.preprocess_points(base_pts, downsampling_resolution=0.25, num_threads=self.threads)
        best = None
        for i, Tg in enumerate(guesses):
            T, fit, err, conv = self.register(base_pts, Tg, src=src)
            if conv and (best is None or fit > best[1]): best = (T, fit, err)
            if i == 0 and conv and fit >= strong_fit:      # odometry prediction already fits: skip the other guesses
                dT0 = np.linalg.inv(guesses[0]) @ T
                if math.hypot(dT0[0, 3], dT0[1, 3]) <= gate[0] and abs(yaw_of(dT0)) <= gate[1]: break
        T, fit, err = best if best else (guesses[0], 0.0, 9.0)
        dT = np.linalg.inv(guesses[0]) @ T
        jt, jr = math.hypot(dT[0, 3], dT[1, 3]), abs(yaw_of(dT))
        ok = (fit >= strong_fit and jt <= max_jump[0] and jr <= max_jump[1]) or (fit >= min_fit and jt <= gate[0] and jr <= gate[1])
        return T, ok, dict(fit=round(fit, 3), err=round(err, 4), jump_m=round(jt, 3), jump_deg=round(math.degrees(jr), 2))

    def relocalize(self, base_pts, step=0.5, yaw_step_deg=30, near_xy=None, radius=None, top=12):
        """global (or local, around near_xy within radius) search; returns (T, inlier, n_candidates)"""
        free, res, ox, oy = self.grid
        ys, xs = np.nonzero(free)
        cx, cy = ox + (xs + 0.5) * res, oy + (ys + 0.5) * res
        k = max(1, int(step / res)); sel = (xs % k == 0) & (ys % k == 0)
        cx, cy = cx[sel], cy[sel]
        if near_xy is not None:
            m = np.hypot(cx - near_xy[0], cy - near_xy[1]) <= radius; cx, cy = cx[m], cy[m]
        src, _ = small_gicp.preprocess_points(base_pts, downsampling_resolution=0.5, num_threads=self.threads)
        cands = []
        for x, y in zip(cx, cy):
            for yd in range(0, 360, yaw_step_deg):
                T0 = se2(x, y, math.radians(yd))
                r = small_gicp.align(self.target, src, self.tree, init_T_target_source=T0, max_correspondence_distance=1.5,
                                     num_threads=self.threads, max_iterations=10)
                cands.append((r.num_inliers / max(src.size(), 1), r.T_target_source))
        cands.sort(key=lambda c: -c[0])
        best = None
        for _, T0 in cands[:top]:
            T, fit, err, conv = self.register(base_pts, T0, max_corr=1.0, iters=30)
            if best is None or fit > best[1]: best = (T, fit)
        return best[0], best[1], len(cands)


class Tracker:
    """init / tracking / lost handling around MapLocalizer. step() takes base_link points and the odom->base_link pose
    of the same instant, returns (T_map_base or None, event dict or None)."""
    def __init__(self, loc, lost_n=5, accept_init=0.6):
        self.loc, self.lost_n, self.accept_init = loc, lost_n, accept_init
        self.T_map_odom = None; self.prev = None; self.prev2 = None; self.fails = 0; self.request = ('global', None); self.n_reject = 0

    def set_pose(self, T): self.request = ('pose', T)
    def set_global(self): self.request = ('global', None)

    def step(self, base, T_ob):
        ev = None
        if self.request is not None:
            kind, T0 = self.request; self.request = None
            if kind == 'global': T, fit, nc = self.loc.relocalize(base)
            else: T, fit, _, _ = self.loc.register(base, T0, max_corr=2.0, iters=40); nc = 1
            self.T_map_odom = T @ np.linalg.inv(T_ob); self.prev = (T, T_ob); self.prev2 = None; self.fails = 0
            return T, dict(msg='initialized' if fit >= self.accept_init else 'initialization weak', fit=round(fit, 3), candidates=nc)
        if self.T_map_odom is None: return None, None
        T_pred = self.T_map_odom @ T_ob
        guesses = [T_pred]
        if self.prev is not None:
            guesses.append(self.prev[0])                                   # robot assumed still
            if self.prev2 is not None:                                     # constant velocity in the map
                guesses.append(self.prev[0] @ (np.linalg.inv(self.prev2[0]) @ self.prev[0]))
        T, ok, info = self.loc.track(base, guesses)
        if ok:
            self.T_map_odom = T @ np.linalg.inv(T_ob); self.prev2, self.prev = self.prev, (T, T_ob); self.fails = 0
            return T, None
        self.fails += 1; self.n_reject += 1
        if self.fails == self.lost_n:
            Tl, fit, nc = self.loc.relocalize(base, near_xy=(T_pred[0, 3], T_pred[1, 3]), radius=1.5)
            if fit >= self.accept_init:
                self.T_map_odom = Tl @ np.linalg.inv(T_ob); self.prev, self.prev2, self.fails = (Tl, T_ob), None, 0
                return Tl, dict(msg='lost -> recovered by local search', fit=round(fit, 3), **{k: v for k, v in info.items() if k != 'fit'})
            self.request = ('global', None)
            ev = dict(msg='lost -> local search failed, global next', fit=round(fit, 3))
        return T_pred, ev
