"""obj_anchor.py — correct the live SLAM camera pose with objects already registered in the Object Pose Map.

The Object Pose Map is built from SLAM poses; this closes the loop the other way. Objects that have been
observed several times are static landmarks. Every new SAM-6D result that sees such an object says where the
camera must be for the object to sit at its registered pose. A small sliding-window pose graph combines
those constraints with the SLAM relative motion:

    nodes      x_i = T_w_cam(t_i), camera poses of the last `window` SAM frames that saw a landmark
    odometry   x_i^-1 x_j  ~  T_slam(t_i)^-1 T_slam(t_j)             (SLAM is accurate over short spans)
    object     x_i^-1 T_w_L ~  T_cam_obj (SAM-6D)                     (position; rotation unless symmetric)
    prior      oldest node stays near its previous estimate            (stands in for the dropped nodes)

The solution gives the camera pose of the newest node, and the correction C = x_last . T_slam(t_last)^-1 is
applied to every SLAM pose until the next update:  T_w_cam_corrected(t) = C . T_slam(t).
Nothing in the SLAM process changes; SLAM keeps tracking, the correction lives on top of its output.

Frames: all poses are of the camera the detections are expressed in (the SAM camera: T_slam . X on a rig).
Units: metres.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

DEG = np.pi / 180.0


def inv(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def rotvec(R):
    return Rotation.from_matrix(R).as_rotvec()


def _num_jac(f, x, h=1e-6):
    f0 = f(x)
    J = np.empty((f0.size, x.size))
    for i in range(x.size):
        e = np.zeros_like(x)
        e[i] = h
        J[:, i] = (f(x + e) - f0) / h
    return J


@dataclass
class AnchorConfig:
    mode: str = "rigid"           # "rigid": one correction C over the window (SLAM relative motion inside the window
                                  #          taken as exact) | "graph": one free pose per node + odometry factors
    window: int = 12              # SAM frames with a landmark observation kept in the graph
    window_s: float = 3.0         # ... and no older than this [s]
    min_lm_rot: int = 3           # rigid: correct the rotation only when this many distinct landmarks are in the window
    max_rot_sigma: float = 0.0    # rigid: ... and only when the window pins every rotation axis to this [deg] (1-sigma from
                                  # the object factors alone, Schur complement over translation); else translation only
    sig_rw_t0: float = 0.005      # rigid: correction random walk between updates [m] ...
    sig_rw_td: float = 0.05       # ... + fraction of the camera path since the last update
    sig_rw_r0: float = 0.3        # [deg] ...
    sig_rw_rd: float = 0.05       # ... + fraction of the camera rotation since the last update
    min_obs: int = 5              # a landmark anchors the camera only after this many fused observations
    gate_m: float = 0.30          # detection -> landmark association gate [m] (same class, after the current correction)
    sig_obs_t0: float = 0.010     # object position noise [m] ...
    sig_obs_tz: float = 0.010     # ... + this much per metre of depth
    sig_obs_r: float = 8.0        # object rotation noise [deg]; not used for symmetric objects
    sig_odo_t0: float = 0.002     # SLAM relative motion noise [m] ...
    sig_odo_td: float = 0.02      # ... + fraction of the distance travelled
    sig_odo_r0: float = 0.2       # [deg] ...
    sig_odo_rd: float = 0.02      # ... + fraction of the rotation angle
    sig_prior_t: float = 0.01     # oldest node vs its previous estimate [m]
    sig_prior_r: float = 0.5      # [deg]
    huber: float = 2.0            # robust loss scale on whitened residuals
    reject: float = 4.0           # drop an object factor whose whitened error stays above this, then re-solve
    max_jump_m: float = 0.5       # refuse an update that moves the correction by more than this
    use_rotation: bool = True     # use object orientation (False = position only)
    freeze: bool = True           # anchor to the landmark pose at the moment it became usable (min_obs), not to its
                                  # running average — the average keeps absorbing the drift of later camera poses
    loop_blend_s: float = 0.0     # >0: at a SLAM loop closure keep the displayed pose continuous (absorb the jump) and
                                  # let it converge to the corrected map over this long [s] — like map->odom in REP-105
    blend_s: float = 0.0          # move to a new correction over this long instead of jumping to it [s] (the correction only
                                  # changes when a SAM-6D result arrives, 2-5 Hz; a jump shows as a step in the live path)
    deadband_m: float = 0.0       # apply only the part of the correction beyond this shift of the current camera [m]
                                  # (soft threshold). On a two-camera rig the extrinsic error makes an object look a few cm
                                  # apart from opposite sides; corrections of that size are not evidence of SLAM error.
    rot_axis: tuple = ()          # yaw-only correction about this world axis (floor normal / gravity), as in the 4-DoF pose
                                  # graph of VINS-Mono: roll and pitch are held by gravity, only yaw drifts. () = full 3-DoF
    min_view_deg: float = 0.0     # a landmark anchors only after it has been seen from directions this far apart [deg]
                                  # (estimates from one side share a view-dependent bias of a few cm)
    rot_from: str = "all"         # rigid: which observations may turn the correction. "sam" = SAM-6D results only; depth-ICP
                                  # results (observe(src="icp"), 10 Hz from nearly one viewpoint, with a per-object bias of
                                  # 1-3 cm) then only shift it, with the rotation of the SAM-6D solve held fixed
    icp_sig_m: float = 0.0        # >0: position noise of a depth-ICP observation [m], whatever the SAM-6D noise (ICP is
                                  # biased by 1-3 cm on partly seen or box-shaped objects; 0 = same as SAM-6D)
    max_rot_step_deg: float = 0.0  # >0: one update turns the correction by at most this much [deg] (SLAM rotation drift is slow,
                                  # 1-2 deg per minute; a bigger step comes from a few biased objects, not from drift)
    rot_group: bool = False       # rotation observability: count every landmark once (its observations in the window share
                                  # one bias, so N of them do not pin the rotation N times better)
    rot_min_view_deg: float = 0.0  # ... and correct the rotation only when min_lm_rot landmarks were seen in the window from
                                  # directions at least this far apart [deg]
    symmetric: frozenset = frozenset()


@dataclass
class _Node:
    t_ns: int
    T_slam: np.ndarray            # camera pose from SLAM
    x: np.ndarray                 # current estimate of the corrected camera pose
    obs: list = field(default_factory=list)   # (landmark id, name, T_w_L, T_cam_obj)
    src: str = "sam"              # "sam" (SAM-6D result) | "icp" (depth ICP)


class ObjectAnchor:
    def __init__(self, cfg: AnchorConfig | None = None):
        self.cfg = cfg or AnchorConfig()
        self.C = np.eye(4)                     # estimated T_corrected <- T_slam
        self.C_applied = np.eye(4)             # the part that is applied (deadband)
        self.now_ns = None                     # caller: arrival time of the result being fed (for blend_s)
        self._C_from = np.eye(4)
        self._t_upd = None
        self._blend_cur = None                 # duration of the transition in progress (None = cfg.blend_s)
        self._loop_end = None
        self.nodes: list[_Node] = []
        self.last_t_ns = None
        self.frozen: dict = {}                 # landmark id -> T_w_L used as the anchor
        self.views: dict = {}                  # landmark key -> unit directions landmark -> camera it was seen from
        self.clusters: dict = {}               # class name -> first positions of its instances (for landmark keys)
        # optional keyframe anchoring of the frozen anchors: attach(T_w, t_ns) -> handle, resolve(handle) -> T_w | None.
        # A frozen anchor then stays fixed relative to the SLAM keyframe it was frozen at, so it keeps the pre-drift
        # position until SLAM itself re-optimises that keyframe (loop closure), and then moves with it.
        self.attach = None
        self.resolve = None
        self._t_now = None
        self.stats = {"updates": 0, "frames": 0, "frames_with_anchor": 0, "factors": 0, "rejected": 0,
                      "refused_jumps": 0, "solve_ms": []}

    # ── use ─────────────────────────────────────────────────────────────────
    def applied_at(self, t_ns=None) -> np.ndarray:
        """the correction in effect at time t: C_applied, reached from the previous one over blend_s seconds"""
        if (self.cfg.blend_s <= 0 and not self._blend_cur) or t_ns is None or self._t_upd is None:
            return self.C_applied
        dur = self._blend_cur if self._blend_cur else self.cfg.blend_s
        a = float(np.clip((t_ns - self._t_upd) / (dur * 1e9), 0.0, 1.0))
        if a >= 1.0:
            return self.C_applied
        D = self.C_applied @ inv(self._C_from)
        out = np.eye(4)
        out[:3, :3] = Rotation.from_rotvec(a * rotvec(D[:3, :3])).as_matrix()
        out[:3, 3] = a * D[:3, 3]
        return out @ self._C_from

    def correct(self, T_slam: np.ndarray, t_ns=None) -> np.ndarray:
        return self.applied_at(t_ns) @ T_slam

    def correction(self, t_ns=None) -> np.ndarray:
        return self.applied_at(t_ns).copy()

    def _apply_deadband(self, T_slam):
        if (self.cfg.blend_s > 0 or self._blend_cur) and self.now_ns is not None:
            self._C_from = self.applied_at(self.now_ns).copy()
            self._t_upd = self.now_ns
            self._blend_cur = max(self.cfg.blend_s, self._remaining_loop_blend(self.now_ns))
        d0 = self.cfg.deadband_m
        if d0 <= 0:
            self.C_applied = self.C.copy()
            return
        shift = float(np.linalg.norm((self.C @ T_slam)[:3, 3] - T_slam[:3, 3]))
        a = 0.0 if shift <= d0 else 1.0 - d0 / shift
        out = np.eye(4)
        out[:3, :3] = Rotation.from_rotvec(a * rotvec(self.C[:3, :3])).as_matrix()
        # scale the camera shift, not C's translation (C rotates about the world origin)
        p = T_slam[:3, 3]
        out[:3, 3] = p + a * ((self.C @ T_slam)[:3, 3] - p) - out[:3, :3] @ p
        self.C_applied = out

    def _remaining_loop_blend(self, t_ns):
        end = self._loop_end
        return 0.0 if end is None or t_ns >= end else (end - t_ns) / 1e9

    def reset_smooth(self, T_before, T_after, t_ns):
        """SLAM loop closure: the pose of the same camera moved from T_before to T_after. Keep what is displayed
        continuous — start from C_from with C_from . T_after = C_old . T_before — and converge to the new map
        (C = I) over loop_blend_s."""
        C_old = self.applied_at(t_ns)
        self.reset()
        self._C_from = C_old @ T_before @ inv(T_after)
        self._t_upd = t_ns
        self._blend_cur = self.cfg.loop_blend_s
        self._loop_end = t_ns + int(self.cfg.loop_blend_s * 1e9)

    def reset(self):
        """the SLAM map was re-optimised (loop closure / map merge): the correction was estimated against the old
        map and is no longer valid — start again from the SLAM pose."""
        self.C = np.eye(4)
        self.C_applied = np.eye(4)
        self._C_from = np.eye(4)
        self._t_upd = None
        self._blend_cur = None
        self._loop_end = None
        self.nodes = []
        self._last_T = None
        self.stats["resets"] = self.stats.get("resets", 0) + 1

    def _key(self, name, p):
        """stable identity of a landmark: class name + which instance (nearest earlier position within the gate).
        Memory ids are not stable — a memory rebuilt after a keyframe update renumbers its landmarks."""
        cl = self.clusters.setdefault(name, [])
        for i, q in enumerate(cl):
            if np.linalg.norm(q - p) < self.cfg.gate_m:
                return (name, i)
        cl.append(np.array(p, float))
        return (name, len(cl) - 1)

    def _record_views(self, T_cam, dets, landmarks):
        c = T_cam[:3, 3]
        for name, T_co in dets:
            p = (T_cam @ T_co)[:3, 3]
            best, bd = None, self.cfg.gate_m
            for lid, lname, T_wl, _ in landmarks:
                if lname == name:
                    d = float(np.linalg.norm(np.asarray(T_wl)[:3, 3] - p))
                    if d < bd:
                        best, bd = np.asarray(T_wl)[:3, 3], d
            if best is not None:
                v = c - best
                n = np.linalg.norm(v)
                if n > 1e-6:
                    lst = self.views.setdefault(self._key(name, best), [])
                    lst.append(v / n)
                    del lst[:-200]

    def _view_spread_deg(self, key) -> float:
        V = np.array(self.views.get(key, []))
        if len(V) < 2:
            return 0.0
        return float(np.degrees(np.arccos(np.clip((V @ V.T).min(), -1.0, 1.0))))

    # ── update ──────────────────────────────────────────────────────────────
    def associate(self, T_cam: np.ndarray, dets, landmarks):
        """dets: [(name, T_cam_obj)], landmarks: [(id, name, T_w_obj, n_obs)] -> [(key, name, T_w_L, T_cam_obj)]"""
        cfg = self.cfg
        if cfg.min_view_deg > 0:
            self._record_views(T_cam, dets, landmarks)

        def ok(name, T):
            return cfg.min_view_deg <= 0 or self._view_spread_deg(self._key(name, np.asarray(T)[:3, 3])) >= cfg.min_view_deg

        if cfg.freeze:
            for lid, lname, T_wl, n_obs in landmarks:
                k = self._key(lname, np.asarray(T_wl)[:3, 3])
                if k not in self.frozen and n_obs >= cfg.min_obs and ok(lname, T_wl):
                    T0 = np.array(T_wl, float)
                    self.frozen[k] = self.attach(T0, self._t_now) if self.attach is not None else T0
            landmarks = []
            for k, h in self.frozen.items():
                Tf = self.resolve(h) if self.resolve is not None else h
                if Tf is not None:
                    landmarks.append((k, k[0], Tf, 1 << 30))
        else:
            landmarks = [(self._key(n, np.asarray(T)[:3, 3]), n, T, c) for _, n, T, c in landmarks if c >= cfg.min_obs and ok(n, T)]
        out, used = [], set()
        for name, T_co in dets:
            p = (T_cam @ T_co)[:3, 3]
            best, bd = None, cfg.gate_m
            for lid, lname, T_wl, n_obs in landmarks:
                if lname != name or n_obs < cfg.min_obs or lid in used:
                    continue
                d = float(np.linalg.norm(T_wl[:3, 3] - p))
                if d < bd:
                    best, bd = (lid, lname, np.asarray(T_wl, float), np.asarray(T_co, float)), d
            if best is not None:
                used.add(best[0])
                out.append(best)
        return out

    def observe(self, t_ns: int, T_slam: np.ndarray, dets, landmarks, src: str = "sam") -> np.ndarray:
        """one SAM-6D result (or depth-ICP result, src="icp"): camera pose from SLAM at the frame time, detections in
        that camera, and the current Object Pose Map (in the corrected frame). Returns the corrected camera pose."""
        self.stats["frames"] += 1
        self._t_now = int(t_ns)
        T_slam = np.asarray(T_slam, float)
        x0 = self.C @ T_slam
        obs = self.associate(x0, dets, landmarks)
        if not obs:
            return self.C_applied @ T_slam
        self.stats["frames_with_anchor"] += 1
        self.nodes.append(_Node(int(t_ns), T_slam, x0, obs, src))
        del self.nodes[:-self.cfg.window]
        self.nodes = [n for n in self.nodes if t_ns - n.t_ns <= self.cfg.window_s * 1e9]
        t0 = time.perf_counter()
        ok = self._solve_rigid() if self.cfg.mode == "rigid" else self._solve()
        self.stats["solve_ms"].append((time.perf_counter() - t0) * 1e3)
        if ok:
            self.stats["updates"] += 1
        self._apply_deadband(T_slam)
        return self.C_applied @ T_slam

    # ── rigid window ────────────────────────────────────────────────────────
    def _factors(self, nodes):
        cfg = self.cfg
        oi, LR, Lt, MR, Mt, sym = [], [], [], [], [], []
        for i, n in enumerate(nodes):
            for lid, name, T_wl, T_co in n.obs:
                oi.append(i); LR.append(T_wl[:3, :3]); Lt.append(T_wl[:3, 3]); MR.append(T_co[:3, :3]); Mt.append(T_co[:3, 3])
                sym.append((name in cfg.symmetric) or not cfg.use_rotation)
        oi = np.array(oi); LR = np.stack(LR); Lt = np.stack(Lt); MR = np.stack(MR); Mt = np.stack(Mt)
        rw = np.where(np.array(sym), 0.0, 1.0 / (cfg.sig_obs_r * DEG))
        st = cfg.sig_obs_t0 + cfg.sig_obs_tz * np.abs(Mt[:, 2])
        return oi, LR, Lt, MR, Mt, rw, st

    def _solve_rigid(self) -> bool:
        """x_i = C . T_slam(t_i) for every node in the window; solve the 6-DoF C."""
        cfg, nodes = self.cfg, self.nodes
        oi, LR, Lt, MR, Mt, rw, st = self._factors(nodes)
        if cfg.icp_sig_m > 0:
            st = np.where(np.array([nodes[i].src == "icp" for i in oi]), cfg.icp_sig_m, st)
        SR = np.stack([n.T_slam[:3, :3] for n in nodes])[oi]
        St = np.stack([n.T_slam[:3, 3] for n in nodes])[oi]
        C0 = self.C
        # random walk of the correction since the last update, from the SLAM motion in between
        Tp = self._last_T if getattr(self, "_last_T", None) is not None else nodes[-1].T_slam
        M = inv(Tp) @ nodes[-1].T_slam
        s_t = cfg.sig_rw_t0 + cfg.sig_rw_td * np.linalg.norm(M[:3, 3])
        s_r = cfg.sig_rw_r0 * DEG + cfg.sig_rw_rd * np.linalg.norm(rotvec(M[:3, :3]))
        # pivot the correction's rotation at the newest camera so that rotation and translation separate
        pc = (C0 @ nodes[-1].T_slam)[:3, 3]
        active = np.ones(len(oi), bool)

        def Cof(d):
            dR = Rotation.from_rotvec(d[:3]).as_matrix()
            C = np.eye(4)
            C[:3, :3] = dR @ C0[:3, :3]
            C[:3, 3] = dR @ (C0[:3, 3] - pc) + pc + d[3:]
            return C

        def resid(d, act):
            C = Cof(d)
            R = np.einsum("ij,njk->nik", C[:3, :3], SR)
            t = St @ C[:3, :3].T + C[:3, 3]
            Rp = np.einsum("nji,njk->nik", R, LR)
            tp = np.einsum("nji,nj->ni", R, Lt - t)
            et = (tp - Mt) / st[:, None]
            er = Rotation.from_matrix(np.einsum("nji,njk->nik", MR, Rp)).as_rotvec() * rw[:, None]
            m = act[:, None].astype(float)
            return np.concatenate([d[:3] / s_r, d[3:] / s_t, (et * m).ravel(), (er * m).ravel()])

        rmask = np.array([nodes[i].src != "icp" for i in oi]) if cfg.rot_from == "sam" else np.ones(len(oi), bool)
        lids = [lid for n in nodes for lid, *_ in n.obs]                 # same order as the factors
        rot_ok = len({l for l, m in zip(lids, rmask) if m}) >= cfg.min_lm_rot
        if rot_ok and cfg.rot_min_view_deg > 0:
            dirs = {}
            for i, (l, m) in enumerate(zip(lids, rmask)):
                if m:
                    c = (C0 @ nodes[oi[i]].T_slam)[:3, 3]
                    v = c - Lt[i]
                    dirs.setdefault(l, []).append(v / max(np.linalg.norm(v), 1e-9))
            wide = sum(1 for V in dirs.values() if len(V) > 1 and
                       np.degrees(np.arccos(np.clip((np.array(V) @ np.array(V).T).min(), -1.0, 1.0))) >= cfg.rot_min_view_deg)
            rot_ok = wide >= cfg.min_lm_rot
        if len(cfg.rot_axis) == 3:                          # yaw only: (yaw, tx, ty, tz)
            ax_ = np.asarray(cfg.rot_axis, float)
            ax_ = ax_ / np.linalg.norm(ax_)
            expand = lambda q: np.r_[q[0] * ax_, q[1:]]
            nrot = 1
        else:
            expand = lambda q: q
            nrot = 3
        if rot_ok and cfg.max_rot_sigma > 0:
            # observability of the rotation from the object factors alone (prior excluded):
            # 1-sigma of the worst rotation axis after marginalising the translation
            J = _num_jac(lambda x: resid(expand(x), active & rmask)[6:], np.zeros(nrot + 3))
            if cfg.rot_group:
                cnt = {}
                for l, m in zip(lids, rmask):
                    if m:
                        cnt[l] = cnt.get(l, 0) + 1
                w = np.array([1.0 / np.sqrt(cnt[l]) if m else 0.0 for l, m in zip(lids, rmask)])
                J = J * np.r_[np.repeat(w, 3), np.repeat(w, 3)][:, None]
            H = J.T @ J
            Hs = H[:nrot, :nrot] - H[:nrot, nrot:] @ np.linalg.pinv(H[nrot:, nrot:]) @ H[nrot:, :nrot]
            lam = max(float(np.linalg.eigvalsh(np.atleast_2d(Hs)).min()), 1e-12)
            rot_ok = (1.0 / np.sqrt(lam)) / DEG <= cfg.max_rot_sigma
        self.stats["rot_updates"] = self.stats.get("rot_updates", 0) + int(rot_ok)

        def robust(fun, q0, to_d, mask):
            """solve over the factors in mask, dropping those whose whitened error stays above cfg.reject"""
            nonlocal active
            d = to_d(q0)
            for _ in range(3):
                sol = least_squares(fun, q0, args=(active & mask,), loss="huber", f_scale=cfg.huber, method="trf", max_nfev=40)
                q0 = sol.x
                d = to_d(sol.x)
                C = Cof(d)
                R = np.einsum("ij,njk->nik", C[:3, :3], SR)
                t = St @ C[:3, :3].T + C[:3, 3]
                e = np.linalg.norm((np.einsum("nji,nj->ni", R, Lt - t) - Mt) / st[:, None], axis=1)
                bad = active & mask & (e > cfg.reject)
                if not bad.any():
                    break
                active &= ~bad
                self.stats["rejected"] += int(bad.sum())
            return d

        if rot_ok:
            d = robust(lambda q, act: resid(expand(q), act), np.zeros(nrot + 3), expand, rmask)
            ang = float(np.linalg.norm(d[:3]))
            clip = cfg.max_rot_step_deg > 0 and ang > cfg.max_rot_step_deg * DEG
            if clip:
                d = np.r_[d[:3] * (cfg.max_rot_step_deg * DEG / ang), d[3:]]
                self.stats["rot_clipped"] = self.stats.get("rot_clipped", 0) + 1
            if clip or (cfg.rot_from == "sam" and not rmask.all()):
                # the rotation from SAM-6D only (and / or clipped); every observation (depth ICP too) then only moves the camera
                dr = d[:3].copy()
                d = robust(lambda q, act: resid(np.r_[dr, q], act), d[3:].copy(), lambda q: np.r_[dr, q], np.ones(len(oi), bool))
        else:
            d = robust(lambda q, act: resid(np.r_[0.0, 0.0, 0.0, q], act), np.zeros(3), lambda q: np.r_[0.0, 0.0, 0.0, q],
                       np.ones(len(oi), bool))
        self.stats["factors"] += int(active.sum())
        if not active.any():
            return False
        C_new = Cof(d)
        T_last = nodes[-1].T_slam
        if np.linalg.norm((C_new @ T_last)[:3, 3] - (self.C @ T_last)[:3, 3]) > cfg.max_jump_m:
            self.stats["refused_jumps"] += 1
            return False
        self.C = C_new
        self._last_T = T_last
        return True

    # ── graph ───────────────────────────────────────────────────────────────
    def _solve(self) -> bool:
        cfg, nodes = self.cfg, self.nodes
        N = len(nodes)
        Rl = np.stack([n.x[:3, :3] for n in nodes])
        tl = np.stack([n.x[:3, 3] for n in nodes])
        # odometry between consecutive nodes
        Z = [inv(nodes[i].T_slam) @ nodes[i + 1].T_slam for i in range(N - 1)]
        ZR = np.stack([z[:3, :3] for z in Z]) if Z else np.zeros((0, 3, 3))
        Zt = np.stack([z[:3, 3] for z in Z]) if Z else np.zeros((0, 3))
        so_t = np.array([cfg.sig_odo_t0 + cfg.sig_odo_td * np.linalg.norm(z[:3, 3]) for z in Z])
        so_r = np.array([(cfg.sig_odo_r0 * DEG) + cfg.sig_odo_rd * np.linalg.norm(rotvec(z[:3, :3])) for z in Z])
        # object factors
        oi, LR, Lt, MR, Mt, sym = [], [], [], [], [], []
        for i, n in enumerate(nodes):
            for lid, name, T_wl, T_co in n.obs:
                oi.append(i); LR.append(T_wl[:3, :3]); Lt.append(T_wl[:3, 3]); MR.append(T_co[:3, :3]); Mt.append(T_co[:3, 3])
                sym.append((name in cfg.symmetric) or not cfg.use_rotation)
        oi = np.array(oi); LR = np.stack(LR); Lt = np.stack(Lt); MR = np.stack(MR); Mt = np.stack(Mt)
        rw = np.where(np.array(sym), 0.0, 1.0 / (cfg.sig_obs_r * DEG))
        st = cfg.sig_obs_t0 + cfg.sig_obs_tz * np.abs(Mt[:, 2])
        active = np.ones(len(oi), bool)

        def poses(d):
            d = d.reshape(N, 6)
            dR = Rotation.from_rotvec(d[:, :3]).as_matrix()
            R = dR @ Rl
            t = np.einsum("nij,nj->ni", dR, tl) + d[:, 3:]
            return R, t

        def resid(d, act):
            R, t = poses(d)
            out = [d[:6] * np.r_[[1 / (cfg.sig_prior_r * DEG)] * 3, [1 / cfg.sig_prior_t] * 3]]
            if N > 1:
                # E = (x_i^-1 x_j)^-1 Z  ->  R_E = R_j^T R_i Z_R,  t_E = R_j^T (R_i Z_t + t_i - t_j)
                Ri, Rj, ti, tj = R[:-1], R[1:], t[:-1], t[1:]
                RE = np.einsum("nji,njk,nkl->nil", Rj, Ri, ZR)
                tE = np.einsum("nji,nj->ni", Rj, np.einsum("nij,nj->ni", Ri, Zt) + ti - tj)
                out.append((Rotation.from_matrix(RE).as_rotvec() / so_r[:, None]).ravel())
                out.append((tE / so_t[:, None]).ravel())
            # predicted T_cam_obj = x^-1 T_w_L
            Ro, to = R[oi], t[oi]
            Rp = np.einsum("nji,njk->nik", Ro, LR)
            tp = np.einsum("nji,nj->ni", Ro, Lt - to)
            et = (tp - Mt) / st[:, None]
            er = Rotation.from_matrix(np.einsum("nji,njk->nik", MR, Rp)).as_rotvec() * rw[:, None]
            m = act[:, None].astype(float)
            out.append((et * m).ravel())
            out.append((er * m).ravel())
            return np.concatenate(out)

        n_res = 6 + 6 * (N - 1) + 6 * len(oi)
        S = lil_matrix((n_res, 6 * N), dtype=int)
        S[0:6, 0:6] = 1
        r = 6
        for i in range(N - 1):
            S[r:r + 6, 6 * i:6 * i + 12] = 1
            r += 6
        for k, i in enumerate(oi):
            S[r + 3 * k:r + 3 * k + 3, 6 * i:6 * i + 6] = 1
        r2 = r + 3 * len(oi)
        for k, i in enumerate(oi):
            S[r2 + 3 * k:r2 + 3 * k + 3, 6 * i:6 * i + 6] = 1

        d = np.zeros(6 * N)
        for _ in range(2):
            sol = least_squares(resid, d, args=(active,), jac_sparsity=S, loss="huber", f_scale=cfg.huber,
                                method="trf", max_nfev=30, x_scale=1.0)
            d = sol.x
            R, t = poses(d)
            Ro, to = R[oi], t[oi]
            tp = np.einsum("nji,nj->ni", Ro, Lt - to)
            e = np.linalg.norm((tp - Mt) / st[:, None], axis=1)
            bad = active & (e > cfg.reject)
            if not bad.any():
                break
            active &= ~bad
            self.stats["rejected"] += int(bad.sum())
        self.stats["factors"] += int(active.sum())
        if not active.any():
            return False
        R, t = poses(d)
        x_new = np.tile(np.eye(4), (N, 1, 1))
        x_new[:, :3, :3] = R
        x_new[:, :3, 3] = t
        C_new = x_new[-1] @ inv(nodes[-1].T_slam)
        T_last = nodes[-1].T_slam
        if np.linalg.norm((C_new @ T_last)[:3, 3] - (self.C @ T_last)[:3, 3]) > cfg.max_jump_m:
            self.stats["refused_jumps"] += 1
            return False
        for n, x in zip(nodes, x_new):
            n.x = x
        self.C = C_new
        return True

    def summary(self):
        s = dict(self.stats)
        ms = np.array(s.pop("solve_ms")) if self.stats["solve_ms"] else np.zeros(1)
        s.update(solve_ms_median=float(np.median(ms)), solve_ms_p95=float(np.percentile(ms, 95)), solve_ms_max=float(ms.max()))
        return s
