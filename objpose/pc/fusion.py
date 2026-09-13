"""fusion.py — time sync + interpolation between SLAM poses and SAM-6D object estimates.

Frames (all 4x4, metres):
    T_w_slam(t)   SLAM camera in the SLAM map, streamed from ORB-SLAM3 on the Mac
    X             T_slam_sam, the rig extrinsic (SAM camera in SLAM-camera coords)
    T_sam_obj     SAM-6D estimate in the SAM camera, at the SAM frame's timestamp s

Sync: every timestamp is on the SLAM host frame clock. SLAM runs at camera rate, SAM-6D
every second or so, and their frames never share a timestamp, so SLAM poses are
SE(3)-interpolated to s.

Keyframe anchoring (slam_comparison_report/realtime_30fps/ORB_POSE_CONSISTENCY_FOR_SAM6D.md):
a live ORB-SLAM3 pose carries the drift accumulated before the next loop closure (up to
~11 cm here), and an object frozen in world coordinates keeps that error after the map is
corrected. So nothing is stored in world coordinates:

    per frame      T_kf_c   = T_w_kf(at tracking time)^-1 . T_w_c(live)
    per estimate   T_kf_obj = T_w_kf(now)^-1 . T_w_c(s) . X . T_sam_obj
    whenever used  T_w_obj  = T_w_kf(latest) . T_kf_obj,   T_w_c(t) = T_w_kf(latest) . T_kf_c(t)

Keyframe poses are refreshed from every pose message and from periodic / loop-closure
`kf_update` messages, so a correction re-positions every object anchored to the moved
keyframes. When ORB-SLAM3 culls a keyframe, anything anchored to it is moved to the nearest
surviving keyframe using the last pose set in which both existed.

Between two SAM-6D reports an object is held static in the (corrected) world and carried
into the moving SAM camera by the SLAM motion — the interpolated estimate.
Streamers without keyframe fields fall back to world anchoring (anchor id None).
"""
from __future__ import annotations

import bisect
import threading
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

TRACKING_OK = ("OK", "OK_KLT")


def inv_se3(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def interp_se3(Ta: np.ndarray, Tb: np.ndarray, alpha: float) -> np.ndarray:
    rots = Rotation.from_matrix(np.stack([Ta[:3, :3], Tb[:3, :3]]))
    out = np.eye(4)
    out[:3, :3] = Slerp([0.0, 1.0], rots)(alpha).as_matrix()
    out[:3, 3] = (1 - alpha) * Ta[:3, 3] + alpha * Tb[:3, 3]
    return out


def mat(v) -> np.ndarray:
    a = np.asarray(v, np.float64)
    if a.size == 12:
        out = np.eye(4)
        out[:3, :] = a.reshape(3, 4)
        return out
    return a.reshape(4, 4)


class KeyframeMap:
    """Latest T_w_kf per keyframe id, plus re-anchoring of culled keyframes."""

    def __init__(self):
        self.T: dict[int, np.ndarray] = {}
        self.map_of: dict[int, int] = {}
        self.redirect: dict[int, tuple[int, np.ndarray]] = {}   # culled id -> (survivor, T_survivor_culled)
        self.updates = 0
        self.culled = 0
        self.max_correction_m = 0.0
        self.snapshot: dict[int, np.ndarray] = {}
        self.snapshot_map = None

    def set_pose(self, kf: int, T_w_kf: np.ndarray, map_id):
        self.T[kf] = T_w_kf
        self.map_of[kf] = map_id

    def apply_update(self, map_id, kfs):
        new = {int(r[0]): mat(r[1:]) for r in kfs}
        # re-anchor culled keyframes with the previous full update: one consistent snapshot of the
        # map, unlike poses picked up from individual pose messages at different drift levels
        prev = self.snapshot if self.snapshot_map == map_id else {}
        old_same_map = {k: prev.get(k, T) for k, T in self.T.items() if self.map_of.get(k) == map_id}
        for k, T in new.items():
            if k in self.T:
                self.max_correction_m = max(self.max_correction_m, float(np.linalg.norm(T[:3, 3] - self.T[k][:3, 3])))
        gone = [k for k in old_same_map if k not in new]
        survivors = [k for k in old_same_map if k in new and (not prev or k in prev)] or \
            [k for k in old_same_map if k in new]
        if gone and survivors:
            P = np.array([old_same_map[k][:3, 3] for k in survivors])
            for k in gone:
                j = survivors[int(np.argmin(np.linalg.norm(P - old_same_map[k][:3, 3], axis=1)))]
                # expressed with the previous pose set, in which both keyframes still existed
                self.redirect[k] = (j, inv_se3(old_same_map[j]) @ old_same_map[k])
                del self.T[k]
                self.culled += 1
        for k, T in new.items():
            self.set_pose(k, T, map_id)
        self.snapshot, self.snapshot_map = new, map_id
        self.updates += 1

    def resolve(self, kf: int):
        """(T_w_kf, map_id) following cull redirects; None if unknown."""
        rel = np.eye(4)
        seen = 0
        while kf not in self.T and kf in self.redirect and seen < 64:
            j, T_j_k = self.redirect[kf]
            rel = T_j_k @ rel
            kf = j
            seen += 1
        if kf not in self.T:
            return None, None
        return self.T[kf] @ rel, self.map_of.get(kf)


@dataclass
class FrameSample:
    t_ns: int
    T_w_c: np.ndarray          # live pose as tracked
    kf: int | None
    T_kf_c: np.ndarray | None
    map_id: int | None


class PoseBuffer:
    """Time-ordered tracking-OK SLAM frames, corrected through the keyframe map on read."""

    def __init__(self, max_gap_s: float = 0.25):
        self.max_gap_ns = int(max_gap_s * 1e9)
        self._t: list[int] = []
        self._s: list[FrameSample] = []
        self.kfs = KeyframeMap()
        self._lock = threading.RLock()
        self.last_state = "NO_DATA"
        self.last_t_ns = 0
        self.count_ok = 0
        self.count_all = 0
        self.map_changes = 0
        self.map_id = None
        self.loop_events_ns: list[int] = []

    def add(self, t_ns: int, state: str, T_wc, map_id=None, kf=None, T_w_kf=None, kf_map_id=None,
            map_changed=False):
        with self._lock:
            self.count_all += 1
            self.last_state, self.last_t_ns = state, t_ns
            if map_id is not None and map_id != self.map_id:
                # a new Atlas map is a new world frame: earlier poses are not comparable.
                # (ORB-SLAM3's map_changed flag also fires on loop closure inside the same
                # map, which keeps the frame, so it must not clear the buffer.)
                if self.map_id is not None:
                    self.map_changes += 1
                    self._t.clear(); self._s.clear()
                self.map_id = map_id
            if map_changed:
                self.loop_events_ns.append(t_ns)
            if state not in TRACKING_OK or T_wc is None:
                return
            self.count_ok += 1
            T = mat(T_wc)
            T_kf_c = None
            if kf is not None and T_w_kf is not None:
                Tk = mat(T_w_kf)
                self.kfs.set_pose(int(kf), Tk, kf_map_id if kf_map_id is not None else map_id)
                T_kf_c = inv_se3(Tk) @ T
            k = bisect.bisect_left(self._t, t_ns)
            self._t.insert(k, t_ns)
            self._s.insert(k, FrameSample(t_ns, T, None if T_kf_c is None else int(kf), T_kf_c, map_id))

    def refine(self, t_ns: int, T_wc) -> bool:
        """replace the live pose of the frame stamped t_ns (e.g. a streamer's delayed smoothed pose)"""
        with self._lock:
            k = bisect.bisect_left(self._t, t_ns)
            if k >= len(self._t) or self._t[k] != t_ns:
                return False
            s = self._s[k]
            T = mat(T_wc)
            if s.kf is not None and s.T_kf_c is not None:
                s.T_kf_c = inv_se3(s.T_w_c @ inv_se3(s.T_kf_c)) @ T     # same keyframe pose as at arrival
            s.T_w_c = T
            return True

    def apply_kf_update(self, map_id, kfs):
        with self._lock:
            self.kfs.apply_update(map_id, kfs)

    def corrected(self, s: FrameSample) -> np.ndarray:
        if s.kf is None:
            return s.T_w_c
        T_w_kf, _ = self.kfs.resolve(s.kf)
        return s.T_w_c if T_w_kf is None else T_w_kf @ s.T_kf_c

    def latest_t(self) -> int:
        with self._lock:
            return self._t[-1] if self._t else 0

    def bracket(self, t_ns: int):
        """(sample_a, sample_b, alpha) around t, or None when outside / across a gap."""
        with self._lock:
            if not self._t:
                return None
            k = bisect.bisect_left(self._t, t_ns)
            if k < len(self._t) and self._t[k] == t_ns:
                return self._s[k], self._s[k], 0.0
            if 0 < k < len(self._t):
                ta, tb = self._t[k - 1], self._t[k]
                if tb - ta <= self.max_gap_ns:
                    return self._s[k - 1], self._s[k], (t_ns - ta) / (tb - ta)
            return None

    def at(self, t_ns: int):
        """(corrected T_w_slam, status) where status is 'interp' | 'exact' | None when unavailable."""
        with self._lock:
            br = self.bracket(t_ns)
            if br is None:
                return None, None
            a, b, u = br
            if a is b:
                return self.corrected(a).copy(), "exact"
            return interp_se3(self.corrected(a), self.corrected(b), u), "interp"

    def anchor_at(self, t_ns: int):
        """keyframe to anchor an estimate at time t: the nearer bracketing frame's reference."""
        with self._lock:
            br = self.bracket(t_ns)
            if br is None:
                return None
            a, b, u = br
            return (a if u < 0.5 else b).kf

    def near_loop_closure(self, t_ns: int, window_ns: int = 1_000_000_000) -> bool:
        with self._lock:
            return any(abs(t_ns - e) <= window_ns for e in self.loop_events_ns)

    def positions(self, step: int = 3) -> list:
        with self._lock:
            return [self.corrected(s)[:3, 3].tolist() for s in self._s[::step]]


@dataclass
class Observation:
    t_ns: int
    kf: int | None             # None = world-anchored (streamer without keyframe fields)
    T_anchor_obj: np.ndarray
    score: float
    near_loop: bool


@dataclass
class ObjectTrack:
    name: str
    current: Observation
    est_count: int = 1
    history: list = field(default_factory=list)   # Observations (bounded)


class Fusion:
    def __init__(self, X_slam_sam: np.ndarray, poses: PoseBuffer):
        self.X = np.asarray(X_slam_sam, np.float64)
        self.poses = poses
        self.objects: dict[str, ObjectTrack] = {}
        self.unplaced = 0
        self.near_loop_estimates = 0
        self._lock = threading.Lock()

    def T_w_sam(self, t_ns: int):
        T, status = self.poses.at(t_ns)
        return (T @ self.X, status) if T is not None else (None, None)

    def world(self, ob: Observation):
        if ob.kf is None:
            return ob.T_anchor_obj
        T_w_kf, _ = self.poses.kfs.resolve(ob.kf)
        return None if T_w_kf is None else T_w_kf @ ob.T_anchor_obj

    def add_estimate(self, t_ns: int, name: str, R, t_mm, score: float) -> dict:
        T_sam_obj = np.eye(4)
        T_sam_obj[:3, :3] = np.asarray(R, np.float64).reshape(3, 3)
        T_sam_obj[:3, 3] = np.asarray(t_mm, np.float64) / 1000.0
        T_ws, status = self.T_w_sam(t_ns)
        if T_ws is None:
            self.unplaced += 1
            return {"name": name, "placed": False, "reason": "no SLAM pose at SAM frame time"}
        T_w_obj = T_ws @ T_sam_obj
        kf = self.poses.anchor_at(t_ns)
        T_w_kf, _ = self.poses.kfs.resolve(kf) if kf is not None else (None, None)
        if T_w_kf is None:
            kf, T_anchor = None, T_w_obj
        else:
            T_anchor = inv_se3(T_w_kf) @ T_w_obj
        near = self.poses.near_loop_closure(t_ns)
        self.near_loop_estimates += int(near)
        ob = Observation(t_ns, kf, T_anchor, float(score), near)
        with self._lock:
            tr = self.objects.get(name)
            if tr is None:
                tr = ObjectTrack(name, ob)
                self.objects[name] = tr
            else:
                tr.est_count += 1
                # an estimate within 1 s of a loop closure does not replace a clean anchor
                if t_ns >= tr.current.t_ns and (not near or tr.current.near_loop):
                    tr.current = ob
            tr.history.append(ob)
            del tr.history[:-500]
        return {"name": name, "placed": True, "slam": status, "T_w_obj": T_w_obj, "kf": kf, "near_loop": near}

    def view(self, t_ns: int, exact_window_ns: int = 20_000_000):
        """Objects as seen by the SAM camera at display time t (all in the corrected map)."""
        T_ws, status = self.T_w_sam(t_ns)
        out = []
        with self._lock:
            tracks = list(self.objects.values())
        for tr in tracks:
            T_w_obj = self.world(tr.current)
            if T_w_obj is None:
                continue
            row = {"name": tr.name, "score": tr.current.score, "T_w_obj": T_w_obj,
                   "age_s": (t_ns - tr.current.t_ns) / 1e9, "est_count": tr.est_count,
                   "anchor_kf": tr.current.kf, "near_loop": tr.current.near_loop,
                   "history": [w[:3, 3].tolist() for w in (self.world(o) for o in tr.history[-20:]) if w is not None]}
            if T_ws is not None:
                row["T_cam_obj"] = inv_se3(T_ws) @ T_w_obj
                row["source"] = "sam6d" if abs(t_ns - tr.current.t_ns) <= exact_window_ns else "slam_interp"
            else:
                row["T_cam_obj"] = None
                row["source"] = "no_slam"
            out.append(row)
        return T_ws, status, out

    def dump_observations(self):
        """every kept observation with its world position under the CURRENT keyframe poses."""
        rows = []
        with self._lock:
            tracks = list(self.objects.values())
        for tr in tracks:
            for o in tr.history:
                w = self.world(o)
                rows.append({"object": tr.name, "t_ns": o.t_ns, "kf": o.kf, "near_loop": o.near_loop,
                             "p_w_final": None if w is None else w[:3, 3].tolist()})
        return rows
