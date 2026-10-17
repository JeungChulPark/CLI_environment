#!/usr/bin/env python3
"""hub_anchor.py — hub.py with the object anchor switched on. hub.py is not modified: this wrapper swaps the
object memory for one that, for every SAM-6D result, first corrects the camera pose with the objects already in
the Object Pose Map (obj_anchor.ObjectAnchor) and then registers the result with the corrected pose; the display
draws the map with the corrected camera pose. Everything else (CLI, lanes, viewer, logs) is hub.py's.

    python objpose/pc/hub_anchor.py [--anchor-set key=value ...] [--anchor-off] <hub.py options>

Extra outputs in --out:  anchor_log.jsonl (one line per SAM-6D result: solve time, correction), anchor_summary.json
"""
from __future__ import annotations
import collections

import copy
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hub as H                                            # noqa: E402
from memory import ObjectMemory, SamFrame                 # noqa: E402
from fusion import inv_se3                                # noqa: E402
from obj_anchor import ObjectAnchor, AnchorConfig         # noqa: E402
from anchor_memory import CorrectedSlamMemory             # noqa: E402

KEEP = ("active", "lost", "remembered")
CFG = AnchorConfig()
ENABLED = True
MEMORIES: list = []
# "slam" (default since the 2026-09-25 fixes): the map stays the keyframe-anchored SLAM map, anchors are frozen relative
# to their keyframe, the correction is reset when SLAM re-optimises its map. "own": the first version (map fed with the
# corrected poses, anchors frozen in world coordinates).
MAP_MODE = "slam"
RESET_ON_LOOP = True
ICP_HZ = 0.0            # depth-ICP camera-object observations per second between SAM-6D results (0 = off)
JUMP_M = 0.05           # a SLAM pose moving more than this between two frames (>1.5 m/s) is a map jump, absorbed when
                        # loop_blend_s > 0 (display stays continuous, converges over loop_blend_s)


class AnchoredMemory(CorrectedSlamMemory):
    def __init__(self, fusion, K, img_size, depth_check=None, **kw):
        super().__init__(fusion, K, img_size, depth_check=depth_check, **kw)
        self.anchor = ObjectAnchor(copy.deepcopy(CFG))
        self.anchor_lock = threading.RLock()      # results thread, depth-ICP thread, pose thread, display
        self.log_path: Path | None = None
        self._log = None
        self._log_lock = threading.Lock()
        MEMORIES.append(self)
        if MAP_MODE in ("slam", "slamcorr"):
            poses = fusion.poses

            def attach(T_w, t_ns):
                kf = poses.anchor_at(fusion.slam_t(t_ns))
                T_w_kf, _ = poses.kfs.resolve(kf) if kf is not None else (None, None)
                return ("w", T_w) if T_w_kf is None else ("kf", kf, inv_se3(T_w_kf) @ T_w)

            def resolve(h):
                if h[0] == "w":
                    return h[1]
                T_w_kf, _ = poses.kfs.resolve(h[1])
                return None if T_w_kf is None else T_w_kf @ h[2]
            self.anchor.attach, self.anchor.resolve = attach, resolve
        view = fusion.view

        def corrected_view(t_ns, exact_window_ns=20_000_000):
            T_ws, status, rows = view(t_ns, exact_window_ns)
            if T_ws is not None and ENABLED:
                with self.anchor_lock:
                    T_ws = self.anchor.correct(T_ws, fusion.slam_t(t_ns))
                for r in rows:
                    r["T_cam_obj"] = inv_se3(T_ws) @ r["T_w_obj"]
            return T_ws, status, rows

        fusion.view = corrected_view

    def add_frame(self, t_ns: int, dets: list, depth=None):
        T_w_sam, _ = self.fusion.T_w_sam(t_ns)
        if T_w_sam is None or not ENABLED:
            return super().add_frame(t_ns, dets, depth)
        if MAP_MODE in ("slam", "slamcorr"):
            return self._add_frame_slam(t_ns, T_w_sam, dets, depth)
        t0 = time.perf_counter()
        rows, dd = [], []
        for name, score, R, t_mm in dets:
            T = np.eye(4)
            T[:3, :3] = np.asarray(R, np.float64).reshape(3, 3)
            T[:3, 3] = np.asarray(t_mm, np.float64) / 1000.0
            rows.append((name, float(score), T))
            dd.append((name, T))
        lms = [(m["object_id"], m["name"], m["T_w_obj"], m["n_obs"]) for m in self.landmarks() if m["status"] in KEEP]
        n_upd = self.anchor.stats["updates"]
        self.anchor.now_ns = self.fusion.poses.latest_t()   # arrival time on the SLAM clock (for blend_s)
        x = self.anchor.observe(int(t_ns), T_w_sam, dd, lms)
        ms = (time.perf_counter() - t0) * 1e3
        with self._lock:
            self.frames.append(SamFrame(int(t_ns), None, x, rows, depth))
            self.frames.sort(key=lambda f: f.t_ns)
        self._write({"t_ns": int(t_ns), "wall": time.time(), "anchor_ms": round(ms, 2),
                     "solve_ms": round(self.anchor.stats["solve_ms"][-1], 2) if self.anchor.stats["updates"] > n_upd else None,
                     "updated": self.anchor.stats["updates"] > n_upd, "n_dets": len(dets), "n_landmarks": len(lms),
                     "shift_mm": round(float(np.linalg.norm(x[:3, 3] - T_w_sam[:3, 3])) * 1000, 1)})

    def _add_frame_slam(self, t_ns, T_w_sam, dets, depth):
        """map: unchanged keyframe-anchored memory; the anchor only corrects the camera pose used for display."""
        t0 = time.perf_counter()
        lms = [(m["object_id"], m["name"], m["T_w_obj"], m["n_obs"]) for m in self.landmarks() if m["status"] in KEEP]
        dd = []
        for name, score, R, t_mm in dets:
            T = np.eye(4)
            T[:3, :3] = np.asarray(R, np.float64).reshape(3, 3)
            T[:3, 3] = np.asarray(t_mm, np.float64) / 1000.0
            dd.append((name, T))
        with self.anchor_lock:
            n_upd = self.anchor.stats["updates"]
            self.anchor.now_ns = self.fusion.poses.latest_t()   # arrival time on the SLAM clock (for blend_s)
            x = self.anchor.observe(int(t_ns), T_w_sam, dd, lms)
            C_reg = self.anchor.C_applied.copy()
        ms = (time.perf_counter() - t0) * 1e3
        # slamcorr: the frame is registered in the map at the corrected camera pose too (slam: SLAM pose, display only)
        super().add_frame(t_ns, dets, depth, C=C_reg if MAP_MODE == "slamcorr" else None)
        self._write({"t_ns": int(t_ns), "wall": time.time(), "anchor_ms": round(ms, 2),
                     "solve_ms": round(self.anchor.stats["solve_ms"][-1], 2) if self.anchor.stats["updates"] > n_upd else None,
                     "updated": self.anchor.stats["updates"] > n_upd, "n_dets": len(dets), "n_landmarks": len(lms),
                     "shift_mm": round(float(np.linalg.norm(x[:3, 3] - T_w_sam[:3, 3])) * 1000, 1)})

    def _write(self, row):
        if self.log_path is None:
            return
        with self._log_lock:
            if self._log is None:
                self._log = open(self.log_path, "a")
            self._log.write(json.dumps(row) + "\n")
            self._log.flush()


H.ObjectMemory = AnchoredMemory
_init_fusion = H.Lane.init_fusion


def init_fusion(self, *a, **kw):
    _init_fusion(self, *a, **kw)
    if isinstance(self.memory, AnchoredMemory):
        self.memory.log_path = Path(self.hub.out) / f"anchor_log{self.sfx}.jsonl"


H.Lane.init_fusion = init_fusion
_remember_depth = H.Hub.remember_depth


def remember_depth(self, t_ns, depth):
    _remember_depth(self, t_ns, depth)
    if depth is not None:
        # recent aligned depth frames sent to SAM-6D (for depth ICP). Several, not only the newest: with a SAM clock
        # lag (sam_tau_s < 0) the newest frame's SLAM pose is always still to come
        if not hasattr(self, "_icp_depths"):
            self._icp_depths = collections.deque(maxlen=30)
        self._icp_depths.append((int(t_ns), depth))


H.Hub.remember_depth = remember_depth


def icp_loop(lane):
    """depth ICP between SAM-6D results: newest depth frame, map objects at the corrected camera pose, their shift
    in the camera -> camera-object observations for the anchor (never registered in the map)."""
    from depth_icp import DepthICP
    hub, mem, fus = lane.hub, lane.memory, lane.fusion
    while not hasattr(hub, "extents") and not hub.stop.is_set():   # lanes are set up before the hub loads the CAD list
        time.sleep(0.1)
    models = {}
    for nm in hub.extents:
        f = H.REPO / "sam6d_realtime/assets/model_points" / f"{nm}.npy"
        if f.exists():
            q = np.load(f).astype(np.float64) / 1000.0
            models[nm] = q[np.random.default_rng(0).choice(len(q), min(1500, len(q)), replace=False)]
    icp = DepthICP(models)
    last = None
    period = 1.0 / ICP_HZ
    skip = {"no_depth": 0, "same_depth": 0, "pose_not_yet": 0, "no_pose": 0, "run": 0}
    t_rep = time.time()
    while not hub.stop.is_set():
        time.sleep(period)
        if time.time() - t_rep > 10:                      # why ICP did (not) run, every 10 s
            H.log(f"[anchor] depth ICP loop: {skip}"); t_rep = time.time()
        buf = list(getattr(hub, "_icp_depths", ()))
        if not buf:
            skip["no_depth"] += 1
            continue
        t_now = lane.poses.latest_t()
        d = next((x for x in reversed(buf) if fus.slam_t(x[0]) <= t_now), None)   # newest frame whose SLAM pose is in
        if d is None:
            skip["pose_not_yet"] += 1
            continue
        if d[0] == last:
            skip["same_depth"] += 1
            continue
        t_d, depth = d
        last = t_d
        T_ws, _ = fus.T_w_sam(t_d)
        if T_ws is None:
            skip["no_pose"] += 1
            continue
        skip["run"] += 1
        t0 = time.perf_counter()
        dm = np.asarray(depth, np.float64) / 1000.0
        lms = [(m["object_id"], m["name"], m["T_w_obj"], m["n_obs"]) for m in mem.landmarks()
               if m["status"] in KEEP and m["n_obs"] >= 5 and m["name"] in models]
        with mem.anchor_lock:
            Tc = mem.anchor.correct(T_ws, fus.slam_t(t_d))
        dd = []
        for _, nm, Tw, _ in lms:
            Tco = inv_se3(Tc) @ Tw
            r = icp.align(Tco, nm, dm, hub.K)
            if r is None or r["inlier"] < 0.5 or np.linalg.norm(r["shift"]) > 0.25:
                continue
            T2 = Tco.copy(); T2[:3, 3] += r["shift"]
            dd.append((nm, T2))
        if dd:
            with mem.anchor_lock:
                mem.anchor.now_ns = lane.poses.latest_t()
                mem.anchor.observe(int(t_d), T_ws, dd, lms, src="icp")
        mem._write({"t_ns": int(t_d), "wall": time.time(), "icp_ms": round((time.perf_counter() - t0) * 1e3, 2),
                    "icp_objects": len(lms), "icp_aligned": len(dd)})


_init_fusion2 = H.Lane.init_fusion


def init_fusion_icp(self, *a, **kw):
    _init_fusion2(self, *a, **kw)
    if ICP_HZ > 0 and ENABLED and isinstance(self.memory, AnchoredMemory):
        threading.Thread(target=icp_loop, args=(self,), daemon=True, name=f"icp-{self.name}").start()


H.Lane.init_fusion = init_fusion_icp
_on_msg = H.Lane.on_msg


def on_msg(self, m):
    prev = getattr(self, "_jprev", None)
    _on_msg(self, m)
    mem = self.memory if isinstance(self.memory, AnchoredMemory) else None
    if mem is not None and m.get("type") == "pose" and ENABLED:
        t = int(m["t_ns"])
        T, _ = self.poses.at(t)
        if T is not None:
            if (CFG.loop_blend_s > 0 and prev is not None and 0 < t - prev[0] < 100_000_000
                    and np.linalg.norm(T[:3, 3] - prev[1][:3, 3]) > JUMP_M):
                with mem.anchor_lock:
                    mem.anchor.reset_smooth(prev[1] @ self.X, T @ self.X, t)
                mem._write({"t_ns": t, "wall": time.time(), "jump_absorbed_mm": round(float(np.linalg.norm(T[:3, 3] - prev[1][:3, 3])) * 1000, 1)})
            self._jprev = (t, T)
    if (RESET_ON_LOOP and m.get("type") == "kf_update" and m.get("reason") not in ("periodic", "end")
            and mem is not None):
        with mem.anchor_lock:
            if CFG.loop_blend_s > 0:     # keep the display continuous, converge to the new map over loop_blend_s
                mem.anchor.reset_smooth(np.eye(4), np.eye(4), self.poses.latest_t())
            else:
                mem.anchor.reset()
        if MAP_MODE == "slamcorr":          # anchors / frames registered with the old corrections: drop the corrections
            a_ = self.memory.anchor
            a_.frozen.clear(); a_.views.clear(); a_.clusters.clear()
            self.memory.new_epoch()
        self.memory._write({"t_ns": int(m.get("t_ns", 0)), "wall": time.time(), "reset": m.get("reason")})


H.Lane.on_msg = on_msg
_write_summary = H.Hub.write_summary


def write_summary(self):
    _write_summary(self)
    out = {"enabled": ENABLED, "config": {k: (sorted(v) if isinstance(v, frozenset) else v) for k, v in vars(CFG).items()},
           "lanes": [m.anchor.summary() for m in MEMORIES]}
    (Path(self.out) / "anchor_summary.json").write_text(json.dumps(out, indent=1))


H.Hub.write_summary = write_summary


def main():
    global ENABLED, MAP_MODE, RESET_ON_LOOP, ICP_HZ
    argv, rest = sys.argv[1:], []
    i = 0
    while i < len(argv):
        if argv[i] == "--anchor-set":
            k, v = argv[i + 1].split("=", 1)
            cur = getattr(CFG, k)
            setattr(CFG, k, (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
            i += 2
        elif argv[i] == "--anchor-map":
            MAP_MODE = argv[i + 1]
            i += 2
        elif argv[i] == "--anchor-no-reset":
            RESET_ON_LOOP = False
            i += 1
        elif argv[i] == "--anchor-icp-hz":
            ICP_HZ = float(argv[i + 1])
            i += 2
        elif argv[i] == "--anchor-off":
            ENABLED = False
            i += 1
        else:
            rest.append(argv[i])
            i += 1
    sys.argv = [sys.argv[0]] + rest
    H.log(f"[anchor] object anchor {'ON' if ENABLED else 'OFF'} map={MAP_MODE} reset_on_loop={RESET_ON_LOOP} icp_hz={ICP_HZ}: {vars(CFG)}")
    H.main()


if __name__ == "__main__":
    main()
