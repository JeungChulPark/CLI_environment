"""Lay a real recording out as an RGB-D sequence with the camera trajectory of a live run of ours.

The baselines get exactly the SLAM output the proposed method had: the Mac ORB-SLAM3 (f2000) poses
logged by the hub during a live run (slam_poses.jsonl), corrected to the run's final keyframe poses
(kf_updates.jsonl 'end': T_w_c' = T_w_kf(final) . T_w_kf(at the time)^-1 . T_w_c, the same correction
the keyframe-anchored map receives; a pose whose reference keyframe was culled takes the correction of the
nearest pose in time), then moved to the recognition camera with the run's T_slam_sam.
Recognition-camera frames are read the way the hub reads them (same clock offset, depth aligned to
colour), and get the pose interpolated at their stamp (linear position, slerp rotation); frames with
no SLAM pose within 100 ms are left out.

Output: ~/DeepLearning/Dataset/real_work/<session>/{rgb,depth}/<id:06d>.png (BGR png, depth uint16 mm)
        gt/real/<scene:06d>/cameras.json  (common.seq_cameras format)

    python real_prep.py --scene 1                  # scene numbers: common.REAL_SESSIONS
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from common import GT_DIR, REAL_SESSIONS

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose" / "pc"))
import catalog  # noqa: E402
from clock import frame_clock  # noqa: E402
from conv_session import ConvSession  # noqa: E402
from hub import SAM_MINUS_SLAM_CLOCK_NS, load_extrinsic  # noqa: E402
from rs_session import RsSession  # noqa: E402

WORK = Path.home() / "DeepLearning/Dataset/real_work"
MAX_GAP_NS = 100_000_000


def hub_opts(session):
    args = catalog.hub_args(session, "orbslam3", 2000)
    return {args[i]: args[i + 1] for i in range(len(args) - 1) if args[i].startswith("--")}


def frame_source(session):
    """The recognition-camera reader of hub.Hub.__init__, same clock handling."""
    o = hub_opts(session)
    sam_dir, ext = o["--sam-session"], o["--extrinsic"]
    xj = json.loads(Path(ext).read_text()) if ext.endswith(".json") else {}
    if (Path(sam_dir) / "rgbd_timestamp_associations.json").exists():
        clk = frame_clock(sam_dir, REPO / "objpose" / "rt" / "clock")
        tau = (int(round(float(xj.get("tau_s", 0.0)) * 1e9))
               if "ftime" in str(xj.get("inputs", "")) or "frame clock" in str(xj.get("clock", "")) else 0)
        return RsSession(sam_dir, offset_ns=-SAM_MINUS_SLAM_CLOCK_NS, times_ns=clk["frame_ns"] + tau), True
    return ConvSession(sam_dir, offset_ns=int(round(float(xj.get("sam_tau_s", 0.0)) * 1e9))), False


def corrected_trajectory(run):
    """(t_ns, T_w_c of the SLAM camera after the final keyframe correction) of the run's tracked poses."""
    final = None
    for line in open(run / "kf_updates.jsonl"):
        d = json.loads(line)
        if d.get("map_id", 0) == 0:
            final = d
    kf = {}
    for r in final["kfs"]:
        T = np.eye(4)
        T[:3] = np.asarray(r[1:13], float).reshape(3, 4)
        kf[int(r[0])] = T
    ts, Ts, C = [], [], []
    for line in open(run / "slam_poses.jsonl"):
        p = json.loads(line)
        if not p.get("T_wc") or p.get("state") != "OK" or p.get("map_id", 0) != 0:
            continue
        k = p.get("ref_kf")
        ok = k in kf and p.get("T_w_kf") is not None and p.get("kf_map_id", 0) == 0
        ts.append(p["t_ns"]); Ts.append(np.asarray(p["T_wc"], float).reshape(4, 4))
        C.append(kf[k] @ np.linalg.inv(np.asarray(p["T_w_kf"], float).reshape(4, 4)) if ok else None)
    # a pose whose reference keyframe was culled takes the correction of the nearest pose in time that has one
    have = np.array([i for i, c in enumerate(C) if c is not None])
    moved = len(have)
    for i, c in enumerate(C):
        if c is None and len(have):
            C[i] = C[have[np.argmin(np.abs(np.asarray(ts)[have] - ts[i]))]]
    Ts = [c @ T if c is not None else T for c, T in zip(C, Ts)]
    return np.asarray(ts, np.int64), np.asarray(Ts), moved


def interp(ts, Ts, t):
    i = int(np.searchsorted(ts, t))
    if i == 0 or i == len(ts):
        j = 0 if i == 0 else len(ts) - 1
        return Ts[j] if abs(ts[j] - t) <= MAX_GAP_NS else None
    t0, t1 = ts[i - 1], ts[i]
    if min(t - t0, t1 - t) > MAX_GAP_NS:
        return None
    a = (t - t0) / max(t1 - t0, 1)
    R = Slerp([0, 1], Rotation.from_matrix([Ts[i - 1][:3, :3], Ts[i][:3, :3]]))(a).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (1 - a) * Ts[i - 1][:3, 3] + a * Ts[i][:3, 3]
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", type=int, required=True)
    ap.add_argument("--run", help="live run of ours whose trajectory is used (default: REAL_SESSIONS)")
    a = ap.parse_args()
    session, run_name = REAL_SESSIONS[a.scene]
    run = REPO / "objpose" / "output" / (a.run or run_name)
    X = np.asarray(json.load(open(run / "summary.json"))["X_slam_sam"], float).reshape(4, 4)
    ts, Ts, moved = corrected_trajectory(run)
    src, raw = frame_source(session)
    out = WORK / session
    (out / "rgb").mkdir(parents=True, exist_ok=True)
    (out / "depth").mkdir(parents=True, exist_ok=True)
    frames, skipped = [], 0
    for i in range(len(src)):
        T = interp(ts, Ts, int(src.t_ns[i]))
        if T is None:
            skipped += 1
            continue
        fid = len(frames)
        if not (out / "depth" / f"{fid:06d}.png").exists():
            fr = src.read(i, with_depth=True)
            depth = src.align(fr.depth_raw) if raw else fr.depth_raw
            cv2.imwrite(str(out / "rgb" / f"{fid:06d}.png"), fr.color_bgr)
            cv2.imwrite(str(out / "depth" / f"{fid:06d}.png"), depth.astype(np.uint16))
        frames.append({"id": fid, "T_wc": (T @ X).tolist(), "stamp_ns": int(src.t_ns[i]), "src": i})
    K = np.asarray(src.K, float).reshape(-1).tolist()
    cams = {"K": K, "width": int(src.W), "height": int(src.H), "depth_scale_mm": 1.0, "seq_dir": str(out),
            "session": session, "run": run.name, "dist": list(map(float, src.kc)), "frames": frames}
    d = GT_DIR / "real" / f"{a.scene:06d}"
    d.mkdir(parents=True, exist_ok=True)
    json.dump(cams, open(d / "cameras.json", "w"))
    print(f"{session}: {len(frames)} frames ({skipped} without a pose), {moved}/{len(ts)} poses with their own keyframe correction (rest: nearest one), "
          f"run {run.name} -> {out}")


if __name__ == "__main__":
    main()
