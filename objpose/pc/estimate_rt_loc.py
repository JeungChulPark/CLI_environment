#!/usr/bin/env python3
"""estimate_rt_loc.py — rig extrinsic from a shared map (map + localization runs).

ORB-SLAM3 f2000 builds an Atlas from the SLAM camera, then the SAM camera is localized
inside that same Atlas. Both trajectories are now in ONE world frame, so the rig
transform is observed directly at every SAM frame:

    X(t) = T_w_slam(t)^-1 . T_w_sam(t)

instead of being inferred through an unknown map-to-map transform (AX = ZB), which is
poorly conditioned when the cart only yaws. X is the robust (Huber IRLS) mean of X(t).

Validation, independent of the fit: 156 SAM-6D detections from the eightcircle_dark run
are placed in the SLAM map with each candidate X; a static object's world position
should not move, so the spread around its median scores the candidate.
"""
from __future__ import annotations

import argparse
import csv
import json
import sqlite3
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "integration"))
from fusion import PoseBuffer  # noqa: E402
from hub import SAM_MINUS_SLAM_CLOCK_NS, load_extrinsic  # noqa: E402
from rig_offset import proj_SO3, read_tum  # noqa: E402
from rs_session import RsSession  # noqa: E402

DATASET = Path("/home/jucpark/DeepLearning/Dataset/260826_etri_eightcircle_dark")


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def pose_buffer(ts, T, max_gap_s):
    b = PoseBuffer(max_gap_s=max_gap_s)
    for t, M in zip(ts, T):
        b.add(int(round(t * 1e9)), "OK", M)
    return b


def load_filter(csv_path: Path | None, min_tracked: int):
    """t_ns -> keep? from the streamer's frames.csv (tracked map points, VO-only flag)."""
    if csv_path is None or not csv_path.exists():
        return None, "no frames.csv filter"
    rows = list(csv.DictReader(csv_path.open()))
    cols = rows[0].keys() if rows else []
    t_col = next((c for c in cols if c in ("t_ns", "stamp_ns", "t")), None)
    trk_col = next((c for c in cols if "tracked" in c.lower()), None)
    vo_col = next((c for c in cols if c.lower() in ("vo", "mbvo", "vo_only", "is_vo")), None)
    if t_col is None:
        return None, f"frames.csv has no time column ({list(cols)})"
    ts, oks = [], []
    for r in rows:
        ok = r.get("state", "OK") in ("OK", "OK_KLT", "2")
        if trk_col and r[trk_col] not in ("", None):
            ok &= int(float(r[trk_col])) >= min_tracked
        if vo_col and r[vo_col] not in ("", None):
            ok &= int(float(r[vo_col])) == 0
        t = r[t_col]
        ts.append(int(t) if "." not in t else int(round(float(t) * 1e9)))
        oks.append(ok)
    order = np.argsort(ts)
    return (np.asarray(ts, np.int64)[order], np.asarray(oks, bool)[order]), \
        f"state OK, {trk_col}>={min_tracked}" + (f", {vo_col}==0" if vo_col else "")


def kept(keep, t_ns, tol_ns=1_000_000):
    """TUM seconds lose ns precision in a double, so match the csv stamp within 1 ms."""
    ts, oks = keep
    i = int(np.searchsorted(ts, t_ns))
    best = min((j for j in (i - 1, i) if 0 <= j < len(ts)), key=lambda j: abs(int(ts[j]) - t_ns))
    return abs(int(ts[best]) - t_ns) <= tol_ns and bool(oks[best])


def samples(map_ts, map_T, loc_ts, loc_T, tau, max_gap, keep, reverse):
    buf = pose_buffer(map_ts, map_T, max_gap)
    X, st = [], []
    for t, T_loc in zip(loc_ts, loc_T):
        t_ns = int(round(t * 1e9))
        if keep is not None and not kept(keep, t_ns):
            continue
        T_map, _ = buf.at(t_ns + int(tau * 1e9))
        if T_map is None:
            continue
        # map camera = SLAM normally; reverse = SAM built the map, SLAM was localized
        Xi = np.linalg.inv(T_map) @ T_loc if not reverse else np.linalg.inv(np.linalg.inv(T_loc) @ T_map)
        X.append(Xi)
        st.append(t)
    return np.array(X), np.array(st)


def robust_mean(Xs, iters=30):
    R = proj_SO3(Xs[:, :3, :3].mean(0))
    t = np.median(Xs[:, :3, 3], axis=0)
    w = np.ones(len(Xs))
    for _ in range(iters):
        dt = np.linalg.norm(Xs[:, :3, 3] - t, axis=1)
        dr = np.array([rot_deg(R.T @ Xi[:3, :3]) for Xi in Xs])
        # joint residual: 1 deg counts like 1 cm, Huber knee from the data
        r = dt * 100 + dr
        k = max(0.5, 1.5 * np.median(r))
        w_new = np.minimum(1.0, k / np.maximum(r, 1e-9))
        R = proj_SO3(np.einsum("n,nij->ij", w_new, Xs[:, :3, :3]) / w_new.sum())
        t = (w_new[:, None] * Xs[:, :3, 3]).sum(0) / w_new.sum()
        if np.abs(w_new - w).max() < 1e-5:
            break
        w = w_new
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, t
    dt = np.linalg.norm(Xs[:, :3, 3] - t, axis=1)
    dr = np.array([rot_deg(R.T @ Xi[:3, :3]) for Xi in Xs])
    return X, dt, dr, w


def object_spread(X, slam_traj: Path, sam_times=None):
    """median / p90 [cm] of static-object world-position spread for rig X.

    sam_times: per-frame SAM times on the same clock as slam_traj (default: association stamps)."""
    if sam_times is None:
        sam_times = RsSession(DATASET / "SAM", -SAM_MINUS_SLAM_CLOCK_NS).t_ns
    con = sqlite3.connect(f"file:{REPO}/sam6d_realtime/data/260826_eightcircle_dark/SAM_0.db3?mode=ro", uri=True)
    tid = con.execute("select id from topics where name='/camera/camera/color/image_raw'").fetchone()[0]
    conv = np.array([r[0] for r in con.execute(
        "select timestamp from messages where topic_id=? order by timestamp", (tid,))])
    dets = [json.loads(l) for l in open(REPO / "sam6d_realtime/output/eightcircle_dark_split_260913/detections.jsonl")]
    ts, T = read_tum(slam_traj)
    buf = pose_buffer(ts, T, 0.25)
    per = {}
    for d in dets:
        i = int(np.searchsorted(conv, d["stamp_ns"]))
        Tw, _ = buf.at(int(sam_times[i]))
        if Tw is None:
            continue
        M = np.eye(4)
        M[:3, :3] = np.array(d["R"]).reshape(3, 3)
        M[:3, 3] = np.array(d["t_mm"]) / 1000
        per.setdefault(d["object"], []).append((Tw @ X @ M)[:3, 3])
    s = np.concatenate([np.linalg.norm(np.array(P) - np.median(P, 0), axis=1) for P in per.values()])
    return {"n": int(len(s)), "median_cm": round(float(np.median(s)) * 100, 2),
            "p90_cm": round(float(np.percentile(s, 90)) * 100, 2)}


def main():
    loc = REPO / "objpose" / "rt" / "loc"
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--map-traj", default=str(loc / "slam_map_traj_tum.txt.optimized.txt"),
                    help="trajectory of the camera that BUILT the atlas, in the saved-atlas frame")
    ap.add_argument("--loc-traj", default=str(loc / "sam_loc_traj_tum.txt"),
                    help="trajectory of the camera localized in that atlas")
    ap.add_argument("--loc-frames-csv", default=None, help="default: <loc-traj>.frames.csv")
    ap.add_argument("--reverse", action="store_true", help="SAM built the map and SLAM was localized")
    ap.add_argument("--min-tracked", type=int, default=60)
    ap.add_argument("--max-gap", type=float, default=0.1)
    ap.add_argument("--tau-range", type=float, default=0.1)
    ap.add_argument("--live-slam-traj", default=str(REPO / "objpose" / "rt" / "slam_traj_tum.txt"),
                    help="per-frame SLAM poses like the live system consumes (for validation)")
    ap.add_argument("--out", default=str(REPO / "objpose" / "rt" / "X_slam_sam_loc.json"))
    a = ap.parse_args()

    map_ts, map_T = read_tum(a.map_traj)
    loc_ts, loc_T = read_tum(a.loc_traj)
    csv_path = Path(a.loc_frames_csv) if a.loc_frames_csv else Path(a.loc_traj + ".frames.csv")
    keep, filt = load_filter(csv_path, a.min_tracked)
    print(f"map poses {len(map_ts)}, localized poses {len(loc_ts)}; filter: {filt}")

    best = None
    for tau in np.round(np.arange(-a.tau_range, a.tau_range + 1e-9, 0.005), 3):
        Xs, st = samples(map_ts, map_T, loc_ts, loc_T, float(tau), a.max_gap, keep, a.reverse)
        if len(Xs) < 50:
            continue
        X, dt, dr, w = robust_mean(Xs)
        score = np.percentile(dt, 90) * 100 + np.percentile(dr, 90)
        if best is None or score < best["score"]:
            best = dict(tau=float(tau), X=X, dt=dt, dr=dr, w=w, st=st, Xs=Xs, score=score)
    if best is None:
        sys.exit("too few overlapping localized samples")
    X, dt, dr, st, Xs = best["X"], best["dt"], best["dr"], best["st"], best["Xs"]

    # stability: fit each quarter of the recording separately
    chunks = []
    for part in np.array_split(np.argsort(st), 4):
        if len(part) < 30:
            continue
        Xc, _, _, _ = robust_mean(Xs[part])
        chunks.append({"t_from_s": round(float(st[part].min() - st.min()), 1),
                       "t_to_s": round(float(st[part].max() - st.min()), 1), "n": int(len(part)),
                       "d_trans_cm": round(float(np.linalg.norm(Xc[:3, 3] - X[:3, 3])) * 100, 2),
                       "d_rot_deg": round(rot_deg(X[:3, :3].T @ Xc[:3, :3]), 2)})

    U, _ = load_extrinsic(DATASET / "camera_extrinsic.urdf")
    prev_path = REPO / "objpose" / "rt" / "X_slam_sam.json"
    prev = np.asarray(json.loads(prev_path.read_text())["T_slam_sam"]) if prev_path.exists() else None
    D = np.linalg.inv(U) @ X
    # reference only: most of this spread is SAM-6D's own error, so it separates rig
    # candidates weakly (on synthetic data it ranked a 4 cm-wrong X above the truth)
    valid = {}
    sam_times = None
    if "ftime" in Path(a.live_slam_traj).name:
        from clock import frame_clock
        sam_times = frame_clock(DATASET / "SAM", REPO / "objpose" / "rt" / "clock")["frame_ns"]
    for traj_name, traj in (("per_frame_slam_poses", Path(a.live_slam_traj)),
                            ("optimized_slam_poses", Path(a.live_slam_traj + ".optimized.txt"))):
        if not traj.exists():
            continue
        valid[traj_name] = {"new_localization_X": object_spread(X, traj, sam_times),
                            "urdf": object_spread(U, traj, sam_times)}
        if prev is not None:
            valid[traj_name]["previous_axzb_X"] = object_spread(prev, traj, sam_times)

    out = {
        "T_slam_sam": X.tolist(),
        "source": "ORB-SLAM3 f2000 shared map: SLAM builds atlas, SAM localized; robust mean of T_w_slam^-1 T_w_sam",
        "map_traj": a.map_traj, "loc_traj": a.loc_traj, "reverse": a.reverse, "filter": filt,
        "tau_s": best["tau"], "n_samples": int(len(Xs)),
        "coverage_of_localized_poses": round(len(Xs) / max(1, len(loc_ts)), 3),
        "residual": {"trans_median_cm": round(float(np.median(dt)) * 100, 2),
                     "trans_p90_cm": round(float(np.percentile(dt, 90)) * 100, 2),
                     "rot_median_deg": round(float(np.median(dr)), 2),
                     "rot_p90_deg": round(float(np.percentile(dr, 90)), 2),
                     "inlier_weight_mean": round(float(best["w"].mean()), 3)},
        "quarters": chunks,
        "vs_urdf": {"translation_diff_cm": round(float(np.linalg.norm(D[:3, 3])) * 100, 2),
                    "rotation_diff_deg": round(rot_deg(D[:3, :3]), 2)},
        "object_consistency_check": valid,
    }
    if prev is not None:
        P = np.linalg.inv(prev) @ X
        out["vs_previous_axzb"] = {"translation_diff_cm": round(float(np.linalg.norm(P[:3, 3])) * 100, 2),
                                   "rotation_diff_deg": round(rot_deg(P[:3, :3]), 2)}
    Path(a.out).write_text(json.dumps(out, indent=1))
    r = out["residual"]
    print(f"X t = {np.round(X[:3, 3], 4).tolist()} m | tau {best['tau']:+.3f} s | n {len(Xs)} "
          f"({out['coverage_of_localized_poses']:.0%} of localized poses)")
    print(f"residual: trans med {r['trans_median_cm']} / p90 {r['trans_p90_cm']} cm, "
          f"rot med {r['rot_median_deg']} / p90 {r['rot_p90_deg']} deg")
    print("quarters:", [(c['t_from_s'], c['d_trans_cm'], c['d_rot_deg']) for c in chunks])
    print("vs urdf:", out["vs_urdf"], "| vs previous:", out.get("vs_previous_axzb"))
    print("object spread:", valid)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
