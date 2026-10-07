#!/usr/bin/env python3
"""compare_runs.py — compare two full runs of the object-pose system (e.g. ORB-SLAM3 vs RTAB-Map).

Both runs replay the same dataset on the same frame clock, so they can be compared frame by
frame. The two SLAM maps have different world frames: the trajectories are aligned with a
rigid (SE(3), no scale) Umeyama fit at common stamps, and that transform also carries one
run's objects into the other's map.

The corrected ("final") trajectory is built through the hub's own keyframe table, so a pose
whose reference keyframe was culled still follows the cull redirects onto the corrected map.
The few poses that reach no surviving keyframe at all are reported as `unrefined_poses` and
left out of the final-map numbers, since their coordinates never received the corrections
being compared.

    python objpose/pc/compare_runs.py objpose/output/<orb run> objpose/output/<rtab run>
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fusion import KeyframeMap, mat  # noqa: E402


def load_run(d: Path, X=None) -> dict:
    """X = T_slam_sam: when given, trajectories are expressed as SAM-camera poses (T_w_slam . X), so runs whose
    SLAM sensors differ (LiDAR vs camera) are compared on the same physical camera."""
    S = json.loads((d / "summary.json").read_text())
    poses = [json.loads(l) for l in open(d / "slam_poses.jsonl")]
    kfu = [json.loads(l) for l in open(d / "kf_updates.jsonl")] if (d / "kf_updates.jsonl").exists() else []
    # Rebuild the hub's own keyframe table by replaying every update, instead of reading the
    # last one. Local mapping culls redundant keyframes, and the table re-anchors each culled id
    # to a surviving neighbour, so a pose that referenced it still reaches the corrected map.
    # Reading only the final update and testing membership loses those redirects: it left a
    # quarter of a 260901 run's poses on their live, still-drifted coordinates and made two maps
    # that both sit within 4 cm of the LiDAR trajectory look 173 cm apart.
    kfs = KeyframeMap()
    for m in kfu:
        kfs.apply_update(m.get("map_id"), m.get("kfs", []))
    t, live, final, refined = [], [], [], []
    for m in poses:
        if m.get("state") not in ("OK", "OK_KLT") or m.get("T_wc") is None:
            continue
        T = mat(m["T_wc"])
        F = T
        T_w_kf = None
        if m.get("ref_kf") is not None and m.get("T_w_kf") is not None:
            T_w_kf, _ = kfs.resolve(int(m["ref_kf"]))
        ok = T_w_kf is not None
        if ok:
            F = T_w_kf @ np.linalg.inv(mat(m["T_w_kf"])) @ T
        if X is not None:
            T, F = T @ X, F @ X
        t.append(int(m["t_ns"]))
        live.append(T[:3, 3])
        final.append(F[:3, 3])
        refined.append(ok)
    lag = np.array([m["recv_lag_s"] for m in poses if m.get("recv_lag_s") is not None]) * 1000
    track = np.array([m.get("track_ms", np.nan) for m in poses], float)
    # with no keyframe update at all there is nothing to correct against, and `final` is `live`
    # everywhere by definition rather than by loss, so the whole run stays comparable
    ref = np.ones(len(t), bool) if not kfs.T else np.array(refined, bool)
    return {"dir": d, "S": S, "poses": poses, "t": np.array(t), "live": np.array(live), "final": np.array(final),
            "refined": ref, "lag": lag, "track": track, "kfu": kfu}


def umeyama(A, B):
    """R, t minimising |R A + t - B| (rigid, no scale)."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((B - cb).T @ (A - ca))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    return R, cb - R @ ca


def lvf(R: dict) -> dict:
    """How far the keyframe corrections moved this run's own poses, over the corrected ones."""
    d = np.linalg.norm(R["live"][R["refined"]] - R["final"][R["refined"]], axis=1) * 100
    if not len(d):
        return {"median": None, "p95": None, "max": None}
    return {"median": round(float(np.median(d)), 2), "p95": round(float(np.percentile(d, 95)), 2),
            "max": round(float(d.max()), 2)}


def obs_positions(S):
    by = {}
    for o in S.get("observations_final", []):
        if o["p_w_final"] is not None:
            by.setdefault(o["object"], []).append((o["t_ns"], np.array(o["p_w_final"])))
    return by


def spread_and_interp(S):
    by = obs_positions(S)
    spread, rows = [], []
    for L in by.values():
        L.sort(key=lambda x: x[0])
        P = np.array([p for _, p in L])
        spread += list(np.linalg.norm(P - np.median(P, 0), axis=1))
        for (ta, pa), (tb, pb) in zip(L, L[1:]):
            if tb > ta:
                rows.append(((tb - ta) / 1e9, np.linalg.norm(pb - pa)))
    return np.array(spread) * 100, np.array(rows)


def kept(S):
    return [l for l in S.get("memory_landmarks", []) if l["status"] in ("active", "lost", "remembered")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_a")
    ap.add_argument("run_b")
    ap.add_argument("--extrinsic-a", default="", help="T_slam_sam json of run A (compare on the SAM camera)")
    ap.add_argument("--extrinsic-b", default="")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    Xa = np.asarray(json.loads(Path(a.extrinsic_a).read_text())["T_slam_sam"]) if a.extrinsic_a else None
    Xb = np.asarray(json.loads(Path(a.extrinsic_b).read_text())["T_slam_sam"]) if a.extrinsic_b else None
    A, B = load_run(Path(a.run_a), Xa), load_run(Path(a.run_b), Xb)
    name = lambda R: R["S"].get("slam_source") or R["poses"][0].get("source", "?")  # noqa: E731
    out = {"runs": {}}
    for tag, R in (("A", A), ("B", B)):
        S = R["S"]
        sp, it = spread_and_interp(S)
        interp = {}
        for lo, hi in ((0, 3), (3, 10), (10, 60)):
            m = (it[:, 0] > lo) & (it[:, 0] <= hi) if len(it) else np.array([], bool)
            if m.sum():
                interp[f"{lo}-{hi}s"] = {"n": int(m.sum()), "median_cm": round(float(np.median(it[m, 1])) * 100, 2),
                                         "p90_cm": round(float(np.percentile(it[m, 1], 90)) * 100, 2)}
        ks = kept(S)
        out["runs"][tag] = {
            "dir": str(R["dir"]), "slam": S.get("slam_source"),
            "slam_poses": S["slam_poses"], "slam_ok": S["slam_ok"], "slam_dropped": S["slam_dropped"],
            "track_ms_mean": round(float(np.nanmean(R["track"])), 2), "track_ms_p99": round(float(np.nanpercentile(R["track"], 99)), 2),
            "arrival_lag_ms_median": round(float(np.median(R["lag"])), 1), "arrival_lag_ms_p99": round(float(np.percentile(R["lag"], 99)), 1),
            "loop_events": S.get("slam_loop_events"), "kf_count": S.get("kf_count"), "kf_updates": S.get("kf_updates"),
            "kf_max_correction_cm": S.get("kf_max_correction_cm"),
            # only over poses that reached the corrected map: an unrefined one has final == live
            # by construction, and counting its zero would pull the median toward 0
            "live_vs_final_traj_cm": lvf(R),
            "unrefined_poses": {"n": int((~R["refined"]).sum()),
                                "pct": round(100 * float((~R["refined"]).mean()), 1),
                                "kf_culled": S.get("kf_culled")},
            "sam6d_frames": S["sam6d_frames"], "objects_unplaced": S["objects_unplaced"],
            "observation_spread_cm": {"median": round(float(np.median(sp)), 2), "p90": round(float(np.percentile(sp, 90)), 2)},
            "interp_prediction": interp,
            "memory_kept": len(ks), "memory_classes": len({l["name"] for l in ks}),
            "memory_deleted": sum(1 for l in S.get("memory_landmarks", []) if l["status"] == "deleted"),
        }

    # trajectory agreement (final, corrected) at common stamps
    common, ia, ib = np.intersect1d(A["t"], B["t"], return_indices=True)
    interp_b = len(common) < 100          # different sensor rates (e.g. LiDAR 10 Hz vs camera 30 Hz)
    if interp_b:
        k = np.searchsorted(B["t"], A["t"])
        ok = (k > 0) & (k < len(B["t"]))
        k = np.clip(k, 1, len(B["t"]) - 1)
        gap = (B["t"][k] - B["t"][k - 1]) / 1e9
        ok &= gap < 0.15
        u = np.where(ok, (A["t"] - B["t"][k - 1]) / np.maximum(B["t"][k] - B["t"][k - 1], 1), 0)[:, None]
        common = A["t"][ok]
    for which in ("final", "live"):
        if interp_b:
            Pa = A[which][ok]
            Pb = ((1 - u) * B[which][k - 1] + u * B[which][k])[ok]
            # an interpolated point is only corrected when both of its neighbours are
            keep = (A["refined"][ok] & B["refined"][k - 1][ok] & B["refined"][k][ok]
                    if which == "final" else np.ones(len(Pa), bool))
        else:
            Pa, Pb = A[which][ia], B[which][ib]
            keep = A["refined"][ia] & B["refined"][ib] if which == "final" else np.ones(len(Pa), bool)
        dropped = int((~keep).sum())
        Pa, Pb = Pa[keep], Pb[keep]
        R_, t_ = umeyama(Pb, Pa)
        err = np.linalg.norm((Pb @ R_.T + t_) - Pa, axis=1) * 100
        out[f"trajectory_{which}_B_aligned_to_A_cm"] = {"n": int(len(Pa)), "rmse": round(float(np.sqrt(np.mean(err ** 2))), 2),
                                                        "median": round(float(np.median(err)), 2), "p95": round(float(np.percentile(err, 95)), 2),
                                                        "max": round(float(err.max()), 2)}
        if which == "final":
            out["trajectory_final_B_aligned_to_A_cm"]["unrefined_dropped"] = dropped
            R_final, t_final = R_, t_

    # objects: memory instances of B carried into A's map
    la = {l["name"]: l for l in sorted(kept(A["S"]), key=lambda l: -l["n_obs"])}
    lb = {l["name"]: l for l in sorted(kept(B["S"]), key=lambda l: -l["n_obs"])}
    obj = {}
    for n in sorted(set(la) | set(lb)):
        if n in la and n in lb:
            pa = mat(la[n]["T_w_obj"])[:3, 3]
            pb = R_final @ mat(lb[n]["T_w_obj"])[:3, 3] + t_final
            obj[n] = {"distance_cm": round(float(np.linalg.norm(pa - pb)) * 100, 2), "n_obs_A": la[n]["n_obs"], "n_obs_B": lb[n]["n_obs"]}
        else:
            obj[n] = {"only_in": "A" if n in la else "B"}
    out["objects_B_in_A_map"] = obj
    names = [n for n in obj if "distance_cm" in obj[n]]
    if len(names) >= 2:
        # frame-free check: pairwise distances between objects inside each map
        PA = np.array([mat(la[n]["T_w_obj"])[:3, 3] for n in names])
        PB = np.array([mat(lb[n]["T_w_obj"])[:3, 3] for n in names])
        dA = np.linalg.norm(PA[:, None] - PA[None], axis=2)
        dB = np.linalg.norm(PB[:, None] - PB[None], axis=2)
        iu = np.triu_indices(len(names), 1)
        diff = np.abs(dA - dB)[iu] * 100
        out["pairwise_object_distance_diff_cm"] = {"pairs": int(len(diff)), "median": round(float(np.median(diff)), 2),
                                                   "max": round(float(diff.max()), 2)}
    txt = json.dumps(out, indent=1, ensure_ascii=False)
    if a.out:
        Path(a.out).write_text(txt)
    print(txt)


if __name__ == "__main__":
    main()
