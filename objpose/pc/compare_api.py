#!/usr/bin/env python3
"""compare_api.py — put several finished runs of the object-pose system in one world.

The four SLAM backends do not share a world. ORB-SLAM3 anchors its map to the first camera
frame (optical: y down, z forward); KISS-ICP and hdl_graph_slam anchor theirs to the first
Velodyne scan (z up). Even the two camera backends start from their own first keyframe. And
SAM-6D only ran in one of them, so the others have no objects of their own.

What this module does, per run:

  1. Poses are of that run's OWN SLAM sensor. Multiplying by its T_slam_sam (recorded in
     summary.json) turns them into poses of the SAM camera — the one physical thing all four
     runs observe. Skipping this would compare the LiDAR runs at the Velodyne and the camera
     runs at the camera, and the lever arm between them, which rotates with the cart, would
     show up as a spurious error. Keyframe corrections are applied first, so each pose is the
     backend's final answer, not its live guess.

  2. RE-FUSION. Every SAM-6D estimate is a pose of the object in the SAM camera frame at an
     instant, so it can be replayed onto any backend: T_w_obj = T_w_sam(t) . T_cam_obj. The
     estimates are the same in every run, so the spread of the result across backends is a
     direct read of what the SLAM choice alone costs at the object. Stamps are moved between
     the runs' clock domains with the sam_tau_s each run was configured with.

  3. A rigid Umeyama fit at matched stamps carries the run, and its re-fused objects, into the
     reference run's world so they can be drawn together.

Object positions are the median over that object's re-fused estimates — the same robust
aggregate for every backend, including the reference, so the comparison is like for like.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
from compare_runs import umeyama  # noqa: E402
from fusion import mat  # noqa: E402

LABELS = {"orbslam3": "ORB-SLAM3", "orbslam3_gyro": "ORB-SLAM3 + IMU", "rtabmap": "RTAB-Map",
          "lidar_kiss_icp": "KISS-ICP", "lidar_hdl_graph_slam": "hdl_graph_slam"}
MATCH_TOL_NS = 30_000_000          # 30 ms: half a 10 Hz LiDAR period


def mat16(T) -> list[float]:
    return [round(float(v), 6) for v in np.asarray(T).reshape(-1)]


def run_label(summary: dict, name: str) -> str:
    src = summary.get("slam_source", "")
    label = (LABELS["orbslam3_gyro"] if src == "orbslam3" and "imu" in name.lower()
             else LABELS.get(src, src or name))
    f = summary.get("features")
    if src == "orbslam3" and f and f != 2000:
        label += f" f{f}"
    return label


def extrinsic_of(summary: dict) -> tuple[np.ndarray, float]:
    """(T_slam_sam, sam_tau_s) of a run, from its summary or the extrinsic file it names."""
    path = Path(summary.get("extrinsic_path", ""))
    if not path.is_absolute():
        path = REPO / path
    j = json.loads(path.read_text()) if path.exists() else {}
    X = mat(summary["X_slam_sam"]) if summary.get("X_slam_sam") else mat(j["T_slam_sam"])
    return X, float(j.get("sam_tau_s", 0.0))


def load_poses(d: Path, X: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Stamps and SAM-camera poses T_w_sam, with the final keyframe correction applied."""
    kfu_path = d / "kf_updates.jsonl"
    kfu = [json.loads(l) for l in open(kfu_path)] if kfu_path.exists() else []
    last_kf = {int(r[0]): mat(r[1:]) for r in kfu[-1]["kfs"]} if kfu else {}
    t, T = [], []
    for line in open(d / "slam_poses.jsonl"):
        m = json.loads(line)
        if m.get("state") not in ("OK", "OK_KLT") or m.get("T_wc") is None:
            continue
        F = mat(m["T_wc"])
        ref = m.get("ref_kf")
        if ref is not None and m.get("T_w_kf") is not None and int(ref) in last_kf:
            F = last_kf[int(ref)] @ np.linalg.inv(mat(m["T_w_kf"])) @ F
        t.append(int(m["t_ns"]))
        T.append(F @ X)
    order = np.argsort(t)
    return np.asarray(t, np.int64)[order], np.asarray(T)[order]


def sample(t_all: np.ndarray, T_all: np.ndarray, t_query: np.ndarray):
    """Nearest pose within MATCH_TOL_NS; returns the poses and a validity mask."""
    i = np.clip(np.searchsorted(t_all, t_query), 1, len(t_all) - 1)
    pick = np.where(np.abs(t_all[i - 1] - t_query) <= np.abs(t_all[i] - t_query), i - 1, i)
    ok = np.abs(t_all[pick] - t_query) <= MATCH_TOL_NS
    return T_all[pick], ok


def estimates(d: Path) -> list[dict]:
    """SAM-6D estimates as T_cam_obj with their stamps."""
    path = d / "sam6d_estimates.jsonl"
    if not path.exists():
        return []
    out = []
    for line in open(path):
        e = json.loads(line)
        if not e.get("placed"):
            continue
        T = np.eye(4)
        T[:3, :3] = np.asarray(e["R"], float)
        T[:3, 3] = np.asarray(e["t_mm"], float) / 1000.0
        out.append({"t_ns": int(e["t_ns"]), "object": e["object"], "T_cam_obj": T,
                    "score": e.get("score")})
    return out


def refuse(est: list[dict], t_all, T_all, dtau_s: float) -> list[dict]:
    """Replay the estimates onto this run's trajectory; median position per object."""
    if not est:
        return []
    tq = np.array([e["t_ns"] for e in est], np.int64) + int(round(dtau_s * 1e9))
    T_w_sam, ok = sample(t_all, T_all, tq)
    by: dict[str, list] = {}
    for e, Tw, good in zip(est, T_w_sam, ok):
        if good:
            by.setdefault(e["object"], []).append(Tw @ e["T_cam_obj"])
    objs = []
    for name, Ts in sorted(by.items()):
        P = np.array([T[:3, 3] for T in Ts])
        med = np.median(P, 0)
        pick = Ts[int(np.argmin(np.linalg.norm(P - med, axis=1)))]   # rotation of the most central
        T = pick.copy()
        T[:3, 3] = med
        objs.append({"name": name, "n_obs": len(Ts), "T_w_obj": mat16(T),
                     "scatter_cm": round(float(np.median(np.linalg.norm(P - med, axis=1))) * 100, 2)})
    return objs


def build(run_dirs: list[Path], reference: str | None = None, traj_step: int = 3) -> dict:
    loaded = []
    for d in run_dirs:
        if not (d / "summary.json").exists():
            continue
        S = json.loads((d / "summary.json").read_text())
        X, tau = extrinsic_of(S)
        t, T = load_poses(d, X)
        if len(t) < 10:
            continue
        loaded.append({"dir": d, "name": d.name, "S": S, "t": t, "T": T, "tau": tau,
                       "label": run_label(S, d.name), "n_est": len(estimates(d))})
    if not loaded:
        return {"reference": None, "runs": []}

    # the reference is the run SAM-6D actually ran in: it owns the estimates and the world
    loaded.sort(key=lambda r: (-r["n_est"], r["name"]))
    if reference:
        loaded.sort(key=lambda r: r["name"] != reference)
    ref = loaded[0]
    est = estimates(ref["dir"])

    out_runs = []
    for r in loaded:
        Rw, tw = np.eye(3), np.zeros(3)
        rmse, n_common = 0.0, len(r["t"])
        if r is not ref:
            # reference stamps, shifted into this run's clock domain
            tq = ref["t"] + int(round((r["tau"] - ref["tau"]) * 1e9))
            Tm, ok = sample(r["t"], r["T"], tq)
            if ok.sum() < 10:
                continue
            A = Tm[ok][:, :3, 3]
            B = ref["T"][ok][:, :3, 3]
            Rw, tw = umeyama(A, B)
            err = np.linalg.norm(A @ Rw.T + tw - B, axis=1)
            rmse, n_common = float(np.sqrt(np.mean(err ** 2)) * 100), int(ok.sum())

        W = np.eye(4)
        W[:3, :3], W[:3, 3] = Rw, tw
        objs = refuse(est, r["t"], r["T"], r["tau"] - ref["tau"])
        for o in objs:                                   # into the reference world
            o["T_w_obj"] = mat16(W @ mat(o["T_w_obj"]))
        traj = r["T"][::traj_step, :3, 3] @ Rw.T + tw

        out_runs.append({
            "name": r["name"], "label": r["label"], "slam_source": r["S"].get("slam_source"),
            "is_reference": r is ref,
            "align": {"rmse_cm": round(rmse, 2), "n_common": n_common},
            "stats": {k: r["S"].get(k) for k in
                      ("slam_poses", "slam_ok", "slam_dropped", "slam_lag_s", "slam_track_ms",
                       "kf_count", "kf_updates", "kf_max_correction_cm", "slam_loop_events",
                       "duration_s")},
            "trajectory": np.round(traj, 4).tolist(),
            "objects": objs,
        })

    # how far apart the backends put the same object, in the reference world
    spread = {}
    for name in sorted({o["name"] for r in out_runs for o in r["objects"]}):
        P = np.array([[o["T_w_obj"][3], o["T_w_obj"][7], o["T_w_obj"][11]]
                      for r in out_runs for o in r["objects"] if o["name"] == name])
        if len(P) > 1:
            spread[name] = {"n_runs": len(P),
                            "max_from_mean_cm": round(float(np.max(np.linalg.norm(P - P.mean(0), axis=1))) * 100, 2)}
    return {"reference": ref["name"], "estimates": len(est), "runs": out_runs,
            "object_spread": spread}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help="objpose/output/<run> directories")
    ap.add_argument("--reference", default=None)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    d = build([Path(p) for p in a.runs], a.reference)
    print(f"reference {d['reference']} with {d['estimates']} SAM-6D estimates")
    for r in d["runs"]:
        print(f"  {r['label']:<18} {r['name']:<24} poses={r['stats']['slam_poses']:<6} "
              f"objects={len(r['objects']):<3} align rmse={r['align']['rmse_cm']:.2f} cm "
              f"(n={r['align']['n_common']}){'  [reference]' if r['is_reference'] else ''}")
    print("object spread across backends:")
    for k, v in d["object_spread"].items():
        print(f"  {k:<22} {v['max_from_mean_cm']:6.2f} cm  ({v['n_runs']} runs)")
    Path(a.out).write_text(json.dumps(d))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
