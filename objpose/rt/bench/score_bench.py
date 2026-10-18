#!/usr/bin/env python3
"""score_bench.py — score every run under out/<session>/<system>/run*/ against the LiDAR pseudo-GT, one table.

Reference: objpose/rt/gt/out/<session>/pgo_tum.txt (T_world_velodyne). Scorer: rt/260901/bigeight/eval_traj.py
(the one every earlier 260901 number came from): SE(3) Umeyama on positions over the whole run, rotation error after
removing the mean offset, RPE over 1 s / 10 s.
  camera systems (orb, orb_gyro, rtab)  reference camera pose = T_wl(t + tau) . T_lidar_cam from a rig json
      260901: rt/260901/rtab_odom/rigs/rig_<name>.json (per session, fitted against KISS-ICP)
      others: bench/rigs/rig_260915.json, ONE rig fitted (calib_rig.py, camera<->LiDAR hand-eye) on CALIB_SESSION's
              orb run against its pseudo-GT and used for every 260910/260915 session
  LiDAR systems (kiss, hdl)             scored in the velodyne frame directly
  kiss = objpose/rt/gt/out/<session>/odom_tum.txt — step 1 of the pseudo-GT itself, so NOT an independent score

    python3 score_bench.py [--out-csv bench/results.csv]
"""
from __future__ import annotations
import argparse, csv, json, subprocess, sys, types
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
BIG = REPO / "objpose/rt/260901/bigeight"
GT = REPO / "objpose/rt/gt/out"
RIGS_260901 = REPO / "objpose/rt/260901/rtab_odom/rigs"
CALIB_SESSION = "260915_eightcircle"
sys.path.insert(0, str(BIG))
import eval_traj  # noqa: E402
from trajlib import load_tum  # noqa: E402

CAMERA = ("orb", "orb_gyro", "rtab")
LIDAR = ("kiss", "hdl")
ORDER = ("orb", "orb_gyro", "rtab", "kiss", "hdl")


def rig_for(session: str) -> Path | None:
    if session.startswith("260901_"):
        return RIGS_260901 / f"rig_{session.replace('260901_cbnu_', '')}.json"
    rig = HERE / "rigs/rig_260915.json"
    if not rig.exists():
        cam = next((HERE / f"out/{CALIB_SESSION}/orb").glob("run*/final_tum.txt"), None)
        if cam is None:
            return None
        rig.parent.mkdir(exist_ok=True)
        subprocess.run([sys.executable, str(BIG / "calib_rig.py"), "--cam", str(cam), "--lidar",
                        str(GT / CALIB_SESSION / "pgo_tum.txt"), "--out", str(rig)], check=True, cwd=BIG,
                       stdout=subprocess.DEVNULL)
    return rig


def runs_of(session: str):
    for sysname in ORDER:
        if sysname == "kiss":
            f = GT / session / "odom_tum.txt"
            if f.exists():
                yield sysname, "run1", "live", f
            continue
        for r in sorted((HERE / "out" / session / sysname).glob("run*")):
            for kind in ("live", "final"):
                f = r / f"{kind}_tum.txt"
                if f.exists() and f.stat().st_size > 0:
                    yield sysname, r.name, kind, f


def maps_of(run_dir: Path) -> int | None:
    """number of maps the camera SLAM used (slam_stream/rtab_stream frames.csv map_id), None if unknown"""
    fc = run_dir / "live_tum.txt.frames.csv"
    if not fc.exists():
        return None
    with open(fc) as fh:
        rows = list(csv.DictReader(fh))
    if not rows or "map_id" not in rows[0]:
        return None
    return len({r["map_id"] for r in rows if r.get("map_id") not in ("", "-1", None)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-csv", default=str(HERE / "results.csv"))
    a = ap.parse_args()
    opts = types.SimpleNamespace(anchor_s=15.0, rpe_s=[1.0, 10.0])
    rows = []
    for sdir in sorted((HERE / "out").glob("26*")):
        s = sdir.name
        gtf = GT / s / "pgo_tum.txt"
        if not gtf.exists():
            print(f"{s}: no pseudo-GT yet"); continue
        tl, Tl = load_tum(gtf)
        rig_path = rig_for(s)
        rig = json.loads(rig_path.read_text()) if rig_path else None
        for sysname, run, kind, f in runs_of(s):
            if sysname in CAMERA:
                if rig is None:
                    continue
                X = np.array(rig["T_lidar_cam"]); tau = rig["time"]["lidar_minus_cam_s"]
                tg, Tg = tl - tau, Tl @ X
            else:
                tg, Tg = tl, Tl
            try:
                r, _ = eval_traj.evaluate(f"{sysname}_{kind}", str(f), tg, Tg, np.array([0.0, 0.0, 1.0]), opts)
            except Exception as e:  # noqa: BLE001 — a broken run is a row with an error, not a crash of the table
                rows.append({"session": s, "system": sysname, "run": run, "traj": kind, "error": str(e)[:80]}); continue
            rows.append({"session": s, "system": sysname, "run": run, "traj": kind,
                         "ate_cm": r["ate"]["pos_cm"]["rmse"], "ate_max_cm": r["ate"]["pos_cm"]["max"],
                         "ate_rot_deg": r["ate"]["rot_deg"]["rmse"], "rpe1_cm": r["rpe_1s"]["trans_cm"]["rmse"],
                         "rpe10_cm": r["rpe_10s"]["trans_cm"]["rmse"], "drift_pct": r["drift"]["pos_pct_of_path_final"],
                         "coverage": r["coverage"], "maps": maps_of(f.parent) if sysname in CAMERA else 1,
                         "rig": rig_path.name if (sysname in CAMERA and rig_path) else "-"})
    keys = ["session", "system", "run", "traj", "ate_cm", "ate_max_cm", "ate_rot_deg", "rpe1_cm", "rpe10_cm", "drift_pct",
            "coverage", "maps", "rig", "error"]
    with open(a.out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore"); w.writeheader(); w.writerows(rows)
    # compact table: final trajectory (live for kiss), ATE cm / coverage
    sessions = sorted({r["session"] for r in rows})
    print(f"{'session':28s}" + "".join(f"{x:>16s}" for x in ORDER))
    for s in sessions:
        cells = []
        for x in ORDER:
            kind = "live" if x == "kiss" else "final"
            rs = [r for r in rows if r["session"] == s and r["system"] == x and r["traj"] == kind and "ate_cm" in r]
            cells.append(f"{np.mean([r['ate_cm'] for r in rs]):8.2f} ({np.mean([r['coverage'] for r in rs]):.2f})" if rs else f"{'-':>16s}")
        print(f"{s:28s}" + "".join(f"{c:>16s}" for c in cells))
    print(f"wrote {a.out_csv}")


if __name__ == "__main__":
    main()
