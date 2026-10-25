#!/usr/bin/env python3
"""loop_truth.py — check every loop ORB-SLAM3 corrected (LOOP_GAP lines) against the KISS-ICP trajectory.
For each loop: where the two cameras really were (KISS pose at ts + rig time offset, times the run's T_lidar_cam)
vs what the loop claimed (resid_m = distance the correction leaves between current and matched camera).
    ~/objpose/lidar/venv/bin/python loop_truth.py [session] [variants...]   (default: digut base dep gy gyd)
"""
import json, re, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent


def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    a = sys.argv[1:]
    s = a[0] if a else "digut"; V = a[1:] or ["base", "dep", "gy", "gyd"]
    O = HERE / f"261022_cbnu_{s}"
    K = np.loadtxt(O / "kiss/kiss_tum.txt")
    out = []
    for v in V:
        for r in range(1, 10):
            log = O / f"{v}_run{r}.log"
            if not log.exists():
                continue
            rig = json.loads((O / f"reps/rig_{v}_run{r}.json").read_text())
            T_lc = np.array(rig["T_lidar_cam"]); tau = rig["time"]["lidar_minus_cam_s"]

            def cam(t):
                i = np.searchsorted(K[:, 0], t + tau)
                i = min(max(i, 1), len(K) - 1)
                k = K[i] if abs(K[i, 0] - t - tau) < abs(K[i-1, 0] - t - tau) else K[i-1]
                T = np.eye(4); T[:3, :3] = quat_R(*k[4:8]); T[:3, 3] = k[1:4]
                return T @ T_lc
            text = log.read_text()
            vetoed = len(re.findall(r"verdict=VETO", text))
            for m in re.finditer(r"^LOOP_GAP\|(.*)$", text, re.M):
                f = dict(kv.split("=", 1) for kv in m.group(1).split("|"))
                Tc, Tm = cam(float(f["ts_cur"])), cam(float(f["ts_matched"]))
                d_true = float(np.linalg.norm(Tc[:3, 3] - Tm[:3, 3]))
                Rrel = Tm[:3, :3].T @ Tc[:3, :3]
                ang = float(np.degrees(np.arccos(np.clip((np.trace(Rrel) - 1) / 2, -1, 1))))
                out.append({"variant": v, "run": r, "cur_id": int(f["cur_id"]), "matched_id": int(f["matched_id"]),
                            "dt_s": round(float(f["ts_cur"]) - float(f["ts_matched"]), 1),
                            "true_dist_m": round(d_true, 2), "true_rot_deg": round(ang, 1),
                            "claimed_dist_m": round(float(f["resid_m"]), 2), "correction_m": round(float(f["corr_m"]), 2),
                            "vetoed_in_run": vetoed})
    (O / "loop_truth.json").write_text(json.dumps(out, indent=1))
    print(f"{'var':4} {'run':3} {'cur':>4} {'match':>5} {'dt s':>6} {'true m':>7} {'true deg':>8} {'claimed m':>9} {'corr m':>6}")
    for x in out:
        print(f"{x['variant']:4} {x['run']:3} {x['cur_id']:4} {x['matched_id']:5} {x['dt_s']:6} {x['true_dist_m']:7} "
              f"{x['true_rot_deg']:8} {x['claimed_dist_m']:9} {x['correction_m']:6}")


if __name__ == "__main__":
    main()
