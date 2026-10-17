#!/usr/bin/env python3
"""compare_features.py — Vis/MaxFeatures against the two things it can plausibly fix.

  lidar lane   KISS odometry, visual loop geometry. Its revisit number IS the loop constraint's
               quality, because nothing else in the lane changes with the feature count. Compare it
               against the ICP lane (sessions/<s>/lidicp) to see whether a richer visual constraint
               closes the gap that motivated using the lidar for loops at all.

  base lane    visual odometry AND visual loops. Its ATE against KISS is an independent measure
               (KISS is not its input), so it says whether more features buy accuracy at all.

Read the revisit column as a change against that session's own input, never across sessions — the
absolute level is dominated by how precisely the robot re-drove its path.
"""
import json, re, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
REF = HERE.parent / "bigeight" / "Userui-MacBookPro"
ALL = ("bigeightcircle", "eightcircle", "longcircle", "digut")
FEATS = ("1000", "2000", "3000")


def tum(p):
    T = np.array([[float(x) for x in l.split()] for l in open(p) if len(l.split()) == 8])
    P = np.zeros((len(T), 4, 4)); P[:, 3, 3] = 1
    P[:, :3, :3] = Rotation.from_quat(T[:, 4:8]).as_matrix()
    P[:, :3, 3] = T[:, 1:4]
    return T[:, 0], P


def revisit(t, xyz, gap=30.0, near=1.0, step=3):
    tree = cKDTree(xyz); d = []
    for i in range(0, len(xyz), step):
        far = [j for j in tree.query_ball_point(xyz[i], near) if abs(t[j] - t[i]) > gap]
        if far:
            d.append(min(np.linalg.norm(xyz[j] - xyz[i]) for j in far))
    return np.array(d)


def stat(log, key):
    try:
        m = re.search(r"loops (\d+), nodes (\d+), wall ([\d.]+) s", Path(log).read_text())
        return {"loops": m[1], "wall": m[3]}[key] if m else "?"
    except OSError:
        return "?"


def path_for(s, lane, f):
    if f == "1000":
        return HERE / "sessions" / s / f"{lane}_tum.txt.optimized.txt", HERE / "sessions" / s / f"{lane}.log"
    return (HERE / "feats" / s / f"{lane}_f{f}_tum.txt.optimized.txt",
            HERE / "feats" / s / f"{lane}_f{f}.log")


def main():
    sessions = sys.argv[1:] or list(ALL)
    print("Vis/MaxFeatures 스윕 — 재방문 일관성(입력 KISS 대비 %) · 괄호는 루프 수\n")
    for s in sessions:
        rig_p = HERE / "rigs" / f"rig_{s}.json"
        if not rig_p.exists():
            print("%-16s (rig 없음)" % s); continue
        rig = json.loads(rig_p.read_text())
        X = np.array(rig["T_lidar_cam"]); Xi = np.linalg.inv(X)
        tau = rig["time"]["lidar_minus_cam_s"]
        tk, Pk = tum(REF / f"260901_cbnu_{s}" / "kiss" / "kiss_tum.txt")
        sl = Slerp(tk, Rotation.from_matrix(Pk[:, :3, :3]))
        base_med = None

        def score(p):
            nonlocal base_med
            te, Pe = tum(p)
            if base_med is None:
                tq = np.clip(te + tau, tk[0], tk[-1])
                P = np.zeros((len(tq), 4, 4)); P[:, 3, 3] = 1
                P[:, :3, :3] = sl(tq).as_matrix()
                for k in range(3):
                    P[:, k, 3] = np.interp(tq, tk, Pk[:, k, 3])
                ref = np.linalg.inv(P[0])
                In = np.einsum('ij,njk,kl->nil', Xi, np.einsum('ij,njk->nik', ref, P), X)
                base_med = np.median(revisit(te, In[:, :3, 3]))
            return np.median(revisit(te, Pe[:, :3, 3]))

        # the ICP lane is the reference the visual lane is trying to reach
        icp_p, icp_log = HERE / "sessions" / s / "lidicp_tum.txt.optimized.txt", HERE / "sessions" / s / "lidicp.log"
        rows = []
        for lane in ("lidar", "base"):
            cells = []
            for f in FEATS:
                p, lg = path_for(s, lane, f)
                if not p.exists():
                    cells.append("     —      "); continue
                cells.append("%+7.1f%% (%3s)" % (100 * (score(p) / base_med - 1), stat(lg, "loops")))
            rows.append((lane, cells))
        icp = "%+7.1f%% (%3s)" % (100 * (score(icp_p) / base_med - 1), stat(icp_log, "loops")) \
            if icp_p.exists() else "     —"
        print("%-16s 입력 %.2f cm" % (s, base_med * 100))
        print("   %-8s %-13s %-13s %-13s   | ICP 루프 %s" % ("", *FEATS, ""))
        for lane, cells in rows:
            print("   %-8s %s" % (lane, " ".join(cells)) + ("   | %s" % icp if lane == "lidar" else ""))
        print()


if __name__ == "__main__":
    main()
