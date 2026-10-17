#!/usr/bin/env python3
"""compare_sweep.py — Icp/MaxTranslation against revisit consistency, per session.

Read with the same caution as compare_sessions.py: the absolute revisit numbers differ four-fold
between sessions because they mostly reflect how precisely the robot re-drives its own path, so only
the change against that session's own input means anything. What the sweep should show, if the cap
really is the reason the lidar-loop lane lost on longcircle, is loops rising with the cap and the
change going negative — and, on the sessions where 0.2 m already won, no gain or a loss, because a
larger cap also lets a wrong link through.
"""
import json, re, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
REF = HERE.parent / "bigeight" / "Userui-MacBookPro"
ALL = ("bigeightcircle", "eightcircle", "longcircle", "digut")
KISS_DRIFT = {"bigeightcircle": 15.3, "eightcircle": 12.8, "longcircle": 44.9, "digut": 35.9}


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


def loops(p):
    try:
        m = re.search(r"loops (\d+)", Path(p).read_text())
        return int(m[1]) if m else None
    except OSError:
        return None


def main():
    sessions = sys.argv[1:] or list(ALL)
    print("Icp/MaxTranslation 스윕 — 재방문 일관성(입력 대비 %)과 받아들인 루프 수\n")
    print("%-16s %8s | %-24s %-24s %-24s %-24s" %
          ("세션", "KISS표류", "0.2 m (기본)", "0.5 m", "1.0 m", "2.0 m"))
    for s in sessions:
        rig_p = HERE / "rigs" / f"rig_{s}.json"
        kiss_p = REF / f"260901_cbnu_{s}" / "kiss" / "kiss_tum.txt"
        if not rig_p.exists():
            print("%-16s (rig 없음)" % s); continue
        rig = json.loads(rig_p.read_text())
        X = np.array(rig["T_lidar_cam"]); Xi = np.linalg.inv(X)
        tau = rig["time"]["lidar_minus_cam_s"]
        tk, Pk = tum(kiss_p)
        sl = Slerp(tk, Rotation.from_matrix(Pk[:, :3, :3]))

        cand = [("0.2", HERE / "sessions" / s / "lidicp_tum.txt.optimized.txt",
                 HERE / "sessions" / s / "lidicp.log")]
        for v in ("0.5", "1.0", "2.0"):
            cand.append((v, HERE / "sweep" / s / f"mt{v}_tum.txt.optimized.txt",
                         HERE / "sweep" / s / f"mt{v}.log"))

        base_med, cells = None, []
        for v, f, lg in cand:
            if not f.exists():
                cells.append("        —              "); continue
            te, Pe = tum(f)
            if base_med is None:
                tq = np.clip(te + tau, tk[0], tk[-1])
                P = np.zeros((len(tq), 4, 4)); P[:, 3, 3] = 1
                P[:, :3, :3] = sl(tq).as_matrix()
                for k in range(3):
                    P[:, k, 3] = np.interp(tq, tk, Pk[:, k, 3])
                ref = np.linalg.inv(P[0])
                In = np.einsum('ij,njk,kl->nil', Xi, np.einsum('ij,njk->nik', ref, P), X)
                base_med = np.median(revisit(te, In[:, :3, 3]))
            m = np.median(revisit(te, Pe[:, :3, 3]))
            cells.append("%+7.1f%%  루프 %3s        " % (100 * (m / base_med - 1), loops(lg) or "?"))
        print("%-16s %6.1fcm | %s" % (s, KISS_DRIFT.get(s, float("nan")), "".join(cells)))


if __name__ == "__main__":
    main()
