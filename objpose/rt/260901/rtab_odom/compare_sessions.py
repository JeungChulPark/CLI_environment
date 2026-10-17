#!/usr/bin/env python3
"""compare_sessions.py — the four odometry/loop configurations across the four 260901_cbnu sessions.

Two things are measured, and they answer different questions:

  ATE vs KISS-ICP   valid for base and imu, where KISS is an independent reference.
                    NOT valid for the lidar lanes: KISS is their odometry, so their ATE only says
                    how far the mapper moved what it was given. Printed, but marked.

  revisit           for each pose, the distance to the nearest pose more than `gap` seconds away.
                    The robot drives its figure/loop several times, so it genuinely revisits; a
                    trajectory that closes on itself scores low. No external reference, so it is
                    comparable across all four lanes — including against the injected input itself,
                    which is what shows whether the mapper helped or hurt. Note it is the very
                    quantity loop closure optimises, so it is biased IN FAVOUR of the mapper: a lane
                    that loses on it has really lost.

    python3 compare_sessions.py [session ...]
"""
import json, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
REF = HERE.parent / "bigeight" / "Userui-MacBookPro"
LANES = ("base", "imu", "lidar", "lidicp")
LABEL = {"base": "기본 VO", "imu": "IMU+VO", "lidar": "LiDAR odom", "lidicp": "LiDAR odom + ICP루프"}
ALL = ("bigeightcircle", "eightcircle", "longcircle", "digut")


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


def loops(log):
    try:
        for ln in open(log):
            if "done:" in ln:
                return int(ln.split("loops ")[1].split(",")[0])
    except (OSError, IndexError):
        pass
    return -1


def main():
    sessions = sys.argv[1:] or list(ALL)
    print("재방문 일관성 (cm) — 궤적이 스스로 닫히는 정도. 입력은 같은 스탬프·프레임·밀도로 재구성.\n")
    print("%-16s %13s | %9s %9s %9s %9s" %
          ("세션", "입력 KISS", *[LABEL[l] for l in LANES]))
    rows = {}
    for s in sessions:
        out = HERE / "sessions" / s
        rig_p = HERE / "rigs" / f"rig_{s}.json"
        kiss_p = REF / f"260901_cbnu_{s}" / "kiss" / "kiss_tum.txt"
        if not out.is_dir() or not rig_p.exists():
            print("%-16s  (없음)" % s); continue
        rig = json.loads(rig_p.read_text())
        X = np.array(rig["T_lidar_cam"]); Xi = np.linalg.inv(X)
        tau = rig["time"]["lidar_minus_cam_s"]
        tk, Pk = tum(kiss_p)
        sl = Slerp(tk, Rotation.from_matrix(Pk[:, :3, :3]))

        cells, base_med = [], None
        for lane in LANES:
            f = out / f"{lane}_tum.txt.optimized.txt"
            if not f.exists():
                cells.append(None); continue
            te, Pe = tum(f)
            if base_med is None:                      # rebuild the input on this lane's stamps
                tq = np.clip(te + tau, tk[0], tk[-1])
                P = np.zeros((len(tq), 4, 4)); P[:, 3, 3] = 1
                P[:, :3, :3] = sl(tq).as_matrix()
                for k in range(3):
                    P[:, k, 3] = np.interp(tq, tk, Pk[:, k, 3])
                ref = np.linalg.inv(P[0])
                In = np.einsum('ij,njk,kl->nil', Xi, np.einsum('ij,njk->nik', ref, P), X)
                base_med = np.median(revisit(te, In[:, :3, 3]))
            cells.append(np.median(revisit(te, Pe[:, :3, 3])))
        rows[s] = (base_med, cells)
        print("%-16s %13.2f | %s" % (s, base_med * 100,
              " ".join("      —  " if c is None else "%9.2f" % (c * 100) for c in cells)))

    print("\n입력 대비 변화 (음수 = 매퍼가 개선)")
    print("%-16s | %9s %9s %9s %9s" % ("세션", *[LABEL[l] for l in LANES]))
    for s, (b, cells) in rows.items():
        print("%-16s | %s" % (s, " ".join("      —  " if c is None else "%+8.1f%%" % (100 * (c / b - 1))
                                          for c in cells)))

    print("\n루프 수")
    print("%-16s | %9s %9s %9s %9s" % ("세션", *[LABEL[l] for l in LANES]))
    for s in rows:
        out = HERE / "sessions" / s
        print("%-16s | %s" % (s, " ".join("%9d" % loops(out / f"{l}.log") for l in LANES)))


if __name__ == "__main__":
    main()
