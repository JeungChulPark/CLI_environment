#!/usr/bin/env python3
"""mapper_delta.py — how much did the RTAB-Map mapper MOVE the odometry it was handed?

Scoring the --ext-odom run against KISS-ICP is circular: KISS IS its odometry input. What is not
circular is the difference between what went in and what came out — that difference is exactly the
mapper's own contribution (loop closures + pose-graph optimisation), and it answers
"can the mapper's decisions damage a good odometry?".

rtab_stream writes T_w_cam with world = the first tracked camera pose, i.e.

    T_w_cam(t) = O^-1 . base0^-1 . correction(t) . T_odom_base(t) . O

and for --ext-odom the odometry itself was built as T_odom_base(t) = T_wb(t0)^-1 . T_wl(t) . X,
X = T_lidar_cam . O^-1.  Substituting and cancelling gives the UNCORRECTED trajectory in the very
same output frame:

    T_w_cam_input(t) = T_lidar_cam^-1 . [ T_wl(tf)^-1 . T_wl(t) ] . T_lidar_cam        (tf = first frame)

so no alignment is needed anywhere: an untouched trajectory gives exactly 0.

    python3 mapper_delta.py --rig RIG.json --kiss kiss_tum.txt live=ext_tum.txt opt=ext_tum.txt.optimized.txt
"""
import argparse, json, math
import numpy as np


def load_tum(p):
    t, T = [], []
    for ln in open(p):
        q = ln.split()
        if len(q) != 8:
            continue
        t.append(float(q[0]))
        x, y, z, qx, qy, qz, qw = map(float, q[1:])
        n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        M = np.eye(4)
        M[:3, :3] = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        M[:3, 3] = (x, y, z)
        T.append(M)
    return np.array(t), np.array(T)


def interp(tl, Tl, ts):
    """pose at each ts by linear position + nearest-rotation (10 Hz source, 30 Hz query)"""
    out = np.zeros((len(ts), 4, 4))
    idx = np.clip(np.searchsorted(tl, ts), 1, len(tl) - 1)
    for k, (i, tq) in enumerate(zip(idx, ts)):
        a, b = i - 1, i
        u = (tq - tl[a]) / (tl[b] - tl[a]) if tl[b] > tl[a] else 0.0
        u = min(max(u, 0.0), 1.0)
        M = np.eye(4)
        M[:3, :3] = Tl[a if u < 0.5 else b][:3, :3]
        M[:3, 3] = Tl[a][:3, 3] * (1 - u) + Tl[b][:3, 3] * u
        out[k] = M
    return out


def rot_deg(R):
    c = np.clip((np.trace(R[:3, :3]) - 1) / 2, -1, 1)
    return math.degrees(math.acos(c))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", required=True)
    ap.add_argument("--kiss", required=True)
    ap.add_argument("runs", nargs="+", help="name=path")
    a = ap.parse_args()
    rig = json.loads(open(a.rig).read())
    X = np.array(rig["T_lidar_cam"])
    Xi = np.linalg.inv(X)
    tau = rig["time"]["lidar_minus_cam_s"]
    tl, Tl = load_tum(a.kiss)

    print("mapper-induced change to the trajectory it was given")
    print("(--ext-odom run: the input is the KISS trajectory itself, so this is the mapper's own contribution)")
    print()
    print("%-8s %7s  %9s %9s %9s   %9s %9s" %
          ("run", "poses", "med cm", "p95 cm", "max cm", "rot med", "rot max"))
    for spec in a.runs:
        name, _, path = spec.partition("=")
        te, Te = load_tum(path)
        if len(te) < 10:
            print("%-8s  !! too few poses" % name)
            continue
        # uncorrected input, expressed in the run's own output frame
        Wl = interp(tl, Tl, te + tau)
        ref0i = np.linalg.inv(Wl[0])
        In = np.einsum("ij,njk->nik", Xi, np.einsum("ij,njk->nik", ref0i, Wl))
        In = np.einsum("nij,jk->nik", In, X)
        dp = np.linalg.norm(Te[:, :3, 3] - In[:, :3, 3], axis=1) * 100
        dr = np.array([rot_deg(np.linalg.inv(In[k]) @ Te[k]) for k in range(len(te))])
        s = np.sort(dp)
        r = np.sort(dr)
        print("%-8s %7d  %9.2f %9.2f %9.2f   %8.3f° %8.3f°" %
              (name, len(te), np.median(s), s[int(.95 * len(s))], s[-1],
               np.median(r), r[-1]))


if __name__ == "__main__":
    main()
