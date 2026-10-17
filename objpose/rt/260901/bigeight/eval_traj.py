#!/usr/bin/env python3
"""eval_traj.py — score camera-SLAM trajectories against the KISS-ICP LiDAR reference.

The reference camera pose is T_wl(t + tau) * T_lidar_cam (rig json from calib_rig.py), so the comparison is
pose-to-pose in the camera frame, not between two different points of the robot.

Per trajectory:
  ate        SE(3) alignment over the whole run (Umeyama on positions), position RMSE / median / max, and the
             rotation error left after that alignment
  drift      alignment on the first `--anchor-s` seconds of motion ONLY, then the error is left to grow:
             this is what a live consumer sees (no hindsight) — position / yaw error vs time, final values
  rpe        relative pose error over `--rpe-s` windows (translation cm, rotation deg) — local consistency
  yaw        heading error statistics (rotation error about the reference's vertical axis)

    python3 eval_traj.py --rig rig.json --gt kiss/kiss_tum.txt --out eval.json name=traj.txt [name=traj.txt ...]
"""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
from trajlib import load_tum, Traj, umeyama_se3, inv, rotvec, se3


def stats(x):
    x = np.asarray(x, float)
    return {"rmse": float(np.sqrt((x ** 2).mean())), "mean": float(x.mean()), "median": float(np.median(x)),
            "p90": float(np.percentile(x, 90)), "max": float(x.max())}


def evaluate(name, path, tg, Tg_cam, up, a, xyz_post=None):
    te, Te = load_tum(path)
    E = Traj(te, Te, 0.2)
    Tq, ok = E.at(tg)
    ok &= np.isfinite(Tq[:, :3, 3]).all(1)
    t = tg[ok]; G = Tg_cam[ok]; Q = Tq[ok]
    res = {"file": str(path), "poses": int(len(te)), "matched": int(ok.sum()), "coverage": float(ok.mean())}

    # --- ATE with full-run alignment
    R, tr = umeyama_se3(Q[:, :3, 3], G[:, :3, 3])
    S = se3(R, tr)
    Qa = S @ Q
    pe = np.linalg.norm(Qa[:, :3, 3] - G[:, :3, 3], axis=1)
    dRot = np.einsum("nji,njk->nik", G[:, :3, :3], Qa[:, :3, :3])
    rv = rotvec(dRot)
    # a constant rotation offset between the two camera frames (extrinsic error) is not SLAM error: remove the mean
    from scipy.spatial.transform import Rotation
    Rm = Rotation.from_matrix(dRot).mean().as_matrix()
    re = np.degrees(np.linalg.norm(rotvec(np.einsum("ji,njk->nik", Rm, dRot)), axis=1))
    res["ate"] = {"pos_cm": {k: v * 100 for k, v in stats(pe).items()}, "rot_deg": stats(re)}

    # --- causal drift: align on the first anchor-s seconds after motion starts
    moving = np.linalg.norm(G[:, :3, 3] - G[0, :3, 3], axis=1) > 0.05
    t_move = t[np.argmax(moving)] if moving.any() else t[0]
    anch = t <= t_move + a.anchor_s
    # full-pose anchor: average of G * Q^-1 over the anchor window (rotation mean + translation LS)
    Rrel = Rotation.from_matrix(np.einsum("nij,nkj->nik", G[anch, :3, :3], Q[anch, :3, :3])).mean().as_matrix()
    trel = (G[anch, :3, 3] - Q[anch, :3, 3] @ Rrel.T).mean(0)
    Qd = se3(Rrel, trel) @ Q
    pd = np.linalg.norm(Qd[:, :3, 3] - G[:, :3, 3], axis=1)
    dRd = np.einsum("nji,njk->nik", G[:, :3, :3], Qd[:, :3, :3])
    rd = np.degrees(np.linalg.norm(rotvec(dRd), axis=1))
    # heading error: rotation error expressed in the world frame, component about the up axis
    dRw = np.einsum("nij,nkj->nik", Qd[:, :3, :3], G[:, :3, :3])
    yaw = np.degrees(rotvec(dRw) @ up)
    tilt = np.degrees(np.linalg.norm(rotvec(dRw) - np.outer(rotvec(dRw) @ up, up), axis=1))
    path_len = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(G[:, :3, 3], axis=0), axis=1))])
    res["drift"] = {"anchor_s": a.anchor_s, "pos_cm": {k: v * 100 for k, v in stats(pd).items()},
                    "pos_cm_final": float(pd[-20:].mean() * 100), "rot_deg": stats(rd), "rot_deg_final": float(rd[-20:].mean()),
                    "yaw_deg": {"rmse": float(np.sqrt((yaw ** 2).mean())), "max_abs": float(np.abs(yaw).max()), "final": float(yaw[-20:].mean())},
                    "tilt_deg": stats(tilt), "pos_pct_of_path_final": float(pd[-20:].mean() / path_len[-1] * 100)}

    # --- RPE
    for d in a.rpe_s:
        n = int(round(d / np.median(np.diff(tg))))
        i = np.arange(0, len(t) - n)
        good = np.abs((t[i + n] - t[i]) - d) < 0.06
        i = i[good]
        dG = inv(G[i]) @ G[i + n]; dQ = inv(Q[i]) @ Q[i + n]
        Eerr = inv(dG) @ dQ
        res[f"rpe_{d:g}s"] = {"n": int(len(i)), "trans_cm": {k: v * 100 for k, v in stats(np.linalg.norm(Eerr[:, :3, 3], axis=1)).items()},
                              "rot_deg": stats(np.degrees(np.linalg.norm(rotvec(Eerr[:, :3, :3]), axis=1)))}
    series = {"t": (t - tg[0]).round(3).tolist(), "ate_pos_cm": (pe * 100).round(2).tolist(), "ate_rot_deg": re.round(3).tolist(),
              "drift_pos_cm": (pd * 100).round(2).tolist(), "drift_yaw_deg": yaw.round(3).tolist(), "drift_tilt_deg": tilt.round(3).tolist(),
              "xyz_aligned": ((Qa @ xyz_post) if xyz_post is not None else Qa)[:, :3, 3].round(4).tolist(),
              "xyz_drift": ((Qd @ xyz_post) if xyz_post is not None else Qd)[:, :3, 3].round(4).tolist()}
    return res, series


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", required=True); ap.add_argument("--gt", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--anchor-s", type=float, default=15.0)
    ap.add_argument("--rpe-s", type=float, nargs="+", default=[1.0, 10.0])
    ap.add_argument("--series", default="", help="also write the per-sample series (for the HTML report) here")
    ap.add_argument("--lidar-frame", default="", help="comma-separated run names whose poses are T_world_velodyne (LiDAR lanes): scored against the raw KISS poses")
    ap.add_argument("traj", nargs="+")
    a = ap.parse_args()
    rig = json.loads(Path(a.rig).read_text())
    X = np.array(rig["T_lidar_cam"]); tau = rig["time"]["lidar_minus_cam_s"]
    tl, Tl = load_tum(a.gt)
    tg = tl - tau                       # lidar stamps on the camera clock
    Tg_cam = Tl @ X
    up = np.array([0.0, 0.0, 1.0])      # KISS world = first velodyne scan, z up
    gt_path = float(np.linalg.norm(np.diff(Tg_cam[:, :3, 3], axis=0), axis=1).sum())
    out = {"gt": str(a.gt), "rig": str(a.rig), "gt_path_m": gt_path, "gt_duration_s": float(tg[-1] - tg[0]), "runs": {}}
    series = {"gt": {"t": (tg - tg[0]).round(3).tolist(), "xyz": Tg_cam[:, :3, 3].round(4).tolist()}, "runs": {}}
    lidar_names = {n for n in a.lidar_frame.split(",") if n}
    for item in a.traj:
        name, path = item.split("=", 1)
        if name in lidar_names:      # LiDAR-frame estimate: reference = KISS poses themselves, no extrinsic / offset
            out["runs"][name], series["runs"][name] = evaluate(name, path, tl, Tl, up, a, xyz_post=X)   # plotted as camera positions
        else:
            out["runs"][name], series["runs"][name] = evaluate(name, path, tg, Tg_cam, up, a)
        r = out["runs"][name]
        print(f"{name:28s} ATE {r['ate']['pos_cm']['rmse']:6.1f} cm (max {r['ate']['pos_cm']['max']:6.1f})  rot {r['ate']['rot_deg']['rmse']:5.2f} deg | "
              f"drift pos rmse {r['drift']['pos_cm']['rmse']:6.1f} max {r['drift']['pos_cm']['max']:6.1f} final {r['drift']['pos_cm_final']:6.1f} cm  "
              f"yaw rmse {r['drift']['yaw_deg']['rmse']:5.2f} max {r['drift']['yaw_deg']['max_abs']:5.2f} tilt rmse {r['drift']['tilt_deg']['rmse']:5.2f} | "
              f"RPE10s {r['rpe_10s']['trans_cm']['rmse']:5.1f} cm {r['rpe_10s']['rot_deg']['rmse']:5.2f} deg  RPE1s {r['rpe_1s']['trans_cm']['rmse']:5.2f} cm {r['rpe_1s']['rot_deg']['rmse']:5.3f} deg")
    Path(a.out).write_text(json.dumps(out, indent=1))
    if a.series:
        Path(a.series).write_text(json.dumps(series))


if __name__ == "__main__":
    main()
