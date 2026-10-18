#!/usr/bin/env python3
"""map_eval.py — map quality of camera SLAM runs against a LiDAR reference map, one session.

Reference map: every deskewed VLP-16 scan of the pseudo-GT run (objpose/rt/gt/out/<s>/scans_cache.npz) placed with
the pseudo-GT pose (pgo_tum.txt), 5 cm voxels. Each SLAM map is put into that frame with the SAME SE(3) that aligns
its trajectory to the reference (Umeyama on camera positions; reference camera = T_wl(t+tau) . T_lidar_cam), so map
error = trajectory error + mapping error, as a user of the map would see it.

Two maps per run:
  native   what the system itself keeps: ORB-SLAM3 map points (<run>/live_tum.txt.mappoints.xyz, sparse features)
           or RTAB-Map's assembled depth cloud (rtabmap-export --cloud, optimised keyframes, 4 m, 5 cm voxels)
  dense    the SAME depth images (every 15th frame, 0.3-4 m, every 4th pixel) back-projected with the run's final
           trajectory, 5 cm voxels: map quality that comes from the poses alone, identical for every system
Localisation: eval_traj (ATE after SE(3) alignment, rotation, RPE 1 s / 10 s) for the live and final trajectories.
Metrics (map -> reference): distance from each map point to the local plane of its 10 nearest reference points
(within 30 cm; points with no reference nearby are "unverified" and left out): median / p90 [cm], share < 5 cm,
plus the point count and the verified share.

    python3 map_eval.py --session 260901_cbnu_bigeightcircle --runs NAME=RUN_DIR ... [--out map_eval.json]
"""
from __future__ import annotations
import argparse, json, sqlite3, struct, subprocess, sys, tempfile
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "objpose/rt/260901/bigeight"))
from trajlib import load_tum, Traj, umeyama_se3, se3  # noqa: E402
import types
import eval_traj  # noqa: E402
import score_bench  # noqa: E402  (rig_for)

DATA = Path.home() / "Documents/DefenseMeta/Dataset"


def voxel(p, v=0.05):
    if len(p) == 0:
        return p
    k = np.floor(p / v).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return p[i]


def ref_map(session):
    C = np.load(REPO / f"objpose/rt/gt/out/{session}/scans_cache.npz")
    tg, Tg = load_tum(REPO / f"objpose/rt/gt/out/{session}/pgo_tum.txt")
    t = C["t_ns"] * 1e-9
    idx = np.searchsorted(tg, t).clip(0, len(tg) - 1)
    pts = []
    for k in range(len(t)):
        q = C["pts"][C["off"][k]:C["off"][k + 1]].astype(np.float64)
        r = np.linalg.norm(q, axis=1)
        q = q[(r > 0.5) & (r < 15.0)]
        T = Tg[idx[k]]
        pts.append(q @ T[:3, :3].T + T[:3, 3])
    return voxel(np.concatenate(pts), 0.05), tg, Tg


def align(traj, tg, Tg_cam):
    """SE(3) taking the run's world into the reference world (same as eval_traj's ATE alignment)"""
    te, Te = load_tum(traj)
    Tq, ok = Traj(te, Te, 0.2).at(tg)
    ok &= np.isfinite(Tq[:, :3, 3]).all(1)
    R, t = umeyama_se3(Tq[ok, :3, 3], Tg_cam[ok, :3, 3])
    return se3(R, t)


def plane_dist(m, tree, ref):
    d, nn = tree.query(m, k=10, distance_upper_bound=0.30)
    ok = np.isfinite(d).all(1)
    nb = ref[nn[ok]]                                   # [n,10,3]
    c = nb.mean(1)
    cov = np.einsum("nki,nkj->nij", nb - c[:, None], nb - c[:, None])
    _, v = np.linalg.eigh(cov)
    nrm = v[:, :, 0]
    return np.abs(np.einsum("ni,ni->n", m[ok] - c, nrm)), ok


def depth_frames(session, every=15):
    db = next((DATA / session / "SLAM").glob("*.db3"))
    con = sqlite3.connect(f"file:{db}?immutable=1", uri=True)
    tid = con.execute("select id from topics where name='/camera/camera/aligned_depth_to_color/image_raw'").fetchone()[0]
    for k, (b,) in enumerate(con.execute("select data from messages where topic_id=? order by timestamp", (tid,))):
        if k % every:
            continue
        b = bytes(b); o = 4
        sec, ns = struct.unpack_from("<iI", b, o); o += 8
        n = struct.unpack_from("<I", b, o)[0]; o += 4 + n
        o = 4 + (o - 4 + 3) // 4 * 4
        h, w = struct.unpack_from("<II", b, o); o += 8
        n = struct.unpack_from("<I", b, o)[0]; enc = b[o + 4:o + 4 + n - 1].decode(); o += 4 + n
        o += 1; o = 4 + (o - 4 + 3) // 4 * 4
        step = struct.unpack_from("<I", b, o)[0]; o += 4
        n = struct.unpack_from("<I", b, o)[0]; o += 4
        assert enc in ("16UC1", "mono16"), enc
        yield sec + ns * 1e-9, np.frombuffer(b, np.uint16, h * w, o).reshape(h, w)


def intrinsics(settings):
    v = {}
    for line in Path(settings).read_text().splitlines():
        for k in ("Camera1.fx", "Camera1.fy", "Camera1.cx", "Camera1.cy", "RGBD.DepthMapFactor"):
            if line.startswith(k + ":"):
                v[k] = float(line.split(":")[1])
    return v["Camera1.fx"], v["Camera1.fy"], v["Camera1.cx"], v["Camera1.cy"], v.get("RGBD.DepthMapFactor", 1000.0)


def dense_map(frames, traj, K, step=4):
    fx, fy, cx, cy, dmf = K
    te, Te = load_tum(traj)
    tr = Traj(te, Te, 0.2)
    out = []
    for t, D in frames:
        T, ok = tr.at(np.array([t]))
        if not ok[0]:
            continue
        d = D[::step, ::step].astype(np.float64) / dmf
        v, u = np.mgrid[0:D.shape[0]:step, 0:D.shape[1]:step]
        m = (d > 0.3) & (d < 4.0)
        z = d[m]
        p = np.column_stack([(u[m] - cx) * z / fx, (v[m] - cy) * z / fy, z])
        out.append(p @ T[0, :3, :3].T + T[0, :3, 3])
    return voxel(np.concatenate(out), 0.05)


def rtab_cloud(db, final_traj):
    """assembled cloud of a rtab_stream database, moved from RTAB's map frame into the TUM trajectory's world"""
    with tempfile.TemporaryDirectory() as td:
        subprocess.run(["rtabmap-export", "--cloud", "--voxel", "0.05", "--max_range", "4", "--ascii", "--opt", "2",
                        "--output_dir", td, "--output", "m", str(db)], check=True, capture_output=True)
        subprocess.run(["rtabmap-export", "--poses_camera", "--opt", "2", "--poses_format", "10", "--output_dir", td,
                        "--output", "p", str(db)], check=True, capture_output=True)
        ply = Path(td) / "m_cloud.ply"
        lines = ply.read_text().splitlines()
        n = int(next(l for l in lines if l.startswith("element vertex")).split()[-1])
        h = lines.index("end_header")
        cloud = np.array([[float(x) for x in l.split()[:3]] for l in lines[h + 1:h + 1 + n]])
        tp, Tp = load_tum(Path(td) / "p_camera_poses.txt")
    te, Te = load_tum(final_traj)
    Tq, ok = Traj(te, Te, 0.2).at(tp)
    R, t = umeyama_se3(Tp[ok, :3, 3], Tq[ok, :3, 3])     # RTAB map frame -> TUM world
    return cloud @ R.T + t


def stats(dist, ok, n):
    if len(dist) == 0:
        return {"points": int(n), "verified": 0.0}
    return {"points": int(n), "verified": float(ok.mean()), "median_cm": float(np.median(dist) * 100),
            "p90_cm": float(np.percentile(dist, 90) * 100), "lt5cm": float((dist < 0.05).mean())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--settings", required=True, help="camera settings yaml (intrinsics for the dense map)")
    ap.add_argument("--out", default="")
    ap.add_argument("runs", nargs="+", help="NAME=RUN_DIR (with live_tum.txt, final_tum.txt and the native map)")
    a = ap.parse_args()
    ref, tl, Tl = ref_map(a.session)
    tree = cKDTree(ref)
    rig = json.loads(score_bench.rig_for(a.session).read_text())
    tg = tl - rig["time"]["lidar_minus_cam_s"]; Tg = Tl @ np.array(rig["T_lidar_cam"])
    frames = list(depth_frames(a.session))
    K = intrinsics(a.settings)
    print(f"reference map {len(ref)} points, {len(frames)} depth frames", flush=True)
    res = {"session": a.session, "reference_points": int(len(ref))}
    for spec in a.runs:
        name, d = spec.split("=", 1); d = Path(d)
        final = d / "final_tum.txt"
        S = align(final, tg, Tg)
        out = {}
        for kind in ("live", "final"):
            r, _ = eval_traj.evaluate(kind, str(d / f"{kind}_tum.txt"), tg, Tg, np.array([0.0, 0.0, 1.0]),
                                      types.SimpleNamespace(anchor_s=15.0, rpe_s=[1.0, 10.0]))
            out[f"loc_{kind}"] = {"ate_cm": r["ate"]["pos_cm"]["rmse"], "ate_max_cm": r["ate"]["pos_cm"]["max"],
                                  "rot_deg": r["ate"]["rot_deg"]["rmse"], "rot_max_deg": r["ate"]["rot_deg"]["max"],
                                  "rpe1_cm": r["rpe_1s"]["trans_cm"]["rmse"], "rpe1_deg": r["rpe_1s"]["rot_deg"]["rmse"],
                                  "rpe10_cm": r["rpe_10s"]["trans_cm"]["rmse"], "coverage": r["coverage"]}
        mp = d / "live_tum.txt.mappoints.xyz"
        db = d / "live_tum.txt.rtabmap.db"
        native = np.loadtxt(mp).reshape(-1, 3) if mp.exists() else (rtab_cloud(db, final) if db.exists() else None)
        if native is not None:
            m = native @ S[:3, :3].T + S[:3, 3]
            dist, ok = plane_dist(m, tree, ref)
            out["native"] = stats(dist, ok, len(m))
        dm = dense_map(frames, final, K)
        m = dm @ S[:3, :3].T + S[:3, 3]
        dist, ok = plane_dist(m, tree, ref)
        out["dense"] = stats(dist, ok, len(m))
        res[name] = out
        print(name, json.dumps(out), flush=True)
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
