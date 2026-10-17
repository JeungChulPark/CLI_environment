#!/usr/bin/env python3
"""build_map_data.py — turn this directory's ORB-SLAM3 runs into data for html/260918_slam_3d_map and add
the sessions to its index.json / summary.json (existing sessions are kept; a session already listed is replaced).

With a LiDAR bag (260915_*): KISS-ICP is the reference. There is no rig calibration for the 260915 rig, so
T_lidar_cam and the time offset are estimated from the run itself (calib_rig.py without --imu, on the optimized
trajectory), then the trajectories are scored with eval_traj.py exactly like the 260901 sessions.

Without one (260826_etri_*): there is no reference, so no metrics. The trajectories are put in a z-up frame
(normal of the best-fit plane of the optimized path) and the point cloud is the depth images back-projected
along the optimized trajectory.

    ~/objpose/lidar/venv/bin/python build_map_data.py [--dataset ~/Documents/DefenseMeta/Dataset] [sessions...]
"""
from __future__ import annotations
import argparse, json, sqlite3, struct, subprocess, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
BIG = HERE.parent / "260901" / "bigeight"
PAGE = HERE.parents[2] / "html" / "260918_slam_3d_map"
sys.path.insert(0, str(BIG))
from trajlib import load_tum, Traj, umeyama_se3, se3  # noqa: E402

LABELS = {"kiss": "KISS-ICP (LiDAR reference)", "orb_live": "ORB-SLAM3 (live)", "orb_opt": "ORB-SLAM3 (optimized)"}
STRIDE = 2            # same thinning as the 260901 data files
CLOUD_MAX = 24000


def r3(a):
    return np.round(np.asarray(a, float), 3).tolist()


def thin_cloud(C, rng):
    if len(C) > CLOUD_MAX:
        C = C[rng.choice(len(C), CLOUD_MAX, replace=False)]
    return r3(C)


def path_len(P):
    return float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())


# ---------------------------------------------------------------- with a LiDAR reference
def with_lidar(name, O, rng):
    py = sys.executable
    run = O / "base_run1_tum.txt"
    rig = O / "rig.json"
    subprocess.run([py, str(BIG / "calib_rig.py"), "--cam", f"{run}.optimized.txt", "--lidar", str(O / "kiss/kiss_tum.txt"),
                    "--out", str(rig)], check=True, cwd=BIG, stdout=subprocess.DEVNULL)
    ev = subprocess.run([py, str(BIG / "eval_traj.py"), "--rig", str(rig), "--gt", str(O / "kiss/kiss_tum.txt"),
                         "--out", str(O / "eval.json"), "--series", str(O / "series.json"),
                         f"orb_live={run}", f"orb_opt={run}.optimized.txt"], check=True, cwd=BIG,
                        capture_output=True, text=True).stdout
    (O / "eval.txt").write_text(ev)
    print(ev, end="")
    E = json.loads((O / "eval.json").read_text()); S = json.loads((O / "series.json").read_text())
    series = [{"key": "kiss", "label": LABELS["kiss"], "xyz": S["gt"]["xyz"][::STRIDE], "m": None, "ref": True}]
    metrics = {}
    for k in ("orb_live", "orb_opt"):
        r = E["runs"][k]
        m = {"ate": round(r["ate"]["pos_cm"]["rmse"], 1), "ate_max": round(r["ate"]["pos_cm"]["max"], 1),
             "final": round(r["drift"]["pos_cm_final"], 1), "rot": round(r["ate"]["rot_deg"]["rmse"], 2)}
        metrics[k] = m
        series.append({"key": k, "label": LABELS[k], "xyz": S["runs"][k]["xyz_aligned"][::STRIDE], "m": m, "ref": False})
    ks = json.loads((O / "kiss/kiss_summary.json").read_text())
    R = json.loads(rig.read_text())
    note = (f"260915 rig는 보정값이 없어 이 실행에서 직접 추정했습니다 (hand-eye {R['T_lidar_cam_fit']['windows']}개 창, "
            f"잔차 {R['T_lidar_cam_fit']['rot_resid_deg_rms']:.2f}° / {R['T_lidar_cam_fit']['trans_resid_m_rms']*100:.1f} cm, "
            f"시간 오프셋 {R['time']['lidar_minus_cam_s']*1000:+.1f} ms). 같은 궤적으로 rig를 맞췄으므로 "
            f"260901 세션보다 수치가 약간 유리하게 나올 수 있습니다.")
    data = {"session": name, "cloud": thin_cloud(np.load(O / "kiss/kiss_map_preview.npy"), rng), "series": series,
            "summary": ks, "gt_path_m": round(E["gt_path_m"], 2), "note": note}
    return data, metrics, ks["path_m"]


# ---------------------------------------------------------------- without a reference
def depth_frames(db, every):
    """(stamp_s, uint16 depth HxW) for every `every`-th aligned depth image (sensor_msgs/Image, CDR)"""
    con = sqlite3.connect(f"file:{db}?immutable=1", uri=True)
    tid = con.execute("select id from topics where name like '%aligned_depth_to_color/image_raw'").fetchone()[0]
    for i, (b,) in enumerate(con.execute("select data from messages where topic_id=? order by timestamp", (tid,))):
        if i % every:
            continue
        b = bytes(b)
        sec, nsec = struct.unpack_from("<iI", b, 4)
        o = 12; n = struct.unpack_from("<I", b, o)[0]; o += 4 + n
        o = 4 + (o - 4 + 3) // 4 * 4
        h, w = struct.unpack_from("<II", b, o); o += 8
        n = struct.unpack_from("<I", b, o)[0]; enc = b[o + 4:o + 3 + n].decode(); o += 4 + n
        o += 1; o = 4 + (o - 4 + 3) // 4 * 4
        step, n = struct.unpack_from("<II", b, o); o += 8
        if enc not in ("16UC1", "mono16"):
            raise SystemExit(f"unexpected depth encoding {enc}")
        yield sec + nsec * 1e-9, np.frombuffer(b, "<u2", h * w, o).reshape(h, w)


def no_lidar(name, D, O, rng):
    sys.path.insert(0, str(HERE))
    from cam_params import camera_info
    db = next((D / "SLAM").glob("*.db3"))
    _, _, K = camera_info(db)
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    to, To = load_tum(O / "base_run1_tum.txt.optimized.txt")
    tl, Tl = load_tum(O / "base_run1_tum.txt")
    # z-up world: normal of the best-fit plane of the path, signed to agree with the camera's "up" (-y)
    P = To[:, :3, 3]
    _, _, Vt = np.linalg.svd(P - P.mean(0))
    n = Vt[2] * (1 if Vt[2] @ (-To[:, :3, 1].mean(0)) > 0 else -1)
    fwd = To[0, :3, 2] - (To[0, :3, 2] @ n) * n; fwd /= np.linalg.norm(fwd)
    Rw = np.stack([fwd, np.cross(n, fwd), n])          # rows: new x (initial heading), y, z (up)
    W = se3(Rw, -Rw @ P[0])
    Wo = W @ To
    # live trajectory in the same frame: align its positions to the optimized one at shared stamps
    Lo, ok = Traj(to, To, 0.5).at(tl)
    R, t = umeyama_se3(Tl[ok, :3, 3], Lo[ok, :3, 3])
    Wl = W @ se3(R, t) @ Tl[ok]
    # cloud: depth back-projected along the optimized trajectory
    tr = Traj(to, Wo, 0.5)
    vv, uu = np.mgrid[4:480:8, 4:640:8]
    pts = []
    for ts, dep in depth_frames(db, 10):
        Tq, okq = tr.at(np.array([ts]))
        if not okq[0]:
            continue
        z = dep[vv, uu].astype(np.float64) * 1e-3
        m = (z > 0.3) & (z < 5.0)
        Xc = np.stack([(uu[m] - cx) / fx * z[m], (vv[m] - cy) / fy * z[m], z[m]], 1)
        pts.append(Xc @ Tq[0, :3, :3].T + Tq[0, :3, 3])
    C = np.concatenate(pts)
    C = C[np.unique(np.floor(C / 0.08).astype(np.int64), axis=0, return_index=True)[1]]
    po = Wo[:, :3, 3]
    series = [{"key": "orb_live", "label": LABELS["orb_live"], "xyz": r3(Wl[::STRIDE, :3, 3]), "m": None, "ref": False},
              {"key": "orb_opt", "label": LABELS["orb_opt"], "xyz": r3(po[::STRIDE]), "m": None, "ref": False}]
    pl = round(path_len(po), 2)
    summary = {"frames": int(len(to)), "path_m": pl, "return_error_cm": round(float(np.linalg.norm(po[-1] - po[0])) * 100, 1),
               "map_preview_points": int(len(C))}
    note = ("LiDAR가 없는 세션이라 기준 궤적과 정확도 수치가 없습니다. 궤적은 최적화 궤적의 주행 평면이 수평이 되도록 "
            "세웠고, 포인트 클라우드는 깊이 영상을 최적화 궤적을 따라 역투영한 것입니다. 실시간 궤적은 최적화 궤적에 정렬해 겹쳤습니다.")
    data = {"session": name, "cloud": thin_cloud(C, rng), "series": series, "summary": summary, "note": note}
    return data, {}, pl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default=str(Path.home() / "Documents/DefenseMeta/Dataset"))
    ap.add_argument("sessions", nargs="*")
    a = ap.parse_args()
    names = a.sessions or sorted(p.name for p in HERE.iterdir() if (p / "base_run1_tum.txt.optimized.txt").exists())
    idx = json.loads((PAGE / "data/index.json").read_text())
    summ = json.loads((PAGE / "data/summary.json").read_text())
    for name in names:
        O, D = HERE / name, Path(a.dataset).expanduser() / name
        rng = np.random.default_rng(0)
        print(f"== {name}")
        if (O / "kiss/kiss_tum.txt").exists():
            data, metrics, pl = with_lidar(name, O, rng)
        else:
            data, metrics, pl = no_lidar(name, D, O, rng)
        (PAGE / "data" / f"{name}.json").write_text(json.dumps(data, separators=(",", ":")))
        keys = [s["key"] for s in data["series"]]
        entry = {"session": name, "path_m": pl, "hdl": False, "series": keys}
        idx = [e for e in idx if e["session"] != name] + [entry]
        summ = [e for e in summ if e["session"] != name] + [dict(entry, metrics=metrics, labels={k: LABELS[k] for k in keys})]
        print(f"   {len(data['cloud'])} cloud points, path {pl} m, metrics: {metrics or 'none (no reference)'}")
    (PAGE / "data/index.json").write_text(json.dumps(idx, indent=1))
    (PAGE / "data/summary.json").write_text(json.dumps(summ, separators=(",", ":")))


if __name__ == "__main__":
    main()
