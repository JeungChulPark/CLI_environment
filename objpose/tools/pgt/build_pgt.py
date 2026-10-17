#!/usr/bin/env python3
"""build_pgt.py — map-based pseudo ground truth for a recorded session (no real GT exists).

1. every estimate on the BUILD frames (proposed and original SAM-6D, run on frames i % 3 == 1) is placed in the
   map with the SAM camera's own ORB-SLAM3 trajectory (offline, loop-closed, globally optimised):
       T_map_obj = T_map_cam(t) . T_cam_obj
2. per object the densest consensus is taken: the position with most estimates within 5 cm, then among those the
   rotation with most estimates within 20 deg; the answer is the median position / chordal-mean rotation of the
   inliers. Objects with fewer than MIN_SUPPORT inliers are left out (not in the scene or never found reliably).
3. on the EVAL frames (i % 3 == 0, never used above) every map object is put back into the camera:
       T_cam_obj(t) = T_map_cam(t)^-1 . T_map_obj
   and its visible part is found with a z-buffer of model surface points and the measured depth (occluders).
   Written as BOP scene_gt.json / scene_gt_info.json, so run_compare.py --gt-boxes and the scorer read it as is.

    python objpose/tools/pgt/build_pgt.py objpose/output/pgt_260915_inlong_middle
"""
from __future__ import annotations

import json, sys
from pathlib import Path

import cv2
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as Rot, Slerp

ROOT = Path(sys.argv[1])
TABLE = json.loads((Path(__file__).resolve().parents[2] / "rt/bop/real_objects.json").read_text())
POS_TOL, ROT_TOL, MIN_SUPPORT = 50.0, 20.0, 50
SRC = {"proposed": ROOT / "runs/build_proposed/000001", "orig": ROOT / "runs/build_orig/000001"}


def load_tum(p):
    a = np.loadtxt(p)
    return (a[:, 0] * 1e9).astype(np.int64), a[:, 1:4] * 1000.0, a[:, 4:8]     # ns, mm, quat xyzw


class Traj:
    def __init__(self, p):
        self.t, self.p, q = load_tum(p)
        self.slerp = Slerp(self.t.astype(np.float64), Rot.from_quat(q))

    def T(self, t_ns):
        if t_ns < self.t[0] or t_ns > self.t[-1]:
            return None
        T = np.eye(4)
        T[:3, :3] = self.slerp([float(t_ns)]).as_matrix()[0]
        T[:3, 3] = [np.interp(t_ns, self.t, self.p[:, k]) for k in range(3)]
        return T


def pose(R, t):
    T = np.eye(4); T[:3, :3] = np.asarray(R, float).reshape(3, 3); T[:3, 3] = np.asarray(t, float).reshape(3); return T


def rot_deg(A, B):
    c = (np.trace(A.T @ B) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def main():
    traj = Traj(ROOT / "slam/sam_orb_tum.txt.optimized.txt")
    times_b = {int(k): v for k, v in json.loads((ROOT / "build/000001/times.json").read_text()).items()}
    per = {}
    for src, d in SRC.items():
        for l in (d / "sam6d_frames.jsonl").read_text().splitlines():
            f = json.loads(l)
            Twc = traj.T(times_b[f["im_id"]])
            if Twc is None:
                continue
            best = {}
            for det in f["dets"]:                      # original SAM-6D may give several per object: keep its best
                if det["object"] not in best or det["score"] > best[det["object"]]["score"]:
                    best[det["object"]] = det
            for det in best.values():
                per.setdefault(det["object"], []).append((src, Twc @ pose(det["R"], det["t_mm"])))
    names = {o["name"]: o for o in TABLE["objects"]}
    world = {}
    report = {}
    for n, rows in sorted(per.items()):
        P = np.array([T[:3, 3] for _, T in rows]); Rs = [T[:3, :3] for _, T in rows]
        dist = np.linalg.norm(P[:, None] - P[None], axis=-1)
        c = int(np.argmax((dist < POS_TOL).sum(1)))
        pin = np.where(dist[c] < POS_TOL)[0]
        rc = max(pin, key=lambda i: sum(rot_deg(Rs[i], Rs[j]) < ROT_TOL for j in pin))
        inl = [j for j in pin if rot_deg(Rs[rc], Rs[j]) < ROT_TOL]
        report[n] = {"estimates": len(rows), "pos_inliers": len(pin), "inliers": len(inl),
                     "by_source": {s: sum(rows[j][0] == s for j in inl) for s in SRC}}
        if len(inl) < MIN_SUPPORT:
            report[n]["kept"] = False
            continue
        t = np.median(P[inl], axis=0)
        M = sum(Rs[j] for j in inl); U, _, Vt = np.linalg.svd(M); R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        world[n] = pose(R, t)
        spread = np.linalg.norm(P[inl] - t, axis=1)
        report[n].update(kept=True, pos_spread_med_mm=float(np.median(spread)),
                         rot_spread_med_deg=float(np.median([rot_deg(R, Rs[j]) for j in inl])))
        print(n, report[n], flush=True)
    for n in sorted(set(names) - set(report)):
        report[n] = {"estimates": 0, "kept": False}
    (ROOT / "pgt_world.json").write_text(json.dumps({"objects": {n: T.tolist() for n, T in world.items()},
                                                     "report": report}, indent=1, ensure_ascii=False))

    # ---- project onto the eval frames
    scene = ROOT / "eval/000001"
    cam = json.loads((scene / "scene_camera.json").read_text())
    times_e = {int(k): v for k, v in json.loads((scene / "times.json").read_text()).items()}
    K = np.asarray(next(iter(cam.values()))["cam_K"]).reshape(3, 3)
    mdir = Path(TABLE["models_dir"])
    pts = {}
    for n in world:
        m = trimesh.load(mdir / f"obj_{names[n]['obj_id']:06d}.ply", process=False)
        if isinstance(m, trimesh.PointCloud) or not hasattr(m, "faces") or len(m.faces) == 0:
            pts[n] = np.asarray(m.vertices)[np.random.default_rng(0).choice(len(m.vertices), 20000)]
        else:
            pts[n] = trimesh.sample.sample_surface(m, 20000, seed=0)[0]
    gt, info = {}, {}
    H, W = 480, 640
    for fid in sorted(times_e):
        Twc = traj.T(times_e[fid])
        depth = cv2.imread(str(scene / "depth" / f"{fid:06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32)
        g, inf = [], []
        if Twc is not None:
            Tcw = np.linalg.inv(Twc)
            for n, Two in world.items():
                Tco = Tcw @ Two
                X = pts[n] @ Tco[:3, :3].T + Tco[:3, 3]
                z = X[:, 2]
                ok = z > 100
                if ok.sum() < 50:
                    continue
                uv = X[ok, :2] / z[ok, None] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
                zz = z[ok]
                u, v = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
                zbuf = np.full((H + 4, W + 4), np.inf)
                for du in (-1, 0, 1):
                    for dv in (-1, 0, 1):
                        uu, vv = np.clip(u + du, -2, W + 1) + 2, np.clip(v + dv, -2, H + 1) + 2
                        np.minimum.at(zbuf, (vv, uu), zz)
                front = zz <= zbuf[np.clip(v, -2, H + 1) + 2, np.clip(u, -2, W + 1) + 2] + 8.0
                inimg = (u >= 0) & (u < W) & (v >= 0) & (v < H)
                surf = front                                   # camera-facing surface, in or out of the image
                ui, vi = np.clip(u, 0, W - 1), np.clip(v, 0, H - 1)
                dm = depth[vi, ui]
                vis = surf & inimg & ((dm == 0) | (dm >= zz - 25.0))
                n_surf = int(surf.sum())
                if n_surf == 0:
                    continue
                vf = float(vis.sum() / n_surf)
                su, sv = u[surf], v[surf]
                x0, y0, x1, y1 = max(su.min(), 0), max(sv.min(), 0), min(su.max(), W - 1), min(sv.max(), H - 1)
                if x1 <= x0 or y1 <= y0:
                    continue
                if vis.sum() > 0:
                    bu, bv = u[vis], v[vis]; bb = [int(bu.min()), int(bv.min()), int(bu.max() - bu.min() + 1), int(bv.max() - bv.min() + 1)]
                else:
                    bb = [-1, -1, -1, -1]
                g.append({"cam_R_m2c": Tco[:3, :3].ravel().tolist(), "cam_t_m2c": Tco[:3, 3].tolist(), "obj_id": names[n]["obj_id"]})
                inf.append({"bbox_obj": [int(x0), int(y0), int(x1 - x0 + 1), int(y1 - y0 + 1)], "bbox_visib": bb,
                            "visib_fract": vf, "px_count_visib": int(vis.sum())})
        gt[str(fid)] = g; info[str(fid)] = inf
    (scene / "scene_gt.json").write_text(json.dumps(gt))
    (scene / "scene_gt_info.json").write_text(json.dumps(info))
    nvis = sum(1 for v in info.values() for i in v if i["visib_fract"] >= 0.1)
    print(f"eval frames {len(gt)}, instances {sum(len(v) for v in gt.values())}, visib>=0.1 {nvis}", flush=True)


if __name__ == "__main__":
    main()
