#!/usr/bin/env python3
"""bench_to_bop.py — static-scene RGB-D video datasets -> BOP scene folders for the real-time comparison tools.

The comparison tools (run_compare.py, eval_cmp.py, display_eval.py, build_memory_maps.py) read a BOP test folder:
  <root>/test/<scene>/rgb/NNNNNN.(png|jpg), depth/NNNNNN.png, scene_camera.json (cam_K, depth_scale, cam_R_w2c, cam_t_w2c),
  scene_gt.json (obj_id, cam_R_m2c, cam_t_m2c in mm), scene_gt_info.json (visib_fract, bbox_visib, ...)
  <root>/models, <root>/models_eval (BOP models in mm)
Images are symlinked, not copied. Frame ids start at 1 (as YCB-V; objpose/tools/ycbv_slam/ycbv_rgbd reads rgb/000001.png ...).

  hope   HOPE-Video (NVIDIA, 10 sequences, robot-arm RealSense D415, ~6 fps keyframes). Object poses are given in cm for
         the HOPE-Video meshes, whose axes differ from the BOP hope models: p_bop = P p_mesh with P = [[0,0,1],[1,0,0],[0,1,0]]
         (checked by projecting the textured BOP models onto the images). Camera extrinsics are world->camera in metres.
  ycbm   YCB-M (xtion camera, 'trajectory' recordings, FAT annotation format) with the YCB-V BOP models.

scene_gt_info is computed here: every object is splatted as a dense point cloud into a z-buffer shared by all objects;
a pixel of an object is visible when that object is in front and the measured depth is not more than VIS_TOL in front of it
(something unmodelled covers it otherwise). visib_fract = visible / full silhouette pixels, bbox_visib from the visible ones.
VIS_TOL is 15 mm for HOPE and 50 mm for YCB-M: YCB-M annotations sit up to ~3 cm off the measured depth in some
scenes (000003: every object 15-30 mm behind the measured surface), and 15 mm marked fully visible objects as
occluded (visib_fract 0.05-0.1 for an unoccluded cracker box). --info-only recomputes scene_gt_info.json from the
existing scene_gt.json and also writes gt_depth_check.json (per scene/object: median measured - model depth on the
visible front pixels).

    python objpose/tools/bench_to_bop.py hope
    python objpose/tools/bench_to_bop.py ycbm
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np
import trimesh

DS = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks")
P_HOPE = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
_pts = {}
YCBM_VIS_TOL = 0.05


def dense_points(models: Path, oid: int, n=60000):
    if (models, oid) not in _pts:
        m = trimesh.load(str(models / f"obj_{oid:06d}.ply"), force="mesh")
        _pts[(models, oid)] = np.asarray(trimesh.sample.sample_surface(m, n)[0], np.float64) / 1000.0
    return _pts[(models, oid)]


def gt_info(objs, K, depth_m, models, W=640, H=480, tol=0.015):
    """objs: [(obj_id, T_cam_obj in m)] -> per object dict (BOP scene_gt_info fields)"""
    zbuf = np.full((H, W), np.inf); owner = np.full((H, W), -1)
    full = []
    for k, (oid, T) in enumerate(objs):
        p = dense_points(models, oid) @ T[:3, :3].T + T[:3, 3]
        p = p[p[:, 2] > 0.05]
        u = np.round(K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]).astype(int)
        v = np.round(K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]).astype(int)
        m = np.zeros((H, W), bool); zo = np.full((H, W), np.inf)
        for du in (0, 1):                                   # 2x2 splat closes the gaps between samples
            for dv in (0, 1):
                uu, vv = u + du, v + dv
                ok = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
                np.minimum.at(zo, (vv[ok], uu[ok]), p[ok, 2]); m[vv[ok], uu[ok]] = True
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)).astype(bool)
        full.append((m, zo))
        front = zo < zbuf
        zbuf[front] = zo[front]; owner[front] = k
    out = []
    for k, (m, zo) in enumerate(full):
        vis = (owner == k) & m
        meas = depth_m
        vis &= ~((meas > 0.1) & (meas < zo - tol))
        n_all, n_vis = int(m.sum()), int(vis.sum())
        def bb(mask):
            ys, xs = np.nonzero(mask)
            return [int(xs.min()), int(ys.min()), int(xs.max() - xs.min() + 1), int(ys.max() - ys.min() + 1)] if len(xs) else [-1, -1, -1, -1]
        out.append({"bbox_obj": bb(m), "bbox_visib": bb(vis), "px_count_all": n_all, "px_count_valid": n_all,
                    "px_count_visib": n_vis, "visib_fract": float(n_vis / n_all) if n_all else 0.0})
    return out


def link(dst: Path, src: Path):
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    os.symlink(os.path.relpath(src, dst.parent), dst)


def recompute_info(root: Path, tol: float):
    """scene_gt_info.json from the existing scene_gt.json / scene_camera.json, plus gt_depth_check.json."""
    check = {}
    for sd in sorted((root / "test").iterdir()):
        cam = json.loads((sd / "scene_camera.json").read_text()); gt = json.loads((sd / "scene_gt.json").read_text())
        info, diffs = {}, {}
        for f in sorted(gt, key=int):
            K = np.array(cam[f]["cam_K"]).reshape(3, 3)
            d = cv2.imread(str(sd / "depth" / f"{int(f):06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float64) * cam[f]["depth_scale"] / 1000.0
            objs = []
            for r in gt[f]:
                T = np.eye(4); T[:3, :3] = np.array(r["cam_R_m2c"]).reshape(3, 3); T[:3, 3] = np.array(r["cam_t_m2c"]) / 1000.0
                objs.append((r["obj_id"], T))
            info[f] = gt_info(objs, K, d, root / "models", tol=tol)
            if int(f) % 10 == 1:                                    # depth agreement on every 10th frame
                for (oid, T) in objs:
                    p = dense_points(root / "models", oid) @ T[:3, :3].T + T[:3, 3]
                    u = np.round(K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]).astype(int); v = np.round(K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]).astype(int)
                    ok = (u >= 0) & (u < 640) & (v >= 0) & (v < 480) & (p[:, 2] > 0.05)
                    zo = np.full(d.shape, np.inf); np.minimum.at(zo, (v[ok], u[ok]), p[ok, 2])
                    m = np.isfinite(zo) & (d > 0.1) & (np.abs(d - zo) < 0.1)
                    if m.sum() > 50:
                        diffs.setdefault(str(oid), []).append(float(np.median((d - zo)[m])) * 1000)
        (sd / "scene_gt_info.json").write_text(json.dumps(info))
        check[sd.name] = {k: round(float(np.median(v)), 1) for k, v in diffs.items()}
        vf = [x["visib_fract"] for v in info.values() for x in v]
        print(sd.name, f"visib_fract median {np.median(vf):.2f}", "depth diff mm", check[sd.name], flush=True)
    (root / "gt_depth_check.json").write_text(json.dumps({"vis_tol_m": tol, "scenes": check}, indent=1))


def write_scene(sd: Path, frames, K, depth_scale, models, tol=0.015):
    """frames: [(rgb_path, depth_path, T_w2c (m), [(obj_id, T_cam_obj m)])]"""
    (sd / "rgb").mkdir(parents=True, exist_ok=True); (sd / "depth").mkdir(exist_ok=True)
    cam, gt, info = {}, {}, {}
    for i, (rgb, dep, Tw2c, objs) in enumerate(frames, 1):
        link(sd / "rgb" / f"{i:06d}.png", rgb); link(sd / "depth" / f"{i:06d}.png", dep)   # .png name even for jpg: ycbv_rgbd reads rgb/NNNNNN.png, OpenCV decodes by content
        cam[str(i)] = {"cam_K": K.ravel().tolist(), "depth_scale": depth_scale,
                       "cam_R_w2c": Tw2c[:3, :3].ravel().tolist(), "cam_t_w2c": (Tw2c[:3, 3] * 1000).tolist()}
        gt[str(i)] = [{"obj_id": oid, "cam_R_m2c": T[:3, :3].ravel().tolist(), "cam_t_m2c": (T[:3, 3] * 1000).tolist()} for oid, T in objs]
        d = cv2.imread(str(dep), cv2.IMREAD_UNCHANGED).astype(np.float64) * depth_scale / 1000.0
        info[str(i)] = gt_info(objs, K, d, models, tol=tol)
    for name, obj in (("scene_camera.json", cam), ("scene_gt.json", gt), ("scene_gt_info.json", info)):
        (sd / name).write_text(json.dumps(obj))
    vf = [x["visib_fract"] for v in info.values() for x in v]
    print(sd.name, len(frames), "frames", f"visib_fract median {np.median(vf):.2f}", flush=True)


def hope():
    H = DS / "HOPE"
    root = H / "video_bop"
    name2id = {l.split('|')[2].strip().replace('&', 'And'): int(l.split('|')[1])
               for l in open(H / "bop/hope/dataset_info.md") if l.startswith('| ') and l.split('|')[1].strip().isdigit()}
    (root / "test").mkdir(parents=True, exist_ok=True)
    for sub in ("models", "models_eval"):
        link(root / sub, H / "bop" / sub)
    for si, sc in enumerate(sorted((H / "hope_video").glob("scene_*"))):
        frames = []
        for js in sorted(sc.glob("[0-9][0-9][0-9][0-9].json")):
            j = json.loads(js.read_text()); stem = js.stem
            K = np.array(j["camera"]["intrinsics"], float)
            Tw2c = np.array(j["camera"]["extrinsics"], float)          # world -> camera, metres
            objs = []
            for o in j["objects"]:
                T = np.array(o["pose"], float); T[:3, 3] /= 100.0       # cm -> m, pose of the HOPE-Video mesh
                Tb = T.copy(); Tb[:3, :3] = T[:3, :3] @ P_HOPE.T        # pose of the BOP model
                objs.append((name2id[o["class"]], Tb))
            frames.append((sc / f"{stem}_rgb.jpg", sc / f"{stem}_depth.png", Tw2c, objs))
        write_scene(root / "test" / f"{si:06d}", frames, K, 1.0, root / "models")


SG_NDDS = np.array([[1, -1, 1], [1, 1, 1], [1, 1, -1], [1, -1, -1], [-1, -1, 1], [-1, 1, 1], [-1, 1, -1], [-1, -1, -1]], float)
A_FAT = np.diag([1, -1, -1]) @ np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)


def kabsch(A, B):
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    return R, cb - R @ ca


def ycbm_pose(o, oset, bop_pts_m):
    """FAT annotation -> pose of the YCB-V BOP model in the camera (m).
    The annotated cuboid corners (camera, cm, NDDS order) fit the box +-dims/2 exactly with the dims taken in (z, x, y)
    order; the FAT model is the YCB model rotated by fixed_model_transform (R0) and re-centred on its box; its axes map to
    the cuboid axes by A_FAT (sign chosen by depth and texture agreement over 16 scenes)."""
    C = np.array(o["cuboid"]) / 100.0
    dims = np.array(oset["cuboid_dimensions"])[[2, 0, 1]] / 100.0
    R, t = kabsch(SG_NDDS * dims / 2, C)
    R0 = np.array(oset["fixed_model_transform"]).T[:3, :3] / 100.0
    q = bop_pts_m @ R0.T
    c = (q.min(0) + q.max(0)) / 2
    T = np.eye(4); T[:3, :3] = R @ A_FAT @ R0; T[:3, 3] = t - R @ A_FAT @ c
    return T


def mean_rot(Rs):
    U, _, Vt = np.linalg.svd(sum(Rs))
    return U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt


def ycbm(camera="xtion"):
    """YCB-M 'trajectory' recordings. The objects stay put, so the camera of frame k is recovered from the object poses:
    world = camera of frame 0, T_ck_w = mean over objects of T_ck_o . T_c0_o^-1 (rotation: chordal mean)."""
    Y = DS / "YCB_M"
    root = Y / f"{camera}_bop"
    ycbv = DS / "YCB_Video"
    table = json.loads(Path("/home/jucpark/DeepLearning/CLI_environment_server/objpose/rt/bop/ycbv_objects.json").read_text())
    code2id = {o["ycb"]: o["obj_id"] for o in table["objects"]}
    (root / "test").mkdir(parents=True, exist_ok=True)
    for sub in ("models", "models_eval"):
        link(root / sub, ycbv / sub)
    for si, sc in enumerate(sorted((Y / camera).iterdir())):
        d = sc / "trajectory"
        cam = json.loads((d / "_camera_settings.json").read_text())["camera_settings"][0]["intrinsic_settings"]
        K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]], float)
        osets = {o["class"]: o for o in json.loads((d / "_object_settings.json").read_text())["exported_objects"]}
        ann = []
        for js in sorted(d.glob("[0-9]*.json")):
            j = json.loads(js.read_text())
            objs = [(code2id[o["class"]], ycbm_pose(o, osets[o["class"]], dense_points(root / "models", code2id[o["class"]], 3000)))
                    for o in j["objects"] if o["class"] in code2id]
            ann.append((js.stem, objs))
        ref = {oid: T for oid, T in ann[0][1]}
        frames = []
        for stem, objs in ann:
            rel = [T @ np.linalg.inv(ref[oid]) for oid, T in objs if oid in ref]          # T_ck_c0 per object
            Tw2c = np.eye(4); Tw2c[:3, :3] = mean_rot([r[:3, :3] for r in rel])
            Tw2c[:3, 3] = np.mean([T[:3, 3] - Tw2c[:3, :3] @ ref[oid][:3, 3] for oid, T in objs if oid in ref], 0)
            frames.append((d / f"{stem}.jpg", d / f"{stem}.depth.png", Tw2c, objs))
        write_scene(root / "test" / f"{si:06d}", frames, K, 0.1, root / "models", tol=YCBM_VIS_TOL)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("dataset", choices=["hope", "ycbm"])
    ap.add_argument("--info-only", action="store_true", help="only recompute scene_gt_info.json (+ gt_depth_check.json)")
    a = ap.parse_args()
    if a.info_only:
        recompute_info(DS / ("HOPE/video_bop" if a.dataset == "hope" else "YCB_M/xtion_bop"), 0.015 if a.dataset == "hope" else YCBM_VIS_TOL)
    else:
        hope() if a.dataset == "hope" else ycbm()
