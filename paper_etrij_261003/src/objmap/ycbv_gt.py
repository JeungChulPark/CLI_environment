"""Build the GT object maps and the common camera trajectories of the YCB-V test sequences.

The camera poses shipped in scene_camera.json (cam_R_w2c / cam_t_w2c) disagree with the
object GT poses by 4-10 mm median and up to 31 mm, while the object poses agree with each
other as a rigid set within 0.1 mm. So the camera of every frame is re-fitted to the object
GT poses (alternating: object world poses <- cameras, cameras <- object world poses), and
the re-fitted trajectory is the one every method gets as input.

Output per scene, gt/ycbv/<scene>/:
  map.json, points/*.npy   GT object map (common format, label = obj_id, CAD surface at T_w_o)
  cameras.json             K, image size, depth scale, per-frame T_wc (camera->world, metres)
  poses_tum.txt            same trajectory as 'frame_id tx ty tz qx qy qz qw'
  visibility.json          per frame {obj_id: visib_fract}
  fit.json                 re-fit residuals (given vs re-fitted cameras)
gt/ycbv_vis/<scene>/ is the same map cut to the CAD surface that was seen: a CAD point is
kept if, in at least one frame (every --vis-step-th), it projects inside the object's
visible mask and within --vis-tol of the measured depth. Fusion methods can only map the
seen surface, so this is the default GT of the IoU / surface-distance scores.

Usage: python ycbv_gt.py [--scenes 48 49 ...]
"""
import argparse
import json

import shutil

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from common import GT_DIR, PROMPTS, YCB_NAMES, YCBV, cad_points, save_map, transform

SEQ = YCBV / "full" / "test"
VIS_STEP = 5      # frames
VIS_TOL = 0.01    # m


def T_of(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, float) / 1000.0
    return T


def mean_pose(Ts):
    """Chordal L2 mean of rigid transforms."""
    Ts = np.asarray(Ts)
    U, _, Vt = np.linalg.svd(Ts[:, :3, :3].sum(0))
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, Ts[:, :3, 3].mean(0)
    return T


def residual(T_wc, T_co, T_wo):
    """Position (mm) and rotation (deg) gap between T_wc*T_co and T_wo."""
    D = np.linalg.inv(T_wo) @ T_wc @ T_co
    ang = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
    return np.linalg.norm(D[:3, 3]) * 1000, ang


def build_scene(scene, iters=5):
    d = SEQ / f"{scene:06d}"
    gt = json.load(open(d / "scene_gt.json"))
    cam = json.load(open(d / "scene_camera.json"))
    info = json.load(open(d / "scene_gt_info.json"))
    frames = sorted(gt, key=int)

    T_co = {f: {o["obj_id"]: T_of(o["cam_R_m2c"], o["cam_t_m2c"]) for o in gt[f]} for f in frames}
    T_wc0 = {f: np.linalg.inv(T_of(cam[f]["cam_R_w2c"], cam[f]["cam_t_w2c"])) for f in frames}
    oids = sorted({o for f in frames for o in T_co[f]})

    T_wc = dict(T_wc0)
    for _ in range(iters):
        T_wo = {i: mean_pose([T_wc[f] @ T_co[f][i] for f in frames if i in T_co[f]]) for i in oids}
        T_wc = {f: mean_pose([T_wo[i] @ np.linalg.inv(T_co[f][i]) for i in T_co[f]]) for f in frames}
    T_wo = {i: mean_pose([T_wc[f] @ T_co[f][i] for f in frames if i in T_co[f]]) for i in oids}

    def stats(cams):
        r = np.array([residual(cams[f], T_co[f][i], T_wo[i]) for f in frames for i in T_co[f]])
        q = lambda x: [round(float(np.percentile(x, p)), 2) for p in (50, 95, 100)]
        return {"pos_mm_p50_p95_max": q(r[:, 0]), "rot_deg_p50_p95_max": q(r[:, 1])}

    fit = {"given": stats(T_wc0), "refit": stats(T_wc)}

    out = GT_DIR / "ycbv" / f"{scene:06d}"
    vis = {f: {o["obj_id"]: round(v["visib_fract"], 4) for o, v in zip(gt[f], info[f])} for f in frames}
    objects = [{"label": i, "T_w_o": T_wo[i], "points": transform(T_wo[i], cad_points(i)),
                "n_obs": sum(vis[f].get(i, 0) > 0.1 for f in frames)} for i in oids]
    save_map(out, objects, {"dataset": "ycbv", "scene": scene, "kind": "gt",
                            "names": {i: YCB_NAMES[i - 1] for i in oids},
                            "prompts": {i: PROMPTS[i - 1] for i in oids}})

    Ks = {tuple(cam[f]["cam_K"]) for f in frames}
    assert len(Ks) == 1, f"scene {scene}: more than one K"
    json.dump({"K": cam[frames[0]]["cam_K"], "width": 640, "height": 480,
               "depth_scale_mm": cam[frames[0]]["depth_scale"],
               "rgb": "rgb/{:06d}.png", "depth": "depth/{:06d}.png", "seq_dir": str(d),
               "frames": [{"id": int(f), "T_wc": T_wc[f].tolist()} for f in frames]},
              open(out / "cameras.json", "w"))
    with open(out / "poses_tum.txt", "w") as fh:
        for f in frames:
            q = Rotation.from_matrix(T_wc[f][:3, :3]).as_quat()
            fh.write(f"{int(f)} " + " ".join(f"{x:.6f}" for x in (*T_wc[f][:3, 3], *q)) + "\n")
    json.dump(vis, open(out / "visibility.json", "w"))
    json.dump(fit, open(out / "fit.json", "w"), indent=1)

    # seen-surface GT
    K = np.array(cam[frames[0]]["cam_K"]).reshape(3, 3)
    seen = {i: np.zeros(len(cad_points(i)), bool) for i in oids}
    for f in frames[::VIS_STEP]:
        depth = np.array(Image.open(d / "depth" / f"{int(f):06d}.png"), np.float64) \
            * cam[f]["depth_scale"] / 1000.0
        for k, o in enumerate(gt[f]):
            i = o["obj_id"]
            if info[f][k]["visib_fract"] <= 0:
                continue
            mask = np.array(Image.open(d / "mask_visib" / f"{int(f):06d}_{k:06d}.png")) > 0
            P = transform(T_co[f][i], cad_points(i))
            uvw = P @ K.T
            u = np.round(uvw[:, 0] / uvw[:, 2]).astype(int)
            v = np.round(uvw[:, 1] / uvw[:, 2]).astype(int)
            ok = (P[:, 2] > 0) & (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
            idx = np.nonzero(ok)[0]
            z = depth[v[idx], u[idx]]
            hit = mask[v[idx], u[idx]] & (z > 0) & (np.abs(z - P[idx, 2]) < VIS_TOL)
            seen[i][idx[hit]] = True
    vout = GT_DIR / "ycbv_vis" / f"{scene:06d}"
    save_map(vout, [{"label": i, "T_w_o": T_wo[i], "points": transform(T_wo[i], cad_points(i)[seen[i]]),
                     "n_obs": objects[k]["n_obs"]} for k, i in enumerate(oids)],
             {"dataset": "ycbv", "scene": scene, "kind": "gt_seen_surface",
              "vis_step": VIS_STEP, "vis_tol_m": VIS_TOL,
              "seen_fraction": {i: round(float(seen[i].mean()), 3) for i in oids}})
    for fn in ("cameras.json", "poses_tum.txt", "visibility.json", "fit.json"):
        shutil.copy(out / fn, vout / fn)
    fit["seen_fraction"] = {i: round(float(seen[i].mean()), 2) for i in oids}
    return fit, len(frames), oids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, nargs="*", default=list(range(48, 60)))
    a = ap.parse_args()
    for s in a.scenes:
        fit, n, oids = build_scene(s)
        print(f"{s:06d} frames={n} objs={oids} given pos {fit['given']['pos_mm_p50_p95_max']} mm"
              f" -> refit {fit['refit']['pos_mm_p50_p95_max']} mm, rot {fit['refit']['rot_deg_p50_p95_max']} deg"
              f"; seen surface {fit['seen_fraction']}")


if __name__ == "__main__":
    main()
