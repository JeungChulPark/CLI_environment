"""Checks of the GT builder and the evaluator before any method is run.

1. depth check: depth pixels inside each GT visible mask, back-projected with the re-fitted
   camera, must lie on the GT CAD surface (a few mm); also done with the shipped cameras
2. evaluator checks on synthetic maps written to out/_selftest_<name>/ycbv/<scene>, scored
   against the seen-surface GT (gt/ycbv_vis):
   gt_copy      GT itself                           -> F1 = 1, IoU = 1, SD = 0
   gt_resample  other CAD samples + 2 mm noise     -> F1 = 1, IoU high, SD ~ 1-2 mm
   cad_full     whole CAD at the GT pose (what a method that places CAD models stores)
                                                    -> F1 = 1, IoU < 1 from the unseen side
   depth_fused  observed depth in GT masks eroded by 3 px (every 10th frame), the kind
                of points a fusion method keeps                -> F1 = 1, lower IoU (visible side only)
   perturbed    one GT dropped, one moved 30 cm, one relabelled, a 'table' and a fake
                target added                        -> counts known in advance (asserted)

Usage: python selftest.py [--scenes 48 49 ...]
"""
import argparse
import json
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import binary_erosion
from scipy.spatial import cKDTree

from common import GT_DIR, OUT, cad_points, load_map, save_map, transform
from evaluate import evaluate_scene, fmt


def backproject(depth_mm, mask, K):
    v, u = np.nonzero(mask & (depth_mm > 0))
    z = depth_mm[v, u] / 1000.0
    return np.stack([(u - K[2]) * z / K[0], (v - K[5]) * z / K[4], z], 1)


def frame_points(scene, fr, cams, step_px=2, erode=0):
    """{obj_id: world points} of the visible mask pixels of frame `fr`."""
    d = cams["seq_dir"]
    gt = frame_points.gt.setdefault(scene, json.load(open(f"{d}/scene_gt.json")))
    depth = np.array(Image.open(f"{d}/depth/{fr:06d}.png"), np.float64) * cams["depth_scale_mm"]
    out = {}
    for k, o in enumerate(gt[str(fr)]):
        m = np.array(Image.open(f"{d}/mask_visib/{fr:06d}_{k:06d}.png")) > 0
        if erode:   # mask edges hit the background behind the object
            m = binary_erosion(m, iterations=erode)
        sub = np.zeros_like(m)
        sub[::step_px, ::step_px] = True
        out[o["obj_id"]] = backproject(depth, m & sub, cams["K"])
    return out


frame_points.gt = {}


def depth_check(scene):
    gdir = GT_DIR / "ycbv" / f"{scene:06d}"   # full CAD surface
    cams = json.load(open(gdir / "cameras.json"))
    shipped = json.load(open(f"{cams['seq_dir']}/scene_camera.json"))
    _, gt = load_map(gdir)
    trees = {o["label"]: cKDTree(o["points"]) for o in gt}
    res = {"refit": [], "shipped": []}
    for fr in cams["frames"][::200]:
        pts = frame_points(scene, fr["id"], cams)
        Ts = {"refit": np.array(fr["T_wc"])}
        c = shipped[str(fr["id"])]
        T = np.eye(4)
        T[:3, :3] = np.array(c["cam_R_w2c"]).reshape(3, 3)
        T[:3, 3] = np.array(c["cam_t_w2c"]) / 1000
        Ts["shipped"] = np.linalg.inv(T)
        for k, Tw in Ts.items():
            for oid, P in pts.items():
                if len(P) > 50:
                    res[k].append(np.median(trees[oid].query(transform(Tw, P))[0]) * 1000)
    return {k: round(float(np.median(v)), 2) for k, v in res.items()}


def write_synthetic(scene, rng):
    gdir = GT_DIR / "ycbv_vis" / f"{scene:06d}"
    _, gt = load_map(gdir)
    cams = json.load(open(gdir / "cameras.json"))
    base = lambda name: OUT / f"_selftest_{name}" / "ycbv" / f"{scene:06d}"

    save_map(base("gt_copy"), [{"label": o["label"], "points": o["points"]} for o in gt])

    _, gt_full = load_map(GT_DIR / "ycbv" / f"{scene:06d}")
    save_map(base("cad_full"), [{"label": o["label"], "points": o["points"]} for o in gt_full])

    rs = []
    for o in gt:   # same seen part, other CAD samples (seen test redone on the GT points' KD-tree)
        P = transform(o["T_w_o"], cad_points(o["label"], seed=1))
        P = P[cKDTree(o["points"]).query(P)[0] < 0.003]
        rs.append({"label": o["label"], "points": P + rng.normal(0, 0.002, P.shape)})
    save_map(base("gt_resample"), rs)

    acc = {o["label"]: [] for o in gt}
    for fr in cams["frames"][::10]:
        for oid, P in frame_points(scene, fr["id"], cams, step_px=4, erode=3).items():
            acc[oid].append(transform(np.array(fr["T_wc"]), P))
    save_map(base("depth_fused"), [{"label": k, "points": np.concatenate(v)} for k, v in acc.items() if v])

    # perturbed: drop gt[0], move gt[1] by 30 cm, relabel gt[2] to a class absent from the scene
    present = {o["label"] for o in gt}
    absent = next(i for i in range(1, 22) if i not in present)
    objs = [{"label": gt[1]["label"], "points": gt[1]["points"] + [0.30, 0, 0]},
            {"label": absent, "points": gt[2]["points"]}]
    objs += [{"label": o["label"], "points": o["points"]} for o in gt[3:]]
    objs.append({"label": "table", "points": rng.random((500, 3))})
    objs.append({"label": absent % 21 + 1 if absent % 21 + 1 not in present else absent,
                 "points": rng.random((500, 3)) + 5})
    save_map(base("perturbed"), objs)
    n = len(gt)
    # TP = n-3, FP = moved + relabelled + fake target = 3, FN = dropped + moved + relabelled = 3
    return {"tp": n - 3, "fp": 3, "fn": 3}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, nargs="*", default=[48, 52, 59])
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    ok = True
    for s in a.scenes:
        print(f"== {s:06d} depth -> GT surface median distance (mm): {depth_check(s)}")
        expect = write_synthetic(s, rng)
        for name in ("gt_copy", "gt_resample", "cad_full", "depth_fused", "depth_fused+clean", "perturbed"):
            r = evaluate_scene(OUT / f"_selftest_{name.split('+')[0]}" / "ycbv" / f"{s:06d}", s,
                               do_clean=name.endswith("+clean"))
            print(f"   {name:17s} TP={r['tp']} FP={r['fp']} FN={r['fn']} F1={r['f1']:.3f} "
                  f"mIoU={fmt(r['mean_iou'])} SD={fmt(r['mean_sd_mm'], 2)}mm  "
                  f"frame F1={r['frame_level']['f1']:.3f}  FP{r['fp_labels']} FN{r['fn_labels']}")
            if name == "gt_copy":
                ok &= r["f1"] == 1 and abs(r["mean_iou"] - 1) < 1e-9 and r["mean_sd_mm"] < 1e-6
            elif name in ("gt_resample", "cad_full", "depth_fused+clean"):
                ok &= r["f1"] == 1
            elif name == "perturbed":
                ok &= all(r[k] == v for k, v in expect.items())
    print("SELFTEST", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
