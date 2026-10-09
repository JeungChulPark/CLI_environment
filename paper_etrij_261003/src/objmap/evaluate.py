"""Score an object map against the GT map of the same scene.

Object level (main table):
  - predictions whose label is not a target object are dropped (table, wall, ...)
  - per class, predictions and GT objects are paired by Hungarian matching on 3D IoU
    (--match iou, pair kept if IoU >= --thr) or on centre distance (--match center,
    pair kept if distance <= --thr metres)
  - TP = kept pairs, FP = unpaired predictions (wrong label included), FN = unpaired GT
  - on TP: 3D IoU of voxel occupancy (--voxel), surface distance = mean of the two
    directed mean nearest-neighbour distances (Chamfer, mm), plus accuracy (pred->GT)
    and completeness (GT->pred)

Predicted points are first reduced to one point per 2 mm voxel (equal density for all).
--clean keeps only the largest connected cluster of each predicted object (1 cm voxels),
for every method alike: 1-2 % of points from mask-edge depth on the background, ~0.8 m
behind the object, can triple the voxel union and sink the IoU of a correctly placed object. Report raw and --clean side by side.

GT points: by default the CAD surface that was seen in the sequence (gt/<dataset>_vis);
--gt <dataset> scores against the full CAD surface instead.

Frame level (supplement): for every frame, every GT object with visib_fract >= --vis is
TP if it is paired in the final map, else FN; FP = unpaired predictions whose centre
projects into that image in front of the camera.

Usage:
  python evaluate.py --pred out/<method>/ycbv/000048 --scene 48
  python evaluate.py --method <method> [--scenes ...]   # all scenes, writes out/<method>/metrics.json
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from common import GT_DIR, OUT, label_to_oid, load_map


def voxels(P, size):
    return set(map(tuple, np.floor(P / size).astype(np.int64)))


def iou3d(A, B, size):
    va, vb = voxels(A, size), voxels(B, size)
    return len(va & vb) / max(1, len(va | vb))


def surface_dist(P, G):
    acc = cKDTree(G).query(P)[0].mean()    # pred -> GT
    comp = cKDTree(P).query(G)[0].mean()   # GT -> pred
    return 500 * (acc + comp), 1000 * acc, 1000 * comp


def match(preds, gts, how, thr, vox):
    """Return [(pred_idx, gt_idx, score)] of kept pairs, same-class only."""
    pairs = []
    for c in {g["oid"] for g in gts} & {p["oid"] for p in preds}:
        pi = [k for k, p in enumerate(preds) if p["oid"] == c]
        gi = [k for k, g in enumerate(gts) if g["oid"] == c]
        S = np.zeros((len(pi), len(gi)))
        for a, i in enumerate(pi):
            for b, j in enumerate(gi):
                if how == "iou":
                    S[a, b] = iou3d(preds[i]["points"], gts[j]["points"], vox)
                else:
                    S[a, b] = -np.linalg.norm(preds[i]["center"] - gts[j]["center"])
        r, k = linear_sum_assignment(-S)
        for a, b in zip(r, k):
            ok = S[a, b] >= thr if how == "iou" else -S[a, b] <= thr
            if ok:
                pairs.append((pi[a], gi[b], float(S[a, b])))
    return pairs


def downsample(P, size=0.002):
    """One point (the mean) per size-m voxel, so every method enters with the same density."""
    if len(P) == 0:
        return P
    key = np.floor(P / size).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    out = np.zeros((len(cnt), 3))
    np.add.at(out, inv, P)
    return out / cnt[:, None]


def clean(P, size=0.01):
    """Keep the largest connected cluster (1 cm voxels, 26-neighbourhood), as DBSCAN-style
    post-processing of fusion mappers does. Applied to every method alike with --clean;
    mask-edge depth on the background otherwise adds clusters far behind the object."""
    if len(P) == 0:
        return P
    key = np.floor(P / size).astype(np.int64)
    uk, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    lo = uk.min(0) - 1
    dims = uk.max(0) - lo + 2
    code = np.ravel_multi_index((uk - lo).T, dims)
    order = np.argsort(code)
    rows, cols = [], []
    for off in np.array(np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])).T.reshape(-1, 3):
        if tuple(off) <= (0, 0, 0):
            continue
        nc = np.ravel_multi_index((uk + off - lo).T, dims)
        pos = np.clip(np.searchsorted(code, nc, sorter=order), 0, len(code) - 1)
        j = order[pos]
        hit = code[j] == nc
        rows.append(np.nonzero(hit)[0])
        cols.append(j[hit])
    r, c = np.concatenate(rows), np.concatenate(cols)
    G = coo_matrix((np.ones(len(r)), (r, c)), shape=(len(uk), len(uk)))
    _, lab = connected_components(G, directed=False)
    big = np.bincount(lab[inv]).argmax()
    return P[lab[inv] == big]


def prep(objs, do_clean=False):
    out = []
    for o in objs:
        oid = label_to_oid(o["label"])
        if oid is None:
            continue
        P = downsample(o["points"])
        P = clean(P) if do_clean else P
        if len(P) == 0:
            continue
        out.append({**o, "points": P, "oid": oid, "center": P.mean(0)})
    return out


def prf(tp, fp, fn):
    p = tp / (tp + fp) if tp + fp else 0.0
    r = tp / (tp + fn) if tp + fn else 0.0
    return p, r, (2 * p * r / (p + r) if p + r else 0.0)


def in_view(center, T_wc, K, w, h):
    pc = np.linalg.inv(T_wc) @ np.append(center, 1)
    if pc[2] <= 0:
        return False
    u = K[0] * pc[0] / pc[2] + K[2]
    v = K[4] * pc[1] / pc[2] + K[5]
    return 0 <= u < w and 0 <= v < h


def evaluate_scene(pred_dir, scene, how="iou", thr=0.25, vox=0.01, vis=0.1, gt="ycbv_vis", do_clean=False):
    gdir = GT_DIR / gt / f"{scene:06d}"
    _, gt = load_map(gdir)
    gts = prep(gt)
    _, pr = load_map(pred_dir)
    n_raw = len(pr)
    preds = prep(pr, do_clean)
    pairs = match(preds, gts, how, thr, vox)

    per_obj = []
    for i, j, _ in pairs:
        P, G = preds[i]["points"], gts[j]["points"]
        sd, acc, comp = surface_dist(P, G)
        per_obj.append({"oid": gts[j]["oid"], "iou": iou3d(P, G, vox), "sd_mm": sd, "acc_mm": acc,
                        "comp_mm": comp,
                        "center_err_mm": 1000 * float(np.linalg.norm(preds[i]["center"] - gts[j]["center"]))})
    tp, fp, fn = len(pairs), len(preds) - len(pairs), len(gts) - len(pairs)
    p, r, f1 = prf(tp, fp, fn)
    paired_p = {i for i, _, _ in pairs}
    paired_g = {gts[j]["oid"] for _, j, _ in pairs}
    res = {"scene": scene, "n_pred_raw": n_raw, "n_pred_target": len(preds), "n_gt": len(gts),
           "tp": tp, "fp": fp, "fn": fn, "precision": p, "recall": r, "f1": f1,
           "fp_labels": sorted(preds[k]["oid"] for k in range(len(preds)) if k not in paired_p),
           "fn_labels": sorted(g["oid"] for g in gts if g["oid"] not in paired_g),
           "mean_iou": float(np.mean([o["iou"] for o in per_obj])) if per_obj else None,
           "mean_sd_mm": float(np.mean([o["sd_mm"] for o in per_obj])) if per_obj else None,
           "per_object": per_obj}

    cams_f = gdir / "cameras.json"
    vis_f = gdir / "visibility.json"
    if cams_f.exists() and vis_f.exists():
        cams = json.load(open(cams_f))
        visj = json.load(open(vis_f))
        K, w, h = cams["K"], cams["width"], cams["height"]
        fp_centers = [preds[k]["center"] for k in range(len(preds)) if k not in paired_p]
        ftp = ffn = ffp = 0
        for fr in cams["frames"]:
            seen = [int(o) for o, v in visj[str(fr["id"])].items() if v >= vis]
            hit = sum(o in paired_g for o in seen)
            ftp += hit
            ffn += len(seen) - hit
            T = np.array(fr["T_wc"])
            ffp += int(sum(in_view(c, T, K, w, h) for c in fp_centers))
        fp_, fr_, ff_ = prf(ftp, ffp, ffn)
        res["frame_level"] = {"tp": ftp, "fp": ffp, "fn": ffn, "precision": fp_, "recall": fr_, "f1": ff_}
    return res


def summarize(rows):
    tp, fp, fn = (sum(r[k] for r in rows) for k in ("tp", "fp", "fn"))
    p, r, f1 = prf(tp, fp, fn)
    objs = [o for r_ in rows for o in r_["per_object"]]
    s = {"tp": tp, "fp": fp, "fn": fn, "precision": p, "recall": r, "f1": f1,
         "mean_iou": float(np.mean([o["iou"] for o in objs])) if objs else None,
         "mean_sd_mm": float(np.mean([o["sd_mm"] for o in objs])) if objs else None,
         "mean_center_err_mm": float(np.mean([o["center_err_mm"] for o in objs])) if objs else None}
    if all("frame_level" in r_ for r_ in rows):
        ft = [sum(r_["frame_level"][k] for r_ in rows) for k in ("tp", "fp", "fn")]
        s["frame_level"] = dict(zip(("tp", "fp", "fn"), ft))
        s["frame_level"].update(zip(("precision", "recall", "f1"), prf(*ft)))
    return s


def fmt(x, d=3):
    return "-" if x is None else f"{x:.{d}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pred", help="one map dir (with --scene)")
    ap.add_argument("--scene", type=int)
    ap.add_argument("--method", help="evaluate out/<method>/<dataset>/<scene> for all scenes")
    ap.add_argument("--dataset", default="ycbv")
    ap.add_argument("--scenes", type=int, nargs="*", default=list(range(48, 60)))
    ap.add_argument("--gt", default=None, help="GT set under gt/: <dataset>_vis (seen surface, default)"
                    " or <dataset> (full CAD surface)")
    ap.add_argument("--match", choices=["iou", "center"], default="iou")
    ap.add_argument("--thr", type=float, default=None, help="IoU >= thr (0.25) or distance <= thr m (0.05)")
    ap.add_argument("--voxel", type=float, default=0.01, help="voxel size for 3D IoU, m")
    ap.add_argument("--clean", action="store_true", help="drop isolated predicted points (all methods alike)")
    ap.add_argument("--vis", type=float, default=0.1, help="frame level: min visib_fract")
    a = ap.parse_args()
    thr = a.thr if a.thr is not None else (0.25 if a.match == "iou" else 0.05)
    gt = a.gt or f"{a.dataset}_vis"
    kw = dict(how=a.match, thr=thr, vox=a.voxel, vis=a.vis, gt=gt, do_clean=a.clean)

    if a.pred:
        r = evaluate_scene(Path(a.pred), a.scene, **kw)
        print(json.dumps({k: v for k, v in r.items() if k != "per_object"}, indent=1))
        return
    rows = []
    print(f"{'scene':>6} {'GT':>3} {'TP':>3} {'FP':>3} {'FN':>3} {'F1':>6} {'mIoU':>6} {'SD mm':>7}  FP labels / FN labels")
    for s in a.scenes:
        d = OUT / a.method / a.dataset / f"{s:06d}"
        if not (d / "map.json").exists():
            print(f"{s:06d}  (no map)")
            continue
        r = evaluate_scene(d, s, **kw)
        rows.append(r)
        print(f"{s:06d} {r['n_gt']:>3} {r['tp']:>3} {r['fp']:>3} {r['fn']:>3} {r['f1']:>6.3f} "
              f"{fmt(r['mean_iou']):>6} {fmt(r['mean_sd_mm'], 1):>7}  {r['fp_labels']} / {r['fn_labels']}")
    s = summarize(rows)
    print(f"{'all':>6} {sum(r['n_gt'] for r in rows):>3} {s['tp']:>3} {s['fp']:>3} {s['fn']:>3} "
          f"{s['f1']:>6.3f} {fmt(s['mean_iou']):>6} {fmt(s['mean_sd_mm'], 1):>7}  "
          f"P={s['precision']:.3f} R={s['recall']:.3f}")
    if "frame_level" in s:
        fl = s["frame_level"]
        print(f"frame level: TP={fl['tp']} FP={fl['fp']} FN={fl['fn']} F1={fl['f1']:.3f}")
    out = OUT / a.method / f"metrics_{gt}_{a.match}{thr}_vox{a.voxel}{'_clean' if a.clean else ''}.json"
    json.dump({"settings": {"gt": gt, "match": a.match, "thr": thr, "voxel": a.voxel, "vis": a.vis,
                            "clean": a.clean},
               "summary": s, "scenes": rows}, open(out, "w"), indent=1)
    print("->", out)


if __name__ == "__main__":
    main()
