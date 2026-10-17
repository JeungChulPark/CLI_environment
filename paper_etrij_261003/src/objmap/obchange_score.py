"""Score every method on ObChange-Kitchen (visits 2..6, 25 objects) and count what the revisits change.

Maps per method (common format): out/<method>/obchange_kitchen/<visit>      each visit mapped alone
                                 out/<method>/obchange_kitchen_all/<visit>  the continuous map at the end of the visit
Per visit, as evaluate.py: same-class Hungarian pairing (IoU >= 0.25 on 1 cm voxels, or centre <= 5 cm),
`--clean` optional, predictions on the unscored skillet/plate left out (evaluate.drop_dont_care).

Revisit counts (continuous maps, visits 3..6), from the GT of consecutive visits; two GT objects are "the same
placement" when they share the class and their centres are within 10 cm:
  removed  an object of visit k-1 with no same placement in visit k (taken away or moved). It was *mapped* if
           the continuous map at the end of visit k-1 held a prediction of its class within 5 cm of it, and is
           *still in the map* at the end of visit k if such a prediction is there again (not paired with a
           visit-k object). The rate that matters is still-in-map / mapped: a method that never mapped an
           object cannot keep it.
  added    an object of visit k with no same placement in visit k-1. It is *found* if it is paired (TP) in visit k.
Output: out/obchange_scores.json (other rooms: out/obchange_scores_<room>.json) and a printed table.
Other rooms: --room BigRoom|LivingArea|Office|SmallRoom; the visits are the GT folders gt/obchange_<room>/<k:06d>.

    python obchange_score.py [--room Kitchen] [--methods ours_cad ours_obs fmfusion dualmap dualmap_bgr]
"""
import argparse, json

import numpy as np

from common import GT_DIR, OUT, load_map
from evaluate import drop_dont_care, iou3d, match, prep, prf, surface_dist

VISITS = [2, 3, 4, 5, 6]   # Kitchen; main() sets the visits of --room from its GT folders
ROOM = "kitchen"
SAME_PLACE, STALE = 0.10, 0.05


def gt_visits(room):
    return sorted(int(p.name) for p in (GT_DIR / f"obchange_{room}").iterdir() if p.name.isdigit() and (p / "map.json").exists())


def gt_visit(k):
    gdir = GT_DIR / f"obchange_{ROOM}" / f"{k:06d}"
    gm, gt = load_map(gdir)
    return gm, gdir, prep(gt)


def score_visit(method, mode, k, how, thr, clean):
    gm, gdir, gts = gt_visit(k)
    d = OUT / method / mode / f"{k:06d}"
    if not (d / "map.json").exists():
        return None
    _, pr = load_map(d)
    preds, dc = drop_dont_care(prep(pr, clean), gm, gdir)
    pairs = match(preds, gts, how, thr, 0.01)
    ious, sds = [], []
    for i, j, _ in pairs:
        ious.append(iou3d(preds[i]["points"], gts[j]["points"], 0.01))
        sds.append(surface_dist(preds[i]["points"], gts[j]["points"])[0])
    tp, fp, fn = len(pairs), len(preds) - len(pairs), len(gts) - len(pairs)
    return {"visit": k, "tp": tp, "fp": fp, "fn": fn, "n_pred": len(preds), "dont_care": len(dc),
            "iou": ious, "sd": sds, "preds": preds, "gts": gts, "pairs": pairs}


def revisit_counts(rows):
    """rows: {visit: score_visit result} of the continuous map."""
    out = {"removed": 0, "removed_mapped": 0, "removed_still_in_map": 0, "added": 0, "added_found": 0, "per_visit": []}
    near = lambda r, g, excl=(): any(p["oid"] == g["oid"] and i not in excl and np.linalg.norm(p["center"] - g["center"]) <= STALE
                                     for i, p in enumerate(r["preds"]))
    for k in VISITS[1:]:
        if k not in rows or rows[k] is None:
            continue
        _, _, prev = gt_visit(k - 1)
        cur = rows[k]["gts"]
        removed = [g for g in prev if not any(h["oid"] == g["oid"] and np.linalg.norm(h["center"] - g["center"]) <= SAME_PLACE for h in cur)]
        added_idx = [j for j, g in enumerate(cur) if not any(h["oid"] == g["oid"] and np.linalg.norm(h["center"] - g["center"]) <= SAME_PLACE for h in prev)]
        paired_p = {i for i, _, _ in rows[k]["pairs"]}
        paired_g = {j for _, j, _ in rows[k]["pairs"]}
        mapped = [g for g in removed if rows.get(k - 1) is not None and near(rows[k - 1], g)]
        stale = sum(near(rows[k], g, paired_p) for g in mapped)
        found = sum(j in paired_g for j in added_idx)
        out["removed"] += len(removed); out["removed_mapped"] += len(mapped); out["removed_still_in_map"] += stale
        out["added"] += len(added_idx); out["added_found"] += found
        out["per_visit"].append({"visit": k, "removed": len(removed), "mapped": len(mapped), "still_in_map": stale,
                                 "added": len(added_idx), "added_found": found})
    return out


def summarize(rows):
    rs = [r for r in rows.values() if r is not None]
    tp, fp, fn = (sum(r[x] for r in rs) for x in ("tp", "fp", "fn"))
    p, r, f1 = prf(tp, fp, fn)
    ious = [v for r in rs for v in r["iou"]]; sds = [v for r in rs for v in r["sd"]]
    return {"visits": len(rs), "tp": tp, "fp": fp, "fn": fn, "precision": p, "recall": r, "f1": f1,
            "mean_iou": float(np.mean(ious)) if ious else None, "mean_sd_mm": float(np.mean(sds)) if sds else None,
            "dont_care": sum(r["dont_care"] for r in rs),
            "per_visit": [{k: r[k] for k in ("visit", "tp", "fp", "fn", "n_pred", "dont_care")} for r in rs]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--methods", nargs="+", default=["ours_cad", "ours_obs", "fmfusion", "dualmap", "dualmap_bgr"])
    ap.add_argument("--room", default="Kitchen", help="Kitchen | BigRoom | LivingArea | Office | SmallRoom")
    a = ap.parse_args()
    global ROOM, VISITS
    ROOM = a.room.lower()
    VISITS = gt_visits(ROOM)
    per, cont = f"obchange_{ROOM}", f"obchange_{ROOM}_all"
    dst = OUT / ("obchange_scores.json" if ROOM == "kitchen" else f"obchange_scores_{ROOM}.json")
    print(ROOM, "visits", VISITS)
    res = {}
    fmt = lambda v, n=3: "-" if v is None else f"{v:.{n}f}"
    print(f"{'method':12s} {'mode':10s} {'match':7s} {'TP':>3} {'FP':>3} {'FN':>3} {'P':>6} {'R':>6} {'F1':>6} {'mIoU':>6} {'SD':>6}  still/mapped/removed  added-found/added")
    for m in a.methods:
        for mode, mname in ((per, "per-visit"), (cont, "continuous")):
            for how, thr, tag in (("iou", 0.25, "iou"), ("center", 0.05, "ctr5cm")):
                clean = m != "ours_cad"
                rows = {k: score_visit(m, mode, k, how, thr, clean) for k in VISITS}
                s = summarize(rows)
                if mode == cont:
                    s["revisit"] = revisit_counts(rows)
                res[f"{m}|{mode}|{tag}"] = s
                rv = s.get("revisit")
                extra = f"  {rv['removed_still_in_map']:>2}/{rv['removed_mapped']:>2}/{rv['removed']:<2}             {rv['added_found']:>2}/{rv['added']}" if rv else ""
                print(f"{m:12s} {mname:10s} {tag:7s} {s['tp']:>3} {s['fp']:>3} {s['fn']:>3} {fmt(s['precision']):>6} "
                      f"{fmt(s['recall']):>6} {fmt(s['f1']):>6} {fmt(s['mean_iou']):>6} {fmt(s['mean_sd_mm'], 1):>6}{extra}", flush=True)
    json.dump(res, open(dst, "w"), indent=1)
    print("->", dst)


if __name__ == "__main__":
    main()
