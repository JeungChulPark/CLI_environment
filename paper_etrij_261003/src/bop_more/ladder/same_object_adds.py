"""Same-object pose accuracy: for each pair of methods, ADD-S AUC (VOCap 0-10 cm) and median ADD-S error
over the (image, object) cases BOTH methods answered (07 §2.3 protocol), plus per-method totals.
  python same_object_adds.py --preds pred_ladder_L0.json,pred_hybS05_900.json [--names 원본,개선안3] --out same_object.json
Prediction files are looked up in paths.OUT (env BOP_OUT). Error of a target = min over its GT instances, as in evaluate.py."""
import json, sys, argparse
from pathlib import Path
import numpy as np, trimesh
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE.parent))
import paths as P, data as D, evaluate as E
info = json.load(open(P.YCBV / "models_eval" / "models_info.json"))
M = {o: np.asarray(trimesh.load(str(P.YCBV / "models_eval" / f"obj_{o:06d}.ply"), process=False).vertices, float) for o in P.OIDS}
ap = argparse.ArgumentParser(); ap.add_argument("--preds", required=True); ap.add_argument("--names", default=""); ap.add_argument("--out", default="")
a = ap.parse_args(); files = a.preds.split(","); names = a.names.split(",") if a.names else [Path(f).stem for f in files]
def errors(pred):
    """{(scene, im, obj): adds_m} for every answered target (correct or not)."""
    out = {}
    for im in pred["images"]:
        key = (im["scene_id"], im["im_id"]); g = {}
        for x in D.gt_instances(*key):                       # all GT instances of an object (IC-BIN has several)
            g.setdefault(x["obj_id"], []).append(x)
        best = {}
        for p in im["preds"]:                                 # one answer per object: the highest score (as evaluate.py)
            if p["obj_id"] not in best or p["score"] > best[p["obj_id"]]["score"]: best[p["obj_id"]] = p
        for o, p in best.items():
            if o not in g or o not in im["targets"]: continue
            out[(key[0], key[1], o)] = min(E.adds_mm(M[o], gi["R"], gi["t_mm"], np.asarray(p["R"]).reshape(3, 3), np.asarray(p["t_mm"]).reshape(3)) for gi in g[o]) / 1000.0
    return out
errs = {n: errors(json.load(open(P.OUT / f))) for n, f in zip(names, files)}
res = {"per_method": {}, "pairs": {}}
for n, e in errs.items():
    v = np.array(list(e.values())); ok = sum(1 for k, x in e.items() if x < 0.1 * info[str(k[2])]["diameter"] / 1000.0)
    res["per_method"][n] = {"answered": len(e), "correct": ok, "adds_auc_all_answered": round(E.vocap(v), 1), "median_err_mm": round(1000 * float(np.median(v)), 2)}
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        A, B = names[i], names[j]; common = sorted(set(errs[A]) & set(errs[B]))
        if not common: continue
        ea = np.array([errs[A][k] for k in common]); eb = np.array([errs[B][k] for k in common])
        res["pairs"][f"{A} vs {B}"] = {"common_answered": len(common),
            A: {"adds_auc": round(E.vocap(ea), 1), "median_err_mm": round(1000 * float(np.median(ea)), 2)},
            B: {"adds_auc": round(E.vocap(eb), 1), "median_err_mm": round(1000 * float(np.median(eb)), 2)},
            "B_better_frac": round(float(np.mean(eb < ea)), 3)}
print(json.dumps(res, ensure_ascii=False, indent=1))
if a.out: json.dump(res, open(a.out, "w"), ensure_ascii=False, indent=1)
