"""Quick check: the 36-image runs (out/quick) against the full 900-image runs on the same images.

Per level: answers, correct, found %, wrong / image, median time and stage times on the 36 images,
and, where a full run exists, the same numbers of the full run restricted to these 36 images plus
the share of (image, object) cases where both runs agree (both correct / both not).
"""
import json, sys
from pathlib import Path
import numpy as np, trimesh
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import paths as P, data as D, evaluate as E

info = json.load(open(P.YCBV / "models_eval" / "models_info.json"))
M = {o: np.asarray(trimesh.load(str(P.YCBV / "models_eval" / f"obj_{o:06d}.ply"), process=False).vertices, float) for o in range(1, 22)}
Q = HERE.parent / "out" / "quick"; F = HERE.parent / "out"


def judge(pred, keep=None):
    res, ok = {}, {}
    for im in pred["images"]:
        key = (im["scene_id"], im["im_id"])
        if keep is not None and key not in keep:
            continue
        g = {x["obj_id"]: x for x in D.gt_instances(*key)}
        ans = {p["obj_id"]: p for p in im["preds"]}
        for o in im["targets"]:
            c = False
            if o in ans:
                p = ans[o]
                c = E.adds_mm(M[o], g[o]["R"], g[o]["t_mm"], np.asarray(p["R"]).reshape(3, 3), np.asarray(p["t_mm"]).reshape(3)) < 0.1 * info[str(o)]["diameter"]
            ok[(key, o)] = (o in ans, c)
        res[key] = im
    n = len(ok); a = sum(v[0] for v in ok.values()); c = sum(v[1] for v in ok.values())
    t = [im["time_ms"] for im in res.values()]
    st = {}
    for im in res.values():
        for k, v in (im.get("stage_ms") or {}).items():
            st.setdefault(k, []).append(v)
    return ok, {"images": len(res), "targets": n, "answers": a, "correct": c, "found_pct": round(100 * c / n, 1),
                "wrong_per_image": round((a - c) / max(len(res), 1), 3), "time_median_ms": round(float(np.median(t)), 0),
                "stage_median_ms": {k: round(float(np.median(v)), 0) for k, v in st.items() if k != "total"}}


levels = ["L0", "L1", "L2", "L1_th04", "L1_th05", "L3", "L4", "L5", "A_noverify", "A"]
out = {}
for L in levels:
    qf = Q / f"pred_ladder_{L}.json"
    if not qf.exists():
        continue
    q = json.load(open(qf)); qok, qm = judge(q)
    row = {"quick": qm}
    ff = F / f"pred_ladder_{L}.json"
    if ff.exists():
        f = json.load(open(ff))
        if len(f["images"]) == 900:
            fok, fm = judge(f, keep={k for k, _ in qok})
            agree = sum(1 for k in qok if qok[k][1] == fok.get(k, (False, False))[1]) / len(qok)
            row["full_same_images"] = fm; row["agree_pct"] = round(100 * agree, 1)
    out[L] = row
    print(L, json.dumps(row, ensure_ascii=False))
json.dump(out, open(HERE / "quick_compare.json", "w"), indent=1)
