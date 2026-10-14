"""Where each visible GT target is lost, per run: no ISM acceptance (detector miss or gate reject),
accepted by ISM but rejected by pose verification, answered but wrong, or found (correct).
Needs evaluate.py's correctness rule; uses its per-image predictions.

    python det_study/loss_stages.py ours_text ours_text_m1_best ...
"""
import json, sys
from collections import Counter
from pathlib import Path
import numpy as np, trimesh
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import paths as P, data as D
import evaluate as E

info = json.load(open(P.YCBV / "models_eval" / "models_info.json"))
models = {o: np.asarray(trimesh.load(str(P.YCBV / "models_eval" / f"obj_{o:06d}.ply"), process=False).vertices, float) for o in range(1, 22)}
out = {}
for key in sys.argv[1:]:
    pred = json.load(open(P.OUT / f"pred_{key}.json"))
    c = Counter(); per = {}
    for im in pred["images"]:
        gts = {g["obj_id"]: g for g in D.gt_instances(im["scene_id"], im["im_id"])}
        acc = {P.oid_of(n) for n in im["ism_accepted"]}
        rej = {P.oid_of(r["object"]) for r in im["pose_rejected"]}
        ans = {p["obj_id"]: p for p in im["preds"]}
        for o in im["targets"]:
            g = gts[o]
            if o in ans:
                p = ans[o]
                e = E.adds_mm(models[o], g["R"], g["t_mm"], np.asarray(p["R"]).reshape(3, 3), np.asarray(p["t_mm"]).reshape(3))
                ok = e is not None and e < 0.1 * info[str(o)]["diameter"]
                s = "found" if ok else "wrong_answer"
            elif o in acc or o in rej:
                s = "pose_verification_reject"
            else:
                s = "no_ism_accept"
            c[s] += 1
            per.setdefault(P.YCB_NAMES[o - 1], Counter())[s] += 1
    n = sum(c.values())
    out[key] = {"total": n, **{k: round(100 * v / n, 1) for k, v in c.items()}, "per_object": per}
    print(key, {k: v for k, v in out[key].items() if k != "per_object"})
json.dump(out, open(Path(__file__).parent / "out" / "loss_stages.json", "w"), indent=1)
