#!/usr/bin/env python3
"""score_designs.py — designs_obchange.py 가 만든 지도(구성 × 방문 × 지도 방식)를 ObChange 정답으로 채점한다. 채점 방식은 inst_iou_obchange.py 와 같다(그 함수를 그대로 쓴다).

  관측 점을 2 cm 안의 가장 가까운 지도 객체(CAD 점)에 배정하고, 정답 물체와 같은 이름의 객체가 0.25 이상 겹치면 맞음(TP). 이름이 붙었는데 맞음이 아닌 객체는 잘못 등록(FP), 맞는 객체가 없는 정답은 놓침(FN).
  <root>/scores.json : [{config, scene, variant, gt, named, tp, tp50, iou_tp(맞은 객체의 겹침 비율 목록), objects[{name, verdict}]}]
    ~/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/score_designs.py --root objpose/output/six_obchange_261011
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE))
import inst_iou_obchange as S   # noqa: E402
ap = argparse.ArgumentParser(); ap.add_argument("--root", type=Path, required=True); ap.add_argument("--tau", type=float, default=0.02); a = ap.parse_args()
TAB = json.loads((S.REPO / "objpose/rt/bop/obchange_objects.json").read_text())["objects"]; YCB = {o["name"]: o["ycb"] for o in TAB}; MODEL = {}; out = []; gtc = {}


def pts(name):
    if name not in MODEL: MODEL[name] = np.load(S.PTS / f"{name}.npy") / 1000.0
    return MODEL[name]


for f in sorted((a.root / "designs").glob("c*_*.json")):
    D = json.loads(f.read_text()); sc = D["scene"]
    if sc not in gtc: gtc[sc] = S.scene_gt(sc)
    P, glab, gn = gtc[sc]; gsz = np.bincount(glab[glab >= 0], minlength=len(gn)); tg = [i for i, g in enumerate(gn) if g in S.CATALOG]
    for v, L in D.items():
        if not isinstance(L, list): continue
        its = [(YCB[m["name"]], pts(m["name"]) @ np.asarray(m["T_w_obj"])[:3, :3].T + np.asarray(m["T_w_obj"])[:3, 3]) for m in L]
        if its:
            plab = S.assign(P, [c for _, c in its], a.tau); psz = np.bincount(plab[plab >= 0], minlength=len(its)); pair = np.zeros((len(gn), len(its)), int); both = (glab >= 0) & (plab >= 0); np.add.at(pair, (glab[both], plab[both]), 1)
            iou = pair / np.maximum(gsz[:, None] + psz[None, :] - pair, 1)
        else: iou = np.zeros((len(gn), 0))
        rep = {}
        for i in tg:
            same = [j for j, (nm, _) in enumerate(its) if nm == gn[i]]
            if same:
                j = same[int(iou[i, same].argmax())]
                if iou[i, j] >= 0.25: rep[i] = j
        best = [float(max([iou[i, j] for j, (nm, _) in enumerate(its) if nm == gn[i]], default=0.0)) for i in tg]
        out.append({"config": D["config"], "scene": sc, "variant": v, "gt": len(tg), "named": int(len(its)), "tp": int(len(rep)), "tp50": int(sum(iou[i, j] >= 0.5 for i, j in rep.items())), "iou_tp": [round(float(iou[i, j]), 3) for i, j in rep.items()], "iou_gt": [round(x, 3) for x in best],
                    "support": [m["support"] for m in L if m["support"] is not None], "verdict": [bool(j in rep.values()) for j in range(len(its))]})
    print(f.stem, flush=True)
(a.root / "scores.json").write_text(json.dumps(out))
(a.root / "gt_centers.json").write_text(json.dumps({sc: [{"name": gn[i], "center": P[glab == i].mean(0).tolist(), "target": gn[i] in S.CATALOG} for i in range(len(gn)) if (glab == i).any()] for sc, (P, glab, gn) in gtc.items()}))
