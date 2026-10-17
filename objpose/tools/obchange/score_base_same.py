#!/usr/bin/env python3
"""score_base_same.py — FM-Fusion · DualMap 의 지도(실시간 재생 결과)를 score_designs.py 와 똑같은 규칙으로 채점한다(같은 정답, 2 cm 배정, 같은 이름 겹침 0.25 이상이면 맞음, 이름이 붙은 나머지는 잘못 등록).
  이름이 물체 목록(YCB 11종)에 없는 객체는 세지 않는다. 지도 폴더는 inst_iou_obchange.py 와 같이 FMF_DIR, DUALMAP_MAPS 로 준다.
    FMF_DIR=… DUALMAP_MAPS=… ~/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/score_base_same.py --scenes smallroom_s2 … --out <json>
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
_a = sys.argv; ap = argparse.ArgumentParser(); ap.add_argument("--scenes", nargs="+", required=True); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--tau", type=float, default=0.02); a = ap.parse_args(); sys.argv = [_a[0]]
import inst_iou_obchange as S   # noqa: E402
out = []
for sc in a.scenes:
    P, glab, gn = S.scene_gt(sc); gsz = np.bincount(glab[glab >= 0], minlength=len(gn)); tg = [i for i, g in enumerate(gn) if g in S.CATALOG]
    for meth, items in S.maps(sc).items():
        if meth not in ("FM-Fusion", "DualMap"): continue
        its = [(nm, np.asarray(c, float)) for nm, c in items if nm in S.CATALOG and len(c)]
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
        out.append({"method": meth, "scene": sc, "gt": len(tg), "named": len(its), "all_objects": len(items), "tp": len(rep), "tp50": int(sum(iou[i, j] >= 0.5 for i, j in rep.items())), "iou_tp": [round(float(iou[i, j]), 3) for i, j in rep.items()]})
        print(sc, meth, out[-1]["gt"], out[-1]["named"], out[-1]["tp"], flush=True)
a.out.write_text(json.dumps(out))
