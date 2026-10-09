"""Break FN/FP of a scored method into causes (all scenes, IoU >= 0.25 pairing as scored):
  FN: 'label' = some prediction of another YCB class has its centre within 5 cm of the GT centre,
      'place' = a prediction of the right class exists within 20 cm but did not pair, 'missed' = neither.
  FP: 'background' = no GT object centre within 20 cm, 'wrong label' = a GT object of another class
      within 5 cm, 'duplicate/offset' = otherwise.
Usage: python fn_fp_breakdown.py <method> [--clean]"""
import json, sys
import numpy as np
from common import load_map, label_to_oid, OUT, GT_DIR

meth = sys.argv[1]; clean = '--clean' in sys.argv
M = json.load(open(OUT / meth / f"metrics_ycbv_vis_iou0.25_vox0.01{'_clean' if clean else ''}.json"))
fn_c = {'label': 0, 'place': 0, 'missed': 0}; fp_c = {'background': 0, 'wrong label': 0, 'duplicate/offset': 0}
for sc in M['scenes']:
    s = sc['scene']
    _, gt = load_map(GT_DIR / 'ycbv_vis' / f'{s:06d}')
    _, pr = load_map(OUT / meth / 'ycbv' / f'{s:06d}')
    G = [(label_to_oid(o['label']), o['points'].mean(0)) for o in gt]
    P = [(label_to_oid(o['label']), o['points'].mean(0)) for o in pr if label_to_oid(o['label'])]
    for oid in sc['fn_labels']:
        g = next(c for l, c in G if l == oid)
        d_other = [np.linalg.norm(c - g) for l, c in P if l != oid]
        d_same = [np.linalg.norm(c - g) for l, c in P if l == oid]
        if d_same and min(d_same) <= 0.20: fn_c['place'] += 1
        elif d_other and min(d_other) <= 0.05: fn_c['label'] += 1
        else: fn_c['missed'] += 1
    used = set()
    for oid in sc['fp_labels']:
        # the FP instances of this label: take those farthest from a same-class GT first
        cands = [i for i, (l, c) in enumerate(P) if l == oid and i not in used]
        if not cands: continue
        i = cands[0]; used.add(i); c = P[i][1]
        dg = [(np.linalg.norm(gc - c), l) for l, gc in G]
        dmin, lmin = min(dg)
        if dmin > 0.20: fp_c['background'] += 1
        elif dmin <= 0.05 and lmin != oid: fp_c['wrong label'] += 1
        else: fp_c['duplicate/offset'] += 1
print(meth, 'clean' if clean else 'raw', 'FN', fn_c, 'FP', fp_c)
