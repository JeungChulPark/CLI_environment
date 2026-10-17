#!/usr/bin/env python3
"""DualMap 출력(map/*.pkl)을 change_eval 의 지도 형식으로 옮긴다. 중심 = 점군 축 정렬 상자 중심(정답 중심과 같은 정의)."""
import glob, json, os, pickle, sys
sys.path.insert(0, os.path.expanduser("~/DeepLearning/third_party/DualMap"))
import numpy as np
NAME = {"coffee can": "002_master_chef_can", "cracker box": "003_cracker_box", "sugar box": "004_sugar_box", "tomato soup can": "005_tomato_soup_can",
        "pudding box": "008_pudding_box", "gelatin box": "009_gelatin_box", "pitcher": "019_pitcher_base", "bowl": "024_bowl", "mug": "025_mug",
        "marker": "040_large_marker", "foam brick": "061_foam_brick", "frying pan": "027_skillet", "plate": "029_plate"}
run, cls, out = sys.argv[1:4]
names = [l.strip() for l in open(cls) if l.strip()]
objs = []
for f in sorted(glob.glob(run + "/map/*.pkl")):
    o = pickle.load(open(f, "rb")); P = np.asarray(o.pcd.points)
    if len(P) == 0: continue
    lab = names[o.class_id] if isinstance(o.class_id, (int, np.integer)) and 0 <= o.class_id < len(names) else None
    lo, hi = np.percentile(P, 1, 0), np.percentile(P, 99, 0)
    objs.append({"name": NAME.get(lab), "label": lab, "center": ((lo + hi) / 2).tolist(), "extent": (hi - lo).tolist(), "n_pts": int(len(P)),
                 "pts": np.round(P[np.random.default_rng(0).choice(len(P), min(len(P), 600), replace=False)], 3).tolist()})
json.dump(objs, open(out, "w"), indent=1)
print(out.split("/")[-1], len(objs), "objects;", [o["label"] for o in objs if o["name"]])
