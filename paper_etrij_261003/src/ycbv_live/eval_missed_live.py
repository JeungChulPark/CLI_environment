"""Why the live runs miss objects: for each scene object never drawn correctly, what the recogniser
did in the processed frames where the object was visible (visib_fract > 0.1).

  not_candidate   : not among the ISM-accepted candidates (detector miss or ISM gate / assignment)
  pose_rejected   : ISM accepted, rejected by pose verification (reason counted)
  answered_wrong  : answered, pose not within ADD-S < 10 % of the diameter
  answered_right  : answered correctly (then the map did not keep it correct)
"""
import json, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
import eval_live as EL

ROOT = Path(__file__).resolve().parents[3] / "objpose" / "output"
FULL = Path.home() / "DeepLearning/Dataset/bop/ycbv/full/test"
T0 = 1_700_000_000_000_000_000
pts, diam = EL.load_models(str(Path.home() / "DeepLearning/Dataset/bop/ycbv"))
tag = sys.argv[1]
res = json.load(open(Path(__file__).parent / f"results_{tag}.json"))["per_scene"]
tot = Counter(); reasons = Counter(); rows = []
for sc, v in res.items():
    missed = [EL.YCB.index(n) + 1 for n, t in v["t_first_s"].items() if t is None]
    if not missed:
        continue
    run = ROOT / f"ycbv_live_{tag}_{int(sc):06d}"; src = FULL / f"{int(sc):06d}"
    gt = json.load(open(src / "scene_gt.json")); gi = json.load(open(src / "scene_gt_info.json"))
    ids = sorted(int(k) for k in gt)
    dets = defaultdict(list)
    for line in open(run / "sam6d" / "detections.jsonl"):
        d = json.loads(line); dets[d["stamp_ns"]].append(d)
    for oid in missed:
        c = Counter(); rr = Counter()
        for line in open(run / "sam6d" / "frames.jsonl"):
            f = json.loads(line); fi = int(round((f["stamp_ns"] - T0) / 1e9 * 30))
            if fi >= len(ids):
                continue
            gl = [(g, i) for g, i in zip(gt[str(ids[fi])], gi[str(ids[fi])]) if g["obj_id"] == oid]
            if not gl or gl[0][1]["visib_fract"] <= 0.1:
                continue
            g = gl[0][0]; name = f"ycbv_{oid:02d}"; c["visible_frames"] += 1
            ans = [d for d in dets[f["stamp_ns"]] if d["object"] == name]
            cand = [p for p in f["diagnostics"]["pem_candidates"] if p["object"] == name]
            if ans:
                T = np.eye(4); T[:3, :3] = np.reshape(g["cam_R_m2c"], (3, 3)); T[:3, 3] = np.array(g["cam_t_m2c"]) / 1000
                P_ = np.eye(4); P_[:3, :3] = np.asarray(ans[0]["R"]); P_[:3, 3] = np.asarray(ans[0]["t_mm"]) / 1000
                c["answered_right" if EL.adds(P_, T, pts[oid]) < 0.1 * diam[oid] else "answered_wrong"] += 1
            elif cand:
                c["pose_rejected"] += 1; rr[cand[0].get("rejection_reason") or "other"] += 1
            else:
                c["not_candidate"] += 1
        rows.append((int(sc), EL.YCB[oid - 1], dict(c), dict(rr)))
        tot.update(c); reasons.update(rr)
for r in rows:
    print(r)
print("TOTAL", dict(tot)); print("REASONS", dict(reasons))
