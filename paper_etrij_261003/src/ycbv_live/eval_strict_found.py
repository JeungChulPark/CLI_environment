"""'Found by t' under three rules, from the display logs (whole map per display frame).

  A ever      : drawn correctly at least once by t (eval_live.py time_to_map; wrong entries never count)
  B now       : the map at t holds a correct entry of the object (it may have gone wrong since)
  C now+clean : as B, and the map at t holds no wrong entry of the same class (no misplaced copy)
Correct = ADD-S < 10 % of the diameter against the GT pose in that frame.
"""
import json, sys
from pathlib import Path
import numpy as np
import eval_live as EL

ROOT = Path(__file__).resolve().parents[3] / "objpose" / "output"
FULL = Path.home() / "DeepLearning/Dataset/bop/ycbv/full/test"
TT = [5, 10, 20, 30, 1e9]
pts, diam = EL.load_models(str(Path.home() / "DeepLearning/Dataset/bop/ycbv"))
res = {}
for tag in sys.argv[1:]:
    cnt = {r: {t: 0 for t in TT} for r in "ABC"}; n = 0
    for run in sorted(ROOT.glob(f"ycbv_live_{tag}_0000??")):
        if not run.is_dir():
            continue
        src = FULL / run.name[-6:]
        gt = json.load(open(src / "scene_gt.json")); gi = json.load(open(src / "scene_gt_info.json"))
        ids = sorted(int(k) for k in gt)
        scene = {g["obj_id"] for f in ids for g, i in zip(gt[str(f)], gi[str(f)]) if i["visib_fract"] > 0.1}
        n += len(scene)
        ever, snaps = {}, []
        for line in open(run / "display_objects.jsonl"):
            d = json.loads(line); fi = d["frame_idx"]
            if fi >= len(ids):
                continue
            g = {x["obj_id"]: x for x in gt[str(ids[fi])]}
            vis = {x["obj_id"]: i["visib_fract"] > 0.1 for x, i in zip(gt[str(ids[fi])], gi[str(ids[fi])])}
            ok, bad = set(), set()
            for o in d["objects"]:
                if not o.get("T_cam_obj"):
                    continue
                oid = EL.obj_id(o["name"].split("#")[0])
                good = False
                if oid in scene and oid in g and oid not in ok:
                    T = np.eye(4); T[:3, :3] = np.reshape(g[oid]["cam_R_m2c"], (3, 3)); T[:3, 3] = np.array(g[oid]["cam_t_m2c"]) / 1000
                    good = EL.adds(EL.mat(o["T_cam_obj"]), T, pts[oid]) < 0.1 * diam[oid]
                (ok if good else bad).add(oid)
                if good and vis.get(oid):
                    ever.setdefault(oid, fi / 30)
            snaps.append((fi / 30, set(ok), set(bad)))
        for t in TT:
            cnt["A"][t] += sum(1 for o in scene if ever.get(o, 1e18) <= t)
            s = [x for x in snaps if x[0] <= t]
            ok, bad = (s[-1][1], s[-1][2]) if s else (set(), set())
            cnt["B"][t] += len(ok & scene)
            cnt["C"][t] += len((ok - bad) & scene)
    res[tag] = {r: {("end" if t == 1e9 else f"{t}s"): round(100 * c / n, 1) for t, c in v.items()} for r, v in cnt.items()}
    print(tag, n, json.dumps(res[tag]))
json.dump(res, open(Path(__file__).parent / "results_strict_found.json", "w"), indent=1)
