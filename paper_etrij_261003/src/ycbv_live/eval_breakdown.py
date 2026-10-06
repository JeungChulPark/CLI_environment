"""End-of-video map breakdown (last display frame of each video, whole map).

Wrong entries:  absent  = class not in the scene
                misplaced = class in the scene, no correct entry of it, entry at the wrong pose
                duplicate = extra entry of a class that already has a correct entry (or a 2nd wrong one)
Missed objects: never   = scene object with no entry of its class in the map
                wrong_only = scene object present only as wrong entries (counted once as missed)
Score = correct / (scene objects + wrong entries).
"""
import json, sys
from collections import Counter
from pathlib import Path
import numpy as np
import eval_live as EL

ROOT = Path(__file__).resolve().parents[3] / "objpose" / "output"
FULL = Path.home() / "DeepLearning/Dataset/bop/ycbv/full/test"
pts, diam = EL.load_models(str(Path.home() / "DeepLearning/Dataset/bop/ycbv"))
out = {}
for tag in sys.argv[1:]:
    c = Counter(); absent_names = Counter(); missed_names = []
    for run in sorted(ROOT.glob(f"ycbv_live_{tag}_0000??")):
        if not run.is_dir():
            continue
        src = FULL / run.name[-6:]
        gt = json.load(open(src / "scene_gt.json")); gi = json.load(open(src / "scene_gt_info.json"))
        ids = sorted(int(k) for k in gt)
        scene = {g["obj_id"] for f in ids for g, i in zip(gt[str(f)], gi[str(f)]) if i["visib_fract"] > 0.1}
        last = None
        for line in open(run / "display_objects.jsonl"):
            d = json.loads(line)
            if d["frame_idx"] < len(ids):
                last = d
        g = {x["obj_id"]: x for x in gt[str(ids[last["frame_idx"]])]}
        ents = []
        for o in last["objects"]:
            if not o.get("T_cam_obj"):
                continue
            oid = EL.obj_id(o["name"].split("#")[0]); good = False
            if oid in scene and oid in g:
                T = np.eye(4); T[:3, :3] = np.reshape(g[oid]["cam_R_m2c"], (3, 3)); T[:3, 3] = np.array(g[oid]["cam_t_m2c"]) / 1000
                good = EL.adds(EL.mat(o["T_cam_obj"]), T, pts[oid]) < 0.1 * diam[oid]
            ents.append((oid, good))
        correct = set()
        for oid, good in ents:
            if good and oid not in correct:
                correct.add(oid); c["correct"] += 1
        seen_wrong = set()
        for oid, good in ents:
            if good and oid in correct and (oid, "c") not in seen_wrong:
                seen_wrong.add((oid, "c")); continue
            if oid not in scene:
                c["wrong_absent"] += 1; absent_names[EL.YCB[oid - 1]] += 1
            elif oid in correct or oid in seen_wrong:
                c["wrong_duplicate"] += 1
            else:
                c["wrong_misplaced"] += 1; seen_wrong.add(oid)
        present = {oid for oid, _ in ents}
        for o in scene:
            if o in correct:
                continue
            c["missed_wrong_only" if o in present else "missed_never"] += 1
            missed_names.append(f"{run.name[-2:]}:{EL.YCB[o - 1]}" + ("(wrong only)" if o in present else ""))
        c["scene"] += len(scene)
    w = c["wrong_absent"] + c["wrong_misplaced"] + c["wrong_duplicate"]
    miss = c["missed_never"] + c["missed_wrong_only"]
    out[tag] = {**c, "wrong_total": w, "missed_total": miss, "counted_as_missed": miss + w,
                "score_pct": round(100 * c["correct"] / (c["scene"] + w), 1),
                "absent_top": absent_names.most_common(6), "missed_names": missed_names}
    print(tag, json.dumps(out[tag], ensure_ascii=False))
json.dump(out, open(Path(__file__).parent / "results_breakdown.json", "w"), indent=1, ensure_ascii=False)
