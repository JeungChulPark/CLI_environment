"""HOPE-Video has no test_targets_bop19.json: write one from video_bop/test/*/scene_gt_info.json.
One target per (image, object) with visib_fract >= 0.1, every STEP-th frame of each scene (6 fps keyframes, so
consecutive frames are nearly the same view). Instances of one object in an image are counted in inst_count.

    python make_hope_targets.py [--root .../HOPE/video_bop] [--step 5] [--min-visib 0.1]
"""
import argparse, json
from collections import Counter
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--root", default=str(Path.home() / "DeepLearning/Datasets/paper_benchmarks/HOPE/video_bop"))
ap.add_argument("--step", type=int, default=5)
ap.add_argument("--min-visib", type=float, default=0.1)
a = ap.parse_args()
root, out = Path(a.root), []
for sd in sorted((root / "test").iterdir()):
    gt, info = json.load(open(sd / "scene_gt.json")), json.load(open(sd / "scene_gt_info.json"))
    for iid in sorted(gt, key=int):
        if (int(iid) - 1) % a.step:
            continue
        c = Counter(g["obj_id"] for g, i in zip(gt[iid], info[iid]) if i["visib_fract"] >= a.min_visib)
        out += [{"scene_id": int(sd.name), "im_id": int(iid), "obj_id": o, "inst_count": n} for o, n in sorted(c.items())]
json.dump(out, open(root / "test_targets_bop19.json", "w"))
print(f"{len({(t['scene_id'], t['im_id']) for t in out})} images, {len(out)} targets -> {root / 'test_targets_bop19.json'}")
