"""Live runs scored with every wrong map object counted as a miss.

Every display frame logs the whole object map as seen from that frame (display_objects.jsonl:
name + T_cam_obj of each map object). At video time t (frame_idx / 30 s) each map object is
  correct : its class is in the scene and its pose in that frame is within ADD-S < 10 % of the
            diameter of the GT pose of that class (one correct object per class at most)
  wrong   : anything else (class not in the scene, wrong place, second copy of a class)
Score(t) = correct(t) / (scene objects + wrong(t)): a wrong map object costs as much as a
missed object. Reported at 5, 10, 20, 30 s and at the end of the video (pooled over the scenes),
plus the mean over all display frames.

    python eval_penalized.py --tags v2,orig,... [--out results_penalized.json]
"""
import argparse, json
from pathlib import Path
import numpy as np
import eval_live as EL

ROOT = Path(__file__).resolve().parents[3] / "objpose" / "output"
TT = [5, 10, 20, 30]


def run_one(run, pts, diam, ycbv):
    sc = int(run.name.split("_")[-1])
    src = Path(ycbv) / f"{sc:06d}"
    gt = json.load(open(src / "scene_gt.json")); gi = json.load(open(src / "scene_gt_info.json"))
    ids = sorted(int(k) for k in gt)
    scene = {g["obj_id"] for f in ids for g, i in zip(gt[str(f)], gi[str(f)]) if i["visib_fract"] > 0.1}
    snaps = []                                   # (t, correct, wrong)
    for line in open(run / "display_objects.jsonl"):
        d = json.loads(line); fi = d["frame_idx"]
        if fi >= len(ids):
            continue
        g = {x["obj_id"]: x for x in gt[str(ids[fi])]}
        ok, wrong = set(), 0
        for o in d["objects"]:
            if not o.get("T_cam_obj"):
                continue
            oid = EL.obj_id(o["name"].split("#")[0])
            good = False
            if oid in scene and oid in g and oid not in ok:
                T = np.eye(4); T[:3, :3] = np.reshape(g[oid]["cam_R_m2c"], (3, 3)); T[:3, 3] = np.array(g[oid]["cam_t_m2c"]) / 1000
                good = EL.adds(EL.mat(o["T_cam_obj"]), T, pts[oid]) < 0.1 * diam[oid]
            if good:
                ok.add(oid)
            else:
                wrong += 1
        snaps.append((fi / 30.0, len(ok), wrong))
    return len(scene), snaps


def at(snaps, t):
    s = [x for x in snaps if x[0] <= t]
    return s[-1] if s else (t, 0, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tags", required=True)
    ap.add_argument("--ycbv", default=str(Path.home() / "DeepLearning/Dataset/bop/ycbv/full/test"))
    ap.add_argument("--bop", default=str(Path.home() / "DeepLearning/Dataset/bop/ycbv"))
    ap.add_argument("--out", default=str(Path(__file__).parent / "results_penalized.json"))
    a = ap.parse_args()
    pts, diam = EL.load_models(a.bop)
    res = {}
    for tag in a.tags.split(","):
        rows = [run_one(r, pts, diam, a.ycbv) for r in sorted(ROOT.glob(f"ycbv_live_{tag}_0000??")) if r.is_dir()]
        n = sum(r[0] for r in rows)
        out = {"scene_objects": n}
        for t in TT + ["end"]:
            c = w = 0
            for _, s in rows:
                x = s[-1] if t == "end" else at(s, t)
                c += x[1]; w += x[2]
            out[f"{t}"] = {"correct": c, "wrong": w, "score_pct": round(100 * c / (n + w), 1),
                           "found_pct": round(100 * c / n, 1)}
        allf = [(c, w, n0) for n0, s in rows for _, c, w in s]
        grid = np.arange(0, 61, 1.0)
        cur = []
        for t in grid:
            c = w = 0
            for _, s_ in rows:
                x = at(s_, t); c += x[1]; w += x[2]
            cur.append((float(t), round(100 * c / n, 2), round(100 * c / (n + w), 2), w))
        out["curve"] = cur                      # (t, found %, score %, wrong)
        out["mean_over_frames_pct"] = round(100 * float(np.mean([c / (n0 + w) for c, w, n0 in allf])), 1)
        res[tag] = out
        print(tag, json.dumps({k: v for k, v in out.items() if k != "curve"}))
    json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
