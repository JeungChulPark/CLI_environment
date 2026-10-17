#!/usr/bin/env python3
"""make_bop_eval.py — official BOP-19 scoring for the estimator comparison on all three datasets.

  bop_root/ycbm   YCB-M (xtion) scenes + YCB-V models; targets = GT instances (visib >= 0.1) on the scored frames
  bop_root/real   recorded lab run (eval frames) + lab CAD; targets from the map-based pseudo GT
  results/        <method>-estcmp_<dataset>-test.csv (all answers of each method, BOP format)
YCB-V uses the standard ycbv folder, its official targets and the CSVs written by run_compare.py --mode bop19.
"""
import json, os, shutil
from pathlib import Path
import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[3]
BL = REPO / "objpose/tools/baselines"
ROOT = BL / "bop_root"
RES = BL / "bop_results"
EST = REPO / "objpose/output/estcmp_260927"
PGT = REPO / "objpose/output/pgt_260915_inlong_middle"
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video")
YCBM = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop/test")


def link(dst, src):
    if dst.is_symlink() or dst.exists():
        return
    os.symlink(src, dst)


def targets(scene_root, frames_of_scene):
    t = []
    for sc, ims in frames_of_scene.items():
        gt = json.loads((scene_root / f"{sc:06d}" / "scene_gt.json").read_text())
        gi = json.loads((scene_root / f"{sc:06d}" / "scene_gt_info.json").read_text())
        for im in sorted(ims):
            cnt = {}
            for r, i in zip(gt.get(str(im), []), gi.get(str(im), [])):
                if i["visib_fract"] >= 0.1:
                    cnt[r["obj_id"]] = cnt.get(r["obj_id"], 0) + 1
            t += [{"scene_id": sc, "im_id": im, "obj_id": o, "inst_count": c} for o, c in sorted(cnt.items())]
    return t


def csv_from_runs(run_dir, name2id, out):
    rows = ["scene_id,im_id,obj_id,score,R,t,time"]
    frames = {}
    for sc in sorted(p for p in run_dir.iterdir() if (p / "sam6d_frames.jsonl").exists()):
        for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
            f = json.loads(l)
            frames.setdefault(int(sc.name), set()).add(f["im_id"])
            tm = f["ms"]["total"] / 1000.0
            for d in f["dets"]:
                R = np.asarray(d["R"]).reshape(-1); t = np.asarray(d["t_mm"]).reshape(-1)
                rows.append(f"{int(sc.name)},{f['im_id']},{name2id[d['object']]},{float(d['score']):.6f},"
                            f"{' '.join(f'{v:.6f}' for v in R)},{' '.join(f'{v:.4f}' for v in t)},{tm:.4f}")
    out.write_text("\n".join(rows) + "\n")
    return frames


def main():
    RES.mkdir(exist_ok=True)
    # ---- YCB-M
    d = ROOT / "ycbm"; d.mkdir(parents=True, exist_ok=True)
    for sub in ("models", "models_eval", "models_fine"):
        link(d / sub, YCBV / sub)
    link(d / "test", YCBM)
    t = json.loads((REPO / "objpose/rt/bop/ycbm_objects.json").read_text())
    n2i = {o["name"]: o["obj_id"] for o in t["objects"]}
    frames = None
    for m in ("orig", "proposed", "gtbox"):
        fr = csv_from_runs(EST / f"ycbm_{m}", n2i, RES / f"{m}-estcmp_ycbm-test.csv")
        frames = frames or fr
    (d / "test_targets_bop19.json").write_text(json.dumps(targets(YCBM, frames)))
    cam = json.loads((YCBM / "000000/scene_camera.json").read_text()); c0 = next(iter(cam.values()))
    K = np.asarray(c0["cam_K"]).reshape(3, 3)
    (d / "camera.json").write_text(json.dumps({"cx": K[0, 2], "cy": K[1, 2], "fx": K[0, 0], "fy": K[1, 1], "width": 640, "height": 480, "depth_scale": c0.get("depth_scale", 1.0)}))
    # ---- recorded run
    d = ROOT / "real"; d.mkdir(parents=True, exist_ok=True)
    (d / "test").mkdir(exist_ok=True)
    link(d / "test" / "000001", PGT / "eval/000001")
    t = json.loads((REPO / "objpose/rt/bop/real_objects.json").read_text())
    n2i = {o["name"]: o["obj_id"] for o in t["objects"]}
    for sub in ("models", "models_eval"):
        md = d / sub; md.mkdir(exist_ok=True)
        info = {}
        for o in t["objects"]:
            src = Path(t["models_dir"]) / f"obj_{o['obj_id']:06d}.ply"
            link(md / src.name, src.resolve())
            v = np.asarray(trimesh.load(src, process=False).vertices)
            mn, mx = v.min(0), v.max(0)
            dia = json.loads((Path(t["models_dir"]) / "models_info.json").read_text())[str(o["obj_id"])]["diameter"]
            info[str(o["obj_id"])] = {"diameter": dia, "min_x": mn[0], "min_y": mn[1], "min_z": mn[2],
                                      "size_x": mx[0] - mn[0], "size_y": mx[1] - mn[1], "size_z": mx[2] - mn[2]}
        (md / "models_info.json").write_text(json.dumps(info, indent=1))
    frames = None
    for m in ("orig", "proposed", "gtbox"):
        fr = csv_from_runs(PGT / f"runs/eval_{m}", n2i, RES / f"{m}-estcmp_real-test.csv")
        frames = frames or fr
    (d / "test_targets_bop19.json").write_text(json.dumps(targets(d / "test", frames)))
    cam = json.loads((PGT / "eval/000001/scene_camera.json").read_text()); c0 = next(iter(cam.values()))
    K = np.asarray(c0["cam_K"]).reshape(3, 3)
    (d / "camera.json").write_text(json.dumps({"cx": K[0, 2], "cy": K[1, 2], "fx": K[0, 0], "fy": K[1, 1], "width": 640, "height": 480, "depth_scale": 1.0}))
    # ---- YCB-V: CSVs of the bop19 runs
    for m in ("orig", "proposed", "gtbox"):
        src = next((EST / f"ycbv_{m}").glob("bop_*_ycbv.csv"))
        shutil.copy(src, RES / f"{m}-estcmp_ycbv-test.csv")
    print("ok")


if __name__ == "__main__":
    main()
