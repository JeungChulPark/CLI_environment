#!/usr/bin/env python3
"""run_pem_batch.py — batched SAM-6D PEM over many (frame,object) bundles.

Replaces the per-bundle subprocess loop (tools/run_pem.sh) which reloaded the
PEM model + MAE backbone + template features on EVERY call (~10-15 s fixed cost
per bundle). Here the PEM model is loaded ONCE and template features are cached
PER OBJECT, so each bundle costs only get_test_data + one forward (~1-3 s).

Reuses run_inference_custom.py's get_test_data / get_templates / visualize
(imported as a module — only its __main__ block is skipped). Run in the
`sam6d_ros_humble` env (gorilla + pointnet2). Reads
outputs/pem_inputs/<bag>/manifest.csv and writes, per bundle,
  <frame_dir>/pem_<object>/sam6d_results/{detection_pem.json, vis_pem.png}
"""
import argparse
import csv
import hashlib
import importlib
import json
import os
import random
import sys
import time

import numpy as np
import torch

REPO = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PEM = os.path.join(REPO, "sam6d_master", "SAM-6D", "Pose_Estimation_Model")
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def _sync():
    """CUDA is async: model(...) returns before the GPU is done. Timing it
    without this measures dispatch (~ms), not compute."""
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def parse_args():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--bags", nargs="+", default=["two_table_diagonal1"])
    p.add_argument("--config", default=os.path.join(PEM, "config", "base.yaml"))
    p.add_argument("--det-score-thresh", type=float, default=0.2)
    return p.parse_args()


def read_manifest(bag):
    path = os.path.join(REPO, "outputs", "pem_inputs", bag, "manifest.csv")
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: (v.strip() if v else v) for k, v in r.items()})
    return rows


def main():
    args = parse_args()
    # PEM imports are relative to its own dir (sys.path + relative MAE ckpt path)
    os.chdir(PEM)
    for sub in ("provider", "utils", "model", os.path.join("model", "pointnet2")):
        sys.path.append(os.path.join(PEM, sub))
    sys.path.insert(0, PEM)
    import gorilla
    import run_inference_custom as ric   # defines get_test_data/get_templates/visualize

    # get_test_data only needs mesh.sample(n) (-> model_points + radius), both
    # constant per object. The heavy part is trimesh.load_mesh (0.2-30s for
    # vertex-color PLYs). So DISK-cache the sampled point pool (.npy, ~tiny) and
    # return a lightweight stub whose .sample(n) serves from it -> the mesh is
    # loaded at most ONCE EVER per object (first run), then never again.
    _PTS_POOL = 8192
    _pts_cache_dir = os.path.join(REPO, "outputs", "pem_inputs", "_model_pts_cache")
    os.makedirs(_pts_cache_dir, exist_ok=True)
    _orig_load = ric.trimesh.load_mesh

    class _StubMesh:
        def __init__(self, pts):
            self._pts = pts
        def sample(self, n):
            if n == len(self._pts):
                return self._pts
            idx = np.random.choice(len(self._pts), n, replace=(n > len(self._pts)))
            return self._pts[idx]

    _mesh_cache = {}
    def _cached_load_mesh(path, *a, **k):
        if path not in _mesh_cache:
            cpath = os.path.join(_pts_cache_dir,
                                 hashlib.md5(path.encode()).hexdigest()[:16] + ".npy")
            if os.path.isfile(cpath):
                pts = np.load(cpath)
            else:
                pts = _orig_load(path, *a, **k).sample(_PTS_POOL).astype(np.float32)
                np.save(cpath, pts)
            _mesh_cache[path] = _StubMesh(pts)
        return _mesh_cache[path]
    ric.trimesh.load_mesh = _cached_load_mesh

    cfg = gorilla.Config.fromfile(args.config)
    cfg.model_name = "pose_estimation_model"
    random.seed(cfg.rd_seed); torch.manual_seed(cfg.rd_seed)

    print("=> creating PEM model (once)")
    MODEL = importlib.import_module(cfg.model_name)
    model = MODEL.Net(cfg.model).to(DEVICE).eval()
    ckpt = os.path.join(PEM, "checkpoints", "sam-6d-pem-base.pth")
    gorilla.solver.load_checkpoint(model=model, filename=ckpt)

    tem_cache = {}   # template_dir -> (tem_pts, tem_feat)
    cache_dir = os.path.join(REPO, "outputs", "pem_inputs", "_tem_feat_cache")
    os.makedirs(cache_dir, exist_ok=True)
    t_tem = t_inf = 0.0   # phase timers
    n_ok = n_fail = 0
    for bag in args.bags:
        rows = read_manifest(bag)
        timing = []   # one row per bundle -> timing_pem.csv
        print(f"\n##### {bag}: {len(rows)} bundles #####")
        for r in rows:
            fdir, obj, cad, tdir, seg = (r["frame_dir"], r["object"], r["cad_path"],
                                         r["template_dir"], r["seg_json"])
            tag = f"{os.path.basename(fdir)}:{obj}"
            try:
                if tdir not in tem_cache:
                    t0 = time.time()
                    # template features depend only on tdir + PEM model -> cache to disk
                    key = hashlib.md5(tdir.encode()).hexdigest()[:16]
                    cpath = os.path.join(cache_dir, f"{key}.pt")
                    if os.path.isfile(cpath):
                        d = torch.load(cpath, map_location=DEVICE)
                        tem_cache[tdir] = (d["tp"], d["tf"])
                    else:
                        all_tem, all_tem_pts, all_tem_choose = ric.get_templates(tdir, cfg.test_dataset)
                        with torch.inference_mode():
                            tp, tf = model.feature_extraction.get_obj_feats(
                                all_tem, all_tem_pts, all_tem_choose)
                        torch.save({"tp": tp.cpu(), "tf": tf.cpu(), "tdir": tdir}, cpath)
                        tem_cache[tdir] = (tp.to(DEVICE), tf.to(DEVICE))
                    t_tem += time.time() - t0
                tem_pts, tem_feat = tem_cache[tdir]

                t0 = time.perf_counter()
                rgb = os.path.join(fdir, "rgb.png")
                depth = os.path.join(fdir, "depth.png")
                cam = os.path.join(fdir, "camera.json")
                input_data, img, whole_pts, model_points, detections = ric.get_test_data(
                    rgb, depth, cam, cad, seg, args.det_score_thresh, cfg.test_dataset)
                _sync(); t_load_b = time.perf_counter() - t0

                t0 = time.perf_counter()
                ninstance = input_data["pts"].size(0)
                with torch.inference_mode():
                    input_data["dense_po"] = tem_pts.repeat(ninstance, 1, 1)
                    input_data["dense_fo"] = tem_feat.repeat(ninstance, 1, 1)
                    out = model(input_data)
                _sync()   # without this the timer measures dispatch, not compute
                t_fwd_b = time.perf_counter() - t0
                t_inf += t_load_b + t_fwd_b
                t_w0 = time.perf_counter()

                if "pred_pose_score" in out:
                    pose_scores = out["pred_pose_score"] * out["score"]
                else:
                    pose_scores = out["score"]
                pose_scores = pose_scores.detach().cpu().numpy()
                pred_rot = out["pred_R"].detach().cpu().numpy()
                pred_trans = out["pred_t"].detach().cpu().numpy() * 1000

                out_dir = os.path.join(fdir, f"pem_{obj}", "sam6d_results")
                os.makedirs(out_dir, exist_ok=True)
                for idx in range(len(detections)):
                    detections[idx]["score"] = float(pose_scores[idx])
                    detections[idx]["R"] = pred_rot[idx].tolist()
                    detections[idx]["t"] = pred_trans[idx].tolist()
                json.dump(detections, open(os.path.join(out_dir, "detection_pem.json"), "w"))

                save_path = os.path.join(out_dir, "vis_pem.png")
                valid = pose_scores == pose_scores.max()
                K = input_data["K"].detach().cpu().numpy()[valid]
                vis = ric.visualize(img, pred_rot[valid], pred_trans[valid],
                                    model_points * 1000, K, save_path)
                vis.save(save_path)
                t_write_b = time.perf_counter() - t_w0
                timing.append((os.path.basename(fdir), obj, ninstance,
                               round(t_load_b, 4), round(t_fwd_b, 4), round(t_write_b, 4),
                               round(t_load_b + t_fwd_b + t_write_b, 4)))
                n_ok += 1
                print(f"  OK  {tag:34s} score={float(pose_scores.max()):.3f}"
                      f"  ({(t_load_b+t_fwd_b)*1000:.0f}ms)")
            except Exception as e:
                n_fail += 1
                print(f"  FAIL {tag:34s} {type(e).__name__}: {e}")

        tim_path = os.path.join(REPO, "outputs", "pem_inputs", bag, "timing_pem.csv")
        with open(tim_path, "w", newline="") as f:
            wcsv = csv.writer(f)
            wcsv.writerow(["frame", "object", "ninstance",
                           "t_load", "t_fwd", "t_write", "t_total"])
            wcsv.writerows(timing)
        print(f"[timing] {tim_path}")

    print(f"\n===== PEM batch done: {n_ok} ok / {n_fail} fail | "
          f"template-load {t_tem:.1f}s, inference {t_inf:.1f}s "
          f"({t_inf/max(1,n_ok):.2f}s/bundle) =====")


if __name__ == "__main__":
    main()
