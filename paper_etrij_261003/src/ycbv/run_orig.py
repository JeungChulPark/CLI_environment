#!/usr/bin/env python3
"""Original SAM-6D on the BOP YCB-V test list, one image at a time.

Same pipeline as paper_etrij_261003/src/same_gpu/bench_orig.py (read its docstring for the
details/adaptations): FastSAM-x proposals -> DINOv2 ViT-L/14 descriptors -> upstream
semantic (avg top-5, conf 0.2) / appearance / geometric scores, final = (sem + appe +
geo*vis)/(2+vis) -> NMS per object (0.25) -> top-1 per object -> PEM (sam-6d-pem-base) on
each top-1 with ISM score > 0.2, one forward per object with that object's 42-template
features. Answer score = PEM pose score x ISM score (upstream).

YCB-V specifics: the reference bank of each image holds only that image's target objects
(test_targets_bop19), selected by indexing the pre-built 21-object bank before the timed
region. Depth is passed raw with the BOP depth_scale (0.1 mm) to the ISM geometric score,
and in mm to PEM. Templates = the same 42-view blenderproc renders our recognizer uses
(render_custom_templates route), CAD = BOP models/obj_0000XX.ply (mm).

    python run_orig.py [--limit N]
"""
import argparse
import importlib
import json
import os
import sys
import time
import types

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import torchvision.transforms as transforms

import data as D
import paths as P

ISM_DIR = P.SAM6D_ORIG / "Instance_Segmentation_Model"
PEM_DIR = P.SAM6D_ORIG / "Pose_Estimation_Model"
DEV = torch.device("cuda:0")

# ------------------------------------------------------------------ stubs (no computation)
_pl = types.ModuleType("pytorch_lightning"); _pl.LightningModule = torch.nn.Module
sys.modules.setdefault("pytorch_lightning", _pl)
_hy = types.ModuleType("hydra"); _hyu = types.ModuleType("hydra.utils")
_hyu.instantiate = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("hydra stub"))
_hy.utils = _hyu
sys.modules.setdefault("hydra", _hy); sys.modules.setdefault("hydra.utils", _hyu)
sys.modules.setdefault("ruamel_yaml", types.ModuleType("ruamel_yaml"))
if tuple(int(p) for p in torch.__version__.split("+")[0].split(".")[:2]) >= (2, 6):
    _tl = torch.load
    def _tl2(*a, **k):
        k.setdefault("weights_only", False); return _tl(*a, **k)
    torch.load = _tl2

sys.path.insert(0, str(ISM_DIR))
for p in ["provider", "utils", "model", "model/pointnet2"]:
    sys.path.append(str(PEM_DIR / p))
from model.dinov2 import CustomDINOv2                      # noqa: E402
from model.detector import Instance_Segmentation_Model      # noqa: E402
from model.utils import Detections                          # noqa: E402
from model.loss import PairwiseSimilarity                   # noqa: E402
from utils.bbox_utils import CropResizePad                  # noqa: E402
import gorilla                                              # noqa: E402
from data_utils import load_im, get_bbox, get_point_cloud_from_depth, get_resize_rgb_choose  # noqa: E402

NS = types.SimpleNamespace
OIDS = list(range(1, 22))


def sync():
    torch.cuda.synchronize()


def tdir(oid):
    return P.TEMPLATES / f"obj_{oid:06d}" / "templates"


def cad(oid):
    return str(P.YCBV / "models" / f"obj_{oid:06d}.ply")


class FastSAMUpstream:
    def __init__(self, ckpt, segmentor_width_size=640):
        from ultralytics import YOLO
        self.model = YOLO(str(ckpt))
        self.args = dict(iou=0.9, conf=0.25, max_det=200, imgsz=segmentor_width_size,
                         verbose=False, device=DEV, half=False, save=False)

    @torch.no_grad()
    def generate_masks(self, image):
        orig_size = image.shape[:2]
        det = self.model.predict(image, **self.args)
        if det[0].masks is None:
            return None
        d = {"masks": det[0].masks.data.to(DEV), "boxes": det[0].boxes.data[:, :4].to(DEV)}
        d["masks"] = F.interpolate(d["masks"].unsqueeze(1).float(), size=(orig_size[0], orig_size[1]),
                                   mode="bilinear", align_corners=False)[:, 0, :, :]
        return d


def build_ism(model_name="dinov2_vitl14", segmentor=None):
    """model_name / segmentor: overridable for the ablation ladder (ladder/run_ladder.py); defaults = upstream."""
    desc = CustomDINOv2(model_name=model_name, token_name="x_norm_clstoken", image_size=224, chunk_size=16,
                        descriptor_width_size=640, checkpoint_dir=str(P.DINOV2_VITL_DIR),
                        patch_size=14, validpatch_thresh=0.5)
    seg = segmentor or FastSAMUpstream(P.FASTSAM_X, 640)
    ism = Instance_Segmentation_Model(
        segmentor_model=seg, descriptor_model=desc,
        onboarding_config=NS(rendering_type="pbr", reset_descriptors=False, level_templates=0),
        matching_config=NS(metric=PairwiseSimilarity(metric="cosine", chunk_size=16),
                           aggregation_function="avg_5", confidence_thresh=0.2),
        post_processing_config=NS(mask_post_processing=NS(min_box_size=0.05, min_mask_size=3e-4),
                                  nms_thresh=0.25),
        log_interval=5, log_dir=str(P.OUT / "_ism_logdir"), visible_thred=0.5, pointcloud_sample_num=2048)
    ism.descriptor_model.model = ism.descriptor_model.model.to(DEV)
    ism.descriptor_model.model.device = DEV
    from PIL import Image
    import trimesh
    descs, appes, pcs = [], [], []
    proc = CropResizePad(224)
    for oid in OIDS:
        td = tdir(oid)
        ims, mks, bxs = [], [], []
        for i in range(42):
            im = Image.open(td / f"rgb_{i}.png"); mk = Image.open(td / f"mask_{i}.png")
            bxs.append(mk.getbbox())
            im = torch.from_numpy(np.array(im.convert("RGB")) / 255).float()
            mk = torch.from_numpy(np.array(mk.convert("L")) / 255).float()
            ims.append(im * mk[:, :, None]); mks.append(mk.unsqueeze(-1))
        ims = torch.stack(ims).permute(0, 3, 1, 2); mks = torch.stack(mks).permute(0, 3, 1, 2)
        bxs = torch.tensor(np.array(bxs))
        ims = proc(images=ims, boxes=bxs).to(DEV); mks = proc(images=mks, boxes=bxs).to(DEV)
        with torch.inference_mode():
            descs.append(ism.descriptor_model.compute_features(ims, token_name="x_norm_clstoken"))
            appes.append(ism.descriptor_model.compute_masked_patch_feature(ims, mks[:, 0, :, :]))
        mesh = trimesh.load_mesh(cad(oid))
        pcs.append(torch.tensor(mesh.sample(2048).astype(np.float32) / 1000.0))
    tp = np.load(P.PREDEF / "obj_poses_level2.npy"); tp[:, :3, 3] *= 0.4
    poses = torch.tensor(tp).to(torch.float32).to(DEV)
    bank = {"descriptors": torch.stack(descs), "appe_descriptors": torch.stack(appes),
            "poses": poses[np.load(P.PREDEF / "idx_all_level0_in_level2.npy"), :, :],
            "pointcloud": torch.stack(pcs).to(DEV)}
    return ism, bank


def select_bank(ism, bank, oids, cache):
    key = tuple(oids)
    if key not in cache:
        idx = torch.tensor([o - 1 for o in oids], device=DEV)
        cache[key] = {"descriptors": bank["descriptors"][idx], "appe_descriptors": bank["appe_descriptors"][idx],
                      "poses": bank["poses"], "pointcloud": bank["pointcloud"][idx]}
    ism.ref_data = cache[key]


_rgb_tf = transforms.Compose([transforms.ToTensor(),
                              transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])


def _tem_single(path, cfg, i):
    rgb = load_im(str(path / f"rgb_{i}.png")).astype(np.uint8)
    xyz = np.load(str(path / f"xyz_{i}.npy")).astype(np.float32) / 1000.0
    mask = load_im(str(path / f"mask_{i}.png")).astype(np.uint8) == 255
    y1, y2, x1, x2 = get_bbox(mask)
    mask = mask[y1:y2, x1:x2]
    rgb = rgb[:, :, ::-1][y1:y2, x1:x2, :]
    if cfg.rgb_mask_flag:
        rgb = rgb * (mask[:, :, None] > 0).astype(np.uint8)
    rgb = _rgb_tf(np.array(cv2.resize(rgb, (cfg.img_size, cfg.img_size), interpolation=cv2.INTER_LINEAR)))
    choose = (mask > 0).astype(np.float32).flatten().nonzero()[0]
    n = cfg.n_sample_template_point
    ci = np.random.choice(len(choose), n) if len(choose) <= n else np.random.choice(len(choose), n, replace=False)
    choose = choose[ci]
    xyz = xyz[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]
    return rgb, get_resize_rgb_choose(choose, [y1, y2, x1, x2], cfg.img_size), xyz


def build_pem():
    import trimesh
    cfg = gorilla.Config.fromfile(str(PEM_DIR / "config/base.yaml"))
    cwd = os.getcwd(); os.chdir(PEM_DIR)
    net = importlib.import_module("pose_estimation_model").Net(cfg.model).to(DEV).eval()
    gorilla.solver.load_checkpoint(model=net, filename=str(P.PEM_CKPT))
    os.chdir(cwd)
    tcfg = cfg.test_dataset
    obj = {}
    for oid in OIDS:
        td = tdir(oid)
        T, Pp, Ch = [], [], []
        for v in range(tcfg.n_template_view):
            i = int(42 / tcfg.n_template_view * v)
            t, c, p = _tem_single(td, tcfg, i)
            T.append(torch.FloatTensor(t).unsqueeze(0).to(DEV)); Ch.append(torch.IntTensor(c).long().unsqueeze(0).to(DEV))
            Pp.append(torch.FloatTensor(p).unsqueeze(0).to(DEV))
        with torch.inference_mode():
            tp, tf = net.feature_extraction.get_obj_feats(T, Pp, Ch)
        mesh = trimesh.load_mesh(cad(oid))
        mp = mesh.sample(tcfg.n_sample_model_point).astype(np.float32) / 1000.0
        obj[oid] = (tp, tf, mp, float(np.max(np.linalg.norm(mp, axis=1))))
    return net, tcfg, obj


def pem_input(rgb_np, depth_mm, K, mask, score, tcfg, mp, radius):
    whole_depth = depth_mm.astype(np.float32) / 1000.0
    whole_pts = get_point_cloud_from_depth(whole_depth, K)
    mask = np.logical_and(mask > 0, whole_depth > 0)
    if np.sum(mask) <= 32:
        return None
    y1, y2, x1, x2 = get_bbox(mask)
    mask = mask[y1:y2, x1:x2]
    choose = mask.astype(np.float32).flatten().nonzero()[0]
    cloud = whole_pts.copy()[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]
    center = np.mean(cloud, axis=0)
    flag = np.linalg.norm(cloud - center, axis=1) < radius * 1.2
    if np.sum(flag) < 4:
        return None
    choose = choose[flag]; cloud = cloud[flag]
    n = tcfg.n_sample_observed_point
    idx = np.random.choice(len(choose), n) if len(choose) <= n else np.random.choice(len(choose), n, replace=False)
    choose = choose[idx]; cloud = cloud[idx]
    rgb = rgb_np.copy()[y1:y2, x1:x2, :][:, :, ::-1]
    if tcfg.rgb_mask_flag:
        rgb = rgb * (mask[:, :, None] > 0).astype(np.uint8)
    rgb = _rgb_tf(np.array(cv2.resize(rgb, (tcfg.img_size, tcfg.img_size), interpolation=cv2.INTER_LINEAR)))
    rc = get_resize_rgb_choose(choose, [y1, y2, x1, x2], tcfg.img_size)
    return {"pts": torch.FloatTensor(cloud).unsqueeze(0).to(DEV), "rgb": rgb.unsqueeze(0).float().to(DEV),
            "rgb_choose": torch.IntTensor(rc).long().unsqueeze(0).to(DEV),
            "score": torch.FloatTensor([score]).to(DEV),
            "model": torch.FloatTensor(mp).unsqueeze(0).to(DEV),
            "K": torch.FloatTensor(K).unsqueeze(0).to(DEV)}


def run_frame(ism, net, tcfg, pobj, oids, bgr, raw, depth_mm, K, ds, det_thresh=0.2):
    T = {}
    sync(); t0 = time.perf_counter()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    props = ism.segmentor_model.generate_masks(rgb)
    out = {"n_fastsam": 0 if props is None else len(props["masks"]), "ism_top1": {}, "preds": [], "skipped": {}}
    if props is None:
        sync(); T["total"] = 1e3 * (time.perf_counter() - t0)
        return T, out
    dets = Detections(props)
    dets.remove_very_small_detections(config=ism.post_processing_config.mask_post_processing)
    sync(); t1 = time.perf_counter(); T["proposals"] = 1e3 * (t1 - t0)
    qd, qa = ism.descriptor_model(rgb, dets)
    sync(); t2 = time.perf_counter(); T["descriptors"] = 1e3 * (t2 - t1)
    sel, pio, sem, best = ism.compute_semantic_score(qd)
    dets.filter(sel)
    qa = qa[sel, :]
    top1 = {}
    if len(dets) > 0:
        appe, ref_aux = ism.compute_appearance_score(best, pio, qa)
        batch = {"depth": torch.from_numpy(raw.astype(np.int32)).unsqueeze(0).to(DEV),
                 "cam_intrinsic": torch.from_numpy(K).unsqueeze(0).to(DEV),
                 "depth_scale": torch.from_numpy(np.array(ds)).unsqueeze(0).to(DEV)}
        uv = ism.project_template_to_image(best, pio, batch, dets.masks)
        geo, vis = ism.compute_geometric_score(uv, dets, qa, ref_aux, visible_thred=ism.visible_thred)
        final = (sem + appe + geo * vis) / (1 + 1 + vis)
        dets.add_attribute("scores", final)
        dets.add_attribute("object_ids", pio)
        dets.apply_nms_per_object_id(nms_thresh=ism.post_processing_config.nms_thresh)
        sc = dets.scores; oid_t = dets.object_ids
        for k in torch.unique(oid_t).tolist():
            idx = torch.nonzero(oid_t == k)[:, 0]
            j = int(idx[torch.argmax(sc[idx])])
            top1[oids[int(k)]] = (j, float(sc[j]))
        masks_np = {o: (dets.masks[j] > 0.5).cpu().numpy() for o, (j, _) in top1.items()}
    sync(); t3 = time.perf_counter(); T["matching"] = 1e3 * (t3 - t2)
    out["ism_top1"] = {str(o): round(s, 4) for o, (_, s) in sorted(top1.items())}
    poses = []
    for o, (j, s) in sorted(top1.items()):
        if s <= det_thresh:
            out["skipped"][str(o)] = "ism_score<=0.2"; continue
        tp, tf, mp, rad = pobj[o]
        inp = pem_input(rgb, depth_mm, K, masks_np[o], s, tcfg, mp, rad)
        if inp is None:
            out["skipped"][str(o)] = "pem_input"; continue
        with torch.inference_mode():
            inp["dense_po"] = tp; inp["dense_fo"] = tf
            r = net(inp)
        ps = float((r["pred_pose_score"] * r["score"])[0]) if "pred_pose_score" in r else float(r["score"][0])
        poses.append((o, s, ps, r["pred_R"][0], r["pred_t"][0]))
    sync(); t4 = time.perf_counter(); T["pem"] = 1e3 * (t4 - t3); T["total"] = 1e3 * (t4 - t0)
    for o, s, ps, R, t in poses:
        out["preds"].append({"obj_id": o, "R": R.cpu().numpy().tolist(),
                             "t_mm": (t.cpu().numpy() * 1000.0).tolist(), "score": ps, "ism_score": s})
    return T, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    out_path = a.out or str(P.OUT / "pred_sam6d.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    items = D.load_targets()
    if a.limit:
        items = items[:a.limit]
    np.random.seed(0); torch.manual_seed(0)
    ism, bank = build_ism()
    net, tcfg, pobj = build_pem()
    cache = {}

    def run(sid, iid, oids):
        bgr, raw, depth_mm, K, ds = D.load_image(sid, iid)
        select_bank(ism, bank, oids, cache)
        return run_frame(ism, net, tcfg, pobj, oids, bgr, raw, depth_mm, K, ds)

    for sid, iid, oids in items[:a.warmup]:
        run(sid, iid, oids)
    res = D.header("sam6d_original", {
        "pipeline": "upstream SAM-6D ISM (FastSAM-x, DINOv2 ViT-L/14) top-1 per object + PEM",
        "fastsam": str(P.FASTSAM_X), "dinov2": "dinov2_vitl14", "det_score_thresh": 0.2,
        "warmup_images": a.warmup})
    res["images"] = []
    for n, (sid, iid, oids) in enumerate(items):
        T, o = run(sid, iid, oids)
        for p in o["preds"]:
            p["time_ms"] = round(T["total"], 2)
        res["images"].append({"scene_id": sid, "im_id": iid, "targets": oids, "time_ms": round(T["total"], 2),
                              "stage_ms": {k: round(v, 2) for k, v in T.items()}, **o})
        if n % 50 == 0 or n == len(items) - 1:
            print(f"[{n + 1}/{len(items)}] {sid}/{iid} {T['total']:.0f} ms fastsam {o['n_fastsam']} "
                  f"top1 {len(o['ism_top1'])} out {len(o['preds'])}", flush=True)
    res["time_ms"] = D.stats([x["time_ms"] for x in res["images"]])
    res["stage_ms"] = {k: D.stats([x["stage_ms"].get(k, 0.0) for x in res["images"]])
                       for k in ("proposals", "descriptors", "matching", "pem")}
    res["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(res, open(out_path, "w"), indent=1, default=float)
    print("->", out_path, res["time_ms"])


if __name__ == "__main__":
    main()
