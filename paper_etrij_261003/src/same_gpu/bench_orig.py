#!/usr/bin/env python3
"""(A) ORIGINAL SAM-6D on THIS GPU, eight objects, same frames as bench_ours.py.

ISM = upstream CNOS/SAM-6D test_step path (Instance_Segmentation_Model/model/detector.py):
  FastSAM segment-everything -> remove_very_small_detections -> DINOv2 cls+patch descriptors
  -> semantic (avg top-5 over 42 templates/object, conf 0.2, argmax object)
  -> appearance -> geometric (template projection IoU x visible ratio)
  -> final = (sem + appe + geo*vis)/(2 + vis) -> NMS per object id (0.25).
  One ISM pass over all eight objects' templates (ref descriptors [8, 42, D]), as the
  upstream BOP test_step does for multi-object datasets.
Output = top-1 detection per object; PEM (upstream Pose_Estimation_Model, sam-6d-pem-base)
  runs on each object's top-1 whose ISM score > det_score_thresh 0.2 (upstream default),
  one forward per object with that object's own 42 template features, as the upstream
  run_inference_custom.py does for one object.

Upstream defaults restored programmatically (repo files untouched): FastSAM-x.pt (local config
was changed to FastSAM-s) and dinov2_vitl14 (local config was changed to vits14). Code is the
sam6d_ws copy; vs upstream GitHub it differs only in no_grad->inference_mode, a faster RLE
encoder (unused here: no JSON writing) and a read-only instrumentation side channel.
The ultralytics 8.0 CustomYOLO wrapper cannot be imported with the installed ultralytics 8.4,
so FastSAM is called through ultralytics 8.4 YOLO.predict with the wrapper's exact overrides
(iou 0.9, conf 0.25 [the wrapper overrides 0.05 -> 0.25], max_det 200, imgsz 640, RGB array
input as upstream passes it), then the upstream bilinear postprocess_resize.
pytorch_lightning / hydra / ruamel_yaml are not installed in the env; they are stubbed (only base classes
/ an unused import), which does not change any computation.

    /home/jucpark/anaconda3/envs/sam6d/bin/python bench_orig.py [--dino dinov2_vitl14|dinov2_vits14]
                                                                [--fastsam x|s]
"""
import argparse
import importlib
import json
import os
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import torchvision.transforms as transforms

import common as C

SAM6D = C.ROOT / "sam6d_ws/sam6d_master/SAM-6D"
ISM_DIR = SAM6D / "Instance_Segmentation_Model"
PEM_DIR = SAM6D / "Pose_Estimation_Model"
OLD = Path("/mnt/d/old/CLI_environment/sam6d_ws")
FASTSAM = {"x": OLD / "sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt",
           "s": OLD / "sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-s.pt"}
DINO_DIR = {"dinov2_vitl14": Path.home() / ".cache/sam6d_orig_ckpt/dinov2",
            "dinov2_vits14": C.ROOT / "sam6d_realtime/sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/dinov2"}
PREDEF = OLD / "sam6d_master/SAM-6D/Instance_Segmentation_Model/utils/poses/predefined_poses"
PEM_CKPT = C.ROOT / "sam6d_realtime/sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth"
DEV = torch.device("cuda:0")

# ------------------------------------------------------------------ stubs (no computation)
_pl = types.ModuleType("pytorch_lightning"); _pl.LightningModule = torch.nn.Module
sys.modules["pytorch_lightning"] = _pl
_hy = types.ModuleType("hydra"); _hyu = types.ModuleType("hydra.utils")
_hyu.instantiate = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("hydra stub"))
_hy.utils = _hyu; sys.modules["hydra"] = _hy; sys.modules["hydra.utils"] = _hyu
sys.modules.setdefault("ruamel_yaml", types.ModuleType("ruamel_yaml"))   # utils/inout yaml I/O, unused
try:
    import pynvml; pynvml.nvmlInit()
except BaseException:
    pass

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
CAD = {"milk": "milk_high/Milk_high.ply",
       "choco_hazelnut_high": "choco_hazelnut_color_high/choco_hazelnut_color_with_normal_vertexcolor.ply",
       "Febreze_high": "Febreze_high/Febreze_color_with_normal_vertexcolor.ply",
       "Mugcup_high": "Mugcup_color_high/Mugcup_color_with_normal_vertexcolor.ply",
       "saffron": "saffron/saffron_color_with_normal_vertexcolor.ply",
       "Sikhye_high": "Sikhye_high/Sikhye_color_with_normal_vertexcolor.ply",
       "Bear": "Bear/bear_color_with_normal_vertexcolor.ply",
       "Dinosaur": "Dinosaur/Dinosaur_color_with_normal_vertexcolor.ply"}


def sync():
    torch.cuda.synchronize()


# ------------------------------------------------------------------ FastSAM (upstream generate_masks)
class FastSAMUpstream:
    def __init__(self, ckpt, segmentor_width_size=640):
        from ultralytics import YOLO
        self.model = YOLO(str(ckpt))
        self.args = dict(iou=0.9, conf=0.25, max_det=200, imgsz=segmentor_width_size,
                         verbose=False, device=DEV, half=False, save=False)
        self.segmentor_width_size = segmentor_width_size

    @torch.no_grad()
    def generate_masks(self, image):
        orig_size = image.shape[:2]
        det = self.model.predict(image, **self.args)
        if det[0].masks is None:
            return None
        masks = det[0].masks.data
        boxes = det[0].boxes.data[:, :4]
        d = {"masks": masks.to(DEV), "boxes": boxes.to(DEV)}
        d["masks"] = F.interpolate(d["masks"].unsqueeze(1).float(), size=(orig_size[0], orig_size[1]),
                                   mode="bilinear", align_corners=False)[:, 0, :, :]
        return d


def template_dir(name):
    import yaml
    cfg = yaml.safe_load(open(C.ROOT / "sam6d_realtime/configs/yolo_ism_objects.yaml"))
    o = [o for o in cfg["objects"] if o["name"] == name][0]
    return C.ROOT / "sam6d_realtime" / o["template_dir"]


# ------------------------------------------------------------------ ISM onboarding (run_inference_custom)
def build_ism(dino_name, fastsam):
    desc = CustomDINOv2(model_name=dino_name, token_name="x_norm_clstoken", image_size=224, chunk_size=16,
                        descriptor_width_size=640, checkpoint_dir=str(DINO_DIR[dino_name]),
                        patch_size=14, validpatch_thresh=0.5)
    seg = FastSAMUpstream(FASTSAM[fastsam], 640)
    ism = Instance_Segmentation_Model(
        segmentor_model=seg, descriptor_model=desc,
        onboarding_config=NS(rendering_type="pbr", reset_descriptors=False, level_templates=0),
        matching_config=NS(metric=PairwiseSimilarity(metric="cosine", chunk_size=16),
                           aggregation_function="avg_5", confidence_thresh=0.2),
        post_processing_config=NS(mask_post_processing=NS(min_box_size=0.05, min_mask_size=3e-4),
                                  nms_thresh=0.25),
        log_interval=5, log_dir=str(C.OUT / "_ism_logdir"), visible_thred=0.5, pointcloud_sample_num=2048)
    ism.descriptor_model.model = ism.descriptor_model.model.to(DEV)
    ism.descriptor_model.model.device = DEV
    from PIL import Image
    import trimesh
    descs, appes, pcs = [], [], []
    proc = CropResizePad(224)
    for nm in C.OBJECTS:
        td = template_dir(nm)
        n = len(list(td.glob("*.npy")))
        ims, mks, bxs = [], [], []
        for i in range(n):
            im = Image.open(td / f"rgb_{i}.png"); mk = Image.open(td / f"mask_{i}.png")
            bxs.append(mk.getbbox())
            im = torch.from_numpy(np.array(im.convert("RGB")) / 255).float()
            mk = torch.from_numpy(np.array(mk.convert("L")) / 255).float()
            ims.append(im * mk[:, :, None]); mks.append(mk.unsqueeze(-1))
        ims = torch.stack(ims).permute(0, 3, 1, 2); mks = torch.stack(mks).permute(0, 3, 1, 2)
        bxs = torch.tensor(np.array(bxs))
        ims = proc(images=ims, boxes=bxs).to(DEV); mks = proc(images=mks, boxes=bxs).to(DEV)
        descs.append(ism.descriptor_model.compute_features(ims, token_name="x_norm_clstoken"))
        appes.append(ism.descriptor_model.compute_masked_patch_feature(ims, mks[:, 0, :, :]))
        mesh = trimesh.load_mesh(str(OLD / "data/cad" / CAD[nm]))
        pcs.append(torch.tensor(mesh.sample(2048).astype(np.float32) / 1000.0))
    tp = np.load(PREDEF / "obj_poses_level2.npy"); tp[:, :3, 3] *= 0.4
    poses = torch.tensor(tp).to(torch.float32).to(DEV)
    ism.ref_data = {"descriptors": torch.stack(descs).data,          # [N_obj, 42, D]
                    "appe_descriptors": torch.stack(appes).data,      # [N_obj, 42, P, D]
                    "poses": poses[np.load(PREDEF / "idx_all_level0_in_level2.npy"), :, :],
                    "pointcloud": torch.stack(pcs).to(DEV)}           # [N_obj, 2048, 3]
    return ism


# ------------------------------------------------------------------ PEM (run_inference_custom)
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
    gorilla.solver.load_checkpoint(model=net, filename=str(PEM_CKPT))
    os.chdir(cwd)
    tcfg = cfg.test_dataset
    obj = {}
    for nm in C.OBJECTS:
        td = template_dir(nm)
        T, P, Ch = [], [], []
        for v in range(tcfg.n_template_view):
            i = int(42 / tcfg.n_template_view * v)
            t, c, p = _tem_single(td, tcfg, i)
            T.append(torch.FloatTensor(t).unsqueeze(0).to(DEV)); Ch.append(torch.IntTensor(c).long().unsqueeze(0).to(DEV))
            P.append(torch.FloatTensor(p).unsqueeze(0).to(DEV))
        with torch.inference_mode():
            tp, tf = net.feature_extraction.get_obj_feats(T, P, Ch)
        mesh = trimesh.load_mesh(str(OLD / "data/cad" / CAD[nm]))
        mp = mesh.sample(tcfg.n_sample_model_point).astype(np.float32) / 1000.0
        obj[nm] = (tp, tf, mp, float(np.max(np.linalg.norm(mp, axis=1))))
    return net, tcfg, obj


def pem_input(rgb_np, depth_raw, K, mask, score, tcfg, mp, radius):
    """_get_pem_test_data for one instance, from memory instead of files (same ops)."""
    whole_depth = depth_raw.astype(np.float32) * 1.0 / 1000.0        # depth_scale 1.0 (mm)
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


# ------------------------------------------------------------------ one frame
def run_frame(ism, net, tcfg, pobj, bgr, depth, K, det_thresh=0.2):
    T = {}
    sync(); t0 = time.perf_counter()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)                      # image_np (RGB uint8)
    props = ism.segmentor_model.generate_masks(rgb)
    n_raw = 0 if props is None else len(props["masks"])
    out = {"n_fastsam": n_raw, "n_after_small": 0, "n_after_conf": 0, "n_after_nms": 0,
           "ism_top1": {}, "pem": {}}
    if props is None:
        sync(); T["proposals"] = 1e3 * (time.perf_counter() - t0)
        T.update(descriptors=0.0, matching=0.0, pem=0.0); T["total"] = T["proposals"]
        return T, out
    dets = Detections(props)
    dets.remove_very_small_detections(config=ism.post_processing_config.mask_post_processing)
    out["n_after_small"] = len(dets)
    sync(); t1 = time.perf_counter(); T["proposals"] = 1e3 * (t1 - t0)
    qd, qa = ism.descriptor_model(rgb, dets)
    sync(); t2 = time.perf_counter(); T["descriptors"] = 1e3 * (t2 - t1)
    sel, pio, sem, best = ism.compute_semantic_score(qd)
    dets.filter(sel)
    qa = qa[sel, :]
    out["n_after_conf"] = len(dets)
    top1 = {}
    if len(dets) > 0:
        appe, ref_aux = ism.compute_appearance_score(best, pio, qa)
        batch = {"depth": torch.from_numpy(depth.astype(np.int32)).unsqueeze(0).to(DEV),
                 "cam_intrinsic": torch.from_numpy(K).unsqueeze(0).to(DEV),
                 "depth_scale": torch.from_numpy(np.array(1.0)).unsqueeze(0).to(DEV)}
        uv = ism.project_template_to_image(best, pio, batch, dets.masks)
        geo, vis = ism.compute_geometric_score(uv, dets, qa, ref_aux, visible_thred=ism.visible_thred)
        final = (sem + appe + geo * vis) / (1 + 1 + vis)
        dets.add_attribute("scores", final)
        dets.add_attribute("object_ids", pio)
        dets.apply_nms_per_object_id(nms_thresh=ism.post_processing_config.nms_thresh)
        out["n_after_nms"] = len(dets)
        sc = dets.scores; oid = dets.object_ids
        for k in torch.unique(oid).tolist():
            idx = torch.nonzero(oid == k)[:, 0]
            j = int(idx[torch.argmax(sc[idx])])
            top1[C.OBJECTS[int(k)]] = (j, float(sc[j]))
        masks_np = {nm: (dets.masks[j] > 0.5).cpu().numpy() for nm, (j, _) in top1.items()}  # force_binary_mask
    sync(); t3 = time.perf_counter(); T["matching"] = 1e3 * (t3 - t2)
    out["ism_top1"] = {nm: round(s, 4) for nm, (_, s) in sorted(top1.items())}
    tp_ = 0.0
    for nm, (j, s) in sorted(top1.items()):
        if s <= det_thresh:
            out["pem"][nm] = "skipped_score<=0.2"; continue
        tp, tf, mp, rad = pobj[nm]
        inp = pem_input(rgb, depth, K, masks_np[nm], s, tcfg, mp, rad)
        if inp is None:
            out["pem"][nm] = "skipped_input"; continue
        with torch.inference_mode():
            inp["dense_po"] = tp.repeat(1, 1, 1); inp["dense_fo"] = tf.repeat(1, 1, 1)
            o = net(inp)
        ps = float((o["pred_pose_score"] * o["score"])[0]) if "pred_pose_score" in o else float(o["score"][0])
        out["pem"][nm] = {"pose_score": round(ps, 4), "t_mm": [round(float(v) * 1000, 1) for v in o["pred_t"][0]]}
    sync(); t4 = time.perf_counter(); T["pem"] = 1e3 * (t4 - t3)
    T["total"] = 1e3 * (t4 - t0)
    return T, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dino", default="dinov2_vitl14")
    ap.add_argument("--fastsam", default="x")
    a = ap.parse_args()
    frames, K = C.load_frames()
    np.random.seed(0); torch.manual_seed(0)
    ism = build_ism(a.dino, a.fastsam)
    net, tcfg, pobj = build_pem()
    for i in C.WARMUP:
        bgr, depth, _ = frames[i]
        run_frame(ism, net, tcfg, pobj, bgr, depth, K)
    sync(); torch.cuda.reset_peak_memory_stats()
    per = []
    for i in C.FRAMES:
        bgr, depth, t_ns = frames[i]
        T, out = run_frame(ism, net, tcfg, pobj, bgr, depth, K)
        n_pem = sum(isinstance(v, dict) for v in out["pem"].values())
        per.append({"frame": i, "t_ns": t_ns, "ms": {k: round(v, 2) for k, v in T.items()},
                    "n_objects_with_ism_detection": len(out["ism_top1"]), "n_pose_output": n_pem, **out})
        print(i, {k: round(v, 1) for k, v in T.items()}, out["n_fastsam"], out["n_after_small"],
              out["n_after_conf"], len(out["ism_top1"]), n_pem, flush=True)
    tag = f"A_original_{a.dino}_fastsam{a.fastsam}"
    res = {
        "pipeline": tag, "gpu": torch.cuda.get_device_name(0), "torch": torch.__version__,
        "objects": C.OBJECTS, "frames": C.FRAMES, "warmup_frames": C.WARMUP,
        "dinov2": a.dino, "fastsam": a.fastsam,
        "total_ms": C.stats([p["ms"]["total"] for p in per]),
        "stage_ms": {k: C.stats([p["ms"][k] for p in per]) for k in ("proposals", "descriptors", "matching", "pem")},
        "n_fastsam_proposals": C.stats([p["n_fastsam"] for p in per]),
        "n_proposals_after_small_filter": C.stats([p["n_after_small"] for p in per]),
        "n_objects_with_ism_detection_per_frame": C.stats([p["n_objects_with_ism_detection"] for p in per]),
        "n_pose_output_per_frame": C.stats([p["n_pose_output"] for p in per]),
        "gpu_mem_peak_MiB": {"max_allocated": round(torch.cuda.max_memory_allocated() / 2**20, 1),
                             "max_reserved": round(torch.cuda.max_memory_reserved() / 2**20, 1)},
        "per_frame": per,
    }
    json.dump(res, open(C.OUT / f"raw_{tag}.json", "w"), indent=1, default=str)
    print(json.dumps({k: v for k, v in res.items() if k != "per_frame"}, indent=1, default=str))


if __name__ == "__main__":
    main()
