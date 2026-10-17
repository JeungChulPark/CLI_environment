#!/usr/bin/env python3
"""orig_sam6d.py — the ORIGINAL SAM-6D (FastSAM ISM + PEM) as a multi-object per-frame estimator.

Baseline for the thesis comparison. It follows SAM-6D's own BOP inference path
(Instance_Segmentation_Model/model/detector.py `test_step` + Pose_Estimation_Model test_bop):

  1. FastSAM segments everything                       (segmentor_model.generate_masks)
  2. very small proposals removed                     (mask_post_processing)
  3. DINOv2 cls / patch descriptors per proposal
  4. semantic score vs the templates of ALL listed objects, proposal -> best object,
     kept if > confidence_thresh (0.2)
  5. appearance + geometric score, final = (sem + appe + geo*vis) / (2 + vis)
  6. NMS per object id (0.25)
  7. PEM on every kept detection with score > det_score_thresh (0.2);
     reported score = PEM pose score x ISM score (as in SAM-6D test_bop)

No YOLO-World, no MobileSAM, no HSV / exclusive assignment, no pose verification: nothing of
this thesis. Model code is a fresh clone of the upstream repo (sam6d_official/, commit 1c2543b),
with the stock FastSAM-s / PEM / DINOv2 checkpoints linked in; the loading helpers come from
this machine's run_batch_inference_fast.py copied next to it. Only the FastSAM call is ported to
ultralytics 8.4 (_FastSAMCompat, same weights and predictor settings).

    core = OrigSam6D(["ycbv_banana", ...], table)       # asset names as in objpose/rt/bop/*.json
    rows, ms, n_props, _ = core.process(bgr, depth_mm_uint16, K)
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
import cv2

REPO = Path(__file__).resolve().parents[3]
SAM6D_DIR = Path(__file__).resolve().parent / "sam6d_official" / "SAM-6D"   # untouched upstream, commit 1c2543b
TEMPLATES = REPO / "sam6d_realtime" / "template"
sys.path.insert(0, str(SAM6D_DIR))
sys.path.insert(0, str(SAM6D_DIR / "Instance_Segmentation_Model"))


class _FastSAMCompat:
    """Drop-in for SAM-6D's model/fast_sam.FastSAM on ultralytics >= 8.1.

    SAM-6D's wrapper imports `ultralytics.yolo` (8.0.x only), but this env needs 8.4 for
    YOLO-World. Same checkpoint (FastSAM-s.pt), same predictor settings as the original:
    conf 0.25 (the wrapper overrides the yaml's 0.05), iou 0.9, max_det 200, imgsz 640,
    non-retina masks resized bilinearly to the input size, boxes in input pixels."""

    class _M:
        def __init__(self, outer): self.outer = outer
        def setup_model(self, device, verbose=False):
            self.outer.current_device = device
            self.outer.yolo.to(device)

    def __init__(self, checkpoint_path, config=None, segmentor_width_size=None, device=None):
        from ultralytics import YOLO
        self.yolo = YOLO(str(checkpoint_path))
        self.iou = float(config.iou_threshold); self.max_det = int(config.max_det)
        self.segmentor_width_size = segmentor_width_size
        self.current_device = device
        self.model = _FastSAMCompat._M(self)

    @torch.inference_mode()
    def generate_masks(self, image):
        import torch.nn.functional as F
        h, w = image.shape[:2]
        # SAM-6D hands the RGB array to a predictor that assumes BGR; kept as is for fidelity
        r = self.yolo.predict(np.ascontiguousarray(image), conf=0.25, iou=self.iou, max_det=self.max_det,
                              imgsz=self.segmentor_width_size, retina_masks=False, verbose=False,
                              device=self.current_device)[0]
        if r.masks is None:
            return {"masks": torch.zeros((0, h, w), device=self.current_device),
                    "boxes": torch.zeros((0, 4), device=self.current_device)}
        masks = r.masks.data.to(self.current_device)
        boxes = r.boxes.data[:, :4].to(self.current_device)
        masks = F.interpolate(masks.unsqueeze(1).float(), size=(h, w), mode="bilinear", align_corners=False)[:, 0]
        return {"masks": masks, "boxes": boxes}


import types as _types                          # noqa: E402
import model as _ism_model_pkg                  # noqa: E402  (SAM-6D ISM package)
_fs = _types.ModuleType("model.fast_sam"); _fs.FastSAM = _FastSAMCompat
sys.modules["model.fast_sam"] = _fs; _ism_model_pkg.fast_sam = _fs
_cwd = os.getcwd()
os.chdir(SAM6D_DIR)
import run_batch_inference_fast as RB           # noqa: E402  (sets up ISM/PEM import paths)
os.chdir(_cwd)
from model.utils import Detections              # noqa: E402  (ISM package)


CKPT_DINO = Path(__file__).resolve().parent / "ckpt_dinov2"


def _load_ism(segmentor, backbone, device):
    """RB.load_ism_model with the DINOv2 backbone selectable. SAM-6D's paper setting is ViT-L/14;
    this machine's SAM-6D copy (and the thesis pipeline) was switched to ViT-S/14."""
    from hydra import initialize, compose
    from hydra.core.global_hydra import GlobalHydra
    from hydra.utils import instantiate
    GlobalHydra.instance().clear()
    cfg_dir = os.path.relpath(os.path.join(RB.ISM_DIR, "configs"), os.path.dirname(os.path.abspath(__file__)))
    with initialize(version_base=None, config_path=cfg_dir):
        cfg = compose(config_name="run_inference.yaml")
    GlobalHydra.instance().clear()
    with initialize(version_base=None, config_path=os.path.join(cfg_dir, "model")):
        cfg.model = compose(config_name="ISM_fastsam.yaml" if segmentor == "fastsam" else "ISM_sam.yaml")
    if segmentor == "fastsam":
        cfg.model.segmentor_model.checkpoint_path = os.path.join(RB.ISM_DIR, "checkpoints", "FastSAM", "FastSAM-s.pt")
    else:
        cfg.model.segmentor_model.sam.checkpoint_dir = os.path.join(RB.ISM_DIR, "checkpoints", "segment-anything", "")
    cfg.model.descriptor_model.model_name = f"dinov2_{backbone}"
    cfg.model.descriptor_model.checkpoint_dir = str(CKPT_DINO) + "/"
    model = instantiate(cfg.model)
    model.descriptor_model.chunk_size = 32
    model.descriptor_model.model = model.descriptor_model.model.to(device)
    model.descriptor_model.model.device = device
    if hasattr(model.segmentor_model, "predictor"):
        model.segmentor_model.predictor.model = model.segmentor_model.predictor.model.to(device)
    else:
        model.segmentor_model.model.setup_model(device=device, verbose=False)
    return model


def _sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


class OrigSam6D:
    def __init__(self, objects: list[str], table: dict, device: str = "cuda:0",
                 det_score_thresh: float = 0.2, segmentor: str = "fastsam", backbone: str = "vitl14",
                 gt_boxes: bool = False):
        self.device = torch.device(device)
        self.objects = list(objects)
        self.det_score_thresh = det_score_thresh
        self.pem_chunk = 16
        by_name = {o["name"]: o for o in table["objects"]}
        models_dir = Path(table["models_dir"])
        cad = {n: str(models_dir / f"obj_{by_name[n]['obj_id']:06d}.ply") for n in self.objects}
        tdir = {n: str(REPO / "sam6d_realtime" / by_name[n]["template_dir"]) if "template_dir" in by_name[n]
                else str(TEMPLATES / n / "templates") for n in self.objects}

        self.gt_mode = gt_boxes
        if gt_boxes:
            # detections are given: GT boxes -> MobileSAM masks (the same masks the proposed
            # pipeline gets), so only SAM-6D's pose stage (PEM) is compared
            from ultralytics import SAM
            self.mobilesam = SAM(str(REPO / "sam6d_realtime" / "mobile_sam.pt"))
        else:
            self._init_ism(segmentor, backbone, cad, tdir)
        self._init_pem(cad, tdir)

    def _init_ism(self, segmentor, backbone, cad, tdir):
        # ---- ISM: one model, reference data of all objects stacked (SAM-6D BOP layout) ----
        self.ism = _load_ism(segmentor, backbone, self.device)
        desc, appe, pcs = [], [], []
        for n in self.objects:
            RB.load_ism_templates(self.ism, tdir[n], cad[n], self.device)
            desc.append(self.ism.ref_data["descriptors"])
            appe.append(self.ism.ref_data["appe_descriptors"])
            pcs.append(self.ism.ref_data["pointcloud"])
        poses = self.ism.ref_data["poses"]
        self.ism.ref_data = {"descriptors": torch.cat(desc, 0), "appe_descriptors": torch.cat(appe, 0),
                             "pointcloud": torch.cat(pcs, 0), "poses": poses}

    def _init_pem(self, cad, tdir):
        # ---- PEM: one network, per-object template features and model points ----
        import gorilla, importlib
        cfg = gorilla.Config.fromfile(os.path.join(RB.PEM_DIR, "config", "base.yaml"))
        cfg.model_name = "pose_estimation_model"
        MODEL = importlib.import_module(cfg.model_name)
        self.pem = MODEL.Net(cfg.model).to(self.device).eval()
        gorilla.solver.load_checkpoint(model=self.pem, filename=os.path.join(RB.PEM_DIR, "checkpoints", "sam-6d-pem-base.pth"))
        self.pcfg = cfg.test_dataset
        self.tem = {}
        for n in self.objects:
            all_tem, all_tem_pts, all_tem_choose = RB._get_pem_templates(tdir[n], self.pcfg, self.device)
            with torch.inference_mode():
                tp, tf = self.pem.feature_extraction.get_obj_feats(all_tem, all_tem_pts, all_tem_choose)
            mp, radius = RB.preload_pem_mesh(cad[n], self.pcfg.n_sample_model_point)
            self.tem[n] = (tp, tf, mp, radius)

    # ------------------------------------------------------------------------------------------
    def _ism(self, rgb, depth_mm, K):
        m = self.ism
        props = m.segmentor_model.generate_masks(rgb)
        det = Detections(props)
        n_props = len(det)
        det.remove_very_small_detections(config=m.post_processing_config.mask_post_processing)
        if len(det) == 0:
            return None, n_props
        qd, qa = m.descriptor_model.forward(rgb, det)
        idx_sel, pred_obj, sem, best_t = m.compute_semantic_score(qd)
        if len(idx_sel) == 0:
            return None, n_props
        det.filter(idx_sel)
        qa = qa[idx_sel, :]
        appe, ref_aux = m.compute_appearance_score(best_t, pred_obj, qa)
        batch = {"depth": torch.from_numpy(depth_mm.astype(np.int32)).unsqueeze(0).to(self.device),
                 "cam_intrinsic": torch.from_numpy(K).unsqueeze(0).to(self.device),
                 "depth_scale": torch.tensor([1.0]).to(self.device)}
        uv = m.project_template_to_image(best_t, pred_obj, batch, det.masks)
        geo, vis = m.compute_geometric_score(uv, det, qa, ref_aux, visible_thred=m.visible_thred)
        final = (sem + appe + geo * vis) / (1 + 1 + vis)
        det.add_attribute("scores", final)
        det.add_attribute("object_ids", pred_obj)
        det.apply_nms_per_object_id(nms_thresh=m.post_processing_config.nms_thresh)
        det.to_numpy()
        return det, n_props

    def _pem(self, name, masks, scores, rgb, depth_m, pts_img, K):
        tp, tf, mp, radius = self.tem[name]
        c = self.pcfg
        R_, C_, CH_, S_ = [], [], [], []
        for mask, sc in zip(masks, scores):
            mask = np.logical_and(mask > 0, depth_m > 0)
            if mask.sum() <= 32:
                continue
            y1, y2, x1, x2 = RB.get_bbox(mask)
            mk = mask[y1:y2, x1:x2]
            choose = mk.astype(np.float32).flatten().nonzero()[0]
            cloud = pts_img[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]
            ctr = cloud.mean(0)
            flag = np.linalg.norm(cloud - ctr, axis=1) < radius * 1.2
            if flag.sum() < 4:
                continue
            choose, cloud = choose[flag], cloud[flag]
            n = c.n_sample_observed_point
            ix = np.random.choice(len(choose), n, replace=len(choose) <= n)
            choose, cloud = choose[ix], cloud[ix]
            crop = rgb[y1:y2, x1:x2, :][:, :, ::-1]            # SAM-6D feeds BGR-ordered crops
            if c.rgb_mask_flag:
                crop = crop * (mk[:, :, None] > 0).astype(np.uint8)
            crop = cv2.resize(crop, (c.img_size, c.img_size), interpolation=cv2.INTER_LINEAR)
            R_.append(RB._rgb_transform(np.array(crop)))
            C_.append(torch.FloatTensor(cloud))
            CH_.append(torch.IntTensor(RB.get_resize_rgb_choose(choose, [y1, y2, x1, x2], c.img_size)).long())
            S_.append(float(sc))
        if not R_:
            return []
        d = self.device
        rows = []
        # per-instance network: chunking only bounds memory (a cluttered image can give >100 detections)
        # a chunk that does not fit the GPU is retried at half the size (same answers, only more passes)
        s, chunk = 0, self.pem_chunk
        while s < len(R_):
            e = min(s + chunk, len(R_)); k = e - s
            inp = out = None
            try:
                inp = {"pts": torch.stack(C_[s:e]).to(d), "rgb": torch.stack(R_[s:e]).to(d),
                       "rgb_choose": torch.stack(CH_[s:e]).to(d), "score": torch.FloatTensor(S_[s:e]).to(d),
                       "model": torch.FloatTensor(mp).unsqueeze(0).repeat(k, 1, 1).to(d),
                       "K": torch.FloatTensor(K).unsqueeze(0).repeat(k, 1, 1).to(d)}
                with torch.inference_mode():
                    inp["dense_po"] = tp.repeat(k, 1, 1)
                    inp["dense_fo"] = tf.repeat(k, 1, 1)
                    out = self.pem(inp)
            except torch.OutOfMemoryError:
                del inp, out
                torch.cuda.empty_cache()
                if chunk == 1:
                    raise
                chunk = max(1, chunk // 2); self.oom_retries = getattr(self, "oom_retries", 0) + 1
                print(f"[orig] PEM chunk of {k} did not fit the GPU, retrying with {chunk}", flush=True)
                continue
            s = e
            if "pred_pose_score" in out:
                ps = (out["pred_pose_score"] * out["score"]).cpu().numpy()
            else:
                ps = out["score"].cpu().numpy()
            Rs = out["pred_R"].cpu().numpy(); ts = out["pred_t"].cpu().numpy() * 1000.0
            rows += [{"object": name, "score": float(ps[i]), "R": Rs[i].tolist(), "t_mm": ts[i].tolist()} for i in range(k)]
            del out, inp
        return rows

    def process_gt(self, bgr, depth_mm, K, boxes):
        """boxes: [(object_name, [x1, y1, x2, y2]), ...] from the ground truth."""
        _sync(); t0 = time.perf_counter()
        rows = []
        if boxes:
            r = self.mobilesam(bgr, bboxes=[list(map(float, b)) for _, b in boxes], verbose=False,
                               device=0 if self.device.type == "cuda" else "cpu")[0]
            masks = r.masks.data.cpu().numpy().astype(bool) if r.masks is not None else np.zeros((0,) + bgr.shape[:2], bool)
            _sync(); t1 = time.perf_counter()
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            depth_m = depth_mm.astype(np.float32) / 1000.0
            pts_img = RB.get_point_cloud_from_depth(depth_m, np.asarray(K))
            for (name, _), m in zip(boxes, masks):
                rows += self._pem(name, [m], [1.0], rgb, depth_m, pts_img, np.asarray(K))
        else:
            t1 = time.perf_counter()
        _sync(); t2 = time.perf_counter()
        return rows, {"ism": round((t1 - t0) * 1000), "pem": round((t2 - t1) * 1000), "total": round((t2 - t0) * 1000)}, len(boxes), None

    def process(self, bgr, depth_mm, K, want_mask=False):
        if self.gt_mode:
            return self.process_gt(bgr, depth_mm, K, getattr(self, "gt_boxes", []))
        _sync(); t0 = time.perf_counter()
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        det, n_props = self._ism(rgb, depth_mm, np.asarray(K, np.float64))
        _sync(); t1 = time.perf_counter()
        rows = []
        if det is not None and len(det.scores):
            depth_m = depth_mm.astype(np.float32) / 1000.0
            pts_img = RB.get_point_cloud_from_depth(depth_m, np.asarray(K))
            keep = det.scores > self.det_score_thresh
            for oi in np.unique(det.object_ids[keep]):
                sel = keep & (det.object_ids == oi)
                rows += self._pem(self.objects[int(oi)], det.masks[sel], det.scores[sel], rgb, depth_m, pts_img, np.asarray(K))
        _sync(); t2 = time.perf_counter()
        ms = {"ism": round((t1 - t0) * 1000), "pem": round((t2 - t1) * 1000), "total": round((t2 - t0) * 1000)}
        return rows, ms, n_props, None
