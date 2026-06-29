"""
run_pipeline.py  —  ISM → PEM 단일 이미지 파이프라인
  - pipeline_config.yaml 을 읽어 실행 (별도 인자 불필요)
  - CLI 인자로 개별 값 덮어쓰기 가능
  - ISM / PEM 각 세부 단계별 소요 시간 출력
  - ISM mask score, PEM 초기/최종 pose score 출력

실행:
  python run_pipeline.py
  python run_pipeline.py --rgb_path D:/other/rgb.png --depth_path D:/other/depth.png --cam_path D:/other/camera.json
"""

import os
import sys
import json
import glob
import time
import argparse
import importlib
import random

import numpy as np
import torch

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
import cv2
import trimesh
import imageio
import yaml
from PIL import Image
import torchvision.transforms as transforms
import torchvision.transforms as T
import pycocotools.mask as cocomask
from skimage.feature import canny
from skimage.morphology import binary_dilation

# ── 경로 상수 ──────────────────────────────────────────────────────────────────
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
ISM_DIR  = os.path.join(ROOT_DIR, "Instance_Segmentation_Model")
PEM_DIR  = os.path.join(ROOT_DIR, "Pose_Estimation_Model")

sys.path.insert(0, ISM_DIR)
for _p in [
    os.path.join(PEM_DIR, "provider"),
    os.path.join(PEM_DIR, "utils"),
    os.path.join(PEM_DIR, "model"),
    os.path.join(PEM_DIR, "model", "pointnet2"),
]:
    if _p not in sys.path:
        sys.path.append(_p)

# ── ISM imports ────────────────────────────────────────────────────────────────
from hydra import initialize, compose
from hydra.utils import instantiate
from omegaconf import OmegaConf

from utils.poses.pose_utils import get_obj_poses_from_template_level, load_index_level_in_level2
from utils.bbox_utils import CropResizePad
from model.utils import Detections, convert_npz_to_json
from utils.inout import load_json, save_json_bop23
from segment_anything.utils.amg import rle_to_mask

# ── PEM imports ────────────────────────────────────────────────────────────────
import gorilla
from data_utils import load_im, get_bbox, get_point_cloud_from_depth, get_resize_rgb_choose
from draw_utils import draw_detections

_rgb_transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])


# ══════════════════════════════════════════════════════════════════════════════
# 유틸
# ══════════════════════════════════════════════════════════════════════════════

def _tick(label: str, t0: float) -> float:
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    elapsed = time.time() - t0
    print(f"    ├─ {label:<45} {elapsed:.3f}s")
    return time.time()


def _header(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def load_config(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ══════════════════════════════════════════════════════════════════════════════
# ISM
# ══════════════════════════════════════════════════════════════════════════════

def run_ism(cfg_user: dict, device: torch.device) -> str:
    """ISM 실행. 단계별 시간 + 마스크 score 출력. detection_ism.json 경로 반환."""
    _header("ISM (Instance Segmentation Model)")

    segmentor_model        = cfg_user["segmentor_model"]
    stability_score_thresh = cfg_user["stability_score_thresh"]
    rgb_path               = cfg_user["rgb_path"]
    depth_path             = cfg_user["depth_path"]
    cam_path               = cfg_user["cam_path"]
    cad_path               = cfg_user["cad_path"]
    template_dir           = cfg_user["template_dir"]
    output_dir             = cfg_user["output_dir"]

    os.makedirs(os.path.join(output_dir, "sam6d_results"), exist_ok=True)

    # ── 1. 모델 로드 ──────────────────────────────────────────────────────────
    t0 = t = time.time()
    from hydra.core.global_hydra import GlobalHydra

    GlobalHydra.instance().clear()
    with initialize(version_base=None, config_path="Instance_Segmentation_Model/configs"):
        cfg = compose(config_name="run_inference.yaml")

    if segmentor_model == "sam":
        GlobalHydra.instance().clear()
        with initialize(version_base=None, config_path="Instance_Segmentation_Model/configs/model"):
            cfg.model = compose(config_name="ISM_sam.yaml")
        cfg.model.segmentor_model.stability_score_thresh = stability_score_thresh
    elif segmentor_model == "fastsam":
        GlobalHydra.instance().clear()
        with initialize(version_base=None, config_path="Instance_Segmentation_Model/configs/model"):
            cfg.model = compose(config_name="ISM_fastsam.yaml")
    else:
        raise ValueError(f"지원하지 않는 segmentor_model: {segmentor_model}")

    # 체크포인트 절대경로 패치
    if segmentor_model == "sam":
        cfg.model.segmentor_model.sam.checkpoint_dir = os.path.join(ISM_DIR, "checkpoints", "segment-anything", "")
    else:
        cfg.model.segmentor_model.checkpoint_path = os.path.join(ISM_DIR, "checkpoints", "FastSAM", "FastSAM-x.pt")
    cfg.model.descriptor_model.checkpoint_dir = os.path.join(ISM_DIR, "checkpoints", "dinov2", "")

    model = instantiate(cfg.model)
    model.descriptor_model.model = model.descriptor_model.model.to(device)
    model.descriptor_model.model.device = device
    if hasattr(model.segmentor_model, "predictor"):
        model.segmentor_model.predictor.model = model.segmentor_model.predictor.model.to(device)
    else:
        model.segmentor_model.model.setup_model(device=device, verbose=True)
    t = _tick("모델 로드 (SAM/FastSAM + DINOv2)", t0)

    # ── 2. 템플릿 로드 & feature 추출 ────────────────────────────────────────
    num_templates = len(glob.glob(os.path.join(template_dir, "*.npy")))
    boxes, masks, templates = [], [], []
    for idx in range(num_templates):
        image = Image.open(os.path.join(template_dir, f"rgb_{idx}.png"))
        mask  = Image.open(os.path.join(template_dir, f"mask_{idx}.png"))
        boxes.append(mask.getbbox())
        image = torch.from_numpy(np.array(image.convert("RGB")) / 255).float()
        mask  = torch.from_numpy(np.array(mask.convert("L")) / 255).float()
        image = image * mask[:, :, None]
        templates.append(image)
        masks.append(mask.unsqueeze(-1))

    templates = torch.stack(templates).permute(0, 3, 1, 2)
    masks     = torch.stack(masks).permute(0, 3, 1, 2)
    boxes     = torch.tensor(np.array(boxes))

    proposal_processor = CropResizePad(224)
    templates_proc     = proposal_processor(images=templates, boxes=boxes).to(device)
    masks_cropped      = proposal_processor(images=masks, boxes=boxes).to(device)

    model.ref_data = {}
    model.ref_data["descriptors"] = model.descriptor_model.compute_features(
        templates_proc, token_name="x_norm_clstoken"
    ).unsqueeze(0).data
    model.ref_data["appe_descriptors"] = model.descriptor_model.compute_masked_patch_feature(
        templates_proc, masks_cropped[:, 0, :, :]
    ).unsqueeze(0).data
    t = _tick(f"템플릿 로드 & feature 추출 ({num_templates}개)", t)

    # ── 3. CAD 포인트클라우드 & pose ─────────────────────────────────────────
    mesh = trimesh.load_mesh(cad_path)
    model_points = mesh.sample(2048).astype(np.float32) / 1000.0
    model.ref_data["pointcloud"] = torch.tensor(model_points).unsqueeze(0).data.to(device)
    template_poses = get_obj_poses_from_template_level(level=2, pose_distribution="all")
    template_poses[:, :3, 3] *= 0.4
    poses = torch.tensor(template_poses).to(torch.float32).to(device)
    model.ref_data["poses"] = poses[load_index_level_in_level2(0, "all"), :, :]
    t = _tick("CAD 메시 샘플링 & 포즈 로드", t)

    # ── 4. 마스크 생성 (SAM/FastSAM) ─────────────────────────────────────────
    rgb = Image.open(rgb_path).convert("RGB")
    detections_raw = model.segmentor_model.generate_masks(np.array(rgb))
    detections     = Detections(detections_raw)
    n_proposals    = len(detections)
    t = _tick(f"마스크 생성 (proposals={n_proposals})", t)

    # ── 5. DINOv2 feature 추출 ───────────────────────────────────────────────
    query_desc, query_appe_desc = model.descriptor_model.forward(np.array(rgb), detections)
    t = _tick("DINOv2 feature 추출", t)

    # ── 6. Semantic score 계산 ───────────────────────────────────────────────
    idx_selected, pred_idx_objects, semantic_score, best_template = model.compute_semantic_score(query_desc)
    detections.filter(idx_selected)
    query_appe_desc = query_appe_desc[idx_selected, :]
    t = _tick(f"Semantic score 계산 (통과={len(idx_selected)})", t)

    # ── 7. Appearance score 계산 ─────────────────────────────────────────────
    appe_scores, ref_aux_descriptor = model.compute_appearance_score(best_template, pred_idx_objects, query_appe_desc)
    t = _tick("Appearance score 계산", t)

    # ── 8. Geometric score 계산 ──────────────────────────────────────────────
    cam_info    = load_json(cam_path)
    depth_raw   = np.array(imageio.imread(depth_path)).astype(np.int32)
    cam_K       = np.array(cam_info["cam_K"]).reshape((3, 3))
    depth_scale = np.array(cam_info["depth_scale"])
    batch = {
        "depth":         torch.from_numpy(depth_raw).unsqueeze(0).to(device),
        "cam_intrinsic": torch.from_numpy(cam_K).unsqueeze(0).to(device),
        "depth_scale":   torch.from_numpy(depth_scale).unsqueeze(0).to(device),
    }
    image_uv = model.project_template_to_image(best_template, pred_idx_objects, batch, detections.masks)
    geometric_score, visible_ratio = model.compute_geometric_score(
        image_uv, detections, query_appe_desc, ref_aux_descriptor,
        visible_thred=model.visible_thred,
    )
    t = _tick("Geometric score 계산", t)

    # ── 9. 최종 score 계산 & 저장 ────────────────────────────────────────────
    final_score = (semantic_score + appe_scores + geometric_score * visible_ratio) / (1 + 1 + visible_ratio)
    detections.add_attribute("scores",     final_score)
    detections.add_attribute("object_ids", torch.zeros_like(final_score))
    detections.to_numpy()

    save_path = os.path.join(output_dir, "sam6d_results", "detection_ism")
    detections.save_to_file(0, 0, 0, save_path, "Custom", return_results=False)
    det_list = convert_npz_to_json(idx=0, list_npz_paths=[save_path + ".npz"])
    save_json_bop23(save_path + ".json", det_list)
    t = _tick("최종 score 계산 & JSON 저장", t)

    total_ism = time.time() - t0
    print(f"    └─ {'ISM 전체':<45} {total_ism:.3f}s")

    # ── ISM 정확도 리포트 ─────────────────────────────────────────────────────
    print(f"\n  [ISM 마스크 Score 리포트]  (총 {len(det_list)}개 detection)")
    print(f"  {'#':>3}  {'Final':>7}  {'Semantic':>9}  {'Appearance':>11}  {'Geometric':>10}  {'VisRatio':>9}")
    print(f"  {'-'*3}  {'-'*7}  {'-'*9}  {'-'*11}  {'-'*10}  {'-'*9}")
    sem_np  = semantic_score.detach().cpu().numpy()
    appe_np = appe_scores.detach().cpu().numpy()
    geo_np  = geometric_score.detach().cpu().numpy()
    vis_np  = visible_ratio.detach().cpu().numpy()
    fin_np  = final_score.detach().cpu().numpy()
    for i in range(len(fin_np)):
        marker = " ◀ best" if i == int(np.argmax(fin_np)) else ""
        print(f"  {i:>3}  {fin_np[i]:>7.4f}  {sem_np[i]:>9.4f}  {appe_np[i]:>11.4f}  {geo_np[i]:>10.4f}  {vis_np[i]:>9.4f}{marker}")

    return save_path + ".json"


# ══════════════════════════════════════════════════════════════════════════════
# PEM
# ══════════════════════════════════════════════════════════════════════════════

def run_pem(cfg_user: dict, seg_path: str):
    """PEM 실행. 단계별 시간 + 초기/최종 pose score 출력."""
    _header("PEM (Pose Estimation Model)")

    rgb_path        = cfg_user["rgb_path"]
    depth_path      = cfg_user["depth_path"]
    cam_path        = cfg_user["cam_path"]
    cad_path        = cfg_user["cad_path"]
    template_dir    = cfg_user["template_dir"]
    output_dir      = cfg_user["output_dir"]
    det_score_thresh = cfg_user["det_score_thresh"]

    t0 = t = time.time()

    # ── 1. 모델 로드 ──────────────────────────────────────────────────────────
    pem_cfg = gorilla.Config.fromfile(os.path.join(PEM_DIR, "config", "base.yaml"))
    pem_cfg.model_name = "pose_estimation_model"
    pem_cfg.gpus       = "0"

    MODEL = importlib.import_module(pem_cfg.model_name)
    pem_model = MODEL.Net(pem_cfg.model)
    pem_model = pem_model.to(DEVICE)
    pem_model.eval()

    checkpoint = os.path.join(PEM_DIR, "checkpoints", "sam-6d-pem-base.pth")
    gorilla.solver.load_checkpoint(model=pem_model, filename=checkpoint)
    t = _tick("모델 로드 (PEM)", t)

    # ── 2. 템플릿 feature 추출 ───────────────────────────────────────────────
    total_nView      = 42
    n_template_view  = pem_cfg.test_dataset.n_template_view
    all_tem, all_tem_choose, all_tem_pts = [], [], []
    for v in range(n_template_view):
        i = int(total_nView / n_template_view * v)
        rgb_t  = load_im(os.path.join(template_dir, f"rgb_{i}.png")).astype(np.uint8)
        xyz_t  = np.load(os.path.join(template_dir, f"xyz_{i}.npy")).astype(np.float32) / 1000.0
        mask_t = load_im(os.path.join(template_dir, f"mask_{i}.png")).astype(np.uint8) == 255
        bbox   = get_bbox(mask_t)
        y1, y2, x1, x2 = bbox
        mask_t = mask_t[y1:y2, x1:x2]
        rgb_t  = rgb_t[:, :, ::-1][y1:y2, x1:x2, :]
        if pem_cfg.test_dataset.rgb_mask_flag:
            rgb_t = rgb_t * (mask_t[:, :, None] > 0).astype(np.uint8)
        rgb_t = cv2.resize(rgb_t, (pem_cfg.test_dataset.img_size, pem_cfg.test_dataset.img_size),
                           interpolation=cv2.INTER_LINEAR)
        rgb_t = _rgb_transform(np.array(rgb_t))
        choose = (mask_t > 0).astype(np.float32).flatten().nonzero()[0]
        n = pem_cfg.test_dataset.n_sample_template_point
        choose_idx = np.random.choice(len(choose), n) if len(choose) <= n \
                     else np.random.choice(len(choose), n, replace=False)
        choose = choose[choose_idx]
        xyz_t  = xyz_t[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]
        rgb_choose = get_resize_rgb_choose(choose, [y1, y2, x1, x2], pem_cfg.test_dataset.img_size)
        all_tem.append(torch.FloatTensor(rgb_t).unsqueeze(0).to(DEVICE))
        all_tem_choose.append(torch.IntTensor(rgb_choose).long().unsqueeze(0).to(DEVICE))
        all_tem_pts.append(torch.FloatTensor(xyz_t).unsqueeze(0).to(DEVICE))

    with torch.inference_mode():
        all_tem_pts, all_tem_feat = pem_model.feature_extraction.get_obj_feats(
            all_tem, all_tem_pts, all_tem_choose
        )
    t = _tick(f"템플릿 feature 추출 ({n_template_view}개 뷰)", t)

    # ── 3. 입력 데이터 로드 ──────────────────────────────────────────────────
    with open(seg_path) as f:
        dets_all = json.load(f)
    dets = [d for d in dets_all if d["score"] > det_score_thresh]
    if not dets:
        print(f"  [경고] det_score_thresh={det_score_thresh} 이상인 detection 없음. PEM 건너뜀.")
        return

    cam_info    = json.load(open(cam_path))
    K           = np.array(cam_info["cam_K"]).reshape(3, 3)
    depth_scale = cam_info["depth_scale"]

    whole_image = load_im(rgb_path).astype(np.uint8)
    if whole_image.ndim == 2:
        whole_image = np.stack([whole_image] * 3, axis=2)

    whole_depth = load_im(depth_path).astype(np.float32) * depth_scale / 1000.0
    whole_pts   = get_point_cloud_from_depth(whole_depth, K)

    mesh         = trimesh.load_mesh(cad_path)
    model_points = mesh.sample(pem_cfg.test_dataset.n_sample_model_point).astype(np.float32) / 1000.0
    radius       = float(np.max(np.linalg.norm(model_points, axis=1)))

    all_rgb, all_cloud, all_rgb_choose, all_score, valid_dets = [], [], [], [], []
    for inst in dets:
        seg   = inst["segmentation"]
        score = inst["score"]
        h, w  = seg["size"]
        try:
            rle = cocomask.frPyObjects(seg, h, w)
        except Exception:
            rle = seg
        mask = cocomask.decode(rle)
        mask = np.logical_and(mask > 0, whole_depth > 0)
        if np.sum(mask) <= 32:
            continue
        bbox = get_bbox(mask)
        y1, y2, x1, x2 = bbox
        mask   = mask[y1:y2, x1:x2]
        choose = mask.astype(np.float32).flatten().nonzero()[0]
        cloud  = whole_pts.copy()[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]
        center = np.mean(cloud, axis=0)
        flag   = np.linalg.norm(cloud - center[None, :], axis=1) < radius * 1.2
        if np.sum(flag) < 4:
            continue
        choose = choose[flag]
        cloud  = cloud[flag]
        n = pem_cfg.test_dataset.n_sample_observed_point
        choose_idx = np.random.choice(len(choose), n) if len(choose) <= n \
                     else np.random.choice(len(choose), n, replace=False)
        choose = choose[choose_idx]
        cloud  = cloud[choose_idx]
        rgb = whole_image.copy()[y1:y2, x1:x2, :][:, :, ::-1]
        if pem_cfg.test_dataset.rgb_mask_flag:
            rgb = rgb * (mask[:, :, None] > 0).astype(np.uint8)
        rgb = cv2.resize(rgb, (pem_cfg.test_dataset.img_size, pem_cfg.test_dataset.img_size),
                         interpolation=cv2.INTER_LINEAR)
        rgb = _rgb_transform(np.array(rgb))
        rgb_choose = get_resize_rgb_choose(choose, [y1, y2, x1, x2], pem_cfg.test_dataset.img_size)
        all_rgb.append(torch.FloatTensor(rgb))
        all_cloud.append(torch.FloatTensor(cloud))
        all_rgb_choose.append(torch.IntTensor(rgb_choose).long())
        all_score.append(score)
        valid_dets.append(inst)

    if not all_rgb:
        print("  [경고] 유효한 포인트클라우드 instance 없음. PEM 건너뜀.")
        return

    input_data = {
        "pts":       torch.stack(all_cloud).to(DEVICE),
        "rgb":       torch.stack(all_rgb).to(DEVICE),
        "rgb_choose":torch.stack(all_rgb_choose).to(DEVICE),
        "score":     torch.FloatTensor(all_score).to(DEVICE),
    }
    ninstance = input_data["pts"].size(0)
    input_data["model"] = torch.FloatTensor(model_points).unsqueeze(0).repeat(ninstance, 1, 1).to(DEVICE)
    input_data["K"]     = torch.FloatTensor(K).unsqueeze(0).repeat(ninstance, 1, 1).to(DEVICE)
    t = _tick(f"입력 데이터 로드 ({ninstance}개 instance)", t)

    # ── 4. PEM forward (초기 포즈 추정) ─────────────────────────────────────
    with torch.inference_mode():
        input_data["dense_po"] = all_tem_pts.repeat(ninstance, 1, 1)
        input_data["dense_fo"] = all_tem_feat.repeat(ninstance, 1, 1)
        out = pem_model(input_data)
    t = _tick("PEM forward (초기→최종 포즈 추정)", t)

    # ── 5. 결과 파싱 ─────────────────────────────────────────────────────────
    # ISM score (입력): detection 단계의 confidence
    ism_scores   = np.array(all_score)

    # PEM 자체 pose score
    if "pred_pose_score" in out:
        pem_pose_scores = out["pred_pose_score"].detach().cpu().numpy()   # 초기 포즈 신뢰도
    else:
        pem_pose_scores = None

    # 최종 score = pose_score * ism_score (있으면)
    if pem_pose_scores is not None:
        final_pose_scores = (out["pred_pose_score"] * out["score"]).detach().cpu().numpy()
    else:
        final_pose_scores = out["score"].detach().cpu().numpy()

    pred_rot   = out["pred_R"].detach().cpu().numpy()
    pred_trans = out["pred_t"].detach().cpu().numpy() * 1000  # m → mm

    t = _tick("결과 파싱", t)

    # ── 6. 저장 ──────────────────────────────────────────────────────────────
    os.makedirs(os.path.join(output_dir, "sam6d_results"), exist_ok=True)
    for idx, det in enumerate(valid_dets):
        valid_dets[idx]["score"] = float(final_pose_scores[idx])
        valid_dets[idx]["R"]     = pred_rot[idx].tolist()
        valid_dets[idx]["t"]     = pred_trans[idx].tolist()

    pem_json = os.path.join(output_dir, "sam6d_results", "detection_pem.json")
    with open(pem_json, "w") as f:
        json.dump(valid_dets, f)

    save_path = os.path.join(output_dir, "sam6d_results", "vis_pem.png")
    best_idx  = int(np.argmax(final_pose_scores))
    from draw_utils import draw_detections
    img_vis = draw_detections(whole_image, pred_rot[[best_idx]], pred_trans[[best_idx]],
                              model_points * 1000,
                              input_data["K"].detach().cpu().numpy()[[best_idx]],
                              color=(255, 0, 0))
    Image.fromarray(np.uint8(img_vis)).save(save_path)
    t = _tick("결과 저장 + 시각화", t)

    total_pem = time.time() - t0
    print(f"    └─ {'PEM 전체':<45} {total_pem:.3f}s")

    # ── PEM 정확도 리포트 ─────────────────────────────────────────────────────
    print(f"\n  [PEM Pose Score 리포트]  (총 {ninstance}개 instance)")
    print(f"  {'#':>3}  {'ISM Score':>10}  {'PEM Pose':>10}  {'Final Score':>12}  Translation(mm)")
    print(f"  {'-'*3}  {'-'*10}  {'-'*10}  {'-'*12}  {'-'*30}")
    for i in range(ninstance):
        ps  = f"{pem_pose_scores[i]:.4f}" if pem_pose_scores is not None else "  N/A  "
        tx, ty, tz = pred_trans[i]
        marker = " ◀ best" if i == best_idx else ""
        print(f"  {i:>3}  {ism_scores[i]:>10.4f}  {ps:>10}  {final_pose_scores[i]:>12.4f}  "
              f"[{tx:7.1f}, {ty:7.1f}, {tz:7.1f}]{marker}")

    print(f"\n  저장: {pem_json}")
    print(f"  시각화: {save_path}")


# ══════════════════════════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════════════════════════

def parse_args():
    cfg_file = os.path.join(ROOT_DIR, "pipeline_config.yaml")
    base     = load_config(cfg_file)

    p = argparse.ArgumentParser(description="SAM-6D ISM→PEM 파이프라인")
    p.add_argument("--config",         default=cfg_file,         help="설정 파일 경로")
    p.add_argument("--rgb_path",       default=base["rgb_path"])
    p.add_argument("--depth_path",     default=base["depth_path"])
    p.add_argument("--cam_path",       default=base["cam_path"])
    p.add_argument("--cad_path",       default=base["cad_path"])
    p.add_argument("--template_dir",   default=base["template_dir"])
    p.add_argument("--output_dir",     default=base["output_dir"])
    p.add_argument("--segmentor_model",        default=base["segmentor_model"])
    p.add_argument("--stability_score_thresh", default=base["stability_score_thresh"], type=float)
    p.add_argument("--det_score_thresh",       default=base["det_score_thresh"],       type=float)
    args = p.parse_args()

    return {
        "rgb_path":               os.path.abspath(args.rgb_path),
        "depth_path":             os.path.abspath(args.depth_path),
        "cam_path":               os.path.abspath(args.cam_path),
        "cad_path":               os.path.abspath(args.cad_path),
        "template_dir":           os.path.abspath(args.template_dir),
        "output_dir":             os.path.abspath(args.output_dir),
        "segmentor_model":        args.segmentor_model,
        "stability_score_thresh": args.stability_score_thresh,
        "det_score_thresh":       args.det_score_thresh,
    }


def main():
    cfg = parse_args()

    print("\n[파이프라인 설정]")
    for k, v in cfg.items():
        print(f"  {k:<28} = {v}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"\n  device = {device}")

    total_start = time.time()

    seg_json = run_ism(cfg, device)
    run_pem(cfg, seg_json)

    print(f"\n{'='*60}")
    print(f"  전체 파이프라인 소요: {time.time() - total_start:.2f}s")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
