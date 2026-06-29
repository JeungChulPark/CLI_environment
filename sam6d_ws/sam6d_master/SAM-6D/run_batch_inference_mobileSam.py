"""
run_batch_inference_mobileSam.py

Mobile-SAM v1 (TinyViT + AutomaticMaskGenerator) 기반
ISM 배치 추론 + 속도/정확도 성능 측정 스크립트.

파이프라인:
  Mobile-SAM AutoMaskGenerator (class-agnostic) → 모든 region mask
  → DINOv2 feature 추출 → 템플릿 매칭 → ISM 탐지 결과

속도 벤치마크 (mask_generation, SAM-6D 640×480 이미지 기준):
  points_per_side=32 : ~1.8s  (213 masks)
  points_per_side=16 : ~0.47s (124 masks) ← 기본값
  points_per_side=8  : ~0.14s (54  masks)

사전 준비:
  pip install git+https://github.com/ChaoningZhang/MobileSAM.git
  # Mobile-SAM 가중치: Instance_Segmentation_Model/checkpoints/mobile_sam/mobile_sam.pt

사용법:
  python run_batch_inference_mobileSam.py \\
      --rgb_dir       /path/to/rgb \\
      --depth_dir     /path/to/depth \\
      --cam_path      /path/to/camera.json \\
      --cad_path      /path/to/obj.ply \\
      --template_dir  /path/to/templates \\
      --output_dir    /path/to/outputs

  또는 pipeline_config.yaml 에 경로를 설정하고 인자 없이 실행:
  python run_batch_inference_mobileSam.py
"""

import os
import sys
import json
import glob
import time
import argparse

import numpy as np
import torch
import cv2
import trimesh
import imageio
import distinctipy
from PIL import Image
from skimage.feature import canny
from skimage.morphology import binary_dilation
import torchvision.transforms as T
import torchvision.transforms as transforms
import pycocotools.mask as cocomask

# ── 경로 상수 ──────────────────────────────────────────────────────────────────
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
ISM_DIR  = os.path.join(ROOT_DIR, "Instance_Segmentation_Model")
MOBILE_SAM_CKPT = os.path.join(ISM_DIR, "checkpoints", "mobile_sam", "mobile_sam.pt")

# ISM 패키지 (model/, utils/, segment_anything/ 등)
sys.path.insert(0, ISM_DIR)

# ── Mobile-SAM (github.com/ChaoningZhang/MobileSAM) ──────────────────────────
from mobile_sam import sam_model_registry as mobile_sam_registry
from mobile_sam import SamAutomaticMaskGenerator as MobileSamGenerator

# ── ISM 내장 segment_anything ────────────────────────────────────────────────
from segment_anything.utils.amg import rle_to_mask

# ── ISM imports ────────────────────────────────────────────────────────────────
from hydra import initialize, compose
from hydra.utils import instantiate
from omegaconf import OmegaConf

from utils.poses.pose_utils import (
    get_obj_poses_from_template_level,
    load_index_level_in_level2,
)
from utils.bbox_utils import CropResizePad, xyxy_to_xywh, force_binary_mask
from model.utils import Detections, mask_to_rle
from utils.inout import load_json, save_json_bop23

# ── RGB transform ─────────────────────────────────────────────────────────────
_rgb_transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])


# ══════════════════════════════════════════════════════════════════════════════
# Mobile-SAM 세그멘터 래퍼
# ══════════════════════════════════════════════════════════════════════════════

class MobileSamSegmentor:
    """
    Mobile-SAM v1 (TinyViT + SamAutomaticMaskGenerator) 래퍼.
    generate_masks(image_np) → {"masks": Tensor(N,H,W), "boxes": Tensor(N,4)}

    points_per_side=16 → 256 grid prompts → 2 decoder passes → ~0.47s
    points_per_side=32 → 1024 grid prompts → 16 decoder passes → ~1.8s
    """

    def __init__(self, checkpoint_path: str, device: torch.device,
                 points_per_side: int = 16,
                 points_per_batch: int = 128,
                 pred_iou_thresh: float = 0.88,
                 stability_score_thresh: float = 0.85):

        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(
                f"Mobile-SAM 가중치 파일이 없습니다: {checkpoint_path}"
            )
        sam = mobile_sam_registry["vit_t"](checkpoint=checkpoint_path)
        sam.to(device=device)
        sam.eval()
        self.mask_generator = MobileSamGenerator(
            sam,
            points_per_side=points_per_side,
            points_per_batch=points_per_batch,
            pred_iou_thresh=pred_iou_thresh,
            stability_score_thresh=stability_score_thresh,
        )
        self.device = device

    @torch.inference_mode()
    def generate_masks(self, image: np.ndarray) -> dict:
        """
        image: (H, W, 3) uint8 numpy array (RGB)
        반환: {"masks": Tensor(N,H,W) float, "boxes": Tensor(N,4) xyxy}
        """
        orig_h, orig_w = image.shape[:2]
        raw = self.mask_generator.generate(image)
        if not raw:
            return {
                "masks": torch.zeros((0, orig_h, orig_w), dtype=torch.float32),
                "boxes": torch.zeros((0, 4), dtype=torch.float32),
            }
        masks = torch.from_numpy(
            np.stack([r["segmentation"] for r in raw]).astype(np.float32)
        )
        boxes_xywh = np.array([r["bbox"] for r in raw], dtype=np.float32)
        boxes_xyxy = boxes_xywh.copy()
        boxes_xyxy[:, 2] += boxes_xyxy[:, 0]
        boxes_xyxy[:, 3] += boxes_xyxy[:, 1]
        return {
            "masks": masks.to(self.device),
            "boxes": torch.from_numpy(boxes_xyxy).to(self.device),
        }


# ══════════════════════════════════════════════════════════════════════════════
# ISM DINOv2 + 스코어링 파이프라인 로딩
# ══════════════════════════════════════════════════════════════════════════════

def load_ism_descriptor_model(device: torch.device, dinov2_chunk_size: int = 32):
    """DINOv2 descriptor 모델만 로드 (segmentor 제외)."""
    from hydra.core.global_hydra import GlobalHydra

    GlobalHydra.instance().clear()
    with initialize(version_base=None,
                    config_path="Instance_Segmentation_Model/configs"):
        cfg = compose(config_name="run_inference.yaml")

    # SAM config 기반으로 descriptor 부분만 가져옴
    GlobalHydra.instance().clear()
    with initialize(version_base=None,
                    config_path="Instance_Segmentation_Model/configs/model"):
        cfg.model = compose(config_name="ISM_sam.yaml")

    ckpt_dino = os.path.join(ISM_DIR, "checkpoints", "dinov2", "")
    cfg.model.descriptor_model.checkpoint_dir = ckpt_dino

    # segmentor는 더미로 대체 — descriptor 모델과 스코어링 로직만 사용
    ckpt_sam = os.path.join(ISM_DIR, "checkpoints", "segment-anything", "")
    cfg.model.segmentor_model.sam.checkpoint_dir = ckpt_sam

    model = instantiate(cfg.model)
    model.descriptor_model.chunk_size = dinov2_chunk_size
    model.descriptor_model.model = model.descriptor_model.model.to(device)
    model.descriptor_model.model.device = device

    return model, cfg


def load_ism_model_with_mobile_sam(device: torch.device,
                                   points_per_side: int = 16,
                                   points_per_batch: int = 128,
                                   stability_score_thresh: float = 0.85,
                                   dinov2_chunk_size: int = 32) -> object:
    """Mobile-SAM v1 AutoMaskGenerator + DINOv2 descriptor 조합으로 ISM 모델 구성."""
    print("[LOAD] DINOv2 descriptor 모델 로딩 중...")
    model, _ = load_ism_descriptor_model(device, dinov2_chunk_size)

    print(f"[LOAD] Mobile-SAM vit_t (points_per_side={points_per_side}) 로딩 중...")
    mobile_sam_seg = MobileSamSegmentor(
        checkpoint_path=MOBILE_SAM_CKPT,
        device=device,
        points_per_side=points_per_side,
        points_per_batch=points_per_batch,
        stability_score_thresh=stability_score_thresh,
    )

    model.segmentor_model = mobile_sam_seg

    return model


def load_ism_templates(model, template_dir: str, cad_path: str, device: torch.device):
    """템플릿 feature + pointcloud 1회 추출 → model.ref_data 저장."""
    num_templates = len(glob.glob(os.path.join(template_dir, "*.npy")))
    boxes, masks, templates = [], [], []
    for idx in range(num_templates):
        image = Image.open(os.path.join(template_dir, f"rgb_{idx}.png"))
        mask  = Image.open(os.path.join(template_dir, f"mask_{idx}.png"))
        boxes.append(mask.getbbox())
        image = torch.from_numpy(np.array(image.convert("RGB")) / 255).float()
        mask  = torch.from_numpy(np.array(mask.convert("L"))   / 255).float()
        image = image * mask[:, :, None]
        templates.append(image)
        masks.append(mask.unsqueeze(-1))

    templates = torch.stack(templates).permute(0, 3, 1, 2)
    masks     = torch.stack(masks).permute(0, 3, 1, 2)
    boxes     = torch.tensor(np.array(boxes))

    proposal_processor = CropResizePad(224)
    templates     = proposal_processor(images=templates, boxes=boxes).to(device)
    masks_cropped = proposal_processor(images=masks,     boxes=boxes).to(device)

    model.ref_data = {}
    model.ref_data["descriptors"] = model.descriptor_model.compute_features(
        templates, token_name="x_norm_clstoken"
    ).unsqueeze(0).data
    model.ref_data["appe_descriptors"] = model.descriptor_model.compute_masked_patch_feature(
        templates, masks_cropped[:, 0, :, :]
    ).unsqueeze(0).data

    template_poses = get_obj_poses_from_template_level(level=2, pose_distribution="all")
    template_poses[:, :3, 3] *= 0.4
    poses = torch.tensor(template_poses).to(torch.float32).to(device)
    model.ref_data["poses"] = poses[load_index_level_in_level2(0, "all"), :, :]

    mesh = trimesh.load_mesh(cad_path)
    model_points = mesh.sample(2048).astype(np.float32) / 1000.0
    model.ref_data["pointcloud"] = (
        torch.tensor(model_points).unsqueeze(0).data.to(device)
    )

    return model


# ══════════════════════════════════════════════════════════════════════════════
# 단일 이미지 ISM 추론
# ══════════════════════════════════════════════════════════════════════════════

def _batch_input_data(depth_path: str, cam_path: str, device: torch.device):
    cam_info    = load_json(cam_path)
    depth       = np.array(imageio.imread(depth_path)).astype(np.int32)
    cam_K       = np.array(cam_info["cam_K"]).reshape((3, 3))
    depth_scale = np.array(cam_info["depth_scale"])
    return {
        "depth":         torch.from_numpy(depth).unsqueeze(0).to(device),
        "cam_intrinsic": torch.from_numpy(cam_K).unsqueeze(0).to(device),
        "depth_scale":   torch.from_numpy(depth_scale).unsqueeze(0).to(device),
    }


def detections_to_json_direct(detections, top_k: int = 0) -> list:
    masks      = detections.masks
    boxes      = detections.boxes
    scores     = detections.scores
    obj_ids    = detections.object_ids
    boxes_xywh = xyxy_to_xywh(boxes)

    results = [
        {
            "scene_id":     0,
            "image_id":     0,
            "category_id":  int(obj_ids[i]) + 1,
            "bbox":         boxes_xywh[i].tolist(),
            "score":        float(scores[i]),
            "time":         0.0,
            "segmentation": mask_to_rle(force_binary_mask(masks[i])),
        }
        for i in range(len(scores))
    ]

    if top_k > 0 and len(results) > top_k:
        results = sorted(results, key=lambda d: d["score"], reverse=True)[:top_k]

    return results


def _visualize_ism(rgb: Image.Image, detections: list, save_path: str) -> Image.Image:
    img    = np.array(rgb.copy())
    gray   = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img    = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    colors = distinctipy.get_colors(max(len(detections), 1))
    alpha  = 0.33

    best_det = max(detections, key=lambda d: d["score"])
    mask = rle_to_mask(best_det["segmentation"])
    edge = canny(mask)
    edge = binary_dilation(edge, np.ones((2, 2)))

    obj_id = best_det["category_id"]
    r = int(255 * colors[obj_id - 1][0])
    g = int(255 * colors[obj_id - 1][1])
    b = int(255 * colors[obj_id - 1][2])
    img[mask, 0] = alpha * r + (1 - alpha) * img[mask, 0]
    img[mask, 1] = alpha * g + (1 - alpha) * img[mask, 1]
    img[mask, 2] = alpha * b + (1 - alpha) * img[mask, 2]
    img[edge, :] = 255

    result = Image.fromarray(np.uint8(img))
    concat = Image.new("RGB", (img.shape[1] * 2, img.shape[0]))
    concat.paste(rgb,    (0,            0))
    concat.paste(result, (img.shape[1], 0))
    concat.save(save_path)
    return concat


def run_ism_single(model, rgb_path: str, depth_path: str, cam_path: str,
                   img_output_dir: str, device: torch.device,
                   top_k_for_output: int = 5,
                   max_proposals: int = 0,
                   no_vis: bool = False,
                   vis_dir: str = None) -> dict:
    """
    Mobile-SAM ISM 단일 이미지 추론.
    반환:
      ok       : bool
      timing   : 단계별 경과시간(초) dict
      scores   : 상위 K개 탐지의 score dict 목록
      n_masks  : 생성된 mask 수 (segmentor 출력 개수)
      error    : 실패 시 에러 메시지
    """
    timing = {}

    def _t(label, t0):
        elapsed = time.time() - t0
        timing[label] = round(elapsed, 4)
        return time.time()

    try:
        t = time.time()
        rgb = Image.open(rgb_path).convert("RGB")
        t = _t("image_load", t)

        # ── Mobile-SAM mask 생성 ────────────────────────────────────────────
        detections = model.segmentor_model.generate_masks(np.array(rgb))
        n_proposals = (
            len(detections["masks"])
            if isinstance(detections, dict) else len(detections)
        )
        detections = Detections(detections)
        t = _t("mask_generation", t)

        # max_proposals 필터 (DINOv2 부하 제한)
        if max_proposals > 0 and len(detections) > max_proposals:
            areas = (
                (detections.boxes[:, 2] - detections.boxes[:, 0]) *
                (detections.boxes[:, 3] - detections.boxes[:, 1])
            )
            _, top_idx = torch.topk(areas, max_proposals)
            detections.filter(top_idx)

        n_after_filter = len(detections)
        timing["proposals"] = f"{n_proposals}→{n_after_filter}"

        if n_after_filter == 0:
            return {"ok": False, "timing": timing, "scores": [],
                    "n_masks": n_proposals, "error": "no proposals after filter"}

        # ── DINOv2 feature 추출 ─────────────────────────────────────────────
        query_desc, query_appe_desc = model.descriptor_model.forward(
            np.array(rgb), detections
        )
        if device.type == "cuda":
            torch.cuda.synchronize()  # GPU 연산 완료 후 정확한 시간 측정
        t = _t("dinov2_feature", t)

        # ── Semantic score ──────────────────────────────────────────────────
        (idx_selected, pred_idx_objects, semantic_score, best_template) = \
            model.compute_semantic_score(query_desc)
        if device.type == "cuda":
            torch.cuda.synchronize()
        t = _t("semantic_score", t)

        detections.filter(idx_selected)
        query_appe_desc = query_appe_desc[idx_selected, :]

        # ── Appearance score ────────────────────────────────────────────────
        appe_scores, ref_aux_desc = model.compute_appearance_score(
            best_template, pred_idx_objects, query_appe_desc
        )
        t = _t("appearance_score", t)

        # ── Geometric score ─────────────────────────────────────────────────
        batch    = _batch_input_data(depth_path, cam_path, device)
        image_uv = model.project_template_to_image(
            best_template, pred_idx_objects, batch, detections.masks
        )
        geometric_score, visible_ratio = model.compute_geometric_score(
            image_uv, detections, query_appe_desc, ref_aux_desc,
            visible_thred=model.visible_thred
        )
        t = _t("geometric_score", t)

        # ── 최종 score 계산 ──────────────────────────────────────────────────
        final_score = (
            (semantic_score + appe_scores + geometric_score * visible_ratio)
            / (1 + 1 + visible_ratio)
        )

        detections.add_attribute("scores",     final_score)
        detections.add_attribute("object_ids", torch.zeros_like(final_score))
        detections.to_numpy()

        det_list = detections_to_json_direct(detections, top_k=top_k_for_output)
        save_json_bop23(os.path.join(img_output_dir, "detection_ism.json"), det_list)
        t = _t("save_json", t)

        # ── 시각화 ────────────────────────────────────────────────────────────
        if not no_vis and det_list:
            stem     = os.path.splitext(os.path.basename(rgb_path))[0]
            vis_path = (os.path.join(vis_dir, f"{stem}.png") if vis_dir
                        else os.path.join(img_output_dir, "vis_ism.png"))
            _visualize_ism(rgb, det_list, vis_path)
        _t("visualization", t)

        scores = [{"final": round(float(d["score"]), 4)} for d in det_list]
        return {"ok": True, "timing": timing, "scores": scores,
                "n_masks": n_proposals}

    except Exception as e:
        import traceback
        traceback.print_exc()
        return {"ok": False, "timing": timing, "scores": [],
                "n_masks": 0, "error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# 성능 요약 통계
# ══════════════════════════════════════════════════════════════════════════════

def compute_performance_summary(results: list, total_elapsed: float) -> dict:
    """
    results: run_ism_single() 반환 dict 목록 (성공 항목만)
    반환: 속도·정확도 통계 dict
    """
    if not results:
        return {}

    # ── 속도 통계 ──────────────────────────────────────────────────────────────
    stage_keys = [
        "image_load", "mask_generation", "dinov2_feature",
        "semantic_score", "appearance_score", "geometric_score",
        "save_json", "visualization",
    ]
    speed = {}
    for key in stage_keys:
        vals = [r["timing"][key] for r in results if key in r["timing"]]
        if vals:
            speed[key] = {
                "mean_s":   round(float(np.mean(vals)),   4),
                "median_s": round(float(np.median(vals)), 4),
                "min_s":    round(float(np.min(vals)),    4),
                "max_s":    round(float(np.max(vals)),    4),
            }

    total_per_img = [
        sum(r["timing"][k] for k in stage_keys if k in r["timing"])
        for r in results
    ]
    speed["total_per_image"] = {
        "mean_s":   round(float(np.mean(total_per_img)),   4),
        "median_s": round(float(np.median(total_per_img)), 4),
        "min_s":    round(float(np.min(total_per_img)),    4),
        "max_s":    round(float(np.max(total_per_img)),    4),
        "fps":      round(len(results) / sum(total_per_img), 3),
    }

    n_masks = [r["n_masks"] for r in results]
    speed["proposals_per_image"] = {
        "mean":   round(float(np.mean(n_masks)),   1),
        "median": float(np.median(n_masks)),
        "min":    int(np.min(n_masks)),
        "max":    int(np.max(n_masks)),
    }

    # ── 정확도(score) 통계 ─────────────────────────────────────────────────────
    all_top1   = [r["scores"][0]["final"] for r in results if r["scores"]]
    all_scores = [s["final"] for r in results for s in r["scores"]]

    accuracy = {}
    if all_top1:
        accuracy["top1_score"] = {
            "mean":   round(float(np.mean(all_top1)),   4),
            "median": round(float(np.median(all_top1)), 4),
            "std":    round(float(np.std(all_top1)),    4),
            "min":    round(float(np.min(all_top1)),    4),
            "max":    round(float(np.max(all_top1)),    4),
        }
    if all_scores:
        accuracy["all_scores"] = {
            "count":  len(all_scores),
            "mean":   round(float(np.mean(all_scores)),   4),
            "median": round(float(np.median(all_scores)), 4),
            "std":    round(float(np.std(all_scores)),    4),
        }
    # score 분포 히스토그램 (0.0~1.0, 10 bins)
    if all_top1:
        hist, edges = np.histogram(all_top1, bins=10, range=(0.0, 1.0))
        accuracy["top1_score_histogram"] = {
            f"{edges[i]:.1f}~{edges[i+1]:.1f}": int(hist[i])
            for i in range(len(hist))
        }

    return {
        "segmentor":     "mobile_sam_vit_t",
        "n_success":     len(results),
        "total_time_s":  round(total_elapsed, 2),
        "speed":         speed,
        "accuracy":      accuracy,
    }


# ══════════════════════════════════════════════════════════════════════════════
# 인자 파싱 / 메인
# ══════════════════════════════════════════════════════════════════════════════

def _load_yaml_config() -> dict:
    cfg_path = os.path.join(ROOT_DIR, "pipeline_config.yaml")
    if not os.path.exists(cfg_path):
        return {}
    import yaml
    with open(cfg_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def parse_args():
    yml = _load_yaml_config()

    p = argparse.ArgumentParser(
        description="Mobile-SAM ISM 배치 추론 + 속도/정확도 성능 측정",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    p.add_argument("--config",       default=None,                    help="다른 yaml 설정 파일 경로")
    p.add_argument("--rgb_dir",      default=yml.get("rgb_dir"),      help="RGB 이미지 폴더")
    p.add_argument("--depth_dir",    default=yml.get("depth_dir"),    help="Depth 이미지 폴더")
    p.add_argument("--cad_path",     default=yml.get("cad_path"),     help="CAD .ply 경로")
    p.add_argument("--template_dir", default=yml.get("template_dir"), help="템플릿 폴더")
    p.add_argument("--output_dir",
                   default=yml.get("batch_output_dir") or yml.get("output_dir"),
                   help="출력 루트 폴더")

    cam = p.add_mutually_exclusive_group()
    cam.add_argument("--cam_path",          default=yml.get("cam_path"),
                     help="공유 camera.json")
    cam.add_argument("--scene_camera_path", default=yml.get("scene_camera_path"),
                     help="BOP scene_camera.json")

    p.add_argument("--img_ext",           default="png")
    p.add_argument("--top_k_for_output",  default=yml.get("top_k_for_pem", 5), type=int,
                   help="저장할 상위 K개 탐지 결과 (기본 5)")
    p.add_argument("--max_proposals",     default=50, type=int,
                   help="DINOv2 전달 전 proposal 수 제한 (0=제한없음)")
    p.add_argument("--dinov2_chunk_size", default=32, type=int,
                   help="DINOv2 배치 처리 크기")
    p.add_argument("--points_per_side",         default=yml.get("points_per_side", 16), type=int,
                   help="AutoMaskGenerator 그리드 크기 (16→0.47s, 32→1.8s)")
    p.add_argument("--points_per_batch",        default=yml.get("points_per_batch", 128), type=int,
                   help="SAM decoder 배치 크기 (클수록 빠름, VRAM 고려)")
    p.add_argument("--stability_score_thresh",  default=yml.get("stability_score_thresh", 0.85), type=float,
                   help="마스크 안정성 점수 임계값")
    p.add_argument("--no_vis",  action="store_true", help="시각화 이미지 생성 생략")
    p.add_argument("--quiet",   action="store_true",
                   default=bool(yml.get("quiet", False)),
                   help="콘솔 출력 억제")

    args = p.parse_args()

    if args.config:
        import yaml
        with open(args.config, "r", encoding="utf-8") as f:
            override = yaml.safe_load(f) or {}
        for key, val in override.items():
            if getattr(args, key, None) is None:
                setattr(args, key, val)

    missing = [k for k in ("rgb_dir", "depth_dir", "cad_path", "template_dir", "output_dir")
               if not getattr(args, k, None)]
    if missing:
        p.error(f"누락된 필수 인자 (CLI 또는 pipeline_config.yaml에 지정): {missing}")
    if not args.cam_path and not args.scene_camera_path:
        p.error("--cam_path 또는 --scene_camera_path 가 필요합니다.")

    return args


def get_cam_path_for_image(stem, scene_camera, tmp_dir):
    img_id = int(stem)
    cam = scene_camera.get(str(img_id)) or scene_camera.get(img_id)
    if cam is None:
        raise KeyError(f"scene_camera.json 에 image_id={img_id} 없음")
    tmp_path = os.path.join(tmp_dir, f"cam_{stem}.json")
    with open(tmp_path, "w") as f:
        json.dump(cam, f)
    return tmp_path


def main():
    args = parse_args()

    # 절대경로 변환
    args.template_dir = os.path.abspath(args.template_dir)
    args.cad_path     = os.path.abspath(args.cad_path)
    args.rgb_dir      = os.path.abspath(args.rgb_dir)
    args.depth_dir    = os.path.abspath(args.depth_dir)
    args.output_dir   = os.path.abspath(args.output_dir)
    if args.cam_path:
        args.cam_path = os.path.abspath(args.cam_path)
    if args.scene_camera_path:
        args.scene_camera_path = os.path.abspath(args.scene_camera_path)

    if not os.path.isdir(args.template_dir):
        print(f"[ERROR] template_dir 없음: {args.template_dir}")
        sys.exit(1)

    scene_camera = None
    if args.scene_camera_path:
        with open(args.scene_camera_path) as f:
            scene_camera = json.load(f)

    rgb_files = sorted(glob.glob(os.path.join(args.rgb_dir, f"*.{args.img_ext}")))
    if not rgb_files:
        print(f"[ERROR] RGB 이미지 없음: {args.rgb_dir}/*.{args.img_ext}")
        sys.exit(1)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] device: {device}  |  이미지 수: {len(rgb_files)}")
    print(f"[INFO] Mobile-SAM vit_t  points_per_side={args.points_per_side}  "
          f"points_per_batch={args.points_per_batch}  "
          f"stability_score_thresh={args.stability_score_thresh}")
    print(f"[INFO] Mobile-SAM 체크포인트: {MOBILE_SAM_CKPT}")

    if not os.path.isfile(MOBILE_SAM_CKPT):
        print(f"\n[ERROR] Mobile-SAM 가중치 파일 없음: {MOBILE_SAM_CKPT}")
        sys.exit(1)

    # ── 모델 1회 로딩 ───────────────────────────────────────────────────────────
    t_load = time.time()
    ism_model = load_ism_model_with_mobile_sam(
        device,
        points_per_side=args.points_per_side,
        points_per_batch=args.points_per_batch,
        stability_score_thresh=args.stability_score_thresh,
        dinov2_chunk_size=args.dinov2_chunk_size,
    )
    print(f"[LOAD] ISM (Mobile-SAM vit_t + DINOv2) 완료 ({time.time()-t_load:.1f}s)")

    t_tmpl = time.time()
    ism_model = load_ism_templates(ism_model, args.template_dir, args.cad_path, device)
    print(f"[LOAD] 템플릿 feature 추출 완료 ({time.time()-t_tmpl:.1f}s)")

    # ── 출력 폴더 생성 ──────────────────────────────────────────────────────────
    os.makedirs(args.output_dir, exist_ok=True)
    vis_dir = os.path.join(args.output_dir, "vis_ism_mobile")
    if not args.no_vis:
        os.makedirs(vis_dir, exist_ok=True)
    tmp_cam_dir = os.path.join(args.output_dir, "_tmp_cam")
    if scene_camera:
        os.makedirs(tmp_cam_dir, exist_ok=True)

    # ── 로그 ────────────────────────────────────────────────────────────────────
    import datetime
    log_path  = os.path.join(args.output_dir, "mobilesam_inference_log.txt")
    log_lines = []
    _quiet    = args.quiet

    def log(line: str = ""):
        if not _quiet:
            print(line)
        log_lines.append(line)

    def flush_log():
        with open(log_path, "w", encoding="utf-8") as f:
            f.write("\n".join(log_lines) + "\n")

    log(f"=== Mobile-SAM ISM Batch Inference ===")
    log(f"시작 시각: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    log(f"device: {device}  |  이미지 수: {len(rgb_files)}")
    log(f"mobile_sam: vit_t  points_per_side={args.points_per_side}  "
        f"points_per_batch={args.points_per_batch}  "
        f"stability_score_thresh={args.stability_score_thresh}")
    log(f"dinov2_chunk_size: {args.dinov2_chunk_size}  "
        f"max_proposals: {args.max_proposals}  "
        f"top_k_for_output: {args.top_k_for_output}")
    log("")

    # ── 이미지 루프 ─────────────────────────────────────────────────────────────
    success_results = []
    fail_list       = []
    total_t         = time.time()

    for idx, rgb_path in enumerate(rgb_files):
        stem       = os.path.splitext(os.path.basename(rgb_path))[0]
        depth_path = os.path.join(args.depth_dir, f"{stem}.{args.img_ext}")
        img_out    = os.path.join(args.output_dir, stem)

        log(f"\n[{idx+1}/{len(rgb_files)}] {stem}")

        if not os.path.isfile(depth_path):
            log(f"  [SKIP] Depth 없음: {depth_path}")
            fail_list.append((stem, "depth missing"))
            flush_log()
            continue

        if scene_camera:
            try:
                cam_path = get_cam_path_for_image(stem, scene_camera, tmp_cam_dir)
            except (KeyError, ValueError) as e:
                log(f"  [SKIP] 카메라 정보 없음: {e}")
                fail_list.append((stem, str(e)))
                flush_log()
                continue
        else:
            cam_path = args.cam_path

        os.makedirs(img_out, exist_ok=True)

        t0  = time.time()
        ret = run_ism_single(
            ism_model, rgb_path, depth_path, cam_path, img_out, device,
            top_k_for_output=args.top_k_for_output,
            max_proposals=args.max_proposals,
            no_vis=args.no_vis,
            vis_dir=vis_dir,
        )
        elapsed = time.time() - t0

        if not ret["ok"]:
            log(f"  [FAIL] ISM ({elapsed:.2f}s)  {ret.get('error', '')}")
            fail_list.append((stem, ret.get("error", "ISM failed")))
            flush_log()
            continue

        # 성공 로그
        log(f"  [OK]  ISM ({elapsed:.2f}s)  masks={ret['n_masks']}")
        for k, v in ret["timing"].items():
            if k == "proposals":
                log(f"          proposals                    {v}")
            else:
                log(f"          {k:<28} {v:.4f}s")

        if ret["scores"]:
            log(f"        탐지 scores ({len(ret['scores'])}개):")
            for i, s in enumerate(ret["scores"]):
                marker = " ◀ best" if i == 0 else ""
                log(f"          [{i}] final={s['final']:.4f}{marker}")
        else:
            log("        탐지 없음 (score 임계값 미달)")

        success_results.append(ret)
        flush_log()

    # ── 성능 요약 ───────────────────────────────────────────────────────────────
    total_elapsed = time.time() - total_t
    n = len(rgb_files)

    log(f"\n{'='*60}")
    log(f"완료: {len(success_results)}/{n} 성공  |  "
        f"총 {total_elapsed:.1f}s  |  평균 {total_elapsed/n:.2f}s/장")
    if fail_list:
        log("실패 목록:")
        for s, r in fail_list:
            log(f"  - {s}: {r}")

    summary = compute_performance_summary(success_results, total_elapsed)
    summary["n_total"]  = n
    summary["n_failed"] = len(fail_list)

    # 속도 요약 출력
    if "speed" in summary:
        sp = summary["speed"]
        log("\n── 속도 요약 (성공 이미지 기준) ──")
        for stage in ["mask_generation", "dinov2_feature", "semantic_score",
                      "appearance_score", "geometric_score"]:
            if stage in sp:
                log(f"  {stage:<28}  "
                    f"평균 {sp[stage]['mean_s']:.4f}s  "
                    f"중앙값 {sp[stage]['median_s']:.4f}s")
        if "total_per_image" in sp:
            tpi = sp["total_per_image"]
            log(f"  {'total_per_image':<28}  "
                f"평균 {tpi['mean_s']:.4f}s  "
                f"FPS {tpi['fps']:.2f}")
        if "proposals_per_image" in sp:
            prop = sp["proposals_per_image"]
            log(f"  {'proposals_per_image':<28}  "
                f"평균 {prop['mean']:.1f}  "
                f"범위 [{prop['min']},{prop['max']}]")

    # 정확도 요약 출력
    if "accuracy" in summary and "top1_score" in summary["accuracy"]:
        acc = summary["accuracy"]["top1_score"]
        log("\n── 정확도 요약 (Top-1 score 기준) ──")
        log(f"  mean={acc['mean']:.4f}  median={acc['median']:.4f}  "
            f"std={acc['std']:.4f}  "
            f"[{acc['min']:.4f}, {acc['max']:.4f}]")

    # summary JSON 저장
    summary_path = os.path.join(args.output_dir, "performance_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    log(f"\n성능 요약 저장: {summary_path}")
    log(f"로그 저장:     {log_path}")
    flush_log()


if __name__ == "__main__":
    main()
