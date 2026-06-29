"""
run_batch_inference_fast.py

ISM + PEM 모델을 이미지마다 subprocess로 재시작하지 않고
한 프로세스 안에서 한 번만 로드한 뒤 이미지 루프를 도는 개선 버전.

기존 run_batch_inference.py 대비 기대 효과
  - 이미지당 모델 로드 시간(~15–18 s) 제거
  - DINOv2 / FastSAM / PEM 체크포인트 각 1회 로드
  - 템플릿 feature 추출 1회

사용법 (기존과 동일):
  python run_batch_inference_fast.py \
      --rgb_dir       /path/to/rgb \
      --depth_dir     /path/to/depth \
      --cam_path      /path/to/camera.json \
      --cad_path      /path/to/obj.ply \
      --template_dir  /path/to/templates \
      --output_dir    /path/to/outputs
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


try:
    import gpustat  # noqa: F401
except BaseException as _e:
    import logging

    logging.warning("gpustat unavailable: %s. GPU monitoring disabled.", _e)
    gpustat = None


def _install_pynvml_stub_if_unavailable():
    """Let CPU-only execution import gorilla even when NVML is unavailable."""
    try:
        import pynvml

        pynvml.nvmlInit()
        return
    except BaseException as exc:
        import logging
        import types

        logging.warning("NVML unavailable: %s. GPU monitoring disabled.", exc)

    stub = types.ModuleType("pynvml")

    def _raise_unavailable(*args, **kwargs):
        raise RuntimeError("NVML is unavailable in this environment")

    stub.NVMLError = RuntimeError
    stub.NVMLError_LibraryNotFound = RuntimeError
    stub.nvmlInit = lambda *args, **kwargs: None
    stub.nvmlShutdown = lambda *args, **kwargs: None
    stub.nvmlDeviceGetCount = lambda *args, **kwargs: 0
    stub.nvmlDeviceGetHandleByIndex = _raise_unavailable
    stub.nvmlDeviceGetMemoryInfo = _raise_unavailable
    sys.modules["pynvml"] = stub

    gpustat_stub = types.ModuleType("gpustat")

    class _GPUStatCollection:
        @staticmethod
        def new_query():
            return []

    gpustat_stub.GPUStatCollection = _GPUStatCollection
    sys.modules["gpustat"] = gpustat_stub


_install_pynvml_stub_if_unavailable()

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


if tuple(int(part) for part in torch.__version__.split("+", 1)[0].split(".")[:2]) >= (2, 6):
    _torch_load = torch.load

    def _torch_load_compat(*args, **kwargs):
        kwargs.setdefault("weights_only", False)
        if not torch.cuda.is_available() and kwargs.get("map_location") is None:
            kwargs["map_location"] = torch.device("cpu")
        return _torch_load(*args, **kwargs)

    torch.load = _torch_load_compat

# ── 경로 상수 ──────────────────────────────────────────────────────────────────
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
ISM_DIR  = os.path.join(ROOT_DIR, "Instance_Segmentation_Model")
PEM_DIR  = os.path.join(ROOT_DIR, "Pose_Estimation_Model")

# ISM 패키지 (model/, utils/, segment_anything/ 등)
sys.path.insert(0, ISM_DIR)

# PEM 모듈 (ISM 뒤에 추가 — model 패키지 충돌 없음)
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

from utils.poses.pose_utils import (
    get_obj_poses_from_template_level,
    load_index_level_in_level2,
)
from utils.bbox_utils import CropResizePad, xyxy_to_xywh, force_binary_mask
from model.utils import Detections, convert_npz_to_json, mask_to_rle
from utils.inout import load_json, save_json_bop23
from segment_anything.utils.amg import rle_to_mask

# ── PEM imports ────────────────────────────────────────────────────────────────
import gorilla
from data_utils import (
    load_im,
    get_bbox,
    get_point_cloud_from_depth,
    get_resize_rgb_choose,
)
from draw_utils import draw_detections

# ── transform ──────────────────────────────────────────────────────────────────
_rgb_transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])

_inv_rgb_transform = T.Compose([
    T.Normalize(
        mean=[-0.485 / 0.229, -0.456 / 0.224, -0.406 / 0.225],
        std=[1 / 0.229, 1 / 0.224, 1 / 0.225],
    ),
])


# ══════════════════════════════════════════════════════════════════════════════
# ISM 관련
# ══════════════════════════════════════════════════════════════════════════════

def load_ism_model(segmentor_model: str, stability_score_thresh: float, device: torch.device,
                   dinov2_chunk_size: int = 32, fastsam_checkpoint: str = None):
    """DINOv2 + FastSAM/SAM 모델을 한 번만 로드.
    fastsam_checkpoint: None이면 fast_sam.yaml 기본값(FastSAM-s.pt) 사용, 직접 지정 가능.
    """
    # hydra는 동일 프로세스에서 두 번 initialize 할 수 없으므로
    # GlobalHydra를 사용 후 명시적으로 해제한다.
    from hydra.core.global_hydra import GlobalHydra

    GlobalHydra.instance().clear()
    with initialize(version_base=None,
                    config_path="Instance_Segmentation_Model/configs"):
        cfg = compose(config_name="run_inference.yaml")

    if segmentor_model == "fastsam":
        GlobalHydra.instance().clear()
        with initialize(version_base=None,
                        config_path="Instance_Segmentation_Model/configs/model"):
            cfg.model = compose(config_name="ISM_fastsam.yaml")
    elif segmentor_model == "sam":
        GlobalHydra.instance().clear()
        with initialize(version_base=None,
                        config_path="Instance_Segmentation_Model/configs/model"):
            cfg.model = compose(config_name="ISM_sam.yaml")
        cfg.model.segmentor_model.stability_score_thresh = stability_score_thresh
    else:
        raise ValueError(f"지원하지 않는 segmentor_model: {segmentor_model}")

    # config 안의 ./checkpoints/... 상대경로를 절대경로로 패치
    # (원본은 cwd=ISM_DIR 인 subprocess에서 실행되므로 상대경로가 통했음)
    ckpt_sam    = os.path.join(ISM_DIR, "checkpoints", "segment-anything", "")
    ckpt_fsam   = fastsam_checkpoint or os.path.join(ISM_DIR, "checkpoints", "FastSAM", "FastSAM-s.pt")
    ckpt_dino   = os.path.join(ISM_DIR, "checkpoints", "dinov2", "")
    if segmentor_model == "sam":
        cfg.model.segmentor_model.sam.checkpoint_dir = ckpt_sam
    elif segmentor_model == "fastsam":
        cfg.model.segmentor_model.checkpoint_path = ckpt_fsam
    cfg.model.descriptor_model.checkpoint_dir = ckpt_dino

    model = instantiate(cfg.model)

    # DINOv2 chunk_size 패치 (기본값 16 → 인자로 받은 값)
    # chunk_size가 클수록 DINOv2 forward 횟수가 줄어 추론이 빨라짐
    model.descriptor_model.chunk_size = dinov2_chunk_size

    # 모델을 device로 이동
    model.descriptor_model.model = model.descriptor_model.model.to(device)
    model.descriptor_model.model.device = device
    if hasattr(model.segmentor_model, "predictor"):
        model.segmentor_model.predictor.model = (
            model.segmentor_model.predictor.model.to(device)
        )
    else:
        model.segmentor_model.model.setup_model(device=device, verbose=True)

    return model


def load_ism_templates(model, template_dir: str, cad_path: str, device: torch.device):
    """
    템플릿 feature + pointcloud를 한 번만 추출하여 model.ref_data 에 저장.
    이후 이미지마다 재호출 불필요.
    """
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

    # 포즈 템플릿 (고정)
    template_poses = get_obj_poses_from_template_level(level=2, pose_distribution="all")
    template_poses[:, :3, 3] *= 0.4
    poses = torch.tensor(template_poses).to(torch.float32).to(device)
    model.ref_data["poses"] = poses[load_index_level_in_level2(0, "all"), :, :]

    # CAD 포인트클라우드 (고정)
    mesh = trimesh.load_mesh(cad_path)
    model_points = mesh.sample(2048).astype(np.float32) / 1000.0
    model.ref_data["pointcloud"] = (
        torch.tensor(model_points).unsqueeze(0).data.to(device)
    )

    return model




def detections_to_json_direct(detections, top_k: int = 0) -> list:
    """
    NPZ 중간 파일 없이 Detections → JSON list 직접 변환.
    기존: save_to_file(NPZ 쓰기) → convert_npz_to_json(NPZ 읽기) → save_json (JSON 쓰기)
    개선: 메모리 내 변환 → save_json (JSON 쓰기만)
    """
    masks   = detections.masks        # numpy (N, H, W)
    boxes   = detections.boxes        # numpy (N, 4) xyxy
    scores  = detections.scores       # numpy (N,)
    obj_ids = detections.object_ids   # numpy (N,)
    boxes_xywh = xyxy_to_xywh(boxes)

    results = [
        {
            "scene_id":    0,
            "image_id":    0,
            "category_id": int(obj_ids[i]) + 1,
            "bbox":        boxes_xywh[i].tolist(),
            "score":       float(scores[i]),
            "time":        0.0,
            "segmentation": mask_to_rle(force_binary_mask(masks[i])),
        }
        for i in range(len(scores))
    ]

    if top_k > 0 and len(results) > top_k:
        results = sorted(results, key=lambda d: d["score"], reverse=True)[:top_k]

    return results


def _batch_input_data(depth_path: str, cam_path: str, device: torch.device):
    cam_info = load_json(cam_path)
    depth    = np.array(imageio.imread(depth_path)).astype(np.int32)
    cam_K    = np.array(cam_info["cam_K"]).reshape((3, 3))
    depth_scale = np.array(cam_info["depth_scale"])

    return {
        "depth":         torch.from_numpy(depth).unsqueeze(0).to(device),
        "cam_intrinsic": torch.from_numpy(cam_K).unsqueeze(0).to(device),
        "depth_scale":   torch.from_numpy(depth_scale).unsqueeze(0).to(device),
    }


def _visualize_ism(rgb: Image.Image, detections: list, save_path: str) -> Image.Image:
    img  = np.array(rgb.copy())
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img  = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    colors = distinctipy.get_colors(len(detections))
    alpha  = 0.33

    best_score = -1.0
    best_det   = detections[0]
    for det in detections:
        if det["score"] > best_score:
            best_score = det["score"]
            best_det   = det

    mask = rle_to_mask(best_det["segmentation"])
    edge = canny(mask)
    edge = binary_dilation(edge, np.ones((2, 2)))
    obj_id  = best_det["category_id"]
    temp_id = obj_id - 1

    r = int(255 * colors[temp_id][0])
    g = int(255 * colors[temp_id][1])
    b = int(255 * colors[temp_id][2])
    img[mask, 0] = alpha * r + (1 - alpha) * img[mask, 0]
    img[mask, 1] = alpha * g + (1 - alpha) * img[mask, 1]
    img[mask, 2] = alpha * b + (1 - alpha) * img[mask, 2]
    img[edge, :] = 255

    result = Image.fromarray(np.uint8(img))
    result.save(save_path)
    concat = Image.new("RGB", (img.shape[1] * 2, img.shape[0]))
    concat.paste(rgb,    (0,            0))
    concat.paste(result, (img.shape[1], 0))
    return concat


def run_ism_single(model, rgb_path: str, depth_path: str, cam_path: str,
                   img_output_dir: str, device: torch.device,
                   top_k_for_pem: int = 5,
                   max_proposals: int = 0,
                   no_vis: bool = False,
                   profile: bool = False,
                   vis_ism_dir: str = None) -> dict:
    """이미 로드된 ISM 모델로 단일 이미지 추론.
    반환: {"ok": bool, "timing": dict, "scores": list}
    """
    timing = {}

    def _t(label, t0):
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        elapsed = time.time() - t0
        timing[label] = round(elapsed, 3)
        return time.time()

    try:
        t = time.time()
        rgb = Image.open(rgb_path).convert("RGB")
        t = _t("image_load", t)

        detections = model.segmentor_model.generate_masks(np.array(rgb))
        n_proposals = len(detections["masks"]) if isinstance(detections, dict) else len(detections)
        detections = Detections(detections)

        if max_proposals > 0 and len(detections) > max_proposals:
            areas = (detections.boxes[:, 2] - detections.boxes[:, 0]) * \
                    (detections.boxes[:, 3] - detections.boxes[:, 1])
            _, top_idx = torch.topk(areas, max_proposals)
            detections.filter(top_idx)
        timing["proposals"] = f"{n_proposals}→{len(detections)}"
        t = _t("mask_generation", t)

        query_desc, query_appe_desc = model.descriptor_model.forward(np.array(rgb), detections)
        t = _t("dinov2_feature", t)

        (idx_selected, pred_idx_objects, semantic_score, best_template) = \
            model.compute_semantic_score(query_desc)
        t = _t("semantic_score", t)

        detections.filter(idx_selected)
        query_appe_desc = query_appe_desc[idx_selected, :]

        appe_scores, ref_aux_desc = model.compute_appearance_score(
            best_template, pred_idx_objects, query_appe_desc)
        t = _t("appearance_score", t)

        batch    = _batch_input_data(depth_path, cam_path, device)
        image_uv = model.project_template_to_image(
            best_template, pred_idx_objects, batch, detections.masks)
        geometric_score, visible_ratio = model.compute_geometric_score(
            image_uv, detections, query_appe_desc, ref_aux_desc,
            visible_thred=model.visible_thred)
        t = _t("geometric_score", t)

        final_score = (
            (semantic_score + appe_scores + geometric_score * visible_ratio)
            / (1 + 1 + visible_ratio)
        )

        # ── Debug(FR-9): 분해 점수를 numpy로 보관 (detection_ism.json 스키마는 불변) ──
        # semantic≈similarity, appearance, geometric, visible_ratio 는 모두
        # 선택된 proposal 순서의 1-D 배열이며 final_score 와 길이가 같다.
        _sem_np = semantic_score.detach().cpu().numpy().reshape(-1)
        _app_np = appe_scores.detach().cpu().numpy().reshape(-1)
        _geo_np = geometric_score.detach().cpu().numpy().reshape(-1)
        _vis_np = visible_ratio.detach().cpu().numpy().reshape(-1)
        _fin_np = final_score.detach().cpu().numpy().reshape(-1)

        detections.add_attribute("scores",     final_score)
        detections.add_attribute("object_ids", torch.zeros_like(final_score))
        detections.to_numpy()

        det_list = detections_to_json_direct(detections, top_k=top_k_for_pem)
        save_json_bop23(os.path.join(img_output_dir, "detection_ism.json"), det_list)
        t = _t("save_json", t)

        if not no_vis:
            stem = os.path.splitext(os.path.basename(rgb_path))[0]
            vis_path = os.path.join(vis_ism_dir, f"{stem}.png") if vis_ism_dir \
                       else os.path.join(img_output_dir, "vis_ism.png")
            vis = _visualize_ism(rgb, det_list, vis_path)
            vis.save(vis_path)
        _t("visualization", t)

        # ── [instrumentation] best/top5 template 선택 메타 (계측 전용) ──
        # detector 가 side-channel 로 노출한 per-template 점수(idx_selected 순서)에서
        # 후보별 best/top5 template 을 유도. best_template[i] 는 _sem_np[i] 와 동일
        # idx_selected 순서이므로 정합 보장. ranking/score 에는 영향 없음.
        _tmpl_scores = getattr(model, "_last_template_scores", None)   # [N_sel, N_obj, N_tmpl]
        _pio = getattr(model, "_last_pred_idx_objects", None)          # [N_sel]
        _bt_np = best_template.detach().cpu().numpy().reshape(-1)

        def _template_meta(i: int) -> dict:
            try:
                if _tmpl_scores is None or _pio is None:
                    return {"best_template_id": None, "best_template_score": None,
                            "top5_template_ids": [], "top5_template_scores": []}
                obj_i = int(_pio[i])
                vec = _tmpl_scores[i, obj_i, :]                        # [N_tmpl]
                k = int(min(5, vec.shape[-1]))
                topv, topi = torch.topk(vec, k)
                ids = [int(x) for x in topi.detach().cpu().numpy().reshape(-1)]
                scs = [round(float(x), 4) for x in topv.detach().cpu().numpy().reshape(-1)]
                return {
                    "best_template_id":     int(_bt_np[i]),
                    "best_template_score":  round(float(vec.max().item()), 4),
                    "top5_template_ids":    ids,
                    "top5_template_scores": scs,
                }
            except Exception as _e:
                print(f"    [ISM WARN] template meta 산출 실패 (i={i}): {_e}")
                return {"best_template_id": None, "best_template_score": None,
                        "top5_template_ids": [], "top5_template_scores": []}

        # 정확도용 score 목록 (분해 점수 포함).
        # detections_to_json_direct 와 동일한 정렬/슬라이스를 재현하여 det_list 순서와 정합.
        _n = int(_fin_np.shape[0])
        if top_k_for_pem > 0 and _n > top_k_for_pem:
            _order = np.argsort(-_fin_np)[:top_k_for_pem]
        else:
            _order = np.arange(_n)
        scores = [
            {
                "final":         round(float(_fin_np[i]), 4),
                "similarity":    round(float(_sem_np[i]), 4),
                "appearance":    round(float(_app_np[i]), 4),
                "geometric":     round(float(_geo_np[i]), 4),
                "visible_ratio": round(float(_vis_np[i]), 4),
                **_template_meta(int(i)),
            }
            for i in _order
        ]

        return {"ok": True, "timing": timing, "scores": scores}

    except Exception as e:
        import traceback
        print(f"    [ISM ERROR] {e}")
        traceback.print_exc()
        return {"ok": False, "timing": timing, "scores": [], "error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# PEM 관련
# ══════════════════════════════════════════════════════════════════════════════

def load_pem_model_and_templates(template_dir: str, device: torch.device = None):
    """PEM 모델 + 템플릿 feature를 한 번만 로드."""
    device = device or DEVICE
    cfg = gorilla.Config.fromfile(os.path.join(PEM_DIR, "config", "base.yaml"))
    cfg.model_name = "pose_estimation_model"
    cfg.gpus       = "0"

    MODEL = importlib.import_module(cfg.model_name)
    model = MODEL.Net(cfg.model)
    model = model.to(device)
    model.eval()

    checkpoint = os.path.join(PEM_DIR, "checkpoints", "sam-6d-pem-base.pth")
    gorilla.solver.load_checkpoint(model=model, filename=checkpoint)

    # 템플릿 feature 추출
    all_tem, all_tem_pts, all_tem_choose = _get_pem_templates(
        template_dir, cfg.test_dataset, device
    )
    with torch.inference_mode():
        all_tem_pts, all_tem_feat = model.feature_extraction.get_obj_feats(
            all_tem, all_tem_pts, all_tem_choose
        )

    # CAD 메시도 1회만 로드 (매 이미지마다 로드하면 수 초 낭비)
    cad_path_abs = None  # main()에서 채워짐 — 여기서는 cfg만 반환
    return model, all_tem_pts, all_tem_feat, cfg


def preload_pem_mesh(cad_path: str, n_sample: int):
    """CAD 메시를 한 번만 로드하여 포인트클라우드와 반지름 반환."""
    mesh         = trimesh.load_mesh(cad_path)
    model_points = mesh.sample(n_sample).astype(np.float32) / 1000.0
    radius       = float(np.max(np.linalg.norm(model_points, axis=1)))
    return model_points, radius


def _get_pem_template_single(path: str, cfg, tem_index: int = 1):
    rgb_path  = os.path.join(path, f"rgb_{tem_index}.png")
    mask_path = os.path.join(path, f"mask_{tem_index}.png")
    xyz_path  = os.path.join(path, f"xyz_{tem_index}.npy")

    rgb  = load_im(rgb_path).astype(np.uint8)
    xyz  = np.load(xyz_path).astype(np.float32) / 1000.0
    mask = load_im(mask_path).astype(np.uint8) == 255

    bbox         = get_bbox(mask)
    y1, y2, x1, x2 = bbox
    mask = mask[y1:y2, x1:x2]

    rgb = rgb[:, :, ::-1][y1:y2, x1:x2, :]
    if cfg.rgb_mask_flag:
        rgb = rgb * (mask[:, :, None] > 0).astype(np.uint8)

    rgb = cv2.resize(rgb, (cfg.img_size, cfg.img_size),
                     interpolation=cv2.INTER_LINEAR)
    rgb = _rgb_transform(np.array(rgb))

    choose = (mask > 0).astype(np.float32).flatten().nonzero()[0]
    n = cfg.n_sample_template_point
    choose_idx = (
        np.random.choice(len(choose), n)
        if len(choose) <= n
        else np.random.choice(len(choose), n, replace=False)
    )
    choose = choose[choose_idx]
    xyz    = xyz[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]

    rgb_choose = get_resize_rgb_choose(choose, [y1, y2, x1, x2], cfg.img_size)
    return rgb, rgb_choose, xyz


def _get_pem_templates(path: str, cfg, device: torch.device = None):
    device = device or DEVICE
    total_nView    = 42
    n_template_view = cfg.n_template_view
    all_tem, all_tem_choose, all_tem_pts = [], [], []

    for v in range(n_template_view):
        i = int(total_nView / n_template_view * v)
        tem, tem_choose, tem_pts = _get_pem_template_single(path, cfg, i)
        all_tem.append(torch.FloatTensor(tem).unsqueeze(0).to(device))
        all_tem_choose.append(torch.IntTensor(tem_choose).long().unsqueeze(0).to(device))
        all_tem_pts.append(torch.FloatTensor(tem_pts).unsqueeze(0).to(device))

    return all_tem, all_tem_pts, all_tem_choose


def _get_pem_test_data(rgb_path, depth_path, cam_path, seg_path,
                       det_score_thresh, cfg, model_points, radius,
                       device: torch.device = None):
    """model_points, radius 는 preload_pem_mesh()로 미리 계산해 전달한다."""
    device = device or DEVICE
    with open(seg_path) as f:
        dets_ = json.load(f)
    dets = [d for d in dets_ if d["score"] > det_score_thresh]
    if not dets:
        return None, None, None, None, None

    cam_info  = json.load(open(cam_path))
    K         = np.array(cam_info["cam_K"]).reshape(3, 3)
    depth_scale = cam_info["depth_scale"]

    whole_image = load_im(rgb_path).astype(np.uint8)
    if whole_image.ndim == 2:
        whole_image = np.stack([whole_image] * 3, axis=2)

    whole_depth = load_im(depth_path).astype(np.float32) * depth_scale / 1000.0
    whole_pts   = get_point_cloud_from_depth(whole_depth, K)

    all_rgb, all_cloud, all_rgb_choose, all_score, all_dets = [], [], [], [], []
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
        bbox        = get_bbox(mask)
        y1, y2, x1, x2 = bbox
        mask        = mask[y1:y2, x1:x2]
        choose      = mask.astype(np.float32).flatten().nonzero()[0]

        cloud  = whole_pts.copy()[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]
        center = np.mean(cloud, axis=0)
        flag   = np.linalg.norm(cloud - center, axis=1) < radius * 1.2
        if np.sum(flag) < 4:
            continue
        choose = choose[flag]
        cloud  = cloud[flag]

        n = cfg.n_sample_observed_point
        idx = (
            np.random.choice(len(choose), n)
            if len(choose) <= n
            else np.random.choice(len(choose), n, replace=False)
        )
        choose = choose[idx]
        cloud  = cloud[idx]

        rgb = whole_image.copy()[y1:y2, x1:x2, :][:, :, ::-1]
        if cfg.rgb_mask_flag:
            rgb = rgb * (mask[:, :, None] > 0).astype(np.uint8)
        rgb = cv2.resize(rgb, (cfg.img_size, cfg.img_size),
                         interpolation=cv2.INTER_LINEAR)
        rgb       = _rgb_transform(np.array(rgb))
        rgb_choose = get_resize_rgb_choose(choose, [y1, y2, x1, x2], cfg.img_size)

        all_rgb.append(torch.FloatTensor(rgb))
        all_cloud.append(torch.FloatTensor(cloud))
        all_rgb_choose.append(torch.IntTensor(rgb_choose).long())
        all_score.append(score)
        all_dets.append(inst)

    if not all_rgb:
        return None, None, None, None, None

    ret = {
        "pts":       torch.stack(all_cloud).to(device),
        "rgb":       torch.stack(all_rgb).to(device),
        "rgb_choose": torch.stack(all_rgb_choose).to(device),
        "score":     torch.FloatTensor(all_score).to(device),
    }
    n = ret["pts"].size(0)
    ret["model"] = torch.FloatTensor(model_points).unsqueeze(0).repeat(n, 1, 1).to(device)
    ret["K"]     = torch.FloatTensor(K).unsqueeze(0).repeat(n, 1, 1).to(device)

    return ret, whole_image, whole_pts.reshape(-1, 3), model_points, all_dets


def _visualize_pem(rgb, pred_rot, pred_trans, model_points, K, save_path) -> Image.Image:
    img    = draw_detections(rgb, pred_rot, pred_trans, model_points, K, color=(255, 0, 0))
    result = Image.fromarray(np.uint8(img))
    result.save(save_path)
    rgb_pil = Image.fromarray(np.uint8(rgb))
    concat  = Image.new("RGB", (img.shape[1] * 2, img.shape[0]))
    concat.paste(rgb_pil, (0,            0))
    concat.paste(result,  (img.shape[1], 0))
    return concat


def run_pem_single(pem_model, all_tem_pts, all_tem_feat, cfg,
                   rgb_path, depth_path, cam_path,
                   seg_path, img_output_dir, det_score_thresh,
                   model_points, radius,
                   device: torch.device = None,
                   no_vis: bool = False,
                   profile: bool = False,
                   vis_pem_dir: str = None) -> dict:
    """이미 로드된 PEM 모델로 단일 이미지 추론.
    반환: {"ok": bool, "timing": dict, "poses": list}
    """
    device = device or DEVICE
    timing = {}

    def _t(label, t0):
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        elapsed = time.perf_counter() - t0
        timing[label] = round(elapsed, 3)
        return time.perf_counter()

    try:
        t = time.perf_counter()
        input_data, img, whole_pts, model_points, detections = _get_pem_test_data(
            rgb_path, depth_path, cam_path, seg_path,
            det_score_thresh, cfg.test_dataset, model_points, radius, device,
        )
        if input_data is None:
            print("    [PEM SKIP] score 임계값 통과 detection 없음")
            return {"ok": False, "timing": timing, "poses": [], "error": "no valid detection"}
        ninstance = input_data["pts"].size(0)
        t = _t("data_preprocessing", t)

        with torch.inference_mode():
            input_data["dense_po"] = all_tem_pts.repeat(ninstance, 1, 1)
            input_data["dense_fo"] = all_tem_feat.repeat(ninstance, 1, 1)
            out = pem_model(input_data)
        t = _t("model_forward", t)

        ism_scores  = input_data["score"].detach().cpu().numpy()
        pem_pose_scores = out["pred_pose_score"].detach().cpu().numpy() \
                          if "pred_pose_score" in out else None
        if pem_pose_scores is not None:
            pose_scores = (out["pred_pose_score"] * out["score"]).detach().cpu().numpy()
        else:
            pose_scores = out["score"].detach().cpu().numpy()
        pred_rot   = out["pred_R"].detach().cpu().numpy()
        pred_trans = out["pred_t"].detach().cpu().numpy() * 1000

        for idx, det in enumerate(detections):
            detections[idx]["score"] = float(pose_scores[idx])
            detections[idx]["R"]     = pred_rot[idx].tolist()
            detections[idx]["t"]     = pred_trans[idx].tolist()

        with open(os.path.join(img_output_dir, "detection_pem.json"), "w") as f:
            json.dump(detections, f)
        t = _t("save_json", t)

        if not no_vis:
            valid    = pose_scores == pose_scores.max()
            K_vis    = input_data["K"].detach().cpu().numpy()[valid]
            stem     = os.path.splitext(os.path.basename(rgb_path))[0]
            vis_path = os.path.join(vis_pem_dir, f"{stem}.png") if vis_pem_dir \
                       else os.path.join(img_output_dir, "vis_pem.png")
            vis = _visualize_pem(img, pred_rot[valid], pred_trans[valid],
                                 model_points * 1000, K_vis, vis_path)
            vis.save(vis_path)
        _t("visualization", t)

        # 정확도용 pose 목록 (R 포함 — skip_file_save 모드에서 JSON 없이 직접 사용)
        poses = []
        for i in range(ninstance):
            tx, ty, tz = pred_trans[i]
            poses.append({
                "ism_score":  round(float(ism_scores[i]),  4),
                "pem_score":  round(float(pem_pose_scores[i]), 4) if pem_pose_scores is not None else None,
                "final_score":round(float(pose_scores[i]), 4),
                "t_mm":       [round(tx, 1), round(ty, 1), round(tz, 1)],
                "R":          pred_rot[i].tolist(),
                "obj_id":     detections[i].get("category_id", i + 1),
            })

        return {"ok": True, "timing": timing, "poses": poses}

    except Exception as e:
        print(f"    [PEM ERROR] {e}")
        return {"ok": False, "timing": timing, "poses": [], "error": str(e)}


# ══════════════════════════════════════════════════════════════════════════════
# 인자 파싱 / 메인
# ══════════════════════════════════════════════════════════════════════════════

def _load_yaml_config() -> dict:
    """ROOT_DIR/pipeline_config.yaml 이 있으면 읽어서 반환, 없으면 빈 dict."""
    cfg_path = os.path.join(ROOT_DIR, "pipeline_config.yaml")
    if not os.path.exists(cfg_path):
        return {}
    import yaml
    with open(cfg_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def parse_args():
    yml = _load_yaml_config()

    p = argparse.ArgumentParser(
        description="SAM-6D Fast Batch Inference (single-process)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # pipeline_config.yaml 값을 기본값으로 사용 — CLI 인자로 덮어쓰기 가능
    p.add_argument("--config",       default=None,                     help="다른 yaml 설정 파일 경로")
    p.add_argument("--rgb_dir",      default=yml.get("rgb_dir"),       help="RGB 이미지 폴더")
    p.add_argument("--depth_dir",    default=yml.get("depth_dir"),     help="Depth 이미지 폴더")
    p.add_argument("--cad_path",     default=yml.get("cad_path"),      help="CAD .ply 경로")
    p.add_argument("--template_dir", default=yml.get("template_dir"),  help="템플릿 폴더")
    p.add_argument("--output_dir",   default=yml.get("batch_output_dir") or yml.get("output_dir"),
                   help="배치 출력 루트 폴더 (config의 batch_output_dir 우선, 없으면 output_dir)")

    cam = p.add_mutually_exclusive_group()
    cam.add_argument("--cam_path",          default=yml.get("cam_path"),          help="공유 camera.json")
    cam.add_argument("--scene_camera_path", default=yml.get("scene_camera_path"), help="BOP scene_camera.json")

    p.add_argument("--segmentor_model",        default=yml.get("segmentor_model", "fastsam"), choices=["sam", "fastsam"])
    p.add_argument("--stability_score_thresh", default=yml.get("stability_score_thresh", 0.97), type=float)
    p.add_argument("--det_score_thresh",       default=yml.get("det_score_thresh", 0.2),        type=float)
    p.add_argument("--img_ext",                default="png")
    p.add_argument("--top_k_for_pem", default=yml.get("top_k_for_pem", 5), type=int,
                   help="ISM 점수 상위 K개만 PEM에 전달 (기본 5).")
    p.add_argument("--max_proposals", default=50, type=int,
                   help="DINOv2에 넘기기 전 FastSAM proposal 수 제한 (기본 50, 0=제한없음).")
    p.add_argument("--dinov2_chunk_size", default=32, type=int,
                   help="DINOv2 배치 크기 (기본 32, 원본 16).")
    p.add_argument("--skip_existing", action="store_true",
                   help="이미 detection_ism.json 이 있는 프레임은 건너뜀 (크래시 후 재개/백필용)")
    p.add_argument("--no_vis", action="store_true",  help="시각화 이미지 생성 생략")
    p.add_argument("--profile", action="store_true", help="단계별 소요시간 출력")
    p.add_argument("--quiet",   action="store_true",
                   default=bool(yml.get("quiet", False)),
                   help="콘솔 출력 억제 — 로그는 inference_log.txt 에만 저장")

    args = p.parse_args()

    # --config 로 다른 yaml 지정 시 재로드 후 미설정 값만 채움
    if args.config:
        import yaml
        with open(args.config, "r", encoding="utf-8") as f:
            override = yaml.safe_load(f) or {}
        for key, val in override.items():
            if getattr(args, key, None) is None:
                setattr(args, key, val)

    # 필수 인자 검증
    missing = [k for k in ("rgb_dir", "depth_dir", "cad_path", "template_dir", "output_dir")
               if not getattr(args, k, None)]
    if missing:
        p.error(f"다음 인자가 없습니다 (CLI 또는 pipeline_config.yaml에 지정하세요): {missing}")
    if not args.cam_path and not args.scene_camera_path:
        p.error("--cam_path 또는 --scene_camera_path 중 하나가 필요합니다 (CLI 또는 pipeline_config.yaml).")

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

    # 모든 경로를 절대경로로 변환
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
    print(f"[INFO] device: {device}")
    print(f"[INFO] 처리할 이미지 수: {len(rgb_files)}")

    # ── 모델 1회 로드 ──────────────────────────────────────────────────────────
    print("\n[LOAD] ISM 모델 로딩 중...")
    t0 = time.time()
    ism_model = load_ism_model(
        args.segmentor_model, args.stability_score_thresh, device,
        dinov2_chunk_size=args.dinov2_chunk_size,
    )
    print(f"[LOAD] ISM 모델 완료 ({time.time()-t0:.1f}s)")

    print("[LOAD] ISM 템플릿 feature 추출 중...")
    t0 = time.time()
    ism_model = load_ism_templates(ism_model, args.template_dir, args.cad_path, device)
    print(f"[LOAD] ISM 템플릿 완료 ({time.time()-t0:.1f}s)")

    print("[LOAD] PEM 모델 + 템플릿 로딩 중...")
    t0 = time.time()
    pem_model, all_tem_pts, all_tem_feat, pem_cfg = load_pem_model_and_templates(
        args.template_dir, device
    )
    print(f"[LOAD] PEM 완료 ({time.time()-t0:.1f}s)")

    print("[LOAD] CAD 메시 1회 로딩 중...")
    t0 = time.time()
    pem_model_points, pem_radius = preload_pem_mesh(
        args.cad_path, pem_cfg.test_dataset.n_sample_model_point
    )
    print(f"[LOAD] CAD 메시 완료 ({time.time()-t0:.1f}s)")

    # ── 시각화 공용 폴더 생성 ──────────────────────────────────────────────────
    vis_ism_dir = os.path.join(args.output_dir, "vis_ism")
    vis_pem_dir = os.path.join(args.output_dir, "vis_pem")
    os.makedirs(vis_ism_dir, exist_ok=True)
    os.makedirs(vis_pem_dir, exist_ok=True)

    # ── 임시 카메라 폴더 (BOP 모드) ────────────────────────────────────────────
    tmp_cam_dir = os.path.join(args.output_dir, "_tmp_cam")
    if scene_camera:
        os.makedirs(tmp_cam_dir, exist_ok=True)

    # ── 로그 파일 준비 ────────────────────────────────────────────────────────
    import datetime
    log_path = os.path.join(args.output_dir, "inference_log.txt")
    log_lines = []
    _quiet = args.quiet

    def log(line: str = ""):
        if not _quiet:
            print(line)
        log_lines.append(line)

    def flush_log():
        with open(log_path, "w", encoding="utf-8") as f:
            f.write("\n".join(log_lines) + "\n")

    # ── 이미지 루프 ────────────────────────────────────────────────────────────
    success_count = 0
    fail_list     = []
    total_t       = time.time()

    for idx, rgb_path in enumerate(rgb_files):
        stem       = os.path.splitext(os.path.basename(rgb_path))[0]
        depth_path = os.path.join(args.depth_dir, f"{stem}.{args.img_ext}")
        img_out    = os.path.join(args.output_dir, stem)
        seg_path   = os.path.join(img_out, "detection_ism.json")

        # Resume support: skip frames whose ISM already ran (used by retry/backfill
        # after an intermittent crash, so coverage accumulates across attempts).
        if getattr(args, "skip_existing", False) and os.path.isfile(seg_path):
            continue

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

        # ISM
        t0     = time.time()
        ism_ret = run_ism_single(ism_model, rgb_path, depth_path, cam_path, img_out, device,
                                 top_k_for_pem=args.top_k_for_pem,
                                 max_proposals=args.max_proposals,
                                 no_vis=args.no_vis,
                                 profile=args.profile,
                                 vis_ism_dir=vis_ism_dir)
        ism_elapsed = time.time() - t0

        if not ism_ret["ok"]:
            log(f"  [FAIL] ISM ({ism_elapsed:.1f}s)  {ism_ret.get('error','')}")
            fail_list.append((stem, "ISM failed"))
            flush_log()
            continue

        # ISM 타이밍 로그
        log(f"  [OK]   ISM ({ism_elapsed:.1f}s)")
        for k, v in ism_ret["timing"].items():
            if k != "proposals":
                log(f"           {k:<28} {v:.3f}s")
        log(f"           proposals                    {ism_ret['timing'].get('proposals','')}")

        # ISM score 로그
        log(f"         ISM 마스크 scores ({len(ism_ret['scores'])}개):")
        for i, s in enumerate(ism_ret["scores"]):
            marker = " ◀ best" if i == 0 else ""
            log(f"           [{i}] final={s['final']:.4f}{marker}")

        # PEM
        t0      = time.time()
        pem_ret = run_pem_single(
            pem_model, all_tem_pts, all_tem_feat, pem_cfg,
            rgb_path, depth_path, cam_path,
            seg_path, img_out, args.det_score_thresh,
            pem_model_points, pem_radius,
            device=device,
            no_vis=args.no_vis,
            profile=args.profile,
            vis_pem_dir=vis_pem_dir,
        )
        pem_elapsed = time.time() - t0

        if not pem_ret["ok"]:
            log(f"  [FAIL] PEM ({pem_elapsed:.1f}s)  {pem_ret.get('error','')}")
            fail_list.append((stem, "PEM failed"))
            flush_log()
            continue

        # PEM 타이밍 로그
        log(f"  [OK]   PEM ({pem_elapsed:.1f}s)")
        for k, v in pem_ret["timing"].items():
            log(f"           {k:<28} {v:.3f}s")

        # PEM pose score 로그
        log(f"         PEM pose scores ({len(pem_ret['poses'])}개):")
        best_idx = max(range(len(pem_ret["poses"])), key=lambda i: pem_ret["poses"][i]["final_score"])
        for i, p in enumerate(pem_ret["poses"]):
            marker = " ◀ best" if i == best_idx else ""
            pem_s  = f"{p['pem_score']:.4f}" if p["pem_score"] is not None else "  N/A "
            tx, ty, tz = p["t_mm"]
            log(f"           [{i}] ism={p['ism_score']:.4f}  pem={pem_s}  "
                f"final={p['final_score']:.4f}  t=[{tx:.1f},{ty:.1f},{tz:.1f}]mm{marker}")

        success_count += 1
        flush_log()

    # ── 최종 요약 ─────────────────────────────────────────────────────────────
    elapsed = time.time() - total_t
    n       = len(rgb_files)
    log(f"\n{'='*55}")
    log(f"완료: {success_count}/{n} 성공  |  총 {elapsed:.1f}s  |  평균 {elapsed/n:.1f}s/장")
    if fail_list:
        log("실패 목록:")
        for s, r in fail_list:
            log(f"  - {s}: {r}")
    log(f"로그 저장: {log_path}")
    flush_log()


if __name__ == "__main__":
    main()
