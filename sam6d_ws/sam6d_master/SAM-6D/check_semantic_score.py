"""
check_semantic_score.py
─────────────────────────────────────────────────────────────────────────────
ISM semantic score 각 세부 단계별 소요 시간 측정 스크립트.

측정 대상
  [DINOv2 forward]
    1. process_rgb_proposals   : 이미지 정규화 + 마스킹 + crop/resize
    2. process_masks_proposals : 마스크 crop/resize
    3. ViT forward (chunk 루프): ViT-Base 12블록 forward per chunk
    4. cls / patch feature 분리 + 마스크 적용 + L2 정규화

  [compute_semantic_score 내부]
    5. 텐서 확장 (repeat)      : query/reference 브로드캐스트용 복사
    6. L2 정규화               : F.normalize
    7. 코사인 유사도 계산       : F.cosine_similarity (N_objects 루프)
    8. permute + clamp
    9. avg_5 집계              : topk(k=5) + mean
   10. argmax (proposal → object 할당)
   11. 임계값 필터링
   12. best_template_pose      : 각 proposal의 최적 템플릿 인덱스 선택

실행:
  cd D:\\LDH_ws\\sam_6d\\SAM-6D
  python check_semantic_score.py
  python check_semantic_score.py --n_repeat 5   # 반복 측정
"""

import os
import sys
import time
import glob
import argparse

import numpy as np
import torch
import torch.nn.functional as F
import yaml
from PIL import Image

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
ISM_DIR  = os.path.join(ROOT_DIR, "Instance_Segmentation_Model")
sys.path.insert(0, ISM_DIR)

from hydra import initialize, compose
from hydra.core.global_hydra import GlobalHydra
from hydra.utils import instantiate
from omegaconf import OmegaConf

from utils.poses.pose_utils import get_obj_poses_from_template_level, load_index_level_in_level2
from utils.bbox_utils import CropResizePad
from model.utils import Detections, BatchedData
from utils.inout import load_json
from model.loss import PairwiseSimilarity


# ══════════════════════════════════════════════════════════════════════════════
# 유틸
# ══════════════════════════════════════════════════════════════════════════════

def cuda_sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()

def tick(label: str, t0: float, results: list, shape_info: str = "") -> float:
    cuda_sync()
    elapsed = time.time() - t0
    mem = torch.cuda.memory_allocated() / 1024**2 if torch.cuda.is_available() else 0
    results.append((label, elapsed, mem, shape_info))
    return time.time()

def print_table(results: list, title: str):
    print(f"\n{'─'*72}")
    print(f"  {title}")
    print(f"{'─'*72}")
    print(f"  {'단계':<42} {'시간(s)':>8}  {'GPU(MB)':>8}  {'텐서 shape'}")
    print(f"  {'─'*42} {'─'*8}  {'─'*8}  {'─'*20}")
    total = 0.0
    for label, elapsed, mem, shape in results:
        print(f"  {label:<42} {elapsed:>8.4f}  {mem:>8.1f}  {shape}")
        total += elapsed
    print(f"  {'─'*42} {'─'*8}")
    print(f"  {'합계':<42} {total:>8.4f}")
    print(f"{'─'*72}")
    return total


# ══════════════════════════════════════════════════════════════════════════════
# 모델 로드
# ══════════════════════════════════════════════════════════════════════════════

def load_model(segmentor_model: str, stability_score_thresh: float, device: torch.device):
    GlobalHydra.instance().clear()
    with initialize(version_base=None, config_path="Instance_Segmentation_Model/configs"):
        cfg = compose(config_name="run_inference.yaml")

    if segmentor_model == "sam":
        GlobalHydra.instance().clear()
        with initialize(version_base=None, config_path="Instance_Segmentation_Model/configs/model"):
            cfg.model = compose(config_name="ISM_sam.yaml")
        cfg.model.segmentor_model.stability_score_thresh = stability_score_thresh
        cfg.model.segmentor_model.sam.checkpoint_dir = os.path.join(
            ISM_DIR, "checkpoints", "segment-anything", "")
    else:
        GlobalHydra.instance().clear()
        with initialize(version_base=None, config_path="Instance_Segmentation_Model/configs/model"):
            cfg.model = compose(config_name="ISM_fastsam.yaml")
        cfg.model.segmentor_model.checkpoint_path = os.path.join(
            ISM_DIR, "checkpoints", "FastSAM", "FastSAM-x.pt")

    cfg.model.descriptor_model.checkpoint_dir = os.path.join(ISM_DIR, "checkpoints", "dinov2", "")
    model = instantiate(cfg.model)

    model.descriptor_model.model = model.descriptor_model.model.to(device)
    model.descriptor_model.model.device = device
    if hasattr(model.segmentor_model, "predictor"):
        model.segmentor_model.predictor.model = model.segmentor_model.predictor.model.to(device)
    else:
        model.segmentor_model.model.setup_model(device=device, verbose=True)

    return model, cfg


def load_templates(model, template_dir: str, cad_path: str, device: torch.device):
    import trimesh
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
    boxes_t   = torch.tensor(np.array(boxes))

    proc = CropResizePad(224)
    templates_proc = proc(images=templates, boxes=boxes_t).to(device)
    masks_cropped  = proc(images=masks,     boxes=boxes_t).to(device)

    model.ref_data = {}
    model.ref_data["descriptors"] = model.descriptor_model.compute_features(
        templates_proc, token_name="x_norm_clstoken"
    ).unsqueeze(0).data
    model.ref_data["appe_descriptors"] = model.descriptor_model.compute_masked_patch_feature(
        templates_proc, masks_cropped[:, 0, :, :]
    ).unsqueeze(0).data

    mesh         = trimesh.load_mesh(cad_path)
    model_points = mesh.sample(2048).astype(np.float32) / 1000.0
    model.ref_data["pointcloud"] = torch.tensor(model_points).unsqueeze(0).data.to(device)

    template_poses = get_obj_poses_from_template_level(level=2, pose_distribution="all")
    template_poses[:, :3, 3] *= 0.4
    poses = torch.tensor(template_poses).to(torch.float32).to(device)
    model.ref_data["poses"] = poses[load_index_level_in_level2(0, "all"), :, :]

    return model, num_templates


# ══════════════════════════════════════════════════════════════════════════════
# DINOv2 세부 타이밍
# ══════════════════════════════════════════════════════════════════════════════

def measure_dinov2(model, rgb_np: np.ndarray, detections, device) -> list:
    """DINOv2 forward 각 세부 단계 타이밍."""
    results = []
    dino = model.descriptor_model
    cuda_sync()

    # 1. process_rgb_proposals
    t = time.time()
    processed_rgbs = dino.process_rgb_proposals(rgb_np, detections.masks, detections.boxes)
    t = tick("1. process_rgb_proposals", t, results,
             f"{list(processed_rgbs.shape)}")

    # 2. process_masks_proposals
    masks_clone = detections.masks.clone()
    processed_masks = dino.process_masks_proposals(masks_clone, detections.boxes)
    t = tick("2. process_masks_proposals", t, results,
             f"{list(processed_masks.shape)}")

    # 3~4. chunk 루프 (ViT forward + feature 분리)
    from model.utils import BatchedData as BD
    batch_rgbs  = BD(batch_size=dino.chunk_size, data=processed_rgbs)
    batch_masks = BD(batch_size=dino.chunk_size, data=processed_masks)
    n_chunks = len(batch_rgbs)

    vit_times, feat_times = [], []
    for i in range(n_chunks):
        rgb_chunk  = batch_rgbs[i]
        mask_chunk = batch_masks[i]

        # ViT forward만 측정
        cuda_sync()
        t_vit = time.time()
        with torch.inference_mode():
            raw_feats = dino.model(rgb_chunk, is_training=True)
        cuda_sync()
        vit_times.append(time.time() - t_vit)

        # feature 분리 + 마스크 + 정규화
        t_feat = time.time()
        patch_features = raw_feats["x_norm_patchtokens"]
        cls_features   = raw_feats["x_norm_clstoken"]
        features_mask  = dino.patch_kernel(mask_chunk).flatten(-2) > dino.validpatch_thresh
        features_mask  = features_mask.unsqueeze(-1).repeat(1, 1, patch_features.shape[-1])
        patch_features = F.normalize(patch_features * features_mask, dim=-1)
        cuda_sync()
        feat_times.append(time.time() - t_feat)

    t_vit_total  = sum(vit_times)
    t_feat_total = sum(feat_times)
    mem = torch.cuda.memory_allocated() / 1024**2 if torch.cuda.is_available() else 0
    results.append((f"3. ViT forward ({n_chunks} chunks, bs={dino.chunk_size})",
                    t_vit_total, mem,
                    f"chunk: [{dino.chunk_size}, 3, 224, 224] → [*, 256, 768]"))
    results.append((f"4. feature 분리 + 마스크 + L2 정규화 ({n_chunks} chunks)",
                    t_feat_total, mem,
                    f"cls:[N,768]  patch:[N,256,768]"))

    return results


# ══════════════════════════════════════════════════════════════════════════════
# Semantic Score 세부 타이밍
# ══════════════════════════════════════════════════════════════════════════════

def measure_semantic_score(model, query_desc, confidence_thresh=0.2) -> list:
    """compute_semantic_score 각 세부 단계 타이밍."""
    results   = []
    ref_desc  = model.ref_data["descriptors"]   # [1, N_templates, 768]
    N_query   = query_desc.shape[0]
    N_objects = ref_desc.shape[0]
    N_templates = ref_desc.shape[1]

    cuda_sync()
    t = time.time()

    # 5. 텐서 확장 (repeat)
    references = ref_desc.clone().unsqueeze(0).repeat(N_query, 1, 1, 1)
    queries    = query_desc.clone().unsqueeze(1).repeat(1, N_templates, 1)
    cuda_sync()
    t = tick("5. 텐서 확장 (repeat)",   t, results,
             f"queries{list(queries.shape)}  refs{list(references.shape)}")

    # 6. L2 정규화
    queries_n    = F.normalize(queries,    dim=-1)
    references_n = F.normalize(references, dim=-1)
    cuda_sync()
    t = tick("6. L2 정규화 (query+ref)", t, results, "")

    # 7. 코사인 유사도 계산 (N_objects 루프)
    from model.utils import BatchedData as BD
    similarity = BD(batch_size=None)
    for idx_obj in range(N_objects):
        sim = F.cosine_similarity(queries_n, references_n[:, idx_obj], dim=-1)
        similarity.append(sim)
    similarity.stack()
    similarity_data = similarity.data
    cuda_sync()
    t = tick(f"7. cosine_similarity (N_obj={N_objects} 루프)", t, results,
             f"→ [{N_query}, {N_objects}, {N_templates}]")

    # 8. permute + clamp
    scores = similarity_data.permute(1, 0, 2).clamp(min=0.0, max=1.0)
    cuda_sync()
    t = tick("8. permute + clamp", t, results,
             f"{list(scores.shape)}")

    # 9. avg_5 집계 (topk + mean)
    score_per_proposal = torch.topk(scores, k=5, dim=-1)[0]
    score_per_proposal = torch.mean(score_per_proposal, dim=-1)
    cuda_sync()
    t = tick("9. avg_5 집계 (topk k=5 + mean)", t, results,
             f"→ [{N_query}, {N_objects}]")

    # 10. argmax (proposal → object 할당)
    score_per_proposal_val, assigned_idx_object = torch.max(score_per_proposal, dim=-1)
    cuda_sync()
    t = tick("10. argmax (proposal→object)", t, results,
             f"score: [{N_query}]")

    # 11. 임계값 필터링
    idx_selected = torch.arange(len(score_per_proposal_val),
                                device=score_per_proposal_val.device
                               )[score_per_proposal_val > confidence_thresh]
    pred_idx_objects = assigned_idx_object[idx_selected]
    semantic_score   = score_per_proposal_val[idx_selected]
    cuda_sync()
    t = tick(f"11. 임계값 필터링 (>{confidence_thresh})", t, results,
             f"통과: {len(idx_selected)}/{N_query}")

    # 12. best_template_pose
    filtered_scores = scores[idx_selected, ...]
    _, best_template_idxes = torch.max(filtered_scores, dim=-1)
    N_q2 = best_template_idxes.shape[0]
    pred_idx_rep = pred_idx_objects[:, None].repeat(1, N_objects)
    best_template_idx = torch.gather(best_template_idxes, dim=1, index=pred_idx_rep)[:, 0]
    cuda_sync()
    t = tick("12. best_template_pose", t, results,
             f"→ [{len(best_template_idx)}]")

    return results, idx_selected, pred_idx_objects, semantic_score, best_template_idx


# ══════════════════════════════════════════════════════════════════════════════
# main
# ══════════════════════════════════════════════════════════════════════════════

def parse_args():
    cfg_path = os.path.join(ROOT_DIR, "pipeline_config.yaml")
    with open(cfg_path, encoding="utf-8") as f:
        yml = yaml.safe_load(f)

    p = argparse.ArgumentParser(description="ISM semantic score 세부 타이밍 측정")
    p.add_argument("--rgb_path",       default=yml.get("rgb_path"))
    p.add_argument("--depth_path",     default=yml.get("depth_path"))
    p.add_argument("--cam_path",       default=yml.get("cam_path"))
    p.add_argument("--cad_path",       default=yml.get("cad_path"))
    p.add_argument("--template_dir",   default=yml.get("template_dir"))
    p.add_argument("--segmentor_model",        default=yml.get("segmentor_model", "fastsam"))
    p.add_argument("--stability_score_thresh", default=yml.get("stability_score_thresh", 0.97), type=float)
    p.add_argument("--n_repeat", default=1, type=int, help="반복 측정 횟수 (평균 계산용)")
    return p.parse_args()


def main():
    args = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"\n[INFO] device: {device}")
    print(f"[INFO] rgb: {args.rgb_path}")
    print(f"[INFO] template_dir: {args.template_dir}")
    print(f"[INFO] segmentor: {args.segmentor_model}")
    print(f"[INFO] n_repeat: {args.n_repeat}")

    # ── 모델 로드 ─────────────────────────────────────────────────────────────
    print("\n[LOAD] 모델 로딩 중...")
    t0 = time.time()
    model, cfg = load_model(args.segmentor_model, args.stability_score_thresh, device)
    print(f"[LOAD] 모델 완료 ({time.time()-t0:.1f}s)")

    print("[LOAD] 템플릿 feature 추출 중...")
    t0 = time.time()
    model, n_templates = load_templates(model, args.template_dir, args.cad_path, device)
    print(f"[LOAD] 템플릿 완료 ({time.time()-t0:.1f}s)  N_templates={n_templates}")

    # ── 마스크 생성 (1회만) ───────────────────────────────────────────────────
    print("\n[PREP] SAM/FastSAM 마스크 생성 중...")
    rgb = Image.open(args.rgb_path).convert("RGB")
    rgb_np = np.array(rgb)
    t0 = time.time()
    detections_raw = model.segmentor_model.generate_masks(rgb_np)
    detections     = Detections(detections_raw)
    n_proposals    = len(detections)
    print(f"[PREP] 마스크 생성 완료 ({time.time()-t0:.1f}s)  N_proposals={n_proposals}")

    # ref descriptor 정보 출력
    ref_desc = model.ref_data["descriptors"]
    print(f"\n[INFO] ref_desc shape   : {list(ref_desc.shape)}"
          f"  (N_objects={ref_desc.shape[0]}, N_templates={ref_desc.shape[1]}, dim={ref_desc.shape[2]})")
    print(f"[INFO] chunk_size(DINOv2): {model.descriptor_model.chunk_size}")

    # ── 반복 측정 ─────────────────────────────────────────────────────────────
    all_dino  = []
    all_sem   = []

    for rep in range(args.n_repeat):
        if args.n_repeat > 1:
            print(f"\n{'='*72}")
            print(f"  반복 {rep+1}/{args.n_repeat}")

        # process_masks_proposals 내부의 in-place unsqueeze_(1)이 detections.masks를
        # 변형하므로 매 반복마다 detections_raw에서 새로 생성해야 함
        detections = Detections(detections_raw)

        # DINOv2 세부 타이밍
        dino_results = measure_dinov2(model, rgb_np, detections, device)

        # query_desc 재추출 (semantic score 입력) — 마찬가지로 새 detections 사용
        detections = Detections(detections_raw)
        with torch.inference_mode():
            query_desc, _ = model.descriptor_model.forward(rgb_np, detections)

        # Semantic score 세부 타이밍
        sem_results, *_ = measure_semantic_score(
            model, query_desc,
            confidence_thresh=cfg.model.matching_config.confidence_thresh
        )

        all_dino.append(dino_results)
        all_sem.append(sem_results)

        dino_total = print_table(dino_results, "DINOv2 forward 세부 타이밍")
        sem_total  = print_table(sem_results,  "compute_semantic_score 세부 타이밍")

    # ── 반복 평균 출력 ────────────────────────────────────────────────────────
    if args.n_repeat > 1:
        print(f"\n{'═'*72}")
        print(f"  {args.n_repeat}회 평균")
        print(f"{'═'*72}")

        def avg_results(all_r):
            n = len(all_r)
            labels = [r[0] for r in all_r[0]]
            for i, label in enumerate(labels):
                times = [all_r[rep][i][1] for rep in range(n)]
                avg   = np.mean(times)
                std   = np.std(times)
                print(f"  {label:<42} {avg:>7.4f}s ± {std:.4f}s")

        print("\n  [DINOv2]")
        avg_results(all_dino)
        print("\n  [Semantic Score]")
        avg_results(all_sem)

    print(f"\n{'═'*72}")
    print("  측정 완료")
    print(f"{'═'*72}\n")


if __name__ == "__main__":
    main()
