"""
visualize_sappe_score.py
------------------------
SAM-6D 논문의 Appearance Matching (sappe) 파이프라인 시각화

논문 파이프라인:
  1. 후보 m의 패치 임베딩 추출 (DINOv2 x_norm_patchtokens, 16x16 = 256 패치)
  2. ssem으로 결정된 Tbest의 패치 임베딩 추출
  3. 각 쿼리 패치 → Tbest 패치 전체와 최대 코사인 유사도 계산
  4. 패치별 최대 유사도 평균 = sappe 스코어

출력 파일 3개 (visualization/image/):
  ├── vis_3_query_patch_emb.png   : 후보 m의 패치 임베딩 히트맵
  ├── vis_4_tbest_patch_emb.png   : Tbest 패치 임베딩 히트맵
  └── vis_5_sappe_comparison.png  : 쿼리 ↔ Tbest 패치 비교 + sappe 스코어

실행 방법 (ISM 디렉토리에서):
  cd D:/LDH_ws/sam_6d/SAM-6D/Instance_Segmentation_Model
  python visualization/code/visualize_sappe_score.py --output_dir ../Data/Example/outputs
"""

import sys
import os
import glob
import argparse
import numpy as np
import torch
import torch.nn.functional as F
import cv2
from PIL import Image, ImageDraw, ImageFont
import matplotlib.cm as cm

# ── 경로 설정 ──────────────────────────────────────────────────────────────
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ISM_DIR  = os.path.normpath(os.path.join(THIS_DIR, "..", ".."))
sys.path.insert(0, ISM_DIR)

from model.sam    import load_sam, CustomSamAutomaticMaskGenerator
from model.dinov2 import CustomDINOv2
from model.utils  import Detections
from utils.bbox_utils import CropResizePad

# ── 논문 하이퍼파라미터 ────────────────────────────────────────────────────
K          = 5        # ssem top-K (ISM_sam.yaml 기준)
PATCH_GRID = 16       # 224 / 14 = 16  →  16×16 = 256 패치

# ── 시각화 상수 ───────────────────────────────────────────────────────────
CROP_SZ  = 224
PAD      = 6
HDR_H    = 26
TOP_N    = 5
BG_COLOR = (30, 30, 30)

# ── 한글 폰트 설정 (cv2.putText 미지원 → PIL 사용) ──────────────────────
_FONT_PATH  = "C:/Windows/Fonts/malgun.ttf"   # Malgun Gothic
_FONT_CACHE = {}


def _font(size: int = 13):
    if size not in _FONT_CACHE:
        try:
            _FONT_CACHE[size] = ImageFont.truetype(_FONT_PATH, size)
        except Exception:
            _FONT_CACHE[size] = ImageFont.load_default()
    return _FONT_CACHE[size]


def put_text(img: np.ndarray, text: str, pos=(2, 4),
             size: int = 13, color=(255, 255, 255), shadow: bool = True) -> np.ndarray:
    """numpy 이미지에 한글/영문 텍스트 삽입 (PIL 사용)"""
    pil = Image.fromarray(img.astype(np.uint8))
    d   = ImageDraw.Draw(pil)
    f   = _font(size)
    if shadow:
        d.text((pos[0] + 1, pos[1] + 1), text, font=f, fill=(0, 0, 0))
    d.text(pos, text, font=f, fill=color)
    return np.array(pil)


def make_bar(width: int, text: str, height: int = HDR_H,
             bg=(50, 50, 50), size: int = 13) -> np.ndarray:
    """헤더 바 이미지 생성 (한글 지원)"""
    bar = np.full((height, width, 3), bg, dtype=np.uint8)
    return put_text(bar, text, (4, 5), size=size, color=(220, 220, 200), shadow=False)


# ── 이미지 조합 유틸 ──────────────────────────────────────────────────────

def hstack(imgs: list, pad: int = PAD, bg=BG_COLOR) -> np.ndarray:
    h   = imgs[0].shape[0]
    sep = np.full((h, pad, 3), bg, dtype=np.uint8)
    out = []
    for im in imgs:
        out.append(im)
        out.append(sep)
    return np.hstack(out[:-1])


def vstack(imgs: list, pad: int = PAD, bg=BG_COLOR) -> np.ndarray:
    w   = imgs[0].shape[1]
    sep = np.full((pad, w, 3), bg, dtype=np.uint8)
    out = []
    for im in imgs:
        out.append(im)
        out.append(sep)
    return np.vstack(out[:-1])


# ── 히트맵 유틸 ──────────────────────────────────────────────────────────

def patch_norm_hm(embs: torch.Tensor, size: int) -> np.ndarray:
    """패치 임베딩 [P, D] → L2 노름 히트맵 RGB [size, size, 3]"""
    norms = embs.norm(dim=-1).cpu().float().numpy()   # [256]
    grid  = norms.reshape(PATCH_GRID, PATCH_GRID)
    grid  = (grid - grid.min()) / (grid.max() - grid.min() + 1e-8)
    grid  = cv2.resize(grid, (size, size), interpolation=cv2.INTER_NEAREST)
    return (cm.viridis(grid)[:, :, :3] * 255).astype(np.uint8)


def sim_hm(max_sims: torch.Tensor, size: int) -> np.ndarray:
    """패치별 최대 유사도 [P] → 히트맵 RGB [size, size, 3]"""
    v    = max_sims.cpu().float().numpy()             # [256]
    grid = v.reshape(PATCH_GRID, PATCH_GRID)
    grid = (grid - grid.min()) / (grid.max() - grid.min() + 1e-8)
    grid = cv2.resize(grid, (size, size), interpolation=cv2.INTER_NEAREST)
    return (cm.plasma(grid)[:, :, :3] * 255).astype(np.uint8)


def overlay(img: np.ndarray, hm: np.ndarray, alpha: float = 0.5) -> np.ndarray:
    """원본 이미지에 히트맵 오버레이"""
    img_r = cv2.resize(img, (hm.shape[1], hm.shape[0]))
    return cv2.addWeighted(img_r, 1 - alpha, hm, alpha, 0)


def box_crop(rgb: np.ndarray, box, size: int) -> np.ndarray:
    x1, y1, x2, y2 = [int(v) for v in box]
    x1 = max(0, x1); y1 = max(0, y1)
    x2 = min(rgb.shape[1], x2); y2 = min(rgb.shape[0], y2)
    c  = rgb[y1:y2, x1:x2]
    if c.size == 0:
        return np.zeros((size, size, 3), dtype=np.uint8)
    return cv2.resize(c, (size, size))


# ══════════════════════════════════════════════════════════════════════════
# 모델 로드
# ══════════════════════════════════════════════════════════════════════════

def load_models(device):
    print("[LOAD] SAM vit_h ...")
    sam = load_sam("vit_h", ISM_DIR)
    sam.to(device=device)
    sam_gen = CustomSamAutomaticMaskGenerator(
        sam=sam,
        stability_score_thresh=0.85,
        segmentor_width_size=640,
    )
    print("[LOAD] DINOv2 vitl14 ...")
    dinov2 = CustomDINOv2(
        model_name="dinov2_vitl14",
        token_name="x_norm_clstoken",
        image_size=224,
        chunk_size=16,
        descriptor_width_size=640,
        checkpoint_dir=ISM_DIR,
        patch_size=14,
    )
    dinov2.model = dinov2.model.to(device)
    dinov2.model.device = device
    return sam_gen, dinov2


# ══════════════════════════════════════════════════════════════════════════
# 템플릿 로드
# ══════════════════════════════════════════════════════════════════════════

def load_templates(template_dir: str, device):
    npy_files = sorted(glob.glob(os.path.join(template_dir, "*.npy")))
    if not npy_files:
        raise FileNotFoundError(
            f"템플릿 파일이 없습니다: {template_dir}\n"
            "먼저 렌더링 스텝(BlenderProc)을 실행해주세요."
        )
    n = len(npy_files)
    print(f"[LOAD] 템플릿 {n}개 로딩: {template_dir}")

    boxes, masks_list, templates_list, raw_imgs = [], [], [], []
    for i in range(n):
        img_pil  = Image.open(os.path.join(template_dir, f"rgb_{i}.png")).convert("RGB")
        mask_pil = Image.open(os.path.join(template_dir, f"mask_{i}.png"))
        raw_imgs.append(np.array(img_pil))
        boxes.append(mask_pil.getbbox())
        img_t  = torch.from_numpy(np.array(img_pil)  / 255.).float()
        mask_t = torch.from_numpy(np.array(mask_pil.convert("L")) / 255.).float()
        img_t  = img_t * mask_t[:, :, None]
        templates_list.append(img_t)
        masks_list.append(mask_t.unsqueeze(-1))

    templates = torch.stack(templates_list).permute(0, 3, 1, 2)  # [T,3,H,W]
    masks     = torch.stack(masks_list).permute(0, 3, 1, 2)      # [T,1,H,W]
    boxes_t   = torch.tensor(np.array(boxes))
    proc      = CropResizePad(224)
    templates_224 = proc(images=templates, boxes=boxes_t).to(device)
    masks_224     = proc(images=masks,     boxes=boxes_t).to(device)
    return templates_224, masks_224, raw_imgs, n


# ══════════════════════════════════════════════════════════════════════════
# vis_3: 후보 m의 패치 임베딩 히트맵
# ══════════════════════════════════════════════════════════════════════════

def make_vis3(rgb_np, boxes, query_patch_embs, ssem_all, sappe_scores, out_path):
    top5 = ssem_all.argsort(descending=True)[:TOP_N].tolist()
    rows = []
    for prop_i in top5:
        crop  = box_crop(rgb_np, boxes[prop_i], CROP_SZ)
        hm    = patch_norm_hm(query_patch_embs[prop_i], CROP_SZ)
        ov    = overlay(crop, hm)

        crop  = put_text(crop, f"m#{prop_i + 1}", (2, 4), size=12, color=(255, 220, 50))
        hm    = put_text(hm,   "패치 노름 히트맵",  (2, 4), size=12, color=(255, 255, 255))
        ov    = put_text(ov,   "오버레이",           (2, 4), size=12, color=(255, 255, 255))

        row   = hstack([crop, hm, ov])
        hdr   = make_bar(row.shape[1],
                         f"후보 m#{prop_i + 1}  |  ssem={ssem_all[prop_i]:.4f}  |  "
                         f"sappe={sappe_scores[prop_i]:.4f}",
                         bg=(50, 50, 80))
        rows.append(np.vstack([hdr, row]))

    panel = vstack(rows, pad=PAD * 2)
    title = make_bar(panel.shape[1],
                     f"후보 m 패치 임베딩 히트맵  (상위 {TOP_N}개, ssem 내림차순)",
                     height=HDR_H + 4, bg=(20, 20, 60), size=14)
    Image.fromarray(np.vstack([title, panel])).save(out_path)
    print(f"[SAVE] {out_path}")


# ══════════════════════════════════════════════════════════════════════════
# vis_4: Tbest 패치 임베딩 히트맵
# ══════════════════════════════════════════════════════════════════════════

def make_vis4(raw_tmpl_imgs, tmpl_patch_embs, ssem_all, best_idx, sappe_scores, out_path):
    top5 = ssem_all.argsort(descending=True)[:TOP_N].tolist()
    rows = []
    for prop_i in top5:
        t_idx = best_idx[prop_i].item()
        tbest = cv2.resize(raw_tmpl_imgs[t_idx], (CROP_SZ, CROP_SZ))
        hm    = patch_norm_hm(tmpl_patch_embs[t_idx], CROP_SZ)
        ov    = overlay(tbest, hm)

        tbest = put_text(tbest, f"Tbest T{t_idx}", (2, 4), size=12, color=(255, 220, 50))
        hm    = put_text(hm,   "패치 노름 히트맵",   (2, 4), size=12, color=(255, 255, 255))
        ov    = put_text(ov,   "오버레이",            (2, 4), size=12, color=(255, 255, 255))

        row   = hstack([tbest, hm, ov])
        hdr   = make_bar(row.shape[1],
                         f"m#{prop_i + 1}의 Tbest=T{t_idx}  |  sappe={sappe_scores[prop_i]:.4f}",
                         bg=(50, 80, 50))
        rows.append(np.vstack([hdr, row]))

    panel = vstack(rows, pad=PAD * 2)
    title = make_bar(panel.shape[1],
                     f"Tbest 패치 임베딩 히트맵  (상위 {TOP_N}개 후보의 Tbest)",
                     height=HDR_H + 4, bg=(20, 60, 20), size=14)
    Image.fromarray(np.vstack([title, panel])).save(out_path)
    print(f"[SAVE] {out_path}")


# ══════════════════════════════════════════════════════════════════════════
# vis_5: sappe 비교 (쿼리 ↔ Tbest 패치 유사도 맵)
# ══════════════════════════════════════════════════════════════════════════

def make_vis5(rgb_np, boxes, query_patch_embs, tmpl_patch_embs,
              raw_tmpl_imgs, ssem_all, best_idx, sappe_scores, max_sims_all, out_path):
    top5 = ssem_all.argsort(descending=True)[:TOP_N].tolist()
    rows = []
    for prop_i in top5:
        t_idx   = best_idx[prop_i].item()

        # 쿼리
        crop    = box_crop(rgb_np, boxes[prop_i], CROP_SZ)
        q_hm    = patch_norm_hm(query_patch_embs[prop_i], CROP_SZ)
        q_ov    = overlay(crop, q_hm)

        # Tbest
        tbest   = cv2.resize(raw_tmpl_imgs[t_idx], (CROP_SZ, CROP_SZ))
        t_hm    = patch_norm_hm(tmpl_patch_embs[t_idx], CROP_SZ)
        t_ov    = overlay(tbest, t_hm)

        # 패치별 최대 유사도 맵
        ms_map  = sim_hm(max_sims_all[prop_i], CROP_SZ)
        ms_ov   = overlay(crop, ms_map, alpha=0.6)

        # 레이블
        crop    = put_text(crop,  f"쿼리 m#{prop_i + 1}", (2, 4), size=12, color=(255, 220, 50))
        q_ov    = put_text(q_ov,  "쿼리 패치 히트맵",      (2, 4), size=12, color=(255, 255, 255))
        tbest   = put_text(tbest, f"Tbest T{t_idx}",       (2, 4), size=12, color=(100, 255, 100))
        t_ov    = put_text(t_ov,  "Tbest 패치 히트맵",     (2, 4), size=12, color=(255, 255, 255))
        ms_ov   = put_text(ms_ov, "최대 유사도 맵",         (2, 4), size=12, color=(255, 255, 255))

        vsep    = np.full((CROP_SZ, 3, 3), (80, 80, 80), dtype=np.uint8)
        row     = hstack([crop, q_ov, vsep, tbest, t_ov, vsep, ms_ov])
        hdr     = make_bar(row.shape[1],
                           f"m#{prop_i + 1} ↔ Tbest T{t_idx}  |  "
                           f"ssem={ssem_all[prop_i]:.4f}  |  sappe={sappe_scores[prop_i]:.4f}",
                           bg=(50, 50, 80))
        rows.append(np.vstack([hdr, row]))

    panel = vstack(rows, pad=PAD * 2)
    title = make_bar(panel.shape[1],
                     "sappe 파이프라인  [쿼리 | 쿼리 히트맵 | Tbest | Tbest 히트맵 | 최대 유사도 맵]",
                     height=HDR_H + 4, bg=(20, 20, 60), size=14)
    Image.fromarray(np.vstack([title, panel])).save(out_path)
    print(f"[SAVE] {out_path}")


# ══════════════════════════════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="SAM-6D sappe 스코어 시각화")
    parser.add_argument("--rgb_path",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/rgb.png")))
    parser.add_argument("--output_dir",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/outputs")))
    parser.add_argument("--save_dir",
                        default=os.path.normpath(os.path.join(THIS_DIR, "..", "image")))
    parser.add_argument("--stability_score_thresh", type=float, default=0.85)
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] device={device}")

    # ── 모델 로드 ──────────────────────────────────────────────────────
    sam_gen, dinov2 = load_models(device)

    # ── 템플릿 로드 ─────────────────────────────────────────────────────
    template_dir  = os.path.join(args.output_dir, "templates")
    templates_224, masks_224, raw_tmpl_imgs, n_tmpl = load_templates(template_dir, device)

    # ── 템플릿 CLS 임베딩 (ssem → Tbest 결정용) ──────────────────────
    print(f"[EMBED] 템플릿 CLS 임베딩 ({n_tmpl}개)...")
    tmpl_cls_embs = dinov2.compute_features(templates_224, token_name="x_norm_clstoken")

    # ── 템플릿 패치 임베딩 (sappe용) ────────────────────────────────
    # masks_224: [T, 1, 224, 224] → squeeze(1) → [T, 224, 224]
    # compute_masked_patch_feature 내부는 3D 마스크를 가정함
    print(f"[EMBED] 템플릿 패치 임베딩 ({n_tmpl}개)...")
    tmpl_patch_embs = dinov2.compute_masked_patch_feature(
        templates_224, masks_224.squeeze(1)
    )  # [T, 256, 1024]

    # ── SAM 마스크 제안 ──────────────────────────────────────────────
    rgb_np = np.array(Image.open(args.rgb_path).convert("RGB"))
    print(f"[SAM] 마스크 제안 생성 중... ({rgb_np.shape[1]}x{rgb_np.shape[0]})")
    with torch.inference_mode():
        det_raw = sam_gen.generate_masks(rgb_np)
    detections = Detections(det_raw)
    N = detections.masks.shape[0]
    print(f"[SAM] 검출 후보 수: {N}개")

    # ── 쿼리 CLS 임베딩 → ssem → Tbest ─────────────────────────────
    print("[EMBED] 쿼리 CLS 임베딩...")
    detections.masks = detections.masks.to(device)
    detections.boxes = detections.boxes.to(device)
    query_cls_embs = dinov2.forward_cls_token(rgb_np, detections)  # [N, 1024]

    q_norm  = F.normalize(query_cls_embs, dim=-1)
    t_norm  = F.normalize(tmpl_cls_embs,  dim=-1)
    sim_cls = (q_norm @ t_norm.T + 1) / 2         # [N, T] → [0, 1]
    topk_v, _ = torch.topk(sim_cls, k=K, dim=-1)
    ssem_all   = topk_v.mean(dim=-1)               # [N]
    best_idx   = sim_cls.argmax(dim=-1)            # [N]

    # ── 쿼리 패치 임베딩 ────────────────────────────────────────────
    print("[EMBED] 쿼리 패치 임베딩...")
    query_patch_embs = dinov2.forward_patch_tokens(rgb_np, detections)  # [N, 256, 1024]

    # ── sappe 계산 ──────────────────────────────────────────────────
    print(f"[SCORE] sappe 계산 중...")
    sappe_list   = []
    max_sims_list = []
    for i in range(N):
        t_idx    = best_idx[i].item()
        q_pat    = query_patch_embs[i]         # [256, 1024] normalized
        t_pat    = tmpl_patch_embs[t_idx]      # [256, 1024] normalized
        sim_mat  = q_pat @ t_pat.T             # [256, 256]
        max_sims = sim_mat.max(dim=-1).values  # [256]
        sappe_list.append(max_sims.mean().item())
        max_sims_list.append(max_sims)

    sappe_scores = torch.tensor(sappe_list)
    max_sims_all = torch.stack(max_sims_list)  # [N, 256]
    print(f"[SCORE] sappe 범위: min={sappe_scores.min():.4f}  max={sappe_scores.max():.4f}")

    # ── 시각화 저장 ─────────────────────────────────────────────────
    boxes_cpu = detections.boxes.cpu()
    path3 = os.path.join(args.save_dir, "vis_3_query_patch_emb.png")
    path4 = os.path.join(args.save_dir, "vis_4_tbest_patch_emb.png")
    path5 = os.path.join(args.save_dir, "vis_5_sappe_comparison.png")

    make_vis3(rgb_np, boxes_cpu, query_patch_embs.cpu(),
              ssem_all.cpu(), sappe_scores, path3)
    make_vis4(raw_tmpl_imgs, tmpl_patch_embs.cpu(),
              ssem_all.cpu(), best_idx.cpu(), sappe_scores, path4)
    make_vis5(rgb_np, boxes_cpu, query_patch_embs.cpu(), tmpl_patch_embs.cpu(),
              raw_tmpl_imgs, ssem_all.cpu(), best_idx.cpu(),
              sappe_scores, max_sims_all.cpu(), path5)

    print(f"\n=== 완료 ===")
    print(f"  후보 수       : {N}개")
    print(f"  템플릿 수     : {n_tmpl}개")
    print(f"  vis_3 (쿼리 패치 임베딩)  → {path3}")
    print(f"  vis_4 (Tbest 패치 임베딩) → {path4}")
    print(f"  vis_5 (sappe 비교)        → {path5}")


if __name__ == "__main__":
    main()
