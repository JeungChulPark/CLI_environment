"""
visualize_ssem_score.py
-----------------------
SAM-6D 논문의 Semantic Matching (ssem) 파이프라인 시각화

논문 파이프라인 (ISM_sam.yaml 기준, K=5):
  1. SAM 마스크 제안 → 후보 m 추출
  2. 각 후보 m을 224x224로 Crop
  3. DINOv2 ViT-L/14로 CLS 임베딩 추출 (1024차원)
  4. 42개 템플릿의 CLS 임베딩과 코사인 유사도 계산
  5. Top-K=5 유사도 평균 = ssem 스코어
  6. 가장 유사한 템플릿 Tbest 결정

출력 파일 2개 (SAM-6D/Instance_Segmentation_Model/Image/):
  ├── vis_1_all_proposals.png  : 전체 후보 m의 Crop 그리드 (ssem 순 정렬)
  └── vis_2_ssem_pipeline.png  : 상위 5개 제안에 대한 전체 파이프라인
                                 [Crop | CLS임베딩 | Top-K 템플릿+임베딩 | Tbest]

실행 방법 (ISM 디렉토리에서):
  cd D:/LDH_ws/sam_6d/SAM-6D/Instance_Segmentation_Model
  python visualization/code/visualize_ssem_score.py --output_dir Data/Example/outputs
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


def _put_text(img: np.ndarray, text: str, pos=(2, 4),
              size: int = 13, color=(255, 255, 255), shadow: bool = True) -> np.ndarray:
    """numpy 이미지에 한글/영문 텍스트 삽입 (PIL 사용)"""
    pil = Image.fromarray(img.astype(np.uint8))
    d   = ImageDraw.Draw(pil)
    f   = _font(size)
    if shadow:
        d.text((pos[0] + 1, pos[1] + 1), text, font=f, fill=(0, 0, 0))
    d.text(pos, text, font=f, fill=color)
    return np.array(pil)

# ── 경로 설정 ──────────────────────────────────────────────────────────────
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ISM_DIR  = os.path.normpath(os.path.join(THIS_DIR, "..", ".."))
sys.path.insert(0, ISM_DIR)

from model.sam    import load_sam, CustomSamAutomaticMaskGenerator
from model.dinov2 import CustomDINOv2
from model.utils  import Detections
from utils.bbox_utils import CropResizePad

# ── 논문 하이퍼파라미터 (ISM_sam.yaml 동일) ─────────────────────────────────
K = 5   # avg_5: top-K 유사도 평균으로 ssem 계산

# ── 시각화 상수 ───────────────────────────────────────────────────────────
CROP_SZ   = 120   # Crop / CLS heatmap / Tbest 셀 크기
TMPL_SZ   = 80    # 개별 템플릿 이미지 셀 크기
PAD       = 4     # 셀 간격
HDR_H     = 20    # 행 헤더(ssem 텍스트) 높이
TOP_N     = 5     # vis_2에 표시할 상위 제안 수
BG_COLOR  = (30, 30, 30)


# ══════════════════════════════════════════════════════════════════════════
# 유틸리티 함수
# ══════════════════════════════════════════════════════════════════════════

def emb_to_heatmap(emb: torch.Tensor, size: int) -> np.ndarray:
    """1024차원 임베딩 벡터 → (size, size, 3) RGB uint8 히트맵"""
    v = emb.cpu().float().numpy()
    v = (v - v.min()) / (v.max() - v.min() + 1e-8)
    side = int(np.sqrt(len(v)))           # 32  (32*32 = 1024)
    tile = v[:side * side].reshape(side, side)
    tile = cv2.resize(tile, (size, size), interpolation=cv2.INTER_NEAREST)
    rgb  = (cm.viridis(tile)[:, :, :3] * 255).astype(np.uint8)
    return rgb


def box_crop(rgb_np: np.ndarray, box, size: int) -> np.ndarray:
    """바운딩박스로 원본 RGB 이미지를 Crop → (size, size, 3) uint8"""
    x1, y1, x2, y2 = [int(v) for v in box]
    x1 = max(0, x1); y1 = max(0, y1)
    x2 = min(rgb_np.shape[1], x2); y2 = min(rgb_np.shape[0], y2)
    crop = rgb_np[y1:y2, x1:x2]
    if crop.size == 0:
        return np.zeros((size, size, 3), dtype=np.uint8)
    return cv2.resize(crop, (size, size))


def draw_label(img: np.ndarray, text: str,
               fg=(255, 255, 255), bg=(0, 0, 0)) -> np.ndarray:
    """이미지 좌상단에 텍스트 레이블 추가 (한글 지원)"""
    return _put_text(img.copy(), text, (2, 3), size=12, color=fg, shadow=True)


def vstack_images(imgs: list, pad: int = PAD,
                  bg: tuple = BG_COLOR) -> np.ndarray:
    """동일 너비 이미지 리스트를 세로로 합치기"""
    w = imgs[0].shape[1]
    sep = np.full((pad, w, 3), bg, dtype=np.uint8)
    rows = []
    for im in imgs:
        rows.append(im)
        rows.append(sep)
    return np.vstack(rows[:-1])


def hstack_images(imgs: list, pad: int = PAD,
                  bg: tuple = BG_COLOR) -> np.ndarray:
    """동일 높이 이미지 리스트를 가로로 합치기"""
    h = imgs[0].shape[0]
    sep = np.full((h, pad, 3), bg, dtype=np.uint8)
    cols = []
    for im in imgs:
        cols.append(im)
        cols.append(sep)
    return np.hstack(cols[:-1])


def make_grid(cell_imgs: list, ncols: int,
              cell_h: int, cell_w: int,
              pad: int = PAD, bg: tuple = BG_COLOR) -> np.ndarray:
    """이미지 리스트 → 그리드 (ncols 열)"""
    n = len(cell_imgs)
    nrows = (n + ncols - 1) // ncols
    gh = nrows * cell_h + (nrows + 1) * pad
    gw = ncols * cell_w + (ncols + 1) * pad
    grid = np.full((gh, gw, 3), bg, dtype=np.uint8)
    for i, img in enumerate(cell_imgs):
        r, c = divmod(i, ncols)
        y = pad + r * (cell_h + pad)
        x = pad + c * (cell_w + pad)
        grid[y:y + cell_h, x:x + cell_w] = img
    return grid


def add_col_header(grid: np.ndarray, text: str,
                   fg=(200, 200, 200)) -> np.ndarray:
    """그리드 상단에 타이틀 바 추가 (한글 지원)"""
    bar = np.full((HDR_H, grid.shape[1], 3), (50, 50, 50), dtype=np.uint8)
    bar = _put_text(bar, text, (4, 4), size=13, color=fg, shadow=False)
    return np.vstack([bar, grid])


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
        checkpoint_dir=ISM_DIR,      # dinov2_vitl14_pretrain.pth 위치
        patch_size=14,
    )
    dinov2.model = dinov2.model.to(device)
    dinov2.model.device = device

    return sam_gen, dinov2


# ══════════════════════════════════════════════════════════════════════════
# 템플릿 로드 & 임베딩
# ══════════════════════════════════════════════════════════════════════════

def load_templates(template_dir: str, device):
    """
    {template_dir}/rgb_{i}.png + mask_{i}.png 로드 후
    CropResizePad(224) 처리 → [T, 3, 224, 224] 텐서와
    시각화용 원본 RGB 이미지 리스트 반환
    """
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

    processor    = CropResizePad(224)
    templates_224 = processor(images=templates, boxes=boxes_t).to(device)
    masks_224     = processor(images=masks,     boxes=boxes_t).to(device)

    return templates_224, masks_224, raw_imgs, n


# ══════════════════════════════════════════════════════════════════════════
# 시각화 파일 1: 전체 후보 Crop 그리드
# ══════════════════════════════════════════════════════════════════════════

def make_vis1(rgb_np, boxes, ssem_all, out_path):
    """
    모든 후보 m의 Crop 이미지를 ssem 내림차순으로 그리드에 배치
    """
    N = len(ssem_all)
    order = ssem_all.argsort(descending=True).tolist()

    cells = []
    for rank, i in enumerate(order):
        crop = box_crop(rgb_np, boxes[i], CROP_SZ)
        crop = draw_label(crop, f"#{i+1} {ssem_all[i]:.3f}")
        cells.append(crop)

    ncols = min(10, N)
    grid  = make_grid(cells, ncols=ncols,
                      cell_h=CROP_SZ, cell_w=CROP_SZ)
    grid  = add_col_header(
        grid,
        f"모든 후보 m의 Crop 이미지  (총 {N}개, ssem 높은 순 정렬) — K={K}"
    )
    Image.fromarray(grid).save(out_path)
    print(f"[SAVE] {out_path}  ({N}개 후보)")


# ══════════════════════════════════════════════════════════════════════════
# 시각화 파일 2: ssem 파이프라인 패널
# ══════════════════════════════════════════════════════════════════════════

def make_vis2(rgb_np, boxes, query_embs, tmpl_embs,
              topk_idxs, best_idx, ssem_all, raw_tmpl_imgs, out_path):
    """
    상위 TOP_N 제안에 대해 ssem 파이프라인 1행 패널:
      [Crop | CLS 임베딩] [TopK 템플릿1 | 임베딩1 | ... | 템플릿K | 임베딩K] [Tbest]
    """
    N    = len(ssem_all)
    top5 = ssem_all.argsort(descending=True)[:TOP_N].tolist()

    # ── 열 헤더 레이블 계산 ─────────────────────────────────────────────
    col_labels = (
        ["CROP", "CLS EMB"]
        + [f"T{k+1}" for k in range(K)]
        + [f"T{k+1}emb" for k in range(K)]
        + ["Tbest"]
    )

    rows = []
    for rank, prop_i in enumerate(top5):
        # 1) Crop
        crop_img = box_crop(rgb_np, boxes[prop_i], CROP_SZ)
        crop_img = draw_label(crop_img, f"m#{prop_i+1}")

        # 2) Query CLS 임베딩 히트맵
        cls_img = emb_to_heatmap(query_embs[prop_i], CROP_SZ)
        cls_img = draw_label(cls_img, "CLS emb")

        # 3) Top-K 템플릿 이미지 & 임베딩 히트맵
        topk_tmpl_imgs = []
        topk_emb_imgs  = []
        for k in range(K):
            t_idx = topk_idxs[prop_i, k].item()
            # 템플릿 원본 RGB (80x80)
            t_img = cv2.resize(raw_tmpl_imgs[t_idx], (TMPL_SZ, TMPL_SZ))
            t_img = draw_label(t_img, f"T{t_idx}")
            topk_tmpl_imgs.append(t_img)
            # 템플릿 CLS 임베딩 히트맵 (80x80)
            t_emb = emb_to_heatmap(tmpl_embs[t_idx], TMPL_SZ)
            topk_emb_imgs.append(t_emb)

        # 템플릿 이미지는 CROP_SZ 높이로 맞추기 위해 패딩
        def pad_to_height(img, h):
            if img.shape[0] == h:
                return img
            padded = np.full((h, img.shape[1], 3), BG_COLOR, dtype=np.uint8)
            padded[:img.shape[0]] = img
            return padded

        topk_tmpl_imgs = [pad_to_height(im, CROP_SZ) for im in topk_tmpl_imgs]
        topk_emb_imgs  = [pad_to_height(im, CROP_SZ) for im in topk_emb_imgs]

        # 4) Tbest 이미지
        b_idx    = best_idx[prop_i].item()
        tbest    = cv2.resize(raw_tmpl_imgs[b_idx], (CROP_SZ, CROP_SZ))
        tbest    = draw_label(tbest, f"Tbest T{b_idx}", fg=(255, 220, 50))

        # ── 행 조립 ──────────────────────────────────────────────────────
        # 섹션 구분선
        vsep = np.full((CROP_SZ, 3, 3), (80, 80, 80), dtype=np.uint8)

        row_cells = (
            [crop_img, cls_img, vsep]
            + topk_tmpl_imgs
            + [vsep]
            + topk_emb_imgs
            + [vsep, tbest]
        )
        row_img = hstack_images(row_cells, pad=PAD)

        # 행 헤더: ssem 점수
        sim_scores = []
        for k in range(K):
            t_idx = topk_idxs[prop_i, k].item()
            # 코사인 유사도를 텍스트로 계산해서 표시
            q = F.normalize(query_embs[prop_i:prop_i+1], dim=-1)
            r = F.normalize(tmpl_embs[t_idx:t_idx+1],   dim=-1)
            s = (q @ r.T).item()
            sim_scores.append(f"{s:.3f}")

        hdr_text = (
            f"Proposal #{prop_i+1}  |  "
            f"ssem={ssem_all[prop_i]:.4f}  |  "
            f"Top-{K} sim: [{', '.join(sim_scores)}]  |  "
            f"Tbest=T{b_idx}"
        )
        hdr = np.full((HDR_H, row_img.shape[1], 3), (50, 50, 80), dtype=np.uint8)
        hdr = _put_text(hdr, hdr_text, (4, 5), size=12, color=(220, 220, 255), shadow=False)
        row_with_hdr = np.vstack([hdr, row_img])
        rows.append(row_with_hdr)

    # ── 열 레이블 행 ─────────────────────────────────────────────────────
    # 전체 패널 조립
    panel = vstack_images(rows, pad=PAD * 2)

    # 최상단 타이틀
    title_h = HDR_H + 4
    title   = np.full((title_h, panel.shape[1], 3), (20, 20, 60), dtype=np.uint8)
    title   = _put_text(title,
                        f"ssem 파이프라인 — 상위 {min(TOP_N, N)}개 제안  "
                        f"[Crop | CLS 임베딩 | Top-K={K} 템플릿+임베딩 | Tbest]",
                        (6, 5), size=13, color=(180, 220, 255), shadow=False)
    panel = np.vstack([title, panel])

    Image.fromarray(panel).save(out_path)
    print(f"[SAVE] {out_path}  (상위 {min(TOP_N, N)}개 제안)")


# ══════════════════════════════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="SAM-6D ssem 스코어 시각화")
    parser.add_argument("--rgb_path",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/rgb.png")),
                        help="쿼리 RGB 이미지 경로")
    parser.add_argument("--output_dir",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/outputs")),
                        help="렌더링 결과 디렉토리 (templates/ 포함)")
    parser.add_argument("--save_dir",
                        default=os.path.normpath(os.path.join(THIS_DIR, "..", "image")),
                        help="시각화 PNG 저장 경로")
    parser.add_argument("--stability_score_thresh", type=float, default=0.85)
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] device={device}")

    # ── 모델 로드 ──────────────────────────────────────────────────────
    sam_gen, dinov2 = load_models(device)

    # ── 템플릿 로드 & 임베딩 ───────────────────────────────────────────
    template_dir = os.path.join(args.output_dir, "templates")
    templates_224, _, raw_tmpl_imgs, n_tmpl = load_templates(template_dir, device)

    print(f"[EMBED] 템플릿 CLS 임베딩 계산 중... ({n_tmpl}개)")
    tmpl_embs = dinov2.compute_features(
        templates_224, token_name="x_norm_clstoken"
    )  # [T, 1024]

    # ── SAM 마스크 제안 ────────────────────────────────────────────────
    rgb_np = np.array(Image.open(args.rgb_path).convert("RGB"))
    print(f"[SAM] 마스크 제안 생성 중... ({rgb_np.shape[1]}x{rgb_np.shape[0]})")
    with torch.inference_mode():
        det_raw = sam_gen.generate_masks(rgb_np)
    detections = Detections(det_raw)
    N = detections.masks.shape[0]
    print(f"[SAM] 검출된 후보 수: {N}개")

    # ── 쿼리 CLS 임베딩 ────────────────────────────────────────────────
    print("[EMBED] 쿼리 CLS 임베딩 계산 중...")
    detections.masks = detections.masks.to(device)
    detections.boxes = detections.boxes.to(device)
    query_embs = dinov2.forward_cls_token(rgb_np, detections)  # [N, 1024]

    # ── ssem 계산 (K=5) ────────────────────────────────────────────────
    print(f"[SCORE] ssem 계산 중 (K={K})...")
    q_norm = F.normalize(query_embs, dim=-1)           # [N, 1024]
    t_norm = F.normalize(tmpl_embs,  dim=-1)           # [T, 1024]
    sim    = q_norm @ t_norm.T                         # [N, T] 코사인 유사도
    sim    = (sim + 1) / 2                             # [0, 1] 범위로 정규화

    topk_vals, topk_idxs = torch.topk(sim, k=K, dim=-1)  # [N, K]
    ssem_all   = topk_vals.mean(dim=-1)                   # [N]
    best_idx   = sim.argmax(dim=-1)                       # [N]

    print(f"[SCORE] ssem 범위: min={ssem_all.min():.4f}  max={ssem_all.max():.4f}")

    # ── 시각화 저장 ────────────────────────────────────────────────────
    path1 = os.path.join(args.save_dir, "vis_1_all_proposals.png")
    path2 = os.path.join(args.save_dir, "vis_2_ssem_pipeline.png")

    make_vis1(rgb_np, detections.boxes.cpu(), ssem_all.cpu(), path1)
    make_vis2(rgb_np, detections.boxes.cpu(), query_embs.cpu(),
              tmpl_embs.cpu(), topk_idxs.cpu(), best_idx.cpu(),
              ssem_all.cpu(), raw_tmpl_imgs, path2)

    print(f"\n=== 완료 ===")
    print(f"  후보 수       : {N}개")
    print(f"  템플릿 수     : {n_tmpl}개")
    print(f"  K             : {K}")
    print(f"  vis_1 (전체 Crop 그리드)  → {path1}")
    print(f"  vis_2 (ssem 파이프라인)   → {path2}")


if __name__ == "__main__":
    main()
