"""
visualize_sgeo_score.py
-----------------------
SAM-6D 논문의 Geometric Matching Score (Sgeo) 파이프라인 시각화

논문 파이프라인:
  1. Tbest의 회전 R + depth로 계산한 translation t → 대략적 포즈 추정
  2. 포즈를 이미지에 투영 → compact 경계 상자 Bo 생성
  3. IoU(Bo, Bm) = Sgeo
  4. rvis (가시 비율) 계산 → Sgeo의 신뢰 가중치
  5. Sm = (ssem + sappe + sgeo * rvis) / (2 + rvis)

용어 설명:
  - "m의 잘라낸 포인트들의 평균 위치":
      SAM 마스크 m으로 depth 이미지를 크롭 → 역투영(backprojection)으로
      3D 좌표 변환 → XYZ 평균 = 물체의 translation t 추정값
  - "compact 경계 상자 (Bo)":
      투영된 3D 점들의 2D 최소/최대 좌표로 만든 가장 작은 AABB(축 정렬 박스)
  - "rvis (가시 비율)":
      쿼리 패치와 Tbest 패치 간 유사도 행렬에서 임계값(0.5) 이상으로
      매칭된 reference 패치 비율. 폐색(occlusion)이 심할수록 rvis가 낮아지며
      최종 공식에서 Sgeo의 가중치를 줄여 신뢰도를 보정함.

출력 파일 (visualization/image/):
  ├── vis_6_sgeo_overview.png  : 전체 이미지 위에 Bm(초록)+Bo(주황) 오버레이
  └── vis_7_sgeo_detail.png    : 후보별 상세 패널
                                 [Crop | Depth | Tbest | Bm·Bo 비교 | 점수 바]

실행 방법 (ISM 디렉토리에서):
  cd D:/LDH_ws/sam_6d/SAM-6D/Instance_Segmentation_Model
  python visualization/code/visualize_sgeo_score.py --output_dir ../Data/Example/outputs
"""

import sys
import os
import glob
import json
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

# ── 하이퍼파라미터 ─────────────────────────────────────────────────────────
K            = 5      # ssem top-K
PATCH_GRID   = 16     # 224 / 14
RVIS_THRED   = 0.5    # visible ratio 임계값 (ISM_sam.yaml 기준)

# ── 시각화 상수 ───────────────────────────────────────────────────────────
CROP_SZ  = 200
PAD      = 6
HDR_H    = 26
TOP_N    = 5
BG_COLOR = (30, 30, 30)

# ── 한글 폰트 ──────────────────────────────────────────────────────────────
_FONT_PATH  = "C:/Windows/Fonts/malgun.ttf"
_FONT_CACHE = {}


def _font(size=13):
    if size not in _FONT_CACHE:
        try:
            _FONT_CACHE[size] = ImageFont.truetype(_FONT_PATH, size)
        except Exception:
            _FONT_CACHE[size] = ImageFont.load_default()
    return _FONT_CACHE[size]


def put_text(img, text, pos=(2, 4), size=13, color=(255, 255, 255), shadow=True):
    pil = Image.fromarray(img.astype(np.uint8))
    d   = ImageDraw.Draw(pil)
    f   = _font(size)
    if shadow:
        d.text((pos[0]+1, pos[1]+1), text, font=f, fill=(0, 0, 0))
    d.text(pos, text, font=f, fill=color)
    return np.array(pil)


def make_bar(width, text, height=HDR_H, bg=(50, 50, 50), size=13):
    bar = np.full((height, width, 3), bg, dtype=np.uint8)
    return put_text(bar, text, (4, 5), size=size, color=(220, 220, 200), shadow=False)


def hstack(imgs, pad=PAD, bg=BG_COLOR):
    h   = imgs[0].shape[0]
    sep = np.full((h, pad, 3), bg, dtype=np.uint8)
    out = []
    for im in imgs:
        out.append(im)
        out.append(sep)
    return np.hstack(out[:-1])


def vstack(imgs, pad=PAD, bg=BG_COLOR):
    w   = imgs[0].shape[1]
    sep = np.full((pad, w, 3), bg, dtype=np.uint8)
    out = []
    for im in imgs:
        out.append(im)
        out.append(sep)
    return np.vstack(out[:-1])


def box_crop(rgb, box, size):
    x1, y1, x2, y2 = [int(v) for v in box]
    x1 = max(0, x1); y1 = max(0, y1)
    x2 = min(rgb.shape[1], x2); y2 = min(rgb.shape[0], y2)
    c  = rgb[y1:y2, x1:x2]
    if c.size == 0:
        return np.zeros((size, size, 3), dtype=np.uint8)
    return cv2.resize(c, (size, size))


# ══════════════════════════════════════════════════════════════════════════
# 카메라 / 기하학 유틸
# ══════════════════════════════════════════════════════════════════════════

def load_camera(camera_json_path):
    """camera.json → K (3×3), depth_scale"""
    with open(camera_json_path) as f:
        data = json.load(f)
    k    = data["cam_K"]
    K    = np.array(k, dtype=np.float32).reshape(3, 3)
    ds   = float(data.get("depth_scale", 1.0))
    return K, ds


def compute_translation(depth_uint16, mask_np, K, depth_scale):
    """
    마스크된 depth로 물체의 3D 평균 위치 계산 (translation t, 단위: meters)
    = "m의 잘라낸 포인트들의 평균 위치"
    """
    H, W = depth_uint16.shape
    u, v = np.meshgrid(np.arange(W, dtype=np.float32),
                       np.arange(H, dtype=np.float32))
    Z    = depth_uint16.astype(np.float32) * depth_scale / 1000.0   # → meters
    X    = (u - K[0, 2]) * Z / K[0, 0]
    Y    = (v - K[1, 2]) * Z / K[1, 1]

    valid = (Z > 0) & (mask_np > 0)
    if valid.sum() == 0:
        return np.zeros(3, dtype=np.float32)

    t = np.array([X[valid].mean(), Y[valid].mean(), Z[valid].mean()],
                 dtype=np.float32)
    return t


def compute_bo(template_dir, tbest_idx, R, t_query, K, img_shape):
    """
    Tbest 템플릿 XYZ + 추정 포즈 → 이미지 투영 → compact 경계 상자 Bo

    template_dir : 렌더링 결과 폴더 (xyz_*.npy, mask_*.png 포함)
    tbest_idx    : 선택된 템플릿 인덱스
    R            : obj_poses[tbest_idx][:3,:3]  (object→camera 회전)
    t_query      : compute_translation()의 결과 [3] (meters)
    K            : 카메라 내부 파라미터 (3×3)
    img_shape    : (H, W) – 이미지 크기 (clamp 용도)
    """
    xyz_path  = os.path.join(template_dir, f"xyz_{tbest_idx}.npy")
    mask_path = os.path.join(template_dir, f"mask_{tbest_idx}.png")

    xyz  = np.load(xyz_path).astype(np.float32)         # [512,512,3] mm
    mask = np.array(Image.open(mask_path))
    valid = mask > 0

    pts_mm = xyz[valid]                                  # [N, 3] mm
    pts_m  = pts_mm / 1000.0                             # mm → meters

    # 1) 물체 포인트를 Tbest 회전으로 camera 방향에 맞게 회전
    pts_r  = (R @ pts_m.T).T                             # [N, 3] meters

    # 2) 점군 중심을 원점으로 이동 후, query translation 적용
    pts_r  = pts_r - pts_r.mean(axis=0)                  # re-center
    pts_c  = pts_r + t_query                             # [N, 3] meters (query camera)

    # 3) 2D 투영
    valid_z = pts_c[:, 2] > 0
    if valid_z.sum() == 0:
        return None, None

    pts_f = pts_c[valid_z]
    u_f   = pts_f[:, 0] / pts_f[:, 2] * K[0, 0] + K[0, 2]
    v_f   = pts_f[:, 1] / pts_f[:, 2] * K[1, 1] + K[1, 2]

    H, W = img_shape
    u_f   = np.clip(u_f, 0, W - 1)
    v_f   = np.clip(v_f, 0, H - 1)

    uv_pts = np.stack([u_f, v_f], axis=1)               # [N, 2] projected 2D
    bo     = np.array([u_f.min(), v_f.min(), u_f.max(), v_f.max()],
                      dtype=np.float32)
    return bo, uv_pts


def compute_iou(bo, bm):
    """
    bo, bm : [x1, y1, x2, y2]
    returns: IoU ∈ [0, 1]
    """
    xi1 = max(bo[0], bm[0]); yi1 = max(bo[1], bm[1])
    xi2 = min(bo[2], bm[2]); yi2 = min(bo[3], bm[3])
    inter_w = max(0.0, xi2 - xi1)
    inter_h = max(0.0, yi2 - yi1)
    inter   = inter_w * inter_h
    if inter == 0:
        return 0.0
    area_bo = (bo[2] - bo[0]) * (bo[3] - bo[1])
    area_bm = (bm[2] - bm[0]) * (bm[3] - bm[1])
    union   = area_bo + area_bm - inter
    return float(inter / (union + 1e-8))


# ══════════════════════════════════════════════════════════════════════════
# 모델 / 템플릿 로드 (sappe와 동일 구조)
# ══════════════════════════════════════════════════════════════════════════

def load_models(device):
    print("[LOAD] SAM vit_h ...")
    sam = load_sam("vit_h", ISM_DIR)
    sam.to(device=device)
    sam_gen = CustomSamAutomaticMaskGenerator(
        sam=sam, stability_score_thresh=0.85, segmentor_width_size=640)

    print("[LOAD] DINOv2 vitl14 ...")
    dinov2 = CustomDINOv2(
        model_name="dinov2_vitl14", token_name="x_norm_clstoken",
        image_size=224, chunk_size=16, descriptor_width_size=640,
        checkpoint_dir=ISM_DIR, patch_size=14)
    dinov2.model = dinov2.model.to(device)
    dinov2.model.device = device
    return sam_gen, dinov2


def load_templates(template_dir, device):
    npy_files = sorted(glob.glob(os.path.join(template_dir, "*.npy")))
    if not npy_files:
        raise FileNotFoundError(f"템플릿 없음: {template_dir}")
    n = len(npy_files)
    print(f"[LOAD] 템플릿 {n}개")

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

    templates = torch.stack(templates_list).permute(0, 3, 1, 2)
    masks     = torch.stack(masks_list).permute(0, 3, 1, 2)
    boxes_t   = torch.tensor(np.array(boxes))
    proc      = CropResizePad(224)
    t224 = proc(images=templates, boxes=boxes_t).to(device)
    m224 = proc(images=masks,     boxes=boxes_t).to(device)
    return t224, m224, raw_imgs, n


# ══════════════════════════════════════════════════════════════════════════
# 점수 바 시각화 유틸
# ══════════════════════════════════════════════════════════════════════════

def score_bar_img(scores_dict, width=220, row_h=22, font_size=12):
    """
    scores_dict = {'ssem': 0.79, 'sappe': 0.38, ...}
    → 점수 바 이미지 반환
    """
    colors = {
        'ssem':  (100, 180, 255),
        'sappe': (100, 255, 160),
        'sgeo':  (255, 160, 60),
        'rvis':  (200, 120, 255),
        'Sm':    (255, 220, 50),
    }
    h   = row_h * len(scores_dict) + PAD * 2
    img = np.full((h, width, 3), BG_COLOR, dtype=np.uint8)

    for idx, (name, val) in enumerate(scores_dict.items()):
        y   = PAD + idx * row_h
        bar_w = int(val * (width - 80))
        bar_w = max(0, min(bar_w, width - 80))
        col = colors.get(name, (180, 180, 180))
        cv2.rectangle(img, (70, y + 3), (70 + bar_w, y + row_h - 3), col, -1)
        label = f"{name}: {val:.3f}"
        img = put_text(img, label, (2, y + 4), size=font_size,
                       color=col, shadow=True)
    return img


# ══════════════════════════════════════════════════════════════════════════
# vis_6: 전체 이미지 오버레이
# ══════════════════════════════════════════════════════════════════════════

def make_vis6(rgb_np, all_bm_boxes, top5_idxs, bo_boxes, sgeo_scores, ssem_all, out_path):
    """
    전체 RGB 위에:
    - 모든 SAM 후보 Bm: 얇은 흰 테두리
    - 상위 5개: Bm (초록 굵은), Bo (주황 굵은)
    """
    canvas = rgb_np.copy()
    H, W   = canvas.shape[:2]

    # 모든 SAM 제안: 얇은 흰 테두리
    for box in all_bm_boxes:
        x1, y1, x2, y2 = [int(v) for v in box]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (180, 180, 180), 1)

    # 상위 5개 후보
    for rank, prop_i in enumerate(top5_idxs):
        bm = all_bm_boxes[prop_i]
        bo = bo_boxes[prop_i]

        # Bm: 초록 굵은 박스
        x1, y1, x2, y2 = [int(v) for v in bm]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (50, 220, 80), 2)
        canvas = put_text(canvas, f"Bm m#{prop_i+1}",
                          (x1, max(0, y1-16)), size=11, color=(50, 220, 80))

        # Bo: 주황 굵은 박스
        if bo is not None:
            bx1, by1, bx2, by2 = [int(v) for v in bo]
            cv2.rectangle(canvas, (bx1, by1), (bx2, by2), (255, 140, 30), 2)
            canvas = put_text(canvas, f"Bo  sgeo={sgeo_scores[prop_i]:.3f}",
                              (bx1, min(H-14, by2+2)), size=11, color=(255, 140, 30))

    # 범례
    legend_y = H - 46
    canvas = put_text(canvas, "■ Bm (SAM 제안 박스)",    (8, legend_y),      size=12, color=(50,  220, 80))
    canvas = put_text(canvas, "■ Bo (투영된 예측 박스)", (8, legend_y + 18), size=12, color=(255, 140, 30))

    title = make_bar(canvas.shape[1],
                     f"Sgeo 개요 — Bm (초록, SAM 제안)  vs  Bo (주황, 포즈 투영)  |  상위 {TOP_N}개 표시",
                     height=HDR_H + 4, bg=(20, 20, 60), size=14)
    Image.fromarray(np.vstack([title, canvas])).save(out_path)
    print(f"[SAVE] {out_path}")


# ══════════════════════════════════════════════════════════════════════════
# vis_7: 후보별 상세 패널
# ══════════════════════════════════════════════════════════════════════════

def make_vis7(rgb_np, depth_uint16, bm_boxes, bo_boxes, uv_pts_all,
              raw_tmpl_imgs, ssem_all, sappe_scores, sgeo_scores, rvis_scores,
              sm_scores, best_idx, out_path):
    """
    각 상위 후보 1행:
    [Query Crop | Depth Heatmap | Tbest | Bm·Bo 비교 | 점수 바]
    """
    top5 = ssem_all.argsort(descending=True)[:TOP_N].tolist()

    # depth → 컬러맵
    depth_f = depth_uint16.astype(np.float32)
    d_norm  = (depth_f - depth_f[depth_f > 0].min()) / \
              (depth_f.max() - depth_f[depth_f > 0].min() + 1e-8)
    depth_colored = (cm.plasma(d_norm)[:, :, :3] * 255).astype(np.uint8)

    rows = []
    for prop_i in top5:
        t_idx = best_idx[prop_i].item()

        # ── 열 1: Query Crop ──────────────────────────────────────────
        crop  = box_crop(rgb_np, bm_boxes[prop_i], CROP_SZ)
        crop  = put_text(crop, f"쿼리 m#{prop_i+1}", (2, 4), size=12,
                         color=(255, 220, 50))

        # ── 열 2: Depth 히트맵 Crop ───────────────────────────────────
        dep   = box_crop(depth_colored, bm_boxes[prop_i], CROP_SZ)
        dep   = put_text(dep, "Depth", (2, 4), size=12, color=(255, 255, 255))

        # ── 열 3: Tbest 템플릿 ────────────────────────────────────────
        tmpl  = cv2.resize(raw_tmpl_imgs[t_idx], (CROP_SZ, CROP_SZ))
        tmpl  = put_text(tmpl, f"Tbest T{t_idx}", (2, 4), size=12,
                         color=(100, 255, 100))

        # ── 열 4: Bm + Bo 비교 이미지 ─────────────────────────────────
        # Bm 영역을 중심으로 패딩한 뒤, Bm(초록)·Bo(주황)·투영점(점) 표시
        bm    = bm_boxes[prop_i]
        bo    = bo_boxes[prop_i]
        cx    = int((bm[0] + bm[2]) / 2)
        cy    = int((bm[1] + bm[3]) / 2)
        pad_s = max(int((bm[2]-bm[0]) * 0.6), int((bm[3]-bm[1]) * 0.6), 30)
        H_i, W_i = rgb_np.shape[:2]
        rx1 = max(0, cx - CROP_SZ // 2 - pad_s)
        ry1 = max(0, cy - CROP_SZ // 2 - pad_s)
        rx2 = min(W_i, rx1 + CROP_SZ + pad_s * 2)
        ry2 = min(H_i, ry1 + CROP_SZ + pad_s * 2)
        region = rgb_np[ry1:ry2, rx1:rx2].copy()

        def to_rel(box):
            """전체 좌표 → region 좌표"""
            return (int(box[0]-rx1), int(box[1]-ry1),
                    int(box[2]-rx1), int(box[3]-ry1))

        # 투영 점 (점군, 파란색)
        uv_pts = uv_pts_all[prop_i]
        if uv_pts is not None:
            for u_p, v_p in uv_pts[::max(1, len(uv_pts)//400)]:
                rx, ry = int(u_p - rx1), int(v_p - ry1)
                if 0 <= rx < region.shape[1] and 0 <= ry < region.shape[0]:
                    cv2.circle(region, (rx, ry), 1, (80, 160, 255), -1)

        # Bm: 초록
        bm_r = to_rel(bm)
        cv2.rectangle(region, (bm_r[0], bm_r[1]), (bm_r[2], bm_r[3]),
                      (50, 220, 80), 2)
        # Bo: 주황
        if bo is not None:
            bo_r = to_rel(bo)
            cv2.rectangle(region, (bo_r[0], bo_r[1]), (bo_r[2], bo_r[3]),
                          (255, 140, 30), 2)

        compare = cv2.resize(region, (CROP_SZ, CROP_SZ))
        compare = put_text(compare, f"Bm(초록) Bo(주황)", (2, 4), size=11,
                           color=(255, 255, 255))
        compare = put_text(compare, f"sgeo={sgeo_scores[prop_i]:.3f}",
                           (2, CROP_SZ - 18), size=12, color=(255, 200, 60))

        # ── 열 5: 점수 바 ─────────────────────────────────────────────
        scores = {
            'ssem':  float(ssem_all[prop_i]),
            'sappe': float(sappe_scores[prop_i]),
            'sgeo':  float(sgeo_scores[prop_i]),
            'rvis':  float(rvis_scores[prop_i]),
            'Sm':    float(sm_scores[prop_i]),
        }
        bar = score_bar_img(scores, width=CROP_SZ + 20, row_h=22)
        bar = cv2.resize(bar, (CROP_SZ + 20, CROP_SZ))

        # ── 행 조합 ───────────────────────────────────────────────────
        vsep = np.full((CROP_SZ, 3, 3), (80, 80, 80), dtype=np.uint8)
        row  = hstack([crop, dep, vsep, tmpl, compare, vsep, bar], pad=PAD)

        hdr  = make_bar(row.shape[1],
                        f"m#{prop_i+1} ↔ Tbest T{t_idx}  |  "
                        f"ssem={ssem_all[prop_i]:.3f}  sappe={sappe_scores[prop_i]:.3f}  "
                        f"sgeo={sgeo_scores[prop_i]:.3f}  rvis={rvis_scores[prop_i]:.3f}  "
                        f"→  Sm={sm_scores[prop_i]:.4f}",
                        bg=(50, 50, 80))
        rows.append(np.vstack([hdr, row]))

    panel = vstack(rows, pad=PAD * 2)
    title = make_bar(panel.shape[1],
                     "Sgeo 상세  [Crop | Depth | Tbest | Bm·Bo·투영점 비교 | 점수 바 (ssem/sappe/sgeo/rvis/Sm)]",
                     height=HDR_H + 4, bg=(20, 20, 60), size=14)
    Image.fromarray(np.vstack([title, panel])).save(out_path)
    print(f"[SAVE] {out_path}")


# ══════════════════════════════════════════════════════════════════════════
# 메인
# ══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="SAM-6D Sgeo 시각화")
    parser.add_argument("--rgb_path",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/rgb.png")))
    parser.add_argument("--depth_path",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/depth.png")))
    parser.add_argument("--camera_path",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/camera.json")))
    parser.add_argument("--output_dir",
                        default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/outputs")))
    parser.add_argument("--save_dir",
                        default=os.path.normpath(os.path.join(THIS_DIR, "..", "image")))
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[INFO] device={device}")

    # ── 카메라 파라미터 로드 ────────────────────────────────────────
    K_mat, depth_scale = load_camera(args.camera_path)
    print(f"[LOAD] K={K_mat[0,0]:.1f}/{K_mat[1,1]:.1f}  cx={K_mat[0,2]:.1f}  depth_scale={depth_scale}")

    # ── 이미지 로드 ─────────────────────────────────────────────────
    rgb_np     = np.array(Image.open(args.rgb_path).convert("RGB"))
    depth_u16  = np.array(Image.open(args.depth_path))
    print(f"[LOAD] RGB {rgb_np.shape}  Depth {depth_u16.shape} ({depth_u16.dtype})")

    # ── 템플릿 포즈 로드 ─────────────────────────────────────────────
    pose_path  = os.path.join(ISM_DIR, "utils/poses/predefined_poses/obj_poses_level0.npy")
    obj_poses  = np.load(pose_path)                 # [42, 4, 4] world→camera
    print(f"[LOAD] obj_poses_level0 {obj_poses.shape}")

    # ── 모델 로드 ───────────────────────────────────────────────────
    sam_gen, dinov2 = load_models(device)

    # ── 템플릿 로드 및 임베딩 ────────────────────────────────────────
    template_dir = os.path.join(args.output_dir, "templates")
    t224, m224, raw_tmpl_imgs, n_tmpl = load_templates(template_dir, device)

    print(f"[EMBED] 템플릿 CLS ({n_tmpl}개)...")
    tmpl_cls   = dinov2.compute_features(t224, token_name="x_norm_clstoken")

    print(f"[EMBED] 템플릿 패치 ({n_tmpl}개)...")
    tmpl_patch = dinov2.compute_masked_patch_feature(t224, m224.squeeze(1))  # [T,256,1024]

    # ── SAM 마스크 제안 ──────────────────────────────────────────────
    print(f"[SAM] 마스크 생성 중...")
    with torch.inference_mode():
        det_raw = sam_gen.generate_masks(rgb_np)
    detections = Detections(det_raw)
    N = detections.masks.shape[0]
    print(f"[SAM] 후보 {N}개")

    detections.masks = detections.masks.to(device)
    detections.boxes = detections.boxes.to(device)

    # ── ssem → Tbest ────────────────────────────────────────────────
    print("[EMBED] 쿼리 CLS...")
    query_cls  = dinov2.forward_cls_token(rgb_np, detections)          # [N,1024]
    q_n = F.normalize(query_cls,  dim=-1)
    t_n = F.normalize(tmpl_cls,   dim=-1)
    sim_cls = (q_n @ t_n.T + 1) / 2                                    # [N,T]
    topk_v, _ = torch.topk(sim_cls, k=K, dim=-1)
    ssem_all   = topk_v.mean(dim=-1)                                    # [N]
    best_idx   = sim_cls.argmax(dim=-1)                                 # [N]

    # ── sappe ────────────────────────────────────────────────────────
    print("[EMBED] 쿼리 패치...")
    query_patch = dinov2.forward_patch_tokens(rgb_np, detections)       # [N,256,1024]

    sappe_list = []
    for i in range(N):
        q_p = query_patch[i]; t_p = tmpl_patch[best_idx[i].item()]
        sm  = (q_p @ t_p.T).max(dim=-1).values
        sappe_list.append(sm.mean().item())
    sappe_scores = torch.tensor(sappe_list)

    # ── rvis ─────────────────────────────────────────────────────────
    print("[SCORE] rvis 계산 중...")
    ref_patch_per_q = tmpl_patch[best_idx]                              # [N,256,1024]
    sim_rvis = torch.matmul(query_patch,
                            ref_patch_per_q.permute(0, 2, 1))          # [N,256,256]
    sim_rvis = sim_rvis.max(dim=1).values                               # [N,256] max over query
    valid_p  = (sim_rvis != 0).sum(dim=1).float() + 1e-6               # [N]
    filt_p   = (sim_rvis > RVIS_THRED).sum(dim=1).float()              # [N]
    rvis_scores = filt_p / valid_p                                      # [N]

    # ── sgeo: Bo 생성 → IoU ─────────────────────────────────────────
    print("[SCORE] sgeo (Bo 생성 + IoU) 계산 중...")
    boxes_cpu = detections.boxes.cpu().numpy()
    masks_cpu = detections.masks.cpu().numpy()
    img_shape = (rgb_np.shape[0], rgb_np.shape[1])

    bo_boxes  = []
    uv_pts_all = []
    sgeo_list  = []

    for i in range(N):
        t_idx = best_idx[i].item()
        R     = obj_poses[t_idx][:3, :3].astype(np.float32)

        # translation: SAM 마스크 m으로 depth에서 3D 평균 위치 계산
        mask_i = masks_cpu[i].astype(np.float32)
        t_q    = compute_translation(depth_u16, mask_i, K_mat, depth_scale)

        # Bo: Tbest XYZ 점군을 포즈 적용 후 투영
        bo, uv_pts = compute_bo(template_dir, t_idx, R, t_q, K_mat, img_shape)
        bo_boxes.append(bo)
        uv_pts_all.append(uv_pts)

        bm  = boxes_cpu[i].astype(np.float32)       # [x1, y1, x2, y2]
        iou = compute_iou(bo, bm) if bo is not None else 0.0
        sgeo_list.append(iou)

    sgeo_scores = torch.tensor(sgeo_list)

    # ── 최종 점수 Sm ────────────────────────────────────────────────
    sm_scores = (ssem_all.cpu() + sappe_scores + sgeo_scores * rvis_scores.cpu()) \
                / (2.0 + rvis_scores.cpu())

    top5_idxs = ssem_all.argsort(descending=True)[:TOP_N].tolist()

    print(f"\n[SCORE] 상위 5개 (ssem 기준):")
    for rank, i in enumerate(top5_idxs):
        print(f"  #{rank+1} m{i+1:3d} | ssem={ssem_all[i]:.3f}  sappe={sappe_scores[i]:.3f}  "
              f"sgeo={sgeo_scores[i]:.3f}  rvis={rvis_scores[i]:.3f}  → Sm={sm_scores[i]:.4f}")

    # ── 시각화 저장 ─────────────────────────────────────────────────
    path6 = os.path.join(args.save_dir, "vis_6_sgeo_overview.png")
    path7 = os.path.join(args.save_dir, "vis_7_sgeo_detail.png")

    make_vis6(rgb_np, boxes_cpu, top5_idxs, bo_boxes,
              sgeo_scores.numpy(), ssem_all.cpu().numpy(), path6)

    make_vis7(rgb_np, depth_u16, boxes_cpu, bo_boxes, uv_pts_all,
              raw_tmpl_imgs, ssem_all.cpu(), sappe_scores, sgeo_scores,
              rvis_scores.cpu(), sm_scores, best_idx.cpu(), path7)

    print(f"\n=== 완료 ===")
    print(f"  vis_6 (sgeo 오버뷰)  → {path6}")
    print(f"  vis_7 (sgeo 상세)    → {path7}")


if __name__ == "__main__":
    main()
