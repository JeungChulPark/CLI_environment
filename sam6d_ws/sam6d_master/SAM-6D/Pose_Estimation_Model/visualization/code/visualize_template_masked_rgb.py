"""
visualize_template_masked_rgb.py

PEM의 ViT Base 입력 직전 이미지를 시각화하는 스크립트.
즉, rendering된 객체 template에서 배경이 제거(마스킹)되고
224x224로 리사이즈된 RGB 이미지를 저장한다.

파이프라인 (run_inference_custom.py의 _get_template 함수와 동일):
  1. rgb_<i>.png  + mask_<i>.png 로드
  2. mask 기반 bounding box 추출 (정사각형 crop)
  3. RGB를 bbox 영역으로 crop
  4. 마스크 적용 → 배경 픽셀을 0으로 설정
  5. 224x224로 리사이즈
  6. PNG로 저장 (정규화 전 uint8 상태 = ViT 입력 직전 시각적 표현)

사용법:
  python visualize_template_masked_rgb.py \\
      --template_dir <template 폴더 경로> \\
      --output_dir   <저장할 폴더 경로> \\
      [--img_size 224] \\
      [--max_views 42] \\
      [--save_grid]
"""

import os
import sys
import glob
import argparse
import numpy as np
import cv2
from PIL import Image

# ── 경로 설정: data_utils.py를 import하기 위해 utils 폴더를 sys.path에 추가 ──
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))                # .../visualization/code
PEM_DIR     = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..'))    # .../Pose_Estimation_Model
UTILS_DIR   = os.path.join(PEM_DIR, 'utils')
sys.path.insert(0, UTILS_DIR)

from data_utils import load_im, get_bbox  # SAM-6D 내부 유틸


# ─────────────────────────────────────────────
# 핵심 전처리 함수 (run_inference_custom._get_template 와 동일 로직)
# ─────────────────────────────────────────────
def preprocess_template(rgb_path: str, mask_path: str, img_size: int = 224) -> np.ndarray:
    """
    단일 template의 마스킹+리사이즈 처리.

    Returns
    -------
    np.ndarray  shape=(img_size, img_size, 3), dtype=uint8, BGR
        배경이 0으로 설정되고 img_size x img_size 로 리사이즈된 RGB 이미지.
        (OpenCV 저장을 위해 BGR 순서로 반환)
    """
    # 1) 로드
    rgb  = load_im(rgb_path).astype(np.uint8)           # (H, W, 3) RGB
    mask = load_im(mask_path).astype(np.uint8) == 255   # (H, W) bool

    # 2) bounding box 추출 (정사각형, 이미지 경계 클램핑 포함)
    bbox = get_bbox(mask)
    y1, y2, x1, x2 = bbox

    # 3) crop
    mask_crop = mask[y1:y2, x1:x2]
    #   load_im은 RGB로 반환하므로 BGR 변환 후 crop
    rgb_crop  = rgb[:, :, ::-1][y1:y2, x1:x2, :]       # BGR

    # 4) 마스크 적용 (배경 → 0)
    rgb_masked = rgb_crop * (mask_crop[:, :, None] > 0).astype(np.uint8)

    # 5) 224x224 리사이즈
    rgb_resized = cv2.resize(rgb_masked, (img_size, img_size),
                             interpolation=cv2.INTER_LINEAR)

    return rgb_resized  # BGR uint8


# ─────────────────────────────────────────────
# 그리드 이미지 생성
# ─────────────────────────────────────────────
def make_grid(images: list, ncols: int = 7, pad: int = 4,
              bg_color=(30, 30, 30)) -> np.ndarray:
    """
    images : list of (H, W, 3) uint8 배열 (BGR)
    """
    if not images:
        return np.zeros((1, 1, 3), dtype=np.uint8)

    h, w = images[0].shape[:2]
    nrows = (len(images) + ncols - 1) // ncols

    canvas_h = nrows * h + (nrows + 1) * pad
    canvas_w = ncols * w + (ncols + 1) * pad
    canvas = np.full((canvas_h, canvas_w, 3), bg_color, dtype=np.uint8)

    for idx, img in enumerate(images):
        r = idx // ncols
        c = idx % ncols
        y0 = pad + r * (h + pad)
        x0 = pad + c * (w + pad)
        canvas[y0:y0 + h, x0:x0 + w] = img

    return canvas


# ─────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="PEM ViT 입력 직전 template 마스킹 이미지 시각화")
    parser.add_argument(
        "--template_dir",
        type=str,
        default=os.path.join(PEM_DIR, "..", "Data", "Example", "outputs", "templates"),
        help="rgb_*.png / mask_*.png 이 있는 template 폴더 경로")
    parser.add_argument(
        "--output_dir",
        type=str,
        default=os.path.join(PEM_DIR, "visualization", "image"),
        help="저장 위치")
    parser.add_argument(
        "--img_size",
        type=int,
        default=224,
        help="리사이즈 크기 (config의 img_size와 동일, 기본 224)")
    parser.add_argument(
        "--max_views",
        type=int,
        default=42,
        help="총 template 뷰 수 (기본 42)")
    parser.add_argument(
        "--save_grid",
        action="store_true",
        default=True,
        help="모든 뷰를 한 장의 그리드 이미지로 추가 저장 (기본 True)")
    parser.add_argument(
        "--no_grid",
        action="store_true",
        default=False,
        help="그리드 이미지 저장 생략")
    args = parser.parse_args()

    template_dir = os.path.abspath(args.template_dir)
    output_dir   = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # template 폴더 이름을 출력 서브폴더로 사용 (객체 구분)
    obj_name    = os.path.basename(os.path.dirname(template_dir)) \
                  if os.path.basename(template_dir) == "templates" \
                  else os.path.basename(template_dir)
    save_subdir = os.path.join(output_dir, f"template_masked_rgb_{obj_name}")
    os.makedirs(save_subdir, exist_ok=True)

    # rgb_*.png 파일 검색
    rgb_files = sorted(glob.glob(os.path.join(template_dir, "rgb_*.png")),
                       key=lambda p: int(os.path.splitext(os.path.basename(p))[0].split("_")[1]))

    if not rgb_files:
        print(f"[ERROR] rgb_*.png 파일을 찾을 수 없습니다: {template_dir}")
        sys.exit(1)

    total_views = len(rgb_files)
    # run_inference_custom.get_templates 와 동일하게 균등 샘플링
    selected_indices = [int(total_views / args.max_views * v)
                        for v in range(min(args.max_views, total_views))]
    selected_indices = sorted(set(selected_indices))

    print(f"[INFO] template 폴더  : {template_dir}")
    print(f"[INFO] 저장 폴더      : {save_subdir}")
    print(f"[INFO] 전체 뷰 수     : {total_views}")
    print(f"[INFO] 처리할 뷰 수   : {len(selected_indices)} (max_views={args.max_views})")
    print(f"[INFO] img_size       : {args.img_size}")
    print()

    saved_images = []
    failed = []

    for idx in selected_indices:
        rgb_path  = os.path.join(template_dir, f"rgb_{idx}.png")
        mask_path = os.path.join(template_dir, f"mask_{idx}.png")

        if not os.path.exists(rgb_path) or not os.path.exists(mask_path):
            print(f"  [SKIP] 파일 없음 → rgb_{idx}.png 또는 mask_{idx}.png")
            failed.append(idx)
            continue

        try:
            img_bgr = preprocess_template(rgb_path, mask_path, args.img_size)
            save_path = os.path.join(save_subdir, f"template_masked_{idx:03d}.png")
            cv2.imwrite(save_path, img_bgr)
            saved_images.append(img_bgr)
            print(f"  [SAVED] view {idx:3d}  →  {os.path.basename(save_path)}")
        except Exception as e:
            print(f"  [ERROR] view {idx}: {e}")
            failed.append(idx)

    print()
    print(f"[DONE] 저장 완료: {len(saved_images)}장  /  실패: {len(failed)}장")

    # 그리드 이미지 저장
    if args.save_grid and not args.no_grid and saved_images:
        grid = make_grid(saved_images, ncols=7, pad=4)
        grid_path = os.path.join(output_dir, f"template_masked_rgb_{obj_name}_grid.png")
        cv2.imwrite(grid_path, grid)
        print(f"[GRID]  그리드 이미지 저장 → {grid_path}")


if __name__ == "__main__":
    main()
