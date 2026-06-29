"""
SAM Mask Proposal Visualization
--------------------------------
SAM-6D ISM 파이프라인의 첫 단계:
  RGB 이미지 -> SAM이 모든 객체 마스크를 자동 제안 -> 시각화

실행 방법 (ISM 디렉토리에서):
  cd D:/LDH_ws/sam_6d/SAM-6D/Instance_Segmentation_Model
  python visualization/code/visualize_sam_proposals.py
  python visualization/code/visualize_sam_proposals.py --rgb_path ../Data/Example/rgb.png --output_path visualization/image/output_sam_proposals.png
"""

import sys
import os
import argparse
import numpy as np
import torch
import cv2
from PIL import Image
import distinctipy
from skimage.feature import canny
from skimage.morphology import binary_dilation

# ISM 모듈 경로 추가 (visualization/code/ -> Instance_Segmentation_Model/)
ISM_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.normpath(ISM_DIR))
from model.sam import load_sam, CustomSamAutomaticMaskGenerator

# 기본 출력 경로: visualization/image/
DEFAULT_OUTPUT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "image", "output_sam_proposals.png")
)


def visualize_all_proposals(rgb: np.ndarray, masks: torch.Tensor, boxes: torch.Tensor) -> Image.Image:
    """
    SAM이 제안한 모든 마스크를 컬러로 시각화.

    Args:
        rgb   : (H, W, 3) uint8 원본 RGB 이미지
        masks : (N, H, W) bool/float 텐서 - N개 마스크
        boxes : (N, 4) 텐서 - 바운딩박스 [x1, y1, x2, y2]

    Returns:
        원본 | 마스크 시각화 좌우 병합 PIL 이미지
    """
    N = masks.shape[0]
    colors = distinctipy.get_colors(N)
    alpha = 0.45

    # 그레이스케일 베이스 위에 마스크 오버레이
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB).astype(np.float32)

    for i in range(N):
        mask_np = masks[i].cpu().numpy().astype(bool)
        if not mask_np.any():
            continue

        r = int(255 * colors[i][0])
        g = int(255 * colors[i][1])
        b = int(255 * colors[i][2])

        # 마스크 색상 블렌딩
        overlay[mask_np, 0] = alpha * r + (1 - alpha) * overlay[mask_np, 0]
        overlay[mask_np, 1] = alpha * g + (1 - alpha) * overlay[mask_np, 1]
        overlay[mask_np, 2] = alpha * b + (1 - alpha) * overlay[mask_np, 2]

        # 엣지 (흰색 테두리)
        edge = canny(mask_np.astype(float))
        edge = binary_dilation(edge, np.ones((2, 2)))
        overlay[edge] = 255

        # 바운딩박스 번호 표시
        x1, y1, x2, y2 = boxes[i].cpu().int().tolist()
        cv2.rectangle(overlay.astype(np.uint8), (x1, y1), (x2, y2),
                      (r, g, b), 1)
        cv2.putText(overlay.astype(np.uint8), str(i + 1), (x1 + 2, y1 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

    overlay = np.clip(overlay, 0, 255).astype(np.uint8)

    # 원본 | 시각화 좌우 병합
    orig_pil = Image.fromarray(rgb)
    vis_pil  = Image.fromarray(overlay)
    concat   = Image.new("RGB", (rgb.shape[1] * 2, rgb.shape[0]))
    concat.paste(orig_pil, (0, 0))
    concat.paste(vis_pil,  (rgb.shape[1], 0))

    return concat


def main():
    parser = argparse.ArgumentParser(description="SAM Mask Proposal Visualization")
    parser.add_argument("--rgb_path", default=os.path.normpath(os.path.join(ISM_DIR, "../Data/Example/rgb.png")),
                        help="입력 RGB 이미지 경로")
    parser.add_argument("--checkpoint_dir", default=os.path.normpath(ISM_DIR),
                        help="sam_vit_h_4b8939.pth 가 있는 디렉토리")
    parser.add_argument("--output_path", default=DEFAULT_OUTPUT,
                        help="저장할 시각화 이미지 경로")
    parser.add_argument("--stability_score_thresh", type=float, default=0.85,
                        help="마스크 안정성 임계값 (낮을수록 더 많은 마스크)")
    args = parser.parse_args()

    # 출력 디렉토리 자동 생성
    os.makedirs(os.path.dirname(args.output_path), exist_ok=True)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[INFO] 사용 디바이스: {device}")

    # 1. SAM 모델 로드
    print("[STEP 1] SAM vit_h 모델 로딩 중...")
    sam = load_sam("vit_h", args.checkpoint_dir)
    sam.to(device=device)

    mask_generator = CustomSamAutomaticMaskGenerator(
        sam=sam,
        stability_score_thresh=args.stability_score_thresh,
        segmentor_width_size=640,
    )
    print("[STEP 1] 모델 로드 완료")

    # 2. 이미지 로드
    print(f"[STEP 2] 이미지 로드: {args.rgb_path}")
    rgb = np.array(Image.open(args.rgb_path).convert("RGB"))
    print(f"         이미지 크기: {rgb.shape[1]}x{rgb.shape[0]}")

    # 3. SAM 마스크 제안 생성
    print("[STEP 3] SAM 마스크 제안 생성 중...")
    with torch.inference_mode():
        detections = mask_generator.generate_masks(rgb)

    masks = detections["masks"]   # (N, H, W)
    boxes = detections["boxes"]   # (N, 4)
    print(f"[STEP 3] 검출된 마스크 수: {masks.shape[0]}개")

    # 4. 시각화 및 저장
    print("[STEP 4] 시각화 생성 중...")
    result = visualize_all_proposals(rgb, masks, boxes)
    result.save(args.output_path)
    print(f"[STEP 4] 저장 완료: {args.output_path}")
    print(f"\n결과 요약")
    print(f"  입력 이미지  : {args.rgb_path}")
    print(f"  검출 마스크  : {masks.shape[0]}개")
    print(f"  출력 이미지  : {args.output_path}  (원본 | SAM 제안 마스크)")


if __name__ == "__main__":
    main()
