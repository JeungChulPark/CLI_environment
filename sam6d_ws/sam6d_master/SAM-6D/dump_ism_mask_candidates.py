#!/usr/bin/env python3
"""Dump raw ISM segmentor mask candidates, one image per candidate."""

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from skimage.feature import canny
from skimage.morphology import binary_dilation


if tuple(int(part) for part in torch.__version__.split("+", 1)[0].split(".")[:2]) >= (2, 6):
    _torch_load = torch.load

    def _torch_load_compat(*args, **kwargs):
        kwargs.setdefault("weights_only", False)
        if not torch.cuda.is_available() and kwargs.get("map_location") is None:
            kwargs["map_location"] = torch.device("cpu")
        return _torch_load(*args, **kwargs)

    torch.load = _torch_load_compat


ROOT_DIR = Path(__file__).resolve().parent
ISM_DIR = ROOT_DIR / "Instance_Segmentation_Model"
sys.path.insert(0, str(ISM_DIR))

from model.fast_sam import FastSAM  # noqa: E402
from model.sam import CustomSamAutomaticMaskGenerator, load_sam  # noqa: E402


def _load_segmentor(args, device):
    if args.segmentor_model == "fastsam":
        from omegaconf import OmegaConf

        cfg = OmegaConf.create(
            {
                "iou_threshold": args.fastsam_iou,
                "conf_threshold": args.fastsam_conf,
                "max_det": args.max_det,
            }
        )
        segmentor = FastSAM(
            checkpoint_path=args.fastsam_checkpoint,
            config=cfg,
            segmentor_width_size=args.segmentor_width_size,
            device=device,
        )
        segmentor.model.setup_model(device=device, verbose=True)
        return segmentor

    sam = load_sam(args.sam_model_type, args.sam_checkpoint_dir)
    sam.to(device=device)
    return CustomSamAutomaticMaskGenerator(
        sam=sam,
        stability_score_thresh=args.stability_score_thresh,
        segmentor_width_size=args.segmentor_width_size,
    )


def _as_numpy(value):
    if torch.is_tensor(value):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def _candidate_image(rgb, mask, box, index, alpha=0.45):
    base = rgb.copy()
    overlay = base.astype(np.float32)

    color = np.array([0, 190, 255], dtype=np.float32)
    overlay[mask] = alpha * color + (1.0 - alpha) * overlay[mask]

    edge = canny(mask.astype(float))
    edge = binary_dilation(edge, np.ones((2, 2)))
    overlay[edge] = [255, 255, 255]

    image = np.clip(overlay, 0, 255).astype(np.uint8)
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    cv2.rectangle(image, (x1, y1), (x2, y2), (30, 255, 30), 2)
    label = f"ISM {index:03d}"
    cv2.putText(
        image,
        label,
        (max(x1, 4), max(y1 - 7, 18)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        label,
        (max(x1, 4), max(y1 - 7, 18)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    return image


def _make_grid(images, columns=5, pad=8):
    if not images:
        return None
    h, w = images[0].shape[:2]
    rows = int(np.ceil(len(images) / columns))
    grid = np.zeros((rows * h + (rows - 1) * pad, columns * w + (columns - 1) * pad, 3), dtype=np.uint8)
    for idx, image in enumerate(images):
        row = idx // columns
        col = idx % columns
        y = row * (h + pad)
        x = col * (w + pad)
        grid[y : y + h, x : x + w] = image
    return grid


def dump_candidates(args):
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")
    print(f"[INFO] device={device}")
    print(f"[INFO] segmentor_model={args.segmentor_model}")
    print(f"[INFO] rgb_path={args.rgb_path}")
    print(f"[INFO] output_dir={output_dir}")

    segmentor = _load_segmentor(args, device)
    rgb = np.array(Image.open(Path(args.rgb_path).expanduser()).convert("RGB"))

    with torch.inference_mode():
        raw = segmentor.generate_masks(rgb)

    masks = _as_numpy(raw["masks"]) > 0
    boxes = _as_numpy(raw["boxes"]).astype(float)
    if args.max_save > 0:
        masks = masks[: args.max_save]
        boxes = boxes[: args.max_save]

    metadata = {
        "rgb_path": str(Path(args.rgb_path).expanduser()),
        "segmentor_model": args.segmentor_model,
        "segmentor_width_size": args.segmentor_width_size,
        "candidate_count": int(len(masks)),
        "candidates": [],
    }

    grid_items = []
    for idx, (mask, box) in enumerate(zip(masks, boxes)):
        area = int(mask.sum())
        x1, y1, x2, y2 = [float(v) for v in box]
        out_name = f"candidate_{idx:03d}_area_{area}.png"
        out_path = output_dir / out_name
        image = _candidate_image(rgb, mask, box, idx)
        Image.fromarray(image).save(out_path)

        if len(grid_items) < args.grid_limit:
            small = cv2.resize(image, (args.grid_cell_width, args.grid_cell_height), interpolation=cv2.INTER_AREA)
            grid_items.append(small)

        metadata["candidates"].append(
            {
                "index": idx,
                "image": out_name,
                "bbox_xyxy": [x1, y1, x2, y2],
                "bbox_xywh": [x1, y1, x2 - x1, y2 - y1],
                "mask_area_px": area,
            }
        )

    with open(output_dir / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)

    grid = _make_grid(grid_items, columns=args.grid_columns)
    if grid is not None:
        Image.fromarray(grid).save(output_dir / "all_candidates_grid.png")

    print(f"[DONE] candidates={len(masks)}")
    print(f"[DONE] metadata={output_dir / 'metadata.json'}")


def main():
    parser = argparse.ArgumentParser(description="Dump raw ISM mask candidates as individual images.")
    parser.add_argument("--rgb_path", default="/home/etri/ros2_ws/output/test/live_input/rgb.png")
    parser.add_argument("--output_dir", default="/home/etri/ros2_ws/output/test/ISM_n")
    parser.add_argument("--segmentor_model", default="fastsam", choices=["fastsam", "sam"])
    parser.add_argument("--segmentor_width_size", type=int, default=640)
    parser.add_argument("--cpu", action="store_true")
    parser.add_argument("--max_save", type=int, default=0, help="0 saves every candidate.")
    parser.add_argument("--grid_limit", type=int, default=80)
    parser.add_argument("--grid_columns", type=int, default=5)
    parser.add_argument("--grid_cell_width", type=int, default=256)
    parser.add_argument("--grid_cell_height", type=int, default=192)

    parser.add_argument(
        "--fastsam_checkpoint",
        default=str(ISM_DIR / "checkpoints" / "FastSAM" / "FastSAM-s.pt"),
    )
    parser.add_argument("--fastsam_iou", type=float, default=0.9)
    parser.add_argument("--fastsam_conf", type=float, default=0.05)
    parser.add_argument("--max_det", type=int, default=200)

    parser.add_argument(
        "--sam_checkpoint_dir",
        default=str(ISM_DIR / "checkpoints" / "segment-anything"),
    )
    parser.add_argument("--sam_model_type", default="vit_h")
    parser.add_argument("--stability_score_thresh", type=float, default=0.97)

    args = parser.parse_args()
    dump_candidates(args)


if __name__ == "__main__":
    main()
