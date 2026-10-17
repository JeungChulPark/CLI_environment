#!/usr/bin/env python3
"""make_gt_box_sam.py — 인식 정확 가정 조건의 FM-Fusion 검출: 데이터셋 정답 상자 → SAM ViT-H 마스크 → prediction 폴더.

제안 방법의 같은 조건(run_compare.py --gt-boxes: 정답 상자 → MobileSAM)과 같은 상자를 쓴다 —
scene_gt_info 의 bbox_visib, visib_fract >= 0.1 (run_compare.gt_boxes_of 와 같은 규칙). 분할기만 각 방법의 것.
라벨은 YCB 이름, 점수 1.0. 저장 형식은 FM-Fusion 원래 스크립트(save_mask_data)와 같다.

    BENCH_TABLE=objpose/rt/bop/ycbm_objects.json ~/DeepLearning/third_party/gsam_venv/bin/python \
        objpose/tools/baselines/fmfusion/make_gt_box_sam.py --data ~/DeepLearning/Datasets/fmfusion_ycbm_260929/scans --gap 2
"""
from __future__ import annotations

import argparse, json, os, sys, types
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[4]
GSAM = Path.home() / "DeepLearning/third_party/Grounded-Segment-Anything"
sys.path[:0] = [str(GSAM / "segment_anything"), str(GSAM), str(GSAM / "GroundingDINO")]
for m in ("litellm", "ram", "ram.models"):
    sys.modules[m] = types.ModuleType(m)
sys.modules["ram"].inference_ram = None
sys.modules["ram.models"].ram = None
import run_ram_ground_sam as R  # noqa: E402
from segment_anything import build_sam, SamPredictor  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--gap", type=int, default=10)
    ap.add_argument("--out-name", default="prediction_gtbox")
    a = ap.parse_args()
    tab = json.loads(Path(os.environ.get("BENCH_TABLE", REPO / "objpose/rt/bop/ycbv_objects.json")).read_text())
    bop = Path(tab["scenes_dir"])
    label = {o["obj_id"]: o["ycb"].split("_", 1)[1].replace("_", " ") for o in tab["objects"]}
    sam = SamPredictor(build_sam(checkpoint=str(GSAM / "checkpoints/sam_vit_h_4b8939.pth")).to("cuda"))
    import cv2
    for sd in sorted(p for p in a.data.iterdir() if p.is_dir()):
        gt = json.loads((bop / sd.name / "scene_gt.json").read_text())
        info = json.loads((bop / sd.name / "scene_gt_info.json").read_text())
        out = sd / a.out_name
        out.mkdir(exist_ok=True)
        n_f = n_b = 0
        for fid in sorted(gt, key=int):
            i = int(fid) - 1
            if i % a.gap:
                continue
            boxes, labs = [], []
            for r, inf in zip(gt[fid], info[fid]):
                x, y, w, h = inf.get("bbox_visib", [0, 0, 0, 0])
                if inf.get("visib_fract", 1.0) >= 0.1 and w > 0 and h > 0 and r["obj_id"] in label:
                    boxes.append([x, y, x + w, y + h]); labs.append({label[r["obj_id"]]: 1.0})
            if not boxes:
                continue
            img = cv2.cvtColor(cv2.imread(str(next((sd / "color").glob(f"frame-{i:06d}.*")))), cv2.COLOR_BGR2RGB)
            sam.set_image(img)
            bt = torch.tensor(boxes, dtype=torch.float32)
            masks, _, _ = sam.predict_torch(point_coords=None, point_labels=None,
                                            boxes=sam.transform.apply_boxes_torch(bt, img.shape[:2]).to("cuda"), multimask_output=False)
            tags = ". ".join(sorted({next(iter(l)) for l in labs}))
            R.save_mask_data(str(out), f"frame-{i:06d}", tags, tags, bt, labs, masks)
            n_f += 1; n_b += len(boxes)
        print(f"{sd.name}: {n_f} frames, {n_b} boxes", flush=True)


if __name__ == "__main__":
    main()
