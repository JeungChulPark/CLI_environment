#!/usr/bin/env python3
"""detect_gsam_real.py — FM-Fusion 의 검출 단계(GroundingDINO → NMS → SAM)를 직접 녹화한 세션에 돌린다 (detect_gsam_prompts.py 의 실제 주행 판).

정답이 없으므로 '자체 검출' 조건만 있다. 문구는 제안 방법(YOLO-World)과 같은 물체 문구
(sam6d_realtime/configs/yolo_ism_objects.yaml 의 yolo_prompt, --objects 로 준 물체만)를 한 문장으로 넣는다.
임계값은 원래 스크립트 기본값: box 0.35, text 0.25, NMS IoU 0.3. SAM ViT-H. 라벨은 문구 → 물체 이름으로 바꿔 저장한다.

    ~/DeepLearning/third_party/gsam_venv/bin/python objpose/tools/baselines/fmfusion/detect_gsam_real.py \
        --data ~/DeepLearning/Datasets/fmfusion_real/scans --objects milk Bear ...
"""
from __future__ import annotations

import argparse, json, sys, time, types
from pathlib import Path

import numpy as np
import torch
import torchvision
import yaml

REPO = Path(__file__).resolve().parents[4]
GSAM = Path.home() / "DeepLearning/third_party/Grounded-Segment-Anything"
sys.path[:0] = [str(GSAM / "segment_anything"), str(GSAM), str(GSAM / "GroundingDINO")]
for m in ("litellm", "ram", "ram.models"):                    # 이 실험은 RAM 을 쓰지 않는다 (detect_gsam_prompts.py 와 같음)
    sys.modules[m] = types.ModuleType(m)
sys.modules["ram"].inference_ram = None
sys.modules["ram.models"].ram = None
import run_ram_ground_sam as R  # noqa: E402
from segment_anything import build_sam, SamPredictor  # noqa: E402

BOX_T, TEXT_T, IOU_T = 0.35, 0.25, 0.3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=Path.home() / "DeepLearning/Datasets/fmfusion_real/scans")
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--objects", nargs="+", required=True)
    ap.add_argument("--out-name", default="prediction_gsam")
    a = ap.parse_args()
    dev = "cuda"
    ism = yaml.safe_load((REPO / "sam6d_realtime/configs/yolo_ism_objects.yaml").read_text())
    prompt = {o["name"]: o["yolo_prompt"] for o in ism["objects"]}
    p2l = {prompt[n]: n for n in a.objects}
    caption = ". ".join(prompt[n] for n in a.objects)
    gd = R.load_model(str(GSAM / "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"),
                      str(GSAM / "checkpoints/groundingdino_swint_ogc.pth"), device=dev)
    sam = SamPredictor(build_sam(checkpoint=str(GSAM / "checkpoints/sam_vit_h_4b8939.pth")).to(dev))
    import cv2
    for s in a.scenes or sorted(p.name for p in a.data.iterdir() if p.is_dir()):
        out = a.data / s / a.out_name
        out.mkdir(exist_ok=True)
        times, n_frames, n_boxes = [], 0, 0
        for p in sorted((a.data / s / "color").glob("frame-*.*")):
            torch.cuda.synchronize(); t0 = time.time()
            image_pil, image_t = R.load_image(str(p))
            boxes, scores, _, phrases = R.get_grounding_output(gd, image_t, caption, BOX_T, TEXT_T, device=dev)
            t_gd = time.time() - t0
            n_frames += 1
            if boxes.size(0) == 0:
                times.append((t_gd, 0.0)); continue
            R.recover_bbox(boxes, image_pil.size)
            keep = torchvision.ops.nms(boxes.cpu(), scores, IOU_T).numpy().tolist()
            boxes = boxes.cpu()[keep]; phrases = [phrases[k] for k in keep]
            R.filter_large_bbox(boxes, phrases, image_pil.size, threshold=0.9)
            img = cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB)
            t1 = time.time()
            sam.set_image(img)
            tb = sam.transform.apply_boxes_torch(boxes, img.shape[:2]).to(dev)
            masks, _, _ = sam.predict_torch(point_coords=None, point_labels=None, boxes=tb, multimask_output=False)
            torch.cuda.synchronize()
            times.append((t_gd, time.time() - t1))
            labs = [{p2l.get(k, k): v for k, v in tm.items()} for tm in phrases]
            R.save_mask_data(str(out), p.stem, caption, caption, boxes, labs, masks)
            n_boxes += len(boxes)
        t = np.array(times)
        print(f"{s}: {n_frames} frames, {n_boxes} boxes, GroundingDINO {1000 * t[:, 0].mean():.0f} ms, SAM {1000 * t[:, 1].mean():.0f} ms  [{caption}]", flush=True)
        (out / "timing.json").write_text(json.dumps({"frames": n_frames, "boxes": n_boxes, "gdino_ms": 1000 * t[:, 0].mean(),
                                                     "sam_ms": 1000 * t[:, 1].mean(), "caption": caption}))


if __name__ == "__main__":
    main()
