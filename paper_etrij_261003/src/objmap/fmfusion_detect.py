"""FM-Fusion front end on YCB-V: GroundingDINO + SAM with the 21 fixed prompts.

Follows run_ram_ground_sam.py of FM-Fusion's Grounded-SAM fork (github.com/glennliu/
Grounded-Segment-Anything): GroundingDINO Swin-T OGC, box threshold 0.35, text threshold
0.25, NMS IoU 0.3, per-box token map {prompt: score}, SAM ViT-H masks, output
frame-XXXXXX_label.json + frame-XXXXXX_mask.png in prediction_no_augment/.
The one change: RAM's per-image tags are replaced by the 21 fixed prompts, given in
every frame (same prompt list as the other methods). The fork's filter_large_bbox() does
not write its result back, so it is a no-op there and is left out here.

Runs in the `fmfusion` conda env:
  ~/anaconda3/envs/fmfusion/bin/python fmfusion_detect.py --scenes 52 [--frame-gap 10]
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision
from PIL import Image

GSAM = Path(os.environ.get("GSAM_FMFUSION", Path.home() / "DeepLearning/baselines/gsam_fmfusion"))
WEIGHTS = Path(os.environ.get("BASELINE_WEIGHTS", Path.home() / "DeepLearning/baselines/weights"))
sys.path.insert(0, str(GSAM))
sys.path.insert(0, str(GSAM / "segment_anything"))   # the package, not the fork's same-named folder
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import GroundingDINO.groundingdino.datasets.transforms as T  # noqa: E402
from GroundingDINO.groundingdino.models import build_model  # noqa: E402
from GroundingDINO.groundingdino.util.slconfig import SLConfig  # noqa: E402
from GroundingDINO.groundingdino.util.utils import clean_state_dict, get_grouped_tokens  # noqa: E402
from segment_anything import SamPredictor, build_sam  # noqa: E402
from ycbv.paths import PROMPTS  # noqa: E402
from common import seq_name  # noqa: E402

FMF_DATA = Path(os.environ.get("FMFUSION_DATA", Path.home() / "DeepLearning/Dataset/bop/ycbv_work/fmfusion"))
BOX_THR, TEXT_THR, NMS_IOU = 0.35, 0.25, 0.3
CAPTION = ". ".join(PROMPTS) + "."
DEVICE = "cuda"


def load_image(path):
    image_pil = Image.open(path).convert("RGB")
    tf = T.Compose([T.RandomResize([800], max_size=1333), T.ToTensor(),
                    T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])
    image, _ = tf(image_pil, None)
    return image_pil, image


def load_dino():
    args = SLConfig.fromfile(str(GSAM / "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"))
    args.device = DEVICE
    model = build_model(args)
    ck = torch.load(WEIGHTS / "groundingdino_swint_ogc.pth", map_location="cpu")
    model.load_state_dict(clean_state_dict(ck["model"]), strict=False)
    return model.eval().to(DEVICE)


def grounding(model, image, caption):
    """get_grounding_output() of the fork: boxes (cxcywh, 0-1), max score, token maps."""
    caption = caption.lower().strip()
    with torch.no_grad():
        out = model(image[None].to(DEVICE), captions=[caption])
    logits = out["pred_logits"].cpu().sigmoid()[0]
    boxes = out["pred_boxes"].cpu()[0]
    keep = logits.max(dim=1)[0] > BOX_THR
    logits, boxes = logits[keep], boxes[keep]
    tok = model.tokenizer
    tokenized = tok(caption)
    groups, pos2prompt = get_grouped_tokens(tokenized["input_ids"], tok)
    assert len(groups) + 1 == len(caption.split("."))
    scores, maps = [], []
    for logit in logits:
        token_map, best = {}, 0.0
        ids = torch.unique(pos2prompt[(logit > TEXT_THR).nonzero(as_tuple=True)[0]])
        for pid in ids:
            pm = groups[pid]
            if logit[pm].min().item() < TEXT_THR:
                continue
            label = tok.decode([tokenized["input_ids"][i] for i in pm])
            token_map[label] = logit[pm].max().item()
            best = max(best, token_map[label])
        scores.append(best)
        maps.append(token_map)
    return boxes, torch.Tensor(scores), maps


def save(out_dir, name, boxes, maps, masks):
    tags = ". ".join(PROMPTS)   # RAM-style tag string: no trailing '.', FM-Fusion splits on it
    data = {"raw_tags": tags, "tags": tags, "mask": [{"value": 0, "label": "background"}]}
    for v, (m, b) in enumerate(zip(maps, boxes), 1):
        data["mask"].append({"value": v, "labels": m, "box": b.numpy().tolist()})
    json.dump(data, open(out_dir / f"{name}_label.json", "w"))
    img = np.zeros(masks.shape[-2:], np.uint8)
    for k, m in enumerate(masks):
        img[m.cpu().numpy()[0]] = k + 1
    cv2.imwrite(str(out_dir / f"{name}_mask.png"), img)


def run_scene(scene, dino, sam, gap, dataset="ycbv"):
    d = FMF_DATA / "scans" / seq_name(dataset, scene)
    out = d / "prediction_no_augment"
    out.mkdir(exist_ok=True)   # an existing link (ObChange scans share one folder) is kept
    frames = sorted((d / "color").glob("frame-*.png"))
    times, n_det = [], 0
    for p in frames:
        name = p.stem
        if int(name.split("-")[1]) % gap != 0 or (out / f"{name}_label.json").exists():
            continue
        t0 = time.time()
        pil, image = load_image(p)
        boxes, scores, maps = grounding(dino, image, CAPTION)
        if boxes.size(0) == 0:
            times.append(time.time() - t0)
            continue
        W, H = pil.size
        boxes = boxes * torch.Tensor([W, H, W, H])
        boxes[:, :2] -= boxes[:, 2:] / 2
        boxes[:, 2:] += boxes[:, :2]
        idx = torchvision.ops.nms(boxes, scores, NMS_IOU).numpy().tolist()
        boxes, maps = boxes[idx], [maps[i] for i in idx]
        rgb = cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB)
        sam.set_image(rgb)
        tb = sam.transform.apply_boxes_torch(boxes, rgb.shape[:2]).to(DEVICE)
        masks, _, _ = sam.predict_torch(point_coords=None, point_labels=None, boxes=tb, multimask_output=False)
        save(out, name, boxes, maps, masks)
        times.append(time.time() - t0)
        n_det += len(maps)
    return len(times), n_det, float(np.mean(times)) if times else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, nargs="*", default=list(range(48, 60)))
    ap.add_argument("--frame-gap", type=int, default=10)
    ap.add_argument("--dataset", default="ycbv", help="see common.seq_cameras")
    a = ap.parse_args()
    dino = load_dino()
    sam = SamPredictor(build_sam(checkpoint=str(WEIGHTS / "sam_vit_h_4b8939.pth")).to(DEVICE))
    log = FMF_DATA / "detect_times.json"
    rec = json.load(open(log)) if log.exists() else {}
    for s in a.scenes:
        n, nd, t = run_scene(s, dino, sam, a.frame_gap, a.dataset)
        print(f"{seq_name(a.dataset, s)}: {n} frames detected, {nd} boxes, {1000 * t:.0f} ms/frame", flush=True)
        if n:
            rec[seq_name(a.dataset, s)] = {"frames": n, "boxes": nd, "ms_per_frame": 1000 * t}
            json.dump(rec, open(log, "w"), indent=1)


if __name__ == "__main__":
    main()
