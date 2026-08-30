#!/usr/bin/env python3
"""Prompt sweep to separate the plush toys (bear vs rabbit vs dinosaur).

On SAM_occlusion frames that contain ONLY the brown teddy bear, run candidate
YOLO-World prompts and, for the top box, compute each toy-template's DINOv2
semantic + masked-appe. We want:
  - a Bear prompt that fires strongly on the bear AND passes Bear gates
  - a Rabbit/Dino prompt that does NOT fire (or fails gates) on the bear
"""
import os, sys, glob
import cv2, torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n

BAG = "SAM_occlusion"
FRAMES = [170, 640, 690, 700, 730, 760]   # bear clearly visible/unoccluded
ROOT = os.path.join(REPO_ROOT, "outputs", "pem_inputs", BAG)

# candidate prompts to test (independent YOLO passes, one class each)
PROMPTS = [
    "teddy bear", "brown teddy bear", "brown bear plush toy",
    "white animal toy", "white rabbit", "white bunny plush toy",
    "green dinosaur toy",
]
# which toy template to score each proposal against
TOYS = ["Bear", "Rabbit"]  # Dinosaur has template but not in config; add if needed


def load_toy(name, model, device, defaults):
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    o = next((x for x in objs if x["name"] == name), None)
    if o is None:  # Dinosaur not in config -> synthesize minimal object
        o = {"name": name, "yolo_prompt": "", "object_id": name,
             "template_dir": f"template/{name}/templates"}
        for k, v in defaults.items():
            o.setdefault(k, v)
        o.setdefault("similarity_threshold", 0.35); o.setdefault("appe_gate", 0.55)
        o.setdefault("match_topk", 5); o.setdefault("top_k", 3)
        o.setdefault("score_threshold", 0.02); o.setdefault("use_mask", True)
        o["cls_cache"] = os.path.join(REPO_ROOT, "outputs", "yolo_ism_object_n",
                                      "template_features", f"{name}_cls.pt")
        o["appe_cache"] = os.path.join(REPO_ROOT, "outputs", "yolo_ism_object_n",
                                       "template_features", f"{name}_appe.pt")
    return o_n.prepare_objects([o], model, device, False)[0]


def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    defaults, _ = o_n.load_config(o_n.DEFAULT_CONFIG)
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    toys = {t: load_toy(t, model, device, defaults) for t in TOYS}
    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    segmentor = yi.build_segmentor(seg_w, device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    from ultralytics import YOLOWorld
    weights = defaults.get("weights", "yolov8m-worldv2.pt")

    imgs = {}
    for fi in FRAMES:
        p = os.path.join(ROOT, f"frame_{fi:06d}", "rgb.png")
        if os.path.exists(p):
            bgr = cv2.imread(p); imgs[fi] = (bgr, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

    print("Scene = brown teddy bear ONLY. YOLO conf averaged over frames where a box fired.\n")
    for prompt in PROMPTS:
        yolo = YOLOWorld(weights)
        yolo.set_classes([prompt])
        confs, gaterows = [], {t: [] for t in TOYS}
        for fi, (bgr, rgb) in imgs.items():
            h, w = bgr.shape[:2]
            res = yolo.predict(bgr, conf=0.02, imgsz=640, verbose=False, device=device)
            cand = []
            if len(res) and res[0].boxes is not None:
                b = res[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                    x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                    cand.append(([x1,y1,x2,y2], float(b.conf[j])))
            cand.sort(key=lambda t: t[1], reverse=True)
            if cand:
                confs.append(cand[0][1])
                for t in TOYS:
                    r = o_n.recognize(toys[t], cand, bgr, rgb, model, device, segmentor, pool)
                    if r["accepted"]:
                        gaterows[t].append((r["best_sem"], r["masked_appe"]))
        fired = f"{len(confs)}/{len(imgs)}"
        avgc = sum(confs)/len(confs) if confs else 0.0
        acc = " ".join(f"{t}:accept={len(gaterows[t])}/{len(imgs)}" for t in TOYS)
        print(f"prompt={prompt!r:26s} fired={fired:>5} avgYOLO={avgc:.3f}  {acc}")


if __name__ == "__main__":
    main()
