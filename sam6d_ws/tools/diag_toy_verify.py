#!/usr/bin/env python3
"""Verify the config fix end-to-end on the brown-bear-only SAM_occlusion scene,
mirroring production (shared YOLO pass over all enabled toy prompts + per-object
recognize). Expect: Bear accepted on most frames, Rabbit accepted on ~none."""
import os, sys
import cv2, torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n

BAG = "SAM_occlusion"
FRAMES = [170, 640, 690, 700, 730, 760, 180, 610, 650, 660]
ROOT = os.path.join(REPO_ROOT, "outputs", "pem_inputs", BAG)

def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    objs = [o for o in objs if o["name"] in ("Bear", "Rabbit", "Dinosaur")]
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    segmentor = yi.build_segmentor(seg_w, device); pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    unique, groups = o_n.build_prompt_groups(objs)

    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(unique)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    print(f"enabled toys={[o['name'] for o in objs]} prompts={unique}\n")
    acc = {o["name"]: 0 for o in objs}; n = 0
    for fi in FRAMES:
        p = os.path.join(ROOT, f"frame_{fi:06d}", "rgb.png")
        if not os.path.exists(p): continue
        n += 1; bgr = cv2.imread(p); rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB); h, w = bgr.shape[:2]
        res = yolo.predict(bgr, conf=min_score, imgsz=640, verbose=False, device=device)
        pb = {i: [] for i in range(len(unique))}
        if len(res) and res[0].boxes is not None:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in pb: pb[ci].append(([x1,y1,x2,y2], float(b.conf[j])))
        names = []
        for pi, objs_here in groups.items():
            cand = sorted(pb.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                r = o_n.recognize(o, cand, bgr, rgb, model, device, segmentor, pool)
                if r["accepted"]: acc[o["name"]] += 1; names.append(o["name"])
        print(f"frame {fi:6d}: accepted={names}")
    print("\n=== accept counts over", n, "bear frames ===")
    for k, v in acc.items(): print(f"  {k:10s} {v}/{n}")

if __name__ == "__main__":
    main()
