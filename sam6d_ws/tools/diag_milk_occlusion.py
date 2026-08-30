#!/usr/bin/env python3
"""Diagnostic: why does milk ISM fail on unoccluded SAM_occlusion frames?

Reuses yolo_ism_object_n.recognize (identical gating to the bridge) on chosen
frames and prints the decision stage + scores so we can separate:
  - proposal-miss  (YOLO-World 'milk carton' returns no/low box)
  - below-sim      (DINOv2 semantic gate rejects)
  - below-appe     (masked-appe gate rejects)
  - detected       (would be accepted)
"""
import os, sys, glob
import cv2, torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n

BAG = sys.argv[1] if len(sys.argv) > 1 else "SAM_occlusion"
FRAMES_ROOT = os.path.join(REPO_ROOT, "outputs", "pem_inputs", BAG)
# detected(170,600,630,640,690,700) vs undetected(should be visible/unoccluded)
TARGET = [int(x) for x in sys.argv[2:]] or [170, 180, 190, 610, 640, 650, 660, 670, 680, 700, 710, 720]

def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    objs = [o for o in objs if o["name"] == "milk"]
    if not objs:
        raise SystemExit("milk not in config")
    ckpt = defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT
    model = yi.build_dinov2(ckpt, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    o = objs[0]
    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    segmentor = yi.build_segmentor(seg_w, device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes([o["yolo_prompt"]])
    min_score = float(o.get("score_threshold", 0.02))
    print(f"prompt='{o['yolo_prompt']}' sim_gate={o['similarity_threshold']} "
          f"appe_gate={o['appe_gate']} score_thr={min_score}\n")
    print(f"{'frame':>7} {'nprop':>5} {'bestYOLO':>8} {'bestSEM':>7} {'mAPPE':>6}  decision")

    for fi in TARGET:
        fd = os.path.join(FRAMES_ROOT, f"frame_{fi:06d}")
        rgbp = os.path.join(fd, "rgb.png")
        if not os.path.exists(rgbp):
            print(f"{fi:7d}  (no rgb.png)"); continue
        bgr = cv2.imread(rgbp); rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = bgr.shape[:2]
        res = yolo.predict(bgr, conf=min_score, imgsz=640, verbose=False, device=device)
        cand = []
        if len(res) and res[0].boxes is not None:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                cand.append(([x1,y1,x2,y2], float(b.conf[j])))
        cand.sort(key=lambda t: t[1], reverse=True)
        r = o_n.recognize(o, cand, bgr, rgb, model, device, segmentor, pool)
        print(f"{fi:7d} {r['num_proposals']:5d} {r['best_yolo']:8.3f} "
              f"{r['best_sem']:7.3f} {r['masked_appe']:6.3f}  {r['decision']}")

if __name__ == "__main__":
    main()
