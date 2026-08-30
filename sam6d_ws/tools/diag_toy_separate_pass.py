#!/usr/bin/env python3
"""Validate SEPARATE-pass routing: run each toy prompt in its OWN YOLO pass and
apply a per-object score_threshold. This avoids the shared-pass argmax/NMS
suppression that lets the bear prompt swallow the rabbit box. Reports the label
each scene/frame would get."""
import os, sys
import cv2, torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n

PI = os.path.join(REPO_ROOT, "outputs", "pem_inputs")
BEARP, RABBITP = "brown bear doll", "white rabbit doll"
BEAR_THR, RABBIT_THR = 0.35, 0.30
SCENES = {
    "occl-bear":    ("SAM_occlusion", [170, 640, 690, 730, 760, 780]),
    "loop2-rabbit": ("SAM_loop2",     [0, 10, 20, 30, 40, 50, 60, 300]),
    "loop2-farbear":("SAM_loop2",     [480, 490, 500, 510]),
    "loop2-distract":("SAM_loop2",    [320, 340]),
}

def top_conf(yolo, bgr, imgsz, device):
    res = yolo.predict(bgr, conf=0.02, imgsz=imgsz, verbose=False, device=device)
    if len(res) and res[0].boxes is not None and len(res[0].boxes):
        return float(res[0].boxes.conf.max())
    return 0.0

def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    defaults, _ = o_n.load_config(o_n.DEFAULT_CONFIG)
    imgsz = int(defaults.get("imgsz", 960))
    from ultralytics import YOLOWorld
    w = defaults.get("weights", "yolov8m-worldv2.pt")
    ybear = YOLOWorld(w); ybear.set_classes([BEARP])
    yrab = YOLOWorld(w); yrab.set_classes([RABBITP])
    print(f"imgsz={imgsz} SEPARATE passes | Bear '{BEARP}' thr={BEAR_THR} | "
          f"Rabbit '{RABBITP}' thr={RABBIT_THR}")
    print(f"{'scene':16s}{'frame':>7}{'bear':>7}{'rabbit':>8}   -> accepted")
    for scene, (bag, fis) in SCENES.items():
        for fi in fis:
            p = os.path.join(PI, bag, f"frame_{fi:06d}", "rgb.png")
            if not os.path.exists(p): continue
            bgr = cv2.imread(p)
            bc = top_conf(ybear, bgr, imgsz, device)
            rc = top_conf(yrab, bgr, imgsz, device)
            acc = []
            if bc >= BEAR_THR: acc.append("Bear")
            if rc >= RABBIT_THR: acc.append("Rabbit")
            print(f"{scene:16s}{fi:7d}{bc:7.3f}{rc:8.3f}   -> {acc}")

if __name__ == "__main__":
    main()
