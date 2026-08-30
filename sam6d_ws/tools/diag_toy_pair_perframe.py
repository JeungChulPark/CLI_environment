#!/usr/bin/env python3
"""Per-frame conf of the chosen bear/rabbit prompt PAIR (production routing:
both prompts set together) across bear / rabbit / far-bear scenes, so we can set
per-object score_threshold guards. Prints, per frame, the top-conf box for each
prompt (bear vs rabbit) so we see who wins and by how much."""
import os, sys
import cv2, torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n

PI = os.path.join(REPO_ROOT, "outputs", "pem_inputs")
BEARP, RABBITP = "brown bear doll", "white rabbit doll"
SCENES = {
    "occl-bear":  ("SAM_occlusion", [170, 600, 640, 690, 730, 760, 780]),
    "loop2-rabbit":("SAM_loop2",    [0, 10, 20, 30, 40, 50, 60]),
    "loop2-farbear":("SAM_loop2",   [480, 490, 500, 510]),
    "loop2-distract":("SAM_loop2",  [300, 320, 340]),   # near table jug/mug, check FP
}

def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    defaults, _ = o_n.load_config(o_n.DEFAULT_CONFIG)
    imgsz = int(defaults.get("imgsz", 960))
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes([BEARP, RABBITP])   # 0=bear, 1=rabbit
    print(f"imgsz={imgsz}  prompts: 0='{BEARP}' 1='{RABBITP}'")
    print(f"{'scene':16s}{'frame':>7}{'bearConf':>9}{'rabbitConf':>11}  winner")
    for scene, (bag, fis) in SCENES.items():
        for fi in fis:
            p = os.path.join(PI, bag, f"frame_{fi:06d}", "rgb.png")
            if not os.path.exists(p): continue
            bgr = cv2.imread(p)
            res = yolo.predict(bgr, conf=0.02, imgsz=imgsz, verbose=False, device=device)
            bc = rc = 0.0
            if len(res) and res[0].boxes is not None and len(res[0].boxes):
                b = res[0].boxes
                for j in range(len(b)):
                    ci = int(b.cls[j]); cf = float(b.conf[j])
                    if ci == 0: bc = max(bc, cf)
                    else: rc = max(rc, cf)
            win = "Bear" if bc > rc else ("Rabbit" if rc > bc else "-")
            print(f"{scene:16s}{fi:7d}{bc:9.3f}{rc:11.3f}  {win}")

if __name__ == "__main__":
    main()
