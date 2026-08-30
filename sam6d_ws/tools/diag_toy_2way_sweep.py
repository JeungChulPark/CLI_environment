#!/usr/bin/env python3
"""2-way prompt sweep: separate brown BEAR vs white RABBIT via YOLO-World conf.

The DINOv2 gates do NOT discriminate plush toys, so labeling rests entirely on
which prompt fires. We need a prompt PAIR such that on the bear the bear-prompt
outranks the rabbit-prompt, and on the rabbit the rabbit-prompt outranks the
bear-prompt. Measures avg top-box conf per prompt on each scene.
  BEAR scene  = SAM_occlusion frames (brown bear, no rabbit present)
  RABBIT scene= SAM_loop2 early frames (white rabbit on near table)
"""
import os, sys
import cv2, torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n

PI = os.path.join(REPO_ROOT, "outputs", "pem_inputs")
SCENES = {
    "BEAR(occl)":   ("SAM_occlusion", [170, 640, 690, 730, 760]),
    "RABBIT(loop2)":("SAM_loop2",     [0, 10, 20, 30, 40]),
}
PROMPTS = [
    "teddy bear", "brown teddy bear", "brown bear", "brown bear doll",
    "white rabbit", "white bunny", "white rabbit doll", "white bunny toy",
    "white rabbit with long ears", "white plush rabbit",
]

def load_imgs():
    out = {}
    for scene, (bag, fis) in SCENES.items():
        lst = []
        for fi in fis:
            p = os.path.join(PI, bag, f"frame_{fi:06d}", "rgb.png")
            if os.path.exists(p): lst.append(cv2.imread(p))
        out[scene] = lst
    return out

def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    from ultralytics import YOLOWorld
    imgs = load_imgs()
    defaults, _ = o_n.load_config(o_n.DEFAULT_CONFIG)
    weights = defaults.get("weights", "yolov8m-worldv2.pt")
    imgsz = int(defaults.get("imgsz", 960))
    print(f"imgsz={imgsz}  (avg top-box YOLO conf per scene; box fired on the toy)\n")
    hdr = f"{'prompt':30s}" + "".join(f"{s:>15s}" for s in SCENES)
    print(hdr)
    for prompt in PROMPTS:
        yolo = YOLOWorld(weights); yolo.set_classes([prompt])
        row = f"{prompt:30s}"
        for scene in SCENES:
            confs = []
            for bgr in imgs[scene]:
                h, w = bgr.shape[:2]
                res = yolo.predict(bgr, conf=0.02, imgsz=imgsz, verbose=False, device=device)
                best = 0.0
                if len(res) and res[0].boxes is not None and len(res[0].boxes):
                    best = float(res[0].boxes.conf.max())
                confs.append(best)
            avg = sum(confs)/len(confs) if confs else 0.0
            row += f"{avg:15.3f}"
        print(row)

if __name__ == "__main__":
    main()
