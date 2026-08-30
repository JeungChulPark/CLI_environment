#!/usr/bin/env python3
"""A/B: YOLO-World imgsz vs milk recall AND per-frame time on SAM_occlusion.

Runs the FULL production pipeline (all enabled objects: shared YOLO pass +
per-object recognize) over every stride-10 frame, at each imgsz, and reports:
  - milk/Bear accepted counts (recall)
  - avg YOLO predict ms/frame  (the part imgsz changes)
  - avg full-frame ms/frame    (predict + all-object DINOv2/appe recognize)
Timing uses cuda.synchronize; a warmup frame per imgsz is excluded.
"""
import os, sys, time
import cv2, torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n

BAG = "SAM_occlusion"
ROOT = os.path.join(REPO_ROOT, "outputs", "pem_inputs", BAG)
IMGSZS = [int(x) for x in (sys.argv[1:] or [640, 960, 1280])]

def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()

def main():
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    # keep only objects with valid CAD (mirror the bridge's object set)
    objs = [o for o in objs if os.path.isfile(o_n._abspath(o.get("cad_ply", "")))]
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    segmentor = yi.build_segmentor(seg_w, device); pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    unique, groups = o_n.build_prompt_groups(objs)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)

    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(unique)

    # preload frames
    frames = []
    for d in sorted(os.listdir(ROOT)):
        p = os.path.join(ROOT, d, "rgb.png")
        if d.startswith("frame_") and os.path.exists(p):
            bgr = cv2.imread(p); frames.append((d, bgr, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
    print(f"objects={len(objs)} prompts={len(unique)} frames={len(frames)}\n")
    print(f"{'imgsz':>6} {'milk':>5} {'Bear':>5} {'allObj':>7} {'YOLOms':>8} {'frameMs':>8}")

    def run_frame(bgr, rgb, imgsz, timeit):
        h, w = bgr.shape[:2]
        if timeit: sync(); t0 = time.perf_counter()
        res = yolo.predict(bgr, conf=min_score, imgsz=imgsz, verbose=False, device=device)
        if timeit: sync(); t_yolo = time.perf_counter() - t0
        pb = {i: [] for i in range(len(unique))}
        if len(res) and res[0].boxes is not None:
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in pb: pb[ci].append(([x1,y1,x2,y2], float(b.conf[j])))
        acc = {}
        for pi, oh in groups.items():
            cand = sorted(pb.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in oh:
                r = o_n.recognize(o, cand, bgr, rgb, model, device, segmentor, pool)
                if r["accepted"]: acc[o["name"]] = acc.get(o["name"], 0) + 1
        if timeit: sync(); t_frame = time.perf_counter() - t0
        return acc, (t_yolo, t_frame) if timeit else (0, 0)

    for imgsz in IMGSZS:
        # warmup (excluded)
        run_frame(frames[0][1], frames[0][2], imgsz, False)
        milk = bear = allobj = 0; y_ms = f_ms = 0.0
        for _, bgr, rgb in frames:
            acc, (ty, tf) = run_frame(bgr, rgb, imgsz, True)
            milk += acc.get("milk", 0); bear += acc.get("Bear", 0)
            allobj += sum(acc.values()); y_ms += ty*1000; f_ms += tf*1000
        n = len(frames)
        print(f"{imgsz:6d} {milk:5d} {bear:5d} {allobj:7d} {y_ms/n:8.1f} {f_ms/n:8.1f}")

if __name__ == "__main__":
    main()
