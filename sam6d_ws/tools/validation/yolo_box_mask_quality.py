#!/usr/bin/env python3
"""YOLO-World box + box-guided SAM mask quality validation v2 (research/validation only).

yolo_box_mask_quality.py is a validation-only tool and must not be shipped as part of
the final production runtime. Keep under tools/validation (or tools/research); do NOT
add to the production ROS package or runtime dependencies. outputs/yolo_test is an
experimental output directory. The sam_yolo env (YOLO/SAM) must not be merged into the
production runtime without PRD approval.

Runs in the isolated `sam_yolo` env. For one dataset (frames dir + optional 0/1 GT):
  1. YOLO-World('milk carton', yolov8m-worldv2 -> s fallback) box per frame, conf sweep.
  2. Box-guided SAM mask (mobile_sam) on top box; mask quality metrics.
  3. Confusion vs GT (SLAM: visibility_labels_thisrun.csv 0/1). Saves box/mask overlays
     and ONLY-genuine-error overlays into failures/. Writes extended per_frame.csv.

GT authority:
  SLAM_with_milk_nomilk -> outputs/validation/SLAM_with_milk_nomilk/visibility_labels_thisrun.csv
    (frame-aligned 311/311; 1=present,0=absent). NOT frame_results.csv.
  only_Milk             -> no file; all frames assumed milk present.
  milk_0609             -> no GT; unlabeled (no TP/TN/FP/FN claims).

Run:
  conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset SLAM_with_milk_nomilk
  conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset only_Milk
  conda run -n sam_yolo python tools/validation/yolo_box_mask_quality.py --dataset milk_0609
"""
import os, csv, json, time, argparse, statistics
import numpy as np
import cv2

ROOT = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PROMPT = "milk carton"
SWEEP = [0.01, 0.02, 0.05, 0.10]
MASK_THR = 0.02      # generate SAM mask for frames with box conf >= this
PRIMARY = 0.05       # operating threshold for per-frame pred_label / confusion / visuals

# gt_mode: "binary_csv" (0/1 file), "all_present", "none"
DATASETS = {
    "SLAM_with_milk_nomilk": dict(
        frames=os.path.join(ROOT, "outputs/validation/SLAM_with_milk_nomilk/frames"),
        gt_mode="binary_csv",
        gt_file=os.path.join(ROOT, "outputs/validation/SLAM_with_milk_nomilk/visibility_labels_thisrun.csv"),
        out=os.path.join(ROOT, "outputs/yolo_test/SLAM_with_milk_nomilk"),
    ),
    "only_Milk": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/only_Milk/frames"),
        gt_mode="all_present",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/only_Milk"),
    ),
    "milk_0609": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/milk_0609/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/milk_0609"),
    ),
    # Generalization bags (no GT -> unknown_gt path). Extracted via rosbags backend in sam_yolo.
    "high_texture_around": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/high_texture_around/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/high_texture_around"),
    ),
    "high_texture_far_close": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/high_texture_far_close/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/high_texture_far_close"),
    ),
    # two_table_* generalization bags (no GT). Frames extracted via rosbags backend in sam_yolo.
    "two_table_around": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/two_table_around/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/two_table_around"),
    ),
    "two_table_around_goback": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/two_table_around_goback/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/two_table_around_goback"),
    ),
    "two_table_diagonal1": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/two_table_diagonal1/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/two_table_diagonal1"),
    ),
    "two_table_diagonal2": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/two_table_diagonal2/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/two_table_diagonal2"),
    ),
    "two_table_goback": dict(
        frames=os.path.join(ROOT, "outputs/yolo_test/two_table_goback/frames"),
        gt_mode="none",
        gt_file=None,
        out=os.path.join(ROOT, "outputs/yolo_test/two_table_goback"),
    ),
}


def load_binary_gt(path):
    gt = {}
    for r in csv.DictReader(open(path)):
        gt[r["frame_id"]] = int(r["milk_visible"])
    return gt


def gt_for(cfg, gt, fid):
    """Return (gt_label, gt_present_or_None, gt_source)."""
    if cfg["gt_mode"] == "binary_csv":
        v = gt.get(fid)
        if v is None:
            return "unlabeled", None, os.path.basename(cfg["gt_file"])
        return ("present" if v == 1 else "absent"), (v == 1), os.path.basename(cfg["gt_file"])
    if cfg["gt_mode"] == "all_present":
        return "present(assumed)", True, "assumption:all_present"
    return "unknown", None, "none"


def confusion_type(gt_present, detected):
    if gt_present is None:
        return "det" if detected else "nodet"
    if gt_present and detected: return "TP"
    if gt_present and not detected: return "FN"
    if (not gt_present) and detected: return "FP"
    return "TN"


def box_mask_metrics(mask, box):
    ys, xs = np.where(mask)
    if len(xs) == 0:
        return dict(mask_area=0, mask_bbox=None, box_mask_iou=0.0, coverage=0.0, leakage=1.0)
    mb = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    x1, y1, x2, y2 = box
    ix1, iy1 = max(x1, mb[0]), max(y1, mb[1])
    ix2, iy2 = min(x2, mb[2]), min(y2, mb[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    a1 = max(0, x2 - x1) * max(0, y2 - y1)
    a2 = (mb[2] - mb[0]) * (mb[3] - mb[1])
    iou = inter / (a1 + a2 - inter) if (a1 + a2 - inter) > 0 else 0.0
    total = int(mask.sum())
    inbox = int(mask[max(0, y1):max(0, y2), max(0, x1):max(0, x2)].sum())
    coverage = inbox / (a1 if a1 > 0 else 1)
    leakage = (total - inbox) / total if total else 1.0
    return dict(mask_area=total, mask_bbox=mb, box_mask_iou=round(iou, 4),
                coverage=round(coverage, 4), leakage=round(leakage, 4))


def _label_img(img, lines, color):
    o = img.copy()
    y = 22
    for ln in lines:
        cv2.putText(o, ln, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        y += 22
    return o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, choices=list(DATASETS))
    ap.add_argument("--weights", default="yolov8m-worldv2.pt")
    ap.add_argument("--limit", type=int, default=None)
    args = ap.parse_args()
    cfg = DATASETS[args.dataset]

    from ultralytics import YOLOWorld, SAM
    frames = sorted(fn[:-4] for fn in os.listdir(cfg["frames"]) if fn.endswith(".png"))
    if args.limit:
        frames = frames[: args.limit]
    gt = load_binary_gt(cfg["gt_file"]) if cfg["gt_mode"] == "binary_csv" else {}
    print(f"[{args.dataset}] frames={len(frames)} gt_mode={cfg['gt_mode']} "
          f"gt_rows={len(gt)} weights={args.weights}")

    try:
        yolo = YOLOWorld(args.weights)
    except Exception as e:
        print(f"[warn] {args.weights} failed ({e}); fallback yolov8s-worldv2.pt")
        args.weights = "yolov8s-worldv2.pt"
        yolo = YOLOWorld(args.weights)
    yolo.set_classes([PROMPT])
    sam = SAM("mobile_sam.pt")

    for sub in ("boxes", "masks", "failures"):
        os.makedirs(os.path.join(cfg["out"], sub), exist_ok=True)

    rows, yolo_ms, mask_ms = [], [], []
    for k, fid in enumerate(frames):
        ip = os.path.join(cfg["frames"], fid + ".png")
        img = cv2.imread(ip)
        t0 = time.time()
        res = yolo.predict(ip, conf=0.001, imgsz=640, verbose=False, device=0)
        yolo_ms.append((time.time() - t0) * 1000)
        b = res[0].boxes
        if b is not None and len(b):
            i = int(np.argmax(b.conf.cpu().numpy()))
            conf = float(b.conf[i]); box = [int(v) for v in b.xyxy[i].cpu().numpy()]
        else:
            conf, box = 0.0, None

        detected = conf >= PRIMARY
        glabel, gpresent, gsrc = gt_for(cfg, gt, fid)
        ctype = confusion_type(gpresent, detected)
        rec = dict(dataset=args.dataset, frame_id=fid, gt_label=glabel, gt_source=gsrc,
                   detected=int(detected), confidence=round(conf, 4), threshold=PRIMARY,
                   pred_label=("present" if detected else "absent"), confusion_type=ctype,
                   bbox_x1=box[0] if box else "", bbox_y1=box[1] if box else "",
                   bbox_x2=box[2] if box else "", bbox_y2=box[3] if box else "",
                   bbox_area=(box[2]-box[0])*(box[3]-box[1]) if box else 0,
                   yolo_ms=round(yolo_ms[-1], 2))

        mask = None
        if box is not None and conf >= MASK_THR:
            t1 = time.time()
            mres = sam(ip, bboxes=[box], verbose=False, device=0)
            mask_ms.append((time.time() - t1) * 1000)
            m = mres[0].masks
            if m is not None and len(m.data):
                mask = m.data[0].cpu().numpy().astype(bool)
                rec.update(box_mask_metrics(mask, box))
        rows.append(rec)

        cs = f"{conf:.2f}"
        if detected and box is not None:
            ov = _label_img(img, [f"{fid} '{PROMPT}' conf={cs}",
                                  f"GT={glabel} pred={rec['pred_label']} ({ctype})"], (0, 255, 0))
            cv2.rectangle(ov, (box[0], box[1]), (box[2], box[3]), (0, 200, 0), 2)
            cv2.imwrite(os.path.join(cfg["out"], "boxes", f"{fid}_conf{cs}_box.png"), ov)
            if mask is not None:
                mo = img.copy(); ovl = mo.copy(); ovl[mask] = (0, 0, 255)
                mo = cv2.addWeighted(ovl, 0.45, mo, 0.55, 0)
                cv2.rectangle(mo, (box[0], box[1]), (box[2], box[3]), (0, 200, 0), 2)
                mo = _label_img(mo, [f"{fid} mask conf={cs}", f"GT={glabel} ({ctype})"], (0, 255, 255))
                cv2.imwrite(os.path.join(cfg["out"], "masks", f"{fid}_conf{cs}_mask.png"), mo)
        if k % 100 == 0:
            print(f"  {k}/{len(frames)}", flush=True)

    save_failures(cfg, rows)
    write_csv(cfg, rows)
    summarize(args.dataset, cfg, rows, gt, yolo_ms, mask_ms)


def save_failures(cfg, rows):
    """Strict: SLAM -> FP/FN only; all_present -> missed only; none -> unknown_gt low/no-det."""
    fdir = os.path.join(cfg["out"], "failures")
    log = []
    for r in rows:
        ct = r["confusion_type"]; conf = r["confidence"]; fid = r["frame_id"]
        reason = None
        if cfg["gt_mode"] == "binary_csv":
            if ct in ("FP", "FN"):
                reason = ct
        elif cfg["gt_mode"] == "all_present":
            if r["detected"] == 0:
                reason = "missed"
        else:  # none
            if conf < PRIMARY:
                reason = "unknown_gt_lowconf" if conf > 0 else "unknown_gt_nodet"
        if reason is None:
            continue
        ip = os.path.join(cfg["frames"], fid + ".png")
        img = cv2.imread(ip)
        box = None
        if r["bbox_x1"] != "":
            box = [r["bbox_x1"], r["bbox_y1"], r["bbox_x2"], r["bbox_y2"]]
            cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (0, 0, 255), 2)
        img = _label_img(img, [f"{fid} conf={conf:.2f} {reason}",
                               f"GT={r['gt_label']} pred={r['pred_label']}"], (0, 0, 255))
        cv2.imwrite(os.path.join(fdir, f"{fid}_conf{conf:.2f}_{reason}.png"), img)
        log.append(dict(frame_id=fid, conf=conf, reason=reason,
                        gt_label=r["gt_label"], confusion_type=ct))
    with open(os.path.join(fdir, "failures.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["frame_id", "conf", "reason", "gt_label", "confusion_type"])
        w.writeheader(); w.writerows(log)
    print(f"[failures] {len(log)} cases -> {fdir}")


def write_csv(cfg, rows):
    path = os.path.join(cfg["out"], "per_frame.csv")
    fields = ["dataset", "frame_id", "gt_label", "gt_source", "detected", "confidence",
              "threshold", "pred_label", "confusion_type", "bbox_x1", "bbox_y1",
              "bbox_x2", "bbox_y2", "bbox_area", "mask_area", "box_mask_iou",
              "coverage", "leakage", "yolo_ms", "mask_ms"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fields})
    print(f"[csv] {path}")


def _var(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.pvariance(xs), 2) if len(xs) > 1 else 0.0


def summarize(name, cfg, rows, gt, yolo_ms, mask_ms):
    print(f"\n=== {name} SUMMARY ===")
    ym = sum(yolo_ms)/len(yolo_ms) if yolo_ms else 0
    mm = sum(mask_ms)/len(mask_ms) if mask_ms else 0
    print(f"[runtime] YOLO {ym:.2f} ms/frame | SAM mask {mm:.2f} ms/det | total~{ym+mm:.2f} ms")
    masked = [r for r in rows if r.get("mask_area")]
    if masked:
        iou = [r["box_mask_iou"] for r in masked]; cov = [r["coverage"] for r in masked]
        leak = [r["leakage"] for r in masked]
        print(f"[mask] n={len(masked)} IoU med={statistics.median(iou):.3f} "
              f"coverage med={statistics.median(cov):.3f} leakage med={statistics.median(leak):.3f}")

    if cfg["gt_mode"] == "binary_csv":
        print(f"[confusion vs {os.path.basename(cfg['gt_file'])}]  (labeled frames only)")
        print(f"{'thr':>5} {'TP':>4} {'FP':>4} {'TN':>4} {'FN':>4} {'P':>6} {'R':>6} {'F1':>6}")
        for thr in SWEEP:
            TP=FP=TN=FN=0
            for r in rows:
                v = gt.get(r["frame_id"])
                if v is None: continue
                det = r["confidence"] >= thr; gp = (v == 1)
                if gp and det: TP+=1
                elif gp and not det: FN+=1
                elif (not gp) and det: FP+=1
                else: TN+=1
            P=TP/(TP+FP) if TP+FP else 0; R=TP/(TP+FN) if TP+FN else 0
            F1=2*P*R/(P+R) if P+R else 0
            print(f"{thr:>5.2f} {TP:>4} {FP:>4} {TN:>4} {FN:>4} {P:>6.3f} {R:>6.3f} {F1:>6.3f}")
    elif cfg["gt_mode"] == "all_present":
        n=len(rows)
        print("[all-present assumption — every frame GT=present]")
        for thr in SWEEP:
            det=sum(1 for r in rows if r["confidence"]>=thr)
            print(f"  thr={thr:.2f} detected={det}/{n} missed={n-det} recall={det/n:.3f}")
        det=[r for r in rows if r["detected"]]
        confs=[r["confidence"] for r in det]
        cxs=[(r["bbox_x1"]+r["bbox_x2"])/2 for r in det if r["bbox_x1"]!=""]
        cys=[(r["bbox_y1"]+r["bbox_y2"])/2 for r in det if r["bbox_x1"]!=""]
        areas=[r["bbox_area"] for r in det]
        mareas=[r["mask_area"] for r in rows if r.get("mask_area")]
        if confs:
            print(f"[conf@{PRIMARY}] mean={statistics.mean(confs):.3f} std={statistics.pstdev(confs):.3f} "
                  f"min={min(confs):.3f} max={max(confs):.3f}")
            print(f"[stability] bbox cx var={_var(cxs):.0f} cy var={_var(cys):.0f} "
                  f"area var={_var(areas):.0f} | mask area var={_var(mareas):.0f}")
    else:
        n=len(rows)
        print("[no GT — unlabeled; no TP/TN/FP/FN claims]")
        for thr in SWEEP:
            det=sum(1 for r in rows if r["confidence"]>=thr)
            print(f"  thr={thr:.2f} detected={det}/{n} detection_rate={det/n:.3f}")
        confs=[r["confidence"] for r in rows if r["detected"]]
        areas=[r["bbox_area"] for r in rows if r["detected"]]
        mareas=[r["mask_area"] for r in rows if r.get("mask_area")]
        if confs:
            print(f"[conf@{PRIMARY}] mean={statistics.mean(confs):.3f} std={statistics.pstdev(confs):.3f} "
                  f"min={min(confs):.3f} max={max(confs):.3f}")
            print(f"[stability] bbox area var={_var(areas):.0f} | mask area var={_var(mareas):.0f}")


if __name__ == "__main__":
    main()
