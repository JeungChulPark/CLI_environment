#!/usr/bin/env python3
"""phase39 리뷰 — BASE 와 ML_REL_D 가 달라진 프레임만 나란히 그린다.

TP/FP 지표는 프레임 단위 '보였나'라서 **박스가 그 물체 위에 있는지는 보지 않는다**
(2026-07 에 이 함정으로 결론이 한 번 뒤집혔다). 늘어난 TP 가 진짜인지 눈으로 확인하려고
바뀐 셀만 골라 그린다. 초록=GT 가시, 빨강=GT 비가시.
"""
import copy, csv, os, sys
import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit"))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import phase21_common as p21
import yolo_ism_object_n as o_n
import yolo_ism as yi
import run_ab as AB

FRAMES = os.path.join(RSRCH, "gt_input", "frames")
OUT = os.path.join(REPO, "outputs", "phase39_multilabel_relative", "viz")


def draw(bgr, acc, vis, title):
    img = bgr.copy()
    drawn = []
    for o, box in sorted(acc.items()):
        x1, y1, x2, y2 = box
        col = (90, 220, 90) if o in vis else (60, 60, 235)
        ins = sum(5 for b in drawn if max(abs(b[0]-x1), abs(b[1]-y1),
                                          abs(b[2]-x2), abs(b[3]-y2)) <= 4)
        drawn.append(box)
        X1, Y1, X2, Y2 = x1+ins, y1+ins, x2-ins, y2-ins
        cv2.rectangle(img, (X1, Y1), (X2, Y2), col, 3)
        t = o.replace("_high", "")
        cv2.rectangle(img, (X1, max(0, Y1-19)), (X1+8*len(t), max(0, Y1-19)+19), col, -1)
        cv2.putText(img, t, (X1+2, max(0, Y1-19)+14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
    bar = np.full((26, img.shape[1], 3), 30, np.uint8)
    cv2.putText(bar, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return np.vstack([bar, img])


def main():
    os.makedirs(OUT, exist_ok=True)
    defaults, objs_cfg = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0")
    device = device if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(copy.deepcopy(objs_cfg), model, device, False)
    name2obj = {o["name"]: o for o in objs}
    prompts, groups = o_n.build_prompt_groups(objs)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
    img_sz = int(defaults.get("imgsz", 640))
    gt = AB.corrected_gt()

    man = list(csv.DictReader(open(os.path.join(
        REPO, "outputs", "phase39_multilabel_relative", "csv", "phase39_manifest.csv"))))
    changed = [m for m in man if set(m["BASE"].split(";")) != set(m["ML_REL_D"].split(";"))]
    print(f"바뀐 프레임 {len(changed)}/{len(man)}")
    rows = []
    for n, m in enumerate(changed, 1):
        ds, fr = m["bag"], int(m["frame"])
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)
        vis = gt[(ds, fr)]
        pb_off = AB.yolo_boxes(yolo, bgr, prompts, min_score, img_sz, device, False)
        pb_ml = AB.yolo_boxes(yolo, bgr, prompts, min_score, img_sz, device, True)
        r = o_n.apply_cross_object_nms(objs, o_n.recognize_frame(
            groups, pb_off, bgr, rgb, norm_full, model, device, segmentor, pool))
        A = {k: v["box"] for k, v in r.items() if v["accepted"] and v["box"]}
        B = AB.relative_assign(objs, pb_ml, bgr, rgb, norm_full, model, device,
                               segmentor, pool, True, name2obj)
        img = np.hstack([draw(bgr, A, vis, f"BASE (현행)  {ds} f{fr}"),
                         draw(bgr, B, vis, "ML_REL_D  (1)+(2)+(3)")])
        fn = f"{ds}_{fr:06d}.png"
        cv2.imwrite(os.path.join(OUT, fn), img)
        sa, sb = set(A), set(B)
        rows.append({"image": fn, "bag": ds, "frame": fr,
                     "gt_visible": ";".join(sorted(vis)),
                     "gain_TP": ";".join(sorted((sb-sa) & vis)),
                     "new_FP": ";".join(sorted((sb-sa) - vis)),
                     "lost_TP": ";".join(sorted((sa-sb) & vis)),
                     "removed_FP": ";".join(sorted((sa-sb) - vis))})
        if n % 40 == 0:
            print(f"  {n}/{len(changed)}", flush=True)
    with open(os.path.join(OUT, "..", "csv", "phase39_changed.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    g = sum(len(r["gain_TP"].split(";")) if r["gain_TP"] else 0 for r in rows)
    nf = sum(len(r["new_FP"].split(";")) if r["new_FP"] else 0 for r in rows)
    lt = sum(len(r["lost_TP"].split(";")) if r["lost_TP"] else 0 for r in rows)
    rf = sum(len(r["removed_FP"].split(";")) if r["removed_FP"] else 0 for r in rows)
    print(f"\n바뀐 셀: 새 TP {g} · 새 FP {nf} · 잃은 TP {lt} · 없어진 FP {rf}")
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
