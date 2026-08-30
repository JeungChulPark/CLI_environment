#!/usr/bin/env python3
"""YOLO-World proposal visualizer (research/validation only).

Sibling of yolo_box_mask_quality.py. That tool keeps ONLY the top-1 box
(np.argmax conf) and throws every other candidate away. This tool instead draws
the full proposal set per frame so we can see *why* milk is mis-detected /
missed: rank-3, rank-5 and the full post-NMS candidate list, each labelled with
rank, score and class/prompt.

Validation-only, exactly like yolo_box_mask_quality.py:
  - MUST NOT ship in the production ROS runtime. YOLO-World is NOT in the
    production SAM-6D path (sam6d_inference_node.py has zero yolo/ultralytics
    refs); it is a shadow front-gate experiment.
  - Runs in the isolated `sam_yolo` conda env.

Input  (READ-ONLY): outputs/yolo_test/<bag>/frames/<idx:06d>.png
                     (already-extracted frames; ros2 bag play is NOT re-run --
                      identical frames, safer, no ROS env needed.)
Output (NEW only)  : outputs/temp/yolo_world_proposal/<bag>/
                       frame_<idx>_top3.jpg / _top5.jpg / _topn.jpg
                       proposals.csv          (per-frame proposal sidecar)

YOLO-World settings mirror yolo_box_mask_quality.py:
  weights yolov8m-worldv2.pt (-> s fallback), prompt "milk carton" (single
  class via set_classes), predict(conf=PREDICT_FLOOR, imgsz=640), default
  ultralytics NMS (iou=0.7) / max_det=300. PRIMARY=0.05 is the operating
  threshold used by the production-baseline tool to call a frame "detected".

Run:
  conda run -n sam_yolo python tools/validation/yolo_world_proposal_viz.py --all
  conda run -n sam_yolo python tools/validation/yolo_world_proposal_viz.py --bag two_table_diagonal1
  conda run -n sam_yolo python tools/validation/yolo_world_proposal_viz.py --all --stride 5
"""
import os, csv, time, argparse
import numpy as np
import cv2

ROOT = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PROMPT = "milk carton"          # identical to yolo_box_mask_quality.py
PREDICT_FLOOR = 0.001           # predict conf floor (full candidate list)
PRIMARY = 0.05                  # operating threshold (detected vs absent)
TOPN_CONF_FLOOR = 0.01          # topn image: drop near-zero junk below this
TOPN_MAX = 20                   # topn image: cap boxes drawn (keeps it readable)

# 7 generalization bags. Input frames already live under outputs/yolo_test/<bag>/frames.
BAGS = [
    "high_texture_around",
    "high_texture_far_close",
    "two_table_around",
    "two_table_around_goback",
    "two_table_diagonal1",
    "two_table_diagonal2",
    "two_table_goback",
]

# Rank -> BGR colour (rank1 = bright green, then yellow/cyan/orange/magenta, rest grey).
RANK_COLORS = [(0, 255, 0), (0, 255, 255), (255, 255, 0), (0, 165, 255), (255, 0, 255)]
REST_COLOR = (160, 160, 160)


def in_frames_dir(bag):
    return os.path.join(ROOT, "outputs/yolo_test", bag, "frames")


def out_dir(bag):
    return os.path.join(ROOT, "outputs/temp/yolo_world_proposal", bag)


def rank_color(rank0):
    return RANK_COLORS[rank0] if rank0 < len(RANK_COLORS) else REST_COLOR


def draw_box(img, box, rank0, conf):
    """Draw one ranked proposal with a readable label."""
    x1, y1, x2, y2 = box
    c = rank_color(rank0)
    cv2.rectangle(img, (x1, y1), (x2, y2), c, 2)
    label = f"#{rank0 + 1} {PROMPT} {conf:.3f}"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    ly = max(0, y1 - th - 4)
    cv2.rectangle(img, (x1, ly), (x1 + tw + 4, ly + th + 4), c, -1)
    cv2.putText(img, label, (x1 + 2, ly + th + 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)


def header(img, lines, color):
    y = 20
    for ln in lines:
        cv2.putText(img, ln, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, ln, (5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        y += 20


def render(img, props, kind, fid, n_total):
    """props: list of (conf, [x1,y1,x2,y2]) sorted desc. Returns an overlay image."""
    o = img.copy()
    if not props:
        header(o, [f"{fid}  {kind.upper()}  NO PROPOSAL (predict floor {PREDICT_FLOOR})",
                   f"prompt='{PROMPT}'"], (0, 0, 255))
        return o
    if kind == "top3":
        sel = props[:3]
    elif kind == "top5":
        sel = props[:5]
    else:  # topn
        sel = [p for p in props if p[0] >= TOPN_CONF_FLOOR][:TOPN_MAX]
    # draw lowest rank first so rank1 ends up on top
    for i in range(len(sel) - 1, -1, -1):
        conf, box = sel[i]
        draw_box(o, box, i, conf)
    top1 = props[0][0]
    det = "DETECTED" if top1 >= PRIMARY else "below-PRIMARY"
    if kind == "topn":
        shown = f"showing {len(sel)} of {n_total} (conf>={TOPN_CONF_FLOOR}, cap {TOPN_MAX})"
    else:
        shown = f"showing {len(sel)} of {n_total}"
    header(o, [f"{fid}  {kind.upper()}  {shown}",
               f"top1={top1:.3f} @PRIMARY={PRIMARY} -> {det}  prompt='{PROMPT}'"],
           (0, 255, 0) if top1 >= PRIMARY else (0, 200, 255))
    return o


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--all", action="store_true", help="process all 7 bags")
    g.add_argument("--bag", choices=BAGS, help="process a single bag")
    ap.add_argument("--weights", default="yolov8m-worldv2.pt")
    ap.add_argument("--stride", type=int, default=1, help="sample every Nth frame (default 1=all)")
    ap.add_argument("--limit", type=int, default=None, help="cap frames per bag (debug)")
    args = ap.parse_args()
    bags = BAGS if args.all else [args.bag]

    from ultralytics import YOLOWorld
    try:
        yolo = YOLOWorld(args.weights)
    except Exception as e:
        print(f"[warn] {args.weights} failed ({e}); fallback yolov8s-worldv2.pt")
        args.weights = "yolov8s-worldv2.pt"
        yolo = YOLOWorld(args.weights)
    yolo.set_classes([PROMPT])
    print(f"[init] weights={args.weights} prompt='{PROMPT}' stride={args.stride}")

    grand = {}
    for bag in bags:
        fdir = in_frames_dir(bag)
        if not os.path.isdir(fdir):
            print(f"[skip] {bag}: no frames dir {fdir}")
            grand[bag] = dict(saved=0, frames=0, no_prop=0, err="no_frames_dir")
            continue
        odir = out_dir(bag)
        os.makedirs(odir, exist_ok=True)
        frames = sorted(fn[:-4] for fn in os.listdir(fdir) if fn.endswith(".png"))
        if args.stride > 1:
            frames = frames[:: args.stride]
        if args.limit:
            frames = frames[: args.limit]

        rows, saved, no_prop = [], 0, 0
        t_bag = time.time()
        for k, fid in enumerate(frames):
            ip = os.path.join(fdir, fid + ".png")
            img = cv2.imread(ip)
            if img is None:
                continue
            res = yolo.predict(ip, conf=PREDICT_FLOOR, imgsz=640, verbose=False, device=0)
            b = res[0].boxes
            props = []
            if b is not None and len(b):
                confs = b.conf.cpu().numpy()
                xyxy = b.xyxy.cpu().numpy()
                order = np.argsort(-confs)
                props = [(float(confs[i]), [int(v) for v in xyxy[i]]) for i in order]
            n_total = len(props)
            if n_total == 0:
                no_prop += 1

            for kind in ("top3", "top5", "topn"):
                ov = render(img, props, kind, fid, n_total)
                cv2.imwrite(os.path.join(odir, f"frame_{fid}_{kind}.jpg"), ov)
                saved += 1

            rec = dict(dataset=bag, frame_id=fid, timestamp_ns="NA",
                       n_proposals=n_total, no_proposal=int(n_total == 0),
                       top1_conf=round(props[0][0], 4) if props else "",
                       top1_detected=int(bool(props) and props[0][0] >= PRIMARY),
                       primary_threshold=PRIMARY)
            for r in range(5):
                if r < n_total:
                    c, bx = props[r]
                    rec[f"rank{r+1}_conf"] = round(c, 4)
                    rec[f"rank{r+1}_bbox"] = f"{bx[0]} {bx[1]} {bx[2]} {bx[3]}"
                else:
                    rec[f"rank{r+1}_conf"] = ""
                    rec[f"rank{r+1}_bbox"] = ""
            rows.append(rec)
            if k % 100 == 0:
                print(f"  [{bag}] {k}/{len(frames)} (no_prop={no_prop})", flush=True)

        fields = (["dataset", "frame_id", "timestamp_ns", "n_proposals", "no_proposal",
                   "top1_conf", "top1_detected", "primary_threshold"]
                  + [f"rank{r}_{w}" for r in range(1, 6) for w in ("conf", "bbox")])
        with open(os.path.join(odir, "proposals.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        dt = time.time() - t_bag
        print(f"[done] {bag}: frames={len(frames)} images={saved} "
              f"no_proposal={no_prop} {dt:.1f}s -> {odir}")
        grand[bag] = dict(saved=saved, frames=len(frames), no_prop=no_prop, sec=round(dt, 1))

    print("\n=== GRAND SUMMARY ===")
    tf = ti = tn = 0
    for bag, s in grand.items():
        print(f"  {bag:24s} frames={s.get('frames',0):5d} images={s.get('saved',0):6d} "
              f"no_proposal={s.get('no_prop',0):4d}")
        tf += s.get("frames", 0); ti += s.get("saved", 0); tn += s.get("no_prop", 0)
    print(f"  {'TOTAL':24s} frames={tf:5d} images={ti:6d} no_proposal={tn:4d}")


if __name__ == "__main__":
    main()
