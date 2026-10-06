"""Gate study 3: re-tune the ISM gate thresholds on held-out YCB-V frames (never the 900 test images).

Frames: det_recall.val_frames() (not in test_targets_bop19, >= 5 frames from every listed frame),
every 3rd -> about 188 frames. Each frame queries its own GT objects (visib_fract >= 0.1), as
the single-image protocol does. Only the ISM stage runs (deployed detector + gates + ownership).

Per (frame, object): TP = accepted with box IoU >= 0.5 to the GT visible or full box,
FP = accepted elsewhere, FN = not accepted. Grid over semantic x appearance x HSV threshold
(x ownership fallback off/on). Chosen: the most TP among settings whose precision is at most
1 point below the deployed setting's.

    python det_study/tune_gates.py --out det_study/out/tune_gates.json
"""
import argparse, itertools, json, sys
from pathlib import Path
import cv2, numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import paths as P  # noqa: E402
import run_ours as RO  # noqa: E402
import det_recall as DR  # noqa: E402


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    i = ix * iy; u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
    return i / u if u > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "out" / "tune_gates.json"))
    ap.add_argument("--step", type=int, default=3)
    a = ap.parse_args()
    frames = DR.val_frames()[::a.step]
    core, o_n = RO.build_core()
    import yolo_ism as yi
    data = []
    for s, i in frames:
        bgr, gts = DR.frame(s, i, "val")
        oids = sorted({g[0] for g in gts if g[3] >= 0.1})
        if oids:
            data.append((bgr, {g[0]: (DR.xyxy(g[1]), DR.xyxy(g[2])) for g in gts if g[3] >= 0.1}, oids))
    print(f"{len(data)} frames", flush=True)
    # detector output depends only on the frame and its object set: compute once
    cache = []
    for bgr, gt, oids in data:
        RO.activate(core, o_n, oids)
        h, w = bgr.shape[:2]
        with o_n.multi_label_nms(core.multi_label):
            res = core.yolo.predict(bgr, conf=core.min_score, imgsz=core.imgsz, verbose=False, device=core.device)
        pb = {k: [] for k in range(len(core.unique_prompts))}
        b = res[0].boxes
        for j in range(len(b) if b is not None else 0):
            xy = b.xyxy[j].tolist()
            x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
            x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
            pb[int(b.cls[j])].append(([x1, y1, x2, y2], float(b.conf[j])))
        for k in pb:
            pb[k].sort(key=lambda t: t[1], reverse=True)
        cache.append((list(core.unique_prompts), pb))
    grid = list(itertools.product([0.25, 0.30, 0.35], [0.50, 0.55, 0.605], [0.0, 0.06, 0.1214], [False, True]))
    out = {"frames": len(data), "rule": "TP: accepted, box IoU >= 0.5 with GT; FP: accepted elsewhere", "results": []}
    for sem, appe, hsv, fb in grid:
        for o in core._all_objs:
            o["similarity_threshold"] = sem; o["appe_v2_gate"] = appe; o["appe_gate"] = appe
            o["hsv_gate_threshold"] = hsv; o["hsv_gate_enabled"] = hsv > 0; o["assignment_fallback"] = fb
        tp = fp = fn = 0
        for (bgr, gt, oids), (_, pb) in zip(data, cache):
            RO.activate(core, o_n, oids)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            r = o_n.recognize_frame_auto(core.groups, pb, bgr, rgb, yi.normalize_rgb(rgb), core.model,
                                         core.device, core.segmentor, core.pool, core.tsim)
            for o in oids:
                d = r[P.obj_name(o)]
                if d.get("accepted") and d.get("box") is not None:
                    if max(iou(d["box"], gt[o][0]), iou(d["box"], gt[o][1])) >= 0.5:
                        tp += 1
                    else:
                        fp += 1
                else:
                    fn += 1
        row = {"sem": sem, "appe": appe, "hsv": hsv, "fallback": fb, "tp": tp, "fp": fp, "fn": fn,
               "recall": round(100 * tp / (tp + fn), 1), "precision": round(100 * tp / max(tp + fp, 1), 1)}
        out["results"].append(row)
        print(row, flush=True)
    base = next(r for r in out["results"] if r["sem"] == 0.35 and r["appe"] == 0.605 and r["hsv"] == 0.1214 and not r["fallback"])
    ok = [r for r in out["results"] if r["precision"] >= base["precision"] - 1.0]
    out["deployed"] = base
    out["chosen"] = max(ok, key=lambda r: (r["tp"], r["precision"]))
    out["chosen_no_fallback"] = max([r for r in ok if not r["fallback"]], key=lambda r: (r["tp"], r["precision"]))
    json.dump(out, open(a.out, "w"), indent=1)
    print("deployed", base); print("chosen", out["chosen"]); print("chosen (no fallback)", out["chosen_no_fallback"])


if __name__ == "__main__":
    main()
