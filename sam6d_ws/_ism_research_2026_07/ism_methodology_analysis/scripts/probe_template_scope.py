#!/usr/bin/env python3
"""probe_template_scope.py — is the CLS-argmax template the right template?

READ-ONLY probe. Imports yolo_ism / yolo_ism_object_n and reuses the production
config + cached template features; modifies nothing. For every detection that
passes the production semantic gate it runs the production MobileSAM step and
then computes the masked-appearance score against **all 42 templates** instead
of only the CLS-argmax one, so we can measure:

  * agreement rate  argmax_cls(template)  vs  argmax_appe(template)
  * appe(cls-top1)  vs  max over 42  vs  top-K(cls) aggregations
  * how many gate decisions would flip at the UNCHANGED production appe_gate
  * per-stage latency (cuda-synchronized) of yolo / dinov2 / mask / appe-1 / appe-42

Nothing here changes the pipeline; the production decision is recorded alongside.
"""
import argparse, csv, glob, json, os, statistics, sys, time

import cv2
import numpy as np
import torch

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))  # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism as yi                      # noqa: E402
import yolo_ism_object_n as o_n            # noqa: E402


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


class T:
    def __init__(self):
        self.acc = {}

    def add(self, k, ms):
        self.acc.setdefault(k, []).append(ms)


def stats(v):
    v = sorted(v)
    if not v:
        return {}
    return {"n": len(v), "median_ms": round(statistics.median(v), 3),
            "mean_ms": round(statistics.fmean(v), 3),
            "p90_ms": round(v[int(0.9 * (len(v) - 1))], 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame-globs", nargs="+", default=[
        os.path.join(REPO, "outputs/rgbd_imu_sdk_bag/SAM_loop1/input/frame_*/rgb.png"),
        os.path.join(REPO, "outputs/rgbd_imu_sdk_bag/SAM_occlusion/input/frame_*/rgb.png"),
        os.path.join(REPO, "outputs/pem_inputs/sam_105314/frame_*/rgb.png"),
    ])
    ap.add_argument("--max-frames-per-glob", type=int, default=20)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"))
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)

    defaults, objs = o_n.load_config(a.config)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    # template features resident on GPU (proposed placement) + CPU copy (current placement)
    gpu_tpl, gpu_seg, cpu_tpl = {}, {}, {}
    for o in objs:
        tp = o["tappe"]                      # list of [Np_i,384], block 11, from cache
        cpu_tpl[o["name"]] = tp
        flat = torch.cat(tp, 0).to(device)
        seg = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long) for i, t in enumerate(tp)]).to(device)
        gpu_tpl[o["name"]] = flat
        gpu_seg[o["name"]] = seg.unsqueeze(0)
        o["_tcls_gpu"] = o["tcls"].to(device)

    from ultralytics import YOLOWorld
    unique_prompts, groups = o_n.build_prompt_groups(objs)
    yolos = []
    for p in unique_prompts:                 # production build_ism_inputs_imu: one pass per prompt
        y = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
        y.set_classes([p])
        yolos.append(y)
    img_sz = int(defaults.get("imgsz", 640))
    min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)

    frames = []
    for g in a.frame_globs:
        fs = sorted(glob.glob(g))[: a.max_frames_per_glob]
        frames += [(os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(f)))) or "?", f)
                   for f in fs]
    print(f"[probe] {len(frames)} frames, {len(objs)} objects, {len(unique_prompts)} prompts, dev={device}")

    tm = T()
    rows = []
    for fi, (tag, fp) in enumerate(frames):
        warm = fi < a.warmup
        bgr = cv2.imread(fp)
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        sync(); t0 = time.perf_counter()
        norm_full = yi.normalize_rgb(rgb)
        sync(); t_pre = (time.perf_counter() - t0) * 1e3

        sync(); t0 = time.perf_counter()
        prompt_boxes = {i: [] for i in range(len(unique_prompts))}
        for pk, y in enumerate(yolos):
            r = y.predict(bgr, conf=min_score, imgsz=img_sz, verbose=False, device=device)
            if not (len(r) and r[0].boxes is not None and len(r[0].boxes)):
                continue
            b = r[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                prompt_boxes[pk].append(([x1, y1, x2, y2], float(b.conf[j])))
        sync(); t_yolo = (time.perf_counter() - t0) * 1e3

        n_fwd = n_mask = 0
        for pi, objs_here in groups.items():
            cand = sorted(prompt_boxes.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                sel = [(b, s) for (b, s) in cand if s >= float(o.get("score_threshold", 0.0))]
                sel = sel[: int(o.get("top_k", 3))]
                if not sel:
                    continue
                # ---- per-object DINOv2 (production recognize(): one forward per proposal) ----
                sync(); t0 = time.perf_counter()
                best = None
                for rank, (box, sc) in enumerate(sel, start=1):
                    crop = yi.crop_resize_pad(norm_full, box)
                    if crop is None:
                        continue
                    n_fwd += 1
                    cls_b, pb = o_n.dinov2_blocks_forward(model, [crop], device, o["_need_blocks"])
                    cls = cls_b[0]
                    sem = yi.semantic_score(cls, o["tcls"], o["match_topk"])
                    if best is None or sem > best[0]:
                        best = (sem, box, sc, rank, cls, {b: pb[b][0] for b in pb})
                sync(); t_dino = (time.perf_counter() - t0) * 1e3
                if best is None or best[0] < o["similarity_threshold"]:
                    continue

                sync(); t0 = time.perf_counter()
                mask = yi.segment_box(segmentor, bgr, best[1], device); n_mask += 1
                sync(); t_mask = (time.perf_counter() - t0) * 1e3
                if mask is None:
                    continue

                qpatch_cpu = best[5][11].cpu()
                q_fg_cpu = yi.masked_query_patches(qpatch_cpu, mask, best[1], pool)[0]
                if q_fg_cpu.shape[0] == 0:
                    continue
                q_fg_gpu = q_fg_cpu.to(device)

                cls_sims = o["tcls"] @ best[4]                    # [42]
                order = torch.argsort(cls_sims, descending=True)
                t_top1 = int(order[0])

                # --- CURRENT: single CLS-top1 template, ragged, on CPU ---
                sync(); t0 = time.perf_counter()
                appe_top1 = yi.masked_appe_score(q_fg_cpu, cpu_tpl[o["name"]][t_top1])
                sync(); t_appe1 = (time.perf_counter() - t0) * 1e3

                # --- PROPOSED: all 42 templates, one flat matmul, on GPU ---
                sync(); t0 = time.perf_counter()
                flat, seg = gpu_tpl[o["name"]], gpu_seg[o["name"]]
                sim = q_fg_gpu @ flat.T
                out = torch.full((q_fg_gpu.shape[0], 42), -1.0, device=device, dtype=sim.dtype)
                out.scatter_reduce_(1, seg.expand(q_fg_gpu.shape[0], -1), sim, reduce="amax")
                per_t = out.mean(dim=0).clamp(0, 1)               # [42] appe per template
                sync(); t_appe42 = (time.perf_counter() - t0) * 1e3

                per_t_c = per_t.cpu()
                a_argmax = int(torch.argmax(per_t_c))
                appe_max42 = float(per_t_c[a_argmax])
                appe_topk = {k: float(per_t_c[order[:k]].max()) for k in (1, 3, 5, 10)}
                appe_topk_mean = {k: float(per_t_c[order[:k]].mean()) for k in (3, 5, 10)}
                appe_rank_of_top1 = int((per_t_c > per_t_c[t_top1]).sum())   # 0 = cls-top1 is best

                if not warm:
                    tm.add("preprocess", t_pre); tm.add("yolo_world", t_yolo)
                    tm.add("dinov2_per_object", t_dino); tm.add("mobilesam_per_box", t_mask)
                    tm.add("appe_1tpl_cpu_ragged", t_appe1); tm.add("appe_42tpl_gpu_flat", t_appe42)
                rows.append({
                    "warmup": int(warm), "bag": tag, "frame": os.path.basename(os.path.dirname(fp)),
                    "object": o["name"], "yolo_conf": round(best[2], 4),
                    "n_proposals": len(sel), "sem": round(best[0], 4),
                    "nq_fg": int(q_fg_cpu.shape[0]),
                    "cls_top1_tpl": t_top1, "appe_argmax_tpl": a_argmax,
                    "tpl_agree": int(t_top1 == a_argmax),
                    "appe_cls_top1": round(appe_top1, 4),
                    "appe_flat_check": round(float(per_t_c[t_top1]), 4),
                    "appe_max42": round(appe_max42, 4),
                    "appe_topk1": round(appe_topk[1], 4), "appe_topk3": round(appe_topk[3], 4),
                    "appe_topk5": round(appe_topk[5], 4), "appe_topk10": round(appe_topk[10], 4),
                    "appe_topk3_mean": round(appe_topk_mean[3], 4),
                    "appe_topk5_mean": round(appe_topk_mean[5], 4),
                    "appe_min42": round(float(per_t_c.min()), 4),
                    "appe_mean42": round(float(per_t_c.mean()), 4),
                    "appe_rank_of_cls_top1": appe_rank_of_top1,
                    "appe_gate": o["appe_gate"],
                    "pass_current": int(appe_top1 >= o["appe_gate"]),
                    "pass_max42": int(appe_max42 >= o["appe_gate"]),
                    "pass_top5max": int(appe_topk[5] >= o["appe_gate"]),
                })
        if not warm:
            tm.add("n_dinov2_forward_per_frame", n_fwd)
            tm.add("n_mobilesam_calls_per_frame", n_mask)

    csv_path = os.path.join(a.out_dir, "template_scope_detections.csv")
    with open(csv_path, "w", newline="") as f:
        wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wtr.writeheader(); wtr.writerows(rows)
    timing = {k: stats(v) for k, v in tm.acc.items()}
    json.dump(timing, open(os.path.join(a.out_dir, "template_scope_timing.json"), "w"), indent=2)

    real = [r for r in rows if not r["warmup"]]
    agree = sum(r["tpl_agree"] for r in real)
    print(f"\n=== {len(real)} steady detections ===")
    print(f"template agreement (cls-top1 == appe-argmax): {agree}/{len(real)} = {agree/max(1,len(real)):.1%}")
    d = [r["appe_max42"] - r["appe_cls_top1"] for r in real]
    print(f"appe_max42 - appe_cls_top1: mean {statistics.fmean(d):.4f}  median {statistics.median(d):.4f}  max {max(d):.4f}")
    print(f"gate flips (fail->pass) with max42: "
          f"{sum(1 for r in real if not r['pass_current'] and r['pass_max42'])}")
    print(f"consistency check |appe_cls_top1 - flat[t_top1]| max = "
          f"{max(abs(r['appe_cls_top1']-r['appe_flat_check']) for r in real):.6f}")
    print(json.dumps(timing, indent=2))
    print(f"-> {csv_path}")


if __name__ == "__main__":
    main()
