#!/usr/bin/env python3
"""new_score_probe.py — per-candidate score table for the new-score study.

Runs YOLO-World (SEPARATE pass per object prompt, as build_ism_inputs_imu.py
does — a shared set_classes(all) pass lets one toy prompt NMS-suppress another)
over every frame of a bag, keeps each object's top_k candidates, and records for
EVERY candidate:

    sem                      semantic_score (CLS top-k mean vs 42 templates)
    sem_top1/top2/margin     raw best/second-best template cosine, and the gap
    sem_entropy              entropy of softmax(template cosines / tau)
    sem_viewvar              std of the 42 template cosines
    appe_b{2,9,11}_box       patch cosine vs best template, WHOLE box crop
    appe_b{2,9,11}_mask      same, restricted to MobileSAM foreground patches
    color_box / color_mask   CIELab a*b* histogram Bhattacharyya coeff vs the
                             best-matching of the 42 template histograms

NO GATES ARE APPLIED — every candidate is written out so the score
distributions (and any gate one might pick) can be studied offline.

Usage:
  python3 tools/new_score_probe.py \
      --bag  /path/to/bag_or_name --frames-dir /path/to/frames \
      --out  /path/to/output_dir
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np
import torch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

import yolo_ism as yi                      # noqa: E402
import yolo_ism_object_n as o_n            # noqa: E402

# Blocks we record. 2 = colour/low-level, 9 = mid texture, 11 = production gate.
PROBE_BLOCKS = [2, 9, 11]

# Colour descriptor: CIELab a*b* only (drop L* -> robust to shading/exposure).
COLOR_BINS = 32
CHROMA_FLOOR = 12.0      # |a*,b*| below this is near-grey; hue is unstable there
SOFTMAX_TAU = 0.02       # MUSE's relative-score temperature, for sem_entropy


# ---------------------------------------------------------------------------
# Colour descriptor
# ---------------------------------------------------------------------------
def _ab_pixels(bgr, mask=None, chroma_floor=CHROMA_FLOOR):
    """(a*, b*) of the selected pixels, or None if too few."""
    if bgr.size == 0:
        return None
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    a = lab[..., 1].astype(np.float32) - 128.0
    b = lab[..., 2].astype(np.float32) - 128.0
    sel = np.ones(a.shape, dtype=bool) if mask is None else mask.astype(bool)
    if chroma_floor > 0:
        chromatic = sel & (np.sqrt(a * a + b * b) >= chroma_floor)
        # apply the floor only if it leaves something to look at (white/grey
        # objects like the milk carton would otherwise vanish entirely)
        if chromatic.sum() >= 30:
            sel = chromatic
    if sel.sum() < 10:
        return None
    return a[sel], b[sel]


def color_descriptor(bgr, mask=None):
    """Several colour representations of one ROI, so the metrics can be compared.

    Bin-to-bin metrics (Bhattacharyya) are brittle across the render-vs-real
    white-balance shift: a modest hue offset moves mass into neighbouring bins
    and the coefficient collapses to ~0. We therefore record, per ROI,
      h32/h16     fine and coarse 2D a*b* histograms
      h16s        h16 blurred by one bin (a cheap cross-bin tolerance)
      ma/mb       1D marginals, for a true cross-bin Wasserstein distance
    so the full run can decide empirically which metric separates best.
    """
    px = _ab_pixels(bgr, mask)
    if px is None:
        return None
    a, b = px
    rng = [[-128, 127], [-128, 127]]
    h32, _, _ = np.histogram2d(a, b, bins=32, range=rng)
    h16, _, _ = np.histogram2d(a, b, bins=16, range=rng)
    ma, _ = np.histogram(a, bins=64, range=(-128, 127))
    mb, _ = np.histogram(b, bins=64, range=(-128, 127))

    def n(x):
        s = x.sum()
        return (x / s).astype(np.float32) if s > 0 else None

    h32, h16, ma, mb = n(h32), n(h16), n(ma), n(mb)
    if h32 is None or h16 is None:
        return None
    h16s = cv2.GaussianBlur(h16, (0, 0), 1.0)
    h16s = h16s / max(h16s.sum(), 1e-12)
    return {"h32": h32, "h16": h16, "h16s": h16s.astype(np.float32),
            "ma": ma, "mb": mb,
            "chroma": float(np.sqrt(a * a + b * b).mean())}


_MARGINAL_CENTERS = (np.arange(64) + 0.5) * (255.0 / 64.0) - 128.0
_MARGINAL_W = 255.0 / 64.0


def _w1(p, q):
    """1D Wasserstein distance between two normalised histograms, in a*b* units."""
    return float(np.abs(np.cumsum(p) - np.cumsum(q)).sum() * _MARGINAL_W)


CHROMA_MIN_VIEW = 3.0     # a template view flatter than this carries no colour


def color_view_distances(desc, tdescs):
    """1D-Wasserstein distance from the ROI to EVERY template view.

    Cross-bin (transport) rather than bin-to-bin: a bin-to-bin coefficient
    collapses to ~0 under the render-vs-real white-balance shift, whereas the
    cumulative-histogram L1 (= 1D Wasserstein-1) degrades linearly.
    """
    if desc is None or tdescs is None:
        return None
    ca = np.cumsum(desc["ma"])
    cb = np.cumsum(desc["mb"])
    da = np.abs(ca[None, :] - tdescs["cum_ma"]).sum(axis=1) * _MARGINAL_W
    db = np.abs(cb[None, :] - tdescs["cum_mb"]).sum(axis=1) * _MARGINAL_W
    return 0.5 * (da + db)


def color_variants(desc, tdescs, k=5):
    """Several ways of turning the 42 per-view distances into one score.

    FIX 1 — drop achromatic views. A view whose mean chroma is ~0 (a can lid or
    bottle cap seen end-on) matches ANY neutral object at distance ~0, so with a
    min-over-views rule it acts as a wildcard and hands false positives the top
    score. Measured: Sikhye view 0 has chroma 0.0 while its real body views have
    chroma 51-56, which is exactly why Sikhye's colour AUC came out at 0.102
    (i.e. reversed).

    FIX 2 — stop using the minimum. One wildcard view can dominate a min; a
    top-k mean cannot be carried by a single view.
    """
    out = {}
    d = color_view_distances(desc, tdescs)
    if d is None or len(d) == 0:
        return {f"color_{a}_{b}": 0.0
                for a in ("min", "top5", "med") for b in ("all", "filt")}
    chroma = tdescs.get("chroma")
    keep = np.ones(len(d), bool) if chroma is None else (chroma >= CHROMA_MIN_VIEW)
    if keep.sum() < 3:          # object really is achromatic — keep everything
        keep = np.ones(len(d), bool)
    for tag, dd in (("all", d), ("filt", d[keep])):
        kk = min(k, len(dd))
        out[f"color_min_{tag}"] = float(np.exp(-dd.min() / 25.0))
        out[f"color_top5_{tag}"] = float(np.exp(-np.sort(dd)[:kk].mean() / 25.0))
        out[f"color_med_{tag}"] = float(np.exp(-np.median(dd) / 25.0))
    out["n_views_kept"] = int(keep.sum())
    return out


def color_emd_sim(desc, tdescs):
    """Historical score: min over ALL views (kept for A/B comparison)."""
    d = color_view_distances(desc, tdescs)
    if d is None:
        return 0.0, 999.0
    m = float(d.min())
    return float(np.exp(-m / 25.0)), m


def color_similarities(desc, tdescs):
    """Best-of-N-templates similarity for each colour metric.

    Returns (bc32, bc16, bc16s, emd_sim, w1_dist). `w1_dist` is the raw mean
    Wasserstein distance over the a*/b* marginals (lower = closer); `emd_sim`
    maps it to [0,1] via exp(-d/25) so it can be multiplied with the other
    channels. The raw distance is kept so the mapping can be re-chosen offline.
    """
    if desc is None or not tdescs:
        return 0.0, 0.0, 0.0, 0.0, 999.0
    bc32 = float(np.sqrt(desc["h32"][None] * tdescs["h32"]).sum(axis=(1, 2)).max())
    bc16 = float(np.sqrt(desc["h16"][None] * tdescs["h16"]).sum(axis=(1, 2)).max())
    bc16s = float(np.sqrt(desc["h16s"][None] * tdescs["h16s"]).sum(axis=(1, 2)).max())
    d = min(0.5 * (_w1(desc["ma"], tdescs["ma"][i]) + _w1(desc["mb"], tdescs["mb"][i]))
            for i in range(len(tdescs["ma"])))
    return bc32, bc16, bc16s, float(np.exp(-d / 25.0)), d


def build_template_colors(template_dir, cache_path, rebuild=False):
    """Stacked colour descriptors for all templates of one object."""
    def _with_cums(d):
        d["cum_ma"] = np.cumsum(d["ma"], axis=1)
        d["cum_mb"] = np.cumsum(d["mb"], axis=1)
        return d

    if os.path.isfile(cache_path) and not rebuild:
        z = np.load(cache_path)
        if "chroma" in z.files:
            return _with_cums({k: z[k] for k in
                               ("h32", "h16", "h16s", "ma", "mb", "chroma")})
    from PIL import Image
    descs = []
    for rp in yi._template_paths(template_dir):
        idx = os.path.basename(rp).split("_")[1].split(".")[0]
        mp = os.path.join(template_dir, f"mask_{idx}.png")
        rgb = np.array(Image.open(rp).convert("RGB"))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        mask = np.array(Image.open(mp).convert("L")) > 0 \
            if os.path.isfile(mp) else None
        d = color_descriptor(bgr, mask)
        if d is not None:
            descs.append(d)
    if not descs:
        return None
    out = {k: np.stack([d[k] for d in descs]).astype(np.float32)
           for k in ("h32", "h16", "h16s", "ma", "mb")}
    out["chroma"] = np.array([d["chroma"] for d in descs], np.float32)
    os.makedirs(os.path.dirname(cache_path), exist_ok=True)
    np.savez_compressed(cache_path, **out)
    return _with_cums(out)


# ---------------------------------------------------------------------------
# Probe
# ---------------------------------------------------------------------------
def iter_frames(frames_dir, stride, max_frames):
    """Yield (idx, name, bgr).

    Supports the per-frame directory layout used by the PEM inputs
    (`<dir>/frame_000123/rgb.png`) as well as a flat directory of images.
    A flat directory is filtered to `frame_*` so stray composites (e.g. the
    SLAM+SAM visualisation frames, which are NOT raw camera images) cannot be
    picked up by accident.
    """
    import glob
    per_frame = sorted(glob.glob(os.path.join(frames_dir, "frame_*", "rgb.png")))
    if per_frame:
        files = [(os.path.basename(os.path.dirname(p)), p) for p in per_frame]
    else:
        flat = sorted(glob.glob(os.path.join(frames_dir, "frame_*.png")) +
                      glob.glob(os.path.join(frames_dir, "frame_*.jpg")))
        files = [(os.path.splitext(os.path.basename(p))[0], p) for p in flat]
    if not files:
        raise SystemExit(f"[frames] no frame_* images under {frames_dir}")
    print(f"[frames] {len(files)} frames from {frames_dir}")
    emitted = 0
    for i, (name, path) in enumerate(files):
        if i % stride != 0:
            continue
        if max_frames and emitted >= max_frames:
            break
        img = cv2.imread(path)
        if img is None:
            continue
        emitted += 1
        yield i, name, img


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", required=True)
    p.add_argument("--frames-dir", default=None)
    p.add_argument("--out", required=True, help="output directory")
    p.add_argument("--config", default=o_n.DEFAULT_CONFIG)
    p.add_argument("--stride", type=int, default=1, help="1 = every frame")
    p.add_argument("--max-frames", type=int, default=0)
    p.add_argument("--topic", default="/camera/camera/color/image_raw")
    p.add_argument("--device", default=None)
    p.add_argument("--rebuild-features", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    defaults, objs = o_n.load_config(args.config)
    device = args.device or defaults.get("device", "cuda:0")
    device = device if torch.cuda.is_available() else "cpu"

    print(f"[dinov2] loading (device={device})")
    ckpt = defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT
    model = yi.build_dinov2(ckpt, device)

    # force every object to carry all probe blocks
    for o in objs:
        o["appe_blocks"] = PROBE_BLOCKS
        o["nms_rank_blocks"] = PROBE_BLOCKS
    objs = o_n.prepare_objects(objs, model, device, args.rebuild_features)

    # colour prototypes
    color_dir = os.path.join(args.out, "_template_colors")
    for o in objs:
        o["tcolor"] = build_template_colors(
            o["template_dir"], os.path.join(color_dir, f"{o['name']}_ab.npz"),
            args.rebuild_features)
        n = 0 if o["tcolor"] is None else len(o["tcolor"]["h32"])
        print(f"[color] {o['name']:20s} {n} template colour descriptors")

    seg_w = o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt"))
    print(f"[seg] loading {seg_w}")
    segmentor = yi.build_segmentor(seg_w, device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    from ultralytics import YOLOWorld
    weights = o_n._abspath(defaults.get("weights", "yolov8m-worldv2.pt"))

    # ONE model per prompt, set_classes called once at construction. A shared
    # set_classes(all) pass lets a stronger prompt NMS-suppress a visually
    # similar object (bear prompt swallowed the white rabbit) — and calling
    # set_classes() repeatedly after the model is on CUDA errors out.
    def _mk(prompt):
        try:
            y = YOLOWorld(weights)
        except Exception:                                    # pragma: no cover
            y = YOLOWorld("yolov8s-worldv2.pt")
        y.set_classes([prompt])
        return y

    yolo_of = {o["name"]: _mk(o["yolo_prompt"]) for o in objs}
    img_sz = int(defaults.get("imgsz", 640))
    print(f"[yolo] imgsz={img_sz}, SEPARATE pass per prompt ({len(objs)} objects)")

    fields = ["frame_idx", "frame_name", "object", "prompt", "rank", "yolo_conf",
              "x1", "y1", "x2", "y2", "box_area", "mask_area",
              "sem", "sem_top1", "sem_top2", "sem_margin", "sem_entropy",
              "sem_viewvar", "best_t"]
    for v in ("box", "mask"):
        fields += [f"color_bc32_{v}", f"color_bc16_{v}", f"color_bc16s_{v}",
                   f"color_emd_{v}", f"color_w1_{v}"]
    # FIX 1 (drop achromatic views) x FIX 2 (min -> top-k mean / median)
    fields += [f"color_{a}_{b}" for a in ("min", "top5", "med")
               for b in ("all", "filt")] + ["n_views_kept"]
    for b in PROBE_BLOCKS:
        fields += [f"appe_b{b}_box", f"appe_b{b}_mask"]

    csv_path = os.path.join(args.out, "candidates.csv")
    fh = open(csv_path, "w", newline="")
    writer = csv.DictWriter(fh, fieldnames=fields)
    writer.writeheader()

    # Cross-object scoring: EVERY candidate scored against EVERY object's
    # templates. Scoring a candidate only against its own object's templates
    # cannot answer "can this channel tell the objects apart?" — both a real
    # choco box and a brown carton score well against their own best match.
    # The discriminative question is whether the correct object wins the
    # 10-way comparison, which needs the full score vector per candidate.
    xfields = ["frame_idx", "cand_id", "src_object", "tgt_object", "rank",
               "sem", "appe_b2", "appe_b9", "appe_b11", "color_emd", "color_w1"]
    xfh = open(os.path.join(args.out, "cross_scores.csv"), "w", newline="")
    xw = csv.DictWriter(xfh, fieldnames=xfields)
    xw.writeheader()
    cand_id = 0

    n_frames = n_rows = 0
    for frame_idx, fname, bgr in iter_frames(
            args.frames_dir, max(1, args.stride), args.max_frames):
        n_frames += 1
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)

        # ---- one YOLO pass PER prompt (never a shared set_classes(all)) ----
        cands = {}          # object name -> [(box, conf), ...]
        for o in objs:
            res = yolo_of[o["name"]].predict(
                bgr, conf=float(o.get("score_threshold", 0.02)),
                imgsz=img_sz, verbose=False, device=device)
            sel = []
            if res and res[0].boxes is not None and len(res[0].boxes):
                bx = res[0].boxes
                order = torch.argsort(bx.conf, descending=True)
                for j in order[: int(o.get("top_k", 3))]:
                    x1, y1, x2, y2 = [int(v) for v in bx.xyxy[j].tolist()]
                    sel.append(([x1, y1, x2, y2], float(bx.conf[j])))
            cands[o["name"]] = sel

        # ---- one MobileSAM call for ALL boxes of the frame ----
        flat = [(o["name"], k, box, conf)
                for o in objs for k, (box, conf) in enumerate(cands[o["name"]], 1)]
        masks = yi.segment_boxes(segmentor, bgr, [f[2] for f in flat], device) \
            if flat else []

        # ---- DINOv2 once per candidate crop ----
        for (oname, rank, box, conf), mask in zip(flat, masks):
            o = next(x for x in objs if x["name"] == oname)
            crop = yi.crop_resize_pad(norm_full, box)
            if crop is None:
                continue
            cls_b, pb = o_n.dinov2_blocks_forward(model, [crop], device, PROBE_BLOCKS)
            cls = cls_b[0]
            qblocks = {b: pb[b][0].cpu() for b in pb}

            sims = (o["tcls"] @ cls).float()                     # [42]
            k = min(int(o.get("match_topk", 5)), sims.shape[0])
            sem = float(torch.topk(sims, k).values.mean())
            top2 = torch.topk(sims, min(2, sims.shape[0])).values
            sem_top1 = float(top2[0])
            sem_top2 = float(top2[1]) if top2.numel() > 1 else 0.0
            probs = torch.softmax(sims / SOFTMAX_TAU, dim=0)
            sem_entropy = float(-(probs * (probs + 1e-12).log()).sum())
            best_t = int(torch.argmax(sims))

            x1, y1, x2, y2 = box
            x1c, y1c = max(0, x1), max(0, y1)
            x2c, y2c = min(bgr.shape[1], x2), min(bgr.shape[0], y2)
            roi = bgr[y1c:y2c, x1c:x2c]
            roi_mask = None if mask is None else mask[y1c:y2c, x1c:x2c]

            row = {
                "frame_idx": frame_idx, "frame_name": fname, "object": oname,
                "prompt": o["yolo_prompt"], "rank": rank, "yolo_conf": round(conf, 5),
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "box_area": max(0, (x2 - x1)) * max(0, (y2 - y1)),
                "mask_area": 0 if mask is None else int(mask.sum()),
                "sem": round(sem, 5), "sem_top1": round(sem_top1, 5),
                "sem_top2": round(sem_top2, 5),
                "sem_margin": round(sem_top1 - sem_top2, 5),
                "sem_entropy": round(sem_entropy, 5),
                "sem_viewvar": round(float(sims.std()), 5),
                "best_t": best_t,
            }
            desc_by_tag = {}
            for tag, m in (("box", None), ("mask", roi_mask)):
                dsc = color_descriptor(roi, m)
                desc_by_tag[tag] = dsc
                bc32, bc16, bc16s, emd, w1 = color_similarities(dsc, o["tcolor"])
                row[f"color_bc32_{tag}"] = round(bc32, 5)
                row[f"color_bc16_{tag}"] = round(bc16, 5)
                row[f"color_bc16s_{tag}"] = round(bc16s, 5)
                row[f"color_emd_{tag}"] = round(emd, 5)
                row[f"color_w1_{tag}"] = round(w1, 3)
            # the fix variants are evaluated on the MASKED descriptor
            for k, v in color_variants(desc_by_tag["mask"], o["tcolor"]).items():
                row[k] = round(v, 5) if isinstance(v, float) else v
            for b in PROBE_BLOCKS:
                tb = o["tappe_blocks"][b][best_t]
                row[f"appe_b{b}_box"] = round(
                    yi.masked_appe_score(qblocks[b], tb), 5)
                if mask is not None:
                    q_fg, _ = yi.masked_query_patches(qblocks[b], mask, box, pool)
                    row[f"appe_b{b}_mask"] = round(yi.masked_appe_score(q_fg, tb), 5)
                else:
                    row[f"appe_b{b}_mask"] = 0.0
            writer.writerow(row)
            n_rows += 1

            # ---- cross-object: same crop vs every object's templates ----
            desc_mask = color_descriptor(roi, roi_mask)
            q_fg = {}
            for b in PROBE_BLOCKS:
                q_fg[b] = yi.masked_query_patches(qblocks[b], mask, box, pool)[0] \
                    if mask is not None else qblocks[b]
            for oo in objs:
                s_oo = (oo["tcls"] @ cls).float()
                kk = min(int(oo.get("match_topk", 5)), s_oo.shape[0])
                bt = int(torch.argmax(s_oo))
                emd, w1 = color_emd_sim(desc_mask, oo["tcolor"])
                xrow = {"frame_idx": frame_idx, "cand_id": cand_id,
                        "src_object": oname, "tgt_object": oo["name"],
                        "rank": rank,
                        "sem": round(float(torch.topk(s_oo, kk).values.mean()), 5),
                        "color_emd": round(emd, 5), "color_w1": round(w1, 3)}
                for b in PROBE_BLOCKS:
                    xrow[f"appe_b{b}"] = round(
                        yi.masked_appe_score(q_fg[b], oo["tappe_blocks"][b][bt]), 5)
                xw.writerow(xrow)
            cand_id += 1

        if n_frames % 25 == 0:
            print(f"  frame {n_frames} ({fname}) rows={n_rows}", flush=True)
            fh.flush(); xfh.flush()

    fh.close(); xfh.close()
    print(f"[done] {n_frames} frames, {n_rows} candidate rows -> {csv_path}")


if __name__ == "__main__":
    main()
