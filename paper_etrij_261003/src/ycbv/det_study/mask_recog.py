"""Mask-source study with recognition: each mask source followed by the same original SAM-6D object decision.

Mask sources (as det_study/mask_study.py):
  fastsam_full   FastSAM-x segment-everything masks (upstream SAM-6D settings)
  fastsam_msam   the same FastSAM-x boxes, masks re-made by MobileSAM
  text_msam      YOLO-World text boxes (method-1 prompts), top-3 per target object (score >= 0.02), MobileSAM masks
Then, identical for all three: upstream post-processing (min box 0.05, min mask 3e-4), DINOv2 descriptors
(--desc, default ViT-S as 개선안 2/3), semantic + appearance + geometric score, NMS per object, top-1 mask
per target object, answer when the ISM score > --thresh (0.5). No PEM.

Scored per (image, target object) of the BOP19 list against the GT visible mask of that object:
  correct = answered with mask IoU >= 0.5;  wrong = answered with IoU < 0.5 (other object / bad mask);
  missed = no answer. Reported: found % (correct / targets), wrong per image, precision, mean IoU of correct
answers, and per-image time (mask, descriptors, matching; CUDA-synchronised medians). Every top-1 score is
stored, so other thresholds can be read off the same run (--thresh only sets the headline).

    python det_study/mask_recog.py [--subset 25] [--desc dinov2_vits14] --out det_study/out/mask_recog_900.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import paths as P  # noqa: E402
import data as D  # noqa: E402
import run_orig as ROG  # noqa: E402
import proposers as PR  # noqa: E402

SR = HERE.parents[3] / "sam6d_realtime"
sys.path.insert(0, str(SR))
import yolo_ism as yi  # noqa: E402

TEST = P.YCBV / "test"
DEV = "cuda:0"


def sync():
    torch.cuda.synchronize()


class Source:
    """generate_masks(rgb) -> {"masks": (N,H,W) float, "boxes": (N,4)} or None, like the upstream segmentor."""

    def __init__(self, kind, fs, msam, text):
        self.kind, self.fs, self.msam, self.text, self.oids = kind, fs, msam, text, []

    def generate_masks(self, rgb):
        if self.kind == "fastsam_full":
            return self.fs.generate_masks(rgb)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self.kind == "fastsam_msam":
            det = self.fs.model.predict(rgb, **self.fs.args)[0]
            boxes = det.boxes.xyxy.tolist() if det.boxes is not None else []
        else:
            self.text.activate(self.oids)
            top = {}
            for d in sorted(self.text.predict(bgr, 0.02), key=lambda d: -d[2]):
                if len(top.setdefault(d[1], [])) < 3:
                    top[d[1]].append(d[0])
            boxes = [b for bs in top.values() for b in bs]
        if not boxes:
            return None
        ms = yi.segment_boxes(self.msam, bgr, boxes, DEV)
        keep = [j for j, m in enumerate(ms) if m is not None and m.any()]
        if not keep:
            return None
        return {"masks": torch.from_numpy(np.stack([ms[j] for j in keep])).float().to(DEV),
                "boxes": torch.tensor([boxes[j] for j in keep], dtype=torch.float32, device=DEV)}


def recognise(ism, oids, rgb, raw, K, ds):
    """upstream ISM up to the top-1 mask per object: {obj_id: (mask bool, score)} and stage times (ms)."""
    T = {}
    sync(); t0 = time.perf_counter()
    props = ism.segmentor_model.generate_masks(rgb)
    out = {}
    if props is None:
        sync(); T["mask"] = 1e3 * (time.perf_counter() - t0); T["desc"] = T["match"] = 0.0
        return out, T, 0
    dets = ROG.Detections(props)
    dets.remove_very_small_detections(config=ism.post_processing_config.mask_post_processing)
    n = len(dets)
    sync(); t1 = time.perf_counter(); T["mask"] = 1e3 * (t1 - t0)
    if n == 0:
        T["desc"] = T["match"] = 0.0
        return out, T, 0
    qd, qa = ism.descriptor_model(rgb, dets)
    sync(); t2 = time.perf_counter(); T["desc"] = 1e3 * (t2 - t1)
    sel, pio, sem, best = ism.compute_semantic_score(qd)
    dets.filter(sel); qa = qa[sel, :]
    if len(dets) > 0:
        appe, ref_aux = ism.compute_appearance_score(best, pio, qa)
        batch = {"depth": torch.from_numpy(raw.astype(np.int32)).unsqueeze(0).to(ROG.DEV),
                 "cam_intrinsic": torch.from_numpy(K).unsqueeze(0).to(ROG.DEV),
                 "depth_scale": torch.from_numpy(np.array(ds)).unsqueeze(0).to(ROG.DEV)}
        uv = ism.project_template_to_image(best, pio, batch, dets.masks)
        geo, vis = ism.compute_geometric_score(uv, dets, qa, ref_aux, visible_thred=ism.visible_thred)
        final = (sem + appe + geo * vis) / (1 + 1 + vis)
        dets.add_attribute("scores", final); dets.add_attribute("object_ids", pio)
        dets.apply_nms_per_object_id(nms_thresh=ism.post_processing_config.nms_thresh)
        for k in torch.unique(dets.object_ids).tolist():
            idx = torch.nonzero(dets.object_ids == k)[:, 0]
            j = int(idx[torch.argmax(dets.scores[idx])])
            out[oids[int(k)]] = ((dets.masks[j] > 0.5).cpu().numpy(), float(dets.scores[j]))
    sync(); T["match"] = 1e3 * (time.perf_counter() - t2)
    return out, T, n


_gt = {}


def gt_mask(s, i, o):
    root = TEST / f"{s:06d}"
    sc = _gt.setdefault(s, json.load(open(root / "scene_gt.json")))
    for k, g in enumerate(sc[str(i)]):
        if g["obj_id"] == o:
            return cv2.imread(str(root / "mask_visib" / f"{i:06d}_{k:06d}.png"), cv2.IMREAD_GRAYSCALE) > 0
    return None


def iou(a, b):
    return float(np.logical_and(a, b).sum()) / float(max(np.logical_or(a, b).sum(), 1))


def summarise(rows, n_img, thresh):
    tg = len(rows)
    ans = [r for r in rows if r["score"] is not None and r["score"] > thresh]
    cor = [r for r in ans if r["iou"] >= 0.5]
    return {"targets": tg, "answers": len(ans), "correct": len(cor), "wrong": len(ans) - len(cor),
            "found_pct": round(100 * len(cor) / max(tg, 1), 1),
            "precision_pct": round(100 * len(cor) / max(len(ans), 1), 1),
            "wrong_per_image": round((len(ans) - len(cor)) / max(n_img, 1), 3),
            "mean_iou_correct": round(float(np.mean([r["iou"] for r in cor])), 3) if cor else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--subset", type=int, default=0)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--desc", default="dinov2_vits14")
    ap.add_argument("--thresh", type=float, default=0.5)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    items = D.load_targets()
    if a.subset:
        items = items[::a.subset]
    if a.limit:
        items = items[:a.limit]
    print(f"{len(items)} images, {a.desc}, ISM > {a.thresh}", flush=True)

    ism, bank = ROG.build_ism(model_name=a.desc)
    fs = ism.segmentor_model                       # FastSAMUpstream
    msam = yi.build_segmentor(str(SR / "mobile_sam.pt"), DEV)
    text = PR.build("text:yolov8m-worldv2.pt:" + str(HERE / "out" / "prompts_best_m.json"))
    sources = {k: Source(k, fs, msam, text) for k in ("fastsam_full", "fastsam_msam", "text_msam")}
    cache = {}

    def run(kind, s, i, oids):
        bgr, raw, _, K, ds = D.load_image(s, i)
        ROG.select_bank(ism, bank, oids, cache)
        src = sources[kind]; src.oids = oids
        ism.segmentor_model = src
        return recognise(ism, oids, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), raw, K, ds)

    for s, i, oids in items[:a.warmup]:
        for k in sources:
            run(k, s, i, oids)

    rows = {k: [] for k in sources}
    times = {k: {"mask": [], "desc": [], "match": [], "props": []} for k in sources}
    for n, (s, i, oids) in enumerate(items):
        for k in sources:
            out, T, nprop = run(k, s, i, oids)
            for x in ("mask", "desc", "match"):
                times[k][x].append(T[x])
            times[k]["props"].append(nprop)
            for o in oids:
                g = gt_mask(s, i, o)
                m, sc = out.get(o, (None, None))
                rows[k].append({"scene": s, "im": i, "obj": o, "score": None if sc is None else round(sc, 4),
                                "iou": 0.0 if (m is None or g is None) else round(iou(m, g), 4)})
        if n % 50 == 0:
            print(n, {k: summarise(v, n + 1, a.thresh)["found_pct"] for k, v in rows.items()}, flush=True)

    res = {"images": len(items), "descriptor": a.desc, "thresh": a.thresh, "label": P.LABEL,
           "rule": "top-1 mask per target object with ISM score > thresh; correct when mask IoU >= 0.5 with the GT "
                   "visible mask of that object", "methods": {}}
    for k in sources:
        t = times[k]; tot = np.array(t["mask"]) + np.array(t["desc"]) + np.array(t["match"])
        r = summarise(rows[k], len(items), a.thresh)
        r["at_thresh"] = {str(th): summarise(rows[k], len(items), th)
                          for th in (0.2, 0.3, 0.4, 0.5, 0.6)}
        r["proposals_per_image"] = round(float(np.mean(t["props"])), 1)
        r["time_ms_median"] = {x: round(float(np.median(t[x])), 1) for x in ("mask", "desc", "match")}
        r["time_ms_median"]["total"] = round(float(np.median(tot)), 1)
        per = {}
        for row in rows[k]:
            p = per.setdefault(P.YCB_NAMES[row["obj"] - 1], [0, 0])
            p[1] += 1
            p[0] += int(row["score"] is not None and row["score"] > a.thresh and row["iou"] >= 0.5)
        r["per_object_found_pct"] = {o: round(100 * c / n_, 1) for o, (c, n_) in per.items()}
        res["methods"][k] = r
        print(k, {x: r[x] for x in ("found_pct", "precision_pct", "wrong_per_image", "mean_iou_correct",
                                    "proposals_per_image", "time_ms_median")}, flush=True)
    res["rows"] = rows
    json.dump(res, open(a.out, "w"), indent=1, ensure_ascii=False)
    print("->", a.out)


if __name__ == "__main__":
    main()
