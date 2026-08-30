#!/usr/bin/env python3
"""eval_feature_variants.py — 특징 변형 변별력 실측 (READ-ONLY, 학습 0).

사람 검증 라벨셋(POS=실제 대상 위 / NEG=다른 물체·부재)에서 각 특징의 AUROC 를 잰다.
"임계를 어디 두느냐"와 무관하게 **특징 자체가 참/거짓을 가르는 능력**만 본다.

변형
  [해상도]  crop 입력 224(현행) vs 336 vs 448   ← DINOv2 는 img_size=518 로 생성돼 있어 가능
  [블록]    block 11(현행) / 9 / 2 / 2+9 / 2+9+11 평균
  [집계]    semantic top5(현행) / top1 / mean ;  appe cls-top1(현행) / max42 / mean42
  [화소]    HSV hist(현행 게이트) · Lab 모멘트 · 그래디언트 방향히스토그램 · LBP 유사
  [타모델]  CLIP ViT-B/32 이미지 임베딩 (템플릿 렌더와 코사인)
산출: outputs/phase30_feature_study/{csv,metrics}/
"""
import argparse, csv, glob, json, os, sys
from collections import defaultdict
import numpy as np
import cv2, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism as yi, yolo_ism_object_n as o_n
OUT = os.path.join(REPO, "outputs", "phase30_feature_study")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
LAB = os.path.join(OUT, "csv", "labeled_crops.csv")


def auroc(pos, neg):
    pos = np.asarray(pos, float); neg = np.asarray(neg, float)
    if not len(pos) or not len(neg): return float("nan")
    a = np.concatenate([pos, neg]); u, inv, cnt = np.unique(a, return_inverse=True, return_counts=True)
    cs = np.cumsum(cnt); st = cs - cnt; avg = (st + cs + 1) / 2.0
    r = avg[inv]; P = len(pos)
    return float((r[:P].sum() - P * (P + 1) / 2.0) / (P * len(neg)))


def crop_resize(norm_full, box, size):
    return yi.crop_resize_pad(norm_full, list(box), target=size)


def pixel_feats(bgr, box, mask=None):
    x1, y1, x2, y2 = box
    c = bgr[y1:y2, x1:x2]
    if c.size == 0: return None
    g = cv2.cvtColor(c, cv2.COLOR_BGR2GRAY)
    g = cv2.resize(g, (64, 64))
    # 그래디언트 방향 히스토그램
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0); gy = cv2.Sobel(g, cv2.CV_32F, 0, 1)
    ang = (np.arctan2(gy, gx) * 180 / np.pi) % 180; mag = np.sqrt(gx**2 + gy**2)
    hog, _ = np.histogram(ang, bins=18, range=(0, 180), weights=mag)
    hog = hog / (hog.sum() + 1e-6)
    # LBP 유사 (8이웃 부호 패턴)
    lbp = np.zeros_like(g, dtype=np.uint8)
    for i, (dy, dx) in enumerate([(-1,-1),(-1,0),(-1,1),(0,1),(1,1),(1,0),(1,-1),(0,-1)]):
        sh = np.roll(np.roll(g, dy, 0), dx, 1)
        lbp |= ((sh >= g).astype(np.uint8) << i)
    lh, _ = np.histogram(lbp, bins=32, range=(0, 256)); lh = lh / (lh.sum() + 1e-6)
    # Lab 모멘트
    lab = cv2.cvtColor(cv2.resize(c, (64, 64)), cv2.COLOR_BGR2LAB).astype(np.float32)
    mom = np.concatenate([[lab[..., k].mean(), lab[..., k].std()] for k in range(3)])
    mom = mom / (np.linalg.norm(mom) + 1e-6)
    return dict(hog=hog, lbp=lh, lab=mom)



def masked_fg(qpatch, mask_full, box, size, dev):
    """해상도 size 에 맞춘 foreground 패치 선택 (grid = size/PATCH)."""
    mt = torch.from_numpy(mask_full[None].astype(np.float32))
    mc = yi.crop_resize_pad(mt, list(box), target=size, mode="nearest")
    if mc is None:
        return qpatch
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    cover = pool(mc.unsqueeze(0))[0].flatten()
    valid = cover > yi.VALID_PATCH_THRESH
    if valid.numel() != qpatch.shape[0]:
        return qpatch
    sel = qpatch[valid]
    return sel if sel.shape[0] > 0 else qpatch


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--sizes", nargs="+", type=int, default=[224, 336, 448])
    a = ap.parse_args()
    rows = list(csv.DictReader(open(LAB)))
    print(f"라벨셋 {len(rows)} (POS {sum(1 for r in rows if r['label']=='POS')})")
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(objs, model, dev, False)
    by = {o["name"]: o for o in objs}
    BLOCKS = [2, 9, 11]
    for o in objs:
        o["_tcls"] = o["tcls"].to(dev)
        o["_flat"] = {b: torch.cat(o["tappe_blocks"][b], 0).to(dev) for b in BLOCKS}
        o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                               for i, t in enumerate(o["tappe_blocks"][11])]).to(dev)
        o["_nview"] = len(o["tappe_blocks"][11])
    pool_op = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    # 프레임별 묶기
    byframe = defaultdict(list)
    for r in rows: byframe[(r["bag"], int(r["frame"]))].append(r)
    feats = defaultdict(lambda: {"POS": [], "NEG": []})
    clipmod = None
    try:
        from ultralytics.nn.text_model import build_text_model
        clipmod = build_text_model("clip:ViT-B/32", torch.device(dev))
        from PIL import Image as PILImage
        # 템플릿 CLIP 임베딩
        clip_t = {}
        for o in objs:
            ps = sorted(glob.glob(os.path.join(o["template_dir"], "rgb_*.png")))[:42]
            ims = [clipmod.image_preprocess(PILImage.open(p).convert("RGB")) for p in ps]
            with torch.no_grad():
                f = clipmod.model.encode_image(torch.stack(ims).to(dev)).float()
            clip_t[o["name"]] = torch.nn.functional.normalize(f, dim=-1).cpu()
    except Exception as e:
        print("CLIP skip:", e); clipmod = None

    for n, ((ds, fr), rs) in enumerate(byframe.items(), 1):
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None: continue
        norm = yi.normalize_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        boxes = [(int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])) for r in rs]
        masks = yi.segment_boxes(
            yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
            if n == -1 else _SEG[0], bgr, [list(b) for b in boxes], dev)
        for SZ in a.sizes:
            crops, keep = [], []
            for i, b in enumerate(boxes):
                c = crop_resize(norm, b, SZ)
                if c is not None: crops.append(c); keep.append(i)
            if not crops: continue
            cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, dev, BLOCKS)
            for ci, i in enumerate(keep):
                r = rs[i]; L = r["label"]; o = by[r["class_name"]]; b = boxes[i]; mk = masks[i]
                sims = (o["_tcls"] @ cls_all[ci].to(dev)).cpu()
                srt = torch.sort(sims, descending=True).values
                feats[f"sem_top5@{SZ}"][L].append(float(srt[:5].mean()))
                if SZ == a.sizes[0]:
                    feats["sem_top1@224"][L].append(float(srt[0]))
                    feats["sem_mean@224"][L].append(float(sims.mean()))
                t1 = int(torch.argmax(sims))
                pv = {}
                for blk in BLOCKS:
                    qp = patch_all[blk][ci]
                    fg = masked_fg(qp.cpu(), mk, b, SZ, dev).to(dev) if mk is not None else qp
                    if fg.shape[0] == 0: pv[blk] = torch.zeros(o["_nview"]); continue
                    sim = fg @ o["_flat"][blk].T
                    per = torch.full((fg.shape[0], o["_nview"]), -1.0, device=dev, dtype=sim.dtype)
                    per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(fg.shape[0], -1), sim, reduce="amax")
                    pv[blk] = per.mean(0).clamp(0, 1).cpu()
                for blk in BLOCKS:
                    feats[f"appe_b{blk}@{SZ}"][L].append(float(pv[blk][t1]))
                feats[f"appe_b2+9@{SZ}"][L].append(float((pv[2][t1] + pv[9][t1]) / 2))
                feats[f"appe_b2+9+11@{SZ}"][L].append(float((pv[2][t1] + pv[9][t1] + pv[11][t1]) / 3))
                if SZ == a.sizes[0]:
                    feats["appe_max42@224"][L].append(float(pv[11].max()))
                    feats["appe_mean42@224"][L].append(float(pv[11].mean()))
        # 화소 특징 + CLIP (해상도 무관, 1회)
        for i, r in enumerate(rs):
            L = r["label"]; b = boxes[i]; o = by[r["class_name"]]
            pf = pixel_feats(bgr, b)
            if pf is not None and o.get("_pixref") is None:
                pass
            feats["hsv_gate(현행)"][L].append(float(r["hsv"]))
            if clipmod is not None:
                from PIL import Image as PILImage
                x1, y1, x2, y2 = b; c = bgr[y1:y2, x1:x2]
                if c.size:
                    im = PILImage.fromarray(cv2.cvtColor(c, cv2.COLOR_BGR2RGB))
                    with torch.no_grad():
                        e = clipmod.model.encode_image(clipmod.image_preprocess(im).unsqueeze(0).to(dev)).float()
                    e = torch.nn.functional.normalize(e, dim=-1).cpu()
                    s = (clip_t[r["class_name"]] @ e.T).squeeze(1)
                    feats["clip_top5"][L].append(float(torch.sort(s, descending=True).values[:5].mean()))
        if n % 40 == 0: print(f"  {n}/{len(byframe)} frames")

    res = []
    for k, d in feats.items():
        if len(d["POS"]) < 20 or len(d["NEG"]) < 20: continue
        res.append({"feature": k, "AUROC": round(auroc(d["POS"], d["NEG"]), 4),
                    "n_pos": len(d["POS"]), "n_neg": len(d["NEG"]),
                    "pos_median": round(float(np.median(d["POS"])), 4),
                    "neg_median": round(float(np.median(d["NEG"])), 4)})
    res.sort(key=lambda x: -x["AUROC"])
    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True); os.makedirs(os.path.join(OUT, "metrics"), exist_ok=True)
    with open(os.path.join(OUT, "csv", "feature_auroc.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(res[0].keys())); w.writeheader(); w.writerows(res)
    json.dump(res, open(os.path.join(OUT, "metrics", "feature_auroc.json"), "w"), indent=2, ensure_ascii=False)
    print("\n=== 특징별 변별력 (AUROC, 높을수록 참/거짓 잘 가름) ===")
    for r in res:
        print(f"  {r['feature']:22s} {r['AUROC']:.4f}   pos중앙 {r['pos_median']:.3f} / neg중앙 {r['neg_median']:.3f}")
    print(f"-> {OUT}")


_SEG = [None]
if __name__ == "__main__":
    import yolo_ism as _yi, yolo_ism_object_n as _on, torch as _t
    _d, _o = _on.load_config(_on.DEFAULT_CONFIG)
    _dev = _d.get("device", "cuda:0") if _t.cuda.is_available() else "cpu"
    _SEG[0] = _yi.build_segmentor(_on._abspath(_d.get("seg_weights", "mobile_sam.pt")), _dev)
    main()
