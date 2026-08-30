#!/usr/bin/env python3
"""dump_box_embeddings.py — boxes.csv 의 확정된 박스에 대해 임베딩 재추출 (READ-ONLY).

negative prototype / 상관분석용. YOLO 는 다시 돌지 않고 저장된 좌표를 그대로 쓴다.
저장: features/box_emb.npz
  uid[N], cls11[N,384], pmean{2,9,11}[N,384] (mask fg patch 평균, L2 정규화), hsv[N,128]
"""
import csv, os, sys
import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE); REPO = os.path.dirname(os.path.dirname(ROOT))  # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism as yi                      # noqa: E402
import yolo_ism_object_n as o_n            # noqa: E402
sys.path.insert(0, HERE)
from dump_candidates import hsv_hist       # noqa: E402

BLOCKS = [2, 9, 11]
rows = list(csv.DictReader(open(os.path.join(ROOT, "features", "boxes.csv"))))
byimg = {}
for r in rows:
    byimg.setdefault(r["img"], []).append(r)

defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

uids, cls11, pm = [], [], {b: [] for b in BLOCKS}
hsv = []
for i, (img, rs) in enumerate(byimg.items()):
    bgr = cv2.imread(img)
    if bgr is None:
        continue
    norm = yi.normalize_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    bxs = [[int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])] for r in rs]
    crops, keep = [], []
    for r, bx in zip(rs, bxs):
        c = yi.crop_resize_pad(norm, bx)
        if c is not None:
            crops.append(c); keep.append((r, bx))
    if not crops:
        continue
    cl, patch = o_n.dinov2_blocks_forward(model, crops, device, BLOCKS)
    masks = yi.segment_boxes(seg, bgr, [b for _, b in keep], device)
    for k, (r, bx) in enumerate(keep):
        m = masks[k]
        uids.append(r["uid"]); cls11.append(cl[k].numpy())
        for b in BLOCKS:
            q = patch[b][k].cpu()
            fg = yi.masked_query_patches(q, m, bx, pool)[0] if m is not None else q
            v = fg.mean(0) if fg.shape[0] else q.mean(0)
            pm[b].append((v / (v.norm() + 1e-9)).numpy())
        x1, y1, x2, y2 = bx
        hsv.append(hsv_hist(bgr[y1:y2, x1:x2], m[y1:y2, x1:x2] if m is not None else None))
    if (i + 1) % 25 == 0:
        print(f"  {i+1}/{len(byimg)} images, {len(uids)} boxes")

out = os.path.join(ROOT, "features", "box_emb.npz")
np.savez_compressed(out, uid=np.array(uids), cls11=np.stack(cls11),
                    **{f"pmean{b}": np.stack(pm[b]) for b in BLOCKS}, hsv=np.stack(hsv))
print(f"[done] {len(uids)} boxes -> {out}")
