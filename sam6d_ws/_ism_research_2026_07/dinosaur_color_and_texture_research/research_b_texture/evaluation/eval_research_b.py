#!/usr/bin/env python3
"""eval_research_b.py — DINOv2 texture verifier: choco vs brown box. READ-ONLY, GPU.

Labeled set (provisional claude-vision, box_labels_merged.csv): 29 accepted-as-choco candidates
= 10 true choco (TP) vs 19 non-choco (17 carton brown-box + 2 milk) FP. Extra negatives: carton
boxes rejected-as-choco. Compares texture aggregations P0,P2,P3,P5,P8 against choco templates
(blocks 2 & 11), reports AUROC choco-vs-notchoco. Object-agnostic algorithm; no real-image ref.

NOTE: provisional labels + small N (~29). Treat AUROC as indicative, not definitive.
"""
import csv, os, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
import cv2
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
RB = os.path.dirname(HERE); ROOT = os.path.dirname(RB)
RSRCH = os.path.dirname(ROOT); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism as yi                # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402

CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
LBL = os.path.join(RSRCH, "ism_accuracy_observation", "labels", "box_labels_merged.csv")
OUT = os.path.join(RB, "results"); os.makedirs(OUT, exist_ok=True)

# ---- labeled choco-decision set ----
rows = list(csv.DictReader(open(LBL)))
data = []   # (uid, ds, frame, box, is_choco)
for r in rows:
    acc_choco = "ACC:choco_hazelnut_high" in r["claims"]
    rej_choco = "rej:choco_hazelnut_high" in r["claims"]
    if not (acc_choco or rej_choco):
        continue
    x = r["uid"].split("|")[2].split("_")
    box = tuple(int(v) for v in x)
    is_choco = 1 if r["true_class"] == "choco_hazelnut_high" else 0
    data.append((r["uid"], r["dataset"], int(r["frame_id"]), box, is_choco, acc_choco))
print(f"labeled choco-decision candidates: {len(data)}  "
      f"(choco={sum(d[4] for d in data)}, non-choco={sum(1-d[4] for d in data)})")

# ---- models + choco templates (block2,11) ----
defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
device = "cuda:0" if torch.cuda.is_available() else "cpu"
model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
choco = [o for o in objs if o["name"] == "choco_hazelnut_high"][0]
need = [2, 11]
choco["tcls"], _ = yi.build_template_cls(choco["template_dir"], model, device, choco["cls_cache"], False)
tpb = o_n.build_template_appe_blocks(
    choco["template_dir"], model, device,
    choco["appe_cache"].replace("_appe.pt", "_appe_b2-11.pt"), need, False)
seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
# flatten template patches per block: list[42] of [Np,384]
TPL = {b: [t.to(device) for t in tpb[b]] for b in need}
NV = len(tpb[11])
TCLS = choco["tcls"].to(device)


def frames_needed():
    by = defaultdict(set)
    for _, ds, f, *_ in data:
        by[ds].add(f)
    return by


def read_frames(ds, want):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    hi = max(want); out = {}
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as rd:
        conns = [c for c in rd.connections if c.topic == "/camera/camera/color/image_raw"]
        i = -1
        for conn, t, raw in rd.messages(connections=conns):
            i += 1
            if i > hi:
                break
            if i not in want:
                continue
            m = rd.deserialize(raw, conn.msgtype)
            b = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            out[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
    return out


def patch_sim(q, tmpl_list, agg):
    """q [Nq,384] on device; per-template amax over cols; return aggregated score."""
    if q.shape[0] == 0:
        return 0.0
    per = []
    for tp in tmpl_list:
        sim = q @ tp.T                     # [Nq, Ntp]
        per.append(sim.max(1).values)      # best template patch per query patch -> [Nq]
    P = torch.stack(per, 1)                # [Nq, 42]
    if agg == "mean_top1":
        return float(P.max(1).values.mean().clamp(0, 1))     # per-query best template, mean over q
    if agg == "toppct":
        v = P.max(1).values
        k = max(1, int(0.7 * v.numel()))
        return float(torch.sort(v).values[-k:].mean().clamp(0, 1))
    return 0.0


scores = defaultdict(list); ys = []
METHODS = ["P0_block11_clstop1", "P2_block11_allT", "P3_b2b11_allT",
           "P5_b2b11_semTop5", "P8_b2b11_toppct"]
by = frames_needed()
for ds, want in by.items():
    fr = read_frames(ds, want)
    for uid, dds, f, box, is_choco, acc in [d for d in data if d[1] == ds]:
        bgr = fr.get(f)
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm = yi.normalize_rgb(rgb)
        crop = yi.crop_resize_pad(norm, list(box))
        if crop is None:
            continue
        cls_all, pb = o_n.dinov2_blocks_forward(model, [crop], device, need)
        cls = cls_all[0]
        mask = yi.segment_box(seg, bgr, list(box), device)
        # masked query patches per block
        q = {}
        for b in need:
            qp = pb[b][0]
            q[b] = (yi.masked_query_patches(qp.cpu(), mask, list(box), pool)[0].to(device)
                    if mask is not None else qp.to(device))
        # semantic top template order
        sims = (TCLS @ cls.to(device)).cpu()
        order = torch.argsort(sims, descending=True).tolist()
        t1 = order[0]
        # P0 current: block11 clstop1 template masked-appe
        s_p0 = patch_sim(q[11], [TPL[11][t1]], "mean_top1")
        s_p2 = patch_sim(q[11], TPL[11], "mean_top1")
        s_p3 = 0.5 * (patch_sim(q[2], TPL[2], "mean_top1") + patch_sim(q[11], TPL[11], "mean_top1"))
        top5 = order[:5]
        s_p5 = 0.5 * (patch_sim(q[2], [TPL[2][i] for i in top5], "mean_top1") +
                      patch_sim(q[11], [TPL[11][i] for i in top5], "mean_top1"))
        s_p8 = 0.5 * (patch_sim(q[2], TPL[2], "toppct") + patch_sim(q[11], TPL[11], "toppct"))
        for name, s in zip(METHODS, [s_p0, s_p2, s_p3, s_p5, s_p8]):
            scores[name].append(s)
        ys.append(is_choco)

ys = np.array(ys)
print(f"\nscored {len(ys)} crops (choco={ys.sum()}, non-choco={len(ys)-ys.sum()})")


def auroc(s, y):
    # rank-based (Mann-Whitney U) AUROC, tie-aware
    s = np.asarray(s, float); y = np.asarray(y)
    P = int(y.sum()); N = len(y) - P
    if P == 0 or N == 0:
        return float("nan")
    order = np.argsort(s, kind="mergesort")
    ranks = np.empty(len(s), float); ranks[order] = np.arange(1, len(s) + 1)
    # average ties
    _, inv, cnt = np.unique(s, return_inverse=True, return_counts=True)
    sums = np.zeros(len(cnt)); np.add.at(sums, inv, ranks)
    ranks = (sums / cnt)[inv]
    return float((ranks[y == 1].sum() - P * (P + 1) / 2) / (P * N))


# persist raw per-candidate scores for downstream steps
with open(os.path.join(OUT, "choco_brownbox_dataset.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["idx", "is_choco"] + METHODS)
    for i in range(len(ys)):
        w.writerow([i, int(ys[i])] + [round(scores[m][i], 5) for m in METHODS])


comp = []
for m in METHODS:
    s = np.array(scores[m])
    a = auroc(s, ys)
    # negative verifier: pick threshold keeping >=9/10 choco, count carton removed
    ch = s[ys == 1]; nc = s[ys == 0]
    thr = np.sort(ch)[max(0, len(ch) - 10)] if len(ch) else 0  # keep ~all choco (min choco score)
    thr = float(np.min(ch)) if len(ch) else 0.0
    removed = int((nc < thr).sum()); kept_choco = int((ch >= thr).sum())
    comp.append(dict(method=m, AUROC=round(a, 3),
                     choco_mean=round(float(ch.mean()), 3), nonchoco_mean=round(float(nc.mean()), 3),
                     thr_keepall_choco=round(thr, 3), FP_removed=removed, choco_kept=kept_choco,
                     N_nonchoco=len(nc)))
with open(os.path.join(OUT, "texture_method_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(comp[0].keys())); w.writeheader(); w.writerows(comp)

print("\n=== texture method comparison (choco vs brown-box, provisional labels, N~29) ===")
print(f"{'method':22s} {'AUROC':>6} {'choco_m':>8} {'nonch_m':>8} {'thr':>6} {'FP_rm':>6} {'ch_keep':>7}")
for c in comp:
    print(f"{c['method']:22s} {c['AUROC']:>6} {c['choco_mean']:>8} {c['nonchoco_mean']:>8} "
          f"{c['thr_keepall_choco']:>6} {c['FP_removed']:>4}/{c['N_nonchoco']} {c['choco_kept']:>5}/{int(ys.sum())}")
print(f"\n-> {OUT}")
