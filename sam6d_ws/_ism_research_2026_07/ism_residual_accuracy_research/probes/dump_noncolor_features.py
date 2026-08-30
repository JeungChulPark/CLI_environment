#!/usr/bin/env python3
"""dump_noncolor_features.py — 비색상 특징(형태·로고·국소 patch·semantic)을 한 번에 계산 (READ-ONLY).

운영 모듈은 import 만 하고 수정하지 않는다. 후보 BBox 는 운영 덤프 그대로 고정하고,
MobileSAM mask 와 DINOv2 patch 를 다시 계산해 아래 특징들의 **점수만** 저장한다
(patch 텐서를 그대로 저장하면 수백 MB 라 스칼라만 남긴다).

형태 (mask 기반, 객체 무관 1개 + 객체별 비교)
  aspect, extent(mask/bbox), solidity, compactness, contour_complexity, hu1..hu7,
  edge_orient_entropy, silhouette_iou_best(42 렌더 mask 와 정규화 IoU 최대)

국소 patch (query masked patch vs 템플릿 patch)  — 지시된 A~I
  A appe_top1cls      현재 방식: CLS-top1 템플릿 1장
  B appe_semtop3      semantic 상위 3장 평균
  C appe_semtop5      semantic 상위 5장 평균
  D appe_max42        42장 중 최대 (기존 지표)
  E appe_mnn          mutual nearest neighbor 비율
  F appe_bidir        양방향 일관성 (q→t, t→q 평균)
  G appe_top20pct     patch 점수 상위 20% 평균
  H appe_pyramid      2x2 사분면별 매칭 평균
  I appe_b2b11        block2 + block11 결합

로고/문자
  edge_density        mask 내부 Canny 에지 비율
  patch_top10pct      patch 점수 상위 10% 평균 (국소 고득점 = 로고 가능성)
  patch_score_std     patch 점수 분산 (균일 표면 vs 인쇄 패턴)
  disc_patch_score    42 view 에서 반복적으로 높은 patch 만 사용한 점수

semantic (전 객체 공통 규칙만)
  sem_top1/top5/mean/bottom5/std/max_min_gap/top1_minus_mean

산출: features/<ds>_feat.csv
"""
import argparse, csv, os, sys, time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism as yi                    # noqa: E402
import yolo_ism_object_n as o_n          # noqa: E402

OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
OUT = os.path.join(ROOT, "features")
os.makedirs(OUT, exist_ok=True)
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
COLOR_TOPIC = "/camera/camera/color/image_raw"
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
BLOCKS = [2, 11]


def bag_frames(ds, want):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    hi = max(want)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == COLOR_TOPIC]
        i = -1
        for conn, t, raw in reader.messages(connections=conns):
            i += 1
            if i > hi:
                break
            if i not in want:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            yield i, (cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
                      if msg.encoding.lower() == "rgb8" else buf.copy())


# ---------------------------------------------------------------- 형태 특징
def shape_feats(msub):
    """mask(sub-image 크기) 에서 시점 무관 형태 기술자."""
    f = {}
    h, w = msub.shape
    m = msub.astype(np.uint8)
    area = int(m.sum())
    f["mask_area"] = area
    f["aspect"] = round(w / max(h, 1), 4)
    f["extent"] = round(area / max(h * w, 1), 4)
    if area < 30:
        return f, None
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return f, None
    c = max(cnts, key=cv2.contourArea)
    a = cv2.contourArea(c); p = cv2.arcLength(c, True)
    hull = cv2.convexHull(c); ha = cv2.contourArea(hull)
    f["solidity"] = round(a / max(ha, 1e-6), 4)
    f["compactness"] = round(4 * np.pi * a / max(p * p, 1e-6), 4)          # 원=1
    ap = cv2.approxPolyDP(c, 0.01 * p, True)
    f["contour_complexity"] = round(len(ap) / max(len(c), 1) * 100, 4)
    hu = cv2.HuMoments(cv2.moments(m)).flatten()
    for i, v in enumerate(hu):
        f[f"hu{i+1}"] = round(float(-np.sign(v) * np.log10(abs(v) + 1e-30)), 4)
    return f, m


def norm_silhouette(m, size=64):
    """정규화 실루엣: 최대 윤곽의 bounding box 로 잘라 size 정사각형으로."""
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    if w < 3 or h < 3:
        return None
    return (cv2.resize(m[y:y+h, x:x+w], (size, size),
                       interpolation=cv2.INTER_NEAREST) > 0).astype(np.uint8)


def edge_feats(sub, msub):
    f = {}
    g = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)
    e = cv2.Canny(g, 60, 160)
    inside = msub > 0
    n = max(int(inside.sum()), 1)
    f["edge_density"] = round(float((e[inside] > 0).sum()) / n, 4)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, 3); gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, 3)
    mag = np.sqrt(gx*gx + gy*gy); ang = (np.arctan2(gy, gx) + np.pi) % np.pi
    sel = inside & (mag > 30)
    if sel.sum() > 20:
        hist, _ = np.histogram(ang[sel], bins=9, range=(0, np.pi))
        p = hist / hist.sum()
        f["edge_orient_entropy"] = round(float(-(p[p > 0] * np.log(p[p > 0])).sum()), 4)
    else:
        f["edge_orient_entropy"] = ""
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", nargs="+", default=DATASETS)
    a = ap.parse_args()

    want = defaultdict(set)
    for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
        if r["user_reviewed"] == "yes":
            want[r["dataset_name"]].add(int(r["frame_id"]))

    boxes = defaultdict(list)
    for ds in DATASETS:
        for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
            fr = int(r["frame_id"])
            if fr in want[ds]:
                boxes[(ds, fr)].append(
                    (r["uid"], [int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])],
                     set(x for x in r["candidate_for"].split(";") if x)))
    print(f"대상 프레임 {sum(len(v) for v in want.values())} / 박스 {sum(len(v) for v in boxes.values())}")

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    # 템플릿 준비: patch flat / view segment / 렌더 mask 실루엣
    TDIR = {"Bear": "Bear", "Dinosaur": "Dinosaur", "Febreze_high": "Febreze_high",
            "Mugcup_high": "Mugcup_color_high", "Rabbit": "Rabbit", "Sauce_high": "Sauce_high",
            "Sikhye_high": "Sikhye_high",
            "choco_hazelnut_high": "choco_hazelnut_color_high",
            "saffron": "saffron", "milk": "milk"}
    for o in objs:
        o["_tcls"] = o["tcls"].to(device)
        o["_flat"] = {b: torch.cat(o["tappe_blocks"][b], 0).to(device) for b in BLOCKS}
        o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                               for i, t in enumerate(o["tappe_blocks"][BLOCKS[0]])]).to(device)
        o["_nview"] = len(o["tappe_blocks"][BLOCKS[0]])
        sil = []
        td = os.path.join(REPO, "template", TDIR[o["name"]], "templates")
        for i in range(42):
            mp = os.path.join(td, f"mask_{i}.png")
            im = cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
            if im is None:
                continue
            s = norm_silhouette((im > 0).astype(np.uint8))
            if s is not None:
                sil.append(s)
        o["_sil"] = np.stack(sil) if sil else None
        # 42 view 에서 반복적으로 높은 patch (discriminative) — 템플릿 patch 의 view 간 일관성
        P = o["_flat"][11]
        o["_tnorm"] = P

    t0 = time.time()
    for ds in a.datasets:
        rows = []
        fr_list = sorted(want[ds])
        for fi, bgr in bag_frames(ds, set(fr_list)):
            bx = boxes.get((ds, fi), [])
            if not bx:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            norm = yi.normalize_rgb(rgb)
            crops, keep = [], []
            for uid, b, cf in bx:
                c = yi.crop_resize_pad(norm, b)
                if c is not None:
                    crops.append(c); keep.append((uid, b, cf))
            if not crops:
                continue
            cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, BLOCKS)
            masks = yi.segment_boxes(seg, bgr, [list(b) for _, b, _ in keep], device)

            for bi, (uid, b, cf) in enumerate(keep):
                x1, y1, x2, y2 = b
                mask = masks[bi]
                sub = bgr[y1:y2, x1:x2]
                msub = (mask[y1:y2, x1:x2] if mask is not None
                        else np.ones(sub.shape[:2], bool))
                sf, m = shape_feats(msub)
                sf.update(edge_feats(sub, msub))
                qsil = norm_silhouette(m) if m is not None else None

                qfg = {}
                for bl in BLOCKS:
                    qp = patch_all[bl][bi]
                    qfg[bl] = (yi.masked_query_patches(qp.cpu(), mask, list(b), pool)[0].to(device)
                               if mask is not None else qp)
                nq = int(qfg[11].shape[0])

                for o in objs:
                    nm = o["name"]
                    sims = (o["_tcls"] @ cls_all[bi].to(device)).cpu().numpy()
                    order = np.argsort(-sims)
                    ss = sims[order]
                    row = {"uid": uid, "dataset": ds, "frame_id": fi, "object": nm,
                           "is_candidate": int(nm in cf), "n_valid_patch": nq, **sf}
                    row.update({
                        "sem_top1": round(float(ss[0]), 5),
                        "sem_top5": round(float(ss[:5].mean()), 5),
                        "sem_mean": round(float(sims.mean()), 5),
                        "sem_bottom5": round(float(ss[-5:].mean()), 5),
                        "sem_std": round(float(sims.std()), 5),
                        "sem_max_min_gap": round(float(ss[0] - ss[-1]), 5),
                        "sem_top1_minus_mean": round(float(ss[0] - sims.mean()), 5)})

                    if nq == 0:
                        rows.append(row); continue
                    Q = qfg[11]                                    # [nq, 384]
                    Sfull = Q @ o["_flat"][11].T                   # [nq, ntpatch]
                    per = torch.full((nq, o["_nview"]), -1.0, device=device, dtype=Sfull.dtype)
                    per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(nq, -1),
                                        Sfull, reduce="amax")
                    pv = per.clamp(0, 1)                           # [nq, 42] patch별 view별 최고
                    vmean = pv.mean(0)                             # view별 점수
                    t1 = int(order[0])
                    row["appe_A_top1cls"] = round(float(vmean[t1]), 5)
                    row["appe_B_semtop3"] = round(float(vmean[order[:3]].mean()), 5)
                    row["appe_C_semtop5"] = round(float(vmean[order[:5]].mean()), 5)
                    row["appe_D_max42"] = round(float(vmean.max()), 5)
                    # G / 로고: patch 단위 상위 백분위
                    best_patch = pv[:, t1]
                    k10 = max(1, nq // 10); k20 = max(1, nq // 5)
                    sp = torch.sort(best_patch, descending=True).values
                    row["appe_G_top20pct"] = round(float(sp[:k20].mean()), 5)
                    row["patch_top10pct"] = round(float(sp[:k10].mean()), 5)
                    row["patch_score_std"] = round(float(best_patch.std()), 5)
                    # E mutual NN (top1 view 의 patch 집합과)
                    st = o["_seg"] == t1
                    St = Sfull[:, st]                              # [nq, ntp_view]
                    if St.shape[1] > 0:
                        q2t = St.argmax(1); t2q = St.argmax(0)
                        mnn = (t2q[q2t] == torch.arange(nq, device=device)).float().mean()
                        row["appe_E_mnn"] = round(float(mnn), 5)
                        row["appe_F_bidir"] = round(float(
                            0.5 * St.max(1).values.mean() + 0.5 * St.max(0).values.mean()), 5)
                    # H spatial pyramid (query patch 를 2x2 로 나눠 각 사분면 평균)
                    side = int(np.sqrt(nq)) if int(np.sqrt(nq)) ** 2 == nq else 0
                    if side >= 4:
                        g = best_patch.reshape(side, side)
                        h2, w2 = side // 2, side // 2
                        quads = [g[:h2, :w2], g[:h2, w2:], g[h2:, :w2], g[h2:, w2:]]
                        row["appe_H_pyramid"] = round(float(
                            torch.stack([q.mean() for q in quads]).mean()), 5)
                    # I block2 + block11
                    Q2 = qfg[2]
                    S2 = Q2 @ o["_flat"][2].T
                    per2 = torch.full((Q2.shape[0], o["_nview"]), -1.0,
                                      device=device, dtype=S2.dtype)
                    per2.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(Q2.shape[0], -1),
                                         S2, reduce="amax")
                    v2 = per2.clamp(0, 1).mean(0)
                    row["appe_b2_top1cls"] = round(float(v2[t1]), 5)
                    row["appe_I_b2b11"] = round(float(0.5 * v2[t1] + 0.5 * vmean[t1]), 5)
                    # discriminative patch: view 간 점수 분산이 큰 patch 만 사용
                    vstd = pv.std(1)
                    kd = max(1, nq // 4)
                    idx = torch.topk(vstd, kd).indices
                    row["disc_patch_score"] = round(float(pv[idx, t1].mean()), 5)
                    # 형태: 정규화 실루엣 IoU 최대
                    if qsil is not None and o["_sil"] is not None:
                        inter = np.logical_and(o["_sil"], qsil[None]).sum((1, 2))
                        uni = np.logical_or(o["_sil"], qsil[None]).sum((1, 2))
                        row["sil_iou_best"] = round(float((inter / np.maximum(uni, 1)).max()), 5)
                        row["sil_iou_mean"] = round(float((inter / np.maximum(uni, 1)).mean()), 5)
                    rows.append(row)

        ks = sorted({k for r in rows for k in r})
        head = ["uid", "dataset", "frame_id", "object", "is_candidate"]
        ks = head + [k for k in ks if k not in head]
        with open(os.path.join(OUT, f"{ds}_feat.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=ks, restval=""); w.writeheader(); w.writerows(rows)
        print(f"  {ds}: {len(rows):,} pair  ({time.time()-t0:.0f}s)")
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()
