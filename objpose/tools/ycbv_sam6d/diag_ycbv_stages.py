#!/usr/bin/env python3
"""diag_ycbv_stages.py — YCB-V 에서 객체가 어느 단계에서 떨어지는지 GT 인스턴스 단위로 센다.

인식기는 run_ycbv_sam6d.py 가 쓰는 것과 같은 설정(configs/yolo_ism_ycbv.yaml, 장면 대상 객체)으로
YOLO-World → ISM(DINOv2 semantic + masked appearance) 을 돌린다(diag_bop_ism.py 와 같은 경로).
화면에 보이는 GT 인스턴스(visib_fract >= 0.1) 하나마다:

  S1  YOLO-World 가 GT 박스(bbox_visib)와 IoU >= 0.5 인 박스를 냈는가 — 라벨 무관
  S2  그 객체 **자신의 프롬프트** 라벨로 IoU >= 0.5 인 박스를 냈는가
  S3  ISM 이 그 객체로 받아들인 박스가 GT 박스와 IoU >= 0.5 인가
  S4  모든 프레임 실행(ycbv_sam6d_all)의 최종 포즈가 그 프레임에 있고 ADD-S < 0.1d 인가 (errors_vs_gt.csv)

S1 은 되는데 S2 가 안 되면 YOLO 가 물체는 봤지만 다른 프롬프트 이름을 붙인 것이고(프롬프트 혼동),
S1 부터 안 되면 YOLO-World 가 그 물체를 아예 박스로 잡지 못한 것이다.

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/ycbv_sam6d/diag_ycbv_stages.py \
        --stride 10 --errors objpose/docs/figs_ycbv_sam6d_gt/errors_vs_gt.csv --out objpose/docs/figs_ycbv_sam6d_gt
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]
SAM6D = REPO / "sam6d_realtime"
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video")
TABLE = REPO / "objpose" / "rt" / "bop" / "ycbv_objects.json"
CONFIG = "configs/yolo_ism_ycbv.yaml"


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--errors", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()
    errors_csv = a.errors.resolve()
    out = a.out.resolve()

    final_ok = set()
    with open(errors_csv) as fh:
        for r in csv.DictReader(fh):
            if r["mode"] == "all" and r["matched"] == "True" and r["ok"] == "True":
                final_ok.add((r["scene"], int(r["k"]), r["obj"]))

    sys.path.insert(0, str(SAM6D))
    os.chdir(SAM6D)
    import torch
    import yolo_ism as yi
    import yolo_ism_object_n as o_n
    import ultralytics.nn.text_model as _tm
    _tm.WEIGHTS_DIR = SAM6D / "assets"
    from ultralytics import YOLOWorld

    table = json.loads(TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    defaults, all_objs = o_n.load_config(str(SAM6D / CONFIG))
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, a.device)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), a.device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)

    stage = collections.defaultdict(lambda: collections.Counter())
    confuse = collections.defaultdict(collections.Counter)      # 물체 → S1 은 되고 S2 는 안 될 때 붙은 프롬프트
    ism_dec = collections.defaultdict(collections.Counter)      # 물체 → S2 는 되고 S3 는 안 될 때 ISM 판정
    yolo_conf = collections.defaultdict(list)                   # 물체 → 자기 프롬프트 박스의 최고 conf (IoU>=0.5)
    per_scene = collections.defaultdict(lambda: collections.defaultdict(collections.Counter))
    samples = []                                                # 그림용: (scene, k, name, gt box, 해당 프레임 yolo 박스들)

    for scene in sorted((YCBV / "test").iterdir()):
        s = scene.name
        gt = json.loads((scene / "scene_gt.json").read_text())
        info = json.loads((scene / "scene_gt_info.json").read_text())
        names = sorted({by_id[r["obj_id"]] for v in gt.values() for r in v})
        objs = [o for o in all_objs if o["name"] in names]
        objs = o_n.prepare_objects(objs, model, a.device, rebuild=False)
        prompts, groups = o_n.build_prompt_groups(objs)
        prompt_of = {o["name"]: prompts.index(o["yolo_prompt"]) for o in objs}
        tsim = o_n.template_similarity(objs)
        # set_classes 는 모델이 GPU 로 올라간 뒤 다시 부르면 장치 충돌이 난다 → 장면마다 새로 적재
        yolo = YOLOWorld(o_n._abspath(defaults.get("weights", "yolov8m-worldv2.pt")))
        yolo.set_classes(prompts)
        min_score = min(float(o.get("score_threshold", 0.02)) for o in objs)
        multi = o_n._multi_label_on(objs[0])
        ids = sorted(gt, key=int)
        for k in range(0, len(ids), a.stride):
            fid = ids[k]
            bgr = cv2.imread(str(scene / "rgb" / f"{int(fid):06d}.png"))
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            with o_n.multi_label_nms(multi):
                r = yolo.predict(bgr, conf=min_score, imgsz=int(defaults.get("imgsz", 640)), verbose=False,
                                 device=a.device)
            h, w = bgr.shape[:2]
            pb = {i: [] for i in range(len(prompts))}
            allb = []
            if len(r) and r[0].boxes is not None:
                b = r[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                    x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                    ci = int(b.cls[j]); cf = float(b.conf[j])
                    pb[ci].append(([x1, y1, x2, y2], cf))
                    allb.append(([x1, y1, x2, y2], ci, cf))
            for kk in pb:
                pb[kk].sort(key=lambda t: t[1], reverse=True)
            res = o_n.recognize_frame_auto(groups, pb, bgr, rgb, yi.normalize_rgb(rgb), model, a.device,
                                           segmentor, pool, tsim)
            for j, g in enumerate(gt[fid]):
                gi = info[fid][j]
                if gi.get("visib_fract", 0) < 0.1 or g["obj_id"] not in by_id:
                    continue
                nm = by_id[g["obj_id"]]
                bx, by_, bw, bh = gi["bbox_visib"]
                gb = [bx, by_, bx + bw, by_ + bh]
                ov = [(bb, ci, cf) for bb, ci, cf in allb if iou(bb, gb) >= 0.5]
                s1 = bool(ov)
                own = [cf for bb, ci, cf in ov if ci == prompt_of[nm]]
                s2 = bool(own)
                rr = res.get(nm, {})
                s3 = bool(rr.get("accepted")) and rr.get("box") is not None and iou(rr["box"], gb) >= 0.5
                s4 = (s, k, nm) in final_ok
                c = stage[nm]
                c["gt"] += 1; c["s1"] += s1; c["s2"] += s2; c["s3"] += s3; c["s4"] += s4
                sc = per_scene[s][nm]
                sc["gt"] += 1; sc["s1"] += s1; sc["s2"] += s2; sc["s3"] += s3; sc["s4"] += s4
                if own:
                    yolo_conf[nm].append(max(own))
                if s1 and not s2:
                    best = max(ov, key=lambda t: t[2])
                    confuse[nm][prompts[best[1]]] += 1
                if s2 and not s3:
                    ism_dec[nm][rr.get("decision", "?")] += 1
                if not s2 and len(samples) < 400:
                    samples.append({"scene": s, "k": k, "obj": nm, "gt_box": gb, "s1": s1,
                                    "boxes": [(bb, prompts[ci], round(cf, 3)) for bb, ci, cf in allb[:15]]})
        print(s, "done", flush=True)

    rep = {"stride": a.stride, "objects": {}, "per_scene": {s: {n: dict(c) for n, c in v.items()} for s, v in per_scene.items()},
           "samples": samples}
    prompt_txt = {o["name"]: o["yolo_prompt"] for o in all_objs}
    L = ["| 객체 | YOLO 프롬프트 | 보이는 GT 인스턴스 | ① YOLO 박스 (라벨 무관) | ② YOLO 박스 (자기 프롬프트) | ③ ISM 통과 | ④ 최종 포즈 맞음 | ①은 되고 ②가 안 될 때 붙은 프롬프트 | ②는 되고 ③이 안 될 때 ISM 판정 |",
         "|" + "---|" * 9]
    for nm, c in sorted(stage.items(), key=lambda kv: kv[1]["s2"] / max(kv[1]["gt"], 1)):
        g = max(c["gt"], 1)
        cf = ", ".join(f"{p} {n}" for p, n in confuse[nm].most_common(2))
        dc = ", ".join(f"{d.replace('no-object', '')} {n}" for d, n in ism_dec[nm].most_common(2))
        L.append(f"| {nm.replace('ycbv_', '')} | {prompt_txt[nm]} | {c['gt']} | {c['s1'] / g * 100:.1f}% | "
                 f"{c['s2'] / g * 100:.1f}% | {c['s3'] / g * 100:.1f}% | {c['s4'] / g * 100:.1f}% | {cf} | {dc} |")
        rep["objects"][nm] = {**dict(c), "prompt": prompt_txt[nm], "confused_as": dict(confuse[nm]),
                              "ism_decisions": dict(ism_dec[nm]),
                              "own_conf_median": float(np.median(yolo_conf[nm])) if yolo_conf[nm] else None}
    tot = collections.Counter()
    for c in stage.values():
        tot.update(c)
    g = tot["gt"]
    L.append(f"| **전체** | | **{g}** | **{tot['s1'] / g * 100:.1f}%** | **{tot['s2'] / g * 100:.1f}%** | "
             f"**{tot['s3'] / g * 100:.1f}%** | **{tot['s4'] / g * 100:.1f}%** | | |")
    out.mkdir(parents=True, exist_ok=True)
    (out / "stages.md").write_text("\n".join(L) + "\n")
    (out / "stages.json").write_text(json.dumps(rep, indent=1, ensure_ascii=False))
    print("\n".join(L))


if __name__ == "__main__":
    main()
