#!/usr/bin/env python3
"""190624 에서 '박스가 애초에 안 생기는가, 점수에서 잘리는가'를 신규 로직 기준으로 다시 가른다.

phase37 은 옛 로직(박스당 1등 라벨)에서 이 분해를 했다. 신규 로직에서는 '1등 라벨에 밀려
박스가 사라진다'는 항목이 사라지고 대신 '주인 경쟁에서 졌다'가 생기므로, 배타 분류를 다시 짠다.

미가림 & 1.5 m 이내 셀 하나마다 (투영점이 그 물체의 위치다):

  A_no_box_at_all    프레임에 YOLO 박스가 하나도 없다
  B_no_box_on_object 박스는 있으나 투영점을 덮는 것이 없다      <- 제안/위치 실패
  C_lost_election    덮는 박스가 있는데 다른 객체가 그 박스의 주인이 됐다
  D_cut_semantic     주인은 됐으나 semantic < 0.35
  E_cut_appe         masked-appe < 0.605
  F_cut_hsv          HSV < 0.1214
  OK                 최종 수락 (그리고 투영점이 박스 안)

A/B 는 "박스가 애초에 없다", C~F 는 "박스는 있었는데 뒤에서 잘렸다".
"""
import argparse, csv, json, os, sys, time
from collections import defaultdict

import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
P37 = os.path.join(RSRCH, "phase37_lowlight_190624")
sys.path.insert(0, HERE); sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism_object_n as o_n
import yolo_ism as yi
import ism_hsv
from eval_190624 import load_clean_cells, BAG, COLOR

ORDER = ["A_no_box_at_all", "B_no_box_on_object", "C_lost_election",
         "D_cut_semantic", "E_cut_appe", "F_cut_hsv", "OK"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--margin", type=int, default=16)
    a = ap.parse_args()

    cells = load_clean_cells(a.stride)
    frames = sorted(cells)
    print(f"[gt] 셀 {sum(len(v) for v in cells.values())} · 프레임 {len(frames)}")

    import copy
    defaults, cfg = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device", "cuda:0")
    dev = dev if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(copy.deepcopy(cfg), model, dev, False)
    n2o = {o["name"]: o for o in objs}
    prompts, _ = o_n.build_prompt_groups(objs)
    tsim = o_n.template_similarity(objs)
    tau = o_n._color_tiebreak_tau(objs[0])
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    ms = min(float(o.get("score_threshold", 0.02)) for o in objs)
    sz = int(defaults.get("imgsz", 640))
    print(f"[cfg] imgsz {sz} · multi_label {o_n._multi_label_on(objs[0])} · "
          f"relative {objs[0].get('relative_assignment_enabled')} · tau {tau}")

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)
    want, img, i = set(frames), {}, -1
    with AnyReader([Path(BAG)], default_typestore=ts) as rd:
        ccon = [c for c in rd.connections if c.topic == COLOR]
        for con, _t, raw in rd.messages(connections=ccon):
            i += 1
            if i in want:
                m = rd.deserialize(raw, con.msgtype)
                b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                img[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()

    tot = defaultdict(int)
    per_obj = defaultdict(lambda: defaultdict(int))
    stolen_by = defaultdict(int)
    t0 = time.time()
    for n, fi in enumerate(frames, 1):
        bgr = img.get(fi)
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)
        with o_n.multi_label_nms(o_n._multi_label_on(objs[0])):
            pr = yolo.predict(bgr, conf=ms, imgsz=sz, verbose=False, device=dev)
        raw_boxes = []
        if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
            b = pr[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                raw_boxes.append(([x1, y1, x2, y2], float(b.conf[j])))
        # assign_frame_relative 과 동일한 유니크 박스 집합
        uniq = []
        for box, conf in raw_boxes:
            if conf < ms:
                continue
            for u in uniq:
                if o_n._iou_xyxy(u["box"], box) > 0.95:
                    u["conf"] = max(u["conf"], conf); break
            else:
                uniq.append({"box": list(box), "conf": conf})
        crops, keep = [], []
        for u in uniq:
            c = yi.crop_resize_pad(norm_full, u["box"])
            if c is not None:
                crops.append(c); keep.append(u)
        sems = masks = patch_all = cls_all = None
        if crops:
            need = sorted({x for o in objs for x in o["_need_blocks"]})
            cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, dev, need)
            masks = yi.segment_boxes(seg, bgr, [u["box"] for u in keep], dev)
            sems = [{o["name"]: float(yi.semantic_score(cls_all[k], o["tcls"], o["match_topk"]))
                     for o in objs} for k in range(len(keep))]

        for name, u_, v_, z in [(c[0], c[1], c[2], c[3]) for c in cells[fi]]:
            if not raw_boxes:
                tot["A_no_box_at_all"] += 1; per_obj[name]["A_no_box_at_all"] += 1; continue
            hits = [k for k, u in enumerate(keep)
                    if u["box"][0]-a.margin <= u_ <= u["box"][2]+a.margin
                    and u["box"][1]-a.margin <= v_ <= u["box"][3]+a.margin]
            if not hits:
                tot["B_no_box_on_object"] += 1; per_obj[name]["B_no_box_on_object"] += 1; continue
            best = None
            for k in hits:
                mask = masks[k]
                hsv = ({o["name"]: float(ism_hsv.shadow_score(bgr, keep[k]["box"], mask,
                                                              o.get("_hsv_proto"))) for o in objs}
                       if mask is not None else {})
                top = max(sems[k], key=sems[k].get)
                if tau > 0 and hsv:
                    tie = [x for x in sems[k] if tsim[top].get(x, 0.0) >= tau]
                    if len(tie) > 1:
                        top = max(tie, key=lambda x: hsv[x])
                st = None
                if top != name:
                    st = ("C_lost_election", top)
                else:
                    o = n2o[name]
                    if sems[k][name] < o["similarity_threshold"]:
                        st = ("D_cut_semantic", None)
                    elif mask is None:
                        st = ("E_cut_appe", None)
                    else:
                        qb = {x: patch_all[x][k].cpu() for x in o["_need_blocks"]}
                        bt = int(torch.argmax(o["tcls"] @ cls_all[k]))
                        ap_ = float(o_n.masked_appe_blocks(qb, mask, keep[k]["box"], pool, o, bt))
                        if ap_ < o_n._appe_gate_of(o):
                            st = ("E_cut_appe", None)
                        elif bool(o.get("hsv_gate_enabled", False)) and hsv and \
                                hsv[name] < float(o.get("hsv_gate_threshold", 0.0)):
                            st = ("F_cut_hsv", None)
                        else:
                            st = ("OK", None)
                if best is None or ORDER.index(st[0]) > ORDER.index(best[0]):
                    best = st                      # 가장 멀리 간 박스를 그 셀의 대표로
            tot[best[0]] += 1; per_obj[name][best[0]] += 1
            if best[0] == "C_lost_election" and best[1]:
                stolen_by[f"{name} <- {best[1]}"] += 1
        if n % 100 == 0:
            print(f"  {n}/{len(frames)}  ({time.time()-t0:.0f}s)", flush=True)

    N = sum(tot.values())
    print(f"\n=== 신규 로직에서 남은 실패가 어디서 나는가 (셀 {N})")
    print(f"{'단계':22s}{'n':>6s}{'%':>8s}   설명")
    desc = {"A_no_box_at_all": "프레임에 YOLO 박스가 0개",
            "B_no_box_on_object": "박스는 있으나 물체를 덮는 것이 없음",
            "C_lost_election": "덮는 박스를 다른 객체가 가져감",
            "D_cut_semantic": "semantic 점수에서 컷",
            "E_cut_appe": "부분 무늬(appe) 점수에서 컷",
            "F_cut_hsv": "색(HSV) 점수에서 컷",
            "OK": "최종 수락"}
    for s in ORDER:
        print(f"{s:22s}{tot[s]:6d}{100*tot[s]/max(N,1):7.1f}%   {desc[s]}")
    box_missing = tot["A_no_box_at_all"] + tot["B_no_box_on_object"]
    cut = sum(tot[s] for s in ("C_lost_election", "D_cut_semantic", "E_cut_appe", "F_cut_hsv"))
    print(f"\n  박스가 애초에 없다   {box_missing:5d}  ({100*box_missing/max(N,1):.1f}%)")
    print(f"  박스는 있는데 잘렸다 {cut:5d}  ({100*cut/max(N,1):.1f}%)")
    print(f"  통과               {tot['OK']:5d}  ({100*tot['OK']/max(N,1):.1f}%)")
    print(f"\n=== 객체별")
    print(f"{'object':22s}" + "".join(f"{s.split('_')[0]:>7s}" for s in ORDER))
    for name in sorted(per_obj):
        print(f"{name:22s}" + "".join(f"{per_obj[name][s]:7d}" for s in ORDER))
    if stolen_by:
        print(f"\n=== 박스를 빼앗긴 조합 상위")
        for k, v in sorted(stolen_by.items(), key=lambda x: -x[1])[:12]:
            print(f"  {k:44s}{v:5d}")
    json.dump({"total": dict(tot), "per_object": {k: dict(v) for k, v in per_obj.items()},
               "stolen_by": dict(stolen_by)},
              open(os.path.join(REPO, "outputs", "phase39_multilabel_relative",
                                "metrics", "stage_190624.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
