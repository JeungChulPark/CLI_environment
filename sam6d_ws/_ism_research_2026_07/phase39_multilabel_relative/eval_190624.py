#!/usr/bin/env python3
"""phase39 — 저조도 세션 190624 에서 개선 전/후를 '위치까지 검증해서' 비교한다.

959셀 GT 는 프레임 단위 '보였나'만 보지만, 190624 는 융합 객체맵을 역투영한 기하 GT 가 있어
**박스가 그 물체 위인지**까지 볼 수 있다(phase37 에서 투영 적중 96.2% 로 검증됨).

세 구성을 같은 프레임에 돌린다.
  OLD  imgsz 640 · multi_label off · 상대판정 off   = 190624 결과를 만든 그 설정
  MID  imgsz 960 · 나머지 OLD 와 동일               = 해상도 버그만 고친 경우
  NEW  imgsz 960 · multi_label on · 상대판정 on · 색결선 0.60

셀 판정 (미가림 & 1.5 m 이내 셀만)
  HIT   그 객체가 수락됐고 투영점이 박스 안       <- 위치까지 맞은 검출
  MISPLACED 수락은 됐는데 투영점이 박스 밖
  MISS  수락 안 됨
프레임에 존재하지 않는(FOV 밖) 객체를 수락한 것은 FP_out 으로 따로 센다.
"""
import argparse, bisect, csv, json, os, sys, time
from collections import defaultdict

import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
P37 = os.path.join(RSRCH, "phase37_lowlight_190624")
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism_object_n as o_n
import yolo_ism as yi

BAG = ("/home/ldh9501/temp_ws/CLI_environment/integration/data/0807_chungbuk/"
       "190624__irregular_multi_loop/sam")
COLOR = "/camera/camera/color/image_raw"
CONFIGS = {
    "OLD": dict(imgsz=640, flags={"yolo_multi_label": False, "relative_assignment_enabled": False,
                                  "color_tiebreak_template_sim": 0.0}),
    "MID": dict(imgsz=960, flags={"yolo_multi_label": False, "relative_assignment_enabled": False,
                                  "color_tiebreak_template_sim": 0.0}),
    "NEW": dict(imgsz=960, flags={"yolo_multi_label": True, "relative_assignment_enabled": True,
                                  "color_tiebreak_template_sim": 0.60}),
    "NEW640": dict(imgsz=640, flags={"yolo_multi_label": True, "relative_assignment_enabled": True,
                                     "color_tiebreak_template_sim": 0.60}),
}


def load_clean_cells(stride):
    """미가림 & z<=1.5m 인 (frame,object) 셀 + 투영 좌표."""
    dm = {(int(r["frame_idx"]), r["object"]): float(r["depth_meas_m"])
          for r in csv.DictReader(open(os.path.join(P37, "depth_omega.csv")))}
    cells = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(P37, "presence_geom.csv"))):
        fi = int(r["frame_idx"])
        if fi % stride or int(r["in_fov"]) != 1:
            continue
        z = float(r["z_m"])
        if not (0.25 <= z <= 1.5):
            continue
        d = dm.get((fi, r["object"]), 0.0)
        if d > 0.05 and d < z - 0.20:            # 가려짐
            continue
        cells[fi].append((r["object"], float(r["u"]), float(r["v"]), z))
    return cells


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--out", default=os.path.join(REPO, "outputs", "phase39_multilabel_relative"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    cells = load_clean_cells(a.stride)
    frames = sorted(cells)
    print(f"[gt] 판정 대상 셀 {sum(len(v) for v in cells.values())} · 프레임 {len(frames)}")

    import copy
    defaults, cfg = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0")
    device = device if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    sets, GRP, TS = {}, {}, {}
    for tag, c in CONFIGS.items():
        oc = copy.deepcopy(cfg)
        for o in oc:
            o.update(c["flags"])
        sets[tag] = o_n.prepare_objects(oc, model, device, False)
        GRP[tag] = o_n.build_prompt_groups(sets[tag])[1]
        TS[tag] = o_n.template_similarity(sets[tag])
    prompts = o_n.build_prompt_groups(sets["NEW"])[0]
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    min_score = min(float(o.get("score_threshold", 0.02)) for o in sets["NEW"])

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)
    want = set(frames)
    img, i = {}, -1
    with AnyReader([Path(BAG)], default_typestore=ts) as rd:
        ccon = [c for c in rd.connections if c.topic == COLOR]
        for con, _t, raw in rd.messages(connections=ccon):
            i += 1
            if i in want:
                m = rd.deserialize(raw, con.msgtype)
                b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                img[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
    print(f"[bag] {len(img)} 프레임 디코드")

    stat = {t: defaultdict(int) for t in CONFIGS}
    per_obj = {t: defaultdict(lambda: [0, 0, 0]) for t in CONFIGS}
    t0 = time.time()
    for n, fi in enumerate(frames, 1):
        bgr = img.get(fi)
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)
        h, w = bgr.shape[:2]
        present = {c[0]: c for c in cells[fi]}
        for tag, c in CONFIGS.items():
            objs = sets[tag]
            with o_n.multi_label_nms(o_n._multi_label_on(objs[0])):
                pr = yolo.predict(bgr, conf=min_score, imgsz=c["imgsz"],
                                  verbose=False, device=device)
            pb = {k: [] for k in range(len(prompts))}
            if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
                b = pr[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                    x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                    ci = int(b.cls[j]) if b.cls is not None else 0
                    if ci in pb:
                        pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
            for k in pb:
                pb[k].sort(key=lambda z: -z[1])
            res = o_n.recognize_frame_auto(GRP[tag], pb, bgr, rgb, norm_full,
                                           model, device, segmentor, pool, TS[tag])
            acc = {k: v["box"] for k, v in res.items() if v["accepted"] and v["box"]}
            for name, (_, u, v, z) in present.items():
                box = acc.get(name)
                if box is None:
                    stat[tag]["MISS"] += 1; per_obj[tag][name][2] += 1
                elif box[0]-16 <= u <= box[2]+16 and box[1]-16 <= v <= box[3]+16:
                    stat[tag]["HIT"] += 1; per_obj[tag][name][0] += 1
                else:
                    stat[tag]["MISPLACED"] += 1; per_obj[tag][name][1] += 1
            for name in acc:
                if name not in present:
                    stat[tag]["FP_out"] += 1
        if n % 100 == 0:
            print(f"  {n}/{len(frames)}  ({time.time()-t0:.0f}s)", flush=True)

    print(f"\n판정 대상 셀 {sum(len(v) for v in cells.values())}  (미가림 · 1.5 m 이내)")
    print(f"{'config':6s}{'HIT':>7s}{'MISPLACED':>11s}{'MISS':>7s}{'위치검증 recall':>16s}{'FP_out':>9s}")
    for t in ("OLD", "MID", "NEW", "NEW640"):
        s = stat[t]; tot = s["HIT"] + s["MISPLACED"] + s["MISS"]
        print(f"{t:6s}{s['HIT']:7d}{s['MISPLACED']:11d}{s['MISS']:7d}"
              f"{s['HIT']/max(tot,1):16.3f}{s['FP_out']:9d}")
    print(f"\n객체별 HIT (OLD -> MID -> NEW)")
    for name in sorted(per_obj["NEW"]):
        print(f"  {name:22s}" + " -> ".join(f"{per_obj[t][name][0]:4d}" for t in ("OLD", "MID", "NEW", "NEW640")))
    json.dump({t: dict(stat[t]) for t in CONFIGS},
              open(os.path.join(a.out, "metrics", "eval_190624.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
