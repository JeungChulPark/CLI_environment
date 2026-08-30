#!/usr/bin/env python3
"""phase41 — 0807 8세션 전체에서 '거리 때문에 못 잡는 것인가'를 확인한다.

## 앞선 측정의 오류와 수정

phase40 은 셀의 가시성을 "**가려졌다고 반증되지 않음**"으로 판정했다. 그런데 depth 는 먼 거리에서
자주 무효(0)라, 벽 뒤 6.7 m 밖의 물체도 '보인다'로 세어졌다. 190624 의 choco 는 **실제로 두 개**가
있고(사용자 확인), 한쪽 선반을 보는 프레임에서 다른 쪽 choco 가 벽을 뚫고 투영돼 전부 미검출로
집계됐다. phase40 이 이를 'GT 위치 오류'로 본 것은 틀렸다 — GT 위치는 맞고 **가시성 판정이 틀렸다**.

여기서는 기준을 뒤집는다: 투영 화소의 **측정 depth 가 존재하고 예상 z 와 ±tol 안에서 일치할 때만**
그 셀을 '보인다'로 센다(= 그 화소에 정말 그 거리의 표면이 있다). 같은 이름의 인스턴스가 여럿이면
**전부 남긴다**(0.4 m 안쪽만 중복으로 합침).

## 셀마다 기록

  z_m, px(기하 예측 크기), edge(테두리 80px), roi_luma, roi_std, omega
  stage : A_no_box / B_no_box_on_obj / C_lost / D_sem / E_appe / F_hsv / OK
"""
import argparse, bisect, csv, glob, json, os, sqlite3, sys, time
from collections import defaultdict

import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
ROOT = os.path.dirname(REPO)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism_object_n as o_n
import yolo_ism as yi
import ism_hsv

OUTR = f"{ROOT}/integration/output/0807_chungbuk"
DATR = f"{ROOT}/integration/data/0807_chungbuk"
COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
ORDER = ["A_no_box", "B_no_box_on_obj", "C_lost", "D_sem", "E_appe", "F_hsv", "OK"]


def q2R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def session_cells(sess, stride, size_m, tol):
    tr = np.loadtxt(f"{OUTR}/{sess}/orb3/trajectory.txt")
    tt = tr[:, 0]
    T = np.zeros((len(tr), 4, 4)); T[:, 3, 3] = 1
    for i, r in enumerate(tr):
        T[i, :3, :3] = q2R(r[4:8]); T[i, :3, 3] = r[1:4]
    om = np.zeros(len(tr))
    for i in range(1, len(tr)-1):
        dR = T[i-1, :3, :3].T @ T[i+1, :3, :3]
        om[i] = np.degrees(np.arccos(np.clip((np.trace(dR)-1)/2, -1, 1))) / max(tt[i+1]-tt[i-1], 1e-6)
    X = np.load(sorted(glob.glob(f"{DATR}/{sess}/calib/*.npy"))[0])
    info = json.load(open(f"{DATR}/{sess}/info.json"))
    K = np.array(info["sam"]["camera"]["K"]).reshape(3, 3)
    W, H = info["sam"]["camera"]["width"], info["sam"]["camera"]["height"]
    db = sorted(glob.glob(f"{DATR}/{sess}/sam/*.db3"))[0]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True); cur = con.cursor()
    cur.execute("select id,name from topics"); tops = cur.fetchall()
    tid = [t for t, n in tops if n == COLOR][0]
    cur.execute("select timestamp from messages where topic_id=? order by timestamp", (tid,))
    cts = np.array([r[0] for r in cur.fetchall()])/1e9
    con.close()
    # 같은 이름이라도 서로 다른 인스턴스는 전부 남긴다 (초코 상자는 실제로 두 개다)
    inst = []
    for o in json.load(open(f"{OUTR}/{sess}/fused/orb3/objects.json"))["objects"]:
        if o["observations"] < 5:
            continue
        p = np.array(o["T_map_obj"])[:3, 3]
        if any(k["object_name"] == o["object_name"] and np.linalg.norm(p-k["p"]) < 0.4 for k in inst):
            continue
        inst.append({"object_name": o["object_name"], "p": p, "id": o["object_id"]})
    cells, omf = defaultdict(list), {}
    j = np.searchsorted(tt, cts)
    for fi, t in enumerate(cts):
        if fi % stride:
            continue
        k = j[fi]
        cand = [x for x in (k-1, k) if 0 <= x < len(tt)]
        if not cand:
            continue
        kk = min(cand, key=lambda x: abs(tt[x]-t))
        if abs(tt[kk]-t) > 0.10:
            continue
        omf[fi] = float(om[kk])
        Tcm = np.linalg.inv(T[kk] @ X)
        for o in inst:
            p = Tcm[:3, :3] @ o["p"] + Tcm[:3, 3]
            if p[2] <= 0.05:
                continue
            u = K[0, 0]*p[0]/p[2] + K[0, 2]; v = K[1, 1]*p[1]/p[2] + K[1, 2]
            if 0 <= u < W and 0 <= v < H:
                cells[fi].append((o["object_name"], o["id"], float(u), float(v), float(p[2])))
    return cells, omf, float(K[0, 0]), W, H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stride", type=int, default=6)
    ap.add_argument("--tol", type=float, default=0.30, help="depth 일치 허용 [m]")
    ap.add_argument("--margin", type=int, default=16)
    ap.add_argument("--out", default=os.path.join(REPO, "outputs", "phase41_distance_check", "cells.csv"))
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)

    ext = json.load(open(f"{ROOT}/integration/cad_extents.json"))
    size_m = {k: max(v["size_mm"])/1000.0 for k, v in ext.items()}

    import copy
    defaults, cfgo = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device", "cuda:0")
    dev = dev if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(copy.deepcopy(cfgo), model, dev, False)
    n2o = {o["name"]: o for o in objs}
    prompts, groups = o_n.build_prompt_groups(objs)
    tsim = o_n.template_similarity(objs)
    tau = o_n._color_tiebreak_tau(objs[0])
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    ms = min(float(o.get("score_threshold", 0.02)) for o in objs)
    sz = int(defaults.get("imgsz", 640))

    sessions = sorted(d for d in os.listdir(OUTR)
                      if os.path.isfile(f"{OUTR}/{d}/fused/orb3/objects.json")
                      and not d.endswith("_phase39"))
    fields = ["session", "frame", "object", "inst", "u", "v", "z_m", "px", "edge",
              "roi_luma", "roi_std", "frame_luma", "omega", "stage"]
    fh = open(a.out, "w", newline=""); wr = csv.DictWriter(fh, fieldnames=fields); wr.writeheader()

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)

    for sess in sessions:
        cells, omf, fx, W, H = session_cells(sess, a.stride, size_m, a.tol)
        frames = sorted(cells)
        want, img, dep, need = set(frames), {}, {}, {}
        with AnyReader([Path(f"{DATR}/{sess}/sam")], default_typestore=ts) as rd:
            ccon = [c for c in rd.connections if c.topic == COLOR]
            dcon = [c for c in rd.connections if c.topic == DEPTH]
            cts = [t for _, t, _ in rd.messages(connections=ccon)]
            dts = [t for _, t, _ in rd.messages(connections=dcon)]
            for fi in frames:
                k = bisect.bisect_left(dts, cts[fi])
                cd = [x for x in (k-1, k) if 0 <= x < len(dts)]
                if cd:
                    need[fi] = min(cd, key=lambda x: abs(dts[x]-cts[fi]))
            i = -1
            for con, _t, raw in rd.messages(connections=ccon):
                i += 1
                if i in want:
                    m = rd.deserialize(raw, con.msgtype)
                    b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                    img[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
            dw = set(need.values()); i = -1
            for con, _t, raw in rd.messages(connections=dcon):
                i += 1
                if i in dw:
                    m = rd.deserialize(raw, con.msgtype)
                    dep[i] = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        kept = 0
        t0 = time.time()
        for n, fi in enumerate(frames, 1):
            bgr = img.get(fi)
            d = dep.get(need.get(fi, -1))
            if bgr is None or d is None:
                continue
            h, w = bgr.shape[:2]
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            fl = round(float(gray.mean()), 2)
            # depth 로 '정말 보인다'가 확인된 셀만 남긴다
            vis = []
            for name, iid, u, v, z in cells[fi]:
                uu, vv = int(round(u)), int(round(v))
                if not (0 <= uu < d.shape[1] and 0 <= vv < d.shape[0]):
                    continue
                win = d[max(0, vv-4):vv+5, max(0, uu-4):uu+5].astype(np.float32).ravel()
                win = win[win > 0]
                if not win.size:
                    continue
                if abs(float(np.median(win))/1000.0 - z) > a.tol:
                    continue
                vis.append((name, iid, u, v, z))
            if not vis:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            nf = yi.normalize_rgb(rgb)
            with o_n.multi_label_nms(o_n._multi_label_on(objs[0])):
                pr = yolo.predict(bgr, conf=ms, imgsz=sz, verbose=False, device=dev)
            uniq = []
            if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
                b = pr[0].boxes
                for j2 in range(len(b)):
                    xy = b.xyxy[j2].tolist()
                    x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                    x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                    cf = float(b.conf[j2])
                    if cf < ms:
                        continue
                    for uq in uniq:
                        if o_n._iou_xyxy(uq["box"], [x1, y1, x2, y2]) > 0.95:
                            uq["conf"] = max(uq["conf"], cf); break
                    else:
                        uniq.append({"box": [x1, y1, x2, y2], "conf": cf})
            crops, keep = [], []
            for uq in uniq:
                c = yi.crop_resize_pad(nf, uq["box"])
                if c is not None:
                    crops.append(c); keep.append(uq)
            sems = masks = cls_all = patch_all = None
            if crops:
                nb = sorted({x for o in objs for x in o["_need_blocks"]})
                cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, dev, nb)
                masks = yi.segment_boxes(seg, bgr, [uq["box"] for uq in keep], dev)
                sems = [{o["name"]: float(yi.semantic_score(cls_all[k2], o["tcls"], o["match_topk"]))
                         for o in objs} for k2 in range(len(keep))]
            for name, iid, u, v, z in vis:
                kept += 1
                px = fx * size_m.get(name, 0.12) / max(z, 1e-3)
                r = max(6, int(px/2))
                gx1, gy1 = max(0, int(u)-r), max(0, int(v)-r)
                gx2, gy2 = min(w, int(u)+r), min(h, int(v)+r)
                g = gray[gy1:gy2, gx1:gx2]
                if not keep:
                    st = "A_no_box"
                else:
                    hits = [k2 for k2, uq in enumerate(keep)
                            if uq["box"][0]-a.margin <= u <= uq["box"][2]+a.margin
                            and uq["box"][1]-a.margin <= v <= uq["box"][3]+a.margin]
                    if not hits:
                        st = "B_no_box_on_obj"
                    else:
                        best = None
                        for k2 in hits:
                            mask = masks[k2]
                            hsv = ({o["name"]: float(ism_hsv.shadow_score(bgr, keep[k2]["box"], mask,
                                                                         o.get("_hsv_proto")))
                                    for o in objs} if mask is not None else {})
                            top = max(sems[k2], key=sems[k2].get)
                            if tau > 0 and hsv:
                                tie = [x for x in sems[k2] if tsim[top].get(x, 0.0) >= tau]
                                if len(tie) > 1:
                                    top = max(tie, key=lambda x: hsv[x])
                            if top != name:
                                s2 = "C_lost"
                            else:
                                o = n2o[name]
                                if sems[k2][name] < o["similarity_threshold"]:
                                    s2 = "D_sem"
                                elif mask is None:
                                    s2 = "E_appe"
                                else:
                                    qb = {x: patch_all[x][k2].cpu() for x in o["_need_blocks"]}
                                    bt = int(torch.argmax(o["tcls"] @ cls_all[k2]))
                                    apv = float(o_n.masked_appe_blocks(qb, mask, keep[k2]["box"],
                                                                       pool, o, bt))
                                    if apv < o_n._appe_gate_of(o):
                                        s2 = "E_appe"
                                    elif bool(o.get("hsv_gate_enabled", False)) and hsv and \
                                            hsv[name] < float(o.get("hsv_gate_threshold", 0.0)):
                                        s2 = "F_hsv"
                                    else:
                                        s2 = "OK"
                            if best is None or ORDER.index(s2) > ORDER.index(best):
                                best = s2
                        st = best
                wr.writerow(dict(session=sess, frame=fi, object=name, inst=iid,
                                 u=round(u, 1), v=round(v, 1), z_m=round(z, 3), px=round(px, 1),
                                 edge=int(u < 80 or u > w-80 or v < 80 or v > h-80),
                                 roi_luma=round(float(g.mean()), 1) if g.size else 0,
                                 roi_std=round(float(g.std()), 1) if g.size else 0,
                                 frame_luma=fl, omega=round(omf.get(fi, 0.0), 1), stage=st))
        print(f"[{sess}] 확인된 가시 셀 {kept} · 프레임 {len(frames)} ({time.time()-t0:.0f}s)", flush=True)
    fh.close()
    print("wrote", a.out)


if __name__ == "__main__":
    main()
