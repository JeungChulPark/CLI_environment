#!/usr/bin/env python3
"""phase37 — "0807 190624 에서 YOLO-World 프롬프트 박스가 아예 안 생긴다"의 영상 증거.

고른 사례는 전부 **±20 프레임 안에서 같은 객체가 정상 검출된** L1a(후보 0) 셀이다.
즉 물체는 확실히 거기 있고 인식 가능한데 그 프레임에서만 해당 프롬프트의 후보가 0개다.

프레임당 4분할 PNG (모든 칸 640x480, 라벨은 OpenCV 한계로 영문):
  A  deployed: 10 prompts in ONE shared pass @640      -> 대상 프롬프트 박스 0개
  B  zoom on the projected object location             -> 물체는 분명히 보인다
  C  same frame, same pass, multi_label=True           -> 박스가 되살아나는가
  D  same frame brightened (gamma) then deployed pass  -> 밝기로는 되살아나지 않는다

초록 십자 = 융합 객체맵에서 이 프레임으로 역투영한 객체 위치.
"""
import argparse, csv, json, os, sys
import cv2, numpy as np

WS = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
sys.path.insert(0, WS)
sys.path.insert(0, os.path.join(WS, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism_object_n as o_n

COLOR = "/camera/camera/color/image_raw"
BAG = ("/home/ldh9501/temp_ws/CLI_environment/integration/data/0807_chungbuk/"
       "190624__irregular_multi_loop/sam")
CELL = (640, 480)


def gamma_to_mean(bgr, target):
    m = float(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).mean())
    if m <= 1 or m >= target:
        return bgr.copy(), 1.0
    g = float(np.log(target / 255.0) / np.log(m / 255.0))
    lut = np.clip(((np.arange(256) / 255.0) ** g) * 255.0, 0, 255).astype(np.uint8)
    return cv2.LUT(bgr, lut), g


def draw(img, boxes, color, hi=None):
    for name, (x1, y1, x2, y2), c in boxes:
        col = (0, 255, 0) if (hi and name == hi) else color
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 3 if (hi and name == hi) else 1)
        t = f"{name} {c:.3f}"
        (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        y = max(th + 4, y1)
        cv2.rectangle(img, (x1, y - th - 4), (x1 + tw + 3, y), col, -1)
        cv2.putText(img, t, (x1 + 2, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)


def cross(img, u, v, r=18, color=(0, 255, 0)):
    u, v = int(round(u)), int(round(v))
    cv2.line(img, (u - r, v), (u + r, v), color, 2)
    cv2.line(img, (u, v - r), (u, v + r), color, 2)
    cv2.circle(img, (u, v), 20, color, 1)


def cell(img, title, sub=""):
    img = cv2.resize(img, CELL)
    out = np.full((CELL[1] + 44, CELL[0], 3), 25, np.uint8)
    out[44:] = img
    cv2.putText(out, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    if sub:
        cv2.putText(out, sub, (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 220, 255), 1, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", default="evidence_cases.json")
    ap.add_argument("--outdir", default="evidence")
    ap.add_argument("--target-luma", type=float, default=132.0)
    ap.add_argument("--max-per-object", type=int, default=1)
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    prompt_of = {o["name"]: o["yolo_prompt"] for o in objs}
    prompts, _ = o_n.build_prompt_groups(objs)
    cases = json.load(open(a.cases))

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)
    need = {c["frame"] for c in cases}
    img, i = {}, -1
    with AnyReader([Path(BAG)], default_typestore=ts) as rd:
        ccon = [c for c in rd.connections if c.topic == COLOR]
        for con, _t, raw in rd.messages(connections=ccon):
            i += 1
            if i in need:
                m = rd.deserialize(raw, con.msgtype)
                b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                img[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()

    from ultralytics.utils import nms as N
    _orig = N.non_max_suppression
    ML = {"on": False}

    def patched(*ar, **kw):
        if ML["on"]:
            kw["multi_label"] = True
        return _orig(*ar, **kw)
    N.non_max_suppression = patched

    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)

    def boxes(im, conf=0.02):
        r = yolo.predict(im, conf=conf, imgsz=640, verbose=False, device="cuda:0")
        o = []
        if len(r) and r[0].boxes is not None and len(r[0].boxes):
            b = r[0].boxes
            for j in range(len(b)):
                x = [int(v) for v in b.xyxy[j].tolist()]
                o.append((prompts[int(b.cls[j])], x, float(b.conf[j])))
        return o

    made = {}
    rowsum = []
    for c in cases:
        obj, f = c["object"], c["frame"]
        if made.get(obj, 0) >= a.max_per_object:
            continue
        u, v, pr = c["u"], c["v"], prompt_of[obj]
        base = img[f]
        on = lambda bs: [b for b in bs if b[0] == pr
                         and b[1][0] - 16 <= u <= b[1][2] + 16 and b[1][1] - 16 <= v <= b[1][3] + 16]

        ML["on"] = False
        bA = boxes(base)
        ML["on"] = True
        bC = boxes(base)
        bright, g = gamma_to_mean(base, a.target_luma)
        ML["on"] = False
        bD = boxes(bright)
        nA, nC, nD = len(on(bA)), len(on(bC)), len(on(bD))

        A_ = base.copy(); draw(A_, bA, (0, 200, 255)); cross(A_, u, v)
        A_ = cell(A_, f"A  DEPLOYED  10 prompts, one shared pass @640   frame {f}  luma {c['frame_luma']:.0f}",
                  f"boxes {len(bA)}   but '{pr}' on object: {nA}")
        s = 100
        x1, y1 = max(0, int(u) - s), max(0, int(v) - s)
        x2, y2 = min(base.shape[1], int(u) + s), min(base.shape[0], int(v) + s)
        cr = cv2.resize(base[y1:y2, x1:x2], CELL, interpolation=cv2.INTER_CUBIC)
        cross(cr, (u - x1) * CELL[0] / (x2 - x1), (v - y1) * CELL[1] / (y2 - y1), r=26)
        B_ = cell(cr, f"B  ZOOM on the projected object location",
                  f"{obj}  at {c['z']:.2f} m  -- the object is plainly visible")
        C0 = base.copy(); draw(C0, bC, (0, 160, 200), hi=pr); cross(C0, u, v)
        C_ = cell(C0, "C  SAME frame, SAME single pass, multi_label=True",
                  f"'{pr}' on object: {nC}" + (f"  conf {max(x[2] for x in on(bC)):.3f}" if nC else ""))
        D0 = bright.copy(); draw(D0, bD, (255, 150, 60)); cross(D0, u, v)
        D_ = cell(D0, f"D  SAME frame brightened (gamma {g:.2f} -> mean {a.target_luma:.0f}), deployed pass",
                  f"'{pr}' on object: {nD}   brightness alone does not bring the box back" if not nD
                  else f"'{pr}' on object: {nD}")
        out = np.vstack([np.hstack([A_, B_]), np.hstack([C_, D_])])
        p = os.path.join(a.outdir, f"{obj}_f{f:06d}.png")
        cv2.imwrite(p, out)
        made[obj] = made.get(obj, 0) + 1
        rowsum.append(dict(object=obj, frame=f, prompt=pr, luma=c["frame_luma"], z=c["z"],
                           deployed=nA, multilabel=nC, brightened=nD,
                           conf_ml=round(max([x[2] for x in on(bC)], default=0.0), 4), png=os.path.basename(p)))
        print(f"  {p}  deployed {nA} | multi_label {nC} | brightened {nD}")
    json.dump(rowsum, open(os.path.join(a.outdir, "summary.json"), "w"), indent=1)
    print(f"\n{len(rowsum)}건 · deployed로 살아난 것 {sum(r['deployed'] for r in rowsum)} "
          f"· multi_label {sum(bool(r['multilabel']) for r in rowsum)} "
          f"· 밝기보정 {sum(bool(r['brightened']) for r in rowsum)}")


if __name__ == "__main__":
    main()
