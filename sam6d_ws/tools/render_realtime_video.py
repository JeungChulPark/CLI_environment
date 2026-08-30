#!/usr/bin/env python3
"""render_realtime_video.py — 실시간 실행 결과를 좌우 비교 mp4 로 만든다.

왼쪽  bag 이 흘려보낸 원본 영상 (모든 프레임, 실시간 순서)
오른쪽 그 순간 SAM-6D 가 **가장 최근에 내놓은** 결과를 얹은 화면
       - CAD 점군을 추정 포즈로 투영
       - 객체마다 Δrot = 직전 추정 대비 회전 변화량 [deg]  <- 흔들림의 직접 지표
       - 아래 띠: 시간에 따른 Δrot 이력

오른쪽이 왼쪽보다 늦게 갱신되는 것이 곧 처리 지연이다. 갱신이 없는 동안에는 직전 결과가
그대로 남아 있으므로, '얼마나 드물게, 얼마나 흔들리며' 갱신되는지가 한눈에 보인다.
"""
import argparse, glob, json, os, sys
from collections import defaultdict, deque

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COLOR = "/camera/camera/color/image_raw"
PALETTE = [(90, 220, 90), (60, 160, 255), (240, 160, 60), (200, 100, 240),
           (80, 220, 220), (140, 140, 255), (120, 200, 160), (255, 200, 120),
           (170, 120, 255), (110, 240, 180)]


def geodesic_deg(A, B):
    c = np.clip((np.trace(np.asarray(A).T @ np.asarray(B)) - 1) / 2, -1, 1)
    return float(np.degrees(np.arccos(c)))


def load_model_points(cfg_objs, n=1500):
    """PEM 이 쓰던 샘플 점 캐시를 재사용한다. 없으면 CAD 를 직접 샘플링."""
    import hashlib
    cdir = os.path.join(REPO, "outputs", "pem_inputs", "_model_pts_cache")
    pts = {}
    for o in cfg_objs:
        cad = o.get("cad_abs")
        if not cad:
            continue
        key = hashlib.md5(cad.encode()).hexdigest()[:16]
        p = os.path.join(cdir, f"{key}.npy")
        if os.path.isfile(p):
            a = np.load(p)
        else:
            try:
                import trimesh
                a = trimesh.load_mesh(cad).sample(8192)
            except Exception:
                continue
        a = np.asarray(a, dtype=np.float32)
        if len(a) > n:
            a = a[np.random.RandomState(0).choice(len(a), n, replace=False)]
        pts[o["name"]] = a / 1000.0            # CAD 는 mm
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--results", required=True, help="realtime_sam6d_node 가 쓴 results.jsonl")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--step", type=int, default=2, help="원본 N 프레임마다 한 장")
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args()

    sys.path.insert(0, REPO)
    import yolo_ism_object_n as o_n
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    for o in objs:
        c = o_n._abspath(o.get("cad_ply", ""))
        o["cad_abs"] = c if os.path.isfile(c) else None
    MP = load_model_points(objs)
    cidx = {o["name"]: i for i, o in enumerate(objs)}
    print(f"[cad] 점군 {len(MP)} 객체")

    recs = [json.loads(l) for l in open(a.results)]
    recs.sort(key=lambda r: r["stamp"])
    print(f"[res] 결과 {len(recs)}건, bag 시각 {recs[0]['stamp']:.1f}~{recs[-1]['stamp']:.1f}")
    # 처리 시작~완료를 bag 시각으로 환산: 벽시계 경과분을 그대로 더한다
    t_wall0, t_bag0 = recs[0]["t_start"], recs[0]["stamp"]
    for r in recs:
        r["ready_bag"] = t_bag0 + (r["t_done"] - t_wall0)     # 이 결과가 화면에 뜨는 시각
        r["lat"] = r["t_done"] - r["t_start"]

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)
    frames = []
    with AnyReader([Path(a.bag)], default_typestore=ts) as rd:
        ccon = [c for c in rd.connections if c.topic == COLOR]
        i = -1
        for con, _t, raw in rd.messages(connections=ccon):
            i += 1
            if i % a.step:
                continue
            m = rd.deserialize(raw, con.msgtype)
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if st < recs[0]["stamp"] - 0.5 or st > recs[-1]["ready_bag"] + 3.0:
                continue
            b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            img = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
            frames.append((st, img))
            if a.max_frames and len(frames) >= a.max_frames:
                break
    print(f"[bag] 프레임 {len(frames)}")
    if not frames:
        raise SystemExit("겹치는 프레임이 없다")

    K = None
    info = os.path.join(os.path.dirname(a.bag.rstrip("/")), "info.json")
    if os.path.isfile(info):
        K = np.array(json.load(open(info))["sam"]["camera"]["K"]).reshape(3, 3)
    if K is None:
        K = np.array([[387.0, 0, 322.7], [0, 386.5, 244.8], [0, 0, 1]])

    # 객체별 회전 이력 -> Δrot
    prevR, hist = {}, defaultdict(lambda: deque(maxlen=400))
    for r in recs:
        for d in r["dets"]:
            nm = d["object"]
            R = np.array(d["R"])
            d["drot"] = geodesic_deg(prevR[nm], R) if nm in prevR else None
            prevR[nm] = R
            hist[nm].append((r["ready_bag"], d["drot"]))

    h, w = frames[0][1].shape[:2]
    STRIP = 120
    H = h + 34 + STRIP
    vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (w*2, H))
    ri = 0
    shown = None
    for st, img in frames:
        while ri < len(recs) and recs[ri]["ready_bag"] <= st:
            shown = recs[ri]; ri += 1
        left = img.copy()
        right = img.copy()
        if shown is not None:
            for d in shown["dets"]:
                nm = d["object"]
                col = PALETTE[cidx.get(nm, 0) % len(PALETTE)]
                P = MP.get(nm)
                if P is not None:
                    R = np.array(d["R"]); t = np.array(d["t_mm"])/1000.0
                    q = (R @ P.T).T + t
                    q = q[q[:, 2] > 1e-3]
                    if len(q):
                        u = (K[0, 0]*q[:, 0]/q[:, 2] + K[0, 2]).astype(np.int32)
                        v = (K[1, 1]*q[:, 1]/q[:, 2] + K[1, 2]).astype(np.int32)
                        m = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                        right[v[m], u[m]] = col
                x1, y1, x2, y2 = d["bbox"]
                cv2.rectangle(right, (x1, y1), (x2, y2), col, 1)
                lab = nm.replace("_high", "")
                if d.get("drot") is not None:
                    lab += f"  d{d['drot']:.0f}deg"
                cv2.putText(right, lab, (x1+2, max(12, y1-4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        def bar(img_, text):
            b = np.full((34, w, 3), 28, np.uint8)
            cv2.putText(b, text, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (255, 255, 255), 1, cv2.LINE_AA)
            return np.vstack([b, img_])
        age = (st - shown["ready_bag"]) if shown else 0.0
        left = bar(left, f"LIVE  bag t={st-frames[0][0]:6.1f}s")
        right = bar(right, "SAM-6D  " + (
            f"lat {shown['lat']:.1f}s | age {age:4.1f}s | obj {shown['n']}" if shown else "wait"))
        # 아래 띠: Δrot 이력
        strip = np.full((STRIP, w*2, 3), 22, np.uint8)
        cv2.putText(strip, "PEM rotation change between consecutive estimates (deg)",
                    (8, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv2.LINE_AA)
        for gy, gv in ((STRIP-8, 0), (STRIP-8-int(0.5*(STRIP-30)), 90), (22, 180)):
            cv2.line(strip, (60, gy), (w*2-8, gy), (55, 55, 55), 1)
            cv2.putText(strip, f"{gv}", (24, gy+4), cv2.FONT_HERSHEY_SIMPLEX, 0.34,
                        (120, 120, 120), 1, cv2.LINE_AA)
        T0, T1 = frames[0][0], frames[-1][0]
        for nm, hs in hist.items():
            col = PALETTE[cidx.get(nm, 0) % len(PALETTE)]
            pts = [(int(60+(t-T0)/max(T1-T0, 1e-6)*(w*2-70)),
                    int(STRIP-8-min(dv, 180)/180.0*(STRIP-30)))
                   for t, dv in hs if dv is not None and t <= st]
            for p in pts:
                cv2.circle(strip, p, 2, col, -1)
        cv2.line(strip, (int(60+(st-T0)/max(T1-T0, 1e-6)*(w*2-70)), 18),
                 (int(60+(st-T0)/max(T1-T0, 1e-6)*(w*2-70)), STRIP-6), (90, 90, 90), 1)
        vw.write(np.vstack([np.hstack([left, right]), strip]))
    vw.release()
    dr = [d["drot"] for r in recs for d in r["dets"] if d.get("drot") is not None]
    print(f"wrote {a.out}")
    if dr:
        dr = np.array(dr)
        print(f"연속 추정 사이 회전 변화 Δrot: 중앙 {np.median(dr):.1f}deg  "
              f"p90 {np.percentile(dr,90):.1f}  최대 {dr.max():.1f}  "
              f">30deg 비율 {(dr>30).mean():.2f}  (n={len(dr)})")
    lat = np.array([r["lat"] for r in recs])
    print(f"처리 지연: 중앙 {np.median(lat):.2f}s  p90 {np.percentile(lat,90):.2f}s  최대 {lat.max():.2f}s")


if __name__ == "__main__":
    main()
