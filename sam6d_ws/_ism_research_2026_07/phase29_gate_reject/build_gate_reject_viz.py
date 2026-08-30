#!/usr/bin/env python3
"""build_gate_reject_viz.py — '정답 후보인데 게이트에서 탈락'한 사례 시각화 (READ-ONLY).

대상: 사람이 정답으로 확인한 후보 189건(phase25 판정). 이 박스들은 실제 대상 위이다.
각 패널에 세 게이트 점수를 **임계 대비 막대**로 그려 왜 탈락했는지 한눈에 보이게 한다.

패널: [프레임+박스 | 후보 crop | TARGET template | 점수막대]
산출: outputs/phase29_gate_reject/{cases,csv}/
"""
import csv, glob, json, os, sys
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism_object_n as o_n

OUT = os.path.join(REPO, "outputs", "phase29_gate_reject")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
POOL = os.path.join(REPO, "outputs", "phase22_training_free_gate", "csv", "candidate_pool.csv")
CHOICE = os.path.join(REPO, "outputs", "phase25_candidate_gt", "candidate_choice_task.csv")
VISCSV = os.path.join(REPO, "outputs", "phase23_location_gt", "fp_fn_verification_task.csv")
FONTP = "/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf"


def font(s):
    try: return ImageFont.truetype(FONTP, s)
    except Exception: return ImageFont.load_default()


def bar_panel(c, w=420, h=560):
    """세 게이트 점수를 임계 대비 비율 막대로."""
    im = Image.new("RGB", (w, h), (24, 24, 28)); d = ImageDraw.Draw(im)
    y = 14
    d.text((14, y), "게이트 점수 (임계 대비)", font=font(18), fill=(255, 220, 120)); y += 34
    for lab, s, t in (("semantic", c["sem"], c["st"]),
                      ("appearance", c["appe"], c["ag"]),
                      ("HSV", c["hsv"], c["ht"])):
        ratio = s / t if t else 0
        ok = s >= t
        col = (110, 220, 130) if ok else (235, 110, 110)
        d.text((14, y), f"{lab}", font=font(16), fill=(225, 225, 225))
        d.text((150, y), f"{s:.3f} / {t:.3f}", font=font(15), fill=(190, 190, 190))
        d.text((300, y), f"×{ratio:.2f}", font=font(16), fill=col)
        y += 24
        # 막대: 1.0 = 임계
        BW = 380; x0 = 14
        d.rectangle([x0, y, x0 + BW, y + 16], fill=(45, 48, 55))
        d.rectangle([x0, y, x0 + int(BW * min(ratio, 1.6) / 1.6), y + 16], fill=col)
        tx = x0 + int(BW * (1.0 / 1.6))
        d.line([tx, y - 3, tx, y + 19], fill=(255, 255, 255), width=2)   # 임계선
        y += 34
    d.text((14, y), "흰 선 = 통과 임계", font=font(13), fill=(160, 160, 160)); y += 30
    d.text((14, y), f"판정: {'ACCEPT' if c['acc'] else 'REJECT'}", font=font(20),
           fill=(110, 220, 130) if c["acc"] else (235, 110, 110)); y += 32
    if not c["acc"]:
        fails = [n for n, ok in (("semantic", c["sem"] >= c["st"]), ("appearance", c["appe"] >= c["ag"]),
                                 ("HSV", c["hsv"] >= c["ht"])) if not ok]
        d.text((14, y), "탈락 게이트: " + ", ".join(fails), font=font(16), fill=(235, 150, 150)); y += 28
        worst = min((c["sem"] / c["st"], "semantic"), (c["appe"] / c["ag"], "appearance"),
                    (c["hsv"] / c["ht"], "HSV"))
        d.text((14, y), f"가장 부족: {worst[1]} (×{worst[0]:.2f})", font=font(15), fill=(220, 180, 180)); y += 34
    d.text((14, y), f"가시등급: {c['vis']}  (1온전 2일부 3극히일부)", font=font(14), fill=(180, 200, 230)); y += 24
    d.text((14, y), f"YOLO conf: {c['conf']:.3f}", font=font(14), fill=(170, 170, 170))
    return cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)


def main():
    os.makedirs(os.path.join(OUT, "cases"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}
    pool = {}
    for r in csv.DictReader(open(POOL)):
        k = (r["dataset"], int(r["frame_id"]), r["object"], int(r["conf_rank"]))
        pool[k] = r
    vis = {}
    for r in csv.DictReader(open(VISCSV)):
        if r["group"] == "fn_check":
            vis[(r["bag_name"], int(r["frame_id"]), r["class_name"])] = r["verdict"].strip()
    rows = []
    for r in csv.DictReader(open(CHOICE)):
        a = r["answer"].strip().upper()
        if a in ("0", "U"): continue
        ds, fr, ob = r["bag_name"], int(r["frame_id"]), r["class_name"]
        p = pool.get((ds, fr, ob, int(a)))
        if not p: continue
        c = dict(sem=float(p["sem_top5"]), appe=float(p["appe11"]), hsv=float(p["hsv"]),
                 st=float(p["sim_thr"]), ag=float(p["appe_gate"]), ht=float(p["hsv_thr"]),
                 conf=float(p["conf"]), vis=vis.get((ds, fr, ob), "?"))
        c["acc"] = c["sem"] >= c["st"] and c["appe"] >= c["ag"] and c["hsv"] >= c["ht"]
        x1, y1, x2, y2 = [int(v) for v in p["uid"].rsplit("|", 1)[1].split("_")]
        bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
        if bgr is None: continue
        H, W = bgr.shape[:2]
        full = bgr.copy(); cv2.rectangle(full, (x1, y1), (x2, y2), (0, 0, 255), 3)
        full = cv2.resize(full, (620, 465))
        crop = bgr[max(0, y1-15):min(H, y2+15), max(0, x1-15):min(W, x2+15)]
        crop = cv2.resize(crop if crop.size else bgr, (280, 280))
        tps = sorted(glob.glob(os.path.join(tdir[ob], "rgb_*.png")))
        tp = cv2.imread(tps[len(tps)//2]) if tps else None
        tp = cv2.resize(tp if tp is not None else np.full((280,280,3),40,np.uint8), (280, 280))
        cv2.putText(crop, "candidate", (6,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        cv2.putText(tp, "TARGET", (6,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,0), 2)
        mid = np.vstack([np.hstack([crop, tp]), np.full((465-280, 560, 3), 18, np.uint8)])
        left = np.hstack([full, mid])
        panel = bar_panel(c, 420, left.shape[0])
        img = np.hstack([left, panel])
        cid = f"{ds}_{fr:06d}_{ob}"
        cv2.imwrite(os.path.join(OUT, "cases", f"{cid}.png"), img)
        fails = [n for n, ok in (("semantic", c["sem"]>=c["st"]), ("appearance", c["appe"]>=c["ag"]),
                                 ("hsv", c["hsv"]>=c["ht"])) if not ok]
        rows.append({"case_id": cid, "bag": ds, "frame": fr, "class_name": ob,
                     "accept": int(c["acc"]), "fail_gates": ";".join(fails),
                     "sem_ratio": round(c["sem"]/c["st"], 3), "appe_ratio": round(c["appe"]/c["ag"], 3),
                     "hsv_ratio": round(c["hsv"]/c["ht"], 3),
                     "min_ratio": round(min(c["sem"]/c["st"], c["appe"]/c["ag"], c["hsv"]/c["ht"]), 3),
                     "visibility": c["vis"], "conf": round(c["conf"], 3),
                     "image": f"{cid}.png"})
    with open(os.path.join(OUT, "csv", "gate_reject_cases.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    rej = [r for r in rows if not r["accept"]]
    print(f"정답 후보 {len(rows)}건 · 게이트 탈락 {len(rej)}건")
    from collections import Counter
    print("탈락 게이트 조합:", dict(Counter(r["fail_gates"] for r in rej).most_common()))
    print(f"아깝게(min_ratio>=0.9) {sum(1 for r in rej if r['min_ratio']>=0.9)}건")
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
