#!/usr/bin/env python3
"""build_candidate_choice_task.py — YOLO top-3 후보 중 '정답이 무엇인가' 사람판정 과제 (READ-ONLY).

목적: 자동 귀속으로는 확정할 수 없던 것을 사람이 직접 가른다.
  ① 선택된 후보가 정답            → 파이프라인 정상(다른 이유로 FN)
  ② 다른 후보가 정답인데 못 골랐다 → 선택(selection) 실패
  ③ 후보 중에 정답이 없다         → YOLO proposal/localization 실패

대상: 후보가 있는 FN 셀(candidate_pool 기준). 각 셀마다 top-3 후보 전부를 보여준다.

패널: [전체 프레임에 ①②③ 박스(선택된 것은 굵게+★)] + [후보 crop 3장] + [TARGET template]
답: 1 / 2 / 3 (그 번호가 정답) · 0 (정답 없음) · U (불확실)

산출: outputs/phase25_candidate_gt/
  cases/<case_id>.png
  candidate_choice_task.csv   (answer 열만 채우면 됨)
"""
import argparse, csv, glob, os, sys
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism_object_n as o_n

OUT = os.path.join(REPO, "outputs", "phase25_candidate_gt")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
POOL = os.path.join(REPO, "outputs", "phase22_training_free_gate", "csv", "candidate_pool.csv")
IDX = os.path.join(REPO, "outputs", "phase21_fn_root_cause_audit", "csv", "gt_object_index.csv")
FONTP = "/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf"
CNUM = [(60, 200, 255), (90, 230, 90), (250, 150, 90)]     # 후보 1,2,3 색 (BGR)


def font(sz):
    try:
        return ImageFont.truetype(FONTP, sz)
    except Exception:
        return ImageFont.load_default()


def template_strip(tdir, w=300, h=150):
    ps = sorted(glob.glob(os.path.join(tdir, "rgb_*.png")))
    if not ps:
        return np.full((h, w, 3), 40, np.uint8)
    ims = []
    for p in [ps[0], ps[len(ps) // 3]]:
        im = cv2.imread(p)
        ims.append(cv2.resize(im if im is not None else np.full((h, w // 2, 3), 40, np.uint8),
                              (w // 2, h)))
    return np.hstack(ims)


def build_panel(ds, fr, obj, cands, tdir):
    bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
    if bgr is None:
        return None
    H, W = bgr.shape[:2]
    full = bgr.copy()
    for i, c in enumerate(cands):
        x1, y1, x2, y2 = c["box"]; col = CNUM[i % 3]
        th = 4 if c["sel"] else 2
        cv2.rectangle(full, (x1, y1), (x2, y2), col, th)
        tag = f"{i+1}" + ("*" if c["sel"] else "")
        cv2.rectangle(full, (x1, max(0, y1 - 26)), (x1 + 34, y1), col, -1)
        cv2.putText(full, tag, (x1 + 4, max(16, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    full = cv2.resize(full, (760, 570))

    # 후보 crop 행
    tiles = []
    for i, c in enumerate(cands):
        x1, y1, x2, y2 = c["box"]; col = CNUM[i % 3]
        crop = bgr[max(0, y1 - 12):min(H, y2 + 12), max(0, x1 - 12):min(W, x2 + 12)]
        crop = cv2.resize(crop if crop.size else bgr, (240, 240))
        cv2.rectangle(crop, (0, 0), (239, 239), col, 6)
        cap = np.full((46, 240, 3), 22, np.uint8)
        cv2.putText(cap, f"{i+1}" + ("  <= SELECTED" if c["sel"] else ""), (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        cv2.putText(cap, f"conf {c['conf']:.3f}", (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (190, 190, 190), 1)
        tiles.append(np.vstack([crop, cap]))
    while len(tiles) < 3:
        tiles.append(np.full((286, 240, 3), 18, np.uint8))
    row = np.hstack(tiles)
    tpl = np.full((286, 300, 3), 25, np.uint8)
    tpl[8:158, 0:300] = template_strip(tdir)
    cv2.putText(tpl, "TARGET template", (8, 182), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
    cv2.putText(tpl, "(the object to find)", (8, 206), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 210, 170), 1)
    bottom = np.hstack([row, tpl])
    bottom = cv2.resize(bottom, (760, int(bottom.shape[0] * 760 / bottom.shape[1])))
    left = np.vstack([full, bottom])

    lines = [(f"{ds}  frame {fr}", 16, (200, 200, 200)),
             (f"대상 class: {obj}", 20, (255, 255, 255)),
             ("", 6, (0, 0, 0)),
             ("YOLO top-3 후보 중 어느 것이", 16, (200, 220, 255)),
             ("실제 대상 물체 위에 있습니까?", 16, (200, 220, 255)),
             ("", 5, (0, 0, 0)),
             ("  1 / 2 / 3 = 그 번호가 정답", 15, (150, 230, 150)),
             ("  0 = 정답인 후보가 없음", 15, (230, 150, 150)),
             ("  U = 불확실", 15, (200, 200, 200)),
             ("", 6, (0, 0, 0)),
             ("★ = 파이프라인이 고른 후보", 14, (255, 220, 120)),
             ("", 4, (0, 0, 0)),
             ("현재 이 셀은 FN(미검출)입니다.", 14, (230, 180, 180))]
    panel = Image.new("RGB", (420, left.shape[0]), (24, 24, 28))
    d = ImageDraw.Draw(panel); y = 12
    for t, s, c in lines:
        d.text((12, y), t, font=font(s), fill=c); y += s + 6
    canvas = Image.new("RGB", (left.shape[1] + 420, left.shape[0]), (0, 0, 0))
    canvas.paste(Image.fromarray(cv2.cvtColor(left, cv2.COLOR_BGR2RGB)), (0, 0))
    canvas.paste(panel, (left.shape[1], 0))
    return cv2.cvtColor(np.array(canvas), cv2.COLOR_RGB2BGR)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--only", default=""); a = ap.parse_args()
    os.makedirs(os.path.join(OUT, "cases"), exist_ok=True)
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}

    label = {(r["dataset"], int(r["frame_id"]), r["object"]): r["phase1c_label"]
             for r in csv.DictReader(open(IDX))}
    cells = {}
    for r in csv.DictReader(open(POOL)):
        k = (r["dataset"], int(r["frame_id"]), r["object"])
        if label.get(k) != "FN":
            continue
        x1, y1, x2, y2 = [int(v) for v in r["uid"].rsplit("|", 1)[1].split("_")]
        cells.setdefault(k, []).append({"box": (x1, y1, x2, y2), "conf": float(r["conf"]),
                                        "rank": int(r["conf_rank"]), "sel": r["is_selected"] == "1",
                                        "uid": r["uid"]})
    rows = []
    # 클래스 번갈아 배치
    order = sorted(cells, key=lambda k: (k[2], k[0], k[1]))
    byc = {}
    for k in order:
        byc.setdefault(k[2], []).append(k)
    inter = []
    while any(byc.values()):
        for c in list(byc):
            if byc[c]:
                inter.append(byc[c].pop(0))
    for k in inter:
        ds, fr, obj = k
        cs = sorted(cells[k], key=lambda c: c["rank"])
        cid = f"{ds}_{fr:06d}_{obj}"
        if a.only and cid != a.only:
            continue
        img = build_panel(ds, fr, obj, cs, tdir[obj])
        if img is None:
            continue
        rel = os.path.join("cases", f"{cid}.png")
        cv2.imwrite(os.path.join(OUT, rel), img)
        rows.append({"case_id": cid, "bag_name": ds, "frame_id": fr, "class_name": obj,
                     "n_cands": len(cs),
                     "selected_rank": next((i + 1 for i, c in enumerate(cs) if c["sel"]), ""),
                     "conf1": round(cs[0]["conf"], 4) if len(cs) > 0 else "",
                     "conf2": round(cs[1]["conf"], 4) if len(cs) > 1 else "",
                     "conf3": round(cs[2]["conf"], 4) if len(cs) > 2 else "",
                     "image_path": rel, "answer": "", "notes": ""})
    p = os.path.join(OUT, "candidate_choice_task.csv")
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    from collections import Counter
    print("=== 후보 선택 GT 과제 생성 ===")
    print(f"  대상: 후보 있는 FN 셀 {len(rows)}건")
    print("  후보 수 분포:", dict(Counter(r["n_cands"] for r in rows)))
    print(f"  -> {p}")


if __name__ == "__main__":
    main()
