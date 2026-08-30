#!/usr/bin/env python3
"""build_fp_fn_verification_task.py — FP 86 + FN 313 사람검증 과제 생성 (READ-ONLY).

TP 위치검증(283건)에 이어, 나머지 판정칸의 타당성을 확인한다.

  FP 86  : GT는 "객체 없음"인데 파이프라인이 accept. 정말 없는가?
           있으면 GT 오류(사실 TP) → precision 과소평가였다는 뜻.
  FN 313 : GT는 "객체 보임"인데 파이프라인이 reject. 정말 보이는가? 어느 정도로?
           안 보이면 GT 오류(사실 TN). 가시 정도까지 받아 "검출 가능한 FN"을 분리한다.
           (Phase 2.1 의 gate 222 결론이 이 313 의 진위에 달려 있음)

패널 구성
  FP : 전체프레임(accept 박스 빨강) + 확대 crop + TARGET template
  FN : 전체프레임(깨끗하게, 크게) + TARGET template + [참고] 거절된 후보 crop
       ※ 가시성 판단이 편향되지 않도록 거절 후보는 '참고' 로만 작게 표시

산출: outputs/phase23_location_gt/
  cases_fpfn/<group>/<case_id>.png
  fp_fn_verification_task.csv     (verdict 열만 채우면 됨)
"""
import argparse, csv, glob, os, sys
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit"))
sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism_object_n as o_n

OUT = os.path.join(REPO, "outputs", "phase23_location_gt")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
IDX = os.path.join(REPO, "outputs", "phase21_fn_root_cause_audit", "csv", "gt_object_index.csv")
FONTP = "/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf"


def font(sz):
    try:
        return ImageFont.truetype(FONTP, sz)
    except Exception:
        return ImageFont.load_default()


def parse_bbox(uid):
    return tuple(int(v) for v in uid.rsplit("|", 1)[1].split("_"))


def template_strip(tdir, w=300, h=150):
    ps = sorted(glob.glob(os.path.join(tdir, "rgb_*.png")))
    if not ps:
        return np.full((h, w, 3), 40, np.uint8)
    picks = [ps[0], ps[len(ps) // 3]]
    ims = [cv2.resize(cv2.imread(p) if cv2.imread(p) is not None
                      else np.full((h, w // 2, 3), 40, np.uint8), (w // 2, h)) for p in picks]
    return np.hstack(ims)


def side_panel(height, lines, width=440):
    panel = Image.new("RGB", (width, height), (24, 24, 28))
    d = ImageDraw.Draw(panel); y = 10
    for t, s, c in lines:
        d.text((12, y), t, font=font(s), fill=c); y += s + 6
    return panel


def attach(top_bgr, lines, width=440):
    panel = side_panel(top_bgr.shape[0], lines, width)
    canvas = Image.new("RGB", (top_bgr.shape[1] + width, top_bgr.shape[0]), (0, 0, 0))
    canvas.paste(Image.fromarray(cv2.cvtColor(top_bgr, cv2.COLOR_BGR2RGB)), (0, 0))
    canvas.paste(panel, (top_bgr.shape[1], 0))
    return cv2.cvtColor(np.array(canvas), cv2.COLOR_RGB2BGR)


def panel_fp(ds, fr, obj, uid, tdir, scores):
    bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
    if bgr is None or not uid:
        return None
    x1, y1, x2, y2 = parse_bbox(uid)
    full = bgr.copy(); cv2.rectangle(full, (x1, y1), (x2, y2), (0, 0, 255), 3)
    full = cv2.resize(full, (620, 465))
    h, w = bgr.shape[:2]
    crop = bgr[max(0, y1 - 20):min(h, y2 + 20), max(0, x1 - 20):min(w, x2 + 20)]
    crop = cv2.resize(crop if crop.size else bgr, (300, 300))
    cv2.putText(crop, "accept box", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(crop, "accept box", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
    tpl = np.full((300, 300, 3), 25, np.uint8)
    tpl[10:160, 0:300] = template_strip(tdir)
    cv2.putText(tpl, "TARGET template", (6, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
    right = np.vstack([np.hstack([crop, tpl])])
    top = np.hstack([full, cv2.resize(right, (620, 465))])
    lines = [("[FP 검증]  현재 판정: FP", 19, (255, 180, 180)),
             (f"{ds}  frame {fr}", 16, (200, 200, 200)),
             (f"대상 class: {obj}", 20, (255, 255, 255)),
             ("", 6, (0, 0, 0)),
             ("GT 는 '이 프레임에 이 객체 없음' 이라고", 15, (200, 220, 255)),
             ("되어 있는데 파이프라인은 accept 했습니다.", 15, (200, 220, 255)),
             ("", 5, (0, 0, 0)),
             ("질문: 이 프레임에 이 객체가 실제로 있습니까?", 16, (255, 235, 140)),
             ("  A = 없다 → FP 맞음(오검출)", 15, (230, 230, 230)),
             ("  B = 있고, 빨간 박스가 그 위 → GT오류(사실 TP)", 15, (150, 230, 150)),
             ("  C = 있지만 빨간 박스는 딴 곳", 15, (230, 200, 150)),
             ("  U = 불확실", 15, (200, 200, 200)),
             ("", 6, (0, 0, 0)),
             (scores, 14, (170, 170, 170))]
    return attach(top, lines)


def panel_fn(ds, fr, obj, uid, tdir, scores, reject_reason):
    bgr = cv2.imread(os.path.join(FRAMES, ds, f"frame_{fr:06d}.png"))
    if bgr is None:
        return None
    full = cv2.resize(bgr.copy(), (860, 645))          # 박스 없이 크게 (편향 방지)
    tpl = np.full((645, 320, 3), 25, np.uint8)
    tpl[20:170, 10:310] = cv2.resize(template_strip(tdir), (300, 150))
    cv2.putText(tpl, "TARGET template", (12, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 255, 200), 1)
    cv2.putText(tpl, "(the object to find)", (12, 226), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 210, 170), 1)
    if uid:                                            # 참고용 거절 후보 crop (작게)
        x1, y1, x2, y2 = parse_bbox(uid); h, w = bgr.shape[:2]
        c = bgr[max(0, y1 - 15):min(h, y2 + 15), max(0, x1 - 15):min(w, x2 + 15)]
        if c.size:
            tpl[300:480, 60:260] = cv2.resize(c, (200, 180))
        cv2.putText(tpl, "[ref] rejected candidate", (12, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 180, 130), 1)
        cv2.putText(tpl, "(pipeline guess; may be wrong)", (12, 512), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 180, 130), 1)
    else:
        cv2.putText(tpl, "NO candidate at all", (12, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 150, 150), 1)
    top = np.hstack([full, tpl])
    lines = [("[FN 검증]  현재 판정: FN", 19, (255, 210, 140)),
             (f"{ds}  frame {fr}", 16, (200, 200, 200)),
             (f"대상 class: {obj}", 20, (255, 255, 255)),
             ("", 6, (0, 0, 0)),
             ("GT 는 '보인다' 인데 파이프라인이 reject 했습니다.", 15, (200, 220, 255)),
             ("", 5, (0, 0, 0)),
             ("질문: 이 객체가 실제로 보입니까? 어느 정도로?", 16, (255, 235, 140)),
             ("  1 = 온전히 보임(대부분 노출)", 15, (150, 230, 150)),
             ("  2 = 일부 보임(절반쯤/부분 가림)", 15, (220, 220, 150)),
             ("  3 = 극히 일부(아주 작거나 심한 가림)", 15, (230, 190, 140)),
             ("  4 = 안 보인다 → GT오류(사실 TN)", 15, (230, 150, 150)),
             ("  U = 불확실", 15, (200, 200, 200)),
             ("", 6, (0, 0, 0)),
             (f"거절 사유: {reject_reason}", 14, (180, 180, 180)),
             (scores, 14, (170, 170, 170))]
    return attach(top, lines)


def main():
    ap = argparse.ArgumentParser(); ap.parse_args()
    for d in ("cases_fpfn/fp_check", "cases_fpfn/fn_check"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}

    rows_idx = list(csv.DictReader(open(IDX)))
    # FN 거절 사유(root cause)
    rc = {}
    p = os.path.join(REPO, "outputs", "phase21_fn_root_cause_audit", "csv", "fn_root_cause_per_object.csv")
    for r in csv.DictReader(open(p)):
        rc[(r["dataset"], int(r["frame_id"]), r["object"])] = r["primary_root_cause"]

    out_rows = []
    fp = [r for r in rows_idx if r["phase1c_label"] == "FP"]
    fn = [r for r in rows_idx if r["phase1c_label"] == "FN"]
    # 클래스 번갈아 배치(부분 완료도 균형 유지)
    def interleave(lst):
        by = {}
        for r in lst:
            by.setdefault(r["object"], []).append(r)
        out = []; i = 0
        while any(by.values()):
            for k in list(by):
                if by[k]:
                    out.append(by[k].pop(0))
        return out

    for grp, lst in (("fp_check", interleave(fp)), ("fn_check", interleave(fn))):
        for r in lst:
            ds = r["dataset"]; fr = int(r["frame_id"]); obj = r["object"]; uid = r["best_uid"]
            sc = (f"sem {r['sem_top5']} / appe {r['appe11']} / hsv {r['hsv']}"
                  if r["has_candidate"] == "1" else "후보 없음")
            if grp == "fp_check":
                img = panel_fp(ds, fr, obj, uid, tdir[obj], sc)
            else:
                img = panel_fn(ds, fr, obj, uid, tdir[obj], sc,
                               rc.get((ds, fr, obj), "-"))
            if img is None:
                continue
            cid = f"{ds}_{fr:06d}_{obj}"
            rel = os.path.join("cases_fpfn", grp, f"{cid}.png")
            cv2.imwrite(os.path.join(OUT, rel), img)
            out_rows.append({"case_id": cid, "group": grp, "bag_name": ds, "frame_id": fr,
                             "class_name": obj, "current_label": r["phase1c_label"],
                             "has_candidate": r["has_candidate"],
                             "bbox_uid": uid, "root_cause": rc.get((ds, fr, obj), ""),
                             "sem": r["sem_top5"], "appe": r["appe11"], "hsv": r["hsv"],
                             "image_path": rel, "verdict": "", "notes": ""})

    path = os.path.join(OUT, "fp_fn_verification_task.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys())); w.writeheader(); w.writerows(out_rows)
    n_fp = sum(1 for r in out_rows if r["group"] == "fp_check")
    n_fn = len(out_rows) - n_fp
    print(f"=== FP/FN 검증 과제 생성 ===")
    print(f"  fp_check {n_fp}건 (A/B/C/U)")
    print(f"  fn_check {n_fn}건 (1/2/3/4/U)  — 후보없음 {sum(1 for r in out_rows if r['group']=='fn_check' and r['has_candidate']!='1')}건 포함")
    print(f"  총 {len(out_rows)}건 -> {path}")


if __name__ == "__main__":
    main()
