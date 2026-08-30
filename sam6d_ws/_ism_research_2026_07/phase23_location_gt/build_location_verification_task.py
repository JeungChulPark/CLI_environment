#!/usr/bin/env python3
"""build_location_verification_task.py — TP 위치 타당성 사람검증 과제 생성 (READ-ONLY).

문제: 사람 GT가 frame-level visibility 뿐이라 TP 판정이 "그 프레임에 객체가 있다 + 그 라벨의
박스를 accept했다"로만 결정되고 **박스 위치를 검증하지 않는다**. 실제로 choco TP 표본 8건 중
4건이 엉뚱한 갈색 택배박스였다.

해결: GT bbox를 새로 그리는 대신, accept된 박스가 대상 객체 위에 있는지 **Y/N 검증**만 받는다.
각 케이스마다 (전체 프레임+박스 / 확대 crop / 대상 객체 template 참조) 패널을 만들어
사람이 즉시 판단할 수 있게 한다.

검증 대상 그룹:
  A. phase1c_TP        : Phase 1C 가 TP로 센 셀 (클래스별 층화표본)  ← TP 신뢰도의 핵심
  B. rescue_recovered  : Phase 2.2 one_borderline_rescue 가 새로 살린 셀 (전수)
  C. rerank_recovered  : rerank_top3 가 새로 살린 셀 (전수)

산출: outputs/phase23_location_gt/
  cases/<group>/<case_id>.png      검증 패널
  location_verification_task.csv   사람이 box_on_target 열만 채우면 되는 표
  contact_sheets/<class>_*.png     클래스별 일괄 검토용
  README.md                        작성법
"""
import argparse, csv, glob, os, sys
from collections import defaultdict
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit"))
sys.path.insert(0, os.path.join(RSRCH, "phase22_training_free"))
sys.path.insert(0, REPO)
import phase21_common as p21
import rules as R
import yolo_ism_object_n as o_n

OUT = os.path.join(REPO, "outputs", "phase23_location_gt")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
POOL = os.path.join(REPO, "outputs", "phase22_training_free_gate", "csv", "candidate_pool.csv")
FONTP = "/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf"


def font(sz):
    try:
        return ImageFont.truetype(FONTP, sz)
    except Exception:
        return ImageFont.load_default()


def parse_bbox(uid):
    return tuple(int(v) for v in uid.rsplit("|", 1)[1].split("_"))


def load_pool():
    FL = {"conf", "sem_top5", "appe11", "hsv", "sim_thr", "appe_gate", "hsv_thr", "sem_all_mean",
          "sem_top1", "sem_median", "appe2", "appe9", "sem_margin_vs_2nd", "appe_margin_vs_2nd",
          "view_median", "view_std", "view_top1_minus_median"}
    IN = {"gt_visible", "conf_rank", "n_cands", "is_selected", "passS", "passA", "passH",
          "accept", "view_pass_count", "frame_id"}
    cells = defaultdict(list)
    for r in csv.DictReader(open(POOL)):
        d = dict(r)
        for k in FL: d[k] = float(d[k])
        for k in IN: d[k] = int(d[k])
        cells[(d["dataset"], d["frame_id"], d["object"])].append(d)
    return cells


def template_thumb(tdir, size=150):
    """대상 객체 reference template 2장을 가로로 붙여 반환."""
    ps = sorted(glob.glob(os.path.join(tdir, "rgb_*.png")))
    if not ps:
        return np.full((size, size * 2, 3), 40, np.uint8)
    picks = [ps[0], ps[len(ps) // 3]]
    ims = []
    for p in picks:
        im = cv2.imread(p)
        if im is None:
            im = np.full((size, size, 3), 40, np.uint8)
        ims.append(cv2.resize(im, (size, size)))
    return np.hstack(ims)


def make_panel(ds, fr, obj, cand, tdir, group):
    """전체프레임(박스) | 확대 crop | template 참조  + 설명 패널."""
    fp = os.path.join(FRAMES, ds, f"frame_{fr:06d}.png")
    bgr = cv2.imread(fp)
    if bgr is None:
        return None
    x1, y1, x2, y2 = parse_bbox(cand["uid"])
    full = bgr.copy()
    cv2.rectangle(full, (x1, y1), (x2, y2), (0, 0, 255), 3)
    full = cv2.resize(full, (520, 390))
    # 확대 crop (여유 20px)
    h, w = bgr.shape[:2]
    cx1, cy1 = max(0, x1 - 20), max(0, y1 - 20)
    cx2, cy2 = min(w, x2 + 20), min(h, y2 + 20)
    crop = bgr[cy1:cy2, cx1:cx2]
    if crop.size == 0:
        crop = bgr.copy()
    crop = cv2.resize(crop, (300, 300))
    cv2.putText(crop, "accept box", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(crop, "accept box", (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
    tpl = template_thumb(tdir, 150)
    tpl_col = np.full((300, 300, 3), 25, np.uint8)
    tpl_col[10:160, 0:300] = cv2.resize(tpl, (300, 150))
    cv2.putText(tpl_col, "TARGET template", (6, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
    cv2.putText(tpl_col, "(this is what it", (6, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 180), 1)
    cv2.putText(tpl_col, " SHOULD be)", (6, 228), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 180), 1)
    right = np.vstack([np.hstack([crop, tpl_col])])
    top = np.hstack([full, cv2.resize(right, (600, 390))])

    area = (x2 - x1) * (y2 - y1)
    lines = [(f"[{group}]  {ds}  frame {fr}", 19, (255, 220, 120)),
             (f"대상 class: {obj}", 20, (255, 255, 255)),
             ("", 5, (0, 0, 0)),
             ("질문: 빨간 박스가 위 'TARGET template' 과", 16, (200, 220, 255)),
             ("      같은 물체 위에 있습니까?", 16, (200, 220, 255)),
             ("  Y = 맞다 / N = 다른 물체다 / U = 불확실", 15, (255, 235, 140)),
             ("", 5, (0, 0, 0)),
             (f"bbox={x1},{y1},{x2},{y2}  area={area}", 14, (190, 190, 190)),
             (f"sem {cand['sem_top5']:.3f} / appe {cand['appe11']:.3f} / hsv {cand['hsv']:.3f}", 14, (190, 190, 190)),
             ("", 5, (0, 0, 0)),
             ("※ GT는 '이 프레임에 객체가 보인다'만 알려줄 뿐", 13, (230, 170, 170)),
             ("   박스 위치는 검증된 적이 없습니다.", 13, (230, 170, 170))]
    panel = Image.new("RGB", (430, top.shape[0]), (24, 24, 28))
    d = ImageDraw.Draw(panel); y = 10
    for t, s, c in lines:
        d.text((12, y), t, font=font(s), fill=c); y += s + 6
    canvas = Image.new("RGB", (top.shape[1] + 430, top.shape[0]), (0, 0, 0))
    canvas.paste(Image.fromarray(cv2.cvtColor(top, cv2.COLOR_BGR2RGB)), (0, 0))
    canvas.paste(panel, (top.shape[1], 0))
    return cv2.cvtColor(np.array(canvas), cv2.COLOR_RGB2BGR)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-class", type=int, default=25, help="phase1c_TP 클래스별 표본 수")
    a = ap.parse_args()
    for d in ("cases/phase1c_TP", "cases/rescue_recovered", "cases/rerank_recovered", "contact_sheets"):
        os.makedirs(os.path.join(OUT, d), exist_ok=True)

    gt = p21.load_gt(); cells = load_pool()
    _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}

    groups = defaultdict(list)   # group -> [(ds,fr,obj,cand)]
    for (ds, fr), vis in gt.items():
        for obj in p21.OBJECTS:
            cs = cells.get((ds, fr, obj))
            if not cs:
                continue
            p0_acc, p0_uid, _ = R.phase1c(cs)
            v = obj in vis
            if v and p0_acc:
                groups["phase1c_TP"].append((ds, fr, obj, next(c for c in cs if c["uid"] == p0_uid)))
                continue
            if v and not p0_acc:
                rc_acc, rc_uid, _ = R.one_borderline_rescue(cs)
                if rc_acc:
                    groups["rescue_recovered"].append((ds, fr, obj, next(c for c in cs if c["uid"] == rc_uid)))
                rr_acc, rr_uid, _ = R.rerank_topk(cs, 3)
                if rr_acc:
                    groups["rerank_recovered"].append((ds, fr, obj, next(c for c in cs if c["uid"] == rr_uid)))

    # phase1c_TP 층화표본 (클래스별 균등, 클래스 순환 배치 → 부분 완료도 유용)
    byc = defaultdict(list)
    for item in groups["phase1c_TP"]:
        byc[item[2]].append(item)
    sampled = []
    for o in p21.OBJECTS:
        lst = byc.get(o, [])
        step = max(1, len(lst) // a.per_class)
        sampled.append(lst[::step][:a.per_class])
    inter = []
    for i in range(max((len(s) for s in sampled), default=0)):
        for s in sampled:
            if i < len(s):
                inter.append(s[i])
    groups["phase1c_TP"] = inter

    rows = []
    for group in ("phase1c_TP", "rescue_recovered", "rerank_recovered"):
        for (ds, fr, obj, cand) in groups[group]:
            cid = f"{ds}_{fr:06d}_{obj}"
            img = make_panel(ds, fr, obj, cand, tdir[obj], group)
            if img is None:
                continue
            rel = os.path.join("cases", group, f"{cid}.png")
            cv2.imwrite(os.path.join(OUT, rel), img)
            x1, y1, x2, y2 = parse_bbox(cand["uid"])
            rows.append({"case_id": cid, "group": group, "bag_name": ds, "frame_id": fr,
                         "class_name": obj, "bbox_x1y1x2y2": f"{x1},{y1},{x2},{y2}",
                         "bbox_area": (x2 - x1) * (y2 - y1),
                         "sem": round(cand["sem_top5"], 4), "appe": round(cand["appe11"], 4),
                         "hsv": round(cand["hsv"], 4), "image_path": rel,
                         "box_on_target": "", "notes": ""})

    with open(os.path.join(OUT, "location_verification_task.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

    # 클래스별 contact sheet (일괄 검토)
    def sheet(paths, out, cols=4, tile=(430, 200)):
        if not paths: return
        rws = (len(paths) + cols - 1) // cols
        sh = Image.new("RGB", (cols * tile[0], rws * tile[1]), (15, 15, 18))
        for i, p in enumerate(paths):
            im = cv2.imread(p)
            if im is None: continue
            sh.paste(Image.fromarray(cv2.cvtColor(cv2.resize(im, tile), cv2.COLOR_BGR2RGB)),
                     ((i % cols) * tile[0], (i // cols) * tile[1]))
        sh.save(out)
    for o in p21.OBJECTS:
        ps = [os.path.join(OUT, r["image_path"]) for r in rows if r["class_name"] == o]
        sheet(ps[:24], os.path.join(OUT, "contact_sheets", f"{o}_verify_sheet.png"))

    n_by = defaultdict(int)
    for r in rows: n_by[r["group"]] += 1
    readme = f"""# 위치 검증 과제 (TP 신뢰도 확정용)

## 왜 필요한가
현재 사람 GT는 **프레임 단위 "이 객체가 보인다"** 정보만 있습니다. 그래서 TP 판정이
`GT에 객체 있음 AND 그 라벨 박스를 accept함` 으로만 이뤄지고 **박스 위치를 검증하지 않습니다.**
실제로 choco TP 표본 8건 중 4건이 엉뚱한 갈색 택배박스였습니다.
→ 지금까지 보고된 TP/FP/F1(Phase 1C·2·2.1·2.2)은 전부 **위치 무검증** 수치입니다.

## 무엇을 하면 되나
`location_verification_task.csv` 의 **`box_on_target` 열만** 채워주세요.

| 값 | 뜻 |
|---|---|
| `Y` | 빨간 박스가 대상 객체(template과 같은 물체) 위에 있다 |
| `N` | 다른 물체 위에 있다 (예: 갈색 택배박스) |
| `U` | 불확실 / 판단 불가 |

각 행의 `image_path` 패널에 **전체 프레임 + 확대 crop + 대상 template**이 함께 있어
비교만 하시면 됩니다. 클래스별 일괄 검토는 `contact_sheets/` 참고.

## 표본 구성 (총 {len(rows)}건)
| group | 건수 | 의미 |
|---|---|---|
| phase1c_TP | {n_by['phase1c_TP']} | Phase 1C 가 TP로 센 셀 (클래스별 최대 {a.per_class}) — **TP 신뢰도의 핵심** |
| rescue_recovered | {n_by['rescue_recovered']} | Phase 2.2 rescue 가 새로 살린 셀 (전수) — +28 TP 주장 검증 |
| rerank_recovered | {n_by['rerank_recovered']} | rerank 가 새로 살린 셀 (전수) |

행은 클래스가 번갈아 나오도록 배치되어 **중간까지만 채워도** 클래스 균형이 유지됩니다.

## 채운 뒤
`eval_location_verified.py` 를 실행하면 위치검증 TP(TP_loc)로
precision/recall/F1 을 재계산하고, Phase 2.2 rescue 주장을 재평가합니다.
"""
    open(os.path.join(OUT, "README.md"), "w").write(readme)
    print(f"=== 위치 검증 과제 생성 ===")
    for g in ("phase1c_TP", "rescue_recovered", "rerank_recovered"):
        print(f"  {g:20s} {n_by[g]:4d} cases")
    print(f"  총 {len(rows)}건 · 패널 이미지 {len(rows)}장 · contact sheet {len(p21.OBJECTS)}장")
    print(f"-> {OUT}/location_verification_task.csv  (box_on_target 열만 Y/N/U 로 채우세요)")
    print(f"-> {OUT}/README.md")


if __name__ == "__main__":
    main()
