#!/usr/bin/env python3
"""make_visual_cases.py — 실제 ros2 bag 입력 프레임 위에 문제/복구를 설명하는 시각화 대량 생성.

원본 이미지 무수정. gt_input 프레임 위에 candidate bbox / mask contour / score / gate 사유 /
Phase1C vs 새 방법 판정을 오버레이한다. GT는 frame-level visibility 뿐이라 GT bbox는 없음(명시).

산출: outputs/phase22_training_free_gate/figures_visual/
  single_overlay/ before_after/ candidate_panels/ root_cause/ by_class/ by_bag/ contact_sheets/ report/
  + manifests(csv) + report/visual_case_index.{md,html}
"""
import argparse, csv, glob, os, sys
from collections import defaultdict
import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rules as R
P21 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "phase21_fn_audit")
sys.path.insert(0, P21)
import phase21_common as p21
sys.path.insert(0, p21.REPO)
import yolo_ism as yi, yolo_ism_object_n as o_n
import torch

REPO = p21.REPO
OUT = os.path.join(REPO, "outputs", "phase22_training_free_gate")
VIS = os.path.join(OUT, "figures_visual")
FRAMES = os.path.join(p21.RSRCH, "gt_input", "frames")
SUBDIRS = ["single_overlay", "before_after", "candidate_panels", "root_cause",
           "by_class", "by_bag", "contact_sheets", "report"]
FONTP = "/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf"
COL = {"selected": (230, 40, 40), "alt": (40, 120, 230), "accepted": (245, 210, 30),
       "gt": (40, 200, 80), "rescued": (255, 140, 0)}


def font(sz):
    try:
        return ImageFont.truetype(FONTP, sz)
    except Exception:
        return ImageFont.load_default()


def parse_bbox(uid):
    return tuple(int(v) for v in uid.rsplit("|", 1)[1].split("_"))


def load_pool():
    from eval_training_free_methods import FLOATS, INTS
    cells = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(OUT, "csv", "candidate_pool.csv"))):
        d = dict(r)
        for k in FLOATS: d[k] = float(d[k])
        for k in INTS: d[k] = int(d[k])
        cells[(d["dataset"], d["frame_id"], d["object"])].append(d)
    return cells


def frame_path(ds, fr):
    return os.path.join(FRAMES, ds, f"frame_{int(fr):06d}.png")


def draw_panel(img_bgr, lines, width=430):
    """오른쪽에 텍스트 패널 붙이기."""
    h = img_bgr.shape[0]
    panel = Image.new("RGB", (width, h), (24, 24, 28))
    d = ImageDraw.Draw(panel); y = 12
    for txt, sz, col in lines:
        d.text((12, y), txt, font=font(sz), fill=col); y += sz + 7
    base = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
    canvas = Image.new("RGB", (base.width + width, h), (0, 0, 0))
    canvas.paste(base, (0, 0)); canvas.paste(panel, (base.width, 0))
    return cv2.cvtColor(np.array(canvas), cv2.COLOR_RGB2BGR)


def draw_box(img, bbox, color, label=None, thick=3, mask=None):
    x1, y1, x2, y2 = bbox
    if mask is not None:
        sub = mask[y1:y2, x1:x2].astype(np.uint8)
        cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            c = c + np.array([[x1, y1]]); cv2.drawContours(img, [c], -1, color[::-1], 2)
    cv2.rectangle(img, (x1, y1), (x2, y2), color[::-1], thick)
    if label:
        cv2.rectangle(img, (x1, y1 - 22), (x1 + 11 * len(label), y1), color[::-1], -1)
        cv2.putText(img, label, (x1 + 2, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


def gate_str(c):
    return f"S{'O' if c['passS'] else 'X'} A{'O' if c['passA'] else 'X'} H{'O' if c['passH'] else 'X'}"


def explain(cell_cands, sel, new_uid, new_acc, method_reason, p0_acc):
    """짧은 사람 해설 문구."""
    if p0_acc and new_acc:
        return "안정 TP (Phase1C·새방법 모두 accept)"
    if (not p0_acc) and new_acc:
        if "rerank" in method_reason:
            return f"Top-1 gate 실패 → 다른 후보가 3 gate 통과({method_reason})로 rerank 복구"
        if "rescue_borderline_A" in method_reason:
            return "appearance만 경계 미달, semantic·HSV strong → rescue 복구"
        if "rescue_borderline_S" in method_reason:
            return "semantic만 경계 미달, 나머지 strong → rescue 복구"
        return f"복구({method_reason})"
    if p0_acc and not new_acc:
        return "새 방법이 오히려 제거(회귀)"
    # both reject
    if sel is None:
        return "후보 없음(proposal miss)"
    g = "semantic" if not sel["passS"] else "appearance" if not sel["passA"] else "HSV" if not sel["passH"] else "?"
    return f"{g} gate 실패로 reject 유지 (hard FN)"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", default="one_borderline_rescue",
                    choices=["one_borderline_rescue", "P1_rerank_top3"])
    ap.add_argument("--per-cap", type=int, default=40, help="category당 저장 상한(다양성)")
    ap.add_argument("--no-mask", action="store_true")
    a = ap.parse_args()
    for s in SUBDIRS:
        os.makedirs(os.path.join(VIS, s), exist_ok=True)

    cells = load_pool(); gt = p21.load_gt()
    rc = {}
    p = os.path.join(REPO, "outputs", "phase21_fn_root_cause_audit", "csv", "fn_root_cause_per_object.csv")
    for r in csv.DictReader(open(p)):
        rc[(r["dataset"], int(r["frame_id"]), r["object"])] = r["primary_root_cause"]

    method_fn = R.one_borderline_rescue if a.method == "one_borderline_rescue" else (lambda c: R.rerank_topk(c, 3))

    # segmentor (mask contour)
    seg = None
    if not a.no_mask and torch.cuda.is_available():
        defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
        seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), "cuda:0")

    # 셀 분류
    cats = defaultdict(list)   # category -> [(ds,fr,obj)]
    decisions = {}
    for (ds, fr), vis in gt.items():
        for o in p21.OBJECTS:
            cs = cells.get((ds, fr, o))
            p0_acc, p0_uid, p0_r = R.phase1c(cs) if cs else (False, None, "no_candidate")
            n_acc, n_uid, n_r = method_fn(cs) if cs else (False, None, "no_candidate")
            v = o in vis
            decisions[(ds, fr, o)] = (p0_acc, p0_uid, p0_r, n_acc, n_uid, n_r, v, cs)
            if v and (not p0_acc) and n_acc:
                cats["recovered_tp" if "rescue" in n_r or "rerank" in n_r else "recovered_tp"].append((ds, fr, o))
                if rc.get((ds, fr, o)) == "RC11_selection" or "rerank" in n_r:
                    cats["selection_fix"].append((ds, fr, o))
            elif (not v) and (not p0_acc) and n_acc:
                cats["new_fp"].append((ds, fr, o))
            elif v and p0_acc and n_acc:
                cats["stable_tp"].append((ds, fr, o))
            elif v and (not n_acc):
                sub = rc.get((ds, fr, o), "hard")
                cats["hard_fn"].append((ds, fr, o))
                cats[f"rc_{sub}"].append((ds, fr, o))

    # 다양성 균형 샘플러: class/bag 고르게
    def balanced(keys, cap):
        by = defaultdict(list)
        for k in keys:
            by[(k[2], k[0])].append(k)   # (class,bag)
        out = []; i = 0
        pools = list(by.values())
        while len(out) < cap and any(pools):
            for pl in pools:
                if pl:
                    out.append(pl.pop())
                    if len(out) >= cap: break
        return out

    manifest = []
    seg_cache = {}

    def get_masks(ds, fr, boxes):
        if seg is None or not boxes:
            return {b: None for b in boxes}
        key = (ds, fr)
        if key not in seg_cache:
            seg_cache[key] = {}
        bgr = cv2.imread(frame_path(ds, fr))
        need = [b for b in boxes if b not in seg_cache[key]]
        if need and bgr is not None:
            ms = yi.segment_boxes(seg, bgr, [list(b) for b in need], "cuda:0")
            for b, m in zip(need, ms):
                seg_cache[key][b] = m
        return {b: seg_cache[key].get(b) for b in boxes}

    def make_single(ds, fr, o, cat):
        p0_acc, p0_uid, p0_r, n_acc, n_uid, n_r, v, cs = decisions[(ds, fr, o)]
        bgr = cv2.imread(frame_path(ds, fr))
        if bgr is None or not cs:
            return None
        img = bgr.copy(); sel = R._sel(cs)
        drawn = [parse_bbox(c["uid"]) for c in cs]
        masks = get_masks(ds, fr, drawn)
        for c in sorted(cs, key=lambda x: x["conf_rank"], reverse=True):
            bb = parse_bbox(c["uid"])
            role = "selected" if c["is_selected"] else "alt"
            if c["uid"] == n_uid and n_acc and not p0_acc:
                role = "rescued"
            draw_box(img, bb, COL[role], f"r{c['conf_rank']} {gate_str(c)}", mask=masks.get(bb))
        lines = [(f"[{cat}]  {ds}", 20, (255, 220, 120)), (f"frame {fr} · class {o}", 18, (230, 230, 230)),
                 (f"GT: {'visible' if v else 'not-visible'} (GT bbox N/A: frame-level GT)", 15, (150, 220, 150)),
                 ("", 6, (0, 0, 0)), ("후보 (conf rank):", 16, (200, 200, 255))]
        for c in sorted(cs, key=lambda x: x["conf_rank"]):
            mk = "◀선택" if c["is_selected"] else ""
            lines.append((f" r{c['conf_rank']} sem{c['sem_top5']:.2f} appe{c['appe11']:.2f} "
                          f"hsv{c['hsv']:.2f} [{gate_str(c)}] {mk}", 14,
                          (255, 235, 120) if c["uid"] == n_uid and n_acc else (215, 215, 215)))
        lines += [("", 6, (0, 0, 0)),
                  (f"threshold: sem≥{sel['sim_thr']:.2f} appe≥{sel['appe_gate']:.2f} hsv≥{sel['hsv_thr']:.3f}", 14, (180, 180, 180)),
                  (f"root-cause: {rc.get((ds,fr,o),'-')}", 15, (230, 180, 180)),
                  (f"Phase1C: {'ACCEPT' if p0_acc else 'reject'}", 17, (120, 230, 120) if p0_acc else (230, 120, 120)),
                  (f"새방법: {'ACCEPT' if n_acc else 'reject'}  ({n_r})", 17, (120, 230, 120) if n_acc else (230, 120, 120)),
                  ("", 6, (0, 0, 0)), ("해설:", 15, (200, 200, 255))]
        expl = explain(cs, sel, n_uid, n_acc, n_r, p0_acc)
        for i in range(0, len(expl), 30):
            lines.append((" " + expl[i:i+30], 15, (240, 240, 200)))
        out_img = draw_panel(img, lines)
        fn = f"{ds}_{int(fr):06d}_{o}_{cat}_overlay.png"
        cv2.imwrite(os.path.join(VIS, "single_overlay", fn), out_img)
        # by_class / by_bag copy(경량: 동일 이미지 재저장)
        cv2.imwrite(os.path.join(VIS, "by_class", f"class_{o}__{fn}"), out_img)
        cv2.imwrite(os.path.join(VIS, "by_bag", f"bag_{ds}__{fn}"), out_img)
        cv2.imwrite(os.path.join(VIS, "root_cause", f"{rc.get((ds,fr,o),'na')}__{fn}"), out_img)
        return fn, expl

    def make_before_after(ds, fr, o, cat):
        p0_acc, p0_uid, p0_r, n_acc, n_uid, n_r, v, cs = decisions[(ds, fr, o)]
        bgr = cv2.imread(frame_path(ds, fr))
        if bgr is None or not cs:
            return None
        drawn = [parse_bbox(c["uid"]) for c in cs]; masks = get_masks(ds, fr, drawn)
        orig = bgr.copy()
        a_img = bgr.copy()  # phase1c
        if p0_uid:
            bb = parse_bbox(p0_uid); draw_box(a_img, bb, COL["accepted"] if p0_acc else COL["selected"],
                                              "ACCEPT" if p0_acc else "reject", mask=masks.get(bb))
        b_img = bgr.copy()  # new
        if n_uid:
            bb = parse_bbox(n_uid); draw_box(b_img, bb, COL["accepted"] if n_acc else COL["selected"],
                                             "ACCEPT" if n_acc else "reject", mask=masks.get(bb))
        for im, tag in ((orig, "input"), (a_img, "Phase1C"), (b_img, "new-method")):
            cv2.putText(im, tag, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
            cv2.putText(im, tag, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 1)
        combo = np.hstack([orig, a_img, b_img])
        expl = explain(cs, R._sel(cs), n_uid, n_acc, n_r, p0_acc)
        lines = [(f"{ds} frame {fr} · {o} · [{cat}]", 18, (255, 220, 120)),
                 (f"Phase1C {'ACCEPT' if p0_acc else 'reject'} → 새방법 {'ACCEPT' if n_acc else 'reject'}", 17,
                  (120, 230, 120)), (f"reason: {n_r}", 15, (220, 220, 180))]
        for i in range(0, len(expl), 44):
            lines.append((" " + expl[i:i+44], 15, (240, 240, 200)))
        out_img = draw_panel(combo, lines, width=480)
        fn = f"{ds}_{int(fr):06d}_{o}_{cat}_beforeafter.png"
        cv2.imwrite(os.path.join(VIS, "before_after", fn), out_img)
        return fn

    def make_candidate_panel(ds, fr, o):
        p0_acc, p0_uid, p0_r, n_acc, n_uid, n_r, v, cs = decisions[(ds, fr, o)]
        bgr = cv2.imread(frame_path(ds, fr))
        if bgr is None or not cs:
            return None
        tiles = []
        for c in sorted(cs, key=lambda x: x["conf_rank"]):
            x1, y1, x2, y2 = parse_bbox(c["uid"])
            crop = bgr[max(0, y1-10):y2+10, max(0, x1-10):x2+10]
            if crop.size == 0:
                continue
            crop = cv2.resize(crop, (160, 160))
            pil = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
            cap = Image.new("RGB", (160, 92), (24, 24, 28)); d = ImageDraw.Draw(cap)
            picked = (c["uid"] == n_uid and n_acc)
            d.text((4, 2), f"rank {c['conf_rank']}{' ◀선택' if c['is_selected'] else ''}", font=font(13),
                   fill=(255, 235, 120) if picked else (220, 220, 220))
            d.text((4, 20), f"sem {c['sem_top5']:.2f}", font=font(13), fill=(120, 230, 120) if c["passS"] else (230, 120, 120))
            d.text((4, 38), f"appe {c['appe11']:.2f}", font=font(13), fill=(120, 230, 120) if c["passA"] else (230, 120, 120))
            d.text((4, 56), f"hsv {c['hsv']:.2f}", font=font(13), fill=(120, 230, 120) if c["passH"] else (230, 120, 120))
            d.text((4, 74), ("accept" if c["accept"] else "reject"), font=font(13),
                   fill=(255, 235, 120) if picked else (180, 180, 180))
            tile = Image.new("RGB", (160, 252), (0, 0, 0)); tile.paste(pil, (0, 0)); tile.paste(cap, (0, 160))
            tiles.append(np.array(tile))
        if not tiles:
            return None
        row = cv2.cvtColor(np.hstack(tiles), cv2.COLOR_RGB2BGR)
        lines = [(f"{ds} f{fr} · {o} candidate panel", 17, (255, 220, 120)),
                 (f"Phase1C 선택=best-by-sem(◀), 판정 {'ACCEPT' if p0_acc else 'reject'}", 15, (200, 220, 200)),
                 (f"새방법 최종: {'ACCEPT '+ (n_uid.rsplit('|',1)[1] if n_uid else '') if n_acc else 'reject'} ({n_r})", 15, (200, 220, 200))]
        out_img = draw_panel(row, lines, width=430)
        fn = f"{ds}_{int(fr):06d}_{o}_candpanel.png"
        cv2.imwrite(os.path.join(VIS, "candidate_panels", fn), out_img)
        return fn

    # ---- 생성 ----
    plan = {"recovered_tp": a.per_cap, "selection_fix": a.per_cap, "new_fp": a.per_cap,
            "stable_tp": min(a.per_cap, 30), "hard_fn": a.per_cap}
    counts = defaultdict(int)
    for cat, cap in plan.items():
        for (ds, fr, o) in balanced(list(dict.fromkeys(cats.get(cat, []))), cap):
            r = make_single(ds, fr, o, cat)
            if not r:
                continue
            fn, expl = r; counts[cat] += 1
            ba = cp = ""
            if cat in ("recovered_tp", "selection_fix", "new_fp"):
                ba = make_before_after(ds, fr, o, cat) or ""
            if cat in ("recovered_tp", "selection_fix"):
                cp = make_candidate_panel(ds, fr, o) or ""
            p0_acc, _, _, n_acc, _, _, v, _ = decisions[(ds, fr, o)]
            manifest.append({"case_id": f"{ds}_{fr}_{o}_{cat}", "bag_name": ds, "frame_id": fr,
                             "class_name": o, "gt_label": "visible" if v else "not_visible",
                             "old_decision": "accept" if p0_acc else "reject",
                             "new_decision": "accept" if n_acc else "reject",
                             "root_cause": rc.get((ds, fr, o), ""), "recovery_method": a.method,
                             "category": cat, "image_path_single_overlay": os.path.join("single_overlay", fn),
                             "image_path_before_after": os.path.join("before_after", ba) if ba else "",
                             "image_path_candidate_panel": os.path.join("candidate_panels", cp) if cp else "",
                             "notes": expl})

    # ---- contact sheets (category & class) ----
    def contact(imgs, path, cols=5, tile=(320, 240)):
        if not imgs:
            return
        rows = (len(imgs) + cols - 1) // cols
        sheet = Image.new("RGB", (cols * tile[0], rows * tile[1]), (15, 15, 18))
        for i, ip in enumerate(imgs):
            im = cv2.imread(ip)
            if im is None:
                continue
            im = cv2.resize(im, tile); pil = Image.fromarray(cv2.cvtColor(im, cv2.COLOR_BGR2RGB))
            sheet.paste(pil, ((i % cols) * tile[0], (i // cols) * tile[1]))
        sheet.save(path)

    for cat in plan:
        imgs = sorted(glob.glob(os.path.join(VIS, "single_overlay", f"*_{cat}_overlay.png")))[:25]
        contact(imgs, os.path.join(VIS, "contact_sheets", f"{cat}_contact_sheet.png"))
    for o in p21.OBJECTS:
        imgs = sorted(glob.glob(os.path.join(VIS, "by_class", f"class_{o}__*")))[:25]
        contact(imgs, os.path.join(VIS, "contact_sheets", f"class_{o}_sheet.png"))

    # ---- manifests + index ----
    os.makedirs(os.path.join(VIS, "report"), exist_ok=True)
    with open(os.path.join(VIS, "visual_cases_manifest.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(manifest[0].keys())); w.writeheader(); w.writerows(manifest)
    for name, catset in [("recovered_cases", {"recovered_tp", "selection_fix"}),
                         ("false_positive_cases", {"new_fp"}), ("hard_fn_cases", {"hard_fn"}),
                         ("selection_fix_cases", {"selection_fix"})]:
        rows = [m for m in manifest if m["category"] in catset]
        with open(os.path.join(VIS, f"{name}.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(manifest[0].keys())); w.writeheader(); w.writerows(rows)
    # class summary counts
    cls_counts = defaultdict(lambda: defaultdict(int))
    for m in manifest:
        cls_counts[m["class_name"]][m["category"]] += 1
    with open(os.path.join(VIS, "class_summary_visual_counts.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["class"] + list(plan.keys()))
        for o in p21.OBJECTS:
            w.writerow([o] + [cls_counts[o][c] for c in plan])

    # markdown + html index
    md = ["# Phase 2.2 Visual Case Index", "",
          f"method = `{a.method}` · 총 {len(manifest)} cases · GT bbox 없음(frame-level visibility)", "",
          "| case | bag | frame | class | old→new | root_cause | overlay | before/after |",
          "|---|---|---|---|---|---|---|---|"]
    for m in manifest:
        md.append(f"| {m['category']} | {m['bag_name']} | {m['frame_id']} | {m['class_name']} | "
                  f"{m['old_decision']}→{m['new_decision']} | {m['root_cause']} | "
                  f"![](../{m['image_path_single_overlay']}) | "
                  f"{m['image_path_before_after']} |")
    open(os.path.join(VIS, "report", "visual_case_index.md"), "w").write("\n".join(md))
    html = ["<html><head><meta charset='utf-8'><style>img{max-width:520px;border:1px solid #ccc}"
            "td{vertical-align:top;font-size:13px;padding:4px}</style></head><body>",
            f"<h2>Phase 2.2 Visual Cases — method {a.method} — {len(manifest)} cases</h2><table>"]
    for m in manifest:
        html.append(f"<tr><td>{m['category']}<br>{m['bag_name']}<br>f{m['frame_id']} {m['class_name']}"
                    f"<br>{m['old_decision']}→{m['new_decision']}<br><b>{m['root_cause']}</b><br>{m['notes']}</td>"
                    f"<td><img src='../{m['image_path_single_overlay']}'></td></tr>")
    html.append("</table></body></html>")
    open(os.path.join(VIS, "report", "visual_case_index.html"), "w").write("\n".join(html))

    total = len(glob.glob(os.path.join(VIS, "**", "*.png"), recursive=True))
    print(f"=== 시각화 생성 완료 (method={a.method}) ===")
    print(f"총 PNG: {total}")
    for c in plan:
        print(f"  {c:16s} {counts[c]}")
    print(f"manifest: {len(manifest)} rows -> {VIS}/visual_cases_manifest.csv")
    print(f"index: {VIS}/report/visual_case_index.{{md,html}}")


if __name__ == "__main__":
    main()
