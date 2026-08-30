#!/usr/bin/env python3
"""phase21_common.py — Phase 2.1 FN 원인분석 공용 (READ-ONLY).

Phase 1C 정본(cur pairs + cur/{ds}_hsv.npz 슬라이스 HSV, eval_phase1c 경로)으로
935 cell의 후보·gate flag를 재현한다. TP622/FP86/FN313 정확 일치가 계약.
"""
import csv, os, sys
from collections import defaultdict
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import ism_hsv, yolo_ism_object_n as o_n   # noqa: E402

GTP = os.path.join(RSRCH, "gt_input", "user_visibility_gt.csv")
CUR = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
OUT = os.path.join(REPO, "outputs", "phase21_fn_root_cause_audit")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
T = 0.1214; TOPK = 3
MASKED = slice(0, 224); HS = slice(96, 224)


def load_gt():
    gt = {}
    for r in csv.DictReader(open(GTP, encoding="utf-8")):
        if r["user_reviewed"] == "yes":
            gt[(r["dataset_name"], int(r["frame_id"]))] = set(
                t.strip() for t in r["visible_objects"].split(";") if t.strip() and t.strip() != "none")
    return gt


def build_protos():
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    tdir = {o["name"]: o["template_dir"] for o in objs}
    PROTO = {}
    for o in objs:
        PROTO[o["name"]] = None if o["name"] == "Dinosaur" else ism_hsv.load_cache(
            os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz"), o["template_dir"])[0]
    PROTO["Dinosaur"] = ism_hsv.build_reference(tdir["Dinosaur"], 9)[0]
    return PROTO


def hsv_sim(hist_row, obj, PROTO):
    if hist_row is None:
        return 1.0
    return float(ism_hsv.similarity(hist_row, PROTO[obj]))


def build_cells(gt, PROTO):
    """(ds,frame,object) -> dict(cands=[...], best=.., accept=bool, gate flags).

    cands: 각 routed 후보의 gate flag. best=선택(best-by-sem_top5).
    """
    cells = {}
    for ds in DATASETS:
        z = np.load(os.path.join(CUR, f"{ds}_hsv.npz"))
        hq = {str(u): h[MASKED][HS].astype(np.float64) for u, h in zip(z["uid"], z["hist"])}
        rows = defaultdict(list)
        for r in csv.DictReader(open(os.path.join(CUR, f"{ds}_pairs.csv"))):
            if (ds, int(r["frame_id"])) not in gt:
                continue
            rows[(ds, int(r["frame_id"]), r["object"])].append(r)
        for k, cs in rows.items():
            obj = k[2]
            routed = [c for c in cs if int(c["routed"]) == 1 and float(c["yolo_conf"]) >= 0.02]
            routed.sort(key=lambda c: -float(c["yolo_conf"]))
            routed = routed[:TOPK]
            if not routed:
                cells[k] = dict(cands=[], best=None, accept=False,
                                passS=False, passA=False, passH=False)
                continue
            cand_flags = []
            for c in routed:
                sem = float(c["sem_top5"]); appe = float(c["appe11_clstop1"])
                simthr = float(c["sim_thr"]); appegate = float(c["appe_gate"])
                hv = hsv_sim(hq.get(c["uid"]), obj, PROTO)
                pS = sem >= simthr; pA = appe >= appegate; pH = hv >= T
                cand_flags.append(dict(uid=c["uid"], conf=float(c["yolo_conf"]), sem=sem, appe=appe,
                                       hsv=hv, simthr=simthr, appegate=appegate,
                                       passS=pS, passA=pA, passH=pH, accept=(pS and pA and pH)))
            best = max(cand_flags, key=lambda c: c["sem"])   # Phase1C selection
            cells[k] = dict(cands=cand_flags, best=best, accept=best["accept"],
                            passS=best["passS"], passA=best["passA"], passH=best["passH"])
    return cells


def grid_from_cells(gt, cells):
    TP = FP = FN = TN = 0
    for (ds, f), vis in gt.items():
        for o in OBJECTS:
            c = cells.get((ds, f, o)); acc = bool(c and c["accept"]); v = o in vis
            if v and acc: TP += 1
            elif acc: FP += 1
            elif v: FN += 1
            else: TN += 1
    return dict(TP=TP, FP=FP, FN=FN, TN=TN)


def iou(a, b):
    """(x1,y1,x2,y2) IoU."""
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def gt_coverage(pred, gtbox):
    """intersection / GT area (작은 객체용)."""
    ix1, iy1 = max(pred[0], gtbox[0]), max(pred[1], gtbox[1])
    ix2, iy2 = min(pred[2], gtbox[2]), min(pred[3], gtbox[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    ga = (gtbox[2] - gtbox[0]) * (gtbox[3] - gtbox[1])
    return inter / ga if ga > 0 else 0.0


def proposal_purity(pred, gtbox):
    """intersection / proposal area."""
    ix1, iy1 = max(pred[0], gtbox[0]), max(pred[1], gtbox[1])
    ix2, iy2 = min(pred[2], gtbox[2]), min(pred[3], gtbox[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    pa = (pred[2] - pred[0]) * (pred[3] - pred[1])
    return inter / pa if pa > 0 else 0.0


def greedy_match(preds, gts, thr=0.5):
    """IoU 기준 one-to-one greedy matching. preds/gts=[(box,conf)|box].
    반환 matched=[(pi,gi,iou)]. tie-break: conf 내림차순(있으면), 그다음 IoU.
    """
    def box(x):
        return x[0] if isinstance(x, (list, tuple)) and len(x) == 2 and not isinstance(x[0], (int, float)) else x
    order = sorted(range(len(preds)),
                   key=lambda i: -(preds[i][1] if isinstance(preds[i], tuple) and len(preds[i]) == 2 else 0.0))
    used = set(); matched = []
    for pi in order:
        pb = box(preds[pi]); best = (-1, 0.0)
        for gi, g in enumerate(gts):
            if gi in used:
                continue
            v = iou(pb, box(g) if isinstance(g, tuple) else g)
            if v > best[1]:
                best = (gi, v)
        if best[0] >= 0 and best[1] >= thr:
            used.add(best[0]); matched.append((pi, best[0], best[1]))
    return matched


def fn_primary_cause(cell):
    """배타적 결정트리. cell = build_cells 값. 반환 (primary, secondary_list).
    RC-1 no_candidate / RC-8 semantic / RC-9 appearance / RC-10 hsv / RC-11 selection.
    """
    if cell["best"] is None:
        return "RC1_no_candidate", []
    # RC-11: 선택된 best는 실패했지만 다른 routed 후보가 전 gate 통과
    if not cell["accept"]:
        if any(c["accept"] for c in cell["cands"]):
            return "RC11_selection", []
        b = cell["best"]; sec = []
        if not b["passS"]:
            prim = "RC8_semantic"
        elif not b["passA"]:
            prim = "RC9_appearance"
        elif not b["passH"]:
            prim = "RC10_hsv"
        else:
            prim = "RC12_unknown"
        # secondary: 동시에 실패한 다른 gate
        for g, name in [("passS", "semantic"), ("passA", "appearance"), ("passH", "hsv")]:
            if not b[g] and name not in prim:
                sec.append(name)
        return prim, sec
    return "TP", []
