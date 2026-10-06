#!/usr/bin/env python3
"""Real intermediate outputs of the zero-shot 6D pipeline for ONE frame (paper figure).

Read-only reuse of the deployed code: Sam6DCore (sam6d_realtime/realtime/sam6d_core.py),
yolo_ism / yolo_ism_object_n helpers and ism_hsv. The ISM stage is replayed step by step
with the same functions and the same order as yolo_ism_object_n.assign_frame_relative so
every candidate's gate scores can be recorded; the decision is cross-checked against the
production recognize_frame_auto() on the same YOLO proposals. PEM runs through
Sam6DCore.process() with the Explorer-v2 diagnostic switched on, which exposes the 300
coarse hypotheses without changing the production selection.

    /home/jucpark/anaconda3/envs/sam6d/bin/python run_real_stage.py [--frame 1808]
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml

ROOT = Path("/home/jucpark/DeepLearning/CLI_environment")
SR = ROOT / "sam6d_realtime"
OUT = Path(__file__).resolve().parent
RUN = ROOT / "objpose/output/live_260915_eightcircle_orbslam3"
DATASET = Path("/home/jucpark/DeepLearning/Dataset/260915_eightcircle/SAM")

sys.path.insert(0, str(ROOT / "objpose/pc"))
from conv_session import ConvSession  # noqa: E402
sys.path.remove(str(ROOT / "objpose/pc"))
sys.path.insert(0, str(SR / "realtime"))
sys.path.insert(0, str(SR))
os.chdir(SR)
import yolo_ism as yi  # noqa: E402
import yolo_ism_object_n as o_n  # noqa: E402
import ism_hsv  # noqa: E402
from sam6d_core import Sam6DCore  # noqa: E402

# live-viewer drawing constants (objpose/pc/hub.py, copied to avoid importing the hub)
PALETTE = {"milk": (240, 240, 240), "choco_hazelnut_high": (60, 60, 200), "Febreze_high": (230, 170, 60),
           "Mugcup_high": (180, 110, 240), "saffron": (80, 200, 240), "Sauce_high": (40, 140, 255),
           "Sikhye_high": (60, 220, 230), "Bear": (60, 120, 190), "Dinosaur": (90, 210, 110)}
BOX_EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
CAND_COLORS = [(66, 135, 245), (245, 130, 48), (60, 180, 75), (230, 25, 75), (145, 30, 180),
               (70, 240, 240), (240, 50, 230), (210, 245, 60), (250, 190, 212), (0, 128, 128),
               (220, 190, 255), (170, 110, 40), (255, 250, 200), (128, 0, 0), (170, 255, 195),
               (128, 128, 0), (255, 215, 180), (0, 0, 128), (128, 128, 128), (255, 225, 25)]


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def r5(v):
    return None if v is None else round(float(v), 5)


# ----------------------------------------------------------------------------- drawing
def corners_from(mn, mx):
    return np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])


def project(P, K):
    z = np.clip(P[:, 2], 1e-6, None)
    return np.column_stack([K[0, 0] * P[:, 0] / z + K[0, 2], K[1, 1] * P[:, 1] / z + K[1, 2]])


def draw_pose(img, K, R, t_m, mn_m, mx_m, col, thick=2, axes=True):
    corners = corners_from(mn_m, mx_m)
    L = 0.6 * float(np.max(mx_m - mn_m))
    pts = np.vstack([corners, [[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]]])
    cam = pts @ np.asarray(R).T + np.asarray(t_m)
    uv = project(cam, K)
    p = [tuple(int(round(v)) for v in q) for q in uv]
    rect = (0, 0, img.shape[1], img.shape[0])
    def seg(a, b, c, w):
        ok, q1, q2 = cv2.clipLine(rect, p[a], p[b])
        if ok:
            cv2.line(img, q1, q2, c, w, cv2.LINE_AA)
    for a, b in BOX_EDGES:
        seg(a, b, col, thick)
    if axes:
        for k, c in ((9, (0, 0, 255)), (10, (0, 200, 0)), (11, (255, 80, 0))):
            seg(8, k, c, 2)
    return uv


def label(img, text, org, col, scale=0.38, bg=(20, 20, 20)):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = int(org[0]), int(org[1])
    x = max(0, min(x, img.shape[1] - tw - 4)); y = max(th + 3, min(y, img.shape[0] - 3))
    cv2.rectangle(img, (x - 2, y - th - 3), (x + tw + 2, y + 3), bg, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, 1, cv2.LINE_AA)


def tight(mask, pad, shape):
    ys, xs = np.where(mask)
    h, w = shape[:2]
    return (max(0, xs.min() - pad), max(0, ys.min() - pad), min(w, xs.max() + 1 + pad), min(h, ys.max() + 1 + pad))


# ----------------------------------------------------------------------------- ISM replay
def ism_replay(core, bgr, timing=None):
    """yolo -> DINOv2 -> MobileSAM -> gates, same calls/order as Sam6DCore.process +
    assign_frame_relative, with per-candidate bookkeeping."""
    T = {}
    sync(); t0 = time.perf_counter()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = bgr.shape[:2]
    with o_n.multi_label_nms(core.multi_label):
        res = core.yolo.predict(bgr, conf=core.min_score, imgsz=core.imgsz, verbose=False, device=core.device)
    sync(); t1 = time.perf_counter(); T["yolo"] = 1e3 * (t1 - t0)
    pb = {i: [] for i in range(len(core.unique_prompts))}
    if len(res) and res[0].boxes is not None and len(res[0].boxes) > 0:
        b = res[0].boxes
        for j in range(len(b)):
            xy = b.xyxy[j].tolist()
            x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
            x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
            ci = int(b.cls[j]) if b.cls is not None else 0
            if ci in pb:
                pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
    for k in pb:
        pb[k].sort(key=lambda t: t[1], reverse=True)

    objs = [o for g in core.groups.values() for o in g]
    by_name = {o["name"]: o for o in objs}
    tau = o_n._color_tiebreak_tau(objs[0])
    norm_full = yi.normalize_rgb(rgb)
    # --- box dedupe exactly as assign_frame_relative (IoU > 0.95 merge, max conf)
    uniq = []
    for pi, lst in pb.items():
        for box, conf in lst:
            for u in uniq:
                if o_n._iou_xyxy(u["box"], box) > 0.95:
                    u["conf"] = max(u["conf"], conf)
                    u["prompts"].append({"prompt": core.unique_prompts[pi], "score": r5(conf)})
                    break
            else:
                uniq.append({"box": list(box), "conf": float(conf),
                             "prompts": [{"prompt": core.unique_prompts[pi], "score": r5(conf)}]})
    thr = min(float(o.get("score_threshold", 0.0)) for o in objs)
    uniq = [u for u in uniq if u["conf"] >= thr]
    crops, keep = [], []
    for u in uniq:
        c = yi.crop_resize_pad(norm_full, u["box"])
        if c is not None:
            crops.append(c); keep.append(u)
    need = sorted({b for o in objs for b in o["_need_blocks"]})
    sync(); t2 = time.perf_counter()
    cls_all, patch_all = o_n.dinov2_blocks_forward(core.model, crops, core.device, need)
    sync(); t3 = time.perf_counter(); T["dinov2"] = 1e3 * (t3 - t2)
    masks = yi.segment_boxes(core.segmentor, bgr, [u["box"] for u in keep], core.device)
    sync(); t4 = time.perf_counter(); T["mobilesam"] = 1e3 * (t4 - t3)

    cands, best = [], {}
    for i, u in enumerate(keep):
        sems = {o["name"]: float(yi.semantic_score(cls_all[i], o["tcls"], o["match_topk"])) for o in objs}
        top = max(sems, key=sems.get)
        top_sem_argmax = top
        mask = masks[i]
        hsv = {}
        if mask is not None:
            hsv = {o["name"]: float(ism_hsv.shadow_score(bgr, u["box"], mask, o.get("_hsv_proto"))) for o in objs}
        tie = []
        if tau > 0.0 and hsv:
            tie = [k for k in sems if core.tsim[top].get(k, 0.0) >= tau]
            if len(tie) > 1:
                top = max(tie, key=lambda k: hsv[k])
        o = by_name[top]
        qb = {b: patch_all[b][i].cpu() for b in o["_need_blocks"]}
        bt = int(torch.argmax(o["tcls"] @ cls_all[i]))
        m_appe = float(o_n.masked_appe_blocks(qb, mask, u["box"], core.pool, o, bt)) if mask is not None else 0.0
        c = {"id": i, "box": u["box"], "yolo_score": r5(u["conf"]), "yolo_prompts": u["prompts"],
             "semantic_all": {k: r5(v) for k, v in sorted(sems.items(), key=lambda kv: -kv[1])},
             "semantic_argmax": top_sem_argmax,
             "color_tiebreak_group": tie if len(tie) > 1 else [],
             "owner": top, "semantic": r5(sems[top]),
             "appearance": r5(m_appe), "color_hsv": r5(hsv.get(top)) if hsv else None,
             "mask_px": int(mask.sum()) if mask is not None else 0, "gates": [],
             "rejected_by": None}
        st = c["gates"]
        sem_ok = sems[top] >= o["similarity_threshold"]
        st.append({"gate": "1_semantic", "score": r5(sems[top]), "thr": o["similarity_threshold"], "pass": bool(sem_ok), "evaluated": True})
        ev = sem_ok
        app_ok = m_appe >= o_n._appe_gate_of(o)
        st.append({"gate": "2_appearance", "score": r5(m_appe), "thr": o_n._appe_gate_of(o), "pass": bool(app_ok), "evaluated": bool(ev)})
        ev = ev and app_ok
        hsv_ok = (not hsv) or hsv[top] >= float(o.get("hsv_gate_threshold", 0.0))
        st.append({"gate": "3_color_hsv", "score": r5(hsv.get(top)) if hsv else None, "thr": float(o.get("hsv_gate_threshold", 0.0)), "pass": bool(hsv_ok), "evaluated": bool(ev)})
        ev = ev and hsv_ok
        if not sem_ok:
            c["rejected_by"] = "1_semantic"
        elif not app_ok:
            c["rejected_by"] = "2_appearance"
        elif not hsv_ok:
            c["rejected_by"] = "3_color_hsv"
        slot = None
        if ev:
            if top in best and m_appe <= best[top]["appe"]:
                slot = "lost_slot(lower_appe_same_object)"
                c["rejected_by"] = "4_exclusive_assignment"
            else:
                if top in best:
                    best[top]["cand"]["exclusive"] = "lost_slot(lower_appe_same_object)"
                    best[top]["cand"]["rejected_by"] = "4_exclusive_assignment"
                best[top] = {"appe": m_appe, "cand": c}
                slot = "slot_winner"
        c["exclusive"] = slot
        c["_mask"] = mask
        cands.append(c)
    sync(); t5 = time.perf_counter()
    # production decision on the same proposals (includes cross-object NMS)
    prod = o_n.recognize_frame_auto(core.groups, pb, bgr, rgb, norm_full, core.model, core.device,
                                    core.segmentor, core.pool, core.tsim)
    sync(); t6 = time.perf_counter()
    T["gates_replay_incl_hsv"] = 1e3 * (t5 - t4)
    T["production_ism_total"] = 1e3 * (t6 - t5)
    for c in cands:
        if c["exclusive"] == "slot_winner":
            r = prod[c["owner"]]
            if r.get("accepted") and list(r["box"]) == c["box"]:
                c["cross_object_nms"] = "kept"; c["final_accept"] = True
            else:
                c["cross_object_nms"] = r.get("decision") + (f" (to {r.get('cross_object_nms_loser_to')})" if r.get("cross_object_nms_loser_to") else "")
                c["final_accept"] = False; c["rejected_by"] = "4_cross_object_nms"
        else:
            c["cross_object_nms"] = None; c["final_accept"] = False
    if timing is not None:
        timing.append(T)
    return pb, cands, prod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", type=int, default=1808)
    ap.add_argument("--reps", type=int, default=10)
    ap.add_argument("--seed", type=int, default=-1, help="-1 = first seed that keeps all ISM objects")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--pem_object", default="")
    a = ap.parse_args()
    run_cfg = yaml.safe_load(open(RUN / "sam6d_config.yaml"))
    rt = run_cfg["runtime"]
    sess = ConvSession(DATASET, offset_ns=2_000_000)
    fr = sess.read(a.frame, with_depth=True)
    bgr, depth, K = fr.color_bgr, fr.depth_raw, sess.K
    print(f"[frame] index {a.frame} t_ns {fr.t_ns} {bgr.shape} depth {depth.dtype}")
    cv2.imwrite(str(OUT / "rgb.png"), bgr)
    d = depth.astype(np.float32)
    lo, hi = 300.0, 3000.0
    dn = np.clip((d - lo) / (hi - lo), 0, 1)
    dc = cv2.applyColorMap((255 * (1 - dn)).astype(np.uint8), cv2.COLORMAP_TURBO)
    dc[d <= 0] = 0
    cv2.imwrite(str(OUT / "depth.png"), dc)

    diag = {"enabled": True, "explorer_v2": {"enabled": True, "capture_profile": "exhaustive_visualization"}}
    core = Sam6DCore(run_cfg["ism"]["config"], run_cfg["ism"]["objects"], rt["device"], rt["det_score_thresh"],
                     appe_rerank=rt.get("appe_rerank"), verify=rt["verify"], pem_diagnostic=diag)
    diag_appe = core.pem.cfg.appe_rerank
    prod_appe = {k: v for k, v in dict(diag_appe).items() if k != "diagnostic"}

    def production_mode(on):
        if on:
            core.pem.cfg.appe_rerank = prod_appe; core.pem_diagnostic = {}; core.pem_explorer_v2 = False
        else:
            core.pem.cfg.appe_rerank = diag_appe; core.pem_diagnostic = dict(diag); core.pem_explorer_v2 = True

    json.dump({o["name"]: o["yolo_prompt"] for o in core.objs}, open(OUT / "prompts.json", "w"), indent=1)

    # ------------------------------------------------ timing (production config, no diagnostics)
    production_mode(True)
    for _ in range(3):
        core.process(bgr, depth, K)
    prod_ms, rep_T = [], []
    for _ in range(a.reps):
        rows_p, ms, nb, _ = core.process(bgr, depth, K)
        prod_ms.append(ms)
    for _ in range(a.reps):
        ism_replay(core, bgr, rep_T)
    med = lambda xs: round(statistics.median(xs), 1)
    pem_only = []
    # PEM alone on fixed ISM masks
    _, cands0, prod0 = ism_replay(core, bgr)
    hits = [(n, r) for n, r in sorted(prod0.items()) if r.get("accepted") and r.get("mask") is not None]
    timing = {
        "gpu": torch.cuda.get_device_name(0), "reps": a.reps, "statistic": "median",
        "production_Sam6DCore.process_ms": {k: med([m[k] for m in prod_ms]) for k in prod_ms[0]},
        "stage_breakdown_ms": {
            "yolo_world": med([t["yolo"] for t in rep_T]),
            "dinov2_batched_forward": med([t["dinov2"] for t in rep_T]),
            "mobilesam": med([t["mobilesam"] for t in rep_T]),
            "gates(semantic+appearance+hsv+assignment, replay)": med([t["gates_replay_incl_hsv"] for t in rep_T]),
        },
        "n_yolo_unique_boxes": len(cands0), "n_accepted": len(hits),
        "note": "production ms 'ism' = DINOv2 + MobileSAM + gates + cross-object NMS; 'pem' = PEM for all accepted objects in one batch (coarse 6000->300, verification, fine).",
    }
    timing["stage_breakdown_ms"]["pem(batch of %d objects)" % len(hits)] = timing["production_Sam6DCore.process_ms"]["pem"]
    json.dump(timing, open(OUT / "timing.json", "w"), indent=1)
    print("[timing]", json.dumps(timing, indent=1))

    # ------------------------------------------------ ISM figures + gates.json
    pb, cands, prod = ism_replay(core, bgr)
    prod_accept = sorted(n for n, r in prod.items() if r.get("accepted"))
    my_accept = sorted(c["owner"] for c in cands if c["final_accept"])
    assert prod_accept == my_accept, (prod_accept, my_accept)
    img = bgr.copy()
    for c in cands:
        col = CAND_COLORS[c["id"] % len(CAND_COLORS)]
        x1, y1, x2, y2 = c["box"]
        cv2.rectangle(img, (x1, y1), (x2, y2), col, 1, cv2.LINE_AA)
        top = max(c["yolo_prompts"], key=lambda p: p["score"])
        extra = f" +{len(c['yolo_prompts']) - 1}" if len(c["yolo_prompts"]) > 1 else ""
        label(img, f"{top['prompt']} {top['score']:.2f}{extra}", (x1 + 1, y1 - 2), col, 0.33)
    cv2.imwrite(str(OUT / "yolo_boxes.png"), img)
    ov = bgr.astype(np.float32).copy()
    for c in cands:
        m = c["_mask"]
        if m is None:
            continue
        col = np.array(CAND_COLORS[c["id"] % len(CAND_COLORS)], np.float32)
        ov[m] = 0.45 * ov[m] + 0.55 * col
    ov = ov.astype(np.uint8)
    for c in cands:
        m = c["_mask"]
        if m is None:
            continue
        cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(ov, cs, -1, CAND_COLORS[c["id"] % len(CAND_COLORS)], 1, cv2.LINE_AA)
    cv2.imwrite(str(OUT / "masks.png"), ov)

    o0 = core.objs[0]
    gates = {
        "frame_index": a.frame, "t_ns": int(fr.t_ns),
        "thresholds": {"yolo_score": o0.get("score_threshold"), "1_semantic(DINOv2 cls cos, top-%d mean)" % o0["match_topk"]: o0["similarity_threshold"],
                       "2_appearance(DINOv2 blocks %s masked patch)" % o_n._blocks_of(o0): o_n._appe_gate_of(o0),
                       "3_color_hsv(HxS hist Bhattacharyya sim)": o0.get("hsv_gate_threshold"),
                       "color_tiebreak_template_sim": o0.get("color_tiebreak_template_sim"),
                       "cross_object_nms_iou": o0.get("cross_object_nms_iou"), "box_dedupe_iou": 0.95},
        "order": ["yolo proposals (multi-label, deduped)", "owner election = argmax semantic over all objects (colour tie-break among template-similar objects)",
                  "1_semantic", "2_appearance", "3_color_hsv", "4 exclusive assignment: one slot per object, best appearance wins", "4b cross-object NMS (IoU>=0.9)"],
        "note": "Gate scores are reported for the elected owner object. 'evaluated'=false means the pipeline stops before that gate; the score is still computed here for reference.",
        "accepted": my_accept, "production_recognize_frame_auto_accepted": prod_accept,
        "candidates": [{k: v for k, v in c.items() if not k.startswith("_")} for c in cands],
    }
    json.dump(gates, open(OUT / "gates.json", "w"), indent=1)

    acc = [c for c in cands if c["final_accept"]]
    for c in acc:
        x1, y1, x2, y2 = tight(c["_mask"], 6, bgr.shape)
        cv2.imwrite(str(OUT / f"crop_rgb_{c['owner']}.png"), bgr[y1:y2, x1:x2])
        cv2.imwrite(str(OUT / f"crop_mask_{c['owner']}.png"), (c["_mask"][y1:y2, x1:x2] * 255).astype(np.uint8))
    pri = {"3_color_hsv": 0, "2_appearance": 1, "4_exclusive_assignment": 2, "4_cross_object_nms": 2, "1_semantic": 3}
    rej = [c for c in cands if c["rejected_by"] in pri and c["_mask"] is not None]
    rej.sort(key=lambda c: (pri[c["rejected_by"]], -c["mask_px"]))
    reject_info = None
    if rej:
        c = rej[0]
        x1, y1, x2, y2 = tight(c["_mask"], 6, bgr.shape)
        cr = bgr[y1:y2, x1:x2].copy()
        cs, _ = cv2.findContours(c["_mask"][y1:y2, x1:x2].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.imwrite(str(OUT / "crop_reject.png"), cr)
        cv2.imwrite(str(OUT / "crop_reject_mask.png"), (c["_mask"][y1:y2, x1:x2] * 255).astype(np.uint8))
        reject_info = {k: v for k, v in c.items() if not k.startswith("_")}
        json.dump(reject_info, open(OUT / "crop_reject.json", "w"), indent=1)

    # ------------------------------------------------ PEM with Explorer diagnostics (same selection)
    # PEM samples its 6000 hypotheses at random: record how stable acceptance is over seeds,
    # then draw the figure from the first seed whose verification keeps every ISM object.
    ism_acc = sorted(n for n, _ in hits)
    stab = []
    for sd in range(a.seeds):
        torch.manual_seed(sd); np.random.seed(sd)
        rr, _, _, _ = core.process(bgr, depth, K)
        stab.append({"seed": sd, "accepted": sorted(r["object"] for r in rr),
                     "rejected": [{"object": x["object"], "reason": x["rejection_reason"], "mask_iou": x["mask_iou"],
                                   "texture_score": x["texture_score"]} for x in core.last_frame_diag["rejections"]]})
    fig_seed = next((t["seed"] for t in stab if t["accepted"] == ism_acc), a.seed) if a.seed < 0 else a.seed
    if a.seed < 0 and fig_seed < 0:
        fig_seed = 0
    json.dump({"ism_accepted": ism_acc, "figure_seed": fig_seed,
               "accept_rate": {n: sum(n in t["accepted"] for t in stab) / len(stab) for n in ism_acc}, "runs": stab},
              open(OUT / "pem_seed_stability.json", "w"), indent=1)
    production_mode(False)
    torch.manual_seed(fig_seed); np.random.seed(fig_seed)
    rows, ms, nb, lab = core.process(bgr, depth, K, want_mask=True)
    print("[pem] diag run ms", ms, [r["object"] for r in rows], "seed", fig_seed,
          "production same seed:", next(t["accepted"] for t in stab if t["seed"] == fig_seed) if fig_seed < a.seeds else None)
    pcs = {p["object"]: p for p in core.last_frame_diag["pem_candidates"]}
    mp = {n: core._pts[n]._pts / 1000.0 for n in core._pts}            # model points used by PEM (m)
    ext_json = json.load(open(ROOT / "integration/cad_extents.json"))

    def extents(n):
        return mp[n].min(0), mp[n].max(0)

    # final overlay (all accepted, live-viewer style)
    fo = bgr.copy()
    for r in rows:
        mn, mx = extents(r["object"])
        uv = draw_pose(fo, K, np.array(r["R"]), np.array(r["t_mm"]) / 1000.0, mn, mx, PALETTE.get(r["object"], (255, 255, 255)))
        tp = uv[:8][np.argmin(uv[:8, 1])]
        label(fo, r["object"], (tp[0] - 20, tp[1] - 6), PALETTE.get(r["object"], (255, 255, 255)), 0.42)
    cv2.imwrite(str(OUT / "final_overlay.png"), fo)
    fo2 = bgr.copy()
    for r in rows:
        e = ext_json.get(r["object"])
        if e is None:
            continue
        draw_pose(fo2, K, np.array(r["R"]), np.array(r["t_mm"]) / 1000.0, np.array(e["min_mm"]) / 1e3, np.array(e["max_mm"]) / 1e3, PALETTE.get(r["object"], (255, 255, 255)))
    cv2.imwrite(str(OUT / "final_overlay_cad_extents_json.png"), fo2)

    # PEM object choice: box-like preference
    names = [r["object"] for r in rows]
    pref = [a.pem_object] if a.pem_object else ["milk", "Febreze_high", "saffron", "Dinosaur"]
    pem_obj = next((n for n in pref if n in names and pcs.get(n, {}).get("explorer_candidates")), None)
    pem = {}
    if pem_obj:
        ex = pcs[pem_obj]["explorer_candidates"]
        dec = pcs[pem_obj]["decision"]
        row = next(r for r in rows if r["object"] == pem_obj)
        flags = ex["flags"].astype(int)
        R300, t300 = ex["R"], ex["t_m"]
        mask_pass = (flags & 4) > 0; tex_pass = (flags & 8) > 0; clus = (flags & 16) > 0; sel = (flags & 32) > 0
        # crop window: ISM mask bbox enlarged
        ism_mask = prod[pem_obj]["mask"].astype(bool)
        x1, y1, x2, y2 = tight(ism_mask, 0, bgr.shape)
        cx, cy, s = (x1 + x2) / 2, (y1 + y2) / 2, 1.5 * max(x2 - x1, y2 - y1)
        cx1, cy1 = int(max(0, cx - s / 2)), int(max(0, cy - s / 2))
        cx2, cy2 = int(min(bgr.shape[1], cx + s / 2)), int(min(bgr.shape[0], cy + s / 2))
        base = bgr[cy1:cy2, cx1:cx2]
        Kc = K.copy(); Kc[0, 2] -= cx1; Kc[1, 2] -= cy1
        sc = 320.0 / max(base.shape[:2])
        base = cv2.resize(base, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
        Kc[:2] *= sc
        mn, mx = extents(pem_obj)
        pts_vis = mp[pem_obj][::4]

        def render(Rk, tk, col, path):
            im = base.copy()
            uv = project(pts_vis @ np.asarray(Rk).T + tk, Kc)
            lay = im.copy()
            for u, v in uv.astype(int):
                if 0 <= u < im.shape[1] and 0 <= v < im.shape[0]:
                    cv2.circle(lay, (u, v), 1, col, -1)
            im = cv2.addWeighted(lay, 0.6, im, 0.4, 0)
            draw_pose(im, Kc, Rk, tk, mn, mx, col, 1, axes=True)
            cv2.imwrite(str(OUT / path), im)

        rank = ex["rank_geo"].astype(int)
        def cat(i):
            if sel[i]: return "selected"
            if not mask_pass[i]: return "rejected_mask_iou"
            if not tex_pass[i]: return "rejected_texture"
            if not clus[i]: return "rejected_not_in_cluster"
            return "survivor_cluster_member"
        cats = {}
        for i in np.argsort(rank):
            cats.setdefault(cat(i), []).append(int(i))
        picks = []
        plan = [("rejected_mask_iou", 2), ("rejected_texture", 1), ("rejected_not_in_cluster", 1), ("survivor_cluster_member", 1), ("selected", 1)]
        for k, n in plan:
            lst = cats.get(k, [])
            if lst:
                step = max(1, len(lst) // (n + 1))
                picks += [(k, lst[min(len(lst) - 1, step * (j + 1) - 1 if n > 1 else 0)]) for j in range(n)]
        while len(picks) < 4:
            lst = [i for i in cats.get("rejected_mask_iou", []) if i not in [p[1] for p in picks]]
            if not lst: break
            picks.append(("rejected_mask_iou", lst[-1]))
        COL = {"selected": (0, 255, 255), "survivor_cluster_member": (255, 200, 0), "rejected_mask_iou": (0, 0, 255),
               "rejected_texture": (0, 140, 255), "rejected_not_in_cluster": (255, 0, 255)}
        hyps = []
        for k, (kind, i) in enumerate(picks):
            render(R300[i], t300[i], COL[kind], f"hyp_{k}.png")
            hyps.append({"file": f"hyp_{k}.png", "index300": int(i), "proposal6000": int(ex["proposal6000"][i]),
                         "rank_geo": int(rank[i]), "status": kind, "geometry_score": r5(ex["geometry"][i]),
                         "mask_iou": r5(ex["mask_iou"][i]), "texture_score": r5(ex["texture"][i]),
                         "t_mm": [round(float(v) * 1000, 1) for v in t300[i]]})
        # fine
        im = base.copy()
        Rf, tf = np.array(row["R"]), np.array(row["t_mm"]) / 1000.0
        uv = project(pts_vis @ Rf.T + tf, Kc)
        lay = im.copy()
        for u, v in uv.astype(int):
            if 0 <= u < im.shape[1] and 0 <= v < im.shape[0]:
                cv2.circle(lay, (u, v), 1, (0, 230, 0), -1)
        im = cv2.addWeighted(lay, 0.45, im, 0.55, 0)
        draw_pose(im, Kc, Rf, tf, mn, mx, (0, 230, 0), 2)
        cv2.imwrite(str(OUT / "fine.png"), im)
        v = row.get("verify") or {}
        pem = {
            "object": pem_obj, "frame_index": a.frame, "torch_seed": fig_seed,
            "n_coarse_proposals": int(core.pcfg.model.coarse_point_matching.nproposal1) if hasattr(core.pcfg.model.coarse_point_matching, "nproposal1") else 6000,
            "n_hypotheses": int(len(R300)),
            "thresholds": {"mask_iou_min": rt["verify"]["mask_iou_min"], "texture_min_score": rt["verify"]["texture_min_score"],
                           "cluster_rotation_deg": rt["verify"]["cluster_rotation_deg"], "cluster_translation_mm": rt["verify"]["cluster_translation_mm"],
                           "cluster_min_occupancy": rt["verify"]["cluster_min_occupancy"]},
            "counts_from_flags": {"mask_pass": int(mask_pass.sum()), "texture_pass": int(tex_pass.sum()), "cluster_member": int(clus.sum())},
            "mask_survivors": v.get("mask_survivors"), "texture_survivors": v.get("texture_survivors"),
            "cluster_size": v.get("cluster_size"), "cluster_occupancy": v.get("cluster_occupancy"),
            "selected_index300": v.get("selected_index300"), "selected_proposal6000_index": v.get("selected_proposal6000_index"),
            "rank_geo_of_selected": v.get("rank_geo"), "selection_method": v.get("selection_method"),
            "final_mask_iou": row.get("mask_iou"), "final_texture_score": row.get("texture_score"),
            "coverage": v.get("coverage"), "size_ratio": v.get("size_ratio"),
            "final_pose": {"R": row["R"], "t_mm": row["t_mm"], "pose_score": row["score"]},
            "crop_window_xyxy": [cx1, cy1, cx2, cy2], "crop_scale": round(sc, 4),
            "hypotheses_rendered": hyps,
            "colour_key(BGR)": COL,
        }
        json.dump(pem, open(OUT / "pem.json", "w"), indent=1)
    allp = {"rows": [{k: r.get(k) for k in ("object", "score", "R", "t_mm", "bbox", "rank_geo", "mask_iou", "texture_score", "cluster_occupancy", "ism")} for r in rows],
            "verify": {r["object"]: {k: (r.get("verify") or {}).get(k) for k in ("mask_survivors", "texture_survivors", "cluster_size", "cluster_occupancy", "mask_iou", "texture_score", "accepted", "rejection_reason")} for r in rows},
            "rejections": core.last_frame_diag.get("rejections"), "pem_input": [{k: v for k, v in p.items() if k in ("object", "bbox", "mask_px", "valid_depth_px", "radius_inliers", "input_rejection", "used_by_pem", "rejection_reason")} for p in pcs.values()],
            "model_points_vs_cad_extents_json_mm": {n: {"model_points_min": [round(float(x) * 1e3, 2) for x in mp[n].min(0)], "model_points_max": [round(float(x) * 1e3, 2) for x in mp[n].max(0)],
                                                         "cad_extents_min": ext_json.get(n, {}).get("min_mm"), "cad_extents_max": ext_json.get(n, {}).get("max_mm")} for n in names}}
    json.dump(allp, open(OUT / "pem_all_objects.json", "w"), indent=1)
    print("[done] accepted ISM", my_accept, "PEM rows", names, "pem object", pem_obj)


if __name__ == "__main__":
    main()
