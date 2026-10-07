#!/usr/bin/env python3
"""Our recognizer on the BOP YCB-V test list, one image at a time.

Deployed pipeline, called exactly as in paper_etrij_261003/src/same_gpu/bench_ours.py and
real_stage/run_real_stage.py: Sam6DCore.process (YOLO-World text prompts -> DINOv2 ViT-S +
MobileSAM + semantic / appearance / HSV gates + exclusive assignment + cross-object NMS ->
PEM -> pose verification: silhouette IoU >= 0.421, texture >= 0.450, consensus).
YCB settings (paper): consensus (verify.cluster_min_occupancy) 0.3 instead of 0.5; HSV hue
correction 0 for every YCB object; every other threshold = configs/yolo_ism_objects.yaml
defaults and the deployed verify defaults.

Per image the recognizer runs over the image's target objects only (test_targets_bop19):
the active object set (prompt list, prompt groups) is switched before the timed region;
YOLO-World's text embeddings for that set are computed there too (set_classes).

--mode text   : YOLO-World text-prompt proposals (deployed)
--mode gtbox  : YOLO-World replaced by the GT 2D boxes (scene_gt_info bbox_obj) of the
                target objects (conf 1.0, under the object's own prompt); rest unchanged.
--mode gtbox_locked : supplementary sanity variant, NOT the protocol: GT boxes as above, and
                the per-box owner election (relative_assignment_enabled) switched off, so a
                GT box can only be claimed by its own object (per-object gates, then
                cross-object NMS, as the deployed pre-Phase-4 path).

--proposer SPEC : (text mode) the YOLO-World call is replaced by a det_study/proposers.py
                proposal source (other prompts, prompt ensembles, larger detectors, CAD visual
                prompts, unions). Its activation for the image's target set happens outside the
                timed region, like set_classes; its inference is inside.

    python run_ours.py --mode text|gtbox|gtbox_locked [--limit N] [--proposer SPEC --tag NAME]
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import torch

import data as D
import paths as P

VERIFY = {"enabled": True, "mask_iou_min": 0.420998, "texture_min_score": 0.449562,
          "cluster_rotation_deg": 20, "cluster_translation_mm": 25,
          "cluster_min_occupancy": P.CONSENSUS_OCC}


def sync():
    torch.cuda.synchronize()


class _Boxes:
    def __init__(self, xyxy, cls, conf):
        self.xyxy = torch.tensor(xyxy, dtype=torch.float32).reshape(-1, 4)
        self.cls = torch.tensor(cls, dtype=torch.float32)
        self.conf = torch.tensor(conf, dtype=torch.float32)

    def __len__(self):
        return len(self.conf)


class _Res:
    def __init__(self, boxes):
        self.boxes = boxes


CORE_KW = {}      # extra Sam6DCore options (verification cost study: appe_rerank topk/stride, precision)


def build_core():
    sys.path.insert(0, str(P.SR / "realtime")); sys.path.insert(0, str(P.SR))
    cwd = os.getcwd(); os.chdir(P.SR)
    import sam6d_core
    import yolo_ism_object_n as o_n
    sam6d_core.REPO = str(P.CORE_REPO)        # PEM assets + CLIP weights for the YCB objects
    np.random.seed(0); torch.manual_seed(0)
    names = [P.obj_name(i) for i in range(1, 22)]
    core = sam6d_core.Sam6DCore(str(P.OURS_CFG), names, "cuda:0", 0.2, verify=dict(VERIFY), **CORE_KW)
    os.chdir(cwd)
    assert sorted(o["name"] for o in core.objs) == names, "missing YCB objects (templates/caches?)"
    core._all_objs = list(core.objs)
    core._active = None
    return core, o_n


def activate(core, o_n, oids):
    key = tuple(oids)
    if core._active == key:
        return
    want = {P.obj_name(i) for i in oids}
    objs = [o for o in core._all_objs if o["name"] in want]
    core.unique_prompts, core.groups = o_n.build_prompt_groups(objs)
    m = core.yolo.model
    cm = getattr(m, "clip_model", None)
    if cm is not None:            # weights moved to cuda with the YOLO model, but .device stayed "cpu"
        cm.device = next(cm.model.parameters()).device
    core.yolo.set_classes(core.unique_prompts)
    core._active = key


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["text", "gtbox", "gtbox_locked"], required=True)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--out", default="")
    ap.add_argument("--proposer", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--no-verify", action="store_true", help="ablation: pose verification off")
    ap.add_argument("--top-k", type=int, default=0, help="proposals per object looked at (default: config, 3)")
    ap.add_argument("--assign-fallback", action="store_true", help="gate study 1: box ownership with fallback")
    ap.add_argument("--verify-topk", type=int, default=0, help="pose verification: geometry candidates checked (default 300)")
    ap.add_argument("--verify-candidates", type=int, default=0,
                    help="pose verification: only the K best geometry candidates are measured (verify.candidate_topk; default all 300)")
    ap.add_argument("--verify-stride", type=int, default=0, help="pose verification: candidate stride (default 1)")
    ap.add_argument("--precision", default="", help="PEM precision (fp32 default | fp16)")
    ap.add_argument("--orig-ism", type=float, default=0.0,
                    help="replace our ISM by the original SAM-6D ISM (FastSAM-x, top-1 per object, ISM score > x); "
                         "our PEM + pose verification run on its masks")
    ap.add_argument("--orig-desc", default="dinov2_vitl14", help="descriptor of --orig-ism (dinov2_vitl14 | dinov2_vits14)")
    ap.add_argument("--orig-seg", default="fastsam_full", choices=["fastsam_full", "fastsam_msam", "text_msam"],
                    help="mask source of --orig-ism: FastSAM-x masks (upstream) | FastSAM-x boxes + MobileSAM | "
                         "YOLO-World text boxes (method-1 prompts, top-3 per target) + MobileSAM")
    ap.add_argument("--ism-only", action="store_true",
                    help="recognition study: stop after the object decision (no PEM) and score the named masks "
                         "against the GT visible masks (correct = IoU >= 0.5 with that object's mask)")
    ap.add_argument("--size-gate", type=float, default=0.0, help="gate study 2: reject masks whose depth extent > x * CAD diagonal")
    ap.add_argument("--gate-sem", type=float, default=0.0, help="gate study 3: semantic threshold override (0.35)")
    ap.add_argument("--gate-appe", type=float, default=0.0, help="gate study 3: masked-appearance threshold override (0.605)")
    ap.add_argument("--gate-hsv", type=float, default=-1.0, help="gate study 3: HSV threshold override (0.1214); 0 = off")
    a = ap.parse_args()
    out = a.out or str(P.OUT / f"pred_ours_{a.mode}{'_' + a.tag if a.tag else ''}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    items = D.load_targets()
    if a.limit:
        items = items[:a.limit]
    if a.no_verify:
        VERIFY["enabled"] = False
    if a.verify_candidates:
        VERIFY["candidate_topk"] = a.verify_candidates
    if a.verify_topk or a.verify_stride:
        CORE_KW["appe_rerank"] = {"topk": a.verify_topk or 300, "stride": a.verify_stride or 1}
    if a.precision:
        CORE_KW["precision"] = a.precision
    core, o_n = build_core()
    real_predict = core.yolo.predict
    gt_holder = {}

    def gt_predict(bgr, **k):
        g = gt_holder["boxes"]
        return [_Res(_Boxes([b for b, _ in g], [c for _, c in g], [1.0] * len(g)))]

    if a.mode.startswith("gtbox"):
        core.yolo.predict = gt_predict
    prop = None
    if a.proposer:
        sys.path.insert(0, str(P.HERE / "det_study"))
        import proposers
        prop = proposers.build(a.proposer)

        def prop_predict(bgr, **k):
            d = prop.predict(bgr, conf=core.min_score)
            ci = [core.unique_prompts.index(P.PROMPTS[o - 1]) for _, o, _ in d]
            return [_Res(_Boxes([b for b, _, _ in d], ci, [s for _, _, s in d]))]
        core.yolo.predict = prop_predict
    if a.top_k:
        for o in core._all_objs:
            o["top_k"] = a.top_k
    for o in core._all_objs:
        if a.assign_fallback:
            o["assignment_fallback"] = True
        if a.gate_sem:
            o["similarity_threshold"] = a.gate_sem
        if a.gate_appe:
            o["appe_v2_gate"] = a.gate_appe; o["appe_gate"] = a.gate_appe
        if a.gate_hsv >= 0:
            o["hsv_gate_threshold"] = a.gate_hsv
            o["hsv_gate_enabled"] = a.gate_hsv > 0
    if a.size_gate:
        core.size_gate_max = a.size_gate
    orig = None
    if a.orig_ism:
        import run_orig as ROG
        import trimesh, trimesh.exchange.load as _tl
        patched = trimesh.load_mesh                  # Sam6DCore serves its PEM points through load_mesh
        trimesh.load_mesh = _tl.load_mesh
        ism_o, bank_o = ROG.build_ism(model_name=a.orig_desc)
        trimesh.load_mesh = patched
        cache_o, cur = {}, {}
        if a.orig_seg != "fastsam_full":
            # mask-source study: the same original ISM / PEM / verification, only the masks change
            import cv2
            import yolo_ism as yi
            msam = yi.build_segmentor(str(P.REPO / "sam6d_realtime" / "mobile_sam.pt"), "cuda:0")
            fs_model, fs_args = ism_o.segmentor_model.model, ism_o.segmentor_model.args
            if a.orig_seg == "text_msam":
                sys.path.insert(0, str(P.HERE / "det_study"))
                import proposers
                tprop = proposers.build("text:yolov8m-worldv2.pt:" + str(P.HERE / "det_study/out/prompts_best_m.json"))

            def box_masks(image):
                bgr_i = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if a.orig_seg == "fastsam_msam":       # FastSAM-x boxes (upstream call), masks by MobileSAM
                    det = fs_model.predict(image, **fs_args)[0]
                    boxes = det.boxes.xyxy.tolist() if det.boxes is not None else []
                else:                                   # YOLO-World text boxes, top-3 per target, by MobileSAM
                    tprop.activate(cur["oids"])
                    top = {}
                    for d in sorted(tprop.predict(bgr_i, 0.02), key=lambda d: -d[2]):
                        if len(top.setdefault(d[1], [])) < 3:
                            top[d[1]].append(d[0])
                    boxes = [b for bs in top.values() for b in bs]
                if not boxes:
                    return None
                ms = yi.segment_boxes(msam, bgr_i, boxes, "cuda:0")
                keep = [j for j, m in enumerate(ms) if m is not None and m.any()]
                if not keep:
                    return None
                return {"masks": torch.from_numpy(np.stack([ms[j] for j in keep])).float().to(ROG.DEV),
                        "boxes": torch.tensor([boxes[j] for j in keep], dtype=torch.float32, device=ROG.DEV)}
            ism_o.segmentor_model.generate_masks = box_masks

        def orig_recognize(groups, pb, bgr, rgb, norm_full, *args, **kw):
            """original ISM (run_orig.run_frame up to the top-1 masks), in our results format"""
            oids = cur["oids"]
            ROG.select_bank(ism_o, bank_o, oids, cache_o)
            props = ism_o.segmentor_model.generate_masks(rgb)
            res = {P.obj_name(o): {"accepted": False, "mask": None, "decision": "no-object(orig)"} for o in oids}
            if props is None:
                return res
            dets = ROG.Detections(props)
            dets.remove_very_small_detections(config=ism_o.post_processing_config.mask_post_processing)
            if len(dets) == 0:
                return res
            qd, qa = ism_o.descriptor_model(rgb, dets)
            sel, pio, sem, best = ism_o.compute_semantic_score(qd)
            dets.filter(sel); qa = qa[sel, :]
            if len(dets) == 0:
                return res
            appe, ref_aux = ism_o.compute_appearance_score(best, pio, qa)
            batch = {"depth": torch.from_numpy(cur["raw"].astype(np.int32)).unsqueeze(0).to(ROG.DEV),
                     "cam_intrinsic": torch.from_numpy(cur["K"]).unsqueeze(0).to(ROG.DEV),
                     "depth_scale": torch.from_numpy(np.array(cur["ds"])).unsqueeze(0).to(ROG.DEV)}
            uv = ism_o.project_template_to_image(best, pio, batch, dets.masks)
            geo, vis = ism_o.compute_geometric_score(uv, dets, qa, ref_aux, visible_thred=ism_o.visible_thred)
            final = (sem + appe + geo * vis) / (1 + 1 + vis)
            dets.add_attribute("scores", final); dets.add_attribute("object_ids", pio)
            dets.apply_nms_per_object_id(nms_thresh=ism_o.post_processing_config.nms_thresh)
            for k in torch.unique(dets.object_ids).tolist():
                idx = torch.nonzero(dets.object_ids == k)[:, 0]
                j = int(idx[torch.argmax(dets.scores[idx])]); sc = float(dets.scores[j])
                if sc <= a.orig_ism:
                    continue
                m = (dets.masks[j] > 0.5).cpu().numpy()
                ys, xs = np.nonzero(m)
                if len(ys) == 0:
                    continue
                res[P.obj_name(oids[int(k)])] = {"accepted": True, "mask": m, "decision": "detected(orig)",
                                                 "box": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1],
                                                 "best_sem": sc, "masked_appe": sc}
            return res
        o_n.recognize_frame_auto = orig_recognize
        core.yolo.predict = lambda bgr, **k: [_Res(_Boxes([], [], []))]
        orig = cur
    cap = {}
    if a.ism_only:                     # keep the decision, hand PEM nothing
        real_rec = o_n.recognize_frame_auto

        def capture(*args, **kw):
            r = real_rec(*args, **kw)
            cap["res"] = r
            return {k: {**v, "accepted": False} for k, v in r.items()}
        o_n.recognize_frame_auto = capture
    if a.mode == "gtbox_locked":
        for o in core._all_objs:
            o["relative_assignment_enabled"] = False

    def run(sid, iid, oids):
        bgr, raw, depth_mm, K, ds = D.load_image(sid, iid)
        activate(core, o_n, oids)
        if orig is not None:
            orig.update(oids=oids, raw=raw, K=K, ds=ds)
        if prop is not None:
            prop.activate(oids)
        if a.mode.startswith("gtbox"):
            g = []
            for inst in D.gt_instances(sid, iid):
                if inst["obj_id"] in oids:
                    x, y, w, h = inst["bbox_obj"]
                    ci = core.unique_prompts.index(P.PROMPTS[inst["obj_id"] - 1])
                    g.append(([x, y, x + w, y + h], ci))
            gt_holder["boxes"] = g
        sync(); t0 = time.perf_counter()
        rows, ms, nb, _ = core.process(bgr, depth_mm, K)
        sync(); t1 = time.perf_counter()
        return rows, ms, nb, 1e3 * (t1 - t0)

    for sid, iid, oids in items[:a.warmup]:
        run(sid, iid, oids)
    res = D.header(f"ours_{a.mode}", {
        "pipeline": "Sam6DCore.process (deployed)", "config": str(P.OURS_CFG), "verify": VERIFY,
        "det_score_thresh": 0.2, "warmup_images": a.warmup,
        "proposals": (prop.name if prop is not None else "YOLO-World text prompts") if a.mode == "text"
        else "GT bbox_obj of target objects",
        "proposer_spec": a.proposer or None,
        "orig_ism": {"thresh": a.orig_ism, "desc": a.orig_desc, "seg": a.orig_seg} if a.orig_ism else None,
        "verify_cost": {"candidates": a.verify_candidates or None, "topk": a.verify_topk, "stride": a.verify_stride, "precision": a.precision or None},
        "gate_study": {"assign_fallback": a.assign_fallback, "size_gate": a.size_gate, "gate_sem": a.gate_sem,
                       "gate_appe": a.gate_appe, "gate_hsv": a.gate_hsv},
        "prompts": {P.obj_name(i + 1): p for i, p in enumerate(P.PROMPTS)}})
    res["images"] = []
    if a.ism_only:
        import cv2
        recog, tms = [], []
        for n, (sid, iid, oids) in enumerate(items):
            cap.clear()
            rows, ms, nb, tot = run(sid, iid, oids)
            tms.append({"total": tot, **ms})
            r = cap.get("res") or {}
            gt = D.scene(sid)["scene_gt"][str(iid)]
            root = P.YCBV / "test" / f"{sid:06d}"
            for o in oids:
                v = r.get(P.obj_name(o)) or {}
                m = v.get("mask") if v.get("accepted") else None
                k = next((j for j, g in enumerate(gt) if g["obj_id"] == o), None)
                iou = 0.0
                if m is not None and k is not None:
                    g = cv2.imread(str(root / "mask_visib" / f"{iid:06d}_{k:06d}.png"), cv2.IMREAD_GRAYSCALE) > 0
                    m = np.asarray(m, bool)
                    iou = float(np.logical_and(m, g).sum()) / float(max(np.logical_or(m, g).sum(), 1))
                recog.append({"scene": sid, "im": iid, "obj": o, "answered": m is not None, "iou": round(iou, 4)})
            if n % 50 == 0 or n == len(items) - 1:
                c = sum(x["answered"] and x["iou"] >= 0.5 for x in recog)
                print(f"[{n + 1}/{len(items)}] found {100 * c / len(recog):.1f}% {tot:.0f} ms", flush=True)
        ans = [x for x in recog if x["answered"]]
        cor = [x for x in ans if x["iou"] >= 0.5]
        res["recognition"] = {
            "targets": len(recog), "answers": len(ans), "correct": len(cor), "wrong": len(ans) - len(cor),
            "found_pct": round(100 * len(cor) / max(len(recog), 1), 1),
            "precision_pct": round(100 * len(cor) / max(len(ans), 1), 1),
            "wrong_per_image": round((len(ans) - len(cor)) / max(len(items), 1), 3),
            "mean_iou_correct": round(float(np.mean([x["iou"] for x in cor])), 3) if cor else None,
            "time_ms_median": {k: round(float(np.median([t[k] for t in tms])), 1) for k in ("total", "yolo", "ism")},
            "per_object_found_pct": {P.obj_name(o): round(100 * sum(x["answered"] and x["iou"] >= 0.5 for x in recog if x["obj"] == o)
                                                          / max(sum(1 for x in recog if x["obj"] == o), 1), 1)
                                     for o in sorted({x["obj"] for x in recog})}}
        res["rows"] = recog
        res["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
        json.dump(res, open(out, "w"), indent=1, default=float)
        print("->", out, {k: v for k, v in res["recognition"].items() if k != "per_object_found_pct"})
        return
    for n, (sid, iid, oids) in enumerate(items):
        rows, ms, nb, tot = run(sid, iid, oids)
        preds = []
        for r in rows:
            preds.append({"obj_id": P.oid_of(r["object"]), "R": r["R"], "t_mm": r["t_mm"],
                          "score": r["score"], "time_ms": round(tot, 2), "bbox": r.get("bbox"),
                          "ism": r.get("ism"), "mask_iou": r.get("mask_iou"),
                          "texture_score": r.get("texture_score"),
                          "cluster_occupancy": r.get("cluster_occupancy")})
        rej = [{k: v for k, v in x.items() if k in ("object", "rejection_reason", "mask_iou",
                                                      "texture_score", "cluster_occupancy")}
               for x in core.last_frame_diag.get("rejections", [])]
        ism_acc = [c["object"] for c in core.last_frame_diag.get("pem_candidates", [])]
        res["images"].append({"scene_id": sid, "im_id": iid, "targets": oids, "time_ms": round(tot, 2),
                              "stage_ms": ms, "n_boxes": int(nb), "ism_accepted": ism_acc,
                              "pose_rejected": rej, "pem_error": core.last_frame_diag.get("pem_error"),
                              "preds": preds})
        if n % 50 == 0 or n == len(items) - 1:
            print(f"[{n + 1}/{len(items)}] {sid}/{iid} {tot:.0f} ms boxes {nb} ism {len(ism_acc)} "
                  f"out {len(preds)}", flush=True)
    res["time_ms"] = D.stats([x["time_ms"] for x in res["images"]])
    res["stage_ms"] = {k: D.stats([x["stage_ms"][k] for x in res["images"]]) for k in ("yolo", "ism", "pem")}
    res["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(res, open(out, "w"), indent=1, default=float)
    print("->", out, res["time_ms"])


if __name__ == "__main__":
    main()
