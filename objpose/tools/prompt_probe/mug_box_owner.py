#!/usr/bin/env python3
"""mug_box_owner.py — who takes the boxes proposed by Mugcup_high's prompt under relative assignment (2026-09-29).

Replays the owner election of yolo_ism_object_n.assign_frame_relative for every box the Mugcup prompt proposes
(same core, same config as the live run): owner = argmax semantic score over all objects, then the colour tie-break
among template-similar objects, then the owner's own gates (similarity, masked appearance, HSV). Each box is also
labelled pink / not pink (mostly pastel pink pixels) so boxes on the actual mug can be told from boxes on other items.
    python objpose/tools/prompt_probe/mug_box_owner.py <run dir> <SAM session> [--step 5] [--out DIR]
"""
import argparse, collections, json, os, sys
from pathlib import Path
import cv2, numpy as np, torch, yaml
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc")); sys.path.insert(0, str(REPO / "sam6d_realtime/realtime")); sys.path.insert(0, str(REPO / "sam6d_realtime"))
ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("session"); ap.add_argument("--obj", default="Mugcup_high")
ap.add_argument("--step", type=int, default=5); ap.add_argument("--out", default=None); ap.add_argument("--save", type=int, default=24)
a = ap.parse_args()
out = Path(a.out or REPO / "objpose/output/mug_box_owner").resolve(); out.mkdir(parents=True, exist_ok=True)
cfg = yaml.safe_load(open(Path(a.run) / "sam6d_config.yaml")); rt = cfg["runtime"]
os.chdir(REPO / "sam6d_realtime")
import verify_config as VC
from sam6d_core import Sam6DCore
import yolo_ism_object_n as o_n, ism_hsv
import yolo_ism as yi
from conv_session import ConvSession
core = Sam6DCore(cfg["ism"]["config"], cfg["ism"].get("objects", []), rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2),
                 verify=rt.get("verify", VC.UNSET), fine_npoint_override=rt.get("fine_npoint"))
objs = core.objs; by = {o["name"]: o for o in objs}; tau = o_n._color_tiebreak_tau(objs[0])
pi = core.unique_prompts.index(by[a.obj]["yolo_prompt"])
thr = min(float(o.get("score_threshold", 0.0)) for o in objs)

def pink_frac(img, box):
    x1, y1, x2, y2 = [int(v) for v in box]
    c = img[max(y1, 0):y2, max(x1, 0):x2]
    if c.size == 0: return 0.0
    h, s, v = cv2.split(cv2.cvtColor(c, cv2.COLOR_BGR2HSV))
    return float((((h >= 140) | (h <= 8)) & (s >= 25) & (s <= 170) & (v >= 90)).mean())

sess = ConvSession(a.session)
rows, C, saved = [], collections.Counter(), 0
for i in range(0, len(sess), a.step):
    bgr = sess.read(i, False).color_bgr; rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB); norm = yi.normalize_rgb(rgb)
    h, w = bgr.shape[:2]
    with o_n.multi_label_nms(core.multi_label):
        r = core.yolo.predict(bgr, conf=core.min_score, imgsz=core.imgsz, verbose=False, device=core.device)[0]
    boxes = []
    for j in range(len(r.boxes)):
        if int(r.boxes.cls[j]) != pi or float(r.boxes.conf[j]) < thr: continue
        xy = r.boxes.xyxy[j].tolist()
        x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
        boxes.append(([x1, y1, max(x1 + 1, min(int(xy[2]), w)), max(y1 + 1, min(int(xy[3]), h))], float(r.boxes.conf[j])))
    if not boxes: continue
    crops = [yi.crop_resize_pad(norm, b) for b, _ in boxes]
    keep = [(b, c, cr) for (b, c), cr in zip(boxes, crops) if cr is not None]
    if not keep: continue
    need = sorted({bb for o in objs for bb in o["_need_blocks"]})
    cls_all, patch_all = o_n.dinov2_blocks_forward(core.model, [k[2] for k in keep], core.device, need)
    masks = yi.segment_boxes(core.segmentor, bgr, [k[0] for k in keep], core.device)
    for n, (box, conf, _) in enumerate(keep):
        sems = {o["name"]: float(yi.semantic_score(cls_all[n], o["tcls"], o["match_topk"])) for o in objs}
        top0 = max(sems, key=sems.get); top = top0; mask = masks[n]
        hsv = {o["name"]: float(ism_hsv.shadow_score(bgr, box, mask, o.get("_hsv_proto"))) for o in objs} if mask is not None else {}
        if tau > 0.0 and hsv:
            tie = [k for k in sems if core.tsim[top].get(k, 0.0) >= tau]
            if len(tie) > 1: top = max(tie, key=lambda k: hsv[k])
        o = by[top]; gate = "pass"
        if sems[top] < o["similarity_threshold"]: gate = "below-sim"
        elif mask is None: gate = "no-mask"
        else:
            qb = {bb: patch_all[bb][n].cpu() for bb in o["_need_blocks"]}
            bt = int(torch.argmax(o["tcls"] @ cls_all[n]))
            if float(o_n.masked_appe_blocks(qb, mask, box, core.pool, o, bt)) < o_n._appe_gate_of(o): gate = "below-appe"
            elif bool(o.get("hsv_gate_enabled", False)) and hsv and hsv[top] < float(o.get("hsv_gate_threshold", 0.0)): gate = "below-hsv"
        pink = pink_frac(bgr, box) >= 0.25
        mug_rank = sorted(sems, key=sems.get, reverse=True).index(a.obj) + 1
        row = {"i": i, "box": box, "conf": round(conf, 3), "pink": pink, "owner_sem": top0, "owner": top,
               "tiebreak": top != top0, "gate": gate, "mug_sem": round(sems[a.obj], 4), "owner_sem_score": round(sems[top], 4),
               "mug_rank": mug_rank, "mug_sim_thr": by[a.obj]["similarity_threshold"],
               "sems": {k: round(v, 4) for k, v in sems.items()}, "hsv": {k: round(v, 4) for k, v in hsv.items()}}
        rows.append(row)
        C[("pink" if pink else "notpink", top, gate)] += 1
        if pink and top != a.obj and saved < a.save:
            img = bgr.copy(); x1, y1, x2, y2 = box
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(img, f"owner {top} ({sems[top]:.3f}) mug {sems[a.obj]:.3f} #{mug_rank}", (8, 22), 0, 0.6, (0, 0, 255), 2)
            cv2.imwrite(str(out / f"f{i:05d}_{n}.jpg"), img); saved += 1
json.dump({"counts": {"|".join(k): v for k, v in C.items()}, "rows": rows}, open(out / "owners.json", "w"), indent=1)
pinkrows = [r for r in rows if r["pink"]]
print(f"boxes from the {a.obj} prompt: {len(rows)} (pink {len(pinkrows)})")
print("pink boxes -> owner:", collections.Counter(r["owner"] for r in pinkrows).most_common())
print("pink boxes -> owner by semantic alone:", collections.Counter(r["owner_sem"] for r in pinkrows).most_common())
print("pink boxes, colour tie-break changed the owner:", sum(r["tiebreak"] for r in pinkrows))
print("pink boxes owned by the mug -> gate:", collections.Counter(r["gate"] for r in pinkrows if r["owner"] == a.obj).most_common())
print("pink boxes, mug semantic rank:", collections.Counter(r["mug_rank"] for r in pinkrows).most_common())
ms = [r["mug_sem"] for r in pinkrows]
if ms: print(f"pink boxes, mug semantic: median {np.median(ms):.3f} p90 {np.percentile(ms, 90):.3f}; mug similarity_threshold {pinkrows[0]['mug_sim_thr']}")
print("not-pink boxes -> owner:", collections.Counter(r["owner"] for r in rows if not r["pink"]).most_common())
