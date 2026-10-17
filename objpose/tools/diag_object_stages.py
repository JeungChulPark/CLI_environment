#!/usr/bin/env python3
"""diag_object_stages.py — where does one object get lost in the proposed pipeline, on a recorded session?
  문구 검출(YOLO-World 상자, 점수) -> 후보 검증(의미·외형·색·배타 판정, PEM 입력으로 가는지) -> 자세 검증(탈락 사유) -> 최종 답
Same core and config as the live run (the run's sam6d_config.yaml), frames read straight from the SAM session.
    python objpose/tools/diag_object_stages.py <run dir> <SAM session dir> Mugcup_high [--step 5] [--save N]
"""
import argparse, collections, json, sys
from pathlib import Path
import cv2, numpy as np, yaml
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "objpose/pc")); sys.path.insert(0, str(REPO / "sam6d_realtime/realtime"))
from rs_session import RsSession
from conv_session import ConvSession

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("session"); ap.add_argument("obj")
ap.add_argument("--step", type=int, default=5); ap.add_argument("--save", type=int, default=12)
ap.add_argument("--out", default=None)
a = ap.parse_args()
cfg = yaml.safe_load(open(Path(a.run) / "sam6d_config.yaml")); rt = cfg["runtime"]
import os; os.chdir(REPO / "sam6d_realtime")
import verify_config as VC
from sam6d_core import Sam6DCore
core = Sam6DCore(cfg["ism"]["config"], cfg["ism"].get("objects", []), rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2),
                 appe_rerank=rt.get("appe_rerank"), verify=rt.get("verify", VC.UNSET), precision=rt.get("precision"),
                 mask_cache=None, pem_schedule=rt.get("pem_schedule"), coarse_npoint_override=rt.get("coarse_npoint"),
                 fine_npoint_override=rt.get("fine_npoint"))
obj = next(o for o in core.objs if o["name"] == a.obj); prompt = obj["yolo_prompt"]; pi = core.unique_prompts.index(prompt)
print("object", a.obj, "prompt", repr(prompt), "prompts", core.unique_prompts)
sess = RsSession(a.session) if (Path(a.session) / "rgbd_timestamp_associations.json").exists() else ConvSession(a.session)
out = Path(a.out or (REPO / f"objpose/output/diag_{a.obj}")); out.mkdir(parents=True, exist_ok=True)
C = collections.Counter(); reasons = collections.Counter(); scores = []; saved = 0; rows = []
for i in range(0, len(sess), a.step):
    fr = sess.read(i, True); depth = sess.align(fr.depth_raw); bgr = fr.color_bgr
    res = core.yolo.predict(bgr, conf=core.min_score, imgsz=core.imgsz, verbose=False, device=core.device)
    b = res[0].boxes if len(res) and res[0].boxes is not None else []
    mine = [(float(b.conf[j]), b.xyxy[j].cpu().numpy().tolist()) for j in range(len(b)) if int(b.cls[j]) == pi]
    dets = core.process(bgr, depth, sess.K)[0]; dg = core.last_frame_diag
    cand = [c for c in dg["pem_candidates"] if c["object"] == a.obj]
    rej = [r for r in dg["rejections"] if r.get("object") == a.obj]
    fin = [d for d in dets if d.get("object") == a.obj and d.get("verified", True)]
    C["frames"] += 1; C["yolo_box"] += bool(mine); C["to_pem"] += bool(cand); C["verified"] += bool(fin)
    C["pose_rejected"] += bool(rej) and not fin
    for r in rej: reasons[r.get("rejection_reason")] += 1
    if mine: scores.append(max(s for s, _ in mine))
    rows.append({"i": i, "yolo": mine, "to_pem": bool(cand), "rej": [r.get("rejection_reason") for r in rej], "ok": bool(fin),
                 "other_objects_on_box": [d.get("object") for d in dets]})
    if mine and not fin and saved < a.save:
        img = bgr.copy()
        for s, (x1, y1, x2, y2) in mine:
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(img, f"{prompt} {s:.2f}", (int(x1), int(y1) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img, f"frame {i}: to_pem={bool(cand)} rej={[r.get('rejection_reason') for r in rej]}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.imwrite(str(out / f"f{i:05d}.jpg"), img); saved += 1
json.dump({"counts": C, "reasons": reasons, "rows": rows}, open(out / "diag.json", "w"))
print(dict(C)); print("pose rejection reasons", reasons.most_common())
if scores: print("yolo score of the object: median %.3f p10 %.3f p90 %.3f (n %d)" % (np.median(scores), np.percentile(scores, 10), np.percentile(scores, 90), len(scores)))
