#!/usr/bin/env python3
"""mug_hue_sweep.py — choose Mugcup_high's hsv_hue_correction_ocv by measurement (2026-09-29).

Uses the boxes recorded by mug_box_owner.py on a converted SAM session:
  positives = pink boxes (on the real mug) whose owner is the mug (the only boxes the mug's HSV gate ever sees)
  negatives = every non-pink box in the same run (boxes on other items), scored against the MUG reference
For each hue correction the mug reference is rebuilt exactly as the runtime cache (ism_hsv.build_reference) and the
gate threshold of the config is applied. Also prints the real-vs-render hue of the mug.
    python objpose/tools/prompt_probe/mug_hue_sweep.py <owners.json> <SAM session>
"""
import json, os, sys
from pathlib import Path
import cv2, numpy as np, yaml
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc")); sys.path.insert(0, str(REPO / "sam6d_realtime"))
os.chdir(REPO / "sam6d_realtime")
import ism_hsv, yolo_ism as yi
from conv_session import ConvSession
owners, session = sys.argv[1], sys.argv[2]
rows = json.load(open(owners))["rows"]
cfg = yaml.safe_load(open("configs/yolo_ism_objects.yaml")); thr = float(cfg["defaults"]["hsv_gate_threshold"])
tdir = next(o["template_dir"] for o in cfg["objects"] if o["name"] == "Mugcup_high")
seg = yi.build_segmentor(cfg["defaults"].get("seg_weights", "mobile_sam.pt"), "cuda:0")
sess = ConvSession(session)
pos = [r for r in rows if r["pink"] and r["owner"] == "Mugcup_high"]
neg = [r for r in rows if not r["pink"]]
cache = {}
def sample(r):
    k = (r["i"], tuple(r["box"]))
    if k not in cache:
        bgr = sess.read(r["i"], False).color_bgr
        m = yi.segment_box(seg, bgr, r["box"], "cuda:0")
        cache[k] = (bgr, m)
    return cache[k]
# real mug hue (masked pixels of positive boxes) vs render hue (template rgb masked)
hr = []
for r in pos:
    bgr, m = sample(r)
    if m is None: continue
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); sel = m.astype(bool) & (hsv[..., 1] > 25)
    hr.append(hsv[..., 0][sel])
hr = np.concatenate(hr) if hr else np.array([])
ht = []
for k in range(0, 42, 3):
    rgb = cv2.imread(f"{tdir}/rgb_{k}.png"); mk = cv2.imread(f"{tdir}/mask_{k}.png", 0)
    if rgb is None or mk is None: continue
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV); sel = (mk > 0) & (hsv[..., 1] > 25); ht.append(hsv[..., 0][sel])
ht = np.concatenate(ht)
circ = lambda h: (np.degrees(np.angle(np.mean(np.exp(1j * np.radians(h.astype(float) * 2))))) % 360) / 2
print(f"hue (OpenCV 0-180) circular mean: real mug {circ(hr):.1f}  render {circ(ht):.1f}  (n real px {hr.size})")
print(f"positives {len(pos)}  negatives {len(neg)}  gate threshold {thr}")
res = {}
for hc in range(-15, 16, 3):
    proto = ism_hsv.build_reference(tdir, hue_correction_ocv=hc)
    if isinstance(proto, tuple): proto = proto[0]
    sp = [ism_hsv.shadow_score(sample(r)[0], r["box"], sample(r)[1], proto) for r in pos if sample(r)[1] is not None]
    sn = [ism_hsv.shadow_score(sample(r)[0], r["box"], sample(r)[1], proto) for r in neg if sample(r)[1] is not None]
    tp = sum(s >= thr for s in sp); fp = sum(s >= thr for s in sn)
    res[hc] = {"pos_pass": tp, "pos_n": len(sp), "pos_median": round(float(np.median(sp)), 4), "neg_pass": fp, "neg_n": len(sn)}
    print(f"hc {hc:+3d}: mug boxes pass {tp:3d}/{len(sp)} (median {np.median(sp):.3f})   non-mug boxes pass {fp:3d}/{len(sn)}")
json.dump({"real_hue": circ(hr), "render_hue": circ(ht), "thr": thr, "sweep": res}, open(Path(owners).with_name("hue_sweep.json"), "w"), indent=1)
