#!/usr/bin/env python3
"""Scan frames near the figure frame for candidates rejected by the appearance or colour
gate (ISM replay only, same code path as run_real_stage.py). Writes scan_rejects.json and,
for the best hit, crop_reject_alt.png / crop_reject_alt_mask.png / crop_reject_alt.json."""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_real_stage as R  # noqa: E402

lo, hi = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (1748, 1868)
cfg = yaml.safe_load(open(R.RUN / "sam6d_config.yaml"))
rt = cfg["runtime"]
core = R.Sam6DCore(cfg["ism"]["config"], cfg["ism"]["objects"], rt["device"], rt["det_score_thresh"],
                   verify=rt["verify"])
sess = R.ConvSession(R.DATASET, offset_ns=2_000_000)
hits = []
best = None
for i in range(lo, hi + 1, 2):
    fr = sess.read(i, with_depth=False)
    _, cands, _ = R.ism_replay(core, fr.color_bgr)
    for c in cands:
        if c["rejected_by"] in ("2_appearance", "3_color_hsv") and c["_mask"] is not None:
            rec = {"frame": i, **{k: v for k, v in c.items() if not k.startswith("_")}}
            hits.append(rec)
            key = (0 if c["rejected_by"] == "3_color_hsv" else 1, abs(i - 1808), -c["mask_px"])
            if best is None or key < best[0]:
                best = (key, rec, fr.color_bgr.copy(), c["_mask"].copy())
json.dump({"range": [lo, hi], "step": 2, "hits": hits}, open(R.OUT / "scan_rejects.json", "w"), indent=1)
print(f"{len(hits)} appearance/colour rejects in frames {lo}..{hi}")
for h in hits:
    print(h["frame"], h["owner"], h["rejected_by"], h["semantic"], h["appearance"], h["color_hsv"], h["box"])
if best:
    _, rec, bgr, m = best
    x1, y1, x2, y2 = R.tight(m, 6, bgr.shape)
    cv2.imwrite(str(R.OUT / "crop_reject_alt.png"), bgr[y1:y2, x1:x2])
    cv2.imwrite(str(R.OUT / "crop_reject_alt_mask.png"), (m[y1:y2, x1:x2] * 255).astype(np.uint8))
    json.dump(rec, open(R.OUT / "crop_reject_alt.json", "w"), indent=1)
    print("chosen", rec["frame"], rec["owner"], rec["rejected_by"])
