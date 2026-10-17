#!/usr/bin/env python3
"""tier_filter.py <run_dir> <out_dir> — keep only answers that passed both checks (score >= 3:
candidate gate and pose verification), i.e. what the map and the screen are fed in score mode."""
import json, sys
from pathlib import Path
src, dst = Path(sys.argv[1]), Path(sys.argv[2])
for sc in sorted(p for p in src.iterdir() if p.is_dir()):
    (dst / sc.name).mkdir(parents=True, exist_ok=True)
    rows = []
    for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
        if not l.strip(): continue
        f = json.loads(l); f["dets"] = [d for d in f["dets"] if d.get("score", 0) >= 3.0]; rows.append(json.dumps(f))
    (dst / sc.name / "sam6d_frames.jsonl").write_text("\n".join(rows) + "\n")
print("ok", dst)
