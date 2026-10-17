#!/usr/bin/env python3
"""display_jumps.py — does the live screen show the correction as steps? From a hub run's display_objects.jsonl (15 Hz):
every drawn box is put back into the map with the SLAM pose the hub received for that moment (uncorrected, T_wc . X).
Camera motion cancels; what moves is (a) the object memory updating the object, (b) the object-anchored correction.
Counts frame-to-frame moves of those map positions above 5 / 20 mm per minute of display.
    python display_jumps.py --x <X json> <run dir> [...]
"""
import argparse, bisect, json
from pathlib import Path
import numpy as np

ap = argparse.ArgumentParser(); ap.add_argument("--x", required=True); ap.add_argument("runs", nargs="+"); a = ap.parse_args()
X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))
from fusion import PoseBuffer                                    # noqa: E402

for d in a.runs:
    d = Path(d)
    delay = int(json.loads((d / "summary.json").read_text()).get("display_delay_s", 0.3) * 1e9)
    ev = [(int(m["t_ns"]), 0, m) for m in map(json.loads, open(d / "slam_poses.jsonl"))]
    ev += [(int(m["t_ns"]), 1, m) for m in map(json.loads, open(d / "kf_updates.jsonl"))]
    ev.sort(key=lambda e: (e[0], e[1]))
    pb = PoseBuffer(0.25); ei = 0
    rows = [json.loads(l) for l in open(d / "display_objects.jsonl")]
    prev, mv = None, []
    for k, r in enumerate(rows):
        t = int(r["t_ns"])
        while ei < len(ev) and ev[ei][0] <= t + delay:        # what the hub had received when it drew frame t
            _, kind, m = ev[ei]; ei += 1
            if kind == 0:
                if m.get("T_wc") is not None and m.get("state") in ("OK", "OK_KLT"):
                    pb.add(int(m["t_ns"]), m["state"], m["T_wc"], m.get("map_id"), kf=m.get("ref_kf"), T_w_kf=m.get("T_w_kf"),
                           kf_map_id=m.get("kf_map_id"), map_changed=bool(m.get("map_changed")))
            else:
                pb.apply_kf_update(m.get("map_id"), m.get("kfs", []))
        T, _ = pb.at(t)
        if T is None:
            prev = None
            continue
        Tws = T @ X
        cur = [(o["name"], (Tws @ np.asarray(o["T_cam_obj"], float).reshape(4, 4))[:3, 3]) for o in r["objects"] if o.get("T_cam_obj") is not None]
        if prev is not None:
            for nm, p in cur:
                c = [q for n2, q in prev if n2 == nm]
                if c:
                    mv.append(min(np.linalg.norm(p - q) for q in c) * 1000)   # same box in the previous frame = nearest one
        prev = cur
    mv = np.array(mv)
    minutes = (rows[-1]["t_ns"] - rows[0]["t_ns"]) / 60e9
    print(f"{d.name}: moves >5 mm {(mv > 5).sum() / minutes:6.1f}/min, >20 mm {(mv > 20).sum() / minutes:5.1f}/min, p99 {np.percentile(mv, 99):5.1f} mm, max {mv.max():6.1f}")
