#!/usr/bin/env python3
"""gt_centers.py — ObChange 정답 물체의 중심을 정답 점 라벨(_GT.anno 의 점 번호 → 재구성 점군)의 축 정렬 상자 중심으로 구해
<bop>/gt/<scene>.json 의 각 물체에 center, extent, n_pts 로 적는다. (.anno 의 4x4 는 원점이 물체 바닥이라 중심이 아니다.)
open3d 가 필요해 dualmap 환경으로 돌린다:  conda run -n dualmap python objpose/tools/obchange/gt_centers.py --room Kitchen"""
import argparse, json, re
from pathlib import Path
import numpy as np, open3d as o3d
OB = Path("/home/jucpark/DeepLearning/Datasets/ObChange")
ap = argparse.ArgumentParser(); ap.add_argument("--room", required=True); a = ap.parse_args()
for g in sorted((OB / "bop" / "gt").glob(f"{a.room.lower()}_s*.json")):
    d = json.loads(g.read_text()); n_s = int(re.findall(r"_s(\d+)", g.stem)[0])
    pts = {}
    pdir = OB / "anno" / a.room / f"scene{n_s}" / "planes"
    for pl in (sorted(pdir.iterdir()) if pdir.exists() else []):
        ga = pl / "merged_plane_clouds_ds002_GT.anno"
        if not ga.exists(): continue
        P = np.asarray(o3d.io.read_point_cloud(str(pl / "merged_plane_clouds_ds002.pcd")).points)
        for line in ga.read_text().splitlines():
            f = line.split()
            if len(f) < 3: continue
            idx = np.array([int(x) for x in f[1:] if x.lstrip("-").isdigit()])
            pts[(pl.name, f[0])] = P[idx[(idx >= 0) & (idx < len(P))]]
    for o in d["objects"]:
        p = pts.get((o["plane"], o["ycb"]))
        if p is None or len(p) == 0:
            o["center"] = None; continue
        lo, hi = np.percentile(p, 1, 0), np.percentile(p, 99, 0)
        o["center"] = ((lo + hi) / 2).tolist(); o["extent"] = (hi - lo).tolist(); o["n_pts"] = int(len(p))
        o["anno_to_center_cm"] = float(np.linalg.norm(np.array(o["T_map_anno"])[:3, 3] - np.array(o["center"])) * 100)
    g.write_text(json.dumps(d, indent=1))
    print(g.stem, [(o["ycb"][4:], None if o["center"] is None else [round(v, 2) for v in o["center"]], o.get("n_pts"), round(o.get("anno_to_center_cm", -1), 1)) for o in d["objects"]])
