#!/usr/bin/env python3
"""FM-Fusion 출력(<run>/<scene>/instance_info.txt + NNNN.ply)을 change_eval 의 지도 형식으로 옮긴다.
이름 = 라벨('sugar box') → YCB 번호 이름, 중심 = 점군 축 정렬 상자 중심(1–99 백분위).
    python3 objpose/tools/obchange/fmfusion_to_maps.py <run> <out>/maps/fmfusion"""
import json, sys
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/baselines/fmfusion"))


def read_ply_xyz(p):
    raw = p.read_bytes(); end = raw.index(b"end_header\n") + len(b"end_header\n"); head = raw[:end].decode()
    n = int(next(l.split()[-1] for l in head.splitlines() if l.startswith("element vertex")))
    props = [l.split() for l in head.splitlines() if l.startswith("property")]
    dt = np.dtype([(pr[2], {"double": "<f8", "float": "<f4", "uchar": "u1"}[pr[1]]) for pr in props])
    v = np.frombuffer(raw[end:end + n * dt.itemsize], dt)
    return np.stack([v["x"], v["y"], v["z"]], 1).astype(float)


run, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
tab = json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]
by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["ycb"] for o in tab}
for sd in sorted(p for p in run.iterdir() if p.is_dir() and (p / "instance_info.txt").exists()):
    objs = []
    for line in (sd / "instance_info.txt").read_text().splitlines():
        if line.startswith("#") or not line.strip(): continue
        f = line.split(";"); lab = f[1].split("(")[0]
        P = read_ply_xyz(sd / f"{int(f[0]):04d}.ply")
        if len(P) == 0: continue
        lo, hi = np.percentile(P, 1, 0), np.percentile(P, 99, 0)
        objs.append({"name": by_label.get(lab), "label": lab, "center": ((lo + hi) / 2).tolist(), "extent": (hi - lo).tolist(),
                     "n_obs": int(f[2]), "n_pts": int(len(P)),
                     "pts": np.round(P[np.random.default_rng(0).choice(len(P), min(len(P), 600), replace=False)], 3).tolist()})
    (out / f"{sd.name}.json").write_text(json.dumps(objs, indent=1))
    print(sd.name, len(objs), [o["label"] for o in objs])
