#!/usr/bin/env python3
"""pose_metrics.py — 최종 지도에 저장된 6D 자세의 정확도(자세를 저장하는 제안 방법만). eval_maps_f1.py --dump 의 장면 json 과 inst_iou json 을 읽는다.

대상 = TP 객체(정답과 같은 이름이고 관측 점 겹침 비율 0.25 이상). 정답마다 같은 이름의 가장 가까운 지도 객체를 쓴다.
  ADD      같은 CAD 점끼리의 평균 거리 [mm]
  ADD-S    추정 자세의 CAD 점에서 정답 자세의 CAD 점까지 최근접 거리의 평균 [mm]
  ADD(-S)  대칭 물체(BOP models_info 의 symmetries_*)는 ADD-S, 나머지는 ADD.  통과 = ADD(-S) < 0.1 × 물체 지름
    <sam6d python> objpose/tools/analysis/pose_metrics.py --dump <dump dir> --inst <inst_iou json> --out <json>
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM   # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument("--dump", type=Path, required=True); ap.add_argument("--inst", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--method", default="제안 방법"); a = ap.parse_args()
tab = json.loads(BM.TABLE.read_text()); info = json.loads((Path(tab["models_dir"]) / "models_info.json").read_text())
oid = {o["name"]: str(o["obj_id"]) for o in tab["objects"]}; model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in tab["objects"]}
sym = {n for n, i in oid.items() if "symmetries_discrete" in info[i] or "symmetries_continuous" in info[i]}
inst = json.loads(a.inst.read_text()); tp = {(r["scene"], r["object"]) for r in inst if r["object"] and r["method"] == a.method and r["iou_name"] >= 0.25}
n_gt = sum(1 for r in inst if r["object"] and r["method"] == a.method); rows = []
for f in sorted(a.dump.glob("*.json")):
    D = json.loads(f.read_text())
    for g in D["gt"]:
        if (D["scene"], g["name"]) not in tp: continue
        T = np.asarray(g["T"], float); G = model[g["name"]] @ T[:3, :3].T + T[:3, 3]
        c = [o for o in D["methods"].get(a.method, []) if o["name"] == g["name"] and o["R"] is not None]
        if not c: continue
        o = min(c, key=lambda o_: np.linalg.norm(np.asarray(o_["center"]) - np.asarray(g["center"]))); P = np.asarray(o["pts"], float)
        if len(P) != len(G): continue
        add = float(np.linalg.norm(P - G, axis=1).mean() * 1000); adds = float(cKDTree(G).query(P)[0].mean() * 1000); dia = info[oid[g["name"]]]["diameter"]
        e = adds if g["name"] in sym else add
        rows.append({"scene": D["scene"], "object": g["name"], "add_mm": add, "adds_mm": adds, "add_s_mm": e, "pass": e < 0.1 * dia, "rot_deg": o.get("rot_deg"), "center_mm": o.get("center_mm")})
q = lambda k: float(np.median([r[k] for r in rows if r[k] is not None]))
S = {"n_gt": n_gt, "n_tp": len(tp), "n_scored": len(rows), "add_mm_p50": q("add_mm"), "adds_mm_p50": q("adds_mm"), "add_s_mm_p50": q("add_s_mm"), "rot_deg_p50": q("rot_deg"), "center_mm_p50": q("center_mm"),
     "add_s_pass_of_tp": 100 * sum(r["pass"] for r in rows) / max(len(rows), 1), "add_s_pass_of_gt": 100 * sum(r["pass"] for r in rows) / n_gt, "symmetric": sorted(sym)}
a.out.write_text(json.dumps({"summary": S, "rows": rows}, ensure_ascii=False)); print({k: (round(v, 1) if isinstance(v, float) else v) for k, v in S.items() if k != "symmetric"}); print("대칭:", [s[5:] for s in sorted(sym)])
