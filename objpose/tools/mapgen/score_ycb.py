#!/usr/bin/env python3
"""score_ycb.py — "같은 인식 결과, 다른 지도 생성" 실험의 YCB-Video · YCB-M 채점. ObChange 채점(score_designs.py, score_base_same.py)과 같은 규칙이다.

  관측 점    정답 카메라 자세로 깊이 영상을 쌓은 점(inst_iou.observed). 정답 라벨 = 정답 자세의 CAD 에서 2 cm 안. 지도 배정 = 지도 객체의 점에서 2 cm 안
  맞음(TP)   정답 물체와 같은 이름의 객체가 관측 점 기준 0.25 이상 겹침. 이름이 붙었는데 맞음이 아닌 객체 = 잘못 등록. 이름이 물체표에 없는 객체는 세지 않는다
  자세       6D 자세를 저장하는 방식(M0~M3)의 맞은 객체마다 ADD-S(추정 자세의 CAD 점 → 정답 자세의 CAD 점 최근접 거리 평균) [mm], 위치 오차 [mm]. 통과 = ADD-S < 물체 지름의 10 %
  방식       <root>/designs/c8_<장면>.json 의 M0(단순 누적) · V0(제안) · 1n(합의) · 2(깊이 다듬기), <root>/c8/ext 의 FM-Fusion · DualMap
    BENCH_ROOT=<…/xtion_bop> BENCH_TABLE=<표> <sam6d python> objpose/tools/mapgen/score_ycb.py --root <rep>/ycbm --out <rep>/ycbm/scores.json
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
REPO = Path(__file__).resolve().parents[3]
for p in ("objpose/tools/analysis", "objpose/tools/ycbv_sam6d", "objpose/tools/obchange"): sys.path.insert(0, str(REPO / p))
ap = argparse.ArgumentParser(); ap.add_argument("--root", type=Path, required=True); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--tau", type=float, default=0.02); ap.add_argument("--frames", type=int, default=120); a = ap.parse_args(); sys.argv = sys.argv[:1]
import inst_iou as I; BM = I.BM            # noqa: E402,E702  (BENCH_ROOT / BENCH_TABLE)
from inst_iou_obchange import read_ply_xyz   # noqa: E402
TAB = json.loads(BM.TABLE.read_text()); table = TAB["objects"]; by_id = {o["obj_id"]: o["name"] for o in table}; model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in table}
of_ycb = {o["ycb"]: o["name"] for o in table}; of_words = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in table}
info = json.loads((Path(TAB["models_dir"]) / "models_info.json").read_text()); diam = {o["name"]: float(info[str(o["obj_id"])]["diameter"]) / 1000.0 for o in table}
POSE = {"M0": "M0", "V0": "M1", "1n": "M2", "2": "M3"}; out = []; tag = (a.root / "c8/ext/tag.txt").read_text().strip() if (a.root / "c8/ext/tag.txt").exists() else None
for f in sorted((a.root / "designs").glob("c*_*.json")):
    D = json.loads(f.read_text()); s = D["scene"]; sc = BM.YCBV / s; ids, Twc, gobj, K = BM.gt_scene(sc, by_id); cam = json.loads((sc / "scene_camera.json").read_text())
    P = I.observed(sc, ids, Twc, cam, max(1, len(ids) // a.frames)); gn = list(gobj); gpts = [model[n] @ gobj[n][:3, :3].T + gobj[n][:3, 3] for n in gn]; glab = I.assign(P, gpts, a.tau); gsz = np.bincount(glab[glab >= 0], minlength=len(gn))
    M = {}
    for v, m in POSE.items(): M[m] = [(o["name"], model[o["name"]] @ np.asarray(o["T_w_obj"])[:3, :3].T + np.asarray(o["T_w_obj"])[:3, 3], np.asarray(o["T_w_obj"], float)) for o in D.get(v, [])]
    fd = a.root / "c8/ext/fmfusion" / s
    if (fd / "instance_info.txt").exists():
        its = []
        for line in (fd / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip(): continue
            q = line.split(";"); Q = read_ply_xyz(fd / f"{int(q[0]):04d}.ply")
            if len(Q): its.append((of_words.get(q[1].split("(")[0]), Q, None))
        M["M4"] = its
    dj = a.root / "c8/ext/dualmap/maps" / str(tag) / f"{s}.json"
    if dj.exists(): M["M5"] = [(of_ycb.get(o["name"]), np.asarray(o["pts"], float), None) for o in json.loads(dj.read_text()) if len(o["pts"])]
    for m, items in M.items():
        its = [x for x in items if x[0] is not None]
        if its:
            plab = I.assign(P, [c for _, c, _ in its], a.tau); psz = np.bincount(plab[plab >= 0], minlength=len(its)); pair = np.zeros((len(gn), len(its)), int); both = (glab >= 0) & (plab >= 0); np.add.at(pair, (glab[both], plab[both]), 1)
            iou = pair / np.maximum(gsz[:, None] + psz[None, :] - pair, 1)
        else: iou = np.zeros((len(gn), 0))
        rep = {}
        for i, n in enumerate(gn):
            same = [j for j, x in enumerate(its) if x[0] == n]
            if same:
                j = same[int(iou[i, same].argmax())]
                if iou[i, j] >= 0.25: rep[i] = j
        pose = []
        for i, j in rep.items():
            if its[j][2] is None: continue
            adds = float(cKDTree(gpts[i]).query(its[j][1])[0].mean()); pose.append({"object": gn[i], "adds_mm": round(adds * 1000, 2), "trans_mm": round(float(np.linalg.norm(its[j][2][:3, 3] - gobj[gn[i]][:3, 3])) * 1000, 2), "pass": bool(adds < 0.1 * diam[gn[i]])})
        out.append({"method": m, "scene": s, "gt": len(gn), "named": len(its), "all_objects": len(items), "tp": len(rep), "tp50": int(sum(iou[i, j] >= 0.5 for i, j in rep.items())), "iou_tp": [round(float(iou[i, j]), 3) for i, j in rep.items()], "pose": pose})
    print(s, {m: (r["tp"], r["named"]) for m in M for r in out[-len(M):] if r["method"] == m}, "gt", len(gn), flush=True)
a.out.write_text(json.dumps(out))
