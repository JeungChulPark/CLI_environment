#!/usr/bin/env python3
"""objlist_combo.py — ObChange 방문마다 각 방법의 최종 지도에 등록된 객체 목록을 뽑고, 객체 하나하나를 정답과 맞춰 본다(combo_obchange_261009 용).

채점 방식은 inst_iou_obchange.py 와 같다(관측 점을 2 cm 안의 가장 가까운 지도 객체에 배정, 정답 라벨과의 겹침 비율). 그 함수를 그대로 불러 쓴다.
  등록 객체마다   이름, 중심 위치, 방향(자세를 저장하는 방법만), 관측 횟수, 가장 많이 겹치는 정답 물체와 겹침 비율,
                  판정: 맞음(같은 이름 정답과 겹침 0.25 이상이고 그 정답의 대표 객체) / 중복(같은 이름 정답과 0.02 이상 겹치지만 대표가 아님: 두 번 올라갔거나 일부만 잡힘) /
                        다른 물체 자리(다른 이름 정답과 0.25 이상 겹침) / 정답 없는 자리
  정답 물체마다   방법별로 맞는 객체가 있는지와 겹침 비율
    ~/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/objlist_combo.py --root objpose/output/combo_obchange_261009 --out <json>
"""
import argparse, json, os, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent; sys.path.insert(0, str(HERE))
import inst_iou_obchange as S   # noqa: E402

POSE = ["A", "B", "C_on", "C_off", "D", "E_on", "E_off"]
ap = argparse.ArgumentParser(); ap.add_argument("--root", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--rooms", nargs="+", default=["office", "livingarea", "smallroom"]); ap.add_argument("--tau", type=float, default=0.02); ap.add_argument("--only", nargs="*", default=None)
a = ap.parse_args(); R = a.root.resolve(); TAG = "combo_rt_261009"
PTS = S.PTS; YCBV = S.YCBV


def load(method, sc):
    """→ [(이름, 점, 부가 정보)] 또는 None(결과 없음)"""
    if method in POSE:
        f = R / method / "maps/proposed" / f"{sc}.json"
        if not f.exists(): return None
        out = []
        for o in json.loads(f.read_text()):
            M = np.load(PTS / f"{YCBV[o['name']]}.npy") / 1000.0; Rm = np.asarray(o["R"]); out.append((o["name"], M @ Rm.T + np.asarray(o["center"]), {"center": o["center"], "R": o["R"], "n_obs": o["n_obs"]}))
        return out
    fm = method.startswith("FM-Fusion"); cad = method.endswith("+ CAD")
    if fm:
        sd = R / "fmfusion_rt" / sc
        if not (sd / "instance_info.txt").exists() or (cad and not (R / "cad_rt/fmf_names.json").exists()): return None
        ren = json.loads((R / "cad_rt/fmf_names.json").read_text()).get(sc) if cad else None
        if cad and ren is None: return None
        out = []
        for line in (sd / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip(): continue
            f = line.split(";"); Q = S.read_ply_xyz(sd / f"{int(f[0]):04d}.ply")
            if len(Q): out.append((ren.get(str(int(f[0]))) if cad else S.BY_LABEL.get(f[1].split("(")[0]), Q, {"center": Q.mean(0).tolist(), "n_obs": int(f[2]) if len(f) > 2 and f[2].strip().lstrip("-").isdigit() else None}))
        return out
    f = (R / "cad_rt/dualmap" if cad else R / "dualmap_rt/maps" / TAG) / f"{sc}.json"
    if not f.exists(): return None
    return [(o["name"], np.asarray(o["pts"], float), {"center": o["center"], "n_obs": o.get("n_obs")}) for o in json.loads(f.read_text()) if len(o["pts"])]


METHODS = POSE + ["FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"]
if a.only: METHODS = [m for m in METHODS if m in a.only]
out = []
for room in a.rooms:
    for n in (2, 3, 4, 5, 6):
        sc = f"{room}_s{n}"; P, glab, gn = S.scene_gt(sc); gsz = np.bincount(glab[glab >= 0], minlength=len(gn)); gcen = [P[glab == i].mean(0).tolist() if gsz[i] else None for i in range(len(gn))]
        rec = {"scene": sc, "gt": [{"name": g, "points": int(gsz[i]), "center": gcen[i], "target": g in S.CATALOG} for i, g in enumerate(gn)], "methods": {}}
        for m in METHODS:
            its = load(m, sc)
            if its is None: continue
            plab = S.assign(P, [c for _, c, _ in its], a.tau); psz = np.bincount(plab[plab >= 0], minlength=len(its))
            pair = np.zeros((len(gn), len(its)), int); both = (glab >= 0) & (plab >= 0); np.add.at(pair, (glab[both], plab[both]), 1)
            iou = pair / np.maximum(gsz[:, None] + psz[None, :] - pair, 1)
            rep = {}                                                      # 정답 i → 같은 이름 객체 중 겹침이 가장 큰 것
            for i, g in enumerate(gn):
                same = [j for j, (nm, _, _) in enumerate(its) if nm == g]
                if same:
                    j = same[int(iou[i, same].argmax())]
                    if iou[i, j] >= 0.25: rep[i] = j
            objs = []
            for j, (nm, _, info) in enumerate(its):
                bi = int(iou[:, j].argmax()) if len(gn) else -1; bv = float(iou[bi, j]) if bi >= 0 else 0.0
                if nm not in S.CATALOG: verdict = "이름 없음"
                elif j in rep.values(): verdict = "맞음"
                elif any(gn[i] == nm and iou[i, j] >= 0.02 for i in range(len(gn))): verdict = "중복"
                elif bv >= 0.25: verdict = "다른 물체 자리"
                else: verdict = "정답 없는 자리"
                Pj = its[j][1]; sub = Pj[np.random.default_rng(0).choice(len(Pj), min(len(Pj), 160), replace=False)]
                objs.append({"pts": np.round(sub, 3).tolist(), "cad": "R" in info, "name": nm, "verdict": verdict, "gt_best": gn[bi] if bi >= 0 and bv > 0 else None, "iou_best": round(bv, 3), "points": int(psz[j]), **info})
            rec["methods"][m] = {"objects": objs, "gt": [{"name": g, "ok": i in rep, "iou_name": round(float(max([iou[i, j] for j, (nm, _, _) in enumerate(its) if nm == g], default=0.0)), 3),
                                                             "iou_any": round(float(iou[i].max()) if len(its) else 0.0, 3), "any_name": its[int(iou[i].argmax())][0] if len(its) else None} for i, g in enumerate(gn)]}
        out.append(rec); print(sc, {m: (sum(o["verdict"] == "맞음" for o in v["objects"]), sum(o["name"] in S.CATALOG for o in v["objects"])) for m, v in rec["methods"].items()}, flush=True)
a.out.parent.mkdir(parents=True, exist_ok=True); a.out.write_text(json.dumps(out, ensure_ascii=False))
