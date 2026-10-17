#!/usr/bin/env python3
"""score_still.py — still_bop.py 결과(구성마다 같은 프레임)의 6D 포즈 지표. 계산은 score_estcmp.py 와 같고 ADD(-S)를 더했다.

프레임마다 물체당 답 하나(점수가 가장 높은 것)를 그 프레임의 정답(BOP scene_gt, 보이는 비율 0.1 이상)과 비교한다.
  recall      정답 인스턴스 중 ADD-S < 지름 10 % 인 답이 있는 비율
  precision   답 중 ADD-S < 지름 10 % 인 비율 (나머지 = 없는 물체를 답했거나 포즈가 틀림)
  wrong_pf    프레임당 틀린 답 수
  adds_auc    ADD-S AUC (0~10 cm, 답이 없으면 무한대)            — PoseCNN 의 YCB-Video 지표
  add_s_auc   ADD(-S) AUC (대칭 물체 ADD-S, 나머지 ADD)          — 같은 방식
  add_s_10    ADD(-S) < 지름 10 % 인 정답 인스턴스 비율
  adds_med    맞힌 답의 ADD-S 중앙값 (mm)
    python3 objpose/tools/mapgen/score_still.py <데이터셋 ycbv|ycbm> <이름=결과 폴더> … [--out scores.json]
"""
import json, sys
from pathlib import Path
import numpy as np, trimesh
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video")
DS = {"ycbv": ("ycbv_objects.json", YCBV / "test"), "ycbm": ("ycbm_objects.json", Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop/test"))}
_PTS = {}


def pts(table, oid):
    if oid not in _PTS:
        v = np.asarray(trimesh.load(Path(table["models_dir"]) / f"obj_{oid:06d}.ply", process=False).vertices)
        _PTS[oid] = v[np.random.default_rng(0).choice(len(v), min(2000, len(v)), replace=False)]
    return _PTS[oid]


def score(ds, root: Path):
    tname, scenes = DS[ds]; table = json.loads((REPO / "objpose/rt/bop" / tname).read_text())
    name2id = {o["name"]: o["obj_id"] for o in table["objects"]}
    info = {int(k): v for k, v in json.loads((YCBV / "models_eval/models_info.json").read_text()).items()}
    diam = {k: v["diameter"] for k, v in info.items()}; sym = {k for k, v in info.items() if v.get("symmetries_discrete") or v.get("symmetries_continuous")}
    c = dict(inst=0, found=0, answers=0, correct=0, add10=0); e_s, e_a, ok_adds, ms, nf = [], [], [], [], 0
    for sc in sorted(p for p in root.iterdir() if (p / "sam6d_frames.jsonl").exists()):
        gt = json.loads((scenes / sc.name / "scene_gt.json").read_text()); gi = json.loads((scenes / sc.name / "scene_gt_info.json").read_text())
        for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
            f = json.loads(l); fid = str(f["im_id"]); nf += 1; ms.append(f["ms"]["total"]); best = {}
            for d in f["dets"]:
                if d["object"] not in best or d["score"] > best[d["object"]]["score"]: best[d["object"]] = d
            gts = {}
            for r, i in zip(gt.get(fid, []), gi.get(fid, [])):
                gts.setdefault(r["obj_id"], []).append((np.asarray(r["cam_R_m2c"]).reshape(3, 3), np.asarray(r["cam_t_m2c"]).reshape(3), i["visib_fract"]))
            es_obj, ea_obj = {}, {}
            for n, d in best.items():
                oid = name2id[n]; c["answers"] += 1; P = pts(table, oid); Re, te = np.asarray(d["R"]).reshape(3, 3), np.asarray(d["t_mm"]).reshape(3)
                best_s, best_a = np.inf, np.inf
                for Rg, tg, _ in gts.get(oid, []):
                    A = P @ Re.T + te; B = P @ Rg.T + tg; s_ = float(cKDTree(B).query(A)[0].mean()); a_ = s_ if oid in sym else float(np.linalg.norm(A - B, axis=1).mean())
                    best_s, best_a = min(best_s, s_), min(best_a, a_)
                es_obj[oid], ea_obj[oid] = best_s, best_a
                if best_s < 0.1 * diam[oid]: c["correct"] += 1; ok_adds.append(best_s)
            for oid, lst in gts.items():
                if oid not in diam: continue
                for _, _, vf in lst:
                    if vf < 0.1: continue
                    c["inst"] += 1; s_ = es_obj.get(oid, np.inf); a_ = ea_obj.get(oid, np.inf)
                    c["found"] += s_ < 0.1 * diam[oid]; c["add10"] += a_ < 0.1 * diam[oid]; e_s.append(s_); e_a.append(a_)
    th = np.linspace(0, 100, 1001); auc = lambda e: float(np.mean([(np.asarray(e) < t).mean() for t in th])) * 100
    return dict(frames=nf, inst=c["inst"], answers=c["answers"], recall=100 * c["found"] / max(c["inst"], 1), precision=100 * c["correct"] / max(c["answers"], 1),
                wrong_pf=(c["answers"] - c["correct"]) / max(nf, 1), adds_auc=auc(e_s), add_s_auc=auc(e_a), add_s_10=100 * c["add10"] / max(c["inst"], 1),
                adds_med=float(np.median(ok_adds)) if ok_adds else None, ms_median=float(np.median(ms)))


if __name__ == "__main__":
    args = sys.argv[1:]; out = None
    if "--out" in args: i = args.index("--out"); out = Path(args[i + 1]); del args[i:i + 2]
    ds, res = args[0], {}
    for kv in args[1:]:
        k, v = kv.split("=", 1); res[k] = score(ds, Path(v)); print(ds, k, {a: (round(b, 2) if isinstance(b, float) else b) for a, b in res[k].items()}, flush=True)
    if out: out.write_text(json.dumps(res, indent=1, ensure_ascii=False))
