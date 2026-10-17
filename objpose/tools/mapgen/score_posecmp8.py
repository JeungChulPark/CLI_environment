#!/usr/bin/env python3
"""score_posecmp8.py — posecmp8.py 결과의 프레임별 6D 포즈 채점(8 구성, 같은 프레임).

대응: 프레임마다 물체 이름별로, 그 이름의 출력과 같은 물체의 정답 인스턴스를 ADD-S 가 가장 작아지도록 일대일로 짝짓는다(헝가리안).
  성공      짝지어진 출력의 ADD-S < 0.1 × 지름(BOP models_eval 의 지름)
  성공률    성공 / 전체 정답 인스턴스(보이는 비율 제한 없음). 출력이 없거나(판정 탈락 · PEM 입력 부족 · 포즈 검증 탈락) 짝이 없으면 실패
  오답      성공이 아닌 출력 = 짝 없는 출력(그 프레임에 없는 물체) + 짝은 있으나 ADD-S ≥ 0.1 d. 프레임당 오답 = 오답 / 프레임 수
  오차      짝지어진 출력 전부의 ADD-S 와 위치 오차 ‖t̂ − t‖ 의 중앙값(성공한 것만 고르지 않는다). ADD(-S)는 쓰지 않는다
ADD-S 는 models_eval 의 꼭짓점 전부로 계산한다.
    python3 objpose/tools/mapgen/score_posecmp8.py <ycbv|ycbm> <결과 폴더> <출력 폴더>
출력  <출력>/eval_<ds>.csv (정답 · 출력 한 줄씩, 탈락 사유 포함), summary_<ds>.csv, summary_<ds>.json, flips_<ds>.csv (구성 쌍의 개선 · 악화)
"""
import csv, json, sys
from pathlib import Path
import numpy as np, trimesh
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video")
SCN = {"ycbv": YCBV / "test", "ycbm": Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop/test")}
CFGS = ["FT0", "FT1", "FS0", "FS1", "YT0", "YT1", "YS0", "YS1"]
LABEL = {"FT": "FastSAM-x + 총점", "FS": "FastSAM-x + 단계별 기준", "YT": "YOLO-World+MobileSAM + 총점", "YS": "YOLO-World+MobileSAM + 단계별 기준",
         "YA": "YOLO 상자를 모든 물체 후보로 + 단계별 기준", "YR": "YOLO-World+MobileSAM + 단계별 기준(완화)", "YB": "YOLO 상자를 모든 물체 후보로 + 단계별 기준(완화)"}
ds, run, out = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3]); out.mkdir(parents=True, exist_ok=True)
CFGS = list(json.loads(next(iter(sorted(run.glob("*.jsonl")))).read_text().splitlines()[0])["out"])     # --variants 결과면 YT1 · YS1 · YA1 · YR1 · YB1
tab = json.loads((REPO / "objpose/rt/bop" / f"{ds}_objects.json").read_text()); name2id = {o["name"]: o["obj_id"] for o in tab["objects"]}; id2name = {v: k for k, v in name2id.items()}
info = {int(k): v for k, v in json.loads((YCBV / "models_eval/models_info.json").read_text()).items()}; DIAM = {k: v["diameter"] for k, v in info.items()}
PTS = {k: np.asarray(trimesh.load(YCBV / "models_eval" / f"obj_{k:06d}.ply", process=False).vertices) for k in DIAM}
TREE = {}


def adds(oid, Re, te, Rg, tg):                       # 추정 자세의 모델 점을 정답 물체 좌표계로 옮겨 모델 점과의 최근접 거리 평균(= ADD-S)
    if oid not in TREE: TREE[oid] = cKDTree(PTS[oid])
    A = (PTS[oid] @ Re.T + te - tg) @ Rg
    return float(TREE[oid].query(A)[0].mean())


rows = []; stat = {c: dict(frames=0, gt=0, gt_v10=0, success=0, success_v10=0, outputs=0, matched=0, wrong=0, wrong_absent=0, wrong_pose=0, adds=[], terr=[]) for c in CFGS}
gt_ok = {c: {} for c in CFGS}
for f in sorted(run.glob("*.jsonl")):
    s = f.stem; gt = json.loads((SCN[ds] / s / "scene_gt.json").read_text()); gi = json.loads((SCN[ds] / s / "scene_gt_info.json").read_text())
    for l in f.read_text().splitlines():
        r = json.loads(l); im = r["im_id"]; G = [(k, g["obj_id"], np.asarray(g["cam_R_m2c"]).reshape(3, 3), np.asarray(g["cam_t_m2c"]).reshape(3), gi[str(im)][k]["visib_fract"]) for k, g in enumerate(gt[str(im)])]
        for c in CFGS:
            st = stat[c]; st["frames"] += 1; outs = r["out"][c]; judge = r["judge"][c[:2]]
            given = [o for o in outs if o["output"]]; st["outputs"] += len(given)
            gstat = {}
            for oid in sorted({g[1] for g in G} | {name2id[o["object"]] for o in given}):
                gs = [g for g in G if g[1] == oid]; os_ = [o for o in given if name2id[o["object"]] == oid]
                pairs = []
                if gs and os_:
                    C = np.array([[adds(oid, np.asarray(o["R"]), np.asarray(o["t_mm"]), g[2], g[3]) for g in gs] for o in os_]); oi, gj = linear_sum_assignment(C)
                    pairs = [(os_[i], gs[j], C[i, j]) for i, j in zip(oi, gj)]
                mo = {id(p[0]) for p in pairs}; mg = {p[1][0] for p in pairs}
                for o, g, e in pairs:
                    te = float(np.linalg.norm(np.asarray(o["t_mm"]) - g[3])); ok = e < 0.1 * DIAM[oid]
                    st["matched"] += 1; st["adds"].append(e); st["terr"].append(te)
                    if not ok: st["wrong"] += 1; st["wrong_pose"] += 1
                    gstat[g[0]] = (ok, e, te, "success" if ok else "pose-error>=0.1d")
                    rows.append([ds, s, im, c, "output", o["object"], g[0], round(g[4], 3), 1, round(e, 2), round(te, 2), int(ok), "" if ok else "pose-error>=0.1d", o.get("score")])
                for o in os_:
                    if id(o) in mo: continue
                    st["wrong"] += 1; st["wrong_absent"] += 1
                    rows.append([ds, s, im, c, "output", o["object"], "", "", 0, "", "", 0, "object-not-in-frame" if not gs else "extra-output", o.get("score")])
                for g in gs:
                    if g[0] in mg: continue
                    nm = id2name.get(oid); o = next((x for x in outs if x["object"] == nm), None)
                    why = (o.get("reason") or "unmatched") if o is not None and not o["output"] else ("duplicate-instance" if o is not None else (judge.get(nm) or {}).get("decision", "not-in-object-list"))
                    gstat[g[0]] = (False, None, None, why)
            for g in G:
                ok, e, te, why = gstat[g[0]]; st["gt"] += 1; st["success"] += ok; v = g[4] >= 0.1; st["gt_v10"] += v; st["success_v10"] += ok and v
                gt_ok[c][(s, im, g[0])] = ok
                rows.append([ds, s, im, c, "gt", id2name.get(g[1], g[1]), g[0], round(g[4], 3), int(e is not None), "" if e is None else round(e, 2), "" if te is None else round(te, 2), int(ok), why, ""])
with open(out / f"eval_{ds}.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["dataset", "scene", "im_id", "config", "kind", "object", "gt_index", "visib_fract", "matched", "adds_mm", "t_err_mm", "success", "reason", "score"]); w.writerows(rows)
summ = {}
for c, st in stat.items():
    a_, t_ = np.asarray(st.pop("adds")), np.asarray(st.pop("terr"))
    summ[c] = dict(config=c, label=LABEL[c[:2]] + (" + 포즈 검증" if c[2] == "1" else ""), **st, success_rate=100 * st["success"] / st["gt"], success_rate_v10=100 * st["success_v10"] / max(st["gt_v10"], 1),
                   wrong_per_frame=st["wrong"] / st["frames"], precision=100 * (st["outputs"] - st["wrong"]) / max(st["outputs"], 1),
                   adds_median=float(np.median(a_)) if len(a_) else None, terr_median=float(np.median(t_)) if len(t_) else None)
with open(out / f"summary_{ds}.csv", "w", newline="") as fh:
    keys = list(next(iter(summ.values())).keys()); w = csv.DictWriter(fh, keys); w.writeheader(); [w.writerow(v) for v in summ.values()]
(out / f"summary_{ds}.json").write_text(json.dumps(summ, indent=1, ensure_ascii=False, default=lambda x: x.item()))
PAIRS = [(f"{g}{j}0", f"{g}{j}1", "포즈 검증 켬") for g in "FY" for j in "TS"] + [(f"{g}T{v}", f"{g}S{v}", "총점→단계별 기준") for g in "FY" for v in "01"] + [("FT1", "YS1", "① → 제안"), ("YT1", "YS1", "⑤ → 제안")]
with open(out / f"flips_{ds}.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["from", "to", "change", "gt_improved(fail->success)", "gt_worsened(success->fail)", "wrong_from", "wrong_to"])
    PAIRS = [p for p in PAIRS if p[0] in stat and p[1] in stat] + [("YS1", y, "변형") for y in ("YA1", "YR1", "YB1") if y in stat]
    for x, y, lab in PAIRS:
        imp = sum(1 for k, v in gt_ok[y].items() if v and not gt_ok[x][k]); wor = sum(1 for k, v in gt_ok[y].items() if not v and gt_ok[x][k])
        w.writerow([x, y, lab, imp, wor, stat[x]["wrong"], stat[y]["wrong"]])
for c, v in summ.items():
    print(ds, c, f"성공률 {v['success_rate']:.1f}% (보임≥0.1: {v['success_rate_v10']:.1f}%)  프레임당 오답 {v['wrong_per_frame']:.3f}  ADD-S 중앙값 {v['adds_median']:.1f} mm  위치 오차 중앙값 {v['terr_median']:.1f} mm  출력 {v['outputs']} 짝 {v['matched']}")
