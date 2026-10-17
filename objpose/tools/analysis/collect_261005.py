#!/usr/bin/env python3
"""collect_261005.py — 세 방법(제안 · FM-Fusion · DualMap) 객체 지도 결과를 해석용 표로 모은다. 새로 돌리는 것은 없고 이미 나온 결과만 읽는다.

  objects.csv        물체 한 개가 한 줄. 정답 물체마다 방법별 판정(맞음/이름 틀림/놓침…)과 오차, 정답에 없는 지도 객체도 한 줄씩.
  scenes.csv         장면(방문) × 방법 한 줄. 개수, 정밀도 · 재현율 · F1, 오차 중앙값.
  recognition.csv    제안 방법의 장면별 인식 기록(처리 프레임, 검출 수, 물체별 검출 수 · 점수, 단계별 시간).
  localization.csv   ObChange 방문별 카메라 자세 비교(데이터셋 tf ↔ ORB-SLAM3).
    python3 objpose/tools/analysis/collect_261005.py [--out ~/콜로키움/분석자료]
"""
import argparse, csv, json, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/obchange"))
from change_eval import OB, CATALOG, MATCH, dist, load_gt   # noqa: E402

YCB = {"YCB-Video": REPO / "objpose/output/fmf_cmp_260928", "YCB-M": REPO / "objpose/output/fmf_cmp_ycbm_260929"}
OBO = REPO / "objpose/output/obchange_261005"
ROOMS = ["kitchen", "office", "livingarea", "smallroom"]
OBM = {"제안 방법": "proposed", "FM-Fusion": "fmfusion", "DualMap": "dualmap"}
KOR = {"match": "맞음", "duplicate": "중복", "wrong_name": "이름 틀림", "far": "먼 위치", "missing": "놓침", "false_obj": "없는 물체", "unnamed": "이름 없음"}
r1 = lambda v: "" if v is None else round(float(v), 2)
r3 = lambda v: round(float(v), 3)
med = lambda v: "" if not v else round(float(np.median(v)), 2)


def prf(match, pred, gt):
    p = 100 * match / pred if pred else 0.0; r = 100 * match / gt if gt else 0.0
    return round(p, 1), round(r, 1), round(2 * p * r / (p + r), 1) if p + r else 0.0


def ycb(objs, scenes):
    """eval_maps_f1.py --dump 가 남긴 장면별 json(정답 좌표계의 좌표 포함)을 읽는다. 좌표 단위 m."""
    for ds, root in YCB.items():
        for f in sorted((root / "eval_own/dump").glob("*.json")):
            D = json.loads(f.read_text()); gc = {g["name"]: g["center"] for g in D["gt"]}
            for method, lst in D["methods"].items():
                c = Counter(); ce, se, re = [], [], []; hit = set()
                for o in lst:
                    c[o["kind"]] += 1
                    if o["kind"] == "unnamed": continue
                    if o["kind"] == "match":
                        hit.add(o["name"]); ce.append(o["center_mm"]); se.append(o["surface_mm"]); re += [o["rot_deg"]] if o.get("rot_deg") is not None else []
                    g = gc.get(o["name"]) if o["kind"] != "wrong_name" else None
                    objs.append([ds, D["scene"], method, o["name"] if g else "", o["name"], KOR[o["kind"]], r1(o.get("center_mm")), r1(o.get("surface_mm")), r1(o.get("rot_deg")),
                                 *([r3(v) for v in g] if g else ["", "", ""]), *[r3(v) for v in o["center"]], ""])
                miss = [nm for nm in gc if nm not in hit]
                for nm in miss:
                    objs.append([ds, D["scene"], method, nm, "", KOR["missing"], "", "", "", *[r3(v) for v in gc[nm]], "", "", "", ""])
                pred = c["match"] + c["duplicate"] + c["wrong_name"] + c["far"]
                scenes.append([ds, D["scene"], method, len(gc), pred, c["match"], c["duplicate"], c["wrong_name"], c["far"], len(miss), "", c["unnamed"],
                               *prf(c["match"], pred, len(gc)), med(ce), med(se), med(re)])


def obchange(objs, scenes):
    """map_eval.py 와 같은 규칙(배경 빼기 없음)으로 물체별 줄을 남긴다. 거리 단위는 mm 로 맞춘다."""
    for room in ROOMS:
        for g in sorted((OB / "gt").glob(f"{room}_s*.json")):
            G = load_gt(g.stem, CATALOG)
            if not G: continue
            for method, key in OBM.items():
                f = OBO / "maps" / key / f"{g.stem}.json"
                if not f.exists(): continue
                M = json.loads(f.read_text()); used = set(); c = Counter(); ce = []
                for gt in G:
                    near = sorted((dist(gt["center"], o["center"]), i) for i, o in enumerate(M) if dist(gt["center"], o["center"]) < MATCH)
                    same = [(d, i) for d, i in near if M[i]["name"] == gt["name"]]
                    row = ["ObChange", g.stem, method, gt["name"]]
                    if same:
                        d, i = same[0]; o = M[i]; used.add(i); c["match"] += 1; ce.append(d * 1000)
                        row += [o["name"], KOR["match"], r1(d * 1000), "", "", *[r3(v) for v in gt["center"]], *[r3(v) for v in o["center"]], o.get("n_obs", "")]
                        for _, j in same[1:]:
                            used.add(j); c["duplicate"] += 1
                            objs.append(["ObChange", g.stem, method, gt["name"], M[j]["name"], KOR["duplicate"], r1(_ * 1000), "", "", *[r3(v) for v in gt["center"]], *[r3(v) for v in M[j]["center"]], M[j].get("n_obs", "")])
                    elif near:
                        d, i = near[0]; o = M[i]; c["wrong_name"] += 1
                        row += [o["name"] or f"(이름 없음: {o.get('label', '')})", KOR["wrong_name"], r1(d * 1000), "", "", *[r3(v) for v in gt["center"]], *[r3(v) for v in o["center"]], o.get("n_obs", "")]
                    else:
                        c["missing"] += 1; row += ["", KOR["missing"], "", "", "", *[r3(v) for v in gt["center"]], "", "", "", ""]
                    objs.append(row)
                cat = [i for i, o in enumerate(M) if o["name"] in CATALOG]
                for i in cat:
                    if i in used: continue
                    o = M[i]; c["false_obj"] += 1
                    dg = min((dist(x["center"], o["center"]) for x in G), default=None)
                    objs.append(["ObChange", g.stem, method, "", o["name"], KOR["false_obj"], r1(dg * 1000) if dg is not None else "", "", "", "", "", "", *[r3(v) for v in o["center"]], o.get("n_obs", "")])
                scenes.append(["ObChange", g.stem, method, len(G), len(cat), c["match"], c["duplicate"], c["wrong_name"], "", c["missing"], c["false_obj"], sum(o["name"] is None for o in M),
                               *prf(c["match"], len(cat), len(G)), med(ce), "", ""])


def recognition(rows):
    """제안 방법 실행 기록(sam6d_frames.jsonl · sam6d_estimates.jsonl). 물체별 검출 수와 점수는 '물체=횟수(평균 점수)' 로 적는다."""
    dirs = [("YCB-Video", d) for d in sorted((YCB["YCB-Video"] / "proposed_own").iterdir())] + [("YCB-M", d) for d in sorted((YCB["YCB-M"] / "proposed_own").iterdir())] \
        + [("ObChange", d) for d in sorted((OBO / "proposed").iterdir()) if d.name.split("_")[0] in ROOMS]
    for ds, d in dirs:
        fr, es = d / "sam6d_frames.jsonl", d / "sam6d_estimates.jsonl"
        if not fr.exists(): continue
        F = [json.loads(l) for l in fr.read_text().splitlines() if l.strip()]
        E = [json.loads(l) for l in es.read_text().splitlines() if l.strip()] if es.exists() else []
        run = json.loads((d / "run.json").read_text()) if (d / "run.json").exists() else {}
        per = defaultdict(list)
        for e in E: per[e["object"]].append(e.get("score", 0))
        ms = lambda k: med([f["ms"][k] for f in F if f.get("ms") and k in f["ms"]])
        rows.append([ds, d.name, run.get("frames", ""), len(F), sum(1 for f in F if f.get("dets")), len(E), med([f.get("n_boxes", 0) for f in F]),
                     ms("yolo"), ms("ism"), ms("pem"), ms("total"), med([f["latency_s"] * 1000 for f in F if "latency_s" in f]),
                     "; ".join(f"{k}={len(v)}({np.mean(v):.2f})" for k, v in sorted(per.items()))])


def localization(rows):
    for room in ROOMS:
        f = OBO / "eval" / f"pose_{room}.json"
        if not f.exists(): continue
        for sc, d in json.loads(f.read_text())["scenes"].items():
            rows.append(["ObChange", sc, d["frames"], r1(d["path_tf_m"]), r1(d["path_orb_m"]), r1(d["traj_diff_rmse_cm"]), r1(d["traj_diff_max_cm"]), r1(d["rot_diff_med_deg"]), r1(d["rot_diff_max_deg"]),
                         r1(d.get("obj_err_med_cm_tf")), r1(d.get("obj_err_med_cm_orb")), r1(d.get("obj_scatter_med_cm_tf")), r1(d.get("obj_scatter_med_cm_orb"))])


def write(p, head, rows):
    with open(p, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f); w.writerow(head); w.writerows(rows)
    print(p, len(rows))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, default=Path.home() / "콜로키움" / "분석자료"); a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    objs, scenes, rec, loc = [], [], [], []
    ycb(objs, scenes); obchange(objs, scenes); recognition(rec); localization(loc)
    write(a.out / "objects.csv", ["데이터", "장면", "방법", "정답 물체", "지도 객체 이름", "판정", "중심 오차(mm)", "표면 오차(mm)", "방향 오차(°)",
                                  "정답 x", "정답 y", "정답 z", "지도 x", "지도 y", "지도 z", "관측 횟수"], objs)
    write(a.out / "scenes.csv", ["데이터", "장면", "방법", "정답 수", "지도에 올린 객체", "맞음", "중복", "이름 틀림", "먼 위치", "놓침", "없는 물체", "이름 없는 객체",
                                 "정밀도(%)", "재현율(%)", "F1(%)", "중심 오차 중앙(mm)", "표면 오차 중앙(mm)", "방향 오차 중앙(°)"], scenes)
    write(a.out / "recognition.csv", ["데이터", "장면", "전체 프레임", "처리한 프레임", "검출이 나온 프레임", "자세 추정 결과 수", "후보 상자 수 중앙", "후보 검출(ms)", "후보 검증(ms)", "자세 추정(ms)",
                                      "합계(ms)", "지연 중앙(ms)", "물체별 결과 수(평균 점수)"], rec)
    write(a.out / "localization.csv", ["데이터", "장면", "프레임", "이동 거리 tf(m)", "이동 거리 ORB-SLAM3(m)", "두 궤적 차 RMSE(cm)", "두 궤적 차 최대(cm)", "회전 차 중앙(°)", "회전 차 최대(°)",
                                       "물체 위치 오차 중앙 tf(cm)", "물체 위치 오차 중앙 ORB(cm)", "물체 위치 흩어짐 tf(cm)", "물체 위치 흩어짐 ORB(cm)"], loc)


if __name__ == "__main__":
    main()
