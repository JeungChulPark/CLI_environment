#!/usr/bin/env python3
"""summarize_fair.py — 관측 점 기준 채점 결과(inst_iou*.json)를 데이터별 표로 정리해 마크다운으로 쓴다.

정답 물체마다 가장 많이 겹치는 지도 객체(겹침 비율 0.25 이상)를 보고 네 가지로 나눈다:
  이름까지 맞음 / 같은 종류 모양의 다른 물체 이름 / 모양이 다른 물체 이름 / 인식 실패(객체가 없거나 이름을 못 붙임)
지도 쪽은 대상 물체 이름이 붙은 객체 수, 그중 맞는 것(같은 이름 겹침 0.25 이상), 잘못 등록된 것, 정밀도 · 재현율 · F1.
    python3 objpose/tools/analysis/summarize_fair.py [--out ~/콜로키움/전체_실험결과_논문용.md]
"""
import argparse, json
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]; O = REPO / "objpose/output"
THR = 0.25
SHAPE = {k: "캔" for k in ("master_chef_can", "tomato_soup_can", "tuna_fish_can")}
SHAPE.update({k: "상자" for k in ("cracker_box", "sugar_box", "pudding_box", "gelatin_box", "foam_brick", "wood_block", "potted_meat_can")})
SHAPE.update({k: "병" for k in ("mustard_bottle", "bleach_cleanser")}); SHAPE.update({k: "집게" for k in ("large_clamp", "extra_large_clamp")})
ROOMS = [("kitchen", "ObChange Kitchen"), ("office", "ObChange Office"), ("livingarea", "ObChange LivingArea"), ("smallroom", "ObChange SmallRoom")]


def key(n):
    if n is None: return None
    n = n.replace("ycbv_", ""); return n[4:] if n[:3].isdigit() else n


def load(path, rename=None, only=None):
    if not Path(path).exists(): return []
    rows = json.loads(Path(path).read_text())
    if only: rows = [r for r in rows if r["method"] == only]
    return [dict(r, method=rename) for r in rows] if rename else rows


def stats(rows):
    R = [r for r in rows if r["object"]]; info = [r for r in rows if not r["object"]]
    if not R: return None
    n = len(R); named = sum(r["named"] for r in info); c = dict(exact=0, same=0, diff=0, fail=0)
    for r in R:
        a, g = key(r.get("any_name")), key(r["object"])
        if r["iou_any"] < THR or a is None: c["fail"] += 1
        elif a == g: c["exact"] += 1
        elif SHAPE.get(a, a) == SHAPE.get(g, g): c["same"] += 1
        else: c["diff"] += 1
    present = [r for r in R if r["iou_any"] >= THR]; ok = [r for r in R if r["iou_name"] >= THR]
    p = len(ok) / max(named, 1); rc = len(ok) / n; d = [r["dist_mm"] for r in ok if r.get("dist_mm") is not None]
    return dict(n=n, present=len(present), iou=float(np.mean([r["iou_any"] for r in present])) if present else 0.0, **c, named=named, ok=len(ok), wrong=named - len(ok),
                P=100 * p, R=100 * rc, F=100 * 2 * p * rc / max(p + rc, 1e-9), dist=float(np.median(d)) if d else None)


def table(title, note, cols):
    """cols: [(방법 이름, stats)]"""
    cols = [(m, s) for m, s in cols if s]
    n = cols[0][1]["n"]; cols = [(m, s) for m, s in cols if s["n"] == n]      # 일부 장면만 돌린 방법은 표에서 뺀다
    pc = lambda v: f"{v} ({100 * v / n:.0f} %)"
    L = [f"## {title}", "", note, "", "| 항목 | " + " | ".join(m for m, _ in cols) + " |", "|---|" + "---|" * len(cols)]
    def row(name, f): L.append(f"| {name} | " + " | ".join(f(s) for _, s in cols) + " |")
    row("정답 자리에 객체가 있음 (이름 무시)", lambda s: pc(s["present"]))
    row("그 객체와 정답의 평균 겹침 비율", lambda s: f"{s['iou']:.2f}")
    row("정답 물체를 이름까지 맞게 올림", lambda s: pc(s["exact"]))
    row("같은 종류 모양의 다른 물체 이름을 붙임", lambda s: pc(s["same"]))
    row("모양이 다른 물체 이름을 붙임", lambda s: pc(s["diff"]))
    row("인식 실패 (객체가 없거나 이름을 못 붙임)", lambda s: pc(s["fail"]))
    row("대상 물체 이름이 붙은 지도 객체 수", lambda s: str(s["named"]))
    row("그중 잘못 등록된 객체", lambda s: str(s["wrong"]))
    row("정밀도 (%)", lambda s: f"{s['P']:.1f}"); row("재현율 (%)", lambda s: f"{s['R']:.1f}"); row("F1 (%)", lambda s: f"{s['F']:.1f}")
    row("관측된 정답 표면에서 지도 객체까지 거리, 중앙값 (mm)", lambda s: "–" if s["dist"] is None else f"{s['dist']:.1f}")
    return L + [""]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, default=Path.home() / "콜로키움" / "전체_실험결과_논문용.md"); a = ap.parse_args()
    M3 = ["제안 방법", "FM-Fusion", "DualMap"]; PL = "DualMap (조정 문구)"
    sets = []
    for name, d, plain in (("YCB-Video (탁상 12장면)", "fmf_cmp_260928", "dualmap_plain_261005/ycbv"), ("YCB-M (탁상 32장면)", "fmf_cmp_ycbm_260929", "dualmap_plain_261005/ycbm")):
        rows = load(O / d / "eval_own/inst_iou_tau20.json") + load(O / plain / "inst_iou_tau20.json", PL, "DualMap")
        sets.append((name, "카메라 자세는 ORB-SLAM3 추정값.", rows))
    ob = load(O / "obchange_261005/eval/inst_iou_tau20.json") + load(O / "obchange_261005/eval/inst_iou_tau20_dmplain.json", PL, "DualMap")
    for pre, name in ROOMS:
        sets.append((name + " (방문 5회)", "카메라 자세는 데이터셋 제공값.", [r for r in ob if r["scene"].startswith(pre + "_")]))
    sets.append(("ObChange 네 방 합계", "", ob))
    L = ["# 객체 지도 실험 결과 (관측된 점 기준 채점)", "",
         "채점: 카메라가 본 표면의 점을 2 cm 안에서 가장 가까운 지도 객체에 배정하고, 정답 물체에 속한 점 집합과의 겹침 비율이 0.25 이상이면 그 자리에 객체가 있는 것으로 본다.",
         "DualMap (조정 문구)는 색 형용사를 뺀 일반 이름 문구와 검출 기준값 0.05 로 돌린 결과이고, 나머지는 세 방법에 같은 문구를 주었다.", ""]
    summ = []
    for name, note, rows in sets:
        cols = [(m, stats([r for r in rows if r["method"] == m])) for m in M3 + [PL]]
        if not cols[0][1]: continue
        L += table(name, f"정답 물체 {cols[0][1]['n']}개. {note}", cols); summ.append((name, cols))
    L += ["## 요약: 데이터별 F1 (%) / 인식 실패 비율 (%)", "", "| 데이터 | " + " | ".join(M3 + [PL]) + " |", "|---|---|---|---|---|"]
    for name, cols in summ:
        L.append(f"| {name} | " + " | ".join(f"{s['F']:.1f} / {100 * s['fail'] / s['n']:.0f}" if s and s["n"] == cols[0][1]["n"] else "–" for _, s in cols) + " |")
    a.out.write_text("\n".join(L) + "\n"); print("\n".join(L[-len(summ) - 3:])); print(a.out)


if __name__ == "__main__":
    main()
