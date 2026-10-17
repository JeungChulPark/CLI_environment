#!/usr/bin/env python3
"""build_report.py — ObChange 에서 여섯 구성 × 지도 방식 12가지(지금 방식 + 설계 조합 11가지)를 정답으로 채점한 결과 보고서.
  six_obchange_report.html / _local.html / _notion.html. 입력: objpose/output/six_obchange_261011/scores.json (score_designs.py), c<구성>/<방문>/run.json. 끝난 방 · 구성만 쓴다.
    python3 objpose/docs/six_obchange_261011/build_report.py"""
import collections, importlib.util, json, re, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]; O = (REPO / "objpose/output" / __import__("os").environ.get("OBJPOSE_OB_OUT", "six_obchange_261011"))
spec = importlib.util.spec_from_file_location("objmap_rep", REPO / "objpose/docs/objmap_eval_261009/build_report.py"); B = importlib.util.module_from_spec(spec); spec.loader.exec_module(B)
Doc, REF = B.Doc, B.REF; J = lambda p: json.loads(Path(p).read_text())
ROOMS = [("smallroom", "SmallRoom"), ("office", "Office"), ("livingarea", "LivingArea"), ("kitchen", "Kitchen")]
CFG = {1: "1. FastSAM-x + ViT-L", 2: "2. FastSAM-x + ViT-S", 3: "3. YOLO-World + FastSAM-x + ViT-L", 4: "4. YOLO-World + FastSAM-x + ViT-S", 5: "5. YOLO-World + MobileSAM + ViT-L", 6: "6. YOLO-World + MobileSAM + ViT-S"}
CB = [("V0", "지금 방식"), ("1", "설계 1"), ("1n", "설계 1′ (빼는 규칙 없앰)"), ("2", "설계 2"), ("3", "설계 3"), ("1+2", "설계 1 + 2"), ("1n+2", "설계 1′ + 2"), ("1+3", "설계 1 + 3"), ("1n+3", "설계 1′ + 3"), ("2+3", "설계 2 + 3"), ("1+2+3", "설계 1 + 2 + 3"), ("1n+2+3", "설계 1′ + 2 + 3")]
room_of = lambda s: s.rsplit("_s", 1)[0]


def agg(rows):
    gt = sum(r["gt"] for r in rows); named = sum(r["named"] for r in rows); tp = sum(r["tp"] for r in rows); tp50 = sum(r["tp50"] for r in rows); io = [x for r in rows for x in r["iou_tp"]]; sup = [x for r in rows for x in r["support"]]
    p = tp / max(named, 1); rc = tp / max(gt, 1)
    return dict(gt=gt, named=named, tp=tp, fp=named - tp, fn=gt - tp, P=100 * p, R=100 * rc, F=100 * 2 * p * rc / max(p + rc, 1e-9), tp50=tp50, iou=float(np.mean(io)) if io else float("nan"), sup=float(np.mean(sup)) if sup else float("nan"))


def main():
    S = J(O / "scores.json"); have = collections.defaultdict(set)
    for r in S: have[(r["config"], room_of(r["scene"]))].add(r["scene"])
    rooms = [(k, nm) for k, nm in ROOMS if all(len(have[(c, k)]) == 5 for c in range(1, 7))]
    if not rooms: sys.exit("여섯 구성이 모두 끝난 방이 없습니다")
    rk = {k for k, _ in rooms}; S = [r for r in S if room_of(r["scene"]) in rk]; cfgs = list(range(1, 7)); sel = lambda c=None, v=None, room=None: [r for r in S if (c is None or r["config"] == c) and (v is None or r["variant"] == v) and (room is None or room_of(r["scene"]) == room)]
    A = {v: agg(sel(v=v)) for v, _ in CB}; n_gt = agg(sel(c=1, v="V0"))["gt"]; d = Doc(); names = " · ".join(nm for _, nm in rooms); best = max(CB, key=lambda x: A[x[0]]["F"]); f1 = lambda x: f"{x:.1f}"
    T = {(c, v): agg(sel(c=c, v=v)) for c in cfgs for v, _ in CB}; bc = max(cfgs, key=lambda c: T[(c, "V0")]["F"])
    run = {c: [J(f) for k, _ in rooms for n in range(2, 7) for f in [O / f"c{c}" / f"{k}_s{n}" / "run.json"] if f.exists()] for c in cfgs}

    d.h2("1. 요약")
    d.table(["질문", "결과"], [
        ["<strong>무엇을 했나</strong>", f"ObChange {names}의 방문 {len(rooms) * 5}회(정답 물체 {n_gt}개)에 여섯 구성을 실시간 재생으로 돌리고, 구성마다 같은 기록으로 객체 지도를 12가지 방식으로 만들어 정답으로 채점했습니다."],
        ["<strong>지도 방식 가운데 F1이 가장 높은 것</strong>", f"{best[1]} — 여섯 구성 합계 F1 {f1(A[best[0]]['F'])} % (지금 방식 {f1(A['V0']['F'])} %). 맞음 {A[best[0]]['tp']}개, 잘못 등록 {A[best[0]]['fp']}개(지금 방식 {A['V0']['tp']}개, {A['V0']['fp']}개)."],
        ["<strong>설계 1′(빼는 규칙 없앰)은 자세를 더 정확하게 만드는가</strong>", f"맞은 객체가 정답과 겹치는 비율이 평균 {A['V0']['iou']:.3f} → {A['1n']['iou']:.3f}, 겹침 0.5 이상인 객체가 {A['V0']['tp50']} → {A['1n']['tp50']}개입니다. 맞음은 {A['V0']['tp']} → {A['1n']['tp']}개, 잘못 등록은 {A['V0']['fp']} → {A['1n']['fp']}개입니다."],
        ["<strong>설계 2(깊이로 확인하고 다듬기)</strong>", f"지금 방식에 설계 2만 더하면 F1 {f1(A['V0']['F'])} → {f1(A['2']['F'])} %, 겹침 평균 {A['V0']['iou']:.3f} → {A['2']['iou']:.3f}. 설계 1′ 뒤에 더하면 {f1(A['1n']['F'])} → {f1(A['1n+2']['F'])} %, 겹침 {A['1n']['iou']:.3f} → {A['1n+2']['iou']:.3f}입니다."],
        ["<strong>설계 3(약한 증거 누적)</strong>", f"지금 방식에 설계 3만 더하면 F1 {f1(A['V0']['F'])} → {f1(A['3']['F'])} %, 맞음 {A['V0']['tp']} → {A['3']['tp']}개, 잘못 등록 {A['V0']['fp']} → {A['3']['fp']}개입니다."],
        ["<strong>여섯 구성 가운데</strong>", f"지금 방식 기준으로 F1이 가장 높은 것은 {CFG[bc]}({f1(T[(bc, 'V0')]['F'])} %)입니다."]], cls="sum", widths=["30%", None])

    d.h2("2. 실험 조건")
    d.dl([("데이터", f"ObChange {names}. 방마다 정답이 있는 방문 5회, 채점 대상 정답 물체 {n_gt}개(YCB-V와 겹치는 11종). 모든 방문에서 11종 전부를 찾게 했습니다."),
          ("실시간 조건", "영상이 10 fps로 도착한다고 보고, 앞 프레임의 처리가 끝난 순간 도착해 있는 최신 프레임 하나만 처리합니다. 카메라 자세는 데이터셋 제공값입니다."),
          ("구성", "실제 환경 실험과 같은 인식기 여섯 가지. 후보 생성 · 마스크 · 특징만 다르고, 물체 판정은 원본 SAM-6D 점수(기준 0.5), 자세 추정과 자세 검증은 항상 켰습니다. 3 · 4번은 YOLO-World 상자마다 가장 잘 겹치는 FastSAM-x 마스크 하나를 씁니다."),
          ("지도 방식", "지금 방식(같은 이름 30 cm 묶기, 위치 평균 · 방향 다수결, 존재 확률, 깊이 검사)과 설계 조합 11가지. 설계의 내용과 기준값은 실제 환경 실험과 같고, ObChange에 맞춰 조정하지 않았습니다. 조합은 1 → 2 → 3 순서로 적용합니다."),
          ("채점", "카메라가 실제로 본 점을 지도 객체에 배정(2 cm)하고, 정답 물체와 같은 이름의 객체가 0.25 이상 겹치면 맞음(TP)입니다. 이름이 붙었는데 맞음이 아닌 객체는 잘못 등록(FP), 맞는 객체가 없는 정답은 놓침(FN)입니다. 겹침 평균은 맞은 객체가 정답과 겹치는 비율의 평균으로, 자세가 정확할수록 큽니다."),
          ("반복", "한 번씩 돌렸습니다.")])
    d.table(["설계", "하는 일"], [["설계 1", "객체마다 묶인 관측 가운데 서로 가까운(25 mm, 20°) 자세의 가장 큰 무리로 자세를 정합니다. 무리가 관측 3개 미만이거나 가중치의 30 % 미만이면 객체를 뺍니다."], ["설계 1′", "설계 1에서 객체를 빼는 규칙만 없앤 것"],
                               ["설계 2", "CAD를 주변 깊이 점에 다시 맞추고(6 cm, 20° 안), 깊이 지지율이 0.2 미만이면 객체를 뺍니다."], ["설계 3", "검증에서 떨어진 후보도 증거로 쌓아, 증거 6개 이상이고 지지율 0.5 이상인 미등록 무리를 등록하고, 10 cm 안에 이름이 다른 객체가 둘이면 증거가 많은 쪽만 남깁니다."]], cls="sum", widths=["14%", None])

    d.h2("3. 지도 방식별 결과")
    d.h3(f"여섯 구성 합계 ({names}, 구성당 정답 물체 {n_gt}개)")
    d.table(["지도 방식", "등록 객체", "맞음 (TP)", "잘못 등록 (FP)", "놓침 (FN)", "정밀도 (%)", "재현율 (%)", "F1 (%)", "겹침 0.5 이상", "맞은 객체의 겹침 평균"],
            [[nm, A[v]["named"], A[v]["tp"], A[v]["fp"], A[v]["fn"], float(A[v]["P"]), float(A[v]["R"]), float(A[v]["F"]), A[v]["tp50"], f"{A[v]['iou']:.3f}"] for v, nm in CB], cls="sum", bars=(5, 6, 7), group=lambda ri, r: "hl" if CB[ri][0] in ("V0", best[0]) else "")
    d.h3("구성별 F1 (%)")
    d.table(["지도 방식"] + [f"구성 {c}" for c in cfgs], [[nm] + [float(T[(c, v)]["F"]) for c in cfgs] for v, nm in CB], cls="sum", bars=tuple(range(1, 7)))
    d.h3("구성별 맞음 / 잘못 등록")
    d.table(["지도 방식"] + [f"구성 {c}" for c in cfgs], [[nm] + [f"{T[(c, v)]['tp']} / {T[(c, v)]['fp']}" for c in cfgs] for v, nm in CB], cls="sum")
    d.h3("구성별 맞은 객체의 겹침 평균")
    d.table(["지도 방식"] + [f"구성 {c}" for c in cfgs], [[nm] + [f"{T[(c, v)]['iou']:.3f}" for c in cfgs] for v, nm in CB], cls="sum")
    if len(rooms) > 1:
        d.h3("방별 F1 (%) — 여섯 구성 합계")
        d.table(["지도 방식"] + [f"{nm} (정답 {agg(sel(c=1, v='V0', room=k))['gt']}개 × 6)" for k, nm in rooms], [[nm] + [float(agg(sel(v=v, room=k))["F"]) for k, _ in rooms] for v, nm in CB], cls="sum", bars=tuple(range(1, len(rooms) + 1)))

    d.h2("4. 구성별 결과 (지금 방식)")
    d.table(["구성", "처리한 프레임 / 도착한 프레임", "프레임당 시간, 중앙값 (ms)", "등록 객체", "맞음", "잘못 등록", "놓침", "F1 (%)", "겹침 평균"],
            [[CFG[c], f"{sum(r['processed'] for r in run[c]):,} / {sum(r['frames'] for r in run[c]):,}", round(float(np.median([r["ms_median"] for r in run[c]]))), T[(c, "V0")]["named"], T[(c, "V0")]["tp"], T[(c, "V0")]["fp"], T[(c, "V0")]["fn"], float(T[(c, "V0")]["F"]), f"{T[(c, 'V0')]['iou']:.3f}"] for c in cfgs], cls="sum", bars=(7,))

    d.h2("5. 읽는 법")
    dl = lambda a_, b_: f"{A[b_]['F'] - A[a_]['F']:+.1f} %p"
    d.ul([f"<strong>설계 1의 빼는 규칙.</strong> 설계 1은 F1 {f1(A['1']['F'])} %, 설계 1′은 {f1(A['1n']['F'])} %입니다(지금 방식 {f1(A['V0']['F'])} %). 맞음은 설계 1 {A['1']['tp']}개, 설계 1′ {A['1n']['tp']}개, 잘못 등록은 {A['1']['fp']}개와 {A['1n']['fp']}개입니다.",
          f"<strong>자세의 정확도.</strong> 맞은 객체의 겹침 평균은 지금 방식 {A['V0']['iou']:.3f}, 설계 1′ {A['1n']['iou']:.3f}, 설계 2 {A['2']['iou']:.3f}, 설계 1′ + 2 {A['1n+2']['iou']:.3f}입니다. 직접 수집한 영상에서는 깊이 지지율로만 볼 수 있었던 것을 여기서는 정답과의 겹침으로 본 것입니다.",
          f"<strong>설계 3.</strong> 지금 방식에 더했을 때 F1 변화는 {dl('V0', '3')}, 설계 1′ + 2에 더했을 때는 {dl('1n+2', '1n+2+3')}입니다. 직접 수집한 영상에서는 초과 등록만 늘렸는데, 물체 종류가 섞여 있고 잘못 등록이 많은 이 데이터에서는 한 자리의 이름 충돌을 정리하는 부분이 작동합니다.",
          "<strong>직접 수집한 영상의 결과와 다른 점</strong>은 그대로 적어 두었습니다. 두 데이터에서 같은 방향인 것만 믿을 만한 결론입니다.",
          "기준값은 직접 수집한 영상 하나를 보고 정한 것을 그대로 썼습니다. 한 번씩 돌린 결과이고, 정답 물체 수가 적어 물체 한두 개에 F1이 몇 %p 움직입니다."])
    d.h2("6. 결과 파일")
    d.ul(["실행: <code>objpose/tools/obchange/run_six_obchange.sh</code>(<code>six_rt.py</code>, 인식기 <code>objpose/tools/lab/live_six.py</code>) → <code>objpose/output/six_obchange_261011/c&lt;구성&gt;/</code>",
          "지도 방식: <code>designs_obchange.py</code>(<code>objpose/tools/lab/map_designs.py</code>) → <code>designs/</code>, 채점: <code>score_designs.py</code> → <code>scores.json</code>", "관련 보고서: <code>objpose/docs/live_six_261010/</code>(직접 수집한 영상, 실제 환경), 이 보고서: <code>objpose/docs/six_obchange_261011/build_report.py</code>"])

    ref = REF.read_text(); head = ref[: ref.index("<div class=\"wrap\">")].replace("<title>YCB 조합 실험 보고</title>", "<title>ObChange 구성 · 지도 방식 비교</title>")
    hdr = ('<header><div class="kicker">실험 보고 · ObChange, 구성 여섯 가지 × 지도 방식 열두 가지 · 2026-10-11</div><h1>ObChange에서의 구성별 · 지도 방식별 객체 지도</h1>'
           f'<p class="lead">정답이 있는 ObChange({names}, 방문 {len(rooms) * 5}회)에서 후보 생성 · 마스크 · 특징의 여섯 조합을 돌리고, 같은 기록으로 지도를 지금 방식과 설계 조합 11가지로 만들어 채점했습니다. 직접 수집한 영상에서는 볼 수 없었던 맞음 · 잘못 등록 · 자세 정확도를 정답으로 확인합니다.</p></header>')
    body = hdr + "\n" + "\n".join(x for x, _ in d.parts)
    (HERE / "six_obchange_report.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "six_obchange_report_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "ObChange 구성 · 지도 방식 비교")
    nb = ("<p>실험 보고 · ObChange · 2026-10-11</p>" + re.sub(r'<header>.*?</div>', "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "")) + "\n" + "\n".join(y for _, y in d.parts)
    (HERE / "six_obchange_report_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    print("rooms", [k for k, _ in rooms], "gt", n_gt)
    for v, nm in CB: print(f"{nm:26s} 등록 {A[v]['named']:4d} TP {A[v]['tp']:3d} FP {A[v]['fp']:4d} FN {A[v]['fn']:3d} F1 {A[v]['F']:5.1f} TP50 {A[v]['tp50']:3d} IoU {A[v]['iou']:.3f} | " + " ".join(f"{T[(c, v)]['F']:4.1f}" for c in cfgs))


if __name__ == "__main__":
    main()
