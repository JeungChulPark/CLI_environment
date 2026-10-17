#!/usr/bin/env python3
"""build_report.py — 실제 환경(노트북 SLAM + 이 PC 인식)에서 여섯 구성 × 영상 8개를 돌리고 지도 방식 네 가지(지금 방식, 설계 1, 1+2, 1+2+3)로 만든 객체 지도 비교 보고서.
  live_six_report.html / _local.html / _notion.html, --pptx 를 주면 ~/콜로키움/전체_실험결과.pptx 뒤에 요약 쪽을 덧붙인다(제목 접두 "실제 환경").
입력: objpose/output/live_six_261010/{c<구성>/<세션>/summary.json, designs/c<구성>_<세션>.json}, 그림 media/ (objpose/tools/lab/render_live_maps.py). 끝난 실행만 쓴다.
    python3 objpose/docs/live_six_261010/build_report.py [--pptx]"""
import argparse, importlib.util, json, re, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]; O = (REPO / "objpose/output" / __import__("os").environ.get("OBJPOSE_LAB_OUT", "live_six_261010")); MED = HERE / "media"
spec = importlib.util.spec_from_file_location("objmap_rep", REPO / "objpose/docs/objmap_eval_261009/build_report.py"); B = importlib.util.module_from_spec(spec); spec.loader.exec_module(B)
Doc, REF = B.Doc, B.REF; J = lambda p: json.loads(Path(p).read_text())
SES = {"260901_cbnu_bigeightcircle": "장소 A · 큰 8자 주행", "260901_cbnu_digut": "장소 A · ㄷ자 주행", "260901_cbnu_eightcircle": "장소 A · 8자 주행", "260901_cbnu_longcircle": "장소 A · 긴 원 주행",
       "260826_etri_differentObject": "장소 B · 서로 다른 물체 배치", "260826_etri_eightcircle_dark": "장소 B · 8자 주행 (어두움)", "260826_etri_longcircle_dark": "장소 B · 긴 원 주행 (어두움)", "260826_etri_similarObject": "장소 B · 닮은 물체 배치"}
CFG = {1: "1. FastSAM-x + ViT-L", 2: "2. FastSAM-x + ViT-S", 3: "3. YOLO-World + FastSAM-x + ViT-L", 4: "4. YOLO-World + FastSAM-x + ViT-S", 5: "5. YOLO-World + MobileSAM + ViT-L", 6: "6. YOLO-World + MobileSAM + ViT-S"}
VN = {"V0": "지금 방식", "V1": "설계 1", "V2": "설계 1+2", "V3": "설계 1+2+3"}; VS = ["V0", "V1", "V2", "V3"]


def load():
    D = {}
    for f in sorted((O / "designs").glob("c*_26*.json")):
        c = int(f.name[1]); s = f.stem.split("_", 1)[1]
        if s in SES and (O / f"c{c}" / s / "summary.json").exists(): D[(c, s)] = J(f)
    return D


def stat(D, keys, v):
    L = [m for k in keys for m in D[k][v]]; kinds = sum(len({m["name"] for m in D[k][v]}) for k in keys); sup = [m["support"] for m in L if m["support"] is not None]
    return dict(n=len(L), kinds=kinds, extra=len(L) - kinds, sup=float(np.mean(sup)) if sup else float("nan"), ok=100 * float(np.mean([s >= 0.5 for s in sup])) if sup else float("nan"), nsup=len(sup))


CB = [("V0", "지금 방식"), ("1", "설계 1"), ("1n", "설계 1′ (빼는 규칙 없앰)"), ("2", "설계 2"), ("3", "설계 3"), ("1+2", "설계 1 + 2"), ("1n+2", "설계 1′ + 2"), ("1+3", "설계 1 + 3"), ("1n+3", "설계 1′ + 3"),
      ("2+3", "설계 2 + 3"), ("1+2+3", "설계 1 + 2 + 3"), ("1n+2+3", "설계 1′ + 2 + 3")]


def combo_rows(D, cfgs, full, per_config=True):
    """조합마다 [이름, 종류, 초과, 지지율, (구성별 종류/초과…)]"""
    rows = []
    for k, nm in CB:
        if any(k not in D[(c, s)] for c in cfgs for s in full): continue
        t = stat(D, [(c, s) for c in cfgs for s in full], k); r = [nm, t["kinds"], t["extra"], f"{t['sup']:.2f}", f"{t['ok']:.0f}"]
        if per_config: r += [(lambda x: f"{x['kinds']} / {x['extra']}")(stat(D, [(c, s) for s in full], k)) for c in cfgs]
        rows.append(r)
    return rows


def runs(D, c):
    fr = fed = sl = dr = 0; ms = []
    for (cc, s) in D:
        if cc != c: continue
        S = J(O / f"c{c}" / s / "summary.json"); L = (S.get("lanes") or [S])[0]; fr += S.get("sam6d_frames") or 0; fed += S.get("sam6d_fed") or 0; sl += L.get("slam_ok") or 0; dr += L.get("slam_dropped") or 0
        F = [json.loads(l) for l in open(O / f"c{c}" / s / "sam6d/frames.jsonl")]; ms += [f["ms"]["total"] for f in F]
    return dict(proc=fr, fed=fed, slam=sl, drop=dr, ms=float(np.median(ms)) if ms else float("nan"), msm=float(np.mean(ms)) if ms else float("nan"))


def chart(D, cfgs, ses):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib import font_manager as fmg
    fp = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"; fmg.fontManager.addfont(fp); plt.rcParams["font.family"] = fmg.FontProperties(fname=fp).get_name()
    fig, ax = plt.subplots(1, 3, figsize=(13.2, 4.4), dpi=150); cols = ["#9e9e9e", "#5b8fd6", "#2e7d32", "#e08a1e"]; x = np.arange(len(cfgs)); w = 0.2
    for a_, key, title, ymax in ((ax[0], "kinds", f"등록된 물체 종류 (최대 {8 * len(ses)})", 8 * len(ses) * 1.12), (ax[1], "extra", "같은 종류의 초과 등록 (적을수록 좋음)", None), (ax[2], "sup", "깊이 지지율 평균 (1에 가까울수록 좋음)", 1.12)):
        for i, v in enumerate(VS):
            vals = [stat(D, [(c, s) for s in ses if (c, s) in D], v)[key] for c in cfgs]; a_.bar(x + (i - 1.5) * w, vals, w * 0.92, color=cols[i], label=VN[v])
            for xi, val in zip(x + (i - 1.5) * w, vals): a_.text(xi, val, f"{val:.2f}" if key == "sup" else f"{val:.0f}", ha="center", va="bottom", fontsize=8.5)
        a_.set_xticks(x); a_.set_xticklabels([f"구성 {c}" for c in cfgs], fontsize=11); a_.set_title(title, fontsize=12.5, loc="left")
        if ymax: a_.set_ylim(0, ymax)
        for sp in ("top", "right"): a_.spines[sp].set_visible(False)
    h_, l_ = ax[0].get_legend_handles_labels(); fig.tight_layout(rect=(0, 0.07, 1, 1)); fig.legend(h_, l_, fontsize=11, frameon=False, ncol=4, loc="lower center"); MED.mkdir(exist_ok=True); f = MED / "glance.png"; fig.savefig(f); plt.close(fig); return f


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--pptx", action="store_true"); a = ap.parse_args()
    D = load(); cfgs = sorted({c for c, _ in D}); ses = [s for s in SES if any((c, s) in D for c in cfgs)]; full = [s for s in ses if all((c, s) in D for c in cfgs)]
    if not D: sys.exit("끝난 실행이 없습니다")
    K = lambda c: [(c, s) for s in full]; T = {c: {v: stat(D, K(c), v) for v in VS} for c in cfgs}; RN = {c: runs(D, c) for c in cfgs}; d = Doc(); tot = 8 * len(full)
    ALL = {v: stat(D, [k for c in cfgs for k in K(c)], v) for v in VS}; cnt = lambda key: sum(D[k]["counts"][key] for c in cfgs for k in K(c))
    # 설계별 변화
    def diff(v0, v1):
        rem = add = 0; mv = []
        for c in cfgs:
            for k in K(c):
                A, Bv = D[k][v0], D[k][v1]; used = set()
                for m in A:
                    cand = [(np.linalg.norm(np.asarray(m["T_w_obj"])[:3, 3] - np.asarray(n["T_w_obj"])[:3, 3]), j) for j, n in enumerate(Bv) if n["name"] == m["name"] and j not in used]
                    if cand and min(cand)[0] < 0.30: used.add(min(cand)[1]); mv.append(min(cand)[0] * 1000)
                    else: rem += 1
                add += len(Bv) - len(used)
        return rem, add, float(np.median(mv)) if mv else 0.0, float(np.percentile(mv, 90)) if mv else 0.0
    d01, d12, d23 = diff("V0", "V1"), diff("V1", "V2"), diff("V2", "V3")

    d.h2("1. 요약")
    best = min(cfgs, key=lambda c: (T[c]["V3"]["extra"], -T[c]["V3"]["kinds"]))
    d.table(["질문", "결과"], [
        ["<strong>실제 환경으로 다시 돌릴 수 있는가</strong>", f"가능합니다. 노트북에서 ORB-SLAM3를 실시간으로 돌리고 이 PC에서 인식과 지도 생성을 하는 구성으로 영상 {len(full)}개 × 구성 {len(cfgs)}가지를 모두 돌렸습니다. 장소 B의 물체 배치 영상 두 개는 카메라 녹화가 하나뿐이라 같은 영상을 SLAM과 인식에 함께 썼습니다."],
        ["<strong>설계 1(합의로 합치기)의 효과</strong>", f"득실이 함께 있습니다. 깊이 지지율이 평균 {ALL['V0']['sup']:.2f} → {ALL['V1']['sup']:.2f}로 오르고 초과 등록이 {ALL['V0']['extra']} → {ALL['V1']['extra']}개로 줄었지만, 자세가 모이지 않았다고 본 객체 {d01[0]}개를 빼면서 등록된 종류도 {ALL['V0']['kinds']} → {ALL['V1']['kinds']}개(최대 {tot * len(cfgs)})로 줄었습니다. 빠진 것 가운데 실제 물체가 섞여 있습니다. 남은 객체의 위치는 중앙값 {d01[2]:.0f} mm, 큰 경우(90 % 지점) {d01[3]:.0f} mm 움직였습니다."],
        ["<strong>설계 2(깊이로 확인하고 다듬기)의 효과</strong>", f"작습니다. 뺀 객체는 {d12[0]}개이고, 자세를 다듬어 지지율 평균이 {ALL['V1']['sup']:.2f} → {ALL['V2']['sup']:.2f}가 됐습니다. 등록된 종류와 초과 등록은 그대로입니다."],
        ["<strong>설계 3(약한 증거 누적)의 효과</strong>", f"지금 기준으로는 도움이 되지 않았습니다. 새로 등록한 {cnt('added_by_design3')}개 가운데 새 종류는 {ALL['V3']['kinds'] - ALL['V2']['kinds']}개뿐이고 나머지는 이미 있는 종류라, 초과 등록이 {ALL['V2']['extra']} → {ALL['V3']['extra']}개로 늘었습니다. 이름 충돌 정리는 {cnt('dropped_name_conflict')}건이었습니다."],
        ["<strong>여섯 구성 가운데</strong>", f"설계 1+2+3 기준으로 초과 등록이 가장 적은 것은 {CFG[best]}입니다(종류 {T[best]['V3']['kinds']}/{tot}, 초과 {T[best]['V3']['extra']}). 구성별 수치는 3절에 있습니다."]],
        cls="sum", widths=["28%", None])
    d.fig(chart(D, cfgs, full), "구성별 · 지도 방식별 합계. 회색 = 지금 방식, 파랑 = 설계 1, 초록 = 설계 1+2, 주황 = 설계 1+2+3", width=1500)

    d.h2("2. 실험 조건")
    d.dl([("실제 환경", "SLAM은 노트북에서 ORB-SLAM3(RGB-D, 특징 3,000개)를 실시간으로 돌려 자세를 LAN으로 보내고, 이 PC가 인식용 카메라 영상에서 물체를 인식해 객체 지도를 만듭니다. 영상은 녹화된 속도 그대로 재생하고, 인식이 밀리면 최신 프레임만 처리합니다. 실시간 뷰어와 같은 경로입니다."),
          ("영상", f"직접 수집한 주행 영상 {len(full)}개. 장소 A의 4개와 장소 B의 8자 · 긴 원 주행은 SLAM용과 인식용 두 카메라를 쓰고(카메라 사이 변환은 기존 보정값), 장소 B의 물체 배치 영상 두 개는 카메라가 하나라 같은 영상을 함께 썼습니다."),
          ("구성", "후보 생성 · 마스크 · 특징의 여섯 조합. 물체 판정은 모두 원본 SAM-6D 점수(기준 0.5)이고, 자세 추정과 자세 검증은 항상 켰습니다. 3 · 4번은 YOLO-World 상자마다 가장 잘 겹치는 FastSAM-x 마스크 하나를 씁니다."),
          ("물체", "실험실 물체 8종이 영상마다 하나씩 놓여 있습니다. 정답 자세는 없습니다."),
          ("지도 방식", "지금 방식은 실행 중에 만들어진 지도 그대로입니다. 설계 1~3은 같은 실행의 기록(프레임별 자세, 검증에서 떨어진 후보, 깊이 영상, 카메라 자세)으로 지도만 다시 만든 것이라, 인식 결과는 네 방식이 완전히 같습니다."),
          ("깊이 지지율", "정답이 없는 영상에서 객체가 실제 표면 위에 놓였는지를 보는 값입니다. 객체의 CAD를 지도 자세로 여러 프레임에 투영해, 깊이 영상과 2 cm 안에서 맞는 점의 비율을 구합니다(앞이 가려진 점은 제외). 1에 가까우면 그 자리에 그 모양의 표면이 있다는 뜻이고, 0에 가까우면 빈 곳에 떠 있다는 뜻입니다. 이름이 맞는지는 알려 주지 않습니다."),
          ("반복", "한 번씩 돌렸습니다.")])
    d.h3("지도 방식")
    d.table(["방식", "하는 일"], [["지금 방식", "같은 이름의 관측을 30 cm 안에서 묶어 위치는 평균, 방향은 다수결. 반복 관측되면 등록하고, 깊이 영상에서 자리가 비어 있으면 지웁니다."],
                              ["설계 1. 합의로 합치기", "객체마다 묶인 관측 가운데 서로 가까운(25 mm, 20°) 자세의 가장 큰 무리만 씁니다. 검증 점수와 거리로 가중합니다. 무리가 관측 3개 미만이거나 가중치의 30 % 미만이면 뺍니다."],
                              ["설계 2. 깊이로 확인하고 다듬기", "객체의 CAD를 주변 깊이 점에 다시 맞추고(6 cm, 20° 안), 지지율이 오르면 그 자세를 씁니다. 판단할 프레임이 3장 이상인데 지지율이 0.2 미만이면 뺍니다."],
                              ["설계 3. 약한 증거 누적", "검증에서 떨어진 후보도 깊이로 3D 위치를 구해 증거로 쌓습니다. 지도에 못 올라간 관측이라도 통과한 자세가 1개 이상이고 같은 자리의 증거가 6개 이상이며 지지율 0.5 이상이면 등록합니다. 10 cm 안에 이름이 다른 객체가 둘이면 증거가 많은 쪽만 남깁니다."]], cls="sum", widths=["24%", None])

    d.h2("3. 결과")
    d.h3(f"구성별 합계 (영상 {len(full)}개, 물체 종류 최대 {tot})")
    rows = []
    for c in cfgs:
        for v in VS:
            t = T[c][v]; rows.append([CFG[c] if v == "V0" else "", VN[v], t["n"], t["kinds"], t["extra"], float(t["sup"]), float(t["ok"])])
    d.table(["구성", "지도 방식", "등록 객체", "등록된 종류", "초과 등록", "깊이 지지율 평균", "지지율 0.5 이상 (%)"], rows, cls="sum", bars=(6,), group=lambda ri, r: "hl" if ri % 4 == 3 else "")
    d.h3("실행 기록")
    d.table(["구성", "인식에 보낸 프레임", "인식이 처리한 프레임", "프레임당 처리 시간, 중앙값 (ms)", "평균 (ms)", "SLAM 추적 프레임", "SLAM이 버린 프레임"],
            [[CFG[c], f"{RN[c]['fed']:,}", f"{RN[c]['proc']:,}", round(RN[c]["ms"]), round(RN[c]["msm"]), f"{RN[c]['slam']:,}", RN[c]["drop"]] for c in cfgs], cls="sum")
    d.h3("영상별 등록 객체 수 (괄호 안은 초과 등록) — 지금 방식 → 설계 1+2+3")
    d.table(["영상"] + [f"구성 {c}" for c in cfgs], [[SES[s]] + [(lambda a0, a3: f"{len(a0)} ({len(a0) - len({m['name'] for m in a0})}) → {len(a3)} ({len(a3) - len({m['name'] for m in a3})})")(D[(c, s)]["V0"], D[(c, s)]["V3"]) if (c, s) in D else "–" for c in cfgs] for s in ses], cls="sum")
    d.h3("설계가 지도를 바꾼 내용 (모든 구성 · 영상 합계)")
    d.table(["단계", "빠진 객체", "새로 들어온 객체", "위치 이동, 중앙값 (mm)", "위치 이동, 90 % 지점 (mm)", "지지율 평균"],
            [["지금 방식", "–", "–", "–", "–", float(ALL["V0"]["sup"])], ["→ 설계 1", d01[0], d01[1], float(d01[2]), float(d01[3]), float(ALL["V1"]["sup"])], ["→ 설계 2", d12[0], d12[1], float(d12[2]), float(d12[3]), float(ALL["V2"]["sup"])],
             ["→ 설계 3", d23[0], d23[1], float(d23[2]), float(d23[3]), float(ALL["V3"]["sup"])]], cls="sum")
    d.h3("설계를 하나씩, 둘씩, 셋 모두 쓴 경우 (모든 구성 · 영상 합계)")
    d.table(["지도 방식", f"등록된 종류 (최대 {tot * len(cfgs)})", "초과 등록", "깊이 지지율 평균", "지지율 0.5 이상 (%)"] + [f"구성 {c}: 종류 / 초과" for c in cfgs], combo_rows(D, cfgs, full), cls="sum", group=lambda ri, r: "hl" if r[0].startswith("설계 1′ + 2") and "3" not in r[0] else "")
    d.ul(["설계 1′은 설계 1에서 \"무리가 관측 3개 미만이거나 가중치의 30 % 미만이면 객체를 뺀다\"는 규칙만 없앤 것입니다. 합의 무리로 자세를 정하는 부분은 그대로입니다. 조합은 언제나 1 → 2 → 3 순서로 적용했습니다.",
          "<strong>설계 1′은 물체를 하나도 잃지 않으면서</strong> 위치를 실제 표면에 더 가깝게 놓습니다. 종류와 초과 등록이 지금 방식과 같고 지지율만 오릅니다. 설계 1에서 종류가 줄었던 것은 전부 빼는 규칙 때문이었습니다.",
          "<strong>설계 2만 쓰면</strong> 지금 방식의 자세를 깊이에 맞춰 다듬어 지지율이 오르지만, 객체 두 개를 빼면서 종류가 하나 줄었습니다. 설계 1′ 뒤에 쓰면(설계 1′ + 2) 빠지는 객체가 없습니다. 자세가 먼저 바로잡혀 깊이 확인을 통과하기 때문입니다.",
          "<strong>설계 3은 어느 조합에 넣어도</strong> 초과 등록을 14~15개 늘립니다. 새 종류는 거의 올리지 못합니다.",
          "<strong>가장 나은 조합은 설계 1′ + 2입니다.</strong> 지금 방식과 같은 수의 물체를 올리고(종류와 초과 등록이 같음) 지지율이 가장 높은 축입니다. 다만 설계 2가 올린 지지율은 설계 2 자신이 기준으로 삼는 값이라, 정답이 있는 데이터에서 따로 확인해야 합니다."])
    d.p(f"쌓인 자료: 검증을 통과한 관측 {cnt('observations'):,}개, 그중 지도 객체에 묶이지 못한 것 {cnt('unassigned_observations'):,}개, 검증에서 떨어진 후보(약한 증거) {cnt('weak_evidence'):,}개.", "note")

    d.h2("4. 영상별 최종 객체 지도")
    d.p("영상 전체로 복원한 장면을 비스듬히 내려다본 그림 위에 지도 객체를 놓았습니다. 열은 여섯 구성, 행은 위에서부터 지금 방식, 설계 1, 설계 1+2, 설계 1+2+3입니다. 초록은 그 종류에서 하나, 빨강은 같은 종류의 초과 등록이고 이름표 옆 숫자는 깊이 지지율입니다. 물체가 두 군데 이상에 모여 있는 영상은 가장 많이 모인 곳을 보입니다.")
    IDX = J(MED / "index.json") if (MED / "index.json").exists() else {}
    for s in ses:
        if s in IDX: d.h3(SES[s]); d.fig(MED / IDX[s], f"{SES[s]} · 열 = 구성 1~6, 행 = 지금 방식 / 설계 1 / 설계 1+2 / 설계 1+2+3", width=2400)

    d.h2("5. 해석")
    d.ul([f"<strong>지금 방식이 이미 대부분의 물체를 올립니다.</strong> 여섯 구성 합계로 {ALL['V0']['kinds']}/{tot * len(cfgs)}종을 등록했고 초과 등록은 {ALL['V0']['extra']}개입니다. 자세 검증을 항상 켠 조건에서는 지도 쪽에서 더 걸러 낼 것이 많지 않습니다.",
          "<strong>설계 1은 위치를 실제 표면에 더 가깝게 놓지만, 삭제 규칙이 지나칩니다.</strong> 무리가 작다는 이유로 뺀 객체에 실제 물체가 들어 있어 등록된 종류가 줄었습니다. 합의 무리로 자세를 정하는 부분만 쓰고, 객체를 빼는 규칙은 없애거나 훨씬 느슨하게 해야 합니다.",
          "<strong>설계 2의 깊이 확인은 이 영상들에서 뺄 것이 없었습니다.</strong> 검증을 통과한 객체는 대부분 실제 표면 위에 있습니다. 자세 다듬기는 지지율을 조금 올리는 정도입니다.",
          "<strong>설계 3은 초과 등록을 늘렸습니다.</strong> 지도에 못 올라간 관측 무리는 대부분 이미 등록된 물체를 다른 자리에서 잘못 본 것이었습니다. 증거 개수만으로는 이를 가려내지 못합니다. 새 종류를 올린 경우는 거의 없어서, 놓친 물체를 되살리는 효과는 확인되지 않았습니다.",
          "<strong>구성 사이의 차이가 지도 방식 사이의 차이보다 큽니다.</strong> ViT-L 구성(1, 3, 5번)은 등록된 종류가 적고, 화면 전체 분할 + ViT-L(1번)은 프레임당 약 0.8초라 처리한 프레임이 다른 구성의 4분의 1입니다."])
    d.h2("6. 읽는 법과 한계")
    d.ul(["정답 자세가 없는 영상입니다. 등록된 종류 수와 초과 등록은 이름과 개수만 본 것이고, 깊이 지지율은 그 자리에 그 모양의 표면이 있는지만 봅니다. 이름이 틀렸는데 모양이 비슷한 경우는 걸러 내지 못합니다.",
          "설계 2는 자신이 쓰는 깊이 지지율로 자세를 고르므로, 설계 2 이후의 지지율이 오르는 것은 당연한 면이 있습니다. 설계 1의 지지율 상승은 깊이를 쓰지 않고 얻은 것이라 독립된 확인입니다.",
          "지금 방식도 방향은 평균이 아니라 다수결로 합칩니다. 설계 1이 바꾼 것은 위치까지 합의 무리로 정하고 무리 밖 관측을 버리는 부분입니다.",
          "설계 2의 삭제 기준은 0.2로 낮게 잡았습니다. 캔처럼 작은 물체는 깊이 잡음과 몇 cm의 위치 오차만으로 지지율이 0.2~0.3까지 내려가서, 0.5로 잡으면 실제 물체가 빠졌습니다.",
          "설계 3의 기준값(증거 6개, 지지율 0.5, 이름 충돌 10 cm)은 이 실험에서 정한 값이고 따로 조정하지 않았습니다.",
          "기준값을 정하는 데 장소 A의 한 영상 · 한 구성의 결과를 보았습니다. ObChange처럼 정답이 있는 데이터에서 설계 1 · 2의 효과를 확인하는 것이 다음 단계입니다.",
          "물체 판정은 여섯 구성 모두 원본 점수 방식입니다. 제안 관문을 쓰는 지금의 제안 방법(실시간 뷰어 설정)과 정확히 같은 구성은 여섯 가지 안에 없습니다.",
          "한 번씩 돌린 결과입니다. 이 PC는 GPU 드라이버 버전이 어긋난 상태(재부팅 전)에서 임시 우회를 넣고 돌렸습니다. 인식 결과에는 영향이 없습니다."])
    d.h2("7. 결과 파일")
    d.ul(["실제 환경 실행: <code>objpose/tools/lab/run_live_six.sh</code>(<code>run_live_one.sh</code>, 인식기 <code>live_six.py</code>) → <code>objpose/output/live_six_261010/c&lt;구성&gt;/&lt;세션&gt;/</code>",
          "지도 방식: <code>objpose/tools/lab/map_designs.py</code>, <code>run_designs_all.sh</code> → <code>objpose/output/live_six_261010/designs/</code>", "지도 그림: <code>objpose/tools/lab/render_live_maps.py</code> → 이 폴더의 <code>media/</code>",
          "관련 보고서: <code>objpose/docs/combo_obchange_261009/</code>(구성별 지도 비교), 이 보고서: <code>objpose/docs/live_six_261010/build_report.py</code>"])

    ref = REF.read_text(); head = ref[: ref.index("<div class=\"wrap\">")].replace("<title>YCB 조합 실험 보고</title>", "<title>실제 환경 구성 · 지도 방식 비교</title>").replace("</style>", "figure{margin:6px 0 4px}figure img{max-width:100%;height:auto;border:1px solid var(--line);border-radius:4px;display:block;background:#fff}figcaption{margin-top:4px;margin-bottom:10px}</style>")
    hdr = ('<header><div class="kicker">실험 보고 · 실제 환경, 구성 여섯 가지 × 지도 방식 네 가지 · 2026-10-10</div><h1>실제 환경에서의 구성별 · 지도 방식별 객체 지도</h1>'
           f'<p class="lead">노트북의 실시간 SLAM과 이 PC의 인식을 연결한 실제 환경에서, 후보 생성 · 마스크 · 특징의 여섯 조합을 직접 수집한 영상 {len(full)}개에 돌렸습니다. 각 실행의 기록으로 객체 지도를 지금 방식과 새 설계 세 가지로 만들어 비교했습니다.</p></header>')
    body = hdr + "\n" + "\n".join(x for x, _ in d.parts)
    (HERE / "live_six_report.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "live_six_report_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "실제 환경 구성 · 지도 방식 비교")
    nb = ("<p>실험 보고 · 실제 환경 · 2026-10-10</p>" + re.sub(r'<header>.*?</div>', "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "")) + "\n" + "\n".join(y for _, y in d.parts)
    (HERE / "live_six_report_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    print("runs", len(D), "configs", cfgs, "full sessions", len(full))
    for c in cfgs: print(CFG[c], {v: (T[c][v]["n"], T[c][v]["kinds"], T[c][v]["extra"], round(T[c][v]["sup"], 2)) for v in VS}, RN[c])
    print("diff", d01, d12, d23, {k: cnt(k) for k in ("observations", "unassigned_observations", "weak_evidence", "added_by_design3", "dropped_name_conflict")})
    if a.pptx: pptx(D, cfgs, full, T, RN, ALL, (d01, d12, d23), tot)


def pptx(D, cfgs, full, T, RN, ALL, dd, tot):
    from PIL import Image
    from pptx import Presentation
    from pptx.util import Inches
    for x in ("docs/ppt/dataset_exp_261005", "docs/ppt/arch_261005", "tools/analysis"): sys.path.insert(0, str(REPO / "objpose" / x))
    from build import slide, lines, text, LM, CW, W
    sys.path.append(str(REPO / "objpose/docs/ppt/fair_results_261005"))
    from replace_tables import title_of
    from append_rooms import grid
    OUT = Path.home() / "콜로키움" / "전체_실험결과.pptx"; PRE = "실제 환경"; prs = Presentation(str(OUT)); ids = prs.slides._sldIdLst
    for i in sorted((i for i, s in enumerate(prs.slides) if title_of(s).startswith(PRE)), reverse=True): prs.part.drop_rel(ids[i].rId); ids.remove(ids[i])
    def pic(s, f, y, hmax):
        iw, ih = Image.open(f).size; w = min(W - 2 * 0.5, hmax * iw / ih); h = w * ih / iw; s.shapes.add_picture(str(f), Inches((W - w) / 2), Inches(y), Inches(w), Inches(h)); return h
    s = slide(prs, PRE + ": 한눈에 보는 결과", f"노트북의 실시간 SLAM과 이 PC의 인식을 연결해 영상 {len(full)}개 × 구성 여섯 가지를 돌리고, 같은 기록으로 지도를 네 방식으로 만들었다.", "")
    h = pic(s, MED / "glance.png", 1.6, 4.2)
    lines(s, LM, 1.6 + h + 0.1, CW, 1.4, [f"지금 방식이 이미 {ALL['V0']['kinds']}/{tot * len(cfgs)}종을 올린다. 설계 1은 깊이 지지율을 {ALL['V0']['sup']:.2f} → {ALL['V1']['sup']:.2f}로 올리지만 객체 {dd[0][0]}개를 빼면서 종류가 {ALL['V1']['kinds']}로 준다. 삭제 규칙이 지나치다.",
                                         f"설계 2는 뺄 것이 없었고(지지율 {ALL['V2']['sup']:.2f}), 설계 3은 새 종류를 거의 올리지 못한 채 초과 등록을 {ALL['V2']['extra']} → {ALL['V3']['extra']}개로 늘렸다. 구성 사이의 차이가 지도 방식 사이의 차이보다 크다."], sz=12.5, gap=3)
    s = slide(prs, PRE + ": 구성 여섯 가지와 지도 방식", "자세 추정과 자세 검증은 항상 켠다. 물체 판정은 모두 원본 점수 방식(기준 0.5)이다.", "")
    R = [["구성", "후보 생성", "마스크", "특징", "처리한 프레임", "프레임당 시간, 중앙값 (ms)"]] + [[f"{c}", "화면 전체 분할" if c <= 2 else "YOLO-World 문구 상자", "FastSAM-x" if c <= 4 else "MobileSAM", "ViT-L" if c % 2 else "ViT-S", f"{RN[c]['proc']:,}", round(RN[c]["ms"])] for c in cfgs]
    y = grid(s, R, [1.0, 3.2, 2.4, 1.4, 2.0, 2.13], 1.7, 0.42, 0.36, 12)
    R = [["지도 방식", "하는 일"], ["지금 방식", "같은 이름의 관측을 30 cm 안에서 묶어 위치는 평균, 방향은 다수결"], ["설계 1", "서로 가까운(25 mm, 20°) 자세의 가장 큰 무리만 쓰고, 검증 점수로 가중. 자세가 모이지 않으면 뺀다"],
         ["설계 2", "CAD를 주변 깊이 점에 다시 맞추고, 빈 곳에 뜬 객체(지지율 0.2 미만)를 뺀다"], ["설계 3", "검증에서 떨어진 후보도 증거로 쌓아 못 올라간 물체를 등록하고, 한 자리의 이름 충돌을 정리한다"]]
    grid(s, R, [2.2, 9.93], y + 0.25, 0.4, 0.36, 12, left=(0, 1))
    s = slide(prs, PRE + ": 구성별 · 지도 방식별 합계", f"영상 {len(full)}개 합계. 물체 종류는 최대 {tot}, 깊이 지지율은 객체가 실제 표면 위에 놓인 정도(1에 가까울수록 좋음)다.", "")
    R = [["구성", "항목"] + [VN[v] for v in VS]]
    for c in cfgs:
        R.append([CFG[c], "종류 / 초과"] + [f"{T[c][v]['kinds']} / {T[c][v]['extra']}" for v in VS]); R.append(["", "깊이 지지율"] + [f"{T[c][v]['sup']:.2f}" for v in VS])
    grid(s, R, [4.4, 1.9, 1.45, 1.45, 1.45, 1.48], 1.7, 0.4, 0.37, 12)
    ex = MED / "live_260901_cbnu_longcircle.jpg"
    if ex.exists():
        im = Image.open(ex); cut = Image.new("RGB", (640 * 2 + 12, 480), "white"); cut.paste(im.crop((2 * 648, 0, 2 * 648 + 640, 480)), (0, 0)); cut.paste(im.crop((2 * 648, 488, 2 * 648 + 640, 968)), (652, 0)); cut.save(MED / "example_design1.jpg", quality=90)
        s = slide(prs, PRE + ": 설계 1이 위치를 바꾼 예", "같은 실행 기록으로 지도만 다시 만들었다. 왼쪽이 지금 방식, 오른쪽이 설계 1(합의로 합치기)이다. 구성 3, 장소 A의 긴 원 주행.", "")
        h = pic(s, MED / "example_design1.jpg", 1.7, 4.4)
        lines(s, LM, 1.7 + h + 0.15, CW, 1.0, ["지금 방식에서 선반 왼쪽 아래에 치우쳐 있던 곰 인형이 설계 1에서는 실제 자리로 약 9 cm 옮겨졌다. 깊이 지지율이 0.6에서 1.0이 됐다.",
                                             "여러 프레임의 자세 가운데 서로 가까운 무리만 쓰면, 무리에서 벗어난 틀린 자세가 평균에 섞이지 않는다."], sz=13, gap=5)
    IDX = J(MED / "index_ppt.json") if (MED / "index_ppt.json").exists() else {}
    for sname in full:
        if sname not in IDX: continue
        s = slide(prs, PRE + f": {SES[sname]}", "지금 방식으로 만든 최종 객체 지도. 윗줄이 구성 1~3, 아랫줄이 구성 4~6이다. 이름표 옆 숫자는 깊이 지지율.", "")
        h = pic(s, MED / IDX[sname], 1.6, 5.3); text(s, LM, 1.6 + h + 0.06, CW, 0.3, [[("윤곽: 초록 = 그 종류에서 하나,  빨강 = 같은 종류의 초과 등록.  배경은 영상 전체로 복원한 장면.  설계별 지도는 보고서에 있다", {"sz": 11.5, "c": "444444"})]])
    prs.part.rename_slide_parts([x.rId for x in ids]); prs.save(str(OUT)); print("pptx slides", len(ids))


if __name__ == "__main__":
    main()
