#!/usr/bin/env python3
"""build_report.py — "성능 차이는 CAD 사용 여부 때문인가"를 확인한 두 실험의 보고서를 HTML 세 가지로 만든다.
  실험 1  FM-Fusion · DualMap 의 지도 객체에 제안 방법의 CAD 템플릿 검증으로 이름을 다시 붙여 같은 방식으로 채점
  실험 2  제안 방법에서 검증 단계를 뺀 경우
  cad_fairness_report.html / _local.html / _notion.html  (양식은 objmap_eval_261009 보고서와 같음)
수치는 objpose/output/reps_261009/rep*/ 에서 끝난 반복(REP_DONE)만 읽어 평균한다. 반복이 더 끝나면 다시 실행하면 된다.
    python3 objpose/docs/cad_fairness_261009/build_report.py"""
import collections, importlib.util, json, re, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]; O = REPO / "objpose/output"
spec = importlib.util.spec_from_file_location("objmap_rep", REPO / "objpose/docs/objmap_eval_261009/build_report.py"); B = importlib.util.module_from_spec(spec); spec.loader.exec_module(B)
Doc, REF = B.Doc, B.REF
J = lambda p: json.loads(Path(p).read_text())
RP = O / "reps_261009"; V = O / "fmf_cmp_260928/eval_own"; M = O / "fmf_cmp_ycbm_260929/eval_own"; OBE = O / "obchange_261005/eval"
ROOMS = [("kitchen", "Kitchen"), ("office", "Office"), ("livingarea", "LivingArea"), ("smallroom", "SmallRoom")]
REPS = sorted((p for p in RP.glob("rep*") if (p / "log.txt").exists() and "REP_DONE" in (p / "log.txt").read_text(errors="ignore")), key=lambda p: int(p.name[3:]))
PRO, NOV, FM, DM = "제안 방법", "SAM-6D + 같은 지도", "FM-Fusion", "DualMap"


def st(rows, thr=0.25):
    R = [r for r in rows if r["object"]]; named = sum(r["named"] for r in rows if not r["object"]); tp = sum(r["iou_name"] >= thr for r in R); n = len(R)
    p = tp / max(named, 1); rc = tp / max(n, 1)
    return dict(n=n, named=named, tp=tp, fp=named - tp, fn=n - tp, P=100 * p, R=100 * rc, F=100 * 2 * p * rc / max(p + rc, 1e-9))


def pick(rows, m): return [r for r in rows if r["method"] == m]


def load(ds):
    """→ {줄 이름: [반복마다 채점 줄 목록]} (원래 이름 줄은 저장된 한 번)"""
    if ds in ("ycbv", "ycbm"):
        base = J((V if ds == "ycbv" else M) / "inst_iou_final.json"); own = [J(r / ds / "inst_iou.json") for r in REPS]; cad = [J(r / ds / "inst_iou_cad.json") for r in REPS]
    else:
        rooms = [ds] if ds != "obchange" else [k for k, _ in ROOMS]
        base = [x for k in rooms for x in J(OBE / f"{k}_final.json")]
        own = [[x for k in rooms for x in pick(J(r / "obchange" / f"proposed_{k}.json"), PRO)] + [dict(x, method=NOV) for k in rooms for x in pick(J(r / "obchange" / f"noverify_{k}.json"), PRO)] for r in REPS]
        cad = [[x for k in rooms for x in J(r / "obchange" / f"cad_{k}.json") if x["method"] != PRO] for r in REPS]
    return {"pro": [pick(x, PRO) for x in own], "nov": [pick(x, NOV) for x in own], "fm0": [pick(base, FM)], "dm0": [pick(base, DM)], "fm": [pick(x, FM) for x in cad], "dm": [pick(x, DM) for x in cad]}


def agg(runs, thr=0.25):
    S = [st(x, thr) for x in runs]; out = {k: float(np.mean([s[k] for s in S])) for k in S[0]}; out["Fmin"] = min(s["F"] for s in S); out["Fmax"] = max(s["F"] for s in S); out["k"] = len(S); out["Fs"] = [s["F"] for s in S]
    return out


def num(v): return int(round(v)) if abs(v - round(v)) < 1e-9 else round(float(v), 1)
def rng(a): return f"{a['Fmin']:.1f}–{a['Fmax']:.1f}" if a["k"] > 1 else "1회"


def row(name, a, a5): return [name, num(a["named"]), num(a["tp"]), num(a["fp"]), num(a["fn"]), float(a["P"]), float(a["R"]), float(a["F"]), rng(a), float(a5["F"])]


HEAD = ["방법", "이름 붙은 객체", "맞음 (TP)", "잘못 등록 (FP)", "놓침 (FN)", "정밀도 (%)", "재현율 (%)", "F1 (%)", "F1 반복 범위", "F1, 겹침 0.5 기준 (%)"]
DSN = [("ycbv", "YCB-Video", "12장면, 정답 물체 55개"), ("ycbm", "YCB-M", "32장면, 정답 물체 165개"), ("obchange", "ObChange 네 방", "방문 20회, 정답 물체 74개")]


def changes():
    """첫 반복에서 이름이 어떻게 바뀌었는지: {(데이터, 방법): Counter}"""
    out = collections.defaultdict(collections.Counter); r = REPS[0]
    kind = lambda x, y: "같은 이름 유지" if x == y and x is not None else ("이름 없음 유지" if x is None and y is None else ("이름 없음 → 이름" if x is None else ("이름 → 이름 없음" if y is None else "다른 이름으로 바뀜")))
    for ds in ("ycbv", "ycbm"):
        for f in (r / ds / "dump_cad").glob("*.json"):
            if f.stem == "stream_stats": continue
            D = J(f)
            for m in (FM, DM):
                for o in D["methods"].get(m, []): out[(ds, m)][kind(o["old_name"], o["name"])] += 1
    tab = J(REPO / "objpose/rt/bop/ycbv_objects.json")["objects"]; by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["ycb"] for o in tab}; cat = set(B.KOR)
    names = J(r / "obchange/cad/fmf_names.json")
    for sc, mp in names.items():
        for line in (O / "obchange_261005/fmfusion" / sc / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip(): continue
            f = line.split(";"); i = str(int(f[0]))
            if i in mp: old = by_label.get(f[1].split("(")[0]); out[("obchange", FM)][kind(old if old in cat else None, mp[i])] += 1
        for o in J(r / "obchange/cad/dualmap" / f"{sc}.json"): out[("obchange", DM)][kind(o["old_name"] if o["old_name"] in cat else None, o["name"])] += 1
    return out


def stream():
    out = {}
    for ds, f in (("ycbv", "ycbv/dump_cad/stream_stats.json"), ("ycbm", "ycbm/dump_cad/stream_stats.json"), ("obchange", "obchange/cad/stream_stats.json")):
        S = J(REPS[0] / f)
        for m in (FM, DM):
            x = [s for s in S if s["method"] == m and s["ms_mean"]]
            out[(ds, m)] = dict(off=sum(s["offered"] for s in x), done=sum(s["processed"] for s in x), ms=float(np.average([s["ms_mean"] for s in x], weights=[s["processed"] for s in x])), iv=x[0]["interval_ms"], obj=float(np.mean([s["objects"] for s in x])))
    return out


def run_stats(ds, tag):
    fr = pr = 0
    for f in (REPS[0] / ds / tag).glob("*/run.json"):
        r = J(f); fr += r["frames"]; pr += r["processed"]
    return fr, pr


def chart(D):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib import font_manager as fmg
    fp = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"; fmg.fontManager.addfont(fp); plt.rcParams["font.family"] = fmg.FontProperties(fname=fp).get_name()
    keys = [("pro", "제안 방법", "#1f5fa8"), ("fm0", "FM-Fusion", "#c9c9c9"), ("fm", "FM-Fusion + CAD 이름", "#e08a1e"), ("dm0", "DualMap", "#e3e3e3"), ("dm", "DualMap + CAD 이름", "#7a9a3a"), ("nov", "제안 방법, 검증 없음", "#b03a3a")]
    fig, ax = plt.subplots(figsize=(12, 4.2), dpi=130); w = 0.13
    for i, (k, lab, col) in enumerate(keys):
        vals = [agg(D[ds][k])["F"] for ds, _, _ in DSN]; x = np.arange(len(DSN)) + (i - 2.5) * w; ax.bar(x, vals, w * 0.92, label=lab, color=col, edgecolor="#555", linewidth=0.5)
        for xi, v in zip(x, vals): ax.text(xi, v + 1, f"{v:.0f}", ha="center", va="bottom", fontsize=9)
    ax.set_xticks(np.arange(len(DSN))); ax.set_xticklabels([n for _, n, _ in DSN], fontsize=12); ax.set_ylabel("F1 (%)"); ax.set_ylim(0, 105); ax.legend(ncol=3, fontsize=10, frameon=False, loc="upper right")
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    fig.tight_layout(); p = HERE / "media"; p.mkdir(exist_ok=True); fig.savefig(p / "f1_bars.png"); plt.close(fig); return p / "f1_bars.png"


def main():
    if not REPS: sys.exit("끝난 반복이 없습니다")
    K = len(REPS); D = {ds: load(ds) for ds, _, _ in DSN}; A = {ds: {k: agg(v) for k, v in D[ds].items()} for ds in D}; A5 = {ds: {k: agg(v, 0.5) for k, v in D[ds].items()} for ds in D}
    d = Doc(); F = lambda ds, k: f"{A[ds][k]['F']:.1f}"

    d.h2("1. 요약")
    d.table(["질문", "결과"], [
        ["<strong>비교 방법에 CAD를 주면 이름 인식 차이가 사라지는가</strong>",
         f"탁상 장면에서는 사라집니다. FM-Fusion의 지도 객체에 CAD 템플릿 검증으로 이름을 다시 붙이면 F1이 YCB-Video {F('ycbv','fm0')} → {F('ycbv','fm')} %, YCB-M {F('ycbm','fm0')} → {F('ycbm','fm')} %로 올라 제안 방법({F('ycbv','pro')} %, {F('ycbm','pro')} %)과 같거나 높습니다. "
         f"DualMap도 {F('ycbv','dm0')} → {F('ycbv','dm')} %, {F('ycbm','dm0')} → {F('ycbm','dm')} %로 오릅니다."],
        ["<strong>방 규모 장면에서도 그런가</strong>",
         f"아닙니다. ObChange 네 방에서는 CAD 이름을 붙여도 FM-Fusion {F('obchange','fm')} %, DualMap {F('obchange','dm')} %로 제안 방법({F('obchange','pro')} %)보다 낮습니다. 맞게 올린 물체는 비교 방법이 더 많지만 "
         f"잘못 등록된 객체가 {num(A['obchange']['fm']['fp'])}개, {num(A['obchange']['dm']['fp'])}개로 제안 방법({num(A['obchange']['pro']['fp'])}개)의 몇 배입니다."],
        ["<strong>제안 방법의 검증 단계는 효과가 있는가</strong>",
         f"있습니다. 검증을 빼면 F1이 YCB-Video {F('ycbv','pro')} → {F('ycbv','nov')} %, YCB-M {F('ycbm','pro')} → {F('ycbm','nov')} %, ObChange {F('obchange','pro')} → {F('obchange','nov')} %로 떨어집니다. "
         f"맞게 올린 물체 수는 비슷한데 잘못 등록된 객체가 YCB-M {num(A['ycbm']['pro']['fp'])} → {num(A['ycbm']['nov']['fp'])}개, ObChange {num(A['obchange']['pro']['fp'])} → {num(A['obchange']['nov']['fp'])}개로 늘어납니다."],
        ["<strong>그러면 논문에서 무엇을 주장할 수 있는가</strong>",
         "이름 인식이 앞서는 이유는 CAD 정보라고 인정하고, 주장은 두 가지로 좁히는 것이 맞습니다. 첫째, CAD를 쓰면서도 물체별 학습 없이 지도에 6D 자세를 저장한다는 점. 둘째, 검증 단계 덕분에 물체가 많은 방에서도 잘못 등록되는 객체가 적다는 점입니다."]],
        cls="sum", widths=["26%", None])
    d.fig(chart(D), f"F1 비교(관측 점 기준 겹침 0.25 이상을 맞음으로 침). 회색은 원래 이름, 색 막대는 CAD로 이름을 다시 붙인 경우. 반복 {K}회 평균", width=1500)

    d.h2("2. 왜 이 실험을 했는가")
    d.p("제안 방법이 FM-Fusion, DualMap보다 좋게 나온 이유가 방법의 차이가 아니라 CAD 모델을 쓰느냐의 차이라는 의견이 있었습니다. 제안 방법은 물체의 CAD 모델을 입력으로 받고, 두 비교 방법은 물체 이름(문구)만 받습니다. 이 의견이 맞는지 두 가지 실험으로 확인했습니다.")
    d.dl([("실험 1. 비교 방법에 CAD를 준다", "FM-Fusion과 DualMap이 만든 지도 객체(점 덩어리)는 그대로 두고, 이름만 제안 방법의 CAD 템플릿 검증으로 다시 붙입니다. 이름 인식의 차이가 CAD 때문이라면 이 실험에서 차이가 사라져야 합니다."),
          ("실험 2. 제안 방법에서 검증을 뺀다", "제안 방법의 후보 검증(CAD 템플릿 비교의 문턱)과 자세 검증을 끄고, 나머지(문구 검출기, 자세 추정, 지도 구성)는 그대로 둡니다. CAD만 쓰면 되는 것인지, 검증 단계가 실제로 기여하는지를 봅니다.")])

    d.h2("3. 실험 조건")
    fr = {ds: run_stats(ds, "proposed") for ds in D}; fn = {ds: run_stats(ds, "noverify") for ds in D}; S = stream()
    d.dl([("데이터", f"YCB-Video 12장면(20,738프레임, 30 fps), YCB-M 32장면(7,252프레임, 6 fps), ObChange 네 방(Kitchen, Office, LivingArea, SmallRoom)의 정답이 있는 방문 20회({fr['obchange'][0]:,}프레임, 10 fps). 일부만 고르지 않고 전부 썼습니다."),
          ("실시간 조건", "영상이 원래 속도로 도착한다고 보고, 추론이 끝난 순간 이미 도착한 프레임 가운데 가장 최신 것 하나만 처리합니다. 그 사이 프레임은 버립니다. "
           f"제안 방법은 YCB-Video {fr['ycbv'][0]:,}프레임 중 {fr['ycbv'][1]:,}장, YCB-M {fr['ycbm'][0]:,}프레임 중 {fr['ycbm'][1]:,}장, ObChange {fr['obchange'][0]:,}프레임 중 {fr['obchange'][1]:,}장을 처리했습니다(첫 반복)."),
          ("반복", f"전체를 {K}회 반복했습니다. 표의 값은 반복 평균이고, F1은 반복 사이의 최소–최대를 함께 적었습니다. 원래 이름의 FM-Fusion, DualMap 줄은 이전에 저장한 한 번의 결과입니다."),
          ("새로 돌린 것", "제안 방법, 검증을 뺀 제안 방법, CAD 이름 판별. 반복마다 처음부터 다시 실행했습니다."),
          ("그대로 쓴 것", "카메라 자세(YCB는 ORB-SLAM3 저장 결과, ObChange는 데이터셋 제공값)와 FM-Fusion, DualMap이 만든 지도. 두 방법을 CAD 입력용으로 고쳐 다시 돌린 것이 아닙니다."),
          ("채점", "이전 보고서와 같습니다. 카메라가 실제로 본 점을 지도 객체에 배정(2 cm)하고, 정답 물체와 같은 이름의 객체가 0.25 이상 겹치면 맞음(TP)입니다. 대상 물체 이름이 붙었는데 맞음이 아닌 객체는 잘못 등록(FP), 맞는 객체가 없는 정답은 놓침(FN)입니다.")])
    d.h3("CAD 이름을 붙이는 방법")
    d.ul(["각 방법이 지도에 프레임을 넣는 주기(초당 3~5회)에 맞춰 이름 판별을 한 번씩 합니다. 판별이 주기보다 오래 걸리면 밀린 프레임은 버리고 최신 프레임으로 넘어갑니다.",
          "지도 객체의 점을 그 프레임에 투영해 2D 상자를 만듭니다. 깊이 영상과 3 cm 안에서 맞는 점만 써서 그 프레임에서 실제로 보이는 부분만 남깁니다.",
          "상자들을 제안 방법의 후보 검증에 그대로 넣습니다. DINOv2 특징으로 CAD 템플릿과 비교하고, MobileSAM 마스크의 외형 유사도까지 통과해야 그 물체로 인정합니다. 문턱값은 제안 방법과 같습니다. 문구 검출기는 쓰지 않습니다.",
          "객체마다 통과한 이름을 모아, 가장 많이 나온 이름이 3번 이상이고 그 객체가 보인 프레임의 10 % 이상이면 그 이름을 붙입니다. 아니면 이름 없는 객체로 둡니다.",
          "찾을 물체 목록은 제안 방법과 같습니다. YCB는 그 장면에 있는 물체, ObChange는 11종 전부입니다."])
    d.table(["데이터", "방법", "장면당 지도 객체 수", "판별 주기 (ms)", "판별 한 번 평균 (ms)", "판별한 프레임 / 주기에 맞춰 도착한 프레임"],
            [[n, m, round(S[(ds, m)]["obj"], 1), int(S[(ds, m)]["iv"]), int(round(S[(ds, m)]["ms"])), f"{S[(ds, m)]['done']:,} / {S[(ds, m)]['off']:,}"] for ds, n, _ in DSN for m in (FM, DM)], cls="sum")
    d.p("판별 한 번은 대부분 주기 안에 끝납니다. 프레임을 버린 것은 주로 객체가 많은 장면의 DualMap입니다.", "note")

    d.h2("4. 실험 1: 비교 방법에 CAD로 이름을 붙였을 때")
    grp = lambda ri, r: "hl" if ri == 0 else ""
    for ds, n, sub in DSN:
        d.h3(f"{n} ({sub})")
        d.table(HEAD, [row("제안 방법", A[ds]["pro"], A5[ds]["pro"]), row("FM-Fusion (원래 이름)", A[ds]["fm0"], A5[ds]["fm0"]), row("FM-Fusion + CAD 이름", A[ds]["fm"], A5[ds]["fm"]),
                       row("DualMap (원래 이름)", A[ds]["dm0"], A5[ds]["dm0"]), row("DualMap + CAD 이름", A[ds]["dm"], A5[ds]["dm"])], cls="sum", bars=(5, 6, 7))
    d.h3("ObChange 방별 F1 (%)")
    RD = {k: load(k) for k, _ in ROOMS}
    d.table(["방", "정답 물체", "제안 방법", "FM-Fusion (원래)", "FM-Fusion + CAD", "DualMap (원래)", "DualMap + CAD"],
            [[nm, agg(RD[k]["pro"])["n"] and num(agg(RD[k]["pro"])["n"])] + [float(agg(RD[k][c])["F"]) for c in ("pro", "fm0", "fm", "dm0", "dm")] for k, nm in ROOMS], cls="sum", bars=(2, 3, 4, 5, 6))
    C = changes(); KS = ["같은 이름 유지", "다른 이름으로 바뀜", "이름 없음 → 이름", "이름 → 이름 없음", "이름 없음 유지"]
    d.h3("이름이 어떻게 바뀌었는가 (지도 객체 수, 첫 반복)")
    d.table(["데이터", "방법"] + KS, [[n, m] + [C[(ds, m)][k] for k in KS] for ds, n, _ in DSN for m in (FM, DM)], cls="sum")
    a = A["ycbv"]; b = A["ycbm"]; o = A["obchange"]
    d.h3("읽는 법")
    d.ul([f"<strong>탁상 장면(YCB)에서는 차이가 사라집니다.</strong> CAD 이름을 붙인 FM-Fusion은 놓친 물체가 YCB-Video {num(a['fm']['fn'])}개, YCB-M {num(b['fm']['fn'])}개로 제안 방법({num(a['pro']['fn'])}개, {num(b['pro']['fn'])}개)보다 적습니다. "
          "물체를 한 덩어리로 잘 떼어 내기만 하면, 그 덩어리가 무엇인지는 CAD 템플릿이 거의 틀리지 않고 알려 준다는 뜻입니다. 이름 인식에서 제안 방법이 앞섰던 것은 CAD 정보의 덕이라는 의견이 맞습니다.",
          f"<strong>잘못 등록된 객체도 크게 줄어듭니다.</strong> YCB-M에서 FM-Fusion은 {num(b['fm0']['fp'])} → {num(b['fm']['fp'])}개, DualMap은 {num(b['dm0']['fp'])} → {num(b['dm']['fp'])}개입니다. 원래 이름에서는 닮은 물체끼리 이름이 섞였는데, CAD 템플릿은 이를 구분합니다.",
          f"<strong>겹침 기준을 0.5로 올리면 순서가 달라집니다.</strong> YCB-Video에서 제안 방법 {A5['ycbv']['pro']['F']:.1f} %, FM-Fusion + CAD {A5['ycbv']['fm']['F']:.1f} %, DualMap + CAD {A5['ycbv']['dm']['F']:.1f} %입니다. 제안 방법은 CAD 모양 전체를 지도에 놓으므로 겹침이 크고, 점군 객체는 일부만 잡히거나 조각으로 나뉜 경우가 있습니다. YCB-M에서는 이 기준에서도 제안 방법 {A5['ycbm']['pro']['F']:.1f} %, FM-Fusion + CAD {A5['ycbm']['fm']['F']:.1f} %로 비슷합니다.",
          f"<strong>방 규모(ObChange)에서는 CAD 이름만으로 부족합니다.</strong> 비교 방법은 방 안의 가구, 벽, 잡동사니까지 객체로 만들고, 그중 일부가 CAD 검증을 통과해 대상 물체 이름을 얻습니다. 잘못 등록된 객체가 FM-Fusion {num(o['fm']['fp'])}개, DualMap {num(o['dm']['fp'])}개, 제안 방법 {num(o['pro']['fp'])}개입니다. "
          "제안 방법은 문구 검출 → CAD 템플릿 비교 → 자세 추정 → 자세 검증을 모두 통과하고 여러 프레임에서 같은 자세로 모여야 지도에 올리므로 이런 객체가 적습니다.",
          f"<strong>방별로 보면 Kitchen은 예외입니다.</strong> Kitchen에서는 FM-Fusion + CAD가 {agg(RD['kitchen']['fm'])['F']:.1f} %로 제안 방법({agg(RD['kitchen']['pro'])['F']:.1f} %)과 비슷하거나 높습니다. 물체가 조리대 위에 모여 있어 탁상 장면에 가깝습니다. 나머지 세 방에서는 제안 방법이 높습니다. "
          f"또 ObChange에서 겹침 기준을 0.5로 올리면 제안 방법 {A5['obchange']['pro']['F']:.1f} %, FM-Fusion + CAD {A5['obchange']['fm']['F']:.1f} %로 제안 방법이 앞서지 못합니다. 이 데이터의 카메라 자세 오차 때문에 제안 방법이 놓은 CAD가 실제 표면과 수 cm 어긋나는 경우가 많습니다.",
          "<strong>CAD 이름을 붙여도 생기지 않는 것은 방향입니다.</strong> 비교 방법의 객체는 여전히 점 덩어리입니다. 이전 보고서에서 이 점군에 CAD를 사후에 맞춰 보았을 때 방향이 30° 넘게 틀린 비율이 YCB-Video에서 FM-Fusion 47 %, DualMap 67 %, 제안 방법 0 %였습니다."])

    d.h2("5. 실험 2: 제안 방법에서 검증 단계를 뺐을 때")
    d.p("검증을 뺀 경우는 문구 검출기가 낸 상자를 CAD 템플릿 비교의 문턱 없이 받아들이고, 추정한 자세를 검증 없이 지도에 넣습니다. 지도 쪽 설정(같은 물체로 묶는 거리, 깊이 검사)은 같습니다.")
    PH = HEAD[:8] + ["F1 반복 범위"]
    for ds, n, sub in DSN:
        d.h3(f"{n} ({sub})"); rows = [row("제안 방법", A[ds]["pro"], A5[ds]["pro"])[:9], row("제안 방법, 검증 없음", A[ds]["nov"], A5[ds]["nov"])[:9]]
        if ds in ("ycbv", "ycbm"):
            ref = st(pick(J(RP / "orig_ref" / f"{ds}_inst_iou.json"), NOV)); rows.append(["(참고) 원본 SAM-6D + 같은 지도, 저장된 1회", ref["named"], ref["tp"], ref["fp"], ref["fn"], ref["P"], ref["R"], ref["F"], "1회"])
        d.table(PH, rows, cls="sum", bars=(5, 6, 7))
    d.h3("지도에 저장된 6D 자세 (YCB, 맞게 올린 객체 대상)")
    pr = []
    for ds, n, _ in DSN[:2]:
        for tag, lab in (("proposed", "제안 방법"), ("noverify", "제안 방법, 검증 없음")):
            P = [J(r / ds / f"pose_{tag}.json")["summary"] for r in REPS]; mean = lambda k: float(np.mean([p[k] for p in P]))
            pr.append([n, lab, round(mean("n_tp"), 1), mean("add_s_mm_p50"), mean("add_s_pass_of_tp"), mean("add_s_pass_of_gt"), mean("rot_deg_p50")])
    d.table(["데이터", "방법", "대상 객체 수", "ADD(-S) 중앙값 (mm)", "ADD(-S) 통과, 대상 중 (%)", "ADD(-S) 통과, 정답 전체 중 (%)", "방향 오차 중앙값 (°)"], pr, cls="sum", bars=(4, 5))
    a, b, o = A["ycbv"], A["ycbm"], A["obchange"]
    d.h3("읽는 법")
    d.ul([f"<strong>검증을 빼도 맞게 올리는 물체는 늘지 않습니다.</strong> YCB-Video {num(a['pro']['tp'])} → {num(a['nov']['tp'])}개, YCB-M {num(b['pro']['tp'])} → {num(b['nov']['tp'])}개, ObChange {num(o['pro']['tp'])} → {num(o['nov']['tp'])}개입니다.",
          f"<strong>잘못 등록되는 객체가 크게 늡니다.</strong> YCB-Video {num(a['pro']['fp'])} → {num(a['nov']['fp'])}개, YCB-M {num(b['pro']['fp'])} → {num(b['nov']['fp'])}개, ObChange {num(o['pro']['fp'])} → {num(o['nov']['fp'])}개입니다. 같은 물체가 조금 다른 자세로 여러 번 올라가거나, 물체가 아닌 곳에 CAD가 놓입니다.",
          "<strong>ObChange에서 차이가 가장 큽니다.</strong> 11종을 모두 찾게 하면 방 안의 다른 물건에 대상 물체 이름이 붙습니다. 검증이 없으면 CAD를 쓰더라도 지도를 쓸 수 없는 수준이 됩니다. CAD를 쓰는 것만으로 결과가 좋아지는 것이 아니라는 근거입니다.",
          "<strong>참고 줄(원본 SAM-6D)은 조건이 다릅니다.</strong> 원본 SAM-6D는 영상 전체를 분할한 뒤 CAD 템플릿 점수로 물체마다 최고 후보 하나를 고릅니다. 장면에 있는 물체만 찾게 한 YCB에서는 이 방식이 놓치는 물체가 적어 F1이 제안 방법보다 높습니다. 대신 처리 속도가 초당 0.6장 수준이고 잘못 등록된 객체가 더 많습니다. 이전에 저장한 한 번의 결과이며 이번에 다시 돌리지 않았습니다."])

    d.h2("6. 반복 사이의 차이")
    hd = ["데이터", "방법"] + [f"{i + 1}회차 F1 (%)" for i in range(K)] + ["최대 − 최소 (%p)"]
    d.table(hd, [[n, lab] + [float(x) for x in A[ds][k]["Fs"]] + [float(A[ds][k]["Fmax"] - A[ds][k]["Fmin"])] for ds, n, _ in DSN for k, lab in (("pro", "제안 방법"), ("nov", "제안 방법, 검증 없음"), ("fm", "FM-Fusion + CAD 이름"), ("dm", "DualMap + CAD 이름"))], cls="sum")
    d.p("실시간 조건에서는 추론 시간에 따라 처리되는 프레임이 반복마다 조금씩 달라집니다. 위 표는 그 영향으로 F1이 얼마나 움직이는지를 보여 줍니다.", "note")

    d.h2("7. 해석")
    d.h3("인정해야 하는 것")
    d.ul(["이름 인식에서 제안 방법이 FM-Fusion, DualMap을 앞선 것은 CAD 정보 덕분입니다. 탁상 장면에서는 비교 방법에 같은 CAD 검증을 붙이면 F1이 제안 방법과 같거나 높아집니다.",
          "제안 방법은 놓치는 물체가 더 많습니다. 문구 검출기가 상자를 내지 못하면 뒤 단계가 아무리 좋아도 지도에 올라가지 않습니다. 점군 지도는 물체를 덩어리로 먼저 떼어 내므로 이 단계에서 유리합니다.",
          "따라서 \"문구만 쓰는 방법보다 인식이 정확하다\"는 식의 주장은 공정한 비교가 아닙니다."])
    d.h3("주장할 수 있는 것")
    d.ul(["CAD를 쓰되 물체별 학습 없이, 지도에 물체의 위치와 방향(6D 자세)을 저장합니다. 비교 방법에 CAD로 이름을 붙여도 방향은 생기지 않습니다.",
          "검증 단계가 지도의 신뢰도를 만듭니다. 같은 CAD, 같은 자세 추정기를 쓰더라도 검증이 없으면 잘못 등록된 객체가 수십 배로 늘어납니다.",
          "물체가 많고 넓은 방에서도 잘못 등록되는 객체가 적습니다. CAD 이름을 붙인 비교 방법보다 ObChange의 F1이 높습니다.",
          "실시간으로 동작합니다. 위 수치는 모두 영상이 원래 속도로 들어올 때의 결과입니다."])
    d.h3("다음에 하면 좋은 것")
    d.ul(["CAD를 쓰는 다른 객체 지도 방법과의 직접 비교. 조사 보고서(<code>objpose/docs/cad_objmap_survey_261009/</code>)에서 고른 후보는 모두 물체별 학습이 필요합니다. 그 차이(학습 유무)를 표로 보이는 것이 CAD 사용 여부 논란에 대한 가장 직접적인 답입니다.",
          "점군 지도가 떼어 낸 덩어리를 제안 방법의 후보로 쓰는 결합. 실험 1은 이 방향이 놓치는 물체를 줄일 수 있음을 보여 줍니다."])

    d.h2("8. 한계")
    d.ul(["실험 1은 비교 방법을 CAD 입력용으로 고쳐 다시 돌린 것이 아니라, 이미 만든 지도의 이름만 다시 붙인 것입니다. 이름 판별이 지도 구성(객체 병합 등)에 주는 영향은 반영되지 않습니다.",
          "이름 판별은 완성된 지도의 객체를 처음부터 투영합니다. 실제로는 지도가 만들어지는 중이므로 앞부분 프레임에서는 객체가 덜 모였을 것입니다. 비교 방법에 유리한 쪽의 단순화입니다.",
          "YCB의 투영에는 데이터셋의 정답 카메라 자세를 썼습니다(지도를 정답 좌표계로 옮긴 뒤 투영). 이것도 비교 방법에 유리한 쪽입니다.",
          "이름을 붙이는 기준(3번 이상, 보인 프레임의 10 % 이상)은 이 실험에서 정한 값이고 따로 조정하지 않았습니다.",
          "원래 이름의 FM-Fusion, DualMap과 카메라 자세는 한 번의 결과입니다. 반복은 제안 방법, 검증 없는 경우, 이름 판별에만 적용했습니다.",
          "ObChange의 정답 물체는 방마다 10~25개로 적어 방별 수치는 물체 한두 개에 크게 움직입니다."])

    d.h2("9. 결과 파일")
    d.ul(["실행: <code>objpose/tools/analysis/run_reps_261009.sh &lt;반복 번호&gt;</code> → <code>objpose/output/reps_261009/rep&lt;번호&gt;/</code> (진행 기록 <code>log.txt</code>)",
          "CAD 이름 판별: <code>objpose/tools/analysis/cad_relabel.py</code> → <code>ycbv|ycbm/dump_cad/</code>, <code>obchange/cad/</code> (판별 시간 <code>stream_stats.json</code>)",
          "채점: <code>inst_iou.py</code> → <code>inst_iou.json</code>(제안, 검증 없음), <code>inst_iou_cad.json</code>(CAD 이름), ObChange는 <code>inst_iou_obchange.py</code> → <code>proposed_&lt;방&gt;.json</code>, <code>noverify_&lt;방&gt;.json</code>, <code>cad_&lt;방&gt;.json</code>",
          "6D 자세: <code>pose_metrics.py</code> → <code>pose_proposed.json</code>, <code>pose_noverify.json</code>", "원본 SAM-6D 참고 줄: <code>objpose/output/reps_261009/orig_ref/</code>",
          "이전 보고서: <code>objpose/docs/objmap_eval_261009/</code>(세 방법 비교), <code>objpose/docs/cad_objmap_survey_261009/</code>(CAD 기반 방법 조사)", "이 보고서: <code>objpose/docs/cad_fairness_261009/build_report.py</code>"])

    ref = REF.read_text(); head = ref[: ref.index("<div class=\"wrap\">")]
    head = head.replace("<title>YCB 조합 실험 보고</title>", "<title>CAD 사용 여부 확인 실험</title>").replace("</style>", "figure{margin:6px 0 4px}figure img{max-width:100%;height:auto;border:1px solid var(--line);border-radius:4px;display:block;background:#fff}figcaption{margin-top:4px;margin-bottom:10px}</style>")
    hdr = ('<header><div class="kicker">실험 보고 · CAD 사용 여부 확인 · 2026-10-09</div><h1>성능 차이는 CAD 사용 여부 때문인가</h1>'
           f'<p class="lead">제안 방법이 FM-Fusion, DualMap보다 좋은 이유가 CAD 모델을 쓰기 때문이라는 의견을 두 실험으로 확인했습니다. 비교 방법의 지도 객체에 CAD로 이름을 다시 붙여 보았고, 제안 방법에서 검증 단계를 빼 보았습니다. '
           f'YCB-Video, YCB-M, ObChange 네 방 전체를 실시간 재생 조건으로 {K}회 반복한 결과입니다.</p></header>')
    body = hdr + "\n" + "\n".join(x for x, _ in d.parts)
    (HERE / "cad_fairness_report.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "cad_fairness_report_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "CAD 사용 여부 확인 실험")
    nb = ("<p>실험 보고 · CAD 사용 여부 확인 · 2026-10-09</p>" + re.sub(r'<header>.*?</div>', "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "")) + "\n" + "\n".join(y for _, y in d.parts)
    (HERE / "cad_fairness_report_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    for f in ("cad_fairness_report.html", "cad_fairness_report_local.html", "cad_fairness_report_notion.html"): print(f, (HERE / f).stat().st_size)
    for ds, n, _ in DSN: print(n, {k: (round(v["F"], 1), num(v["tp"]), num(v["fp"]), num(v["fn"])) for k, v in A[ds].items()})


if __name__ == "__main__":
    main()
