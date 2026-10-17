#!/usr/bin/env python3
"""build_report.py — ObChange 3개 방에서 자세 추정 구성 7개와 FM-Fusion · DualMap(원래 이름 / CAD 이름)의 최종 객체 지도를 비교한 보고서.
  combo_obchange_report.html / _local.html / _notion.html (양식은 objmap_eval_261009 보고서와 같음), registered_objects.csv (등록 객체 전체 목록)
입력: objpose/output/combo_obchange_261009/ (run_combo_obchange.sh, run_base_rt_obchange.sh) 와 objlist.json (objpose/tools/obchange/objlist_combo.py). 끝난 방법만 표에 넣는다.
    ~/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/objlist_combo.py --root objpose/output/combo_obchange_261009 --out objpose/output/combo_obchange_261009/objlist.json
    python3 objpose/docs/combo_obchange_261009/build_report.py"""
import collections, csv, importlib.util, json, re, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]; O = REPO / "objpose/output/combo_obchange_261009"
spec = importlib.util.spec_from_file_location("objmap_rep", REPO / "objpose/docs/objmap_eval_261009/build_report.py"); B = importlib.util.module_from_spec(spec); spec.loader.exec_module(B)
Doc, REF, KOR = B.Doc, B.REF, dict(B.KOR)
J = lambda p: json.loads(Path(p).read_text())
ROOMS = [("office", "Office"), ("livingarea", "LivingArea"), ("smallroom", "SmallRoom")]
POSE = ["A", "B", "C_on", "C_off", "D", "E_on", "E_off"]; BASE = ["FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"]
LAB = {"A": "A. 원본 SAM-6D", "B": "B. 화면 전체 분할 + 원본 판정 + 검증", "C_on": "C. 문구 상자 + 원본 판정 + 검증", "C_off": "C′. C에서 검증 끔", "D": "D. 제안 방법, 완화 설정",
       "E_on": "E. 제안 방법, 배포 설정", "E_off": "E′. E에서 검증 끔", "FM-Fusion": "FM-Fusion", "FM-Fusion + CAD": "FM-Fusion + CAD 이름", "DualMap": "DualMap", "DualMap + CAD": "DualMap + CAD 이름"}
SHORT = {"A": "A", "B": "B", "C_on": "C", "C_off": "C′", "D": "D", "E_on": "E", "E_off": "E′", "FM-Fusion": "FMF", "FM-Fusion + CAD": "FMF+CAD", "DualMap": "DM", "DualMap + CAD": "DM+CAD"}
ORDER = ["A", "B", "C_on", "C_off", "D", "E_on", "E_off"] + BASE
VERD = ["맞음", "중복", "다른 물체 자리", "정답 없는 자리"]
ko = lambda n: KOR.get(n, n)


def proc(m):
    """(처리한 프레임, 도착한 프레임, 프레임당 처리 시간 중앙값 ms)"""
    if m in POSE:
        R = [J(f) for f in (O / m).glob("*/run.json")]; return sum(r["processed"] for r in R), sum(r["frames"] for r in R), float(np.median([r["ms_median"] for r in R])) if R else None
    if m.startswith("FM-Fusion"):
        fd = Path.home() / "DeepLearning/Datasets/fmfusion_obchange_261005/scans"; T = [J(f) for k, _ in ROOMS for n in range(2, 7) for f in [fd / f"{k}_s{n}/prediction_gsam_rt/timing.json"] if f.exists()]
        fr = sum(len(J(Path("/home/jucpark/DeepLearning/Datasets/ObChange/bop/test") / f"{k}_s{n}/scene_camera.json")) for k, _ in ROOMS for n in range(2, 7))
        return sum(t["frames"] for t in T), fr, float(np.mean([t["gdino_ms"] + t["sam_ms"] for t in T])) if T else None
    done = tot = 0
    for f in (O / "dualmap_rt/combo_rt_261009_in").glob("*_s?.log"):
        x = re.findall(r"REALTIME processed (\d+) of (\d+)", f.read_text(errors="ignore"))
        if x: done += int(x[-1][0]); tot += int(x[-1][1]) * 2
    return done, tot, None


LO = REPO / "objpose/output/lab_combo_261010"; LMED = HERE / "media_lab"
LPOSE = ["A", "B", "C", "C_off", "D", "E", "E_off"]; LORDER = LPOSE + BASE
LLAB = {"A": "A. 원본 SAM-6D", "B": "B. 화면 전체 분할 + 원본 판정 + 검증", "C": "C. 문구 상자 + 원본 판정 + 검증", "C_off": "C′. C에서 검증 끔", "D": "D. 제안 방법, 완화 설정", "E": "E. 제안 방법, 배포 설정", "E_off": "E′. E에서 검증 끔",
        "FM-Fusion": "FM-Fusion", "FM-Fusion + CAD": "FM-Fusion + CAD 이름", "DualMap": "DualMap", "DualMap + CAD": "DualMap + CAD 이름"}
LSHORT = {"A": "A", "B": "B", "C": "C", "C_off": "C′", "D": "D", "E": "E", "E_off": "E′", "FM-Fusion": "FMF", "FM-Fusion + CAD": "FMF+CAD", "DualMap": "DM", "DualMap + CAD": "DM+CAD"}
LKO = {"milk": "우유", "choco_hazelnut_high": "초코 상자", "Febreze_high": "페브리즈", "Mugcup_high": "머그컵", "saffron": "세제 통", "Sikhye_high": "식혜 캔", "Bear": "곰 인형", "Dinosaur": "공룡 인형"}
LSES = {"260901_cbnu_bigeightcircle": "장소 A · 큰 8자 주행", "260901_cbnu_digut": "장소 A · ㄷ자 주행", "260901_cbnu_eightcircle": "장소 A · 8자 주행", "260901_cbnu_longcircle": "장소 A · 긴 원 주행",
        "260826_etri_differentObject": "장소 B · 서로 다른 물체 배치", "260826_etri_eightcircle_dark": "장소 B · 8자 주행 (어두움)", "260826_etri_longcircle_dark": "장소 B · 긴 원 주행 (어두움)", "260826_etri_similarObject": "장소 B · 닮은 물체 배치"}


def lab_data():
    """→ (세션 목록, 방법 목록, objlist, 방법별 처리 프레임 (처리, 도착))"""
    L = J(LO / "objlist.json"); ses = [s for s in LSES if s in L]; methods = [m for m in LORDER if all(m in L[s] for s in ses)]; pr = {}
    for m in methods:
        if m in LPOSE: R = [J(f) for f in (LO / m).glob("*/run.json")]; pr[m] = (sum(r["processed"] for r in R), sum(r["frames"] for r in R))
        elif m.startswith("FM-Fusion"):
            T = [J(f) for s in ses for f in [Path.home() / f"DeepLearning/Datasets/fmfusion_lab_261010/scans/{s}/prediction_gsam_rt/timing.json"] if f.exists()]; pr[m] = (sum(t["frames"] for t in T), sum(t["offered"] for t in T) * 3)
        else:
            x = [re.findall(r"REALTIME processed (\d+) of (\d+)", f.read_text(errors="ignore")) for f in (LO / "dualmap_rt/lab_rt_261010_in").glob("26*.log")]; pr[m] = (sum(int(v[-1][0]) for v in x if v), sum(int(v[-1][1]) * 2 for v in x if v))
    return ses, methods, L, pr


def lab_stat(L, ses, m):
    named = [o for s in ses for o in L[s][m] if o["name"]]; kinds = sum(len({o["name"] for o in L[s][m] if o["name"]}) for s in ses)
    return dict(named=len(named), kinds=kinds, extra=sum(o["extra"] for o in named), unnamed=sum(1 for s in ses for o in L[s][m] if not o["name"]))


def lab_section(d):
    if not (LO / "objlist.json").exists(): return
    ses, methods, L, pr = lab_data(); IDX = J(LMED / "index.json") if (LMED / "index.json").exists() else {}; S = {m: lab_stat(L, ses, m) for m in methods}; total = 8 * len(ses)
    LMIDX = J(HERE / "media_map/index_lab.json") if (HERE / "media_map/index_lab.json").exists() else {}
    d.h2("7. 직접 수집한 영상에서의 결과")
    d.p(f"실시간 뷰어로 시험해 온 주행 영상 {len(ses)}개(두 장소, 각 4개)에 같은 방법 {len(methods)}가지를 실시간 재생으로 돌렸습니다. 물체는 실험실 물체 8종이고, 모든 영상에 8종이 하나씩 놓여 있습니다. "
        "이 영상들은 물체의 정답 자세가 없어서 맞음과 잘못 등록을 셀 수 없습니다. 대신 정답 없이 셀 수 있는 두 가지를 봅니다. 8종 가운데 몇 종이 지도에 올라갔는지, 그리고 같은 종류가 두 개 이상 올라간 초과 등록이 몇 개인지입니다. 종류마다 물체가 하나뿐이므로 초과 등록은 모두 잘못 올라간 객체입니다.")
    d.dl([("영상", "초당 10장으로 뽑은 " + f"{sum(len(J(Path.home() / 'DeepLearning/Datasets/lab_sessions_bop/test' / s / 'scene_camera.json')) for s in ses):,}장. 영상마다 카메라 하나를 씁니다. 인식용 카메라 녹화가 있으면 그것을, 없는 두 영상(장소 B의 물체 배치 영상)은 SLAM용 카메라 녹화를 썼습니다."),
          ("카메라 자세", "같은 영상으로 이 PC에서 ORB-SLAM3(RGB-D)를 돌린 결과를 모든 방법이 함께 씁니다. 실시간 뷰어에서 쓰던 두 카메라 구성(SLAM용 카메라와 인식용 카메라)과는 다릅니다."),
          ("자세 추정 구성", "A~E의 뜻은 2절과 같습니다. 문구와 관문 기준값은 실시간 뷰어에서 쓰는 설정(실험실 물체용)이고, D는 거기서 관문 기준만 낮추고 배정 대안을 켠 것입니다. 자세 검증의 합의 기준은 0.5입니다."),
          ("FM-Fusion, DualMap", "ObChange 때와 같은 방식으로 실시간 조건에서 돌렸습니다. FM-Fusion의 문구는 제안 방법과 같고, DualMap은 색 없는 일반 이름(상자, 캔, 머그 등)입니다."),
          ("그림의 색", "초록은 그 종류에서 관측이 가장 많은 객체, 빨강은 같은 종류의 초과 등록입니다. 초록이 실제 물체 자리에 놓였는지는 그림에서 확인해야 합니다.")])
    d.h3(f"영상 {len(ses)}개 합계")
    d.table(["방법", "처리한 프레임 / 도착한 프레임", "이름 붙은 등록 객체", f"등록된 종류 수 (최대 {total})", "초과 등록", "이름 없는 객체"],
            [[LLAB[m], f"{pr[m][0]:,} / {pr[m][1]:,}", S[m]["named"], S[m]["kinds"], S[m]["extra"], S[m]["unnamed"] if m not in LPOSE else "–"] for m in methods], cls="sum", group=lambda ri, r: "hl" if methods[ri] == "E" else "")
    d.h3("영상별 등록 객체 수 (괄호 안은 초과 등록)")
    d.table(["영상"] + [LSHORT[m] for m in methods], [[LSES[s]] + [f"{sum(bool(o['name']) for o in L[s][m])} ({sum(o['extra'] for o in L[s][m])})" for m in methods] for s in ses], cls="sum")
    d.h3(f"물체 종류별 등록 객체 수 (영상 {len(ses)}개 합계, 종류마다 {len(ses)}개가 정상)")
    d.table(["물체"] + [LSHORT[m] for m in methods], [[LKO[n]] + [sum(o["name"] == n for s in ses for o in L[s][m]) for m in methods] for n in LKO], cls="sum")
    d.p("표 머리의 약자: A~E′는 자세 추정 구성, FMF는 FM-Fusion, DM은 DualMap입니다.", "note")
    for s in ses:
        d.h3(LSES[s])
        for x in LMIDX.get(s, []):
            d.fig(HERE / "media_map" / x["file"], f"{LSES[s]} · 최종 객체 지도" + (f" (물체 무리 {x['cluster']})" if len(LMIDX[s]) > 1 else ""), width=1500)
        for x in IDX.get(s, []):
            if x["view"] > 1: continue
            d.fig(LMED / x["file"], f"{LSES[s]}, 시점 {x['view']} ({x['t_s']:.0f}초)" + (" · 자세 추정 구성 일곱 가지" if x["part"] == "a" else " · 제안 방법(E)과 FM-Fusion, DualMap"), width=1400)
        def cnt(m):
            c = collections.Counter(LKO[o["name"]] for o in L[s][m] if o["name"]); return ", ".join(f"{n} ×{v}" if v > 1 else n for n, v in sorted(c.items(), key=lambda kv: -kv[1])) or "–"
        d.table(["방법", "등록", "초과", "등록된 객체"], [[LLAB[m], sum(bool(o["name"]) for o in L[s][m]), sum(o["extra"] for o in L[s][m]), cnt(m)] for m in methods], cls="sum", widths=["24%", "6%", "6%", None])
    with open(HERE / "registered_objects_lab.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["video", "method", "name", "extra", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz", "n_obs"])
        for s in ses:
            for m in methods:
                for o in L[s][m]:
                    if not o["name"]: continue
                    q = [""] * 4
                    if o.get("R") is not None:
                        Rm = np.asarray(o["R"]); qw = np.sqrt(max(0, 1 + np.trace(Rm))) / 2
                        q = [round(float(v), 4) for v in ((qw, (Rm[2, 1] - Rm[1, 2]) / (4 * qw), (Rm[0, 2] - Rm[2, 0]) / (4 * qw), (Rm[1, 0] - Rm[0, 1]) / (4 * qw)) if qw > 1e-6 else (0, 1, 0, 0))]
                    w.writerow([LSES[s], LLAB[m], o["name"], int(o["extra"])] + [round(float(v), 4) for v in o["center"]] + q + [o.get("n_obs")])


def main():
    L = J(O / "objlist.json"); methods = [m for m in ORDER if all(m in r["methods"] for r in L)]
    if not methods: sys.exit("끝난 방법이 없습니다")
    room_of = lambda sc: sc.rsplit("_s", 1)[0]

    def st(m, rooms=None):
        rs = [r for r in L if rooms is None or room_of(r["scene"]) in rooms]; n = tp = named = 0; c = collections.Counter()
        for r in rs:
            tg = {g["name"] for g in r["gt"] if g["target"]}
            for g in r["methods"][m]["gt"]:
                if g["name"] in tg: n += 1; tp += g["ok"]
            for o in r["methods"][m]["objects"]:
                if o["verdict"] != "이름 없음": named += 1; c[o["verdict"]] += 1
        p = tp / max(named, 1); rc = tp / max(n, 1)
        return dict(n=n, tp=tp, named=named, fp=named - tp, fn=n - tp, P=100 * p, R=100 * rc, F=100 * 2 * p * rc / max(p + rc, 1e-9), c=c)
    S = {m: st(m) for m in methods}; PR = {m: proc(m) for m in methods}; d = Doc()
    best = max(methods, key=lambda m: S[m]["F"]); n_gt = S[methods[0]]["n"]
    F = lambda m: f"{S[m]['F']:.1f}"; has = lambda *ms: all(m in S for m in ms)

    # ---------- 1 요약 ----------
    d.h2("1. 요약")
    rows = []
    if has("E_on", "E_off", "C_on", "C_off"):
        rows.append(["<strong>자세 검증은 지도에서도 효과가 있는가</strong>", f"있습니다. 검증만 끈 짝과 비교하면 F1이 E {F('E_off')} → {F('E_on')} %, C {F('C_off')} → {F('C_on')} %입니다. 잘못 등록된 객체가 E {S['E_off']['fp']} → {S['E_on']['fp']}개, C {S['C_off']['fp']} → {S['C_on']['fp']}개로 줄어듭니다."])
    if has("B", "C_on"):
        rows.append(["<strong>후보를 문구 상자로 만드는 것과 화면 전체 분할의 차이</strong>", f"B(화면 전체 분할) F1 {F('B')} %, 맞음 {S['B']['tp']}개, 잘못 등록 {S['B']['fp']}개. C(문구 상자) F1 {F('C_on')} %, 맞음 {S['C_on']['tp']}개, 잘못 등록 {S['C_on']['fp']}개입니다."])
    if has("C_on", "D", "E_on"):
        rows.append(["<strong>판정 방식(원본 점수 / 제안 관문)의 차이</strong>", f"원본 판정 C {F('C_on')} %(맞음 {S['C_on']['tp']}, 잘못 등록 {S['C_on']['fp']}), 제안 관문 완화 D {F('D')} %(맞음 {S['D']['tp']}, 잘못 등록 {S['D']['fp']}), 제안 관문 배포 E {F('E_on')} %(맞음 {S['E_on']['tp']}, 잘못 등록 {S['E_on']['fp']})입니다."])
    if has("A"):
        rows.append(["<strong>원본 SAM-6D를 그대로 지도에 넣으면</strong>", f"F1 {F('A')} %입니다. 맞음 {S['A']['tp']}개, 잘못 등록 {S['A']['fp']}개. 초당 {PR['A'][0] / max(PR['A'][1] / 10, 1):.1f}장을 처리했습니다."])
    if has("FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"):
        rows.append(["<strong>FM-Fusion, DualMap에 CAD 이름을 붙이면</strong>", f"FM-Fusion {F('FM-Fusion')} → {F('FM-Fusion + CAD')} %, DualMap {F('DualMap')} → {F('DualMap + CAD')} %입니다. 잘못 등록된 객체는 FM-Fusion {S['FM-Fusion']['fp']} → {S['FM-Fusion + CAD']['fp']}개, DualMap {S['DualMap']['fp']} → {S['DualMap + CAD']['fp']}개입니다."])
    if (LO / "objlist.json").exists():
        ses_, ms_, L_, _ = lab_data(); X = {m: lab_stat(L_, ses_, m) for m in ms_}
        rows.append(["<strong>직접 수집한 영상에서는</strong>", f"물체 8종이 하나씩 놓인 영상 {len(ses_)}개에서, 제안 방법(E)은 {X['E']['named']}개를 등록했고 같은 종류가 두 번 올라간 초과 등록이 {X['E']['extra']}개입니다. "
                     f"검증을 끄면(E′) 초과 등록이 {X['E_off']['extra']}개, 원본 SAM-6D(A)는 {X['A']['extra']}개입니다. FM-Fusion {X['FM-Fusion']['extra']}개, DualMap {X['DualMap']['extra']}개이고, CAD 이름을 붙이면 {X['FM-Fusion + CAD']['extra']}개, {X['DualMap + CAD']['extra']}개로 줄지만 "
                     f"8종 가운데 올라간 종류가 {X['FM-Fusion + CAD']['kinds']}, {X['DualMap + CAD']['kinds']}종(최대 {8 * len(ses_)})으로 빠지는 물체가 생깁니다. 정답이 없는 영상이라 위치가 맞는지는 그림으로 확인했습니다(7절)."])
    rows.append(["<strong>ObChange에서 F1이 가장 높은 것</strong>", f"{LAB[best]} ({F(best)} %, 맞음 {S[best]['tp']}/{n_gt}, 잘못 등록 {S[best]['fp']}개)"])
    d.table(["질문", "결과"], rows, cls="sum", widths=["30%", None])

    # ---------- 2 조건 ----------
    d.h2("2. 실험 조건")
    d.dl([("데이터", f"ObChange의 Office, LivingArea, SmallRoom. 방마다 정답이 있는 방문 5회, 모두 15회. 채점 대상 정답 물체는 {n_gt}개입니다(YCB-V와 겹치는 11종). Kitchen은 뺐습니다."),
          ("실시간 조건", "영상이 10 fps로 도착한다고 보고, 앞 프레임의 처리가 끝난 순간 이미 도착한 프레임 가운데 가장 최신 것 하나만 처리합니다. 그 사이 프레임은 버립니다. 느린 구성일수록 적은 프레임으로 지도를 만듭니다."),
          ("찾을 물체", "모든 방문에서 11종 전부를 찾게 했습니다. 조합 보고서의 \"물체를 모르는 조건\"에 해당합니다."),
          ("자세 추정 구성", "조합 보고서(<code>objpose/docs/ycb_combo_261008</code>)의 핵심 다섯 구성과 검증만 끈 짝 둘. 평가 스크립트를 고치지 않고 입력만 실시간 재생으로 바꿔 끼웠으므로 문구, 관문 기준값, 색상 관문, 합의 기준(0.3)이 조합 보고서와 같습니다."),
          ("객체 지도", "일곱 구성 모두 같은 지도 구성기를 씁니다(같은 물체로 묶는 거리 0.30 m, 깊이 검사). 방문마다 지도를 새로 만듭니다. 카메라 자세는 데이터셋 제공값입니다."),
          ("FM-Fusion", "검출(GroundingDINO + SAM)이 끝난 순간 도착해 있는 최신 프레임만 처리하도록 다시 돌렸습니다. 검출된 프레임을 지도에 합치는 시간은 실시간 조건에 넣지 않았습니다. 문구는 제안 방법의 원래 문구입니다."),
          ("DualMap", "초당 5장으로 도착하는 프레임 가운데 처리가 밀리면 최신 것만 넣도록 다시 돌렸습니다. 문구는 색 없는 일반 이름, 검출 기준 0.05입니다."),
          ("+ CAD 이름", "두 방법이 만든 지도 객체에 제안 방법의 CAD 템플릿 검증으로 이름을 다시 붙였습니다. 판별도 실시간 재생으로 합니다(<code>objpose/docs/cad_fairness_261009</code>와 같은 방식)."),
          ("채점", "카메라가 실제로 본 점을 지도 객체에 배정(2 cm)하고, 정답 물체와 같은 이름의 객체가 0.25 이상 겹치면 맞음입니다. 대상 물체 이름이 붙었는데 맞음이 아닌 객체는 잘못 등록, 맞는 객체가 없는 정답은 놓침입니다."),
          ("반복", "한 번씩 돌렸습니다. 오늘 낮의 반복 실험에서 제안 방법의 F1은 반복 사이에 2.4 %p까지 움직였습니다.")])
    d.h3("구성")
    d.table(["구성", "① 후보", "② 마스크", "③ 특징", "④ 판정", "⑥ 자세 검증"],
            [["A. 원본 SAM-6D", "화면 전체 분할", "FastSAM-x", "ViT-L", "원본 점수, 기준 0.2", "없음"], ["B", "화면 전체 분할", "FastSAM-x", "ViT-S", "원본 점수, 기준 0.5", "켬"],
             ["C / C′", "YOLO-World 문구 상자", "MobileSAM", "ViT-S", "원본 점수, 기준 0.5", "켬 / 끔"], ["D", "YOLO-World 문구 상자", "MobileSAM", "ViT-S", "제안 관문 완화 + 배정 대안", "켬"],
             ["E / E′", "YOLO-World 문구 상자", "MobileSAM", "ViT-S", "제안 관문 배포 + 상자 독점 배정", "켬 / 끔"]], cls="sum")

    # ---------- 3 최종 지도 ----------
    d.h2("3. 최종 지도 비교")
    d.h3(f"세 방 합계 (방문 15회, 정답 물체 {n_gt}개)")
    hz = lambda m: f"{PR[m][0]:,} / {PR[m][1]:,}" if PR[m][1] else "–"
    d.table(["방법", "처리한 프레임 / 도착한 프레임", "이름 붙은 등록 객체", "맞음 (TP)", "잘못 등록 (FP)", "놓침 (FN)", "정밀도 (%)", "재현율 (%)", "F1 (%)"],
            [[LAB[m], hz(m), S[m]["named"], S[m]["tp"], S[m]["fp"], S[m]["fn"], float(S[m]["P"]), float(S[m]["R"]), float(S[m]["F"])] for m in methods], cls="sum", bars=(6, 7, 8), group=lambda ri, r: "hl" if methods[ri] == "E_on" else "")
    d.p("FM-Fusion과 DualMap의 등록 객체 수는 대상 물체 11종의 이름이 붙은 객체만 센 것입니다. 이름 없는 객체는 뺐습니다.", "note")
    d.h3("방별 F1 (%) · 맞음 / 잘못 등록")
    RS = {m: {k: st(m, {k}) for k, _ in ROOMS} for m in methods}
    d.table(["방법"] + [f"{nm} (정답 {RS[methods[0]][k]['n']}개)" for k, nm in ROOMS] + [f"{nm} 맞음 / 잘못 등록" for _, nm in ROOMS],
            [[LAB[m]] + [float(RS[m][k]["F"]) for k, _ in ROOMS] + [f"{RS[m][k]['tp']} / {RS[m][k]['fp']}" for k, _ in ROOMS] for m in methods], cls="sum", bars=(1, 2, 3))
    d.h3("등록된 객체의 판정 분포 (개)")
    d.dl([("맞음", "정답 물체와 이름이 같고 0.25 이상 겹치는, 그 정답의 대표 객체"), ("중복", "정답 물체와 이름이 같고 그 물체와 겹치지만(0.02 이상) 대표가 아닌 객체. 같은 물체가 지도에 두 번 올라갔거나, 일부만 잡혀 겹침이 0.25에 못 미친 경우"),
          ("다른 물체 자리", "다른 이름의 정답 물체(11종 밖의 물체 포함)와 0.25 이상 겹치는 객체. 이름을 틀린 경우"), ("정답 없는 자리", "어떤 정답 물체와도 0.25 이상 겹치지 않는 객체. 가구, 벽, 빈 곳 등")])
    d.table(["방법", "이름 붙은 등록 객체"] + VERD, [[LAB[m], S[m]["named"]] + [S[m]["c"][v] for v in VERD] for m in methods], cls="sum")

    # ---------- 4 물체 종류별 ----------
    d.h2("4. 물체 종류별 결과")
    names = [n for n in KOR if any(g["name"] == n and g["target"] for r in L for g in r["gt"])]
    cnt = {n: sum(g["name"] == n for r in L for g in r["gt"]) for n in names}
    d.h3("맞게 올린 수 / 정답 수")
    d.table(["물체", "정답 수"] + [SHORT[m] for m in methods], [[ko(n), cnt[n]] + [sum(g["ok"] for r in L for g in r["methods"][m]["gt"] if g["name"] == n) for m in methods] for n in names], cls="sum")
    d.h3("잘못 등록된 수 (그 이름으로 등록됐지만 맞음이 아닌 객체)")
    allnames = [n for n in KOR]
    d.table(["물체"] + [SHORT[m] for m in methods], [[ko(n)] + [sum(o["name"] == n and o["verdict"] not in ("맞음", "이름 없음") for r in L for o in r["methods"][m]["objects"]) for m in methods] for n in allnames], cls="sum")
    d.p("표 머리의 약자: A~E′는 자세 추정 구성, FMF는 FM-Fusion, DM은 DualMap입니다.", "note")

    # ---------- 5 정답 물체별 ----------
    d.h2("5. 정답 물체 하나하나의 결과")
    d.p("칸의 숫자는 그 정답 물체와 같은 이름으로 등록된 객체의 겹침 비율입니다. 0.25 이상이면 맞음(굵게)입니다. \"–\"는 같은 이름의 객체가 겹치지 않는 경우이고, 괄호 안은 그 자리에 대신 올라간 객체의 이름입니다.")
    for k, nm in ROOMS:
        d.h3(nm); rows = []
        for r in L:
            if room_of(r["scene"]) != k: continue
            for gi, g in enumerate(r["gt"]):
                if not g["target"]: continue
                row = [f"방문 {r['scene'].rsplit('_s', 1)[1]}", ko(g["name"])]
                for m in methods:
                    x = r["methods"][m]["gt"][gi]
                    if x["ok"]: row.append(f"<strong>{x['iou_name']:.2f}</strong>")
                    elif x["iou_name"] > 0: row.append(f"{x['iou_name']:.2f}")
                    elif x["iou_any"] >= 0.25 and x["any_name"] and x["any_name"] != g["name"]: row.append(f"– ({ko(x['any_name'])})")
                    else: row.append("–")
                rows.append(row)
        d.table(["방문", "정답 물체"] + [SHORT[m] for m in methods], rows, cls="sum")

    # ---------- 6 방문별 등록 객체 ----------
    d.h2("6. 방문별 등록 객체")
    MED = HERE / "media"; IDX = J(MED / "index.json") if (MED / "index.json").exists() else {}; MIDX = J(HERE / "media_map/index_obchange.json") if (HERE / "media_map/index_obchange.json").exists() else {}
    d.p("방문마다 먼저 최종 객체 지도를 보입니다. 방문 영상 전체로 복원한 장면을 비스듬히 내려다본 그림 위에 각 방법의 지도에 등록된 객체를 놓은 것이고, 가려진 객체도 모두 그렸습니다. 물체가 떨어져 놓인 방문은 무리마다 한 장입니다. 그 아래는 실제 카메라 영상을 확대해 겹쳐 그린 그림입니다.")
    d.p("그림은 실제 카메라 영상 위에 각 방법의 최종 지도 객체를 겹쳐 그린 것입니다. 윤곽 색은 초록 = 맞음, 노랑 = 중복(같은 물체가 한 번 더 올라갔거나 일부만 잡힘), 보라 = 다른 물체 자리에 틀린 이름, 빨강 = 정답이 없는 자리이고, 검은 원은 정답 물체의 위치입니다. 그 프레임에서 다른 것에 가려진 객체는 그리지 않았고, 이름표는 맞음 전부와 나머지 중 화면에서 큰 것 일곱 개까지만 붙였습니다. 시점이 둘인 방문의 두 번째 시점은 발표자료에 있습니다.")
    d.p("방문마다 각 방법의 최종 지도에 올라간 객체입니다(대상 물체 이름이 붙은 것만). 같은 이름이 여러 개면 \"×개수\"로 적었습니다. 객체마다의 위치, 방향, 관측 횟수는 <code>registered_objects.csv</code>에 있습니다.")

    def lst(objs, pred):
        c = collections.Counter(ko(o["name"]) for o in objs if pred(o)); return ", ".join(f"{n} ×{v}" if v > 1 else n for n, v in c.most_common()) or "–"
    for r in L:
        k = room_of(r["scene"]); nm = dict(ROOMS)[k]; tg = [ko(g["name"]) for g in r["gt"] if g["target"]]; other = [g["name"] for g in r["gt"] if not g["target"]]
        d.h3(f"{nm} 방문 {r['scene'].rsplit('_s', 1)[1]} · 정답: {', '.join(tg) or '없음'}" + (f" (대상 밖의 물체 {len(other)}개)" if other else ""))
        for x in MIDX.get(r["scene"], []):
            d.fig(HERE / "media_map" / x["file"], f"{nm} 방문 {r['scene'].rsplit('_s', 1)[1]} · 최종 객체 지도" + (f" (물체 무리 {x['cluster']})" if len(MIDX[r["scene"]]) > 1 else ""), width=1500)
        for x in IDX.get(r["scene"], []):
            if x["view"] != 1 or not x["zoom"]: continue
            d.fig(MED / x["file"], f"{nm} 방문 {r['scene'].rsplit('_s', 1)[1]}" + (", 확대" if x["zoom"] else "") + (" · 자세 추정 구성 일곱 가지" if x["part"] == "a" else " · 제안 방법(E)과 FM-Fusion, DualMap"), width=1400)
        d.table(["방법", "맞음", "잘못 등록"], [[LAB[m], lst(r["methods"][m]["objects"], lambda o: o["verdict"] == "맞음"), lst(r["methods"][m]["objects"], lambda o: o["verdict"] in VERD[1:])] for m in methods], cls="sum", widths=["24%", "24%", None])
    with open(HERE / "registered_objects.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["room", "visit", "method", "name", "verdict", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz", "n_obs", "best_overlap_gt", "best_overlap"])
        for r in L:
            for m in methods:
                for o in r["methods"][m]["objects"]:
                    if o["verdict"] == "이름 없음": continue
                    q = [""] * 4
                    if o.get("R") is not None:
                        Rm = np.asarray(o["R"]); t = np.trace(Rm); qw = np.sqrt(max(0, 1 + t)) / 2
                        q = [round(float(v), 4) for v in ((qw, (Rm[2, 1] - Rm[1, 2]) / (4 * qw), (Rm[0, 2] - Rm[2, 0]) / (4 * qw), (Rm[1, 0] - Rm[0, 1]) / (4 * qw)) if qw > 1e-6 else (0, 1, 0, 0))]
                    w.writerow([room_of(r["scene"]), r["scene"].rsplit("_s", 1)[1], LAB[m], o["name"], o["verdict"]] + [round(float(v), 4) for v in o["center"]] + q + [o.get("n_obs"), o.get("gt_best") or "", o["iou_best"]])

    # ---------- 7 직접 수집한 영상 ----------
    lab_section(d)
    # ---------- 8 해석 ----------
    d.h2("8. 읽는 법과 한계")
    d.ul(["조합 보고서는 사진 한 장씩의 자세 정확도(ADD-S)를 보았고, 이 보고서는 같은 구성을 영상에 실시간으로 돌려 만든 지도를 봅니다. 한 장 평가에서 좋은 구성이 지도에서도 좋은지를 확인하는 것이 목적입니다.",
          "느린 구성은 처리하는 프레임이 적습니다. 지도 구성기는 여러 프레임에서 같은 자세가 모여야 객체를 등록하므로, 프레임 수가 적으면 맞는 물체도 등록되지 못할 수 있습니다. 구성 사이의 차이에는 정확도와 속도의 효과가 함께 들어 있습니다.",
          "ObChange의 카메라 자세는 로봇이 스스로 추정한 값이라 수 cm의 오차가 있고, 그만큼 겹침 비율이 전체적으로 낮게 나옵니다.",
          "정답 물체가 방마다 10~25개로 적어 방별 수치는 물체 한두 개에 크게 움직입니다. 한 번씩만 돌렸습니다.",
          "FM-Fusion의 지도 통합 시간은 실시간 조건에 넣지 않았습니다. FM-Fusion에 유리한 쪽의 단순화입니다.",
          "CAD 이름 판별은 완성된 지도의 객체를 첫 프레임부터 투영합니다. 비교 방법에 유리한 쪽의 단순화입니다.",
          "문구와 관문 기준값은 YCB 데이터에서 정한 값 그대로이고 ObChange에 맞춰 조정하지 않았습니다.",
          "직접 수집한 영상은 정답 자세가 없습니다. 등록된 종류 수와 초과 등록은 이름과 개수만 본 것이고, 객체가 옳은 자리에 놓였는지는 세지 못합니다. 또 이 영상들은 제안 방법의 설정을 맞춰 온 환경이라 제안 방법에 유리합니다.",
          "직접 수집한 영상의 카메라 자세는 영상마다 카메라 하나로 구했습니다. 실시간 뷰어의 두 카메라 구성과 다릅니다."])
    d.h2("9. 결과 파일")
    d.ul(["실행: <code>objpose/tools/baselines/run_combo_obchange.sh</code>(구성 7개, <code>combo_rt.py</code>), <code>run_base_rt_obchange.sh</code>(FM-Fusion, DualMap) → <code>objpose/output/combo_obchange_261009/</code>",
          "구성별 결과: <code>&lt;구성&gt;/&lt;방문&gt;/sam6d_frames.jsonl</code>(프레임별 자세), <code>&lt;구성&gt;/maps/proposed/&lt;방문&gt;.json</code>(최종 지도), <code>score_&lt;방&gt;.json</code>",
          "등록 객체 목록: <code>objpose/tools/obchange/objlist_combo.py</code> → <code>objlist.json</code>, 이 폴더의 <code>registered_objects.csv</code>",
          "직접 수집한 영상: <code>objpose/tools/lab/</code>(<code>extract_lab_bop.py</code>, <code>run_slam_lab.sh</code>, <code>run_lab_combo.sh</code>, <code>run_base_lab.sh</code>, <code>lab_objlist.py</code>) → <code>objpose/output/lab_combo_261010/</code>, 이 폴더의 <code>registered_objects_lab.csv</code>",
          "관련 보고서: <code>objpose/docs/ycb_combo_261008/</code>(조합 실험), <code>objpose/docs/cad_fairness_261009/</code>(CAD 사용 여부), 이 보고서: <code>objpose/docs/combo_obchange_261009/build_report.py</code>"])

    ref = REF.read_text(); head = ref[: ref.index("<div class=\"wrap\">")].replace("<title>YCB 조합 실험 보고</title>", "<title>ObChange 구성별 객체 지도 비교</title>").replace("</style>", "figure{margin:6px 0 4px}figure img{max-width:100%;height:auto;border:1px solid var(--line);border-radius:4px;display:block;background:#fff}figcaption{margin-top:4px;margin-bottom:10px}</style>")
    hdr = ('<header><div class="kicker">실험 보고 · ObChange 구성별 객체 지도 · 2026-10-09</div><h1>ObChange에서 자세 추정 구성별 객체 지도 비교</h1>'
           f'<p class="lead">조합 실험의 자세 추정 구성 일곱 가지를 ObChange 영상에 실시간으로 돌려 각각 객체 지도를 만들고, FM-Fusion과 DualMap(원래 이름과 CAD 이름)의 지도와 함께 비교했습니다. '
           f'Office, LivingArea, SmallRoom의 방문 15회, 정답 물체 {n_gt}개이며 최종 지도에 등록된 객체를 기준으로 봅니다. 표에 있는 방법은 {len(methods)}가지입니다. '
           f'같은 방법들을 직접 수집한 주행 영상 8개에 돌린 결과는 7절에 있습니다.</p></header>')
    body = hdr + "\n" + "\n".join(x for x, _ in d.parts)
    (HERE / "combo_obchange_report.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "combo_obchange_report_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "ObChange 구성별 객체 지도 비교")
    nb = ("<p>실험 보고 · ObChange 구성별 객체 지도 · 2026-10-09</p>" + re.sub(r'<header>.*?</div>', "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "")) + "\n" + "\n".join(y for _, y in d.parts)
    (HERE / "combo_obchange_report_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    for f in ("combo_obchange_report.html", "combo_obchange_report_local.html", "combo_obchange_report_notion.html", "registered_objects.csv"): print(f, (HERE / f).stat().st_size)
    for m in methods: print(f"{LAB[m]:34s} named {S[m]['named']:4d} tp {S[m]['tp']:3d} fp {S[m]['fp']:4d} fn {S[m]['fn']:3d} F1 {S[m]['F']:5.1f}  {dict(S[m]['c'])}  proc {PR[m]}")


if __name__ == "__main__":
    main()
