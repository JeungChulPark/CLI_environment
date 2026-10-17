#!/usr/bin/env python3
"""build_report.py — 지금까지의 실험(데이터셋 × 객체 자세 추정 구성 여섯 가지 × 객체 지도 생성 방식 열두 가지)을 한 보고서로 정리한다.
  summary_report.html / _local.html / _notion.html, --pptx 를 주면 ~/콜로키움/전체_실험결과.pptx 뒤에 요약 쪽을 덧붙인다(제목 접두 "종합").
  데이터  ObChange(정답 있음): objpose/output/six_obchange_261011/scores.json          → 맞음 · 잘못 등록 · F1 · 정답과의 겹침
          직접 수집한 영상 8개(정답 없음, 실제 환경): objpose/output/live_six_261010/designs → 등록된 종류 · 초과 등록 · 깊이 지지율
  그림    media/ (objpose/tools/obchange/render_six_obchange.py, objpose/tools/lab/render_live_maps.py). 이 스크립트가 필요한 그림을 다시 만든다(--no-render 로 생략).
    python3 objpose/docs/summary_261011/build_report.py [--pptx] [--no-render]"""
import argparse, collections, importlib.util, json, re, subprocess, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]; MED = HERE / "media"; OB = (REPO / "objpose/output" / __import__("os").environ.get("OBJPOSE_OB_OUT", "six_obchange_261011")); PY = str(Path.home() / "anaconda3/envs/sam6d/bin/python")


def mod(name, path):
    spec = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m


L6 = mod("live_rep", REPO / "objpose/docs/live_six_261010/build_report.py"); O6 = mod("ob_rep", REPO / "objpose/docs/six_obchange_261011/build_report.py")
Doc, REF, J = L6.Doc, L6.REF, L6.J
CFG = {1: "1. FastSAM-x + ViT-L", 2: "2. FastSAM-x + ViT-S", 3: "3. YOLO-World + FastSAM-x + ViT-L", 4: "4. YOLO-World + FastSAM-x + ViT-S", 5: "5. YOLO-World + MobileSAM + ViT-L", 6: "6. YOLO-World + MobileSAM + ViT-S", 7: "7. 제안 방법, 완화 설정 (제안 관문 완화 + 배정 대안)", 8: "8. 제안 방법, 배포 설정 (제안 관문)"}
SPEC = {1: ("화면 전체 분할 (FastSAM-x)", "FastSAM-x", "ViT-L", "원본 점수 0.5"), 2: ("화면 전체 분할 (FastSAM-x)", "FastSAM-x", "ViT-S", "원본 점수 0.5"), 3: ("문구 상자 (YOLO-World)", "FastSAM-x", "ViT-L", "원본 점수 0.5"),
        4: ("문구 상자 (YOLO-World)", "FastSAM-x", "ViT-S", "원본 점수 0.5"), 5: ("문구 상자 (YOLO-World)", "MobileSAM", "ViT-L", "원본 점수 0.5"), 6: ("문구 상자 (YOLO-World)", "MobileSAM", "ViT-S", "원본 점수 0.5"),
        7: ("문구 상자 (YOLO-World)", "MobileSAM", "ViT-S", "제안 관문 완화 + 배정 대안"), 8: ("문구 상자 (YOLO-World)", "MobileSAM", "ViT-S", "제안 관문")}
CB = O6.CB; VNAME = dict(CB); VSHORT = {"V0": "지금", "1": "1", "1n": "1′", "2": "2", "3": "3", "1+2": "1+2", "1n+2": "1′+2", "1+3": "1+3", "1n+3": "1′+3", "2+3": "2+3", "1+2+3": "1+2+3", "1n+2+3": "1′+2+3"}; VK = [v for v, _ in CB]; CS = [1, 2, 3, 4, 5, 6]          # main() 에서 두 데이터에 모두 끝난 구성으로 다시 정한다
room_of = lambda s: s.rsplit("_s", 1)[0]


def heat(fname, M, rows, cols, title, fmt, good_high=True, vmin=None, vmax=None):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib import font_manager as fmg
    fp = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"; fmg.fontManager.addfont(fp); plt.rcParams["font.family"] = fmg.FontProperties(fname=fp).get_name()
    M = np.asarray(M, float); fig, ax = plt.subplots(figsize=(10.5, 3.6), dpi=150); im = ax.imshow(M, cmap="RdYlGn" if good_high else "RdYlGn_r", aspect="auto", vmin=vmin, vmax=vmax)
    ax.set_xticks(range(len(cols))); ax.set_xticklabels(cols, fontsize=10.5); ax.set_yticks(range(len(rows))); ax.set_yticklabels(rows, fontsize=10.5); ax.set_title(title, fontsize=13, loc="left")
    for i in range(M.shape[0]):
        for j in range(M.shape[1]): ax.text(j, i, fmt.format(M[i, j]), ha="center", va="center", fontsize=10, color="black")
    ax.set_xlabel("객체 지도 생성 방식 (지금 = 지금 방식, 숫자 = 설계 번호, 1′ = 설계 1에서 빼는 규칙 없앰)", fontsize=10); fig.tight_layout(); MED.mkdir(exist_ok=True); fig.savefig(MED / fname); plt.close(fig); return MED / fname


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--pptx", action="store_true"); ap.add_argument("--no-render", action="store_true"); a = ap.parse_args()
    # ---------- ObChange ----------
    global CS
    S = J(OB / "scores.json"); have = collections.defaultdict(set); D = L6.load()
    for r in S: have[(r["config"], room_of(r["scene"]))].add(r["scene"])
    CS = [c for c in range(1, 9) if c <= 6 or (len(have[(c, "smallroom")]) == 5 and sum((c, s) in D for s in L6.SES) == len(L6.SES))]      # 7 · 8 은 두 데이터에서 모두 끝났을 때만 넣는다
    S = [r for r in S if r["config"] in CS]
    rooms = [(k, nm) for k, nm in O6.ROOMS if all(len(have[(c, k)]) == 5 for c in CS)]; rk = {k for k, _ in rooms}; S = [r for r in S if room_of(r["scene"]) in rk]
    sel = lambda c=None, v=None, room=None: [r for r in S if (c is None or r["config"] == c) and (v is None or r["variant"] == v) and (room is None or room_of(r["scene"]) == room)]
    A = {v: O6.agg(sel(v=v)) for v in VK}; T = {(c, v): O6.agg(sel(c=c, v=v)) for c in CS for v in VK}; n_gt = T[(1, "V0")]["gt"]; ob_names = " · ".join(nm for _, nm in rooms)
    run = {c: [J(f) for k, _ in rooms for n in range(2, 7) for f in [OB / f"c{c}" / f"{k}_s{n}" / "run.json"] if f.exists()] for c in CS}
    ob_proc = {c: (sum(r["processed"] for r in run[c]), sum(r["frames"] for r in run[c]), float(np.mean([r["ms_mean"] for r in run[c]]))) for c in CS}
    # ---------- 직접 수집한 영상 ----------
    ses = [s for s in L6.SES if all((c, s) in D for c in CS)]; LA = {v: L6.stat(D, [(c, s) for c in CS for s in ses], v) for v in VK}; LT = {(c, v): L6.stat(D, [(c, s) for s in ses], v) for c in CS for v in VK}; lab_max = 8 * len(ses)
    LR = {c: L6.runs(D, c) for c in CS}
    # ---------- 선택 규칙 ----------
    v0 = "V0"
    safe = [v for v in VK if v != v0 and A[v]["F"] >= A[v0]["F"] - 1e-9 and A[v]["tp"] >= A[v0]["tp"] and LA[v]["kinds"] >= LA[v0]["kinds"] and LA[v]["extra"] <= LA[v0]["extra"]]      # 두 데이터 어디에서도 지금 방식보다 나빠지지 않는 방식
    bestv = max(safe, key=lambda v: (A[v]["tp50"], A[v]["iou"])) if safe else v0; topF = max(VK, key=lambda v: A[v]["F"])
    okc = [c for c in CS if LT[(c, bestv)]["kinds"] >= max(LT[(x, bestv)]["kinds"] for x in CS) - 1]                                                                                    # 직접 수집한 영상에서 물건을 놓치지 않는 구성
    bestc = max(okc, key=lambda c: (T[(c, bestv)]["F"], -ob_proc[c][2])); rank = sorted(CS, key=lambda c: -T[(c, bestv)]["F"])
    f1 = lambda x: f"{x:.1f}"; sg = lambda x: f"{x:+.1f}"
    # ---------- 그림 ----------
    MED.mkdir(exist_ok=True)
    h_ob = heat("heat_ob_f1.png", [[T[(c, v)]["F"] for v in VK] for c in CS], [f"구성 {c}" for c in CS], [VSHORT[v] for v in VK], f"ObChange F1 (%) — {ob_names}, 구성당 정답 물체 {n_gt}개", "{:.0f}")
    h_ob2 = heat("heat_ob_iou.png", [[T[(c, v)]["tp50"] for v in VK] for c in CS], [f"구성 {c}" for c in CS], [VSHORT[v] for v in VK], "ObChange: 정답과 0.5 이상 겹치는 객체 수 (자세가 정확한 객체)", "{:.0f}")
    h_lab = heat("heat_lab_kinds.png", [[LT[(c, v)]["kinds"] for v in VK] for c in CS], [f"구성 {c}" for c in CS], [VSHORT[v] for v in VK], f"직접 수집한 영상: 등록된 물체 종류 (최대 {lab_max})", "{:.0f}", vmin=lab_max - 16, vmax=lab_max)
    h_lab2 = heat("heat_lab_extra.png", [[LT[(c, v)]["extra"] for v in VK] for c in CS], [f"구성 {c}" for c in CS], [VSHORT[v] for v in VK], "직접 수집한 영상: 같은 종류의 초과 등록 (적을수록 좋음)", "{:.0f}", good_high=False)
    ob_show = [f"{k}_s{6 if k != 'office' else 3}" for k, _ in rooms]; lab_show = ["260901_cbnu_longcircle", "260826_etri_longcircle_dark"]
    if not a.no_render:
        env = {"PYTHONPATH": str(REPO / "objpose/tools/shims_nvml"), "PATH": "/usr/bin:/bin", "HOME": str(Path.home())}
        subprocess.run([PY, str(REPO / "objpose/tools/obchange/render_six_obchange.py"), "--root", str(OB), "--out", str(MED), "--rows", v0, bestv, "--wrap", str(len(CS)), "--configs"] + [str(c) for c in CS] + ["--scenes"] + ob_show, env=env, capture_output=True)
        subprocess.run([PY, str(REPO / "objpose/tools/obchange/render_six_obchange.py"), "--root", str(OB), "--out", str(MED), "--rows", bestv, "--wrap", str(len(CS) // 2), "--suffix", "_ppt", "--configs"] + [str(c) for c in CS] + ["--scenes"] + ob_show, env=env, capture_output=True)
        subprocess.run([PY, str(REPO / "objpose/tools/lab/render_live_maps.py"), "--out", str(MED), "--rows", v0, bestv, "--wrap", str(len(CS)), "--suffix", "_sum", "--configs"] + [str(c) for c in CS], env=env, capture_output=True)
        subprocess.run([PY, str(REPO / "objpose/tools/lab/render_live_maps.py"), "--out", str(MED), "--rows", bestv, "--wrap", str(len(CS) // 2), "--suffix", "_ppt", "--configs"] + [str(c) for c in CS] + ["--sessions"] + lab_show, env=env, capture_output=True)
        ob_all = [f"{k}_s{n}" for k, _ in rooms for n in range(2, 7)]
        subprocess.run([PY, str(REPO / "objpose/tools/analysis/render_pose_map.py"), "--data", "obchange", "--config", str(bestc), "--variant", bestv, "--out", str(MED), "--scenes"] + ob_all, env=env, capture_output=True)
        subprocess.run([PY, str(REPO / "objpose/tools/analysis/render_pose_map.py"), "--data", "lab", "--config", str(bestc), "--variant", bestv, "--out", str(MED)], env=env, capture_output=True)
    d = Doc()

    # ---------- 1 결론 ----------
    d.h2("1. 결론")
    d.table(["항목", "결론과 근거"], [
        ["<strong>가장 좋은 조합</strong>", f"<strong>{CFG[bestc]}</strong> + <strong>{VNAME[bestv]}</strong>. ObChange에서 F1 {f1(T[(bestc, bestv)]['F'])} %(맞음 {T[(bestc, bestv)]['tp']}/{n_gt}, 잘못 등록 {T[(bestc, bestv)]['fp']}), 직접 수집한 영상에서 {LT[(bestc, bestv)]['kinds']}/{lab_max}종 등록 · 초과 등록 {LT[(bestc, bestv)]['extra']}개입니다."],
        ["<strong>지도 생성 방식: " + VNAME[bestv] + "</strong>", f"두 데이터 어디에서도 지금 방식보다 나빠지지 않으면서 자세를 가장 정확하게 만든 방식입니다. ObChange에서 맞음 {A[v0]['tp']} → {A[bestv]['tp']}, 잘못 등록 {A[v0]['fp']} → {A[bestv]['fp']}, 정답과 0.5 이상 겹치는 객체 {A[v0]['tp50']} → {A[bestv]['tp50']}개, 겹침 평균 {A[v0]['iou']:.3f} → {A[bestv]['iou']:.3f}. 직접 수집한 영상에서 등록된 종류 {LA[v0]['kinds']} → {LA[bestv]['kinds']}, 초과 등록 {LA[v0]['extra']} → {LA[bestv]['extra']}, 깊이 지지율 {LA[v0]['sup']:.2f} → {LA[bestv]['sup']:.2f}."],
        ["<strong>자세 추정 구성: " + CFG[bestc] + "</strong>", f"직접 수집한 영상에서 물건을 놓치지 않는 구성({', '.join(str(c) for c in okc)}번) 가운데 ObChange F1이 가장 높습니다. ObChange F1 순서는 " + " > ".join(f"{c}번 {f1(T[(c, bestv)]['F'])}" for c in rank) + f"입니다({VNAME[bestv]} 기준)."],
        ["<strong>ObChange F1만 보면</strong>", f"지도 방식 가운데 F1이 가장 높은 것은 {VNAME[topF]}({f1(A[topF]['F'])} %, 지금 방식 {f1(A[v0]['F'])} %)입니다." + ("" if topF == bestv else f" 그러나 직접 수집한 영상에서 등록된 종류 {LA[v0]['kinds']} → {LA[topF]['kinds']}, 초과 등록 {LA[v0]['extra']} → {LA[topF]['extra']}로 지금 방식보다 나빠지는 면이 있어 고르지 않았습니다.")],
        ["<strong>믿을 수 있는 정도</strong>", f"모든 실행은 한 번씩입니다. ObChange는 구성당 정답 물체가 {n_gt}개라 물체 하나에 F1이 약 {100 / n_gt:.0f} %p 움직이고, 직접 수집한 영상은 정답 자세가 없습니다. 두 데이터에서 같은 방향으로 나온 것(설계 1′의 자세 개선)은 믿을 만하고, 구성 사이의 몇 %p 차이는 반복 실행으로 확인해야 합니다."]],
        cls="sum", widths=["24%", None])

    # ---------- 2 무엇을 비교했나 ----------
    d.h2("2. 무엇을 비교했나")
    d.p("세 가지를 바꿔 가며 모든 조합을 돌렸습니다. 데이터셋 두 가지, 객체 자세 추정 구성 여섯 가지, 객체 지도 생성 방식 열두 가지입니다. 모든 실행은 영상이 원래 속도로 들어오고 처리가 밀리면 최신 프레임만 쓰는 실시간 조건입니다.")
    d.h3("데이터셋")
    d.table(["데이터셋", "내용", "카메라 자세", "정답", "쓸 수 있는 지표"], [
        [f"ObChange ({ob_names})", f"방마다 방문 5회, 모두 {len(rooms) * 5}회. YCB 물체 11종을 찾게 함. 채점 대상 정답 물체 {n_gt}개", "데이터셋 제공값", "있음", "맞음(TP) · 잘못 등록(FP) · 놓침(FN) · F1, 정답과의 겹침"],
        [f"직접 수집한 영상 {len(ses)}개", "두 장소에서 4개씩. 실험실 물체 8종이 하나씩 놓여 있음", "노트북에서 ORB-SLAM3 실시간(실제 환경)", "없음", f"등록된 종류(최대 {lab_max}) · 같은 종류의 초과 등록 · 깊이 지지율"]], cls="sum")
    d.h3("객체 자세 추정 구성 (자세 추정과 자세 검증은 항상 켬)")
    d.table(["구성", "후보 생성", "마스크", "특징", "물체 판정", "ObChange: 처리한 프레임 / 도착", "ObChange: 프레임당 평균 (ms)", "수집 영상: 처리한 프레임"],
            [[str(c), SPEC[c][0], SPEC[c][1], SPEC[c][2], SPEC[c][3], f"{ob_proc[c][0]:,} / {ob_proc[c][1]:,}", round(ob_proc[c][2]), f"{LR[c]['proc']:,}"] for c in CS], cls="sum")
    d.h3("객체 지도 생성 방식")
    d.table(["방식", "하는 일"], [["지금 방식", "같은 이름의 관측을 30 cm 안에서 묶어 위치는 평균, 방향은 다수결. 반복 관측되면 등록하고, 깊이 영상에서 자리가 비어 있으면 지움"],
                              ["설계 1", "객체마다 묶인 관측 가운데 서로 가까운(25 mm, 20°) 자세의 가장 큰 무리로 자세를 정함. 무리가 작으면(관측 3개 미만 또는 가중치 30 % 미만) 객체를 뺌"], ["설계 1′", "설계 1에서 객체를 빼는 규칙만 없앤 것"],
                              ["설계 2", "CAD를 주변 깊이 점에 다시 맞추고, 깊이 지지율이 0.2 미만이면 객체를 뺌"], ["설계 3", "검증에서 떨어진 후보도 증거로 쌓아 미등록 무리를 등록하고(증거 6개 이상, 지지율 0.5 이상), 10 cm 안의 이름 충돌을 정리"],
                              ["조합", "1 → 2 → 3 순서로 적용. 설계를 하나씩, 둘씩, 셋 모두 쓴 것과 설계 1′로 바꾼 것까지 열두 가지"]], cls="sum", widths=["14%", None])

    # ---------- 3 ObChange ----------
    d.h2(f"3. ObChange 결과 (정답으로 채점, {ob_names})")
    d.fig(h_ob, "구성(행) × 지도 방식(열)의 F1. 초록일수록 높음", width=1500); d.fig(h_ob2, "같은 표를 '정답과 0.5 이상 겹치는 객체 수'로 본 것. 자세가 정확할수록 많음", width=1500)
    d.h3("지도 방식별 합계 (여섯 구성 합)")
    d.table(["지도 방식", "등록 객체", "맞음 (TP)", "잘못 등록 (FP)", "놓침 (FN)", "정밀도 (%)", "재현율 (%)", "F1 (%)", "겹침 0.5 이상", "겹침 평균"],
            [[VNAME[v], A[v]["named"], A[v]["tp"], A[v]["fp"], A[v]["fn"], float(A[v]["P"]), float(A[v]["R"]), float(A[v]["F"]), A[v]["tp50"], f"{A[v]['iou']:.3f}"] for v in VK], cls="sum", bars=(5, 6, 7), group=lambda ri, r: "hl" if VK[ri] in (v0, bestv) else "")
    d.h3("구성별 결과 (지금 방식 → " + VNAME[bestv] + ")")
    d.table(["구성", "맞음", "잘못 등록", "F1 (%)", "겹침 0.5 이상", "겹침 평균"],
            [[CFG[c], f"{T[(c, v0)]['tp']} → {T[(c, bestv)]['tp']}", f"{T[(c, v0)]['fp']} → {T[(c, bestv)]['fp']}", f"{f1(T[(c, v0)]['F'])} → {f1(T[(c, bestv)]['F'])}", f"{T[(c, v0)]['tp50']} → {T[(c, bestv)]['tp50']}", f"{T[(c, v0)]['iou']:.3f} → {T[(c, bestv)]['iou']:.3f}"] for c in CS], cls="sum", group=lambda ri, r: "hl" if CS[ri] == bestc else "")
    if len(rooms) > 1:
        d.h3("방별 F1 (%) — 여섯 구성 합")
        d.table(["지도 방식"] + [nm for _, nm in rooms], [[VNAME[v]] + [float(O6.agg(sel(v=v, room=k))["F"]) for k, _ in rooms] for v in VK], cls="sum", bars=tuple(range(1, len(rooms) + 1)))
        d.h3("방별 F1 (%) — 구성별, " + VNAME[bestv])
        d.table(["구성"] + [f"{nm} (정답 {O6.agg(sel(c=1, v=v0, room=k))['gt']}개)" for k, nm in rooms], [[CFG[c]] + [float(O6.agg(sel(c=c, v=bestv, room=k))["F"]) for k, _ in rooms] for c in CS], cls="sum", bars=tuple(range(1, len(rooms) + 1)))

    # ---------- 4 직접 수집 ----------
    d.h2(f"4. 직접 수집한 영상 결과 (실제 환경, 영상 {len(ses)}개)")
    d.fig(h_lab, f"등록된 물체 종류. 구성당 최대 {lab_max}", width=1500); d.fig(h_lab2, "같은 종류의 초과 등록. 물체가 종류마다 하나뿐이므로 모두 잘못 올라간 객체", width=1500)
    d.h3("지도 방식별 합계 (여섯 구성 합)")
    d.table(["지도 방식", "등록 객체", f"등록된 종류 (최대 {lab_max * 6})", "초과 등록", "깊이 지지율 평균", "지지율 0.5 이상 (%)"], [[VNAME[v], LA[v]["n"], LA[v]["kinds"], LA[v]["extra"], f"{LA[v]['sup']:.2f}", f"{LA[v]['ok']:.0f}"] for v in VK], cls="sum", group=lambda ri, r: "hl" if VK[ri] in (v0, bestv) else "")
    d.h3("구성별 결과 (지금 방식 → " + VNAME[bestv] + ")")
    d.table(["구성", f"등록된 종류 (최대 {lab_max})", "초과 등록", "깊이 지지율", "처리한 프레임", "프레임당 시간, 중앙값 (ms)"],
            [[CFG[c], f"{LT[(c, v0)]['kinds']} → {LT[(c, bestv)]['kinds']}", f"{LT[(c, v0)]['extra']} → {LT[(c, bestv)]['extra']}", f"{LT[(c, v0)]['sup']:.2f} → {LT[(c, bestv)]['sup']:.2f}", f"{LR[c]['proc']:,}", round(LR[c]["ms"])] for c in CS], cls="sum", group=lambda ri, r: "hl" if CS[ri] == bestc else "")

    # ---------- 5 근거 ----------
    d.h2("5. 왜 이 조합인가")
    d.h3("단계 1. 지도 생성 방식 고르기")
    d.p("고르는 기준은 두 단계입니다. 먼저 <strong>두 데이터 어디에서도 지금 방식보다 나빠지지 않는 방식</strong>만 남깁니다(ObChange의 F1과 맞음 수가 줄지 않고, 직접 수집한 영상의 등록된 종류가 줄지 않고 초과 등록이 늘지 않을 것). 그 가운데 <strong>자세를 가장 정확하게 만드는 방식</strong>(ObChange에서 정답과 0.5 이상 겹치는 객체가 가장 많은 것)을 고릅니다.")
    d.table(["지도 방식", "ObChange F1 (%)", "지금 방식 대비", "ObChange 맞음", "ObChange 겹침 0.5 이상", "수집 영상: 종류", "수집 영상: 초과 등록", "두 데이터 모두 나빠지지 않음"],
            [[VNAME[v], float(A[v]["F"]), "–" if v == v0 else sg(A[v]["F"] - A[v0]["F"]), A[v]["tp"], A[v]["tp50"], LA[v]["kinds"], LA[v]["extra"], "기준" if v == v0 else ("예" if v in safe else "아니오")] for v in VK], cls="sum", bars=(1,), group=lambda ri, r: "hl" if VK[ri] == bestv else "")
    bul = [f"<strong>설계 1′</strong>은 묶는 규칙과 등록 규칙을 건드리지 않고 자세만 다시 정하므로, 맞음과 잘못 등록의 개수가 지금 방식과 같습니다(ObChange 맞음 {A[v0]['tp']} → {A['1n']['tp']}, 잘못 등록 {A[v0]['fp']} → {A['1n']['fp']}). 달라지는 것은 자세의 정확도로, 정답과 0.5 이상 겹치는 객체가 {A[v0]['tp50']} → {A['1n']['tp50']}개가 됩니다. 직접 수집한 영상에서도 깊이 지지율이 {LA[v0]['sup']:.2f} → {LA['1n']['sup']:.2f}로 같은 방향입니다.",
           f"<strong>설계 1의 빼는 규칙</strong>은 ObChange에서 잘못 등록을 {A[v0]['fp']} → {A['1']['fp']}개로 줄이지만 맞음도 {A[v0]['tp']} → {A['1']['tp']}개로 줄이고, 직접 수집한 영상에서는 등록된 종류를 {LA[v0]['kinds']} → {LA['1']['kinds']}로 줄입니다. 실제 물체를 지도에서 빼는 대가가 있습니다.",
           f"<strong>설계 2</strong>는 ObChange에서 F1을 {f1(A[v0]['F'])} → {f1(A['2']['F'])} %, 설계 1′ 뒤에 붙였을 때 {f1(A['1n']['F'])} → {f1(A['1n+2']['F'])} %로 바꿉니다. 직접 수집한 영상에서 깊이 지지율이 오른 것은 설계 2가 그 값을 기준으로 자세를 고르기 때문이고, 정답으로 본 결과와는 다릅니다.",
           f"<strong>설계 3</strong>은 ObChange에서 지금 방식에 더하면 F1 {sg(A['3']['F'] - A[v0]['F'])} %p, 설계 1′에 더하면 {sg(A['1n+3']['F'] - A['1n']['F'])} %p입니다. 직접 수집한 영상에서는 초과 등록을 {LA[v0]['extra']} → {LA['3']['extra']}개로 늘립니다. 데이터에 따라 방향이 갈립니다."]
    d.ul(bul)
    d.h3("단계 2. 자세 추정 구성 고르기")
    d.p(f"지도 방식을 {VNAME[bestv]}로 두고 여섯 구성을 비교합니다. 먼저 직접 수집한 영상에서 물건을 놓치지 않는 구성만 남기고(등록된 종류가 가장 많은 구성과 1개 이내), 그 가운데 ObChange F1이 높은 것을 고릅니다. F1이 비슷하면 빠른 쪽을 택합니다.")
    d.table(["구성", "수집 영상: 종류", "수집 영상: 초과", "놓치지 않음", "ObChange F1 (%)", "ObChange 맞음 / 잘못 등록", "ObChange 겹침 0.5 이상", "ObChange 프레임당 평균 (ms)"],
            [[CFG[c], LT[(c, bestv)]["kinds"], LT[(c, bestv)]["extra"], "예" if c in okc else "아니오", float(T[(c, bestv)]["F"]), f"{T[(c, bestv)]['tp']} / {T[(c, bestv)]['fp']}", T[(c, bestv)]["tp50"], round(ob_proc[c][2])] for c in CS], cls="sum", bars=(4,), group=lambda ri, r: "hl" if CS[ri] == bestc else "")
    vl = [c for c in CS if SPEC[c][2] == "ViT-L"]; vs = [c for c in CS if SPEC[c][2] == "ViT-S"]
    d.ul([f"<strong>ViT-L과 ViT-S.</strong> 직접 수집한 영상에서 ViT-L 구성({', '.join(map(str, vl))}번)은 등록된 종류가 {', '.join(str(LT[(c, bestv)]['kinds']) for c in vl)}개, ViT-S 구성({', '.join(map(str, vs))}번)은 {', '.join(str(LT[(c, bestv)]['kinds']) for c in vs)}개입니다. ViT-L은 공룡 인형을 곰 인형 쪽으로 판정해 놓쳤습니다. ObChange F1은 ViT-L {', '.join(f1(T[(c, bestv)]['F']) for c in vl)} %, ViT-S {', '.join(f1(T[(c, bestv)]['F']) for c in vs)} %입니다.",
          f"<strong>후보 생성.</strong> 화면 전체 분할(1, 2번)은 프레임당 {round(ob_proc[1][2])}, {round(ob_proc[2][2])} ms, 문구 상자({CS[2]}~{CS[-1]}번)는 {round(min(ob_proc[c][2] for c in CS[2:]))}~{round(max(ob_proc[c][2] for c in CS[2:]))} ms입니다. 실시간 조건에서는 빠른 구성이 더 많은 프레임을 봅니다.",
          f"<strong>고른 구성({bestc}번)</strong>은 직접 수집한 영상에서 {LT[(bestc, bestv)]['kinds']}/{lab_max}종을 올리고 ObChange F1이 {f1(T[(bestc, bestv)]['F'])} %입니다. 2위와의 차이는 {f1(T[(bestc, bestv)]['F'] - max(T[(c, bestv)]['F'] for c in okc if c != bestc))} %p입니다."])

    # ---------- 6 지도 그림 ----------
    d.h2("6. 최종 객체 지도: 여섯 구성 비교")
    d.p(f"영상 전체로 복원한 장면을 비스듬히 내려다본 그림 위에 지도 객체를 놓았습니다. 열은 구성 1~6, 윗줄은 지금 방식, 아랫줄은 {VNAME[bestv]}입니다.")
    d.h3("ObChange (초록 = 맞음, 빨강 = 잘못 등록, 검은 원 = 정답 물체)")
    IO = J(MED / "index_ob.json") if (MED / "index_ob.json").exists() else {}
    for s in ob_show:
        if s in IO: d.fig(MED / IO[s], f"{dict(rooms)[room_of(s)]} 방문 {s.rsplit('_s', 1)[1]}", width=2400)
    d.h3("직접 수집한 영상 (초록 = 그 종류에서 하나, 빨강 = 같은 종류의 초과 등록, 숫자 = 깊이 지지율)")
    IL = J(MED / "index_sum.json") if (MED / "index_sum.json").exists() else {}
    for s in ses:
        if s in IL: d.fig(MED / IL[s], L6.SES[s], width=2400)

    # ---------- 6b 고른 조합의 지도와 저장된 자세 ----------
    d.h2("7. 고른 조합이 실제로 만든 객체 지도와 저장된 자세")
    d.p(f"가장 좋은 조합({CFG[bestc]} + {VNAME[bestv]})이 만든 지도입니다. 지도에는 객체마다 이름과 6D 자세(위치 3개, 방향 3개)가 저장됩니다. 그림은 복원한 장면 위에, 저장된 자세 그대로 물체의 CAD 모형을 놓고 그 자세의 좌표축(빨강 x, 초록 y, 파랑 z, 길이 6 cm)을 그린 것입니다. 표는 같은 객체들의 저장 값입니다. 위치는 지도 좌표계 기준(m), 방향은 롤 · 피치 · 요(°)입니다.")
    for key, title, names, note_h in (("obchange", "ObChange", lambda s_: f"{dict(rooms)[room_of(s_)]} 방문 {s_.rsplit('_s', 1)[1]}", "정답과의 판정"), ("lab", "직접 수집한 영상", lambda s_: L6.SES[s_], "깊이 지지율")):
        f = MED / f"poses_{key}.json"
        if not f.exists(): continue
        PJ = J(f)
        if PJ["config"] != bestc or PJ["variant"] != bestv: continue
        d.h3(title + (" (검은 원 = 정답 물체의 위치)" if key == "obchange" else ""))
        order = [s_ for k, _ in rooms for n in range(2, 7) for s_ in [f"{k}_s{n}"]] if key == "obchange" else ses
        for s_ in order:
            if s_ not in PJ["poses"]: continue
            for x in PJ["index"].get(s_, []): d.fig(MED / x["file"], names(s_) + (f" · 물체 {x['objects']}개가 모인 곳" if len(PJ["index"][s_]) > 1 else ""), width=1100)
            rows_ = [[o["label"], f"{o['xyz_m'][0]:.3f}", f"{o['xyz_m'][1]:.3f}", f"{o['xyz_m'][2]:.3f}", f"{o['rpy_deg'][0]:.1f}", f"{o['rpy_deg'][1]:.1f}", f"{o['rpy_deg'][2]:.1f}", o["n_obs"] if o["n_obs"] is not None else "–", (o["note"] or "–").replace("지지율 ", "")] for o in PJ["poses"][s_]]
            if rows_: d.table([names(s_) + " · 객체", "x (m)", "y (m)", "z (m)", "롤 (°)", "피치 (°)", "요 (°)", "관측 수", note_h], rows_, cls="sum")
            else: d.p(names(s_) + ": 등록된 객체가 없습니다.", "note")
    # ---------- 7 한계 ----------
    d.h2("8. 한계와 다음에 할 일")
    d.ul(["모든 실행이 한 번씩입니다. 실시간 조건에서는 실행마다 처리되는 프레임이 달라지므로, 구성 사이의 몇 %p 차이는 반복 실행으로 확인해야 합니다.",
          f"ObChange의 정답 물체는 구성당 {n_gt}개이고, 카메라 자세가 로봇이 스스로 추정한 값이라 수 cm의 오차가 있습니다. 겹침 비율이 전체적으로 낮게 나옵니다.",
          "직접 수집한 영상은 정답 자세가 없습니다. 등록된 종류와 초과 등록은 이름과 개수만 본 것이고, 깊이 지지율은 그 자리에 그 모양의 표면이 있는지만 봅니다.",
          ("구성 7 · 8은 물체 판정이 제안 관문이고 나머지는 원본 SAM-6D 점수 방식입니다. 7 · 8의 관문 기준값은 데이터마다의 설정 파일을 그대로 썼습니다(ObChange는 YCB용 설정으로 색상 관문이 꺼져 있고, 직접 수집한 영상은 실시간 뷰어 설정으로 켜져 있음). " if 7 in CS else "물체 판정은 여섯 구성 모두 원본 SAM-6D 점수 방식입니다. 제안 관문을 쓰는 구성(7 · 8번)은 아직 돌고 있어 이 표에 없습니다. ") + "ViT-L의 판정 기준(0.5)은 ViT-S에 맞춘 값을 그대로 썼습니다.",
          "설계 1~3의 기준값은 직접 수집한 영상 하나를 보고 정했고 ObChange에 맞춰 조정하지 않았습니다. 설계 3은 두 데이터에서 방향이 갈리므로 기준을 다시 잡아야 합니다.",
          "다음에 할 일: 고른 조합을 세 번 이상 반복해 차이가 유지되는지 확인, 설계 3의 이름 충돌 정리 부분만 따로 떼어 평가, ViT-L용 판정 기준 재설정."])
    d.h2("9. 결과 파일")
    d.ul(["ObChange: <code>objpose/output/six_obchange_261011/</code> (실행 <code>objpose/tools/obchange/run_six_obchange.sh</code>, 지도 <code>designs_obchange.py</code>, 채점 <code>score_designs.py</code>), 보고서 <code>objpose/docs/six_obchange_261011/</code>",
          "직접 수집한 영상: <code>objpose/output/live_six_261010/</code> (실행 <code>objpose/tools/lab/run_live_six.sh</code>, 지도 <code>map_designs.py</code>), 보고서 <code>objpose/docs/live_six_261010/</code>",
          "이 보고서: <code>objpose/docs/summary_261011/build_report.py</code>, 발표자료 <code>~/콜로키움/전체_실험결과.pptx</code>의 \"종합\" 쪽"])

    ref = REF.read_text(); head = ref[: ref.index("<div class=\"wrap\">")].replace("<title>YCB 조합 실험 보고</title>", "<title>객체 지도 실험 종합</title>").replace("</style>", "figure{margin:6px 0 4px}figure img{max-width:100%;height:auto;border:1px solid var(--line);border-radius:4px;display:block;background:#fff}figcaption{margin-top:4px;margin-bottom:10px}</style>")
    hdr = ('<header><div class="kicker">실험 종합 · 데이터셋 × 자세 추정 구성 × 지도 생성 방식 · 2026-10-11</div><h1>객체 지도 실험 종합: 어떤 조합이 가장 좋은가</h1>'
           f'<p class="lead">데이터셋 두 가지(ObChange {ob_names}, 직접 수집한 영상 {len(ses)}개), 객체 자세 추정 구성 여섯 가지, 객체 지도 생성 방식 열두 가지를 모두 조합해 실시간 조건으로 돌린 결과를 한데 모았습니다. 조합마다의 수치와 최종 객체 지도를 보이고, 가장 좋은 조합을 고른 근거를 적었습니다.</p></header>')
    body = hdr + "\n" + "\n".join(x for x, _ in d.parts)
    (HERE / "summary_report.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "summary_report_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "객체 지도 실험 종합")
    nb = ("<p>실험 종합 · 2026-10-11</p>" + re.sub(r'<header>.*?</div>', "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "")) + "\n" + "\n".join(y for _, y in d.parts)
    (HERE / "summary_report_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    print("rooms", [k for k, _ in rooms], "gt", n_gt, "| safe", safe, "bestv", bestv, "topF", topF, "| okc", okc, "bestc", bestc)
    for c in CS: print(c, "OB", f1(T[(c, v0)]["F"]), "→", f1(T[(c, bestv)]["F"]), "tp/fp", T[(c, bestv)]["tp"], T[(c, bestv)]["fp"], "tp50", T[(c, bestv)]["tp50"], "| LAB", LT[(c, bestv)]["kinds"], LT[(c, bestv)]["extra"], "| ms", round(ob_proc[c][2]))
    for v in VK: print(f"{VNAME[v]:24s} OB F1 {A[v]['F']:5.1f} tp {A[v]['tp']:3d} fp {A[v]['fp']:3d} tp50 {A[v]['tp50']:3d} iou {A[v]['iou']:.3f} | LAB {LA[v]['kinds']} {LA[v]['extra']} {LA[v]['sup']:.2f}")
    if a.pptx: pptx(locals())


def pptx(E):
    from PIL import Image
    from pptx import Presentation
    from pptx.util import Inches
    for x in ("docs/ppt/dataset_exp_261005", "docs/ppt/arch_261005", "tools/analysis"): sys.path.insert(0, str(REPO / "objpose" / x))
    from build import slide, lines, text, LM, CW, W
    sys.path.append(str(REPO / "objpose/docs/ppt/fair_results_261005"))
    from replace_tables import title_of
    from append_rooms import grid
    A, T, LA, LT, bestv, bestc, v0, n_gt, lab_max, rooms, ses, ob_names, okc, safe, topF, ob_proc, LR = (E[k] for k in ("A", "T", "LA", "LT", "bestv", "bestc", "v0", "n_gt", "lab_max", "rooms", "ses", "ob_names", "okc", "safe", "topF", "ob_proc", "LR"))
    f1 = lambda x: f"{x:.1f}"; OUT = Path.home() / "콜로키움" / "전체_실험결과.pptx"; PRE = "종합"; prs = Presentation(str(OUT)); ids = prs.slides._sldIdLst
    for i in sorted((i for i, s in enumerate(prs.slides) if title_of(s).startswith(PRE + ":")), reverse=True): prs.part.drop_rel(ids[i].rId); ids.remove(ids[i])

    def pic(s, f, y, hmax, wmax=None):
        iw, ih = Image.open(f).size; w = min(wmax or (W - 2 * 0.5), hmax * iw / ih); h = w * ih / iw; s.shapes.add_picture(str(f), Inches((W - w) / 2), Inches(y), Inches(w), Inches(h)); return h
    # 1 결론
    s = slide(prs, PRE + ": 가장 좋은 조합", "데이터셋 두 가지 × 자세 추정 구성 여섯 가지 × 지도 생성 방식 열두 가지를 모두 실시간 조건으로 돌려 고른 결과다.", "")
    R = [["", "고른 것", "ObChange (정답 있음)", f"직접 수집한 영상 {len(ses)}개 (실제 환경)"],
         ["자세 추정 구성", CFG[bestc], f"F1 {f1(T[(bestc, bestv)]['F'])} %  ·  맞음 {T[(bestc, bestv)]['tp']}/{n_gt}  ·  잘못 등록 {T[(bestc, bestv)]['fp']}", f"{LT[(bestc, bestv)]['kinds']}/{lab_max}종 등록  ·  초과 등록 {LT[(bestc, bestv)]['extra']}"],
         ["지도 생성 방식", VNAME[bestv], f"자세가 정확한 객체(겹침 0.5 이상) {A[v0]['tp50']} → {A[bestv]['tp50']}개", f"깊이 지지율 {LA[v0]['sup']:.2f} → {LA[bestv]['sup']:.2f}, 물체 수는 그대로"]]
    y = grid(s, R, [2.0, 3.6, 3.6, 2.93], 1.75, 0.42, 0.62, 12.5, left=(1, 2, 3))
    lines(s, LM, y + 0.3, CW, 3.2, [
        f"지도 생성 방식은 두 데이터 어디에서도 지금 방식보다 나빠지지 않는 것만 남기고, 그 가운데 자세를 가장 정확하게 만드는 것을 골랐다. 남은 방식: {', '.join(VNAME[v] for v in safe) or '없음'}.",
        f"{VNAME[bestv]}은 물체를 묶고 등록하는 규칙은 그대로 두고 자세만 다시 정한다. 그래서 맞음과 잘못 등록의 수는 같고(ObChange {A[v0]['tp']} → {A[bestv]['tp']}, {A[v0]['fp']} → {A[bestv]['fp']}) 자세만 좋아진다.",
        f"자세 추정 구성은 직접 수집한 영상에서 물건을 놓치지 않는 것({', '.join(map(str, okc))}번) 가운데 ObChange F1이 가장 높은 것을 골랐다. ViT-L 구성은 공룡 인형을 놓쳐 제외됐다.",
        "모든 실행은 한 번씩이다. 구성 사이의 몇 %p 차이는 반복 실행으로 확인이 필요하고, 두 데이터에서 같은 방향으로 나온 것은 지도 방식의 자세 개선이다."], sz=13, gap=7)
    # 2 무엇을 비교했나
    s = slide(prs, PRE + ": 비교한 것", "모든 실행에서 자세 추정과 자세 검증은 켰고, 영상은 원래 속도로 들어오며 처리가 밀리면 최신 프레임만 쓴다.", "")
    R = [["구성", "후보 생성", "마스크", "특징", "ObChange 프레임당 평균 (ms)", "수집 영상에서 처리한 프레임"]] + [[str(c), SPEC[c][0] + ("" if c <= 6 else " · " + SPEC[c][3]), SPEC[c][1], SPEC[c][2], round(ob_proc[c][2]), f"{LR[c]['proc']:,}"] for c in CS]
    y = grid(s, R, [0.9, 3.4, 2.0, 1.2, 2.5, 2.13], 1.7, 0.42, 0.34, 12)
    R = [["지도 생성 방식", "하는 일"], ["지금 방식", "같은 이름의 관측을 30 cm 안에서 묶어 위치는 평균, 방향은 다수결"], ["설계 1 / 1′", "서로 가까운 자세의 가장 큰 무리로 자세를 정한다. 1은 무리가 작으면 객체를 빼고, 1′은 빼지 않는다"],
         ["설계 2", "CAD를 주변 깊이 점에 다시 맞추고, 빈 곳에 뜬 객체를 뺀다"], ["설계 3", "검증에서 떨어진 후보도 증거로 쌓아 미등록 물체를 올리고, 한 자리의 이름 충돌을 정리한다"]]
    y = grid(s, R, [2.2, 9.93], y + 0.2, 0.4, 0.34, 12, left=(0, 1))
    text(s, LM, y + 0.12, CW, 0.5, [[(f"데이터: ObChange {ob_names}(방문 {len(rooms) * 5}회, 정답 물체 {n_gt}개)와 직접 수집한 영상 {len(ses)}개(물체 8종이 하나씩, 정답 자세 없음, 노트북의 실시간 SLAM 사용). 지도 방식은 설계를 하나씩, 둘씩, 셋 모두 쓴 열두 가지.", {"sz": 12, "c": "444444"})]])
    # 3 ObChange
    s = slide(prs, PRE + ": ObChange 결과", f"구성(행) × 지도 생성 방식(열). 위는 F1, 아래는 정답과 0.5 이상 겹치는 객체 수(자세가 정확한 객체). {ob_names}, 구성당 정답 물체 {n_gt}개.", "")
    h = pic(s, MED / "heat_ob_f1.png", 1.6, 2.72); pic(s, MED / "heat_ob_iou.png", 1.6 + h + 0.05, 2.72)
    # 4 수집 영상
    s = slide(prs, PRE + ": 직접 수집한 영상 결과", f"구성(행) × 지도 생성 방식(열). 위는 등록된 물체 종류(최대 {lab_max}), 아래는 같은 종류의 초과 등록. 실제 환경에서 영상 {len(ses)}개.", "")
    h = pic(s, MED / "heat_lab_kinds.png", 1.6, 2.72); pic(s, MED / "heat_lab_extra.png", 1.6 + h + 0.05, 2.72)
    # 5 지도 방식 근거
    s = slide(prs, PRE + ": 지도 생성 방식을 고른 근거", "여섯 구성 합계. 두 데이터 어디에서도 지금 방식보다 나빠지지 않는 방식 가운데, 자세가 정확한 객체가 가장 많은 것을 골랐다.", "")
    R = [["지도 생성 방식", "ObChange\nF1 (%)", "ObChange\n맞음 / 잘못 등록", "ObChange\n겹침 0.5 이상", "수집 영상\n종류", "수집 영상\n초과 등록", "두 데이터 모두\n나빠지지 않음"]]
    for v in VK: R.append([VNAME[v], f1(A[v]["F"]), f"{A[v]['tp']} / {A[v]['fp']}", A[v]["tp50"], LA[v]["kinds"], LA[v]["extra"], "기준" if v == v0 else ("예" if v in safe else "아니오")])
    y = grid(s, R, [3.5, 1.3, 2.1, 1.6, 1.2, 1.3, 1.13], 1.65, 0.62, 0.325, 12, fill=lambda i, j: "eef3f9" if VK[i - 1] == bestv else "ffffff")
    text(s, LM, y + 0.1, CW, 0.5, [[(f"ObChange F1만 보면 {VNAME[topF]}({f1(A[topF]['F'])} %)이 가장 높지만" + (" 고른 방식과 같다." if topF == bestv else f", 직접 수집한 영상에서 종류 {LA[v0]['kinds']} → {LA[topF]['kinds']}, 초과 등록 {LA[v0]['extra']} → {LA[topF]['extra']}로 나빠지는 면이 있다."), {"sz": 12.5})]])
    # 6 구성 근거
    s = slide(prs, PRE + ": 자세 추정 구성을 고른 근거", f"지도 생성 방식은 {VNAME[bestv]}. 직접 수집한 영상에서 물건을 놓치지 않는 구성 가운데 ObChange F1이 가장 높은 것을 골랐다.", "")
    R = [["구성", "수집 영상\n종류 / 초과", "놓치지\n않음", "ObChange\nF1 (%)", "ObChange\n맞음 / 잘못 등록", "ObChange\n겹침 0.5 이상", "프레임당\n평균 (ms)"]]
    for c in CS: R.append([CFG[c], f"{LT[(c, bestv)]['kinds']} / {LT[(c, bestv)]['extra']}", "예" if c in okc else "아니오", f1(T[(c, bestv)]["F"]), f"{T[(c, bestv)]['tp']} / {T[(c, bestv)]['fp']}", T[(c, bestv)]["tp50"], round(ob_proc[c][2])])
    y = grid(s, R, [4.3, 1.6, 1.0, 1.3, 1.9, 1.5, 0.53 + 0.0], 1.7, 0.62, 0.42, 12.5, fill=lambda i, j: "eef3f9" if CS[i - 1] == bestc else "ffffff")
    vl = [c for c in CS if SPEC[c][2] == "ViT-L"]; vs = [c for c in CS if SPEC[c][2] == "ViT-S"]
    lines(s, LM, y + 0.2, CW, 1.6, [f"ViT-L 구성({', '.join(map(str, vl))}번)은 직접 수집한 영상에서 공룡 인형을 곰 인형 쪽으로 판정해 놓쳤다(등록된 종류 {', '.join(str(LT[(c, bestv)]['kinds']) for c in vl)} 대 ViT-S {', '.join(str(LT[(c, bestv)]['kinds']) for c in vs)}).",
                                    f"화면 전체 분할(1, 2번)은 프레임당 {round(ob_proc[1][2])}, {round(ob_proc[2][2])} ms로 문구 상자 구성({round(min(ob_proc[c][2] for c in CS[2:]))}~{round(max(ob_proc[c][2] for c in CS[2:]))} ms)보다 느려 같은 시간에 보는 프레임이 적다.",
                                    f"ObChange는 구성당 정답 물체가 {n_gt}개라 물체 하나에 F1이 약 {100 / n_gt:.0f} %p 움직인다. 순위가 붙어 있는 구성끼리는 반복 실행으로 확인해야 한다."], sz=12.5, gap=5)
    # 7~ 지도
    IO = J(MED / "index_ob_ppt.json") if (MED / "index_ob_ppt.json").exists() else {}; IL = J(MED / "index_ppt.json") if (MED / "index_ppt.json").exists() else {}
    for sc, f in IO.items():
        s = slide(prs, PRE + f": 최종 객체 지도 · ObChange {dict(rooms)[room_of(sc)]} 방문 {sc.rsplit('_s', 1)[1]}", f"{VNAME[bestv]}로 만든 지도. 윗줄이 구성 1~{len(CS) // 2}, 아랫줄이 구성 {len(CS) // 2 + 1}~{len(CS)}이다.", "")
        h = pic(s, MED / f, 1.6, 5.3); text(s, LM, 1.6 + h + 0.06, CW, 0.3, [[("윤곽: 초록 = 맞음,  빨강 = 잘못 등록.  검은 원 = 정답 물체.  배경은 방문 영상 전체로 복원한 장면", {"sz": 11.5, "c": "444444"})]])
    for sc, f in IL.items():
        s = slide(prs, PRE + f": 최종 객체 지도 · {L6.SES[sc].split(' · ')[0]}", f"직접 수집한 영상, {VNAME[bestv]}로 만든 지도. 윗줄이 구성 1~{len(CS) // 2}, 아랫줄이 구성 {len(CS) // 2 + 1}~{len(CS)}이다. 이름표 옆 숫자는 깊이 지지율.", "")
        h = pic(s, MED / f, 1.6, 5.3); text(s, LM, 1.6 + h + 0.06, CW, 0.3, [[("윤곽: 초록 = 그 종류에서 하나,  빨강 = 같은 종류의 초과 등록.  정답 자세가 없는 영상이다", {"sz": 11.5, "c": "444444"})]])
    prs.part.rename_slide_parts([x.rId for x in ids]); prs.save(str(OUT)); print("pptx slides", len(ids), "종합", sum(1 for s in prs.slides if title_of(s).startswith(PRE + ":")))


if __name__ == "__main__":
    main()
