#!/usr/bin/env python3
"""build_easy.py — 실제 환경 실험(구성 여섯 가지 × 영상 8개 × 지도 방식 네 가지)의 결과를 처음 보는 사람도 읽을 수 있게 풀어 쓴 보고서.
  live_six_easy.html / _local.html / _notion.html. 수치는 objpose/output/live_six_261010/ 의 실행 기록과 designs/*.json 에서 읽는다.
    python3 objpose/docs/live_six_261010/build_easy.py"""
import collections, glob, importlib.util, json, os, re
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]; O = REPO / "objpose/output/live_six_261010"; MED = HERE / "media"
spec = importlib.util.spec_from_file_location("live_rep", HERE / "build_report.py"); R = importlib.util.module_from_spec(spec); spec.loader.exec_module(R)
Doc, REF, J, VS, VN = R.Doc, R.REF, R.J, R.VS, R.VN
KO = {"milk": "우유", "choco_hazelnut_high": "초코 상자", "Febreze_high": "페브리즈", "Mugcup_high": "머그컵", "saffron": "세제 통", "Sikhye_high": "식혜 캔", "Bear": "곰 인형", "Dinosaur": "공룡 인형"}
NAME = {1: "① 전체 분할 + 큰 모델", 2: "② 전체 분할 + 작은 모델", 3: "③ 문구 상자 + FastSAM 마스크 + 큰 모델", 4: "④ 문구 상자 + FastSAM 마스크 + 작은 모델", 5: "⑤ 문구 상자 + MobileSAM 마스크 + 큰 모델", 6: "⑥ 문구 상자 + MobileSAM 마스크 + 작은 모델"}


def per_config():
    out = {}
    for c in range(1, 7):
        ms = collections.defaultdict(list); n = fed = 0; cand = collections.Counter(); acc = collections.Counter(); busy = []
        for d in sorted(x for x in glob.glob(str(O / f"c{c}" / "26*")) if os.path.isdir(x)):
            fed += J(d + "/summary.json").get("sam6d_fed") or 0
            for l in open(d + "/sam6d/frames.jsonl"):
                f = json.loads(l); n += 1
                for k, v in f["ms"].items(): ms[k].append(v)
                pc = f["diagnostics"]["pem_candidates"]
                for x in pc: cand[x["object"]] += 1
                for nm in f["objects"]: acc[nm] += 1
                if pc: busy.append(f["ms"]["total"])
        out[c] = dict(frames=n, fed=fed, total=float(np.mean(ms["total"])), ism=float(np.mean(ms["ism"])), pem=float(np.mean(ms["pem"])), busy=float(np.mean(busy)), cand=cand, acc=acc)
    return out


def bars(fname, groups, series, title, ylabel, fmt="{:.0f}", colors=("#5b8fd6", "#e08a1e", "#2e7d32", "#9e9e9e")):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib import font_manager as fmg
    fp = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"; fmg.fontManager.addfont(fp); plt.rcParams["font.family"] = fmg.FontProperties(fname=fp).get_name()
    fig, ax = plt.subplots(figsize=(9.5, 3.6), dpi=140); x = np.arange(len(groups)); w = 0.8 / len(series)
    for i, (lab, vals) in enumerate(series):
        xs = x + (i - (len(series) - 1) / 2) * w; ax.bar(xs, vals, w * 0.9, label=lab, color=colors[i % len(colors)])
        for xi, v in zip(xs, vals): ax.text(xi, v, fmt.format(v), ha="center", va="bottom", fontsize=10)
    ax.set_xticks(x); ax.set_xticklabels(groups, fontsize=11); ax.set_ylabel(ylabel, fontsize=11); ax.set_title(title, fontsize=13, loc="left"); ax.legend(fontsize=10.5, frameon=False, ncol=len(series), loc="upper center", bbox_to_anchor=(0.5, -0.12))
    for sp in ("top", "right"): ax.spines[sp].set_visible(False)
    fig.tight_layout(); MED.mkdir(exist_ok=True); fig.savefig(MED / fname); plt.close(fig); return MED / fname


def main():
    D = R.load(); cfgs = [1, 2, 3, 4, 5, 6]; ses = [s for s in R.SES if all((c, s) in D for c in cfgs)]; K = lambda c: [(c, s) for s in ses]; tot = 8 * len(ses)
    T = {c: {v: R.stat(D, K(c), v) for v in VS} for c in cfgs}; ALL = {v: R.stat(D, [k for c in cfgs for k in K(c)], v) for v in VS}; P = per_config(); d = Doc()
    miss = {c: collections.Counter(n for s in ses for n in KO if n not in {m["name"] for m in D[(c, s)]["V0"]}) for c in cfgs}
    added = sum(D[k]["counts"]["added_by_design3"] for c in cfgs for k in K(c)); weak = sum(D[k]["counts"]["weak_evidence"] for c in cfgs for k in K(c)); obs = sum(D[k]["counts"]["observations"] for c in cfgs for k in K(c))
    # 설계 1 이 뺀 객체 수, 위치 이동
    rem = 0; mv = []
    for c in cfgs:
        for k in K(c):
            used = set()
            for m in D[k]["V0"]:
                cand = [(np.linalg.norm(np.asarray(m["T_w_obj"])[:3, 3] - np.asarray(n["T_w_obj"])[:3, 3]), j) for j, n in enumerate(D[k]["V1"]) if n["name"] == m["name"] and j not in used]
                if cand and min(cand)[0] < 0.30: used.add(min(cand)[1]); mv.append(min(cand)[0] * 1000)
                else: rem += 1
    f1 = lambda v: f"{v:.2f}"

    # ---------------- 0. 한 장 요약 ----------------
    d.h2("한 장 요약")
    d.table(["궁금한 것", "답"], [
        ["<strong>무엇을 했나</strong>", f"로봇이 돌아다니며 찍은 영상 {len(ses)}개에서, 물건 8가지가 <em>어디에 어떤 방향으로</em> 놓여 있는지를 지도로 만들었습니다. 물건을 찾는 방법을 여섯 가지로, 지도를 만드는 방법을 네 가지로 바꿔 가며 결과를 비교했습니다."],
        ["<strong>① 큰 모델(ViT-L)과 작은 모델(ViT-S)</strong>", f"작은 모델이 더 좋았습니다. 큰 모델은 공룡 인형을 거의 찾지 못했고(8개 영상 중 {miss[1]['Dinosaur']}, {miss[3]['Dinosaur']}, {miss[5]['Dinosaur']}개에서 놓침), 계산이 느려서 같은 시간에 볼 수 있는 장면도 적었습니다."],
        ["<strong>② 전체 분할(FastSAM-x)과 문구 상자(YOLO-World)</strong>", f"찾은 물건 수는 거의 같았습니다({T[2]['V0']['kinds']}개 대 {T[4]['V0']['kinds']}, {T[6]['V0']['kinds']}개, 최대 {tot}). 차이는 속도입니다. 문구 상자 쪽이 물건을 알아보는 단계가 약 3배 빨라, 같은 시간에 장면을 {100 * (P[6]['frames'] / P[2]['frames'] - 1):.0f}~{100 * (P[4]['frames'] / P[2]['frames'] - 1):.0f} % 더 처리했습니다."],
        ["<strong>③ 지도 만드는 방법 네 가지</strong>", f"처음 만든 설계 1, 2, 3은 지금 쓰는 방법({ALL['V0']['kinds']}/{tot * 6})을 이기지 못했습니다. 다만 설계 1에서 물건을 빼는 규칙을 없애고 설계 2와 함께 쓰면(설계 1′ + 2), 같은 수의 물건을 올리면서 위치가 더 정확해졌습니다. 설계 3은 어느 조합에서도 잘못 올린 물건만 늘렸습니다."],
        ["<strong>그래서 지금 가장 좋은 조합은</strong>", f"작은 모델(ViT-S)을 쓰는 ②, ④, ⑥번 구성에, 지도는 지금 쓰는 방법이나 설계 1′ + 2입니다. 셋 모두 {tot}개 중 {T[2]['V0']['kinds']}~{T[6]['V0']['kinds']}개를 찾았습니다. 그중 ④번(문구 상자 + FastSAM 마스크 + 작은 모델)이 가장 빠릅니다."]],
        cls="sum", widths=["27%", None])

    # ---------------- 1. 배경 ----------------
    d.h2("1. 먼저 알아야 할 것")
    d.h3("무엇을 만드는 실험인가")
    d.p("로봇이 방 안을 돌아다니면서 카메라로 영상을 찍습니다. 이 영상에서 선반이나 탁자 위에 놓인 물건을 찾아, \"우유는 여기에 이런 방향으로, 머그컵은 저기에\" 하고 적어 둔 것이 <strong>객체 지도</strong>입니다. 사람이 방에 들어가 한 번 둘러보고 물건 위치를 기억하는 것과 같은 일입니다.")
    d.p("이 일을 하려면 두 가지를 알아야 합니다. 카메라(로봇)가 지금 어디에 있는지, 그리고 카메라에 찍힌 물건이 카메라에서 볼 때 어디에 어떤 방향으로 있는지입니다. 앞의 것은 SLAM이라는 프로그램이 맡고, 뒤의 것이 이 실험에서 여러 방법으로 바꿔 본 <strong>물체 자세 추정</strong>입니다.")
    d.h3("물건 하나를 찾는 과정")
    d.table(["단계", "하는 일", "쉬운 비유"], [
        ["1. 후보 찾기", "사진에서 \"물건일 것 같은 영역\"을 고릅니다.", "책상 사진을 보고 \"여기, 여기, 여기에 뭔가 있다\"고 동그라미를 치는 일"],
        ["2. 윤곽 따기", "고른 영역에서 물건의 정확한 테두리(마스크)를 그립니다.", "동그라미 친 곳을 가위로 오려 내는 일"],
        ["3. 무슨 물건인지 알아보기", "오려 낸 조각을, 미리 준비한 물건의 3D 모형 사진들과 비교합니다.", "오려 낸 조각을 물건 도감과 대조하는 일"],
        ["4. 자세 구하기", "그 물건이 카메라에서 얼마나 떨어져 있고 어느 쪽을 보고 있는지 계산합니다.", "\"1 m 앞에, 손잡이가 오른쪽을 향해\"라고 적는 일"],
        ["5. 자세 검증", "계산한 자세가 사진과 정말 맞는지 다시 확인하고, 아니면 버립니다.", "적은 내용을 사진과 한 번 더 대조하는 일"],
        ["6. 지도에 합치기", "여러 장면에서 구한 자세를 모아 지도에 물건 하나로 등록합니다.", "여러 번 본 것을 종합해 수첩에 한 줄로 정리하는 일"]], cls="sum", widths=["20%", "42%", None])
    d.p("이번 실험은 1~3단계를 여섯 가지로 바꿨고(4, 5단계는 항상 같게 켰습니다), 6단계를 네 가지로 바꿨습니다.")
    d.h3("이 보고서에 나오는 낱말")
    d.dl([("FastSAM-x (전체 분할)", "사진 한 장을 넣으면, 무엇인지는 모른 채 눈에 띄는 덩어리를 전부 오려 내는 프로그램입니다. 한 장에 20~40개쯤 나옵니다."),
          ("YOLO-World (문구 상자)", "\"우유 팩\", \"분홍 머그컵\"처럼 글로 적어 주면 사진에서 그것 같은 곳에 네모 상자를 쳐 주는 프로그램입니다. 한 장에 몇 개만 나옵니다."),
          ("MobileSAM", "네모 상자를 주면 그 안의 물건 테두리를 그려 주는 가벼운 프로그램입니다."),
          ("ViT-L, ViT-S", "사진 조각의 특징을 숫자로 바꿔 주는 모델(DINOv2)의 큰 것과 작은 것입니다. 3단계에서 조각과 도감을 비교할 때 씁니다. 큰 것이 더 자세히 보지만 느립니다."),
          ("등록된 종류", f"물건 8가지 가운데 지도에 올라간 가짓수입니다. 영상이 {len(ses)}개라 한 구성의 최고점은 {tot}입니다. 많을수록 좋습니다."),
          ("초과 등록", "같은 물건이 지도에 두 번 이상 올라간 경우입니다. 실제로는 물건이 하나씩뿐이므로 나머지는 잘못 올라간 것입니다. 적을수록 좋습니다."),
          ("깊이 지지율", "지도에 올린 물건 자리에 실제로 그 모양의 표면이 있는지를 깊이 카메라로 확인한 값입니다. 1이면 딱 맞고, 0이면 허공에 떠 있다는 뜻입니다. 이 영상들은 정답이 없어서 대신 쓰는 지표입니다.")])
    d.h3("실험 환경")
    d.ul([f"영상 {len(ses)}개: 두 장소에서 각 4개. 물건 8가지(우유, 초코 상자, 페브리즈, 머그컵, 세제 통, 식혜 캔, 곰 인형, 공룡 인형)가 하나씩 놓여 있습니다.",
          "실제 로봇을 쓸 때와 같은 방식으로 돌렸습니다. 노트북이 카메라 위치를 실시간으로 계산해 보내고, 이 PC가 물건을 찾아 지도를 만듭니다. 영상은 찍힌 속도 그대로 흘러가고, 계산이 늦으면 그 사이 장면은 놓칩니다.",
          "그래서 <strong>느린 방법은 볼 수 있는 장면이 줄어듭니다.</strong> 이것이 결과를 읽을 때 가장 중요한 점입니다."])

    # ---------------- 2. 여섯 구성 ----------------
    d.h2("2. 물건을 찾는 여섯 가지 방법과 결과")
    d.table(["구성", "1. 후보 찾기", "2. 윤곽 따기", "3. 알아보기", "찾은 종류 (최대 " + str(tot) + ")", "초과 등록", "처리한 장면 수", "장면당 시간 (ms)"],
            [[NAME[c].split(" ", 1)[0], "전체 분할 (FastSAM-x)" if c <= 2 else "문구 상자 (YOLO-World)", "FastSAM-x" if c <= 4 else "MobileSAM", "큰 모델 (ViT-L)" if c % 2 else "작은 모델 (ViT-S)", T[c]["V0"]["kinds"], T[c]["V0"]["extra"], f"{P[c]['frames']:,}", round(P[c]["total"])] for c in cfgs],
            cls="sum", group=lambda ri, r: "hl" if ri % 2 == 1 else "")
    d.p("색칠한 줄이 작은 모델을 쓴 구성입니다. 지도는 지금 쓰는 방법으로 만든 것입니다.", "note")

    # ---------------- 3. 질문 1 ----------------
    d.h2("3. 질문 1 — 큰 모델(ViT-L)과 작은 모델(ViT-S)은 왜 차이가 났나")
    d.p("나머지를 똑같이 두고 3단계의 모델만 바꾼 짝이 세 쌍 있습니다(①과 ②, ③과 ④, ⑤와 ⑥).")
    d.table(["짝", "찾은 종류: 큰 모델 → 작은 모델", "처리한 장면: 큰 모델 → 작은 모델", "알아보기 단계 시간: 큰 모델 → 작은 모델"],
            [[f"{a_}번과 {b_}번", f"{T[a_]['V0']['kinds']} → {T[b_]['V0']['kinds']}", f"{P[a_]['frames']:,} → {P[b_]['frames']:,}", f"{P[a_]['ism']:.0f} → {P[b_]['ism']:.0f} ms"] for a_, b_ in ((1, 2), (3, 4), (5, 6))], cls="sum")
    d.fig(bars("q1_dino.png", ["①②", "③④", "⑤⑥"], [("큰 모델 (ViT-L)", [P[c]["cand"]["Dinosaur"] for c in (1, 3, 5)]), ("작은 모델 (ViT-S)", [P[c]["cand"]["Dinosaur"] for c in (2, 4, 6)])], "\"공룡 인형\"으로 알아본 횟수 (영상 8개 합계)", "횟수"),
          "큰 모델을 쓰면 공룡 인형을 알아보는 횟수가 크게 줄어듭니다", width=1100)
    d.h3("이유 1. 큰 모델은 공룡 인형을 알아보지 못했습니다")
    d.ul([f"큰 모델을 쓴 세 구성은 공룡 인형을 8개 영상 중 {miss[1]['Dinosaur']}개, {miss[3]['Dinosaur']}개, {miss[5]['Dinosaur']}개에서 놓쳤습니다. 작은 모델을 쓴 세 구성은 한 번도 놓치지 않았습니다. 찾은 종류의 차이는 거의 전부 이 한 물건에서 나옵니다.",
          f"3단계에서 \"이 조각은 공룡 인형\"이라고 판정한 횟수를 세어 보면, 문구 상자 구성에서 작은 모델은 {P[4]['cand']['Dinosaur']:,}번, 큰 모델은 {P[3]['cand']['Dinosaur']}번입니다.",
          f"같은 구성에서 \"곰 인형\"으로 판정한 횟수는 큰 모델이 더 많습니다({P[3]['cand']['Bear']:,}번 대 {P[4]['cand']['Bear']}번). 곰 인형과 공룡 인형은 크기와 재질이 비슷한 봉제 인형이라, 큰 모델은 공룡 인형 조각을 곰 인형 쪽으로 판정한 것으로 보입니다.",
          "판정은 \"도감과 닮은 정도\"가 기준 점수(0.5)를 넘어야 통과합니다. 이 기준은 모델을 바꿔도 같은 값을 썼습니다. 큰 모델이 내는 점수의 범위가 작은 모델과 달라서, 같은 기준이 큰 모델에는 맞지 않았을 가능성이 있습니다. 이 부분은 점수 분포를 따로 확인해야 확정할 수 있습니다."])
    d.h3("이유 2. 큰 모델은 느려서 장면을 덜 봅니다")
    d.ul([f"전체 분할과 함께 쓰면 차이가 가장 큽니다. 한 장에 조각이 20~40개 나오는데 조각마다 큰 모델을 돌리므로 알아보기 단계가 {P[1]['ism']:.0f} ms 걸립니다(작은 모델은 {P[2]['ism']:.0f} ms). 그 결과 ①번은 장면을 {P[1]['frames']:,}개만 처리했고 ②번은 {P[2]['frames']:,}개를 처리했습니다.",
          f"문구 상자와 함께 쓰면 조각이 몇 개뿐이라 차이가 줄어듭니다({P[3]['ism']:.0f} ms 대 {P[4]['ism']:.0f} ms). 그래도 큰 모델 쪽이 장면을 조금 덜 처리합니다.",
          "지도는 같은 물건을 여러 번 봐야 등록하므로, 본 장면이 적으면 스쳐 지나간 물건이 지도에 올라가지 못합니다."])
    d.p("<strong>정리:</strong> 이 실험에서는 큰 모델이 더 정확하지 않았습니다. 특정 물건(공룡 인형)을 놓쳤고 더 느렸습니다. 큰 모델이 본래 나쁘다기보다, 판정 기준을 작은 모델에 맞춘 채 모델만 바꾼 영향이 큽니다.")

    # ---------------- 4. 질문 2 ----------------
    d.h2("4. 질문 2 — 전체 분할(FastSAM-x)과 문구 상자(YOLO-World)는 왜 차이가 났나")
    d.p("작은 모델을 쓴 세 구성(②, ④, ⑥)을 비교합니다. 3단계 모델이 같으므로 차이는 1, 2단계에서 옵니다.")
    d.table(["구성", "1. 후보 찾기", "2. 윤곽 따기", "찾은 종류", "초과 등록", "처리한 장면", "알아보기 단계 (ms)", "후보가 있는 장면의 전체 시간 (ms)"],
            [[NAME[c].split(" ", 1)[0], "전체 분할" if c <= 2 else "문구 상자", "FastSAM-x" if c <= 4 else "MobileSAM", T[c]["V0"]["kinds"], T[c]["V0"]["extra"], f"{P[c]['frames']:,}", round(P[c]["ism"]), round(P[c]["busy"])] for c in (2, 4, 6)], cls="sum")
    d.fig(bars("q2_time.png", ["② 전체 분할", "④ 문구 상자 + FastSAM", "⑥ 문구 상자 + MobileSAM"], [("알아보기 단계 (ms)", [P[c]["ism"] for c in (2, 4, 6)]), ("자세 구하기 + 검증 (ms)", [P[c]["pem"] for c in (2, 4, 6)])], "장면 하나를 처리하는 평균 시간", "ms"),
          "후보를 전부 오려 내는 ②번은 알아보기 단계가 깁니다. 자세를 구하고 검증하는 시간은 세 구성이 비슷합니다", width=1100)
    d.h3("찾은 물건 수는 거의 같습니다")
    d.ul([f"세 구성 모두 {tot}개 중 {T[2]['V0']['kinds']}~{T[6]['V0']['kinds']}개를 찾았습니다. 물건이 선반과 탁자 위에 잘 보이게 놓여 있어서, 어느 방법으로 후보를 뽑아도 결국 같은 물건에 닿습니다.",
          "전체 분할은 \"무엇을 찾을지 모른 채 다 오려 내므로\" 놓치는 후보가 적고, 문구 상자는 \"찾으려는 것만 골라내므로\" 쓸데없는 후보가 적습니다. 이 영상들에서는 두 장점이 비겼습니다."])
    d.h3("차이는 속도에서 납니다")
    d.ul([f"전체 분할은 조각을 많이 만들고 그 조각마다 도감과 비교하므로 알아보기 단계가 평균 {P[2]['ism']:.0f} ms입니다. 문구 상자는 {P[4]['ism']:.0f}~{P[6]['ism']:.0f} ms로 약 3분의 1입니다.",
          f"그만큼 같은 시간에 더 많은 장면을 처리합니다({P[2]['frames']:,}개 대 {P[4]['frames']:,}, {P[6]['frames']:,}개).",
          "\"전체 분할은 점을 하나씩 찍어 가며 후보를 뽑아 느리다\"는 것은 원래의 SAM 이야기입니다. FastSAM은 그 부분을 한 번에 처리하도록 바꾼 것이라 오려 내는 것 자체는 빠릅니다(약 15 ms). 느린 것은 오려 낸 조각이 많아 뒤의 비교가 길어지기 때문입니다.",
          f"전체 시간에서 가장 큰 몫은 어느 구성이든 4, 5단계(자세 구하기와 검증)입니다. 그래서 1~3단계에서 줄인 시간이 처리한 장면 수로는 {100 * (P[6]['frames'] / P[2]['frames'] - 1):.0f}~{100 * (P[4]['frames'] / P[2]['frames'] - 1):.0f} % 차이로 나타납니다."])
    d.h3("문구 상자 안에서, 윤곽을 FastSAM으로 따는 것과 MobileSAM으로 따는 것")
    d.ul([f"④번(FastSAM 마스크)과 ⑥번(MobileSAM 마스크)은 찾은 종류가 {T[4]['V0']['kinds']}개와 {T[6]['V0']['kinds']}개, 초과 등록이 {T[4]['V0']['extra']}개와 {T[6]['V0']['extra']}개입니다. 사실상 같습니다.",
          f"④번이 조금 더 빨라 장면을 더 처리했습니다({P[4]['frames']:,}개 대 {P[6]['frames']:,}개)."])
    d.p("<strong>정리:</strong> 무엇을 찾을지 미리 아는 상황(물건 목록이 정해진 경우)에서는 문구 상자가 같은 결과를 더 빨리 냅니다. 전체 분할이 유리한 것은 문구로 설명하기 어려운 물건이 있거나, 문구 검출기가 그 물건에 반응하지 않는 경우입니다.")

    # ---------------- 5. 질문 3 ----------------
    d.h2("5. 질문 3 — 지도를 만드는 네 가지 방법")
    d.h3("왜 다른 방법을 생각했나")
    d.p("지금 쓰는 방법은 4, 5단계가 내준 자세를 그대로 믿고 모으기만 합니다. 그래서 자세 추정이 틀리면 지도도 그대로 틀립니다. 지도를 만드는 쪽에서도 스스로 확인하는 일을 늘리면 덜 흔들리지 않을까 하는 생각에서 세 가지 설계를 만들어 보았습니다. 네 방법 모두 <strong>같은 실행 기록</strong>으로 지도만 다시 만든 것이라, 물건을 찾은 결과는 완전히 같고 합치는 방법만 다릅니다.")
    d.table(["방법", "어떻게 합치나", "쉬운 비유"], [
        ["<strong>지금 방법</strong>", "같은 이름의 물건이 30 cm 안에서 여러 번 보이면 하나로 묶습니다. 위치는 평균을 내고, 방향은 가장 많이 나온 것을 고릅니다. 자리가 비어 보이면 지웁니다.", "여러 사람이 말한 위치의 평균을 적는 것"],
        ["<strong>설계 1. 합의로 합치기</strong>", "여러 번 구한 자세 가운데 서로 가까운 것끼리 무리를 짓고, 가장 큰 무리만 씁니다. 무리에서 동떨어진 자세는 버립니다. 무리가 너무 작으면 그 물건은 지도에서 뺍니다.", "여러 사람 말 중에 서로 일치하는 다수의 말만 믿고, 혼자 다른 말을 하는 사람은 빼는 것"],
        ["<strong>설계 2. 깊이로 확인하고 다듬기</strong>", "설계 1의 결과에 더해, 물건의 3D 모형을 지도 자리에 놓고 깊이 카메라가 본 실제 표면과 맞는지 확인합니다. 조금 어긋나면 표면에 맞게 밀어 넣고, 허공에 떠 있으면 지웁니다.", "적어 둔 자리에 직접 손을 뻗어 물건이 만져지는지 확인하는 것"],
        ["<strong>설계 3. 약한 증거 누적</strong>", "설계 2의 결과에 더해, 5단계 검증에서 떨어져 버려졌던 후보도 \"여기 뭔가 그 물건 같은 게 있었다\"는 약한 증거로 모읍니다. 증거가 많이 쌓인 자리는 지도에 올립니다.", "확신은 없지만 \"저기서 본 것 같다\"는 말이 여러 번 나오면 수첩에 적는 것"]], cls="sum", widths=["20%", "50%", None])
    d.h3("결과")
    d.table(["지도 방법", "찾은 종류 (최대 " + str(tot * 6) + ")", "초과 등록", "깊이 지지율 평균", "한 줄 평가"], [
        ["지금 방법", ALL["V0"]["kinds"], ALL["V0"]["extra"], f1(ALL["V0"]["sup"]), "물건을 가장 많이 올림"],
        ["설계 1", ALL["V1"]["kinds"], ALL["V1"]["extra"], f1(ALL["V1"]["sup"]), f"위치는 더 정확해졌지만 물건 {rem}개를 지나치게 뺌"],
        ["설계 1 + 2", ALL["V2"]["kinds"], ALL["V2"]["extra"], f1(ALL["V2"]["sup"]), "설계 1에서 거의 달라지지 않음"],
        ["설계 1 + 2 + 3", ALL["V3"]["kinds"], ALL["V3"]["extra"], f1(ALL["V3"]["sup"]), "초과 등록만 늘어남"]], cls="sum", group=lambda ri, r: "hl" if ri == 0 else "")
    d.p("여섯 구성, 영상 8개를 모두 합친 값입니다.", "note")
    if (MED / "glance.png").exists(): d.fig(MED / "glance.png", "구성별로 본 같은 결과. 회색 = 지금 방법, 파랑 = 설계 1, 초록 = 설계 1+2, 주황 = 설계 1+2+3", width=1500)
    d.h3("설계 1: 위치는 좋아졌지만, 빼는 규칙이 지나쳤습니다")
    d.ul([f"<strong>좋아진 점.</strong> 물건이 실제 표면에 더 잘 맞게 놓였습니다(깊이 지지율 {f1(ALL['V0']['sup'])} → {f1(ALL['V1']['sup'])}). 지도에 적힌 위치는 보통 {np.median(mv):.0f} mm, 많이 움직인 경우 {np.percentile(mv, 90):.0f} mm 옮겨졌습니다.",
          f"<strong>나빠진 점.</strong> \"자세가 한 무리로 모이지 않는다\"며 물건 {rem}개를 지도에서 뺐는데, 그 가운데 진짜 물건이 들어 있었습니다. 찾은 종류가 {ALL['V0']['kinds']} → {ALL['V1']['kinds']}로 줄었습니다.",
          "<strong>원인.</strong> 카메라 위치 추정에 몇 cm의 오차가 있어서, 같은 물건이라도 본 시점에 따라 자세가 조금씩 다르게 찍힙니다. 서로 2.5 cm 안에 있어야 한 무리로 치는 기준이 이 오차보다 엄격했습니다."])
    if (MED / "example_design1.jpg").exists(): d.fig(MED / "example_design1.jpg", "설계 1이 위치를 고친 예. 왼쪽이 지금 방법, 오른쪽이 설계 1. 선반 아랫단의 곰 인형이 약 9 cm 옮겨져 실제 자리에 맞았습니다(이름 옆 숫자가 깊이 지지율, 0.6 → 1.0)", width=1300)
    d.h3("설계 2: 할 일이 없었습니다")
    d.ul(["\"허공에 떠 있는 물건을 지운다\"는 규칙에 걸린 물건이 하나도 없었습니다. 5단계 검증을 통과한 자세는 이미 대부분 실제 표면 위에 있기 때문입니다.",
          f"표면에 맞게 밀어 넣는 다듬기로 깊이 지지율이 {f1(ALL['V1']['sup'])} → {f1(ALL['V2']['sup'])}가 됐습니다. 다만 이 방법은 깊이 지지율이 오르는 쪽으로 자세를 고르므로, 이 숫자가 오른 것만으로 더 정확해졌다고 말하기는 어렵습니다."])
    d.h3("설계 3: 오히려 나빠졌습니다")
    d.ul([f"버려졌던 후보 {weak:,}개를 증거로 모아 물건 {added}개를 새로 지도에 올렸습니다. 그런데 새로 찾은 종류는 {ALL['V3']['kinds'] - ALL['V2']['kinds']}개뿐이고, 나머지는 이미 지도에 있는 물건이 다른 자리에 한 번 더 올라간 것이었습니다. 초과 등록이 {ALL['V2']['extra']} → {ALL['V3']['extra']}개로 늘었습니다.",
          "검증에서 떨어진 후보는 대부분 \"다른 물건을 그 물건으로 잘못 본 것\"이었습니다. 이런 것이 같은 자리에서 반복되면 증거 개수가 쌓이므로, 개수만으로는 진짜와 가짜를 가릴 수 없었습니다.",
          "놓친 물건을 되살리려던 목적은 이루지 못했습니다."])
    d.h3("설계를 따로따로, 또 섞어서 써 보면")
    d.p("위 표는 설계를 1 → 1+2 → 1+2+3으로 쌓아 간 결과입니다. 각 설계가 혼자서는 어떤지, 둘씩 섞으면 어떤지도 돌려 보았습니다. 또 설계 1에서 문제였던 \"무리가 작으면 물건을 뺀다\"는 규칙만 없앤 것을 <strong>설계 1′</strong>이라 부르고 함께 넣었습니다.")
    d.table(["지도 방법", f"찾은 종류 (최대 {tot * 6})", "초과 등록", "깊이 지지율 평균", "지지율 0.5 이상 (%)"], R.combo_rows(D, cfgs, ses, per_config=False), cls="sum", group=lambda ri, r: "hl" if r[0] in ("지금 방식", "설계 1′ + 2") else "")
    c1n = R.stat(D, [(c, s_) for c in cfgs for s_ in ses], "1n"); c1n2 = R.stat(D, [(c, s_) for c in cfgs for s_ in ses], "1n+2"); c2 = R.stat(D, [(c, s_) for c in cfgs for s_ in ses], "2"); c3 = R.stat(D, [(c, s_) for c in cfgs for s_ in ses], "3")
    d.ul([f"<strong>설계 1′(빼는 규칙 없앰).</strong> 찾은 종류 {c1n['kinds']}개, 초과 등록 {c1n['extra']}개로 지금 방법과 똑같고, 깊이 지지율만 {f1(ALL['V0']['sup'])} → {f1(c1n['sup'])}로 올랐습니다. 설계 1에서 물건이 줄었던 것은 전부 빼는 규칙 탓이었습니다.",
          f"<strong>설계 2만.</strong> 지금 방법의 자세를 실제 표면에 맞게 다듬어 지지율이 {f1(c2['sup'])}가 됐습니다. 대신 물건 두 개를 지워 종류가 {c2['kinds']}개로 하나 줄었습니다.",
          f"<strong>설계 3만.</strong> 초과 등록이 {ALL['V0']['extra']} → {c3['extra']}개로 늘고 종류는 오히려 {c3['kinds']}개로 줄었습니다. 다른 설계와 섞어도 초과 등록이 14~15개씩 늘어납니다.",
          f"<strong>설계 1′ + 2.</strong> 찾은 종류 {c1n2['kinds']}개, 초과 등록 {c1n2['extra']}개, 지지율 {f1(c1n2['sup'])}. 물건을 하나도 잃지 않으면서 지지율이 가장 높은 축입니다. 설계 1′이 자세를 먼저 바로잡아 주므로 설계 2의 확인에서 빠지는 물건이 없습니다."])
    d.h3("어느 것이 가장 좋은가")
    d.table(["기준", "가장 좋은 방법", "이유"], [
        ["물건을 빠뜨리지 않는가", "<strong>지금 방법</strong>", f"찾은 종류가 가장 많습니다({ALL['V0']['kinds']}/{tot * 6})."],
        ["잘못 올린 것이 적은가", "지금 방법, 설계 1, 설계 1+2가 비슷", f"초과 등록 {ALL['V0']['extra']}, {ALL['V1']['extra']}, {ALL['V2']['extra']}개. 설계 3만 {ALL['V3']['extra']}개로 많습니다."],
        ["위치가 정확한가", "설계 1′ + 2", "실제 표면에 가장 잘 맞습니다. 다만 정답과 비교한 것이 아니라 깊이 영상으로 본 간접 확인입니다."],
        ["셋을 함께 보면", "<strong>설계 1′ + 2</strong>", "지금 방법과 같은 수의 물건을 올리면서 위치가 더 정확합니다."]], cls="sum", widths=["24%", "26%", None])
    d.p("<strong>결론:</strong> 원래 만든 설계 1, 2, 3 가운데서는 지금 방법을 이기는 것이 없었습니다. 그러나 설계 1에서 물건을 빼는 규칙을 없애면(설계 1′) 지금 방법과 똑같은 물건을 올리면서 위치가 더 정확해지고, 여기에 설계 2를 더한 <strong>설계 1′ + 2</strong>가 이번 실험에서 가장 나은 조합입니다. 설계 3은 어떤 조합에서도 잘못 올린 물건만 늘려서 지금 형태로는 쓸 이유가 없습니다. 다만 \"위치가 더 정확하다\"는 판단은 깊이 영상으로 본 간접 확인이므로, 정답이 있는 데이터로 한 번 더 확인해야 합니다.")
    d.p("한 가지 더 눈여겨볼 점은, 자세 검증(5단계)을 항상 켠 조건에서는 지도 쪽에서 더 걸러 낼 것이 많지 않았다는 것입니다. 이전 실험에서 검증을 끄면 잘못 올린 물건이 몇 배로 늘었습니다. 지도의 품질은 지도를 합치는 방법보다 자세 검증에서 더 많이 결정됩니다.", "note")

    # ---------------- 6. 주의 ----------------
    d.h2("6. 이 결과를 볼 때 주의할 점")
    d.ul(["<strong>정답이 없는 영상입니다.</strong> 물건의 정확한 위치를 따로 재어 둔 것이 없어서, \"8가지 중 몇 가지가 올라갔나\", \"같은 것이 두 번 올라갔나\", \"그 자리에 표면이 있나\"만 볼 수 있습니다. 이름을 틀리게 붙였는데 모양이 비슷한 경우는 잡아내지 못합니다.",
          "<strong>한 번씩만 돌렸습니다.</strong> 실시간으로 돌리면 실행할 때마다 처리되는 장면이 조금씩 달라집니다. 종류 한두 개의 차이는 우연일 수 있습니다.",
          "<strong>물건을 알아보는 판정 방식은 여섯 구성이 같습니다.</strong> 원본 SAM-6D의 점수 방식(기준 0.5)으로 통일했습니다. 원래 제안 방법이 쓰는 판정 방식(여러 기준을 모두 통과해야 하는 방식)은 작은 모델용 기준값만 있어서 이번 여섯 구성에는 넣지 않았습니다.",
          "<strong>③, ④번의 뜻.</strong> YOLO-World가 친 상자마다, FastSAM-x가 오려 낸 조각 가운데 그 상자와 가장 잘 겹치는 것 하나를 골라 쓰는 방식으로 돌렸습니다.",
          "<strong>설계 1~3의 기준값</strong>은 영상 하나의 결과를 보고 정했고 따로 다듬지 않았습니다. 기준을 바꾸면 결과가 달라질 수 있습니다.",
          "<strong>장소 B의 영상 두 개</strong>는 카메라 녹화가 하나뿐이라, 같은 영상을 위치 계산과 물건 찾기에 함께 썼습니다."])
    d.h2("7. 더 자세한 자료")
    d.ul(["수치와 영상별 지도 그림이 모두 들어 있는 보고서: <code>objpose/docs/live_six_261010/live_six_report_local.html</code>", "발표자료: <code>~/콜로키움/전체_실험결과.pptx</code> 58~69쪽",
          "실행 기록: <code>objpose/output/live_six_261010/</code>, 이 보고서를 만든 스크립트: <code>objpose/docs/live_six_261010/build_easy.py</code>"])

    ref = REF.read_text(); head = ref[: ref.index("<div class=\"wrap\">")].replace("<title>YCB 조합 실험 보고</title>", "<title>객체 지도 실험 쉽게 읽기</title>").replace("</style>", "figure{margin:6px 0 4px}figure img{max-width:100%;height:auto;border:1px solid var(--line);border-radius:4px;display:block;background:#fff}figcaption{margin-top:4px;margin-bottom:10px}</style>")
    hdr = ('<header><div class="kicker">쉽게 읽는 실험 보고 · 2026-10-10</div><h1>물건을 찾아 지도에 올리는 여러 방법, 무엇이 달랐나</h1>'
           f'<p class="lead">로봇이 찍은 영상 {len(ses)}개에서 물건 8가지의 위치와 방향을 지도로 만드는 실험입니다. 물건을 찾는 방법 여섯 가지와 지도를 만드는 방법 네 가지를 실제 사용 환경 그대로 돌려 비교했습니다. 전문 용어는 처음 나올 때 풀어서 설명했습니다.</p></header>')
    body = hdr + "\n" + "\n".join(x for x, _ in d.parts)
    (HERE / "live_six_easy.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "live_six_easy_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "객체 지도 실험 쉽게 읽기")
    nb = ("<p>쉽게 읽는 실험 보고 · 2026-10-10</p>" + re.sub(r'<header>.*?</div>', "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "")) + "\n" + "\n".join(y for _, y in d.parts)
    (HERE / "live_six_easy_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    for f in ("live_six_easy.html", "live_six_easy_local.html", "live_six_easy_notion.html"): print(f, (HERE / f).stat().st_size)
    print({c: (T[c]["V0"]["kinds"], T[c]["V0"]["extra"], P[c]["frames"], round(P[c]["ism"]), round(P[c]["pem"]), P[c]["cand"]["Dinosaur"], P[c]["cand"]["Bear"], dict(miss[c])) for c in cfgs}); print("rem", rem, "mv", np.median(mv), np.percentile(mv, 90), "added", added, "weak", weak)


if __name__ == "__main__":
    main()
