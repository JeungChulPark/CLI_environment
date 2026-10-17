#!/usr/bin/env python3
"""build_report.py — 객체 지도 비교 실험(제안 · FM-Fusion · DualMap) 보고서를 HTML 세 가지로 만든다.
  objmap_eval_report.html         꾸밈 있는 판(조각 형식, ycb_combo_261008 보고서와 같은 양식)
  objmap_eval_report_local.html   같은 내용, 혼자 열리는 판
  objmap_eval_report_notion.html  표와 글만 있는 판(붙여 넣기용, 그림 없음)
수치는 모두 채점 결과 json 에서 읽는다. 그림은 발표자료용으로 만들어 둔 것을 JPEG 로 줄여 넣는다.
    python3 objpose/docs/objmap_eval_261009/build_report.py"""
import base64, collections, io, json, re, sys
from pathlib import Path
import numpy as np
from PIL import Image

HERE = Path(__file__).parent; REPO = Path(__file__).resolve().parents[3]; O = REPO / "objpose/output"; FR = REPO / "objpose/docs/ppt/fair_results_261005"; MED = FR / "media"
for d in ("docs/ppt/dataset_exp_261005", "docs/ppt/arch_261005", "tools/analysis"): sys.path.insert(0, str(REPO / "objpose" / d))
import build as _b                                    # noqa: F401,E402  (dataset_exp 의 build 를 먼저 불러 이름 충돌을 막는다)
import summarize_fair as SF                           # noqa: E402
sys.path.append(str(FR))
from replace_tables import boot, CAT                  # noqa: E402
from append_rooms import room_data                    # noqa: E402

REF = REPO / "objpose/docs/ycb_combo_261008/ycb_combo_report.html"
M3 = ["제안 방법", "FM-Fusion", "DualMap"]
V = O / "fmf_cmp_260928/eval_own"; M = O / "fmf_cmp_ycbm_260929/eval_own"; OBE = O / "obchange_261005/eval"; PL = O / "dualmap_plain_261005"
KOR = {"002_master_chef_can": "커피 캔", "003_cracker_box": "크래커 상자", "004_sugar_box": "설탕 상자", "005_tomato_soup_can": "수프 캔", "008_pudding_box": "푸딩 상자", "009_gelatin_box": "젤라틴 상자",
       "019_pitcher_base": "피처", "024_bowl": "그릇", "025_mug": "머그컵", "040_large_marker": "마커", "061_foam_brick": "스펀지 벽돌"}
EN = {"master_chef_can": "커피 캔", "cracker_box": "크래커 상자", "sugar_box": "설탕 상자", "tomato_soup_can": "수프 캔", "mustard_bottle": "머스터드 병", "tuna_fish_can": "참치 캔", "pudding_box": "푸딩 상자",
      "gelatin_box": "젤라틴 상자", "potted_meat_can": "통조림 햄 캔", "banana": "바나나", "pitcher_base": "피처", "bleach_cleanser": "표백제 통", "bowl": "그릇", "mug": "머그컵", "power_drill": "전동 드릴",
      "wood_block": "나무 블록", "scissors": "가위", "large_marker": "마커", "large_clamp": "큰 클램프", "extra_large_clamp": "아주 큰 클램프", "foam_brick": "스펀지 벽돌"}
J = lambda p: json.loads(Path(p).read_text())
f1 = lambda v: f"{v:.1f}"


# ---------- HTML 조각 ----------
class Doc:
    def __init__(self): self.parts = []
    def add(self, rich, plain=None): self.parts.append((rich, rich if plain is None else plain))
    def h2(self, t): self.add(f"<h2>{t}</h2>")
    def h3(self, t): self.add(f"<h3>{t}</h3>")
    def p(self, t, cls=None): self.add(f'<p class="{cls}">{t}</p>' if cls else f"<p>{t}</p>", f"<p>{t}</p>")
    def ul(self, items): self.add("<ul>" + "".join(f"<li>{x}</li>" for x in items) + "</ul>")
    def dl(self, items):
        self.add('<dl class="cond">' + "".join(f"<dt>{a}</dt><dd>{b}</dd>" for a, b in items) + "</dl>",
                 "<table><tbody>" + "".join(f"<tr><th>{a}</th><td>{b}</td></tr>" for a, b in items) + "</tbody></table>")
    def table(self, head, rows, cls="sum", bars=(), group=None, widths=None):
        """rows 의 칸: 글자 또는 숫자. bars = 막대를 그릴 열 번호(0~100 값). group = 색 띠를 줄 행 판별 함수."""
        th = "".join(f'<th style="width:{widths[i]}">{h}</th>' if widths and widths[i] else f"<th>{h}</th>" for i, h in enumerate(head))
        rich, plain = [], []
        for ri, r in enumerate(rows):
            cr, cp = [], []
            for ci, c in enumerate(r):
                isnum = isinstance(c, (int, float, np.integer, np.floating)); txt = (f1(c) if isinstance(c, (float, np.floating)) else str(c))
                if ci in bars and isnum: cr.append(f'<td class="num"><span class="bar {"b2" if bars.index(ci) % 2 else "b1"}" style="--w:{max(0, min(100, float(c))):.1f}%"></span><span class="v">{txt}</span></td>')
                elif isnum: cr.append(f'<td class="num">{txt}</td>')
                else: cr.append(f"<td>{txt}</td>")
                cp.append(f"<td>{txt}</td>")
            k = group(ri, r) if group else ""
            rich.append(f'<tr class="{k}">' + "".join(cr) + "</tr>"); plain.append("<tr>" + "".join(cp) + "</tr>")
        self.add(f'<div class="scroll"><table class="{cls}"><thead><tr>{th}</tr></thead><tbody>' + "".join(rich) + "</tbody></table></div>",
                 "<table><thead><tr>" + "".join(f"<th>{h}</th>" for h in head) + "</tr></thead><tbody>" + "".join(plain) + "</tbody></table>")
    def fig(self, path, cap, width=1500):
        im = Image.open(path).convert("RGB")
        if im.width > width: im = im.resize((width, round(im.height * width / im.width)), Image.LANCZOS)
        buf = io.BytesIO(); im.save(buf, "JPEG", quality=82); b = base64.b64encode(buf.getvalue()).decode()
        self.add(f'<figure><img alt="{cap}" src="data:image/jpeg;base64,{b}"><figcaption class="note">{cap}</figcaption></figure>', f"<p><em>(그림 생략: {cap})</em></p>")


def stats_of(rows): return {m: SF.stats([r for r in rows if r["method"] == m]) for m in M3}


def f1ci(rows, m):
    scenes = sorted({r["scene"] for r in rows}); by = {sc: [r for r in rows if r["method"] == m and r["scene"] == sc] for sc in scenes}
    c = boot(scenes, lambda ss: (lambda t: t["F"] if t else None)(SF.stats([r for sc in ss for r in by[sc]]))); return f"{c[0]:.0f}–{c[1]:.0f}"


def key(n):
    n = n.replace("ycbv_", ""); return n[4:] if n[:3].isdigit() else n


def main():
    Y = J(V / "inst_iou_final.json"); Ym = J(M / "inst_iou_final.json"); rooms = [("kitchen", "Kitchen"), ("office", "Office"), ("livingarea", "LivingArea"), ("smallroom", "SmallRoom")]
    RM = {r: J(OBE / f"{r}_final.json") for r, _ in rooms}; Ob = [x for r, _ in rooms for x in RM[r]]
    DS = [("YCB-Video", Y), ("YCB-M", Ym), ("ObChange 네 방", Ob)]; ST = {d: stats_of(rows) for d, rows in DS}
    d = Doc()

    # ---------- 1 요약 ----------
    fpsum = {m: sum(ST[x][m]["wrong"] for x, _ in DS) for m in M3}
    FIT = {"YCB-Video": J(V / "fit_pose.json"), "YCB-M": J(M / "fit_pose.json")}
    far = lambda ds, m: 100 * float(np.mean([r["rot_sym_deg"] > 30 for r in FIT[ds]["rows"] if r["method"] == m]))
    stab = J(O / "checkpoints_261006/stability.json"); SB = {}
    for m in M3:
        Rm = [r for r in stab if r["method"] == m]; keys = sorted({(r["scene"], r["object"]) for r in Rm}); step, lost, ever, flips = [], 0, 0, 0
        for k in keys:
            seq = sorted((r for r in Rm if (r["scene"], r["object"]) == k), key=lambda r: r["pp"]); s_ = [r["state"] for r in seq]
            if "ok" in s_: ever += 1; lost += s_[-1] != "ok"; i0 = s_.index("ok"); flips += sum(1 for a, b in zip(s_[i0:], s_[i0 + 1:]) if (a == "ok") != (b == "ok"))
            Pz = [(r["pp"], np.array(r["pos"])) for r in seq if r["pos"] is not None]; step += [np.linalg.norm(q - p) * 1000 for (a, p), (b, q) in zip(Pz, Pz[1:]) if b - a == 10]
        SB[m] = dict(ever=ever, lost=lost, flips=flips, p50=float(np.median(step)), p95=float(np.percentile(step, 95)))
    rep = {}
    for f in sorted((Path.home() / "DeepLearning/Datasets/Replica_dl").glob("eval_*_*.log")):
        mm = re.search(r"\{'fmiou': ([0-9.]+),\s*'macc': ([0-9.]+),\s*'miou': ([0-9.]+)\}", f.read_text(errors="ignore"))
        if mm: rep[f.stem.replace("eval_", "").replace("_", "")] = (float(mm.group(3)), float(mm.group(1)), float(mm.group(2)))
    rmean = [float(np.mean([v[i] for v in rep.values()])) for i in range(3)] if rep else [float("nan")] * 3
    sweep = [("제안 방법과 같은 문구 (색 포함)", "0.2", "기본 (2 cm)", "검출기 다수결", V / "inst_iou_tau20.json"), ("일반 이름", "0.2", "기본 (2 cm)", "검출기 다수결", PL / "ycbv_c20/inst_plain_c20.json"),
             ("일반 이름", "0.2", "기본 (2 cm)", "CLIP 특징", PL / "ycbv_c20/inst_clip_named.json"), ("일반 이름", "0.05", "기본 (2 cm)", "검출기 다수결", PL / "ycbv/inst_plain_c05.json"),
             ("일반 이름", "0.05", "기본 (2 cm)", "CLIP 특징", PL / "ycbv/inst_clip_named.json"), ("일반 이름", "0.05", "기본 (2 cm)", "CLIP 특징, 모든 객체", PL / "ycbv/inst_clip_all.json"),
             ("일반 이름", "0.05", "탁상 규모 (5 mm)", "검출기 다수결", PL / "ycbv_tt/inst_plain_c05_tt.json"), ("일반 이름", "0.05", "탁상 규모 (5 mm)", "CLIP 특징", PL / "ycbv_tt/inst_clip_named.json")]
    SW = [(a, b, c, e, SF.stats([r for r in J(f) if r["method"] == "DualMap"])) for a, b, c, e, f in sweep]; swF = [x[4]["F"] for x in SW]
    yv, ym, ob = ST["YCB-Video"], ST["YCB-M"], ST["ObChange 네 방"]
    d.h2("1. 요약")
    d.table(["질문", "결과"], [
        ["<strong>지도에 올라간 객체가 맞는가</strong>", f"제안 방법이 가장 정확합니다. 잘못 올린 객체(FP)가 세 데이터를 합쳐 제안 방법 {fpsum['제안 방법']}개, FM-Fusion {fpsum['FM-Fusion']}개, DualMap {fpsum['DualMap']}개입니다. "
         f"F1 score는 YCB-Video {f1(yv['제안 방법']['F'])} / {f1(yv['FM-Fusion']['F'])} / {f1(yv['DualMap']['F'])} %, YCB-M {f1(ym['제안 방법']['F'])} / {f1(ym['FM-Fusion']['F'])} / {f1(ym['DualMap']['F'])} %, "
         f"ObChange 네 방 {f1(ob['제안 방법']['F'])} / {f1(ob['FM-Fusion']['F'])} / {f1(ob['DualMap']['F'])} %입니다(제안 방법 / FM-Fusion / DualMap 순)."],
        ["<strong>정답 물체를 빠뜨리지 않는가</strong>", f"제안 방법은 FM-Fusion보다 많이 빠뜨리는 데이터가 둘입니다. 정답 물체의 {100 - yv['제안 방법']['R']:.0f} %(YCB-Video), {100 - ym['제안 방법']['R']:.0f} %(YCB-M), {100 - ob['제안 방법']['R']:.0f} %(ObChange)를 지도에 올리지 못했습니다. "
         f"Recall은 YCB-M에서 FM-Fusion {f1(ym['FM-Fusion']['R'])} % 대 제안 방법 {f1(ym['제안 방법']['R'])} %, ObChange에서 {f1(ob['FM-Fusion']['R'])} % 대 {f1(ob['제안 방법']['R'])} %입니다. 놓친 물체는 여섯 종류에 몰려 있습니다."],
        ["<strong>지도에 저장된 6D 자세는 정확한가</strong>", f"YCB-Video에서 ADD(-S) 중앙값 {J(V / 'pose_final.json')['summary']['add_s_mm_p50']:.1f} mm, 방향 오차 중앙값 {J(V / 'pose_final.json')['summary']['rot_deg_p50']:.1f}°입니다. "
         f"비교 방법의 객체 점군에 CAD를 맞춰 자세를 구하면 방향이 30° 넘게 틀리는 비율이 YCB-Video에서 FM-Fusion {far('YCB-Video', 'FM-Fusion'):.0f} %, DualMap {far('YCB-Video', 'DualMap'):.0f} %이고 제안 방법은 {far('YCB-Video', '제안 방법'):.0f} %입니다."],
        ["<strong>지도가 만들어지는 동안 안정적인가</strong>", f"영상을 10 %씩 더 넣을 때 객체 중심이 움직인 거리의 중앙값이 제안 방법 {SB['제안 방법']['p50']:.1f} mm, FM-Fusion {SB['FM-Fusion']['p50']:.1f} mm, DualMap {SB['DualMap']['p50']:.1f} mm입니다. "
         f"맞는 상태와 아닌 상태 사이를 오간 횟수는 {SB['제안 방법']['flips']}회, {SB['FM-Fusion']['flips']}회, {SB['DualMap']['flips']}회입니다."],
        ["<strong>DualMap의 수치가 낮은 것은 설정 탓인가</strong>", f"아닙니다. 문구, 검출 기준값, 점군 해상도, 이름을 정하는 방식을 바꿔 여덟 가지로 돌려도 YCB-Video의 F1 score가 {min(swF):.0f}~{max(swF):.0f} %입니다. "
         f"고치지 않은 DualMap을 Replica 여덟 장면에 돌리면 mIoU {rmean[0]:.3f}로 논문의 0.254와 거의 같아, 설치에는 문제가 없습니다."],
        ["<strong>쓴 지표는 표준인가</strong>", "계산식(Precision, Recall, F1, IoU 기준, ADD(-S))은 표준입니다. 무엇을 맞음으로 치는가(관측된 점의 겹침 0.25, 배정 거리 2 cm, 이름 일치)와 안정성 지표는 이 실험에서 정한 것입니다. 3장에 구분해 두었습니다."]],
        widths=["24%", None])

    # ---------- 2 실험 조건 ----------
    d.h2("2. 실험 조건"); d.h3("비교한 세 방법")
    d.p("세 방법에 같은 컬러·깊이 영상과 같은 카메라 자세를 주고, 실행이 끝났을 때의 객체 지도를 정답과 비교했습니다.")
    d.table(["항목", "제안 방법", "FM-Fusion (RA-L 2024)", "DualMap (RA-L 2025)"], [
        ["<strong>물체에 대해 미리 주는 것</strong>", "이름, CAD 모델", "이름", "이름"], ["<strong>문구 검출기</strong>", "YOLO-World", "GroundingDINO", "YOLO-World"],
        ["<strong>물체를 찾는 방법</strong>", "문구 검출기로 후보를 찾고 CAD 템플릿과 비교해 확인", "문구 검출기와 분할 모델(SAM)", "문구 검출기와 분할 모델(FastSAM 등)"],
        ["<strong>이름을 정하는 근거</strong>", "CAD 템플릿과의 외형 비교", "프레임마다 검출기가 낸 이름을 누적", "검출기 이름의 다수결(공식 평가는 CLIP 특징과 문구의 유사도)"],
        ["<strong>지도에 저장하는 것</strong>", "물체의 6D 자세(위치와 방향)", "물체의 점군", "물체의 점군"], ["<strong>물체의 방향</strong>", "저장함", "없음", "없음"],
        ["<strong>카메라 자세</strong>", "외부에서 받음", "외부에서 받음", "외부에서 받음"]], widths=["20%", None, None, None])
    d.h3("데이터")
    d.table(["데이터", "장면(방문)", "전체 프레임", "정답 물체", "카메라 자세", "쓴 목적"], [
        ["YCB-Video", "12장면", "20,738", "55", "ORB-SLAM3 추정값", "등록 정확도, 6D 자세, 지도 생성 중의 안정성, 프레임 단위 인식"],
        ["YCB-M", "32장면", "7,252", "165", "ORB-SLAM3 추정값", "등록 정확도, 6D 자세"],
        ["ObChange Kitchen", "방문 6회", "2,994", "25", "데이터셋 제공값", "물체 배치가 바뀌는 공간"], ["ObChange Office", "방문 6회", "4,184", "10", "데이터셋 제공값", "같음"],
        ["ObChange LivingArea", "방문 6회", "6,026", "14", "데이터셋 제공값", "같음"], ["ObChange SmallRoom", "방문 6회", "16,648", "25", "데이터셋 제공값", "같음"]])
    d.p("ObChange는 방마다 첫 방문이 물체가 없는 기준 방문입니다. 정답 물체 수는 물체가 놓인 다섯 방문에서, CAD 템플릿이 있는 11종만 센 값입니다. BigRoom은 FM-Fusion 실행을 중간에 멈춰 포함하지 않았습니다.", "note")
    d.h3("그 밖의 조건")
    d.dl([("제안 방법", "영상을 실제 속도로 흘려보내며 처리가 끝나는 대로 다음 프레임을 받음. 처리한 프레임은 YCB-Video 1,828장, YCB-M 3,736장, ObChange 네 방 23,048장"),
          ("FM-Fusion", "문구 목록을 GroundingDINO에 직접 주고 SAM으로 마스크를 만든 뒤 통합. 복셀 5 mm(원래 2 cm는 탁상 물체가 걸러져 바꿈). YCB-Video는 10장마다, YCB-M은 2장마다, ObChange는 3장마다 1장. 논문의 태그 모델 단계는 쓰지 않음"),
          ("DualMap", "색 형용사를 뺀 일반 이름 문구, 검출 기준값 0.05. YCB-Video는 10장마다, YCB-M과 ObChange는 2장마다 1장. 충돌을 막는 수정 세 군데가 들어 있음. 9장에 다른 설정의 결과가 있음"),
          ("문구", "제안 방법과 FM-Fusion은 같은 문구(예: blue coffee can, red mug). DualMap은 coffee can, mug처럼 색을 뺀 이름"),
          ("카메라 자세", "YCB 두 데이터는 ORB-SLAM3(RGB-D) 추정 궤적을 세 방법에 똑같이 줌. 채점할 때 정답 카메라 궤적에 한 번 정렬. ObChange는 녹화 파일에 든 로봇의 위치 추정값을 그대로 씀"),
          ("ObChange의 방향", "정답 방향을 믿기 어려워 방향 오차와 ADD(-S)는 YCB 두 데이터에서만 잼")])

    # ---------- 3 채점 ----------
    d.h2("3. 채점 방식")
    d.p("점군 지도와 자세 지도를 같은 조건으로 비교하려고, 카메라가 실제로 본 표면의 점만 평가했습니다. 점군은 본 면만 복원하고 CAD는 물체 전체를 놓기 때문에, 물체 전체를 기준으로 삼으면 CAD를 쓰는 쪽이 저절로 유리해집니다.")
    d.dl([("관측 점", "YCB는 정답 카메라 자세로 깊이 영상을 쌓은 점(3 mm 간격으로 솎음). ObChange는 데이터셋이 주는 재구성 점군"),
          ("정답 라벨", "YCB는 정답 자세의 CAD 표면에서 2 cm 안인 점. ObChange는 데이터셋의 점 라벨"),
          ("지도 객체에 배정", "관측 점을 2 cm 안에서 가장 가까운 지도 객체에 배정. 제안 방법은 저장된 자세에 놓은 CAD 점, 나머지는 복원 점"),
          ("맞는 자리", "지도 객체에 배정된 점 집합과 정답 물체의 점 집합의 겹침 비율(IoU)이 0.25 이상"),
          ("TP", "정답 물체를 맞는 이름으로 맞는 자리에 올린 수"), ("FP", "대상 물체 이름이 붙어 지도에 올라갔지만 정답과 맞지 않는 객체 수(없는 물체, 틀린 이름, 중복)"), ("FN", "맞게 올리지 못한 정답 물체 수"),
          ("Precision, Recall, F1", "TP / (TP + FP), TP / (TP + FN), 둘의 조화 평균"), ("Class-agnostic recall", "이름과 상관없이 정답 자리에 객체가 올라가 있는 비율"),
          ("95 % 구간", "장면(방문)을 복원 추출로 2,000번 다시 뽑아 같은 지표를 계산한 값의 2.5~97.5 % 범위")])
    d.h3("표준 지표와 이 실험에서 정한 것")
    d.table(["지표", "다른 논문에서의 쓰임", "이 실험에서 달라진 점"], [
        ["Precision, Recall, F1", "널리 쓰는 계산식. 객체 지도에서는 Clio, Hydra 등이 객체 단위로 사용", "맞음의 기준(겹침 0.25, 배정 거리 2 cm, 이름 일치)을 직접 정함"],
        ["IoU 0.25 / 0.5", "3D 인스턴스 분할의 표준 기준(AP25, AP50). FM-Fusion이 사용", "신뢰도 순 정렬 없이 Precision·Recall만 냄. 관측된 점만 씀"],
        ["ADD(-S), ADD(-S) &lt; 0.1d", "6D 자세 추정의 표준", "프레임마다가 아니라 최종 지도의 TP 객체에 대해 잼"], ["Rotation error", "표준", "없음"],
        ["지도 생성 중의 안정성", "표준 아님", "직접 만듦"], ["방문별 ○ △ × – ● 표, TN", "표준 아님", "직접 만듦"]], widths=["22%", None, None])

    # ---------- 4 등록 정확도 ----------
    d.h2("4. 객체 등록 정확도")
    d.p("막대는 Precision(녹색)과 Recall(청색)입니다. 왼쪽에 색 띠가 있는 줄이 제안 방법입니다. F1의 괄호는 95 % 구간입니다.", "note")
    rows = []
    for ds, rws in DS:
        for m in M3:
            x = ST[ds][m]; tpi = [r["iou_name"] for r in rws if r["method"] == m and r["object"] and r["iou_name"] >= SF.THR]
            rows.append([f"{ds} ({x['n']})", m, x["ok"], x["wrong"], x["n"] - x["ok"], float(x["P"]), float(x["R"]), f"{x['F']:.1f} ({f1ci(rws, m)})", float(100 * x["present"] / x["n"]), x["same"] + x["diff"],
                         f"{np.mean(tpi):.2f}" if tpi else "–", f1(x["dist"]) if x["dist"] is not None else "–"])
    d.table(["데이터 (정답 수)", "방법", "TP", "FP", "FN", "Precision %", "Recall %", "F1 % (95 % 구간)", "Class-agnostic recall %", "다른 이름으로 올림", "Mean IoU (TP)", "표면 거리 mm"], rows, cls="grid7", bars=(5, 6),
            group=lambda i, r: "g2" if r[1] == "제안 방법" else "")
    d.ul([f"<strong>FP와 F1.</strong> 제안 방법의 FP가 가장 적어 F1이 세 데이터 모두에서 가장 높습니다. YCB-M과 ObChange에서는 95 % 구간이 비교 방법과 겹치지 않습니다. YCB-Video에서는 제안 방법({f1ci(Y, '제안 방법')})과 FM-Fusion({f1ci(Y, 'FM-Fusion')})의 구간이 겹칩니다.",
          "<strong>Recall.</strong> YCB-M과 ObChange에서는 FM-Fusion이 더 높습니다. 제안 방법이 물체를 가장 잘 찾는 것은 아닙니다.",
          "<strong>Class-agnostic recall.</strong> 비교 방법은 정답 자리에 객체를 거의 다 올립니다. 틀리는 것은 이름과 객체 수입니다.",
          "<strong>표면 거리.</strong> 관측된 정답 표면에서 TP 객체까지 거리의 중앙값입니다. 세 방법의 차이가 작습니다."])
    d.h3("ObChange 방별")
    rows = []
    for r, name in rooms:
        s_ = stats_of(RM[r])
        for m in M3: x = s_[m]; rows.append([f"{name} ({x['n']})", m, x["ok"], x["wrong"], x["n"] - x["ok"], float(x["P"]), float(x["R"]), float(x["F"])])
    d.table(["방 (정답 수)", "방법", "TP", "FP", "FN", "Precision %", "Recall %", "F1 %"], rows, cls="grid7", bars=(5, 6), group=lambda i, r: "g2" if r[1] == "제안 방법" else "")
    d.p("F1은 네 방 모두 제안 방법이 가장 높습니다. TP는 Kitchen에서만 제안 방법이 가장 많고 나머지 세 방에서는 FM-Fusion이 같거나 더 많습니다. 방마다 정답 물체가 10~25개라 한두 개 차이로 비율이 크게 움직입니다.", "note")
    d.h3("결과 예")
    d.fig(MED / "ycbv/one/overlay_000054.png", "YCB-Video 장면 54. 실제 영상에 지도 객체를 겹친 모습. 왼쪽부터 정답, 제안 방법, FM-Fusion, DualMap. 초록은 맞는 객체, 빨강은 중복이거나 정답과 먼 자리의 객체")
    d.fig(MED / "ycbv/map4c_000054_view.png", "같은 장면의 지도를 위에서 본 모습. 점선은 정답 윤곽")

    # ---------- 5 바뀌는 공간 ----------
    d.h2("5. 물체 배치가 바뀌는 공간 (ObChange)")
    d.p("같은 방을 여섯 번 방문하고 방문마다 물체 배치가 다릅니다. 방문마다 지도를 새로 만들어 그 방문의 정답 배치와 비교했습니다. 한 지도를 방문 사이에 이어서 갱신한 실험이 아닙니다.")
    put = {m: [0, 0, 0] for m in M3}; ab = {m: [0, 0] for m in M3}; nput = nab = 0; marks = {}
    for r, _ in rooms:
        _, here, mark = room_data(r); marks[r] = (here, mark)
        for v in here:
            for o in sorted(CAT):
                inn = o in here[v]; nput += inn; nab += not inn
                for m in M3:
                    k = mark(m, v, o)
                    if inn: put[m]["○△×".index(k)] += 1
                    else: ab[m]["–●".index(k)] += 1
    d.table(["항목 (네 방 합계)", "대상 칸 수"] + M3, [["놓인 물체를 맞는 자리에서 맞는 이름으로 올림", nput] + [put[m][0] for m in M3], ["놓인 물체를 맞게 올리지 못함 (지도에 없거나 다른 자리)", nput] + [put[m][1] + put[m][2] for m in M3],
                                                   ["놓이지 않은 물체를 지도에 올림", nab] + [ab[m][1] for m in M3]])
    d.p("한 칸은 '대상 물체 종류 × 방문'입니다. 대상 물체 11종, 방문 24회(물체가 없는 첫 방문 포함)입니다.", "note")
    d.h3("Kitchen 방문별")
    here, mark = marks["kitchen"]
    d.table(["물체"] + [f"{v}번째 방문" for v in here], [[("<strong>" + KOR[o] + "</strong>")] + [(" ".join(mark(m, v, o) for m in M3)) + (" *" if o in here[v] else "") for v in here] for o in sorted(CAT)])
    d.p("칸 안의 순서는 제안 방법, FM-Fusion, DualMap입니다. * 표시는 그 방문에 물체가 실제로 놓여 있던 칸입니다. 놓여 있던 칸: ○ 맞는 자리에서 인식, △ 그 이름의 객체가 지도에 있지만 다른 자리, × 지도에 없음. 놓여 있지 않던 칸: – 지도에도 없음, ● 없는 물체인데 지도에 올라감.", "note")
    d.fig(MED / "kitchen/layout_kitchen.png", "ObChange Kitchen의 다섯 방문. 위는 위에서 본 정답 배치, 아래는 그 방문의 카메라 영상")
    d.fig(MED / "kitchen/overlay_kitchen_s6.png", "Kitchen 6번째 방문. 왼쪽부터 정답, 제안 방법, FM-Fusion, DualMap. 초록은 맞는 객체, 주황은 정답 자리에 다른 이름, 빨강은 정답이 없는 자리에 올라간 객체")

    # ---------- 6 6D 자세 ----------
    d.h2("6. 지도에 저장된 6D 자세")
    d.h3("제안 방법의 저장 자세")
    rows = []
    for ds, f in (("YCB-Video", V / "pose_final.json"), ("YCB-M", M / "pose_final.json")):
        s_ = J(f)["summary"]; rows.append([ds, s_["n_tp"], f1(s_["add_s_mm_p50"]), f1(s_["add_s_pass_of_tp"]), f1(s_["add_s_pass_of_gt"]), f1(s_["rot_deg_p50"]), f1(s_["center_mm_p50"])])
    d.table(["데이터", "대상 (TP)", "ADD(-S) 중앙값 mm", "ADD(-S) &lt; 0.1d, TP 대비 %", "정답 전체 대비 %", "방향 오차 중앙값 °", "중심 오차 중앙값 mm"], rows)
    d.p("대칭 물체(그릇, 클램프 2종, 스펀지 벽돌, 마커, 커피 캔, 나무 블록)는 ADD-S를, 나머지는 ADD를 썼습니다. 놓친 물체는 TP 대비 값에 들어가지 않으므로, 6D 자세 논문의 값과 견줄 때는 '정답 전체 대비' 열을 봐야 합니다.", "note")
    d.h3("비교 방법의 점군에 CAD를 맞춰 얻은 자세와의 비교")
    d.p("FM-Fusion과 DualMap은 자세를 내지 않습니다. 그래서 두 방법이 맞게 올린 객체(TP)의 점군에 CAD 모델을 맞춰 자세를 구했습니다. 시작 방향 64개에서 점-점 ICP를 돌려, 객체 점에서 모델까지의 거리가 가장 작은 결과를 골랐습니다. 정답은 쓰지 않았습니다.")
    rows = []
    for ds in ("YCB-Video", "YCB-M"):
        n_gt = FIT[ds]["n_gt"]
        for m in M3:
            Rm = [r for r in FIT[ds]["rows"] if r["method"] == m]
            rows.append([ds, m, Rm[0]["source"], len(Rm), f"{np.median([r['rot_sym_deg'] for r in Rm]):.1f}", float(far(ds, m)), f"{np.median([r['add_s_mm'] for r in Rm]):.1f}", float(100 * np.mean([r["pass"] for r in Rm])),
                         f"{100 * sum(r['pass'] for r in Rm) / n_gt:.1f}", f"{100 * np.mean([r['oracle_pass'] for r in Rm]):.1f}" if "oracle_pass" in Rm[0] else "–"])
    d.table(["데이터", "방법", "자세를 얻은 방식", "대상 (TP)", "방향 오차 중앙값 °", "30° 넘게 틀린 비율 %", "ADD(-S) 중앙값 mm", "ADD(-S) &lt; 0.1d %", "정답 전체 대비 %", "정답을 알고 골랐을 때 %"], rows, cls="grid7", bars=(5, 7),
            group=lambda i, r: "g2" if r[1] == "제안 방법" else "")
    d.ul(["점군에 맞춘 자세는 위치는 맞아도 방향이 뒤집히는 경우가 많습니다. 상자는 뒤집어도, 캔은 돌려도 모양이 같기 때문입니다.",
          "마지막 열은 64개 결과 가운데 정답에 가장 가까운 것을 골랐을 때의 통과율입니다. 맞는 자세가 후보에는 있지만 점군만으로는 그것을 고르지 못한다는 뜻입니다. YCB-M에서는 이 값이 FM-Fusion 쪽에서 제안 방법과 비슷합니다.",
          "방향 오차는 물체의 대칭을 고려한 값입니다."])
    d.fig(MED / "fit/rot_cdf.png", "방향 오차가 그 값 이하인 객체의 비율")
    d.fig(MED / "fit/fit_000054.png", "YCB-Video 장면 54. 각 자세에 CAD를 놓아 실제 영상에 겹친 모습. 검은 선은 정답, 초록은 ADD(-S)가 지름의 10 % 미만, 빨강은 그 이상. 숫자는 방향 오차")

    # ---------- 7 안정성 ----------
    d.h2("7. 지도가 만들어지는 동안의 안정성"); d.h3("제안 방법: 저장된 자세가 갱신될 때마다 움직이는 양")
    rows = []
    for ds, k in (("YCB-Video", "ycbv"), ("YCB-M", "ycbm"), ("ObChange Kitchen", "kitchen")):
        Jt = J(O / "jitter_261006" / k / "jitter.json"); q = lambda kk: float(np.median([r[kk] for r in Jt]))
        rows.append([ds, len(Jt), f"{q('raw_scatter_mm_p50'):.1f} / {q('raw_scatter_mm_p95'):.1f}", f"{q('step_mm_p50'):.2f} / {q('step_mm_p95'):.2f}", f"{q('dev_from_final_mm_max_after10'):.1f}",
                     f"{q('rot_step_deg_p50'):.2f} / {q('rot_step_deg_p95'):.2f}", f"{sum(r['rot_jumps_over5deg'] for r in Jt)}회 ({sum(r['rot_jumps_over5deg'] > 0 for r in Jt)}개 물체)"])
    d.table(["데이터", "객체 수", "프레임별 추정값의 흩어짐 mm (중앙 / 95 %)", "저장 위치의 갱신당 이동 mm", "10회 갱신 뒤 최대 이탈 mm", "저장 방향의 갱신당 변화 °", "5° 넘는 방향 변화"], rows)
    d.p("인식 결과가 한 장 들어올 때마다 지도에 저장된 객체의 위치와 방향을 기록했습니다. 값은 객체별 값의 중앙값입니다. 프레임마다의 추정값은 3~5 mm씩 흩어지지만 저장된 위치는 한 번 갱신될 때 0.1~0.4 mm 움직입니다. 저장된 위치도 영상 전체로는 3~5 mm 범위에서 천천히 이동합니다.", "note")
    d.fig(O / "jitter_261006/ycbv/jitter_000054.png", "YCB-Video 장면 54의 네 물체. 회색 점은 그 프레임의 추정값, 파란 선은 지도에 저장된 값. 세로축은 마지막 저장 위치와의 거리")
    d.h3("세 방법 비교 (YCB-Video)")
    d.p("FM-Fusion과 DualMap은 실행이 끝났을 때의 지도만 파일로 남깁니다. 그래서 영상의 앞 10 %만 넣고 실행해 지도를 얻고, 처음부터 다시 앞 20 %만 넣고 실행하는 식으로 시점마다 따로 돌렸습니다. 제안 방법은 한 번의 실행에서 같은 시점의 지도 상태를 꺼냈습니다.")
    d.table(["항목"] + M3, [["시점 사이에 객체 중심이 움직인 거리, 중앙값 / 95 % (mm)"] + [f"{SB[m]['p50']:.1f} / {SB[m]['p95']:.1f}" for m in M3], ["맞는 상태와 아닌 상태 사이를 오간 횟수"] + [SB[m]["flips"] for m in M3],
                              ["한 번 맞게 올라갔다가 마지막 지도에서 틀어진 물체"] + [f"{SB[m]['lost']} / {SB[m]['ever']}" for m in M3]])
    d.p("객체 중심은 제안 방법은 저장된 자세의 CAD 중심, 나머지는 점의 평균입니다. 점군 지도는 점이 쌓이면서 중심이 옮겨 가는 것이 정상 동작이라, 이 지표는 점군 방법에 불리합니다.", "note")
    d.fig(MED / "jitter/ckpt.png", "왼쪽: 맞는 이름으로 지도에 있는 정답 물체의 비율. 오른쪽: 객체 중심과 정답 중심의 거리")

    # ---------- 8 프레임 단위 ----------
    d.h2("8. 프레임 단위 인식과 지도 등록 (YCB-Video)")
    FS = J(V / "frame_states.json"); rows = []; ST5 = ["맞음", "틀린 자세", "다른 이름", "이름 없음", "없음"]
    for m in M3:
        Rm = [r for r in FS if r["method"] == m]; F = sum(r["frames"] for r in Rm); seen = [r for r in Rm if r["맞음"] > 0]
        rows.append([m] + [f"{100 * sum(r[k] for r in Rm) / F:.1f}" for k in ST5] + [f"{100 * sum(r['drops'] for r in Rm) / max(sum(r['pairs_from_ok'] for r in Rm), 1):.1f}", len(seen), sum(r["in_map"] for r in seen), sum(not r["in_map"] for r in seen)])
    d.table(["방법", "맞게 인식한 프레임 %", "자세가 틀림 %", "다른 이름 %", "이름을 못 붙임 %", "결과 없음 %", "맞은 프레임의 다음 프레임에서 맞지 않은 비율 %", "한 번이라도 맞게 인식한 물체", "그중 지도에 맞게 올라감", "올라가지 않음"], rows)
    d.ul(["화면에 보이는 정답 물체마다, 각 방법이 처리한 프레임 하나하나의 인식 결과를 정답과 대조했습니다. 제안 방법은 위치 오차 30 mm 미만, 나머지는 정답 상자와 절반 이상 겹치고 이름이 같으면 맞음입니다.",
          "DualMap은 검출기가 같은 물체에 이름이 다른 상자를 여러 개 냅니다. 프레임 대부분에서 맞는 이름이 나온 물체도 지도에서는 다른 이름이 된 경우가 있습니다(장면 48의 커피 캔: 225장 중 223장에서 맞는 이름).",
          "DualMap의 프레임별 기록은 같은 설정으로 다시 돌려 얻었습니다. 처리한 프레임 수는 방법마다 다릅니다."])

    # ---------- 9 DualMap ----------
    d.h2("9. DualMap 점검"); d.h3("설정별 결과 (YCB-Video, 정답 55개)")
    rows = [[a, b, c, e, x["ok"], x["wrong"], x["n"] - x["ok"], float(x["P"]), float(x["R"]), float(x["F"]), f1(100 * x["present"] / x["n"])] for a, b, c, e, x in SW]
    for m in ("FM-Fusion", "제안 방법"): x = yv[m]; rows.append([f"(참고) {m}", "", "", "", x["ok"], x["wrong"], x["n"] - x["ok"], float(x["P"]), float(x["R"]), float(x["F"]), f1(100 * x["present"] / x["n"])])
    d.table(["물체 이름 문구", "검출 기준값", "점군 해상도", "이름을 정하는 방식", "TP", "FP", "FN", "Precision %", "Recall %", "F1 %", "Class-agnostic recall %"], rows, cls="grid7", bars=(7, 8), group=lambda i, r: "g2" if i == 3 else ("g0" if i >= len(SW) else ""))
    ck = lambda p: SF.stats([r for r in J(p) if r["method"] == "DualMap"])["F"]
    d.ul([f"왼쪽에 색 띠가 있는 줄이 이 보고서의 다른 표에 쓴 설정입니다. 여덟 가지 설정에서 F1은 {min(swF):.0f}~{max(swF):.0f} %입니다.",
          f"DualMap의 공식 평가 코드는 객체에 쌓인 CLIP 특징으로 이름을 정합니다. 이 방식으로 바꾸면 F1이 오릅니다. YCB-Video {f1(SW[3][4]['F'])} → {f1(SW[4][4]['F'])} %, YCB-M {f1(ym['DualMap']['F'])} → {f1(ck(PL / 'ycbm/inst_clip_named.json'))} %. ObChange Kitchen은 {f1(stats_of(RM['kitchen'])['DualMap']['F'])} → {f1(ck(OBE / 'kitchen_clip_named.json'))} %로 조금 내려갑니다. 다른 표의 DualMap 값은 검출기 이름의 다수결을 쓴 것이라 DualMap에 조금 불리합니다.",
          "기본 설정은 방 규모에 맞춰져 있습니다. 점군을 2 cm 간격으로 줄이고 4 %만 남기므로 10~20 cm 물체는 수십~수백 점이 됩니다."])
    d.h3("수치가 낮은 원인")
    d.ul(["한 물체를 여러 조각으로 올립니다. 정답 물체 하나에 겹쳐 올라간 지도 객체가 평균 2.6개입니다(FM-Fusion 1.5개). 조각마다 이름이 따로 붙습니다.", "검출기가 같은 물체에 다른 이름의 상자를 여러 개 냅니다(프레임의 46 %).",
          "논문은 의자·책상 같은 범주를 구분하는 과제로 평가했습니다. 닮은 상품을 서로 구분하는 것은 그 평가에 없는 조건입니다."])
    d.h3("논문 수치 재현 (Replica)")
    d.table(["장면", "mIoU", "F-mIoU", "mAcc"], [[k] + [f"{x:.3f}" for x in v] for k, v in sorted(rep.items())] + [[f"<strong>이번 실행 평균 ({len(rep)}장면)</strong>"] + [f"{x:.3f}" for x in rmean], ["<strong>논문 표 II (8장면 평균)</strong>", "0.254", "0.521", "0.402"]])
    d.p("내려받은 DualMap을 고치지 않은 기본 설정으로 돌리고 DualMap이 제공하는 채점 코드로 채점했습니다. Replica의 영상과 궤적은 원 배포 서버가 응답하지 않아 같은 자료의 다른 배포본에서 받았습니다.", "note")

    # ---------- 10 놓치는 물체 ----------
    d.h2("10. 제안 방법이 놓치는 물체")
    cnt = collections.defaultdict(lambda: [0, 0])
    for rws in (Y, Ym, RM["kitchen"]):
        for r in rws:
            if r["object"] and r["method"] == "제안 방법": cnt[key(r["object"])][0] += 1; cnt[key(r["object"])][1] += r["iou_name"] < SF.THR
    tot = sum(v[0] for v in cnt.values()); mis = sum(v[1] for v in cnt.values()); top = sorted(cnt.items(), key=lambda x: (-x[1][1] / x[1][0], -x[1][1]))[:6]
    stg = J(REPO / "objpose/docs/figs_ycbv_sam6d_gt/stages.json")["objects"]
    rows = []
    for k, (n, mcount) in top:
        s_ = stg.get("ycbv_" + k); rows.append([EN.get(k, k), n, mcount, float(100 * mcount / n)] + ([f"{100 * s_['s1'] / s_['gt']:.0f}", f"{100 * s_['s2'] / s_['gt']:.0f}", f"{100 * s_['s3'] / s_['gt']:.0f}", f"{100 * s_['s4'] / s_['gt']:.0f}"] if s_ else ["–"] * 4))
    rest = [(k, v) for k, v in cnt.items() if k not in dict(top)]
    rows.append([f"나머지 {len(rest)}종", sum(v[0] for _, v in rest), sum(v[1] for _, v in rest), float(100 * sum(v[1] for _, v in rest) / sum(v[0] for _, v in rest)), "", "", "", ""])
    d.table(["물체", "정답 수", "놓친 수", "놓친 비율 %", "검출기가 상자를 냄 %", "자기 이름으로 냄 %", "템플릿 검증 통과 %", "자세까지 맞음 %"], rows, bars=(3,))
    d.ul([f"YCB-Video, YCB-M, ObChange Kitchen의 정답 {tot}개 가운데 {mis}개를 이름까지 맞게 올리지 못했고, 그중 {sum(v[1] for _, v in top)}개가 위 여섯 종류입니다.",
          "오른쪽 네 열은 9월 22일에 YCB-Video에서 프레임 단위로 잰 단계별 통과율입니다(문구를 조정하기 전의 설정).",
          "후보를 문구 검출기가 낸 상자에서만 만들기 때문에, 검출기가 잡지 못하거나 다른 물체 문구로 잡은 물체는 뒤 단계에서 되살릴 수 없습니다.",
          "무늬가 없고 대칭인 그릇은 검출은 되지만 템플릿 비교와 자세 수렴 조건을 통과하지 못합니다."])

    # ---------- 11 해석, 12 한계, 13 결과 파일 ----------
    d.h2("11. 해석"); d.h3("주장할 수 있는 것")
    d.ul(["학습 없는 6D 자세 추정과 SLAM을 결합해, 물체의 위치와 방향을 담은 객체 지도를 만듭니다.", "지도에 올라간 객체를 믿을 수 있습니다. 잘못 등록되는 객체가 비교한 두 방법보다 훨씬 적습니다.",
          "점군 지도에 CAD를 사후에 맞추는 방식으로는 같은 6D 자세를 얻기 어렵습니다. 방향의 3분의 1에서 3분의 2가 뒤집힙니다.", "지도가 만들어지는 동안 객체의 위치와 방향이 안정적으로 유지됩니다."])
    d.h3("주장할 수 없는 것")
    d.ul(["물체를 더 많이 찾는다는 것. YCB-M과 ObChange에서는 FM-Fusion이 정답 물체를 더 많이 올립니다.", "물체 배치가 바뀌었을 때 지도를 갱신한다는 것. 방문마다 지도를 새로 만들어 평가했습니다. 두 방문을 이어 넣어 본 초기 실험에서는 따로 만든 지도보다 결과가 나빴습니다.",
          "기하 정확도에서 크게 앞선다는 것. 관측된 표면 기준 거리는 세 방법이 비슷합니다."])
    d.h2("12. 한계")
    d.ul(["제안 방법만 CAD 모델을 씁니다. 이름 구분에서 앞서는 것은 이 정보 덕분이고, CAD를 쓰는 다른 방법과의 비교는 하지 않았습니다.",
          "비교 방법의 설정(DualMap의 문구와 검출 기준값)을 평가에 쓴 ObChange Kitchen의 한 방문을 보고 정했습니다.", "이름이 정확히 같아야 맞음으로 쳤습니다. 범주 수준으로 채점하면 비교 방법의 점수가 오릅니다.",
          "배정 거리 2 cm와 겹침 기준 0.25는 이 실험에서 정한 값입니다. 배정 거리를 1 cm로 줄이면 겹침 비율이 전체적으로 내려가고 DualMap이 가장 크게 영향을 받습니다.",
          "FM-Fusion은 논문의 태그 모델 단계 없이 물체 이름 목록을 검출기에 직접 주었습니다.", "세 방법이 처리한 프레임 수가 다릅니다. 실행을 반복했을 때의 차이는 재지 않았습니다.",
          "ObChange의 카메라 자세는 로봇이 스스로 추정한 값이라 독립된 정답이 아닙니다. 방마다 정답 물체가 10~25개로 적습니다.", "안정성 비교는 YCB-Video에서만 했습니다."])
    d.h2("13. 결과 파일")
    d.ul(["채점: <code>objpose/tools/analysis/inst_iou.py</code>(YCB), <code>objpose/tools/obchange/inst_iou_obchange.py</code>(ObChange) → <code>inst_iou_final.json</code>, <code>&lt;방&gt;_final.json</code>",
          "6D 자세: <code>pose_metrics.py</code> → <code>pose_final.json</code>, <code>fit_pose_from_points.py</code> → <code>fit_pose.json</code>", "안정성: <code>map_jitter.py</code> → <code>objpose/output/jitter_261006/</code>, <code>run_checkpoints.sh</code>, <code>ckpt_stability.py</code> → <code>objpose/output/checkpoints_261006/stability.json</code>",
          "프레임 단위: <code>frame_states.py</code> → <code>frame_states.json</code>", "DualMap: <code>objpose/tools/baselines/dualmap/run_dualmap_bop.py</code>, <code>dualmap_relabel.py</code> → <code>objpose/output/dualmap_plain_261005/</code>",
          "YCB 결과 폴더 <code>objpose/output/fmf_cmp_260928/</code>, <code>fmf_cmp_ycbm_260929/</code>, ObChange <code>objpose/output/obchange_261005/</code>", "발표자료 <code>~/콜로키움/전체_실험결과.pptx</code>, <code>발표자료_261006.pptx</code>",
          "이 보고서: <code>objpose/docs/objmap_eval_261009/build_report.py</code>"])

    # ---------- 쓰기 ----------
    ref = REF.read_text(); head = ref[: ref.index("<div class=\"wrap\">")]
    head = head.replace("<title>YCB 조합 실험 보고</title>", "<title>객체 지도 비교 실험</title>").replace("</style>", "figure{margin:6px 0 4px}figure img{max-width:100%;height:auto;border:1px solid var(--line);border-radius:4px;display:block;background:#fff}figcaption{margin-top:4px;max-width:90ch}</style>")
    hdr = ('<header><div class="kicker">실험 보고 · 객체 지도 비교 · 2026-10-09</div><h1>객체 지도 비교 실험: 제안 방법 · FM-Fusion · DualMap</h1>'
           '<p class="lead">SLAM의 카메라 자세와 학습 없는 6D 자세 추정을 결합해 만든 객체 지도를, 최근의 객체 지도 방법 두 가지와 같은 영상·같은 카메라 자세에서 비교했습니다. '
           '데이터는 YCB-Video 12장면(정답 물체 55개), YCB-M 32장면(165개), ObChange 네 방의 방문 20회(74개)입니다. 지도에 올라간 객체가 맞는지, 저장된 6D 자세가 정확한지, 지도가 만들어지는 동안 안정적인지, 물체 배치가 바뀌는 공간에서도 유지되는지를 봤습니다.</p></header>')
    body = hdr + "\n" + "\n".join(a for a, _ in d.parts)
    (HERE / "objmap_eval_report.html").write_text(head + '<div class="wrap">\n' + body + "\n</div>\n")
    (HERE / "objmap_eval_report_local.html").write_text('<!doctype html>\n<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"></head><body style="margin:0">\n' + head + '<div class="wrap">\n' + body + "\n</div>\n</body></html>\n")
    nref = (REF.parent / "ycb_combo_report_notion.html").read_text(); nhead = nref[: nref.index("<body>") + 6].replace("YCB 조합 실험 보고", "객체 지도 비교 실험")
    nb = ("<p>실험 보고 · 객체 지도 비교 · 2026-10-09</p>" + re.sub(r'<header>.*?</div>', "", hdr, flags=re.S).replace("</header>", "").replace(' class="lead"', "")) + "\n" + "\n".join(b for _, b in d.parts)
    (HERE / "objmap_eval_report_notion.html").write_text(nhead + "\n\n" + nb + "\n</body>\n</html>\n")
    for f in ("objmap_eval_report.html", "objmap_eval_report_local.html", "objmap_eval_report_notion.html"): print(f, (HERE / f).stat().st_size)


if __name__ == "__main__":
    main()
