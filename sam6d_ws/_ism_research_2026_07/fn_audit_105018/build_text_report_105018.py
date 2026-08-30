#!/usr/bin/env python3
"""build_text_report_105018.py — terminal/editor-readable Markdown FN audit.

Reads the SAME frames.jsonl / summary.json the HTML uses. No re-inference, no
operational-code changes. Emits sam_105018_fn_report.md: summary tables + one
block per GT-labelled frame with every object's gate scores and FN cause, so the
whole audit is legible in a text editor without a browser.
"""
import json
import os

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(REPO, "outputs", "sam_105018_fn_audit")
recs = sorted((json.loads(l) for l in open(os.path.join(OUT, "frames.jsonl"), encoding="utf-8")),
              key=lambda r: r["frame_index"])
S = json.load(open(os.path.join(OUT, "summary.json"), encoding="utf-8"))
M = json.load(open(os.path.join(OUT, "run_meta.json"), encoding="utf-8"))
CAUSES = ["NO_YOLO_BBOX", "SEMANTIC_REJECT", "APPEARANCE_REJECT", "HSV_REJECT",
          "SELECTION_OR_OUTPUT_MISS", "UNKNOWN"]
t0 = recs[0]["timestamp_ns"]

L = []
def p(s=""): L.append(s)


def flag(v, thr, passed):
    if v is None:
        return "—"
    mark = "P" if passed == 1 else ("F" if passed == 0 else "?")
    return f"{v:.3f}/{thr}{mark}"


def hsv_pass_of(o):
    if o["hsv"] is None:
        return None
    if o["hsv_pass"] is not None:
        return o["hsv_pass"]
    return int(o["hsv"] >= o["hsv_thr"])


t = S["totals"]; fc = S["fn_by_cause"]
p("# sam_105018 — 전 프레임 FN 원인 감사 (Phase 1C 운영 기준)")
p()
p(f"- YOLO `{M.get('weights','yolov8m-worldv2.pt')}` imgsz {M['imgsz']} conf {M['op_conf']} "
  f"(raw 진단 pass {M['raw_conf']}), 프롬프트별 개별 pass → recognize → cross-object NMS(rank=appe)")
p(f"- semantic thr 0.35 · appe gate 0.55 · HSV gate ON thr {M['thresholds']['milk']['hsv_gate_threshold']} "
  f"· Dinosaur hue +{M['thresholds']['Dinosaur']['hsv_hue_correction_ocv']} ocv · training-free gate OFF")
p(f"- 원자료 `frames.jsonl` · 전체 CSV `sam_105018_all_frames.csv` · overlay `frames/frame_XXXXXX.jpg` (401장)")
p(f"- 이미지까지 보려면 서버: `python -m http.server 8017 --directory {OUT}` → "
  f"VS Code 'PORTS' 탭에서 8017 포워딩 → 브라우저 http://127.0.0.1:8017/sam_105018_all_frames.html")
p()
p("## 요약")
p()
p("| 지표 | 값 |")
p("|---|---|")
p(f"| 전체 프레임 수 | {t['frames']} |")
p(f"| GT-라벨 프레임 수 | {t['labelled']} |")
p(f"| GT-visible object-frame 수 | {t['visible_of']} |")
p(f"| 정상 검출 (TP) | {t['tp']} |")
p(f"| **FN** | **{t['fn']}** |")
p(f"| FP | {t['fp']} |")
for c in CAUSES:
    p(f"| &nbsp;&nbsp;{c} | {fc[c]} |")
p()
p("> 원인은 '선택된 후보가 죽은 지점'이다. 정답 위치 후보가 semantic에서 먼저 탈락해 엉뚱한 박스가 선택된 뒤 "
  "뒷단(HSV 등)에서 죽으면 라벨은 뒷단이지만 진짜 병목은 localization/selection이다. 아래 FN 표의 '후보 best sem' 열로 판별.")
p()
p("## 객체별")
p()
p("| object | visible | detected | fn | no_yolo | semantic | appearance | hsv | selection | fp |")
p("|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|")
for n in M["objects"]:
    o = S["per_object"].get(n, {})
    p(f"| {n} | {o.get('visible',0)} | {o.get('detected',0)} | {o.get('fn',0)} | "
      f"{o.get('NO_YOLO_BBOX',0)} | {o.get('SEMANTIC_REJECT',0)} | {o.get('APPEARANCE_REJECT',0)} | "
      f"{o.get('HSV_REJECT',0)} | {o.get('SELECTION_OR_OUTPUT_MISS',0)} | {o.get('fp',0)} |")
p()

# ---- FN 상세 (원인별) ----
p("## FN 상세 (GT-visible인데 미검출, 원인별)")
fn_rows = [(o["cause"], r["frame_index"], o)
           for r in recs if r["gt_labelled"]
           for o in r["objects"] if o["gt_visible"] and not o["detected"]]
for cause in CAUSES:
    items = [x for x in fn_rows if x[0] == cause]
    if not items:
        continue
    p()
    p(f"### {cause} — {len(items)}건")
    p()
    p("| frame | object | raw/op bbox | 선택 bbox | sem/thr | appe/gate | hsv/thr | 후보수 best-sem | 비고 |")
    p("|--:|---|--:|---|---|---|---|---|---|")
    for _, fi, o in items:
        sel = ",".join(map(str, o["sel_box"])) if o["sel_box"] else "—"
        sem = flag(o["sem"], o["sem_thr"], o["sem_pass"] if o["sel_box"] else (0 if o["yolo_sel_n"] else None))
        appe = flag(o["appe"] if o["sel_box"] else None, o["appe_thr"], o["appe_pass"])
        hv = flag(o["hsv"], o["hsv_thr"], hsv_pass_of(o))
        best = max((c["sem"] for c in o["cands"] if c["sem"] is not None), default=None)
        bests = f"{len(o['cands'])}개 / {best:.3f}" if best is not None else f"{len(o['cands'])}개"
        note = f"NMS억제←{o['suppressed_by']}" if o["suppressed_by"] else ""
        p(f"| {fi} | {o['object']} | {o['yolo_raw_n']}/{o['yolo_op_n']} | {sel} | {sem} | {appe} | {hv} | {bests} | {note} |")
p()

# ---- FP 상세 ----
fp_rows = [(r["frame_index"], o) for r in recs if r["gt_labelled"]
           for o in r["objects"] if o["gt_visible"] == 0 and o["detected"]]
if fp_rows:
    p(f"## FP 상세 — {len(fp_rows)}건 (visible 아님인데 검출)")
    p()
    p("| frame | object | 선택 bbox | sem | appe | hsv |")
    p("|--:|---|---|--:|--:|--:|")
    for fi, o in fp_rows:
        sel = ",".join(map(str, o["sel_box"])) if o["sel_box"] else "—"
        p(f"| {fi} | {o['object']} | {sel} | {o['sem']:.3f} | {(o['appe'] or 0):.3f} | "
          f"{(o['hsv'] if o['hsv'] is not None else 0):.3f} |")
    p()

# ---- 프레임별 상세 (GT 라벨 프레임만) ----
p("## 프레임별 상세 (GT-라벨 94프레임, 시간순)")
p()
p("각 프레임에서 GT-visible이거나 최종 검출됐거나 후보가 선택된 객체만 표시.")
for r in recs:
    if not r["gt_labelled"]:
        continue
    ts = round((r["timestamp_ns"] - t0) / 1e9, 2)
    vis = ", ".join(r["gt_visible"]) if r["gt_visible"] else "none"
    det = ", ".join(r["detected"]) if r["detected"] else "없음"
    fns = ", ".join(f"{o['object']}({o['cause']})" for o in r["objects"]
                    if o["gt_visible"] and not o["detected"]) or "—"
    p()
    p(f"### frame {r['frame_index']}  t={ts}s")
    p(f"- GT visible: {vis}")
    p(f"- 검출: {det}")
    p(f"- FN: {fns}")
    p(f"- overlay: `frames/frame_{r['frame_index']:06d}.jpg`")
    p()
    p("| object | GT | raw/op | sem/thr | appe/gate | hsv/thr | status | 원인 |")
    p("|---|:-:|--:|---|---|---|---|---|")
    rows = [o for o in r["objects"] if o["gt_visible"] or o["detected"] or o["sel_box"]]
    for o in sorted(rows, key=lambda o: (-(o["gt_visible"] or 0), -o["detected"], o["object"])):
        g = "Y" if o["gt_visible"] else "·"
        sem = flag(o["sem"], o["sem_thr"], o["sem_pass"] if (o["sel_box"] or o["yolo_sel_n"]) else None)
        appe = flag(o["appe"] if o["sel_box"] else None, o["appe_thr"], o["appe_pass"])
        hv = flag(o["hsv"], o["hsv_thr"], hsv_pass_of(o))
        cause = "DETECTED" if o["detected"] else o["cause"]
        p(f"| {o['object']} | {g} | {o['yolo_raw_n']}/{o['yolo_op_n']} | {sem} | {appe} | {hv} | {o['final_status']} | {cause} |")

md = "\n".join(L) + "\n"
path = os.path.join(OUT, "sam_105018_fn_report.md")
open(path, "w", encoding="utf-8").write(md)
print(f"[md] {path}  ({len(md)/1024:.0f} KB, {len(L)} lines)")
