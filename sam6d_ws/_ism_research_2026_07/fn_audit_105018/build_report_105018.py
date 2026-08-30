#!/usr/bin/env python3
"""build_report_105018.py — frames.jsonl -> all-frames CSV + standalone HTML audit page."""
import csv
import json
import os
from collections import Counter, defaultdict

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(REPO, "outputs", "sam_105018_fn_audit")
JL = os.path.join(OUT, "frames.jsonl")
CSV_PATH = os.path.join(OUT, "sam_105018_all_frames.csv")
HTML_PATH = os.path.join(OUT, "sam_105018_all_frames.html")

CAUSES = ["NO_YOLO_BBOX", "SEMANTIC_REJECT", "APPEARANCE_REJECT", "HSV_REJECT",
          "SELECTION_OR_OUTPUT_MISS", "UNKNOWN"]

recs = [json.loads(l) for l in open(JL, encoding="utf-8")]
recs.sort(key=lambda r: r["frame_index"])
meta = json.load(open(os.path.join(OUT, "run_meta.json"), encoding="utf-8"))
t0 = recs[0]["timestamp_ns"]

# ------------------------------------------------------------------ CSV
FIELDS = ["frame_index", "timestamp_ns", "timestamp_s", "object_name", "gt_labelled",
          "gt_visible", "yolo_bbox_exists", "yolo_bbox_count",
          "yolo_raw_bbox_count", "yolo_raw_best_conf", "yolo_selected_count",
          "selected_bbox", "candidate_rank",
          "semantic_score", "semantic_threshold", "semantic_pass",
          "appearance_score", "appearance_threshold", "appearance_pass",
          "hsv_score", "hsv_threshold", "hsv_pass", "hsv_computed_by_gate",
          "mask_area_px", "final_detected", "final_status", "fn_root_cause",
          "reject_reason", "suppressed_by", "overlay_path"]

with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=FIELDS)
    w.writeheader()
    for r in recs:
        for o in r["objects"]:
            w.writerow({
                "frame_index": r["frame_index"], "timestamp_ns": r["timestamp_ns"],
                "timestamp_s": round((r["timestamp_ns"] - t0) / 1e9, 4),
                "object_name": o["object"],
                "gt_labelled": int(r["gt_labelled"]),
                "gt_visible": "" if o["gt_visible"] is None else o["gt_visible"],
                "yolo_bbox_exists": int(o["yolo_op_n"] > 0),
                "yolo_bbox_count": o["yolo_op_n"],
                "yolo_raw_bbox_count": o["yolo_raw_n"],
                "yolo_raw_best_conf": "" if o["yolo_raw_best"] is None else o["yolo_raw_best"],
                "yolo_selected_count": o["yolo_sel_n"],
                "selected_bbox": "" if not o["sel_box"] else ";".join(map(str, o["sel_box"])),
                "candidate_rank": o["sel_rank"] or "",
                "semantic_score": o["sem"], "semantic_threshold": o["sem_thr"],
                "semantic_pass": o["sem_pass"],
                "appearance_score": o["appe"] if o["sel_box"] else "",
                "appearance_threshold": o["appe_thr"],
                "appearance_pass": "" if o["appe_pass"] is None else o["appe_pass"],
                "hsv_score": "" if o["hsv"] is None else o["hsv"],
                "hsv_threshold": o["hsv_thr"],
                "hsv_pass": "" if o["hsv"] is None else (
                    o["hsv_pass"] if o["hsv_pass"] is not None else int(o["hsv"] >= o["hsv_thr"])),
                "hsv_computed_by_gate": o["hsv_computed_by_gate"],
                "mask_area_px": o["mask_area"],
                "final_detected": o["detected"], "final_status": o["final_status"],
                "fn_root_cause": o["cause"] if o["final_status"] == "FN" else "",
                "reject_reason": o["cause"],
                "suppressed_by": o["suppressed_by"] or "",
                "overlay_path": r["image"],
            })

# ------------------------------------------------------------------ summary
objects = meta["objects"]
tot = {"frames": len(recs), "labelled": sum(1 for r in recs if r["gt_labelled"]),
       "visible_of": 0, "tp": 0, "fn": 0, "fp": 0}
fn_by_cause = Counter()
per_obj = {n: Counter() for n in objects}
for r in recs:
    for o in r["objects"]:
        n = o["object"]
        per_obj[n]["det_all"] += o["detected"]
        if not r["gt_labelled"]:
            continue
        if o["gt_visible"]:
            tot["visible_of"] += 1
            per_obj[n]["visible"] += 1
            if o["detected"]:
                tot["tp"] += 1; per_obj[n]["detected"] += 1
            else:
                tot["fn"] += 1; per_obj[n]["fn"] += 1
                fn_by_cause[o["cause"]] += 1
                per_obj[n][o["cause"]] += 1
        elif o["detected"]:
            tot["fp"] += 1; per_obj[n]["fp"] += 1

summary = {"totals": tot, "fn_by_cause": {c: fn_by_cause.get(c, 0) for c in CAUSES},
           "per_object": {n: dict(per_obj[n]) for n in objects}}
json.dump(summary, open(os.path.join(OUT, "summary.json"), "w"), indent=2)

# ------------------------------------------------------------------ HTML payload
payload = []
for r in recs:
    payload.append({
        "i": r["frame_index"], "t": round((r["timestamp_ns"] - t0) / 1e9, 3),
        "img": r["image"], "lab": int(r["gt_labelled"]),
        "vis": r["gt_visible"] or [], "det": r["detected"],
        "o": [{
            "n": o["object"], "g": o["gt_visible"],
            "rn": o["yolo_raw_n"], "rb": o["yolo_raw_best"],
            "on": o["yolo_op_n"], "sn": o["yolo_sel_n"],
            "bx": o["sel_box"], "rk": o["sel_rank"],
            "sm": o["sem"], "st": o["sem_thr"], "sp": o["sem_pass"],
            "ap": o["appe"] if o["sel_box"] else None, "at": o["appe_thr"], "aps": o["appe_pass"],
            "hv": o["hsv"], "ht": o["hsv_thr"],
            "hp": (o["hsv_pass"] if o["hsv_pass"] is not None else
                   (None if o["hsv"] is None else int(o["hsv"] >= o["hsv_thr"]))),
            "hg": o["hsv_computed_by_gate"],
            "d": o["decision"], "det": o["detected"], "sb": o["suppressed_by"],
            "fs": o["final_status"], "c": o["cause"],
            "cd": [{"r": c["rank"], "b": c["box"], "y": c["yolo_conf"], "s": c["sem"]}
                   for c in o["cands"]],
        } for o in r["objects"]],
    })

DATA = json.dumps({"frames": payload, "summary": summary, "meta": meta,
                   "causes": CAUSES, "objects": objects}, separators=(",", ":"))

HTML = """<!doctype html><html lang="ko"><meta charset="utf-8">
<title>sam_105018 — all-frame FN root-cause audit (Phase 1C)</title>
<style>
:root{--bg:#12151a;--card:#1b202a;--line:#2c3340;--fg:#e6e9ef;--mut:#95a0b3;
 --ok:#3ecf6b;--bad:#ff5f5f;--warn:#ffb020;--blue:#5aa9ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);
 font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
header{position:sticky;top:0;z-index:20;background:#0e1116ee;backdrop-filter:blur(6px);
 border-bottom:1px solid var(--line);padding:10px 16px}
h1{font-size:16px;margin:0 0 6px}
.sub{color:var(--mut);font-size:11px;margin-bottom:8px}
.chips{display:flex;flex-wrap:wrap;gap:6px;align-items:center}
button.f{background:var(--card);color:var(--fg);border:1px solid var(--line);
 border-radius:14px;padding:4px 10px;cursor:pointer;font:inherit;font-size:11px}
button.f.on{background:var(--blue);color:#05121f;border-color:var(--blue);font-weight:700}
select,input{background:var(--card);color:var(--fg);border:1px solid var(--line);
 border-radius:6px;padding:4px 8px;font:inherit;font-size:11px}
main{padding:14px 16px;max-width:1500px;margin:0 auto}
table.sum{border-collapse:collapse;font-size:11.5px;margin:8px 0 4px;width:100%;overflow-x:auto;display:block}
table.sum th,table.sum td{border:1px solid var(--line);padding:3px 8px;text-align:right;white-space:nowrap}
table.sum th{background:#232a36;color:var(--mut);text-align:center}
table.sum td:first-child,table.sum th:first-child{text-align:left}
.kpis{display:flex;flex-wrap:wrap;gap:8px;margin:6px 0 10px}
.kpi{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:6px 12px}
.kpi b{display:block;font-size:18px}.kpi span{color:var(--mut);font-size:10.5px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;
 margin:0 0 14px;padding:10px;scroll-margin-top:150px}
.card.fn{border-left:3px solid var(--bad)}.card.tp{border-left:3px solid var(--ok)}
.chead{display:flex;gap:12px;align-items:baseline;flex-wrap:wrap;margin-bottom:6px}
.chead b{font-size:14px}.chead span{color:var(--mut);font-size:11px}
img.ov{width:100%;max-width:1330px;border-radius:6px;cursor:zoom-in;display:block;background:#000}
table.d{border-collapse:collapse;font-size:11.5px;margin-top:8px;width:100%}
table.d th,table.d td{border:1px solid var(--line);padding:2px 6px;white-space:nowrap}
table.d th{background:#232a36;color:var(--mut)}
.p{color:var(--ok)}.f2{color:var(--bad)}.m{color:var(--mut)}
tr.cd td{background:#161b24;font-size:10.5px;white-space:normal}
body.hidecd tr.cd{display:none}
.tag{border-radius:4px;padding:0 5px;font-size:10.5px;font-weight:700}
.t-TP{background:#123d22;color:var(--ok)}.t-FN{background:#3d1414;color:var(--bad)}
.t-FP{background:#3d3113;color:var(--warn)}.t-TN{background:#232a36;color:var(--mut)}
.t-NO_GT_LABEL{background:#1d2634;color:#7f93b5}
#modal{position:fixed;inset:0;background:#000d;display:none;z-index:99;
 align-items:center;justify-content:center;cursor:zoom-out}
#modal img{max-width:98vw;max-height:98vh;image-rendering:pixelated}
.nav{position:fixed;right:14px;bottom:14px;z-index:30;display:flex;gap:6px}
.nav button{background:var(--blue);color:#05121f;border:0;border-radius:8px;
 padding:8px 12px;font:inherit;font-weight:700;cursor:pointer}
.legend{color:var(--mut);font-size:11px;margin:4px 0 10px}
</style>
<header>
<h1>sam_105018 — 전 프레임 FN 원인 감사 <span style="color:var(--mut);font-weight:400">(Phase 1C 운영 기준)</span></h1>
<div class="sub" id="cfg"></div>
<div class="chips">
  <button class="f on" data-f="all">전체 프레임</button>
  <button class="f" data-f="visible">GT-visible만</button>
  <button class="f" data-f="fn">FN만</button>
  <button class="f" data-f="tp">정상 검출만</button>
  <button class="f" data-f="NO_YOLO_BBOX">YOLO BBox 없음</button>
  <button class="f" data-f="SEMANTIC_REJECT">Semantic 탈락</button>
  <button class="f" data-f="APPEARANCE_REJECT">Appearance 탈락</button>
  <button class="f" data-f="HSV_REJECT">HSV 탈락</button>
  <button class="f" data-f="SELECTION_OR_OUTPUT_MISS">Selection/output 누락</button>
  <button class="f" data-f="UNKNOWN">UNKNOWN</button>
  <button class="f" data-f="fp">FP만</button>
  <button class="f" data-f="unlabelled">GT 미라벨</button>
  <select id="obj"><option value="">객체 전체</option></select>
  <input id="qframe" size="10" placeholder="frame 검색">
  <input id="qobj" size="12" placeholder="객체명 검색">
  <label style="font-size:11px;color:var(--mut)"><input type="checkbox" id="showcd" checked style="vertical-align:-2px"> 후보 상세</label>
  <span id="cnt" style="color:var(--mut);font-size:11px"></span>
</div>
</header>
<main>
<div id="summary"></div>
<div class="legend">bbox 색: <span style="color:#5aa9ff">파랑=YOLO 후보</span> ·
 <span style="color:#3ecf6b">초록=최종 선택</span> ·
 <span style="color:#ff5f5f">빨강=최종 reject</span> — mask contour 동일 색.
 회색 HSV 값은 gate 이전 단계에서 탈락하여 <b>참고용 shadow 계산</b>(판정에 미사용).</div>
<div id="list"></div>
</main>
<div class="nav"><button id="prev">◀ 이전</button><button id="next">다음 ▶</button></div>
<div id="modal"><img></div>
<script>
const D = __DATA__;
const F = D.frames, S = D.summary, M = D.meta;
document.getElementById('cfg').textContent =
 `objects ${D.objects.length} · YOLO ${M.weights||'yolov8m-worldv2'} imgsz ${M.imgsz} conf ${M.op_conf}`
 + ` (raw 진단 pass ${M.raw_conf}) · semantic thr 0.35 · appe gate 0.55 · HSV gate ON thr ${M.thresholds.milk.hsv_gate_threshold}`
 + ` · Dinosaur hue +${M.thresholds.Dinosaur.hsv_hue_correction_ocv} ocv · training-free gate OFF`;

// ---------------- summary tables ----------------
const t = S.totals, fc = S.fn_by_cause;
const kpi = (v,l)=>`<div class="kpi"><b>${v}</b><span>${l}</span></div>`;
let html = '<div class="kpis">'
 + kpi(t.frames,'전체 프레임')
 + kpi(t.labelled,'GT 라벨 프레임')
 + kpi(t.visible_of,'GT-visible object-frame')
 + kpi(t.tp,'정상 검출 (TP)')
 + kpi(t.fn,'FN')
 + kpi(t.fp,'FP')
 + '</div><div class="kpis">'
 + D.causes.map(c=>kpi(fc[c],c)).join('')
 + '</div>';
html += '<table class="sum"><tr><th>object</th><th>visible</th><th>detected</th><th>fn</th>'
 + D.causes.map(c=>`<th>${c.toLowerCase()}</th>`).join('')
 + '<th>fp</th><th>det(전프레임)</th></tr>';
for (const n of D.objects){ const p=S.per_object[n]||{};
 html += `<tr><td>${n}</td><td>${p.visible||0}</td><td>${p.detected||0}</td><td>${p.fn||0}</td>`
  + D.causes.map(c=>`<td>${p[c]||0}</td>`).join('')
  + `<td>${p.fp||0}</td><td>${p.det_all||0}</td></tr>`; }
html += '</table>';
document.getElementById('summary').innerHTML = html;

const sel = document.getElementById('obj');
D.objects.forEach(n=>{const o=document.createElement('option');o.value=n;o.textContent=n;sel.appendChild(o);});

// ---------------- rendering ----------------
const fmt=(v,thr,p)=>v===null||v===undefined?'<span class="m">-</span>'
  :`<span class="${p===1?'p':(p===0?'f2':'m')}">${v.toFixed(3)}</span><span class="m">/${thr}</span> ${p===1?'PASS':(p===0?'FAIL':'')}`;

function rowHtml(o){
  const g = o.g===null?'<span class="m">-</span>':(o.g?'<b style="color:#9ad">visible</b>':'<span class="m">no</span>');
  const hsvNote = (o.hv!==null && !o.hg) ? ' <span class="m" title="gate 이전 탈락 → shadow 계산">*</span>' : '';
  return `<tr><td>${o.n}</td><td>${g}</td>`
   + `<td>${o.rn}</td><td>${o.on}</td><td>${o.sn}</td>`
   + `<td>${o.bx?o.bx.join(','):'<span class="m">-</span>'}</td><td>${o.rk||'<span class="m">-</span>'}</td>`
   + `<td>${fmt(o.sm,o.st,o.bx?o.sp:(o.sn?0:null))}</td>`
   + `<td>${fmt(o.ap,o.at,o.aps)}</td>`
   + `<td>${fmt(o.hv,o.ht,o.hp)}${hsvNote}</td>`
   + `<td>${o.d}</td>`
   + `<td><span class="tag t-${o.fs}">${o.fs}</span></td>`
   + `<td>${o.c}${o.sb?' ← '+o.sb:''}</td></tr>`
   + (o.cd.length?`<tr class="cd"><td colspan="13"><span class="m">후보 ${o.cd.length}개 (rank / bbox / YOLO conf / semantic):</span> `
       + o.cd.map(c=>`<span class="${o.bx&&c.b.join(',')===o.bx.join(',')?'p':'m'}">#${c.r} [${c.b.join(',')}] y=${c.y.toFixed(3)} s=${c.s===null?'-':c.s.toFixed(3)}</span>`).join(' &nbsp;|&nbsp; ')
       + `</td></tr>`:'');
}

function cardHtml(f){
  const rows = f.o.filter(o=>o.g || o.det || o.bx || o.on>0);
  const cls = f.o.some(o=>o.fs==='FN')?'fn':(f.o.some(o=>o.fs==='TP')?'tp':'');
  return `<div class="card ${cls}" id="f${f.i}" data-i="${f.i}">
   <div class="chead"><b>frame ${f.i}</b><span>t=${f.t}s</span>
    <span>GT: ${f.lab?(f.vis.length?f.vis.join(', '):'none'):'<i>미라벨</i>'}</span>
    <span>최종 검출: ${f.det.length?f.det.join(', '):'없음'}</span>
    <span>FN: ${f.o.filter(o=>o.fs==='FN').map(o=>o.n+'('+o.c+')').join(', ')||'-'}</span></div>
   <img class="ov" loading="lazy" src="${f.img}">
   <table class="d"><tr><th>object</th><th>GT</th><th>raw bbox</th><th>conf통과</th><th>최종후보</th>
    <th>선택 bbox</th><th>rank</th><th>semantic</th><th>appearance</th><th>HSV</th>
    <th>decision</th><th>status</th><th>원인</th></tr>
    ${rows.map(rowHtml).join('')}</table></div>`;
}

let cur = 'all';
function match(f){
  const on = sel.value, qo = document.getElementById('qobj').value.trim().toLowerCase();
  const qf = document.getElementById('qframe').value.trim();
  if (qf!=='' && String(f.i)!==qf) return false;
  let os = f.o;
  if (on) os = os.filter(o=>o.n===on);
  if (qo) os = os.filter(o=>o.n.toLowerCase().includes(qo));
  if (!os.length) return false;
  switch(cur){
    case 'all': return true;
    case 'visible': return os.some(o=>o.g===1);
    case 'fn': return os.some(o=>o.fs==='FN');
    case 'tp': return os.some(o=>o.fs==='TP');
    case 'fp': return os.some(o=>o.fs==='FP');
    case 'unlabelled': return !f.lab;
    default: return os.some(o=>o.fs==='FN' && o.c===cur);
  }
}
let shown = [];
function render(){
  shown = F.filter(match);
  document.getElementById('cnt').textContent = `${shown.length} / ${F.length} frames`;
  document.getElementById('list').innerHTML = shown.map(cardHtml).join('');
  idx = 0;
}
document.querySelectorAll('button.f').forEach(b=>b.onclick=()=>{
  document.querySelectorAll('button.f').forEach(x=>x.classList.remove('on'));
  b.classList.add('on'); cur=b.dataset.f; render();});
sel.onchange = render;
document.getElementById('qobj').oninput = render;
document.getElementById('qframe').oninput = render;
document.getElementById('showcd').onchange = e =>
  document.body.classList.toggle('hidecd', !e.target.checked);

// nav + zoom
let idx = 0;
function go(d){ if(!shown.length) return; idx=Math.max(0,Math.min(shown.length-1,idx+d));
  document.getElementById('f'+shown[idx].i).scrollIntoView({behavior:'smooth',block:'start'}); }
document.getElementById('prev').onclick=()=>go(-1);
document.getElementById('next').onclick=()=>go(1);
document.addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT'||e.target.tagName==='SELECT') return;
  if(e.key==='ArrowLeft')go(-1); if(e.key==='ArrowRight')go(1);
  if(e.key==='Escape')document.getElementById('modal').style.display='none';});
const modal=document.getElementById('modal');
document.addEventListener('click',e=>{
  if(e.target.classList.contains('ov')){modal.querySelector('img').src=e.target.src;modal.style.display='flex';}
  else if(e.target===modal||e.target.parentElement===modal)modal.style.display='none';});
render();
</script></html>
"""

open(HTML_PATH, "w", encoding="utf-8").write(HTML.replace("__DATA__", DATA))

n_img = len([f for f in os.listdir(os.path.join(OUT, "frames")) if f.endswith(".jpg")])
print(json.dumps(summary, indent=2, ensure_ascii=False))
print(f"\nCSV   {CSV_PATH}")
print(f"HTML  {HTML_PATH}  ({os.path.getsize(HTML_PATH)/1e6:.1f} MB)")
print(f"overlays {n_img}")
