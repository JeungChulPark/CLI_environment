#!/usr/bin/env python3
"""serve_change_review.py — Phase3 변화 판정 UI → http://127.0.0.1:8778

질문: 이 박스가 실제로 그 객체 위에 있습니까?  T=그렇다 / F=아니다 / U=불확실
변화 방향(추가/제거)과 조합해 '좋은 변화/나쁜 변화'는 자동 판정한다.
저장: outputs/phase34_change_review/change_review_task.csv 의 answer/notes
"""
import csv, http.server, json, os, shutil, socketserver, sys, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase34_change_review")
CSV_PATH = os.path.join(OUT, "change_review_task.csv"); BAK = CSV_PATH + ".bak"
PORT = int(os.environ.get("GT_PORT", "8778")); HOST = os.environ.get("GT_HOST", "0.0.0.0")

if not os.path.isfile(CSV_PATH):
    raise SystemExit(f"[err] {CSV_PATH} 없음 — build_change_task.py 먼저 실행")
if not os.path.exists(BAK): shutil.copy2(CSV_PATH, BAK)
with open(CSV_PATH, newline="", encoding="utf-8") as f:
    rd = csv.DictReader(f); FIELDS = list(rd.fieldnames); ROWS = list(rd)
for r in ROWS: r.setdefault("answer", ""); r.setdefault("notes", "")
CLASSES = sorted({r["class_name"] for r in ROWS})
print(f"{len(ROWS)}건 적재 (완료 {sum(1 for r in ROWS if r['answer'].strip())})")


def save():
    tmp = CSV_PATH + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader(); w.writerows(ROWS)
    os.replace(tmp, CSV_PATH)


PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>변화 판정</title><style>
:root{--bg:#12141a;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b;--wn:#ffb44c;--add:#3caeff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#191c22;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
.bar{height:6px;background:#232830;border-radius:3px;overflow:hidden;flex:1;min-width:140px}.bar>i{display:block;height:100%;background:var(--ok);width:0}
main{max-width:1400px;margin:0 auto;padding:10px 14px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;margin-top:12px}
button.c{background:#1f242c;border:2px solid var(--line);color:var(--fg);border-radius:8px;padding:16px 10px;cursor:pointer;font-size:16px;font-weight:700}
button.c small{display:block;font-weight:400;color:var(--dim);font-size:12px;margin-top:3px}
button.c[data-v=T].on{border-color:var(--ok);background:#173324}
button.c[data-v=F].on{border-color:var(--bad);background:#3a1f1f}
button.c[data-v=U].on{border-color:var(--wn);background:#3a2f17}
button.c.good{border-left:6px solid var(--ok)}button.c.poor{border-left:6px solid var(--bad)}
.nav{display:flex;gap:8px;align-items:center;margin-top:12px;flex-wrap:wrap}
button.nv{background:#232830;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:9px 16px;cursor:pointer}
input[type=text],select{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:8px 10px}
.muted{color:var(--dim);font-size:12px}.cls{font-size:16px;font-weight:700;color:#ffe08a}
.note{background:#181c22;border-left:3px solid #4c9aff;padding:7px 12px;margin:8px 0;border-radius:0 6px 6px 0;font-size:13px}
.add{color:var(--add);font-weight:700}.rem{color:var(--bad);font-weight:700}
.verdict{font-weight:700;margin-left:8px}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style><header><div class=hrow>
<b>이 박스가 실제로 그 객체 위에 있습니까?</b>
<span class=tag id=cid></span><span class=cls id=cls></span><span class=muted id=loc></span><span class=muted id=pos></span>
<div class=bar><i id=prog></i></div><span id=done>0</span><span class=muted>/<span id=tot>0</span></span>
<select id=fcls></select>
<select id=fk><option value=todo selected>미완료만</option><option value=all>전체</option>
<option value=lost_TP>집계상 TP감소만</option><option value=gain_TP>집계상 TP증가만</option>
<option value=new_FP>집계상 FP증가만</option><option value=removed_FP>집계상 FP감소만</option></select>
</div></header><main>
<div class=note id=meta></div><img id=img alt="">
<div class=grid id=grid></div>
<div class=nav><button class=nv id=prev><kbd>←</kbd> 이전</button><button class=nv id=next>다음 <kbd>Space</kbd></button>
<input type=text id=note placeholder="메모 (선택)" style="flex:1;min-width:220px"></div>
<div class=muted style="margin-top:8px"><kbd>T</kbd> / <kbd>F</kbd> / <kbd>U</kbd> · 선택지 순서는 변화 방향에 맞춰 바뀝니다(위=좋은 변화) · <span class=add>파란 박스=신규가 추가한 검출</span> · <span class=rem>빨간 박스=신규가 없앤 검출</span> · 회색 얇은 선=같은 프레임의 다른 검출 · 즉시 저장</div>
</main><script>
let rows=[],view=[],i=0;const $=s=>document.querySelector(s);const isDone=r=>(r.answer||'').trim()!=='';
function cur(){return rows[view[i]];}
function verdict(r){const a=(r.answer||'').trim().toUpperCase();if(!a||a==='U')return '';
 const good=(r.action==='추가')===(a==='T');
 return `<span class=verdict style="color:${good?'#3ddc84':'#ff6b6b'}">→ ${good?'좋은 변화':'나쁜 변화'}</span>`;}
function render(){
 if(!view.length){$('#cid').textContent='해당 없음';$('#img').removeAttribute('src');$('#meta').innerHTML='';return;}
 if(i<0)i=0;if(i>=view.length)i=view.length-1;const r=cur();
 $('#cid').textContent=r.case_id;$('#cls').textContent=r.class_name;
 $('#loc').textContent=`${r.bag_name} f${r.frame_id}`;$('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${view[i]}`;$('#note').value=r.notes||'';
 $('#meta').innerHTML=`<b>신규 파이프라인이 이 검출을 <span class="${r.action==='추가'?'add':'rem'}">${r.action}</span>했다</b>`+
  ` <span class=muted>(${r.agg_label} — 이 라벨은 GT 목록만 보고 붙인 것이라 틀릴 수 있음)</span>${verdict(r)}`+
  `<br><span class=muted>GT 보임: ${r.gt_visible||'—'}</span>`+
  `<br><span class=muted>현행: ${r.OFF||'—'}</span><br><span class=muted>신규: ${r.ON||'—'}</span>`;
 const add=r.action==='추가';
 const OPT=add?[['T','T · 맞다 — 그 객체 위','좋은 변화: 놓치던 객체를 찾음 (FN→TP)'],
                ['F','F · 아니다 — 다른 물체','나쁜 변화: 오검출이 추가됨 (TN→FP)'],
                ['U','U · 불확실','판단 보류']]
              :[['F','F · 아니다 — 다른 물체','좋은 변화: 오검출을 지움 (FP→TN)'],
                ['T','T · 맞다 — 그 객체 위','나쁜 변화: 진짜 검출을 잃음 (TP→FN)'],
                ['U','U · 불확실','판단 보류']];
 $('#grid').innerHTML=OPT.map(o=>`<button class="c ${o[2].startsWith('좋은')?'good':(o[2].startsWith('나쁜')?'poor':'')}" data-v="${o[0]}">${o[1]}<small>${o[2]}</small></button>`).join('');
 document.querySelectorAll('button.c').forEach(b=>{b.onclick=()=>pick(b.dataset.v);
  b.classList.toggle('on',b.dataset.v===(r.answer||'').trim().toUpperCase());});
 const d=rows.filter(isDone).length;$('#done').textContent=d;$('#tot').textContent=rows.length;
 $('#prog').style.width=(100*d/rows.length)+'%';}
function commit(){const r=cur();fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({index:view[i],answer:r.answer,notes:r.notes})});}
function pick(v){const r=cur();r.answer=((r.answer||'').trim().toUpperCase()===v)?'':v;
 commit();render();if((r.answer||'').trim())setTimeout(()=>step(1),140);}
function step(d){let j=i+d;while(j>=0&&j<view.length&&isDone(rows[view[j]]))j+=d;
 if(j<0||j>=view.length)j=Math.max(0,Math.min(view.length-1,i+d));i=j;render();}
function applyFilter(){const c=$('#fcls').value,f=$('#fk').value;
 view=rows.map((r,n)=>n).filter(n=>{const r=rows[n];
  if(c!=='all'&&r.class_name!==c)return false;
  if(f==='todo')return !isDone(r);if(f==='all')return true;return r.kind===f;});
 i=Math.max(0,view.findIndex(n=>!isDone(rows[n])));render();}
$('#prev').onclick=()=>step(-1);$('#next').onclick=()=>step(1);
$('#fcls').onchange=applyFilter;$('#fk').onchange=applyFilter;
$('#note').oninput=e=>{cur().notes=e.target.value;commit();};
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;const r=cur();if(!r)return;
 const k=e.key.toUpperCase();
 if(k==='T'||k==='F'||k==='U'){e.preventDefault();pick(k);return;}
 if(e.key===' '||e.key==='ArrowRight'){e.preventDefault();step(1);}
 if(e.key==='Backspace'||e.key==='ArrowLeft'){e.preventDefault();step(-1);}});
fetch('/api/rows').then(r=>r.json()).then(d=>{rows=d.rows;
 $('#fcls').innerHTML='<option value=all>전체 class</option>'+d.classes.map(c=>`<option value="${c}">${c}</option>`).join('');
 applyFilter();});
</script></html>"""


class H(http.server.BaseHTTPRequestHandler):
    def _s(self, code, ct, body):
        self.send_response(code); self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        p = urllib.parse.urlparse(self.path).path
        if p == "/favicon.ico": return self._s(204, "image/x-icon", b"")
        if p == "/healthz": return self._s(200, "text/plain", f"ok rows={len(ROWS)}".encode())
        if p in ("/", "/index.html"): return self._s(200, "text/html; charset=utf-8", PAGE.encode())
        if p == "/api/rows":
            return self._s(200, "application/json; charset=utf-8",
                           json.dumps({"rows": ROWS, "classes": CLASSES}, ensure_ascii=False).encode())
        if p.startswith("/img/"):
            try: rel = ROWS[int(p[5:])]["image_path"]
            except Exception: return self._s(404, "text/plain", b"no")
            fp = rel if os.path.isabs(rel) else os.path.join(OUT, rel)
            if not os.path.isfile(fp): return self._s(404, "text/plain", b"missing")
            with open(fp, "rb") as f: return self._s(200, "image/png", f.read())
        return self._s(404, "text/plain", b"no")

    def do_POST(self):
        if urllib.parse.urlparse(self.path).path != "/api/save":
            return self._s(404, "text/plain", b"no")
        n = int(self.headers.get("Content-Length", "0"))
        try:
            d = json.loads(self.rfile.read(n) or b"{}"); r = ROWS[int(d["index"])]
        except Exception:
            return self._s(400, "text/plain", b"bad")
        for k in ("answer", "notes"):
            if k in d: r[k] = str(d[k])
        save()
        return self._s(200, "application/json",
                       json.dumps({"done": sum(1 for x in ROWS if x["answer"].strip())}).encode())

    def log_message(self, *a): pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        miss = [r["case_id"] for r in ROWS if not os.path.isfile(os.path.join(OUT, r["image_path"]))]
        print(f"  행 {len(ROWS)} · 이미지 누락 {len(miss)}"); raise SystemExit(0 if not miss else 1)
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n  저장 : {CSV_PATH}\n")
    with S((HOST, PORT), H) as h: h.serve_forever()
