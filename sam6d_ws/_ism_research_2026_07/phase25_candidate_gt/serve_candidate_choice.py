#!/usr/bin/env python3
"""serve_candidate_choice.py — YOLO top-3 후보 정답 판정 웹 뷰어 (표준 라이브러리만).

질문: "top-3 후보 중 어느 것이 실제 대상 물체 위에 있습니까?"
  1/2/3 = 그 번호가 정답 · 0 = 정답 후보 없음 · U = 불확실
저장: candidate_choice_task.csv 의 answer / notes 열 (원자적 교체).

실행:
  ~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase25_candidate_gt/serve_candidate_choice.py
  브라우저: http://127.0.0.1:8769
"""
import csv, http.server, json, os, shutil, socketserver, sys, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase25_candidate_gt")
CSV_PATH = os.path.join(OUT, "candidate_choice_task.csv")
BAK = CSV_PATH + ".bak"
PORT = int(os.environ.get("GT_PORT", "8769"))
HOST = os.environ.get("GT_HOST", "0.0.0.0")

if not os.path.isfile(CSV_PATH):
    raise SystemExit(f"[err] {CSV_PATH} 없음 — build_candidate_choice_task.py 먼저 실행")
if not os.path.exists(BAK):
    shutil.copy2(CSV_PATH, BAK)
with open(CSV_PATH, newline="", encoding="utf-8") as f:
    rd = csv.DictReader(f); FIELDS = list(rd.fieldnames); ROWS = list(rd)
for r in ROWS:
    r.setdefault("answer", ""); r.setdefault("notes", "")
CLASSES = sorted({r["class_name"] for r in ROWS})
print(f"{len(ROWS)}건 적재 (완료 {sum(1 for r in ROWS if r['answer'].strip())})")


def save():
    tmp = CSV_PATH + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader(); w.writerows(ROWS)
    os.replace(tmp, CSV_PATH)


PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>후보 정답 판정</title>
<style>
:root{--bg:#12141a;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b;--wn:#ffb44c}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#191c22;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
.bar{height:6px;background:#232830;border-radius:3px;overflow:hidden;flex:1;min-width:140px}
.bar>i{display:block;height:100%;background:var(--ok);width:0}
main{max-width:1400px;margin:0 auto;padding:10px 14px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:9px;margin-top:12px}
button.c{background:#1f242c;border:2px solid var(--line);color:var(--fg);border-radius:8px;
  padding:15px 10px;cursor:pointer;font-size:16px;text-align:center;font-weight:700}
button.c small{display:block;font-weight:400;color:var(--dim);font-size:12px;margin-top:3px}
button.c.on{border-color:var(--ok);background:#173324}
button.c[data-v="0"].on{border-color:var(--bad);background:#3a1f1f}
button.c[data-v="U"].on{border-color:var(--wn);background:#3a2f17}
.nav{display:flex;gap:8px;align-items:center;margin-top:12px;flex-wrap:wrap}
button.nv{background:#232830;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:9px 16px;cursor:pointer}
input[type=text],select{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:8px 10px}
.muted{color:var(--dim);font-size:12px}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
.cls{font-size:16px;font-weight:700;color:#ffe08a}
</style>
<header><div class=hrow>
 <b>top-3 후보 중 어느 것이 실제 대상 물체 위에 있습니까?</b>
 <span class=tag id=cid></span><span class=cls id=cls></span>
 <span class=muted id=loc></span><span class=muted id=pos></span>
 <div class=bar><i id=prog></i></div><span id=done>0</span><span class=muted>/<span id=tot>0</span></span>
 <select id=fcls></select>
 <select id=ffill><option value=todo selected>미완료만</option><option value=all>전체</option><option value=0>0(정답없음)만</option><option value=mis>선택실패만</option></select>
</div></header>
<main>
 <img id=img alt="">
 <div class=grid id=grid></div>
 <div class=nav>
  <button class=nv id=prev><kbd>←</kbd> 이전</button>
  <button class=nv id=next>다음 <kbd>Space</kbd></button>
  <input type=text id=note placeholder="메모 (선택)" style="flex:1;min-width:220px">
 </div>
 <div class=muted style="margin-top:8px">
  <kbd>1</kbd><kbd>2</kbd><kbd>3</kbd> 정답 후보 번호 · <kbd>0</kbd> 정답 없음 · <kbd>U</kbd> 불확실 · ★=파이프라인이 고른 후보 · 선택 즉시 저장
 </div>
</main>
<script>
let rows=[],view=[],i=0; const $=s=>document.querySelector(s);
const isDone=r=>(r.answer||'').trim()!=='';
function cur(){return rows[view[i]];}
function render(){
 if(!view.length){$('#cid').textContent='해당 없음';$('#img').removeAttribute('src');$('#grid').innerHTML='';return;}
 if(i<0)i=0; if(i>=view.length)i=view.length-1;
 const r=cur(), n=parseInt(r.n_cands);
 $('#cid').textContent=r.case_id; $('#cls').textContent=r.class_name;
 $('#loc').textContent=`${r.bag_name} f${r.frame_id} · 후보 ${n}개 · 선택=${r.selected_rank}`;
 $('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${view[i]}`; $('#note').value=r.notes||'';
 const confs=[r.conf1,r.conf2,r.conf3];
 let opts=[];
 for(let k=1;k<=n;k++) opts.push([String(k),`후보 ${k}`+(String(k)===r.selected_rank?' ★':''),`conf ${confs[k-1]}`]);
 opts.push(['0','0 정답 없음','YOLO 실패']); opts.push(['U','U 불확실','']);
 $('#grid').innerHTML=opts.map(o=>`<button class=c data-v="${o[0]}">${o[1]}<small>${o[2]}</small></button>`).join('');
 document.querySelectorAll('button.c').forEach(b=>{
  b.onclick=()=>pick(b.dataset.v);
  b.classList.toggle('on',b.dataset.v===(r.answer||'').trim().toUpperCase());});
 const d=rows.filter(isDone).length;
 $('#done').textContent=d; $('#tot').textContent=rows.length;
 $('#prog').style.width=(100*d/rows.length)+'%';}
function commit(){const r=cur();
 fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({index:view[i],answer:r.answer,notes:r.notes})});}
function pick(v){const r=cur();
 if((r.answer||'').trim().toUpperCase()===v){r.answer='';}else{r.answer=v;}
 commit(); render(); if((r.answer||'').trim()) setTimeout(()=>step(1),110);}
function step(d){let j=i+d;
 while(j>=0&&j<view.length&&isDone(rows[view[j]])) j+=d;
 if(j<0||j>=view.length) j=Math.max(0,Math.min(view.length-1,i+d)); i=j; render();}
function applyFilter(){const c=$('#fcls').value, f=$('#ffill').value;
 view=rows.map((r,n)=>n).filter(n=>{const r=rows[n];
  if(c!=='all'&&r.class_name!==c)return false;
  if(f==='todo')return !isDone(r);
  if(f==='0')return (r.answer||'').trim()==='0';
  if(f==='mis'){const a=(r.answer||'').trim(); return a&&a!=='0'&&a!=='U'&&a!==r.selected_rank;}
  return true;});
 i=Math.max(0,view.findIndex(n=>!isDone(rows[n]))); render();}
$('#prev').onclick=()=>step(-1); $('#next').onclick=()=>step(1);
$('#fcls').onchange=applyFilter; $('#ffill').onchange=applyFilter;
$('#note').oninput=e=>{cur().notes=e.target.value; commit();};
addEventListener('keydown',e=>{ if(e.target.tagName==='INPUT')return;
 const r=cur(); if(!r)return; const n=parseInt(r.n_cands); const k=e.key.toUpperCase();
 if(['1','2','3'].includes(k)&&parseInt(k)<=n){e.preventDefault();pick(k);return;}
 if(k==='0'||k==='U'){e.preventDefault();pick(k);return;}
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
        if p == "/favicon.ico":
            return self._s(204, "image/x-icon", b"")
        if p == "/healthz":
            return self._s(200, "text/plain; charset=utf-8", f"ok rows={len(ROWS)}".encode())
        if p in ("/", "/index.html"):
            return self._s(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        if p == "/api/rows":
            return self._s(200, "application/json; charset=utf-8",
                           json.dumps({"rows": ROWS, "classes": CLASSES}, ensure_ascii=False).encode())
        if p.startswith("/img/"):
            try:
                rel = ROWS[int(p[5:])]["image_path"]
            except (ValueError, IndexError):
                return self._s(404, "text/plain", b"no")
            fp = rel if os.path.isabs(rel) else os.path.join(OUT, rel)
            if not os.path.isfile(fp):
                return self._s(404, "text/plain", b"missing")
            with open(fp, "rb") as f:
                return self._s(200, "image/png", f.read())
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
            if k in d:
                r[k] = str(d[k])
        save()
        return self._s(200, "application/json",
                       json.dumps({"done": sum(1 for x in ROWS if x["answer"].strip())}).encode())

    def log_message(self, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        miss = [r["case_id"] for r in ROWS if not os.path.isfile(os.path.join(OUT, r["image_path"]))]
        print(f"  행 {len(ROWS)} · 완료 {sum(1 for r in ROWS if r['answer'].strip())} · 이미지 누락 {len(miss)}")
        raise SystemExit(0 if not miss else 1)
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n  저장 : {CSV_PATH}\n  종료 : Ctrl+C\n")
    with S((HOST, PORT), H) as httpd:
        httpd.serve_forever()
