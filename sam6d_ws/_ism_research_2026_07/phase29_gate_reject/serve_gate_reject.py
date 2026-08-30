#!/usr/bin/env python3
"""serve_gate_reject.py — '정답 후보인데 게이트 탈락' 뷰어 → http://127.0.0.1:8773"""
import csv, http.server, json, os, socketserver, urllib.parse
HERE=os.path.dirname(os.path.abspath(__file__)); RSRCH=os.path.dirname(HERE); REPO=os.path.dirname(RSRCH)
OUT=os.path.join(REPO,"outputs","phase29_gate_reject"); CASES=os.path.join(OUT,"cases")
ROWS=list(csv.DictReader(open(os.path.join(OUT,"csv","gate_reject_cases.csv"),encoding="utf-8")))
ROWS.sort(key=lambda r:-float(r["min_ratio"]))
CLS=sorted({r["class_name"] for r in ROWS}); PORT=int(os.environ.get("GT_PORT","8773"))
print(f"{len(ROWS)}건 (탈락 {sum(1 for r in ROWS if r['accept']=='0')})")
PAGE="""<!doctype html><html lang=ko><meta charset=utf-8><title>게이트 탈락 사례</title><style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#171a1f;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
select,button{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px 12px;cursor:pointer}
main{padding:10px 14px}#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.muted{color:var(--dim);font-size:12px}.note{background:#181c22;border-left:3px solid #4c9aff;padding:8px 12px;margin:8px 0;border-radius:0 6px 6px 0;font-size:13px}
.bad{color:var(--bad);font-weight:700}kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style><header><div class=hrow>
<b>정답 후보인데 게이트가 거절한 사례</b><span class=tag id=name></span><span class=muted id=pos></span><span class=muted id=info></span>
<select id=fg><option value=all>전체 탈락</option><option value=near selected>아깝게(≥0.9×임계)</option>
<option value=appearance>appearance 탈락</option><option value=semantic>semantic 탈락</option>
<option value=both>둘 다 탈락</option><option value=hsv>HSV 탈락</option></select>
<select id=fc></select><select id=fv><option value=all>전체 가시등급</option><option value=1>1 온전</option><option value=2>2 일부</option><option value=3>3 극히일부</option></select>
<button id=prev>← 이전</button><button id=next>다음 →</button></div></header>
<main><div class=note id=diff></div><img id=img alt="">
<div class=muted style="margin-top:8px">왼쪽=프레임(빨강=정답 후보) · 가운데=후보 crop / TARGET 템플릿 · 오른쪽=게이트 점수(흰 선=임계) · <kbd>←</kbd><kbd>→</kbd></div></main>
<script>
let all=[],view=[],i=0;const $=s=>document.querySelector(s);
function cur(){return all[view[i]];}
function render(){if(!view.length){$('#name').textContent='해당 없음';$('#img').removeAttribute('src');$('#diff').innerHTML='';return;}
 if(i<0)i=0;if(i>=view.length)i=view.length-1;const r=cur();
 $('#name').textContent=r.case_id;$('#pos').textContent=`${i+1} / ${view.length}`;
 $('#info').textContent=`가시등급 ${r.visibility} · conf ${r.conf}`;
 $('#img').src=`/img/${view[i]}`;
 $('#diff').innerHTML=`<b>class:</b> ${r.class_name} &nbsp; <b class=bad>탈락 게이트: ${r.fail_gates||'—'}</b><br>`+
  `임계 대비 &nbsp; semantic ×${r.sem_ratio} &nbsp; appearance ×${r.appe_ratio} &nbsp; HSV ×${r.hsv_ratio}`;}
function applyFilter(){const g=$('#fg').value,c=$('#fc').value,v=$('#fv').value;
 view=all.map((r,n)=>n).filter(n=>{const r=all[n];if(r.accept==='1')return false;
  if(c!=='all'&&r.class_name!==c)return false; if(v!=='all'&&r.visibility!==v)return false;
  if(g==='near')return parseFloat(r.min_ratio)>=0.9;
  if(g==='both')return r.fail_gates.includes('semantic')&&r.fail_gates.includes('appearance');
  if(g!=='all')return r.fail_gates.split(';').includes(g); return true;});
 i=0;render();}
$('#prev').onclick=()=>{i=Math.max(0,i-1);render();};$('#next').onclick=()=>{i=Math.min(view.length-1,i+1);render();};
['fg','fc','fv'].forEach(k=>$('#'+k).onchange=applyFilter);
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;
 if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();i=Math.min(view.length-1,i+1);render();}
 if(e.key==='ArrowLeft'){e.preventDefault();i=Math.max(0,i-1);render();}});
fetch('/api/rows').then(r=>r.json()).then(d=>{all=d.rows;
 $('#fc').innerHTML='<option value=all>전체 class</option>'+d.cls.map(c=>`<option>${c}</option>`).join('');applyFilter();});
</script></html>"""
class H(http.server.BaseHTTPRequestHandler):
    def _s(s,c,ct,b):
        s.send_response(c);s.send_header("Content-Type",ct);s.send_header("Content-Length",str(len(b)))
        s.send_header("Cache-Control","no-store");s.end_headers();s.wfile.write(b)
    def do_GET(s):
        p=urllib.parse.urlparse(s.path).path
        if p=="/favicon.ico":return s._s(204,"image/x-icon",b"")
        if p=="/healthz":return s._s(200,"text/plain",f"ok rows={len(ROWS)}".encode())
        if p in ("/","/index.html"):return s._s(200,"text/html; charset=utf-8",PAGE.encode())
        if p=="/api/rows":return s._s(200,"application/json; charset=utf-8",
            json.dumps({"rows":ROWS,"cls":CLS},ensure_ascii=False).encode())
        if p.startswith("/img/"):
            try:fp=os.path.join(CASES,ROWS[int(p[5:])]["image"])
            except Exception:return s._s(404,"text/plain",b"no")
            if not os.path.isfile(fp):return s._s(404,"text/plain",b"missing")
            return s._s(200,"image/png",open(fp,"rb").read())
        return s._s(404,"text/plain",b"no")
    def log_message(s,*a):pass
class S(socketserver.ThreadingTCPServer):allow_reuse_address=True;daemon_threads=True
if __name__=="__main__":
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n")
    with S(("0.0.0.0",PORT),H) as h:h.serve_forever()
