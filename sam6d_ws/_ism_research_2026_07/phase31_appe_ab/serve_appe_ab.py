#!/usr/bin/env python3
"""serve_appe_ab.py — appearance 특징 A/B 뷰어 → http://127.0.0.1:8774"""
import csv, http.server, json, os, socketserver, urllib.parse
from collections import Counter
HERE=os.path.dirname(os.path.abspath(__file__)); RSRCH=os.path.dirname(HERE); REPO=os.path.dirname(RSRCH)
OUT=os.path.join(REPO,"outputs","phase31_appe_ab"); VIZ=os.path.join(OUT,"viz")
ROWS=list(csv.DictReader(open(os.path.join(OUT,"csv","appe_ab_manifest.csv"),encoding="utf-8")))
MET=json.load(open(os.path.join(OUT,"metrics","appe_ab.json")))
BAGS=sorted({r["bag"] for r in ROWS}); PORT=int(os.environ.get("GT_PORT","8774"))
g=Counter(); nf=Counter()
for r in ROWS:
    for o in filter(None,r["B2_gain_TP"].split(";")): g[o]+=1
    for o in filter(None,r["B2_new_FP"].split(";")): nf[o]+=1
print(f"{len(ROWS)}프레임 · B2가 얻은 TP {sum(g.values())} · 새 FP {sum(nf.values())}")
PAGE="""<!doctype html><html lang=ko><meta charset=utf-8><title>appearance A/B</title><style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#171a1f;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
select,button{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px 12px;cursor:pointer}
main{padding:10px 14px}#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.cap{display:flex;margin-top:6px;color:var(--dim);font-size:12px;text-align:center}.cap>div{flex:1}
.muted{color:var(--dim);font-size:12px}.note{background:#181c22;border-left:3px solid #4c9aff;padding:8px 12px;margin:8px 0;border-radius:0 6px 6px 0;font-size:13px}
.gain{color:var(--ok);font-weight:700}.fp{color:var(--bad);font-weight:700}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style><header><div class=hrow>
<b>원본 | A 현행 block11 | B1 block2+9(보수) | B2 block2+9(TP우선)</b>
<span class=tag id=name></span><span class=muted id=pos></span><span class=muted id=sum></span>
<select id=f><option value=changed selected>A와 달라진 프레임</option><option value=gain>B2가 더 찾은 것(TP↑)</option>
<option value=fp>B2의 새 오검출(FP↑)</option><option value=dup>새 FP 중 '다른 객체와 같은 박스'</option><option value=indep>새 FP 중 '독립 박스'</option><option value=lost>B2가 놓친 것</option><option value=all>전체</option></select>
<select id=fb></select><button id=prev>← 이전</button><button id=next>다음 →</button></div></header>
<main><div class=note id=diff></div><img id=img alt="">
<div class=cap><div>원본</div><div>A 현행 (block11, gate 0.55)</div><div>B1 block2+9 (gate 0.620)</div><div>B2 block2+9 (gate 0.605)</div></div>
<div class=muted style="margin-top:8px"><span class=gain>초록 박스=GT에 있는 객체(TP)</span> · <span class=fp>빨강 박스=GT에 없는 객체(FP)</span> · 같은 좌표 박스가 겹치면 안쪽으로 들여서 그림 · <kbd>←</kbd><kbd>→</kbd></div></main>
<script>
let all=[],view=[],i=0;const $=s=>document.querySelector(s);
function cur(){return all[view[i]];}
function render(){if(!view.length){$('#name').textContent='해당 없음';$('#img').removeAttribute('src');$('#diff').innerHTML='';return;}
 if(i<0)i=0;if(i>=view.length)i=view.length-1;const r=cur();
 $('#name').textContent=r.name;$('#pos').textContent=`${i+1} / ${view.length}`;$('#img').src=`/img/${view[i]}`;
 let h=`<b>GT 보임:</b> ${r.gt_visible||'—'}<br><b>A:</b> ${r.A||'—'}<br><b>B2:</b> ${r.B2||'—'}`;
 if(r.B2_gain_TP)h+=`<br><span class=gain>B2가 더 찾음: ${r.B2_gain_TP}</span>`;
 if(r.B2_new_FP)h+=`<br><span class=fp>B2 새 오검출: ${r.B2_new_FP}</span>`;
 if(r.B2_dup_box)h+=`<br><span class=fp>↳ 다른 객체와 <b>같은 박스</b>에 이중 라벨: ${r.B2_dup_box}</span>`;
 if(r.B2_lost_TP)h+=`<br><span class=fp>B2가 놓침: ${r.B2_lost_TP}</span>`;
 $('#diff').innerHTML=h;}
function applyFilter(){const f=$('#f').value,b=$('#fb').value;
 view=all.map((r,n)=>n).filter(n=>{const r=all[n];if(b!=='all'&&r.bag!==b)return false;
  if(f==='changed')return r.changed==='1';if(f==='gain')return !!r.B2_gain_TP;
  if(f==='fp')return !!r.B2_new_FP;if(f==='dup')return !!r.B2_dup_box;
  if(f==='indep')return !!r.B2_new_FP&&r.B2_new_FP.split(';').some(o=>!(r.B2_dup_box||'').split(';').includes(o));if(f==='lost')return !!r.B2_lost_TP;return true;});
 i=0;render();}
$('#prev').onclick=()=>{i=Math.max(0,i-1);render();};$('#next').onclick=()=>{i=Math.min(view.length-1,i+1);render();};
$('#f').onchange=applyFilter;$('#fb').onchange=applyFilter;
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;
 if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();i=Math.min(view.length-1,i+1);render();}
 if(e.key==='ArrowLeft'){e.preventDefault();i=Math.max(0,i-1);render();}});
fetch('/api/data').then(r=>r.json()).then(d=>{all=d.rows;
 $('#fb').innerHTML='<option value=all>전체 bag</option>'+d.bags.map(b=>`<option>${b}</option>`).join('');
 const m=d.met;$('#sum').innerHTML=`A TP${m.A.TP}/FP${m.A.FP} F1 ${m.A.f1} &nbsp;|&nbsp; B1 TP${m.B1.TP}/FP${m.B1.FP} F1 ${m.B1.f1} &nbsp;|&nbsp; <b>B2 TP${m.B2.TP}/FP${m.B2.FP} F1 ${m.B2.f1}</b>`;
 applyFilter();});
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
        if p=="/api/data":return s._s(200,"application/json; charset=utf-8",
            json.dumps({"rows":ROWS,"bags":BAGS,"met":MET},ensure_ascii=False).encode())
        if p.startswith("/img/"):
            try:fp=os.path.join(VIZ,ROWS[int(p[5:])]["image"])
            except Exception:return s._s(404,"text/plain",b"no")
            if not os.path.isfile(fp):return s._s(404,"text/plain",b"missing")
            return s._s(200,"image/png",open(fp,"rb").read())
        return s._s(404,"text/plain",b"no")
    def log_message(s,*a):pass
class S(socketserver.ThreadingTCPServer):allow_reuse_address=True;daemon_threads=True
if __name__=="__main__":
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n")
    with S(("0.0.0.0",PORT),H) as h:h.serve_forever()
