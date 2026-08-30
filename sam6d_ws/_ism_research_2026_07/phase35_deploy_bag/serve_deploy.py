#!/usr/bin/env python3
"""serve_deploy.py — 승격된 배포 설정의 ros2 bag 인식 결과 뷰어 → http://127.0.0.1:8779"""
import csv, http.server, json, os, socketserver, urllib.parse
HERE=os.path.dirname(os.path.abspath(__file__)); RSRCH=os.path.dirname(HERE); REPO=os.path.dirname(RSRCH)
OUT=os.path.join(REPO,"outputs","phase35_deploy_bag_viz"); VIZ=os.path.join(OUT,"viz")
ROWS=list(csv.DictReader(open(os.path.join(OUT,"csv","deploy_manifest.csv"),encoding="utf-8")))
BAGS=sorted({r["bag"] for r in ROWS}); PORT=int(os.environ.get("GT_PORT","8779"))
G=[r for r in ROWS if r["has_gt"]=="1"]
def cnt(rs,k): return sum(len([x for x in r[k].split(";") if x]) for r in rs)
MET={"frames":len(ROWS),"gt_frames":len(G),"det":sum(int(r["n_det"]) for r in ROWS),
     "TP":cnt(G,"tp"),"FP":cnt(G,"fp"),"FN":cnt(G,"miss")}
print(f"{len(ROWS)}프레임 적재 · {MET}")
PAGE="""<!doctype html><html lang=ko><meta charset=utf-8><title>배포 설정 인식 결과</title><style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b;--nu:#3caeff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#171a1f;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
select,button{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px 12px;cursor:pointer}
main{padding:10px 14px}#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.cap{display:flex;margin-top:6px;color:var(--dim);font-size:12px;text-align:center}.cap>div{flex:1}
.muted{color:var(--dim);font-size:12px}
.note{background:#181c22;border-left:3px solid #4c9aff;padding:8px 12px;margin:8px 0;border-radius:0 6px 6px 0;font-size:13px}
.g{color:var(--ok);font-weight:700}.b{color:var(--bad);font-weight:700}.n{color:var(--nu);font-weight:700}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style><header><div class=hrow>
<b>ros2 bag · 배포 설정(appe 2+9 g0.605 + cross-object NMS) 인식 결과</b>
<span class=tag id=name></span><span class=muted id=pos></span><span class=muted id=sum></span>
<select id=f><option value=all selected>전체 프레임</option><option value=det>검출 있는 프레임</option>
<option value=gt>GT 있는 프레임</option><option value=fp>GT대비 오검출 있는 프레임</option>
<option value=miss>GT대비 놓친 객체 있는 프레임</option><option value=none>검출 0 프레임</option></select>
<select id=fb></select><button id=prev>← 이전</button><button id=next>다음 →</button></div></header>
<main><div class=note id=diff></div><img id=img alt="">
<div class=cap><div>원본 (ros2 bag 입력)</div><div>인식 결과</div></div>
<div class=muted style="margin-top:8px"><span class=g>초록=GT에 있는 객체</span> · <span class=b>빨강=GT에 없는 객체</span> · <span class=n>파랑=GT 없는 프레임(정답 라벨 없음)</span> · <kbd>←</kbd><kbd>→</kbd></div></main>
<script>
let all=[],view=[],i=0;const $=s=>document.querySelector(s);
function cur(){return all[view[i]];}
function render(){if(!view.length){$('#name').textContent='해당 없음';$('#img').removeAttribute('src');$('#diff').innerHTML='';return;}
 if(i<0)i=0;if(i>=view.length)i=view.length-1;const r=cur();
 $('#name').textContent=r.name;$('#pos').textContent=`${i+1} / ${view.length}`;$('#img').src=`/img/${view[i]}`;
 let h=`<b>검출:</b> ${r.detected||'—'} (${r.n_det}개)`;
 if(r.has_gt==='1'){h+=`<br><b>GT 보임:</b> ${r.gt_visible||'—'}`;
  if(r.tp)h+=`<br><span class=g>일치: ${r.tp}</span>`;
  if(r.fp)h+=`<br><span class=b>GT에 없음: ${r.fp}</span>`;
  if(r.miss)h+=`<br><span class=b>놓침: ${r.miss}</span>`;}
 else h+=`<br><span class=n>이 프레임은 GT 라벨이 없습니다 (박스는 파랑으로 표시)</span>`;
 $('#diff').innerHTML=h;}
function applyFilter(){const f=$('#f').value,b=$('#fb').value;
 view=all.map((r,n)=>n).filter(n=>{const r=all[n];if(b!=='all'&&r.bag!==b)return false;
  if(f==='det')return +r.n_det>0;if(f==='gt')return r.has_gt==='1';
  if(f==='fp')return !!r.fp;if(f==='miss')return !!r.miss;
  if(f==='none')return +r.n_det===0;return true;});
 i=0;render();}
$('#prev').onclick=()=>{i=Math.max(0,i-1);render();};$('#next').onclick=()=>{i=Math.min(view.length-1,i+1);render();};
$('#f').onchange=applyFilter;$('#fb').onchange=applyFilter;
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;
 if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();i=Math.min(view.length-1,i+1);render();}
 if(e.key==='ArrowLeft'){e.preventDefault();i=Math.max(0,i-1);render();}});
fetch('/api/data').then(r=>r.json()).then(d=>{all=d.rows;
 $('#fb').innerHTML='<option value=all>전체 bag</option>'+d.bags.map(b=>`<option>${b}</option>`).join('');
 const m=d.met;$('#sum').innerHTML=`${m.frames}프레임 · 검출 ${m.det}건 · GT프레임 ${m.gt_frames}: TP${m.TP} FP${m.FP} FN${m.FN}`;
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
