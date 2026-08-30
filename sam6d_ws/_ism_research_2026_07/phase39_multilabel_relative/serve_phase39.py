#!/usr/bin/env python3
"""serve_phase39.py — 다중라벨+상대판정+색결선으로 달라진 프레임 검수 뷰어 → http://127.0.0.1:8779

왼쪽 = 현행 배포, 오른쪽 = 신규(①다중 라벨 ②상대 판정 ③색 결선 τ0.60).
초록 = GT 가시 목록에 있는 라벨(TP), 빨강 = 없는 라벨(FP).

⚠ 이 검수의 목적: TP/FP 지표는 프레임 단위 '보였나'만 보고 **박스가 그 물체 위인지는 보지
않는다**. 새로 얻은 TP 가 진짜 그 물체 위인지 눈으로 확인하는 것이 이 페이지의 일이다.
"""
import csv, http.server, json, os, socketserver, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase39_multilabel_relative")
VIZ = os.path.join(OUT, "viz")
ROWS = list(csv.DictReader(open(os.path.join(OUT, "csv", "phase39_changed.csv"), encoding="utf-8")))
MET = json.load(open(os.path.join(OUT, "metrics", "phase39.json")))
for n, r in enumerate(ROWS):
    r["idx"] = n
BAGS = sorted({r["bag"] for r in ROWS})
PORT = int(os.environ.get("GT_PORT", "8779"))
print(f"{len(ROWS)}개 변화 프레임 적재")

PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>Phase39 검수 — 다중라벨+상대판정+색결선</title><style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b;--acc:#4c9aff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.55 -apple-system,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#171a1f;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
select,button{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px 12px;cursor:pointer}
button:hover,select:hover{border-color:var(--acc)}
main{padding:10px 14px}#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.cap{display:flex;margin-top:6px;color:var(--dim);font-size:12px;text-align:center}.cap>div{flex:1}
.muted{color:var(--dim);font-size:12px}
.note{background:#181c22;border-left:3px solid var(--acc);padding:8px 12px;margin:8px 0;border-radius:0 6px 6px 0;font-size:13px}
.g{color:var(--ok);font-weight:700}.b{color:var(--bad);font-weight:700}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style><header><div class=hrow>
<b>왼쪽 현행 &nbsp;|&nbsp; 오른쪽 신규(①다중라벨 ②상대판정 ③색결선 τ0.60)</b>
<span class=tag id=name></span><span class=muted id=pos></span><span class=muted id=sum></span>
<select id=f>
<option value=all selected>변화 전체</option>
<option value=gain>새로 찾은 것(TP↑)</option>
<option value=newfp>새 오검출(FP↑)</option>
<option value=rmfp>없어진 오검출(FP↓)</option>
<option value=lost>놓치게 된 것(TP↓)</option></select>
<select id=fb></select><button id=prev>← 이전</button><button id=next>다음 →</button></div></header>
<main><div class=note id=diff></div><img id=img alt="">
<div class=cap><div>현행 배포</div><div>신규 (①+②+③)</div></div>
<div class=muted style="margin-top:8px"><span class=g>초록 = GT 가시 목록에 있는 라벨</span> · <span class=b>빨강 = 없는 라벨</span> ·
같은 좌표 박스는 안쪽으로 들여 그림 · <kbd>←</kbd><kbd>→</kbd> 로 이동 ·
<b>보는 요령</b> 초록 박스가 정말 그 물체를 감싸고 있는지 확인 — 지표는 위치를 보지 않는다</div></main>
<script>
let all=[],view=[],i=0;const $=s=>document.querySelector(s);
function render(){if(!view.length){$('#name').textContent='해당 없음';$('#img').removeAttribute('src');$('#diff').innerHTML='';return;}
 if(i<0)i=0;if(i>=view.length)i=view.length-1;const r=all[view[i]];
 $('#name').textContent=`${r.bag} f${r.frame}`;$('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${r.idx}`;
 let h=`<b>GT 보임:</b> ${r.gt_visible||'—'}`;
 if(r.gain_TP)h+=`<br><span class=g>새로 찾음: ${r.gain_TP}</span>`;
 if(r.removed_FP)h+=`<br><span class=g>없어진 오검출: ${r.removed_FP}</span>`;
 if(r.new_FP)h+=`<br><span class=b>새 오검출: ${r.new_FP}</span>`;
 if(r.lost_TP)h+=`<br><span class=b>놓치게 됨: ${r.lost_TP}</span>`;
 $('#diff').innerHTML=h;}
function applyFilter(){const f=$('#f').value,b=$('#fb').value;
 view=all.map((r,n)=>n).filter(n=>{const r=all[n];if(b!=='all'&&r.bag!==b)return false;
  if(f==='gain')return !!r.gain_TP;if(f==='newfp')return !!r.new_FP;
  if(f==='rmfp')return !!r.removed_FP;if(f==='lost')return !!r.lost_TP;return true;});
 i=0;render();}
$('#prev').onclick=()=>{i=Math.max(0,i-1);render();};$('#next').onclick=()=>{i=Math.min(view.length-1,i+1);render();};
$('#f').onchange=applyFilter;$('#fb').onchange=applyFilter;
addEventListener('keydown',e=>{if(e.target.tagName==='SELECT')return;
 if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();i=Math.min(view.length-1,i+1);render();}
 if(e.key==='ArrowLeft'){e.preventDefault();i=Math.max(0,i-1);render();}});
fetch('/api/data').then(r=>r.json()).then(d=>{all=d.rows;
 $('#fb').innerHTML='<option value=all>전체 bag</option>'+d.bags.map(b=>`<option>${b}</option>`).join('');
 const m=d.met;$('#sum').innerHTML=`현행 TP${m.BASE.TP}/FP${m.BASE.FP} F1 ${m.BASE.f1} &nbsp;→&nbsp; `+
   `<b>신규 TP${m.ML_REL_D.TP}/FP${m.ML_REL_D.FP} F1 ${m.ML_REL_D.f1}</b>`;
 applyFilter();});
</script></html>"""


class H(http.server.BaseHTTPRequestHandler):
    def _s(s, c, ct, b):
        s.send_response(c); s.send_header("Content-Type", ct)
        s.send_header("Content-Length", str(len(b)))
        s.send_header("Cache-Control", "no-store"); s.end_headers(); s.wfile.write(b)

    def do_GET(s):
        p = urllib.parse.urlparse(s.path).path
        if p == "/favicon.ico":
            return s._s(204, "image/x-icon", b"")
        if p == "/healthz":
            return s._s(200, "text/plain", f"ok rows={len(ROWS)}".encode())
        if p in ("/", "/index.html"):
            return s._s(200, "text/html; charset=utf-8", PAGE.encode())
        if p == "/api/data":
            return s._s(200, "application/json; charset=utf-8",
                        json.dumps({"rows": ROWS, "bags": BAGS, "met": MET},
                                   ensure_ascii=False).encode())
        if p.startswith("/img/"):
            try:
                fp = os.path.join(VIZ, ROWS[int(p[5:])]["image"])
            except Exception:
                return s._s(404, "text/plain", b"no")
            if not os.path.isfile(fp):
                return s._s(404, "text/plain", b"missing")
            return s._s(200, "image/png", open(fp, "rb").read())
        return s._s(404, "text/plain", b"no")

    def log_message(s, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n")
    with S(("0.0.0.0", PORT), H) as h:
        h.serve_forever()
