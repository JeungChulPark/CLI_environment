#!/usr/bin/env python3
"""serve_yoloe_compare.py — YOLO-World vs YOLOE 비교 뷰어 (표준 라이브러리만).

패널: [원본 | YOLO-World | YOLOE multi-K | YOLOE montage]
필터: 전체 / YOLO-World만 찾은 것 / YOLOE만 찾은 것 / 둘 다 놓친 것 / bag별
실행 → http://127.0.0.1:8772
"""
import csv, http.server, json, os, socketserver, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase28_yoloe")
VIZ = os.path.join(OUT, "viz")
MAN = os.path.join(OUT, "csv", "yoloe_compare_manifest.csv")
MET = os.path.join(OUT, "metrics", "yoloe_compare.json")
PORT = int(os.environ.get("GT_PORT", "8772")); HOST = os.environ.get("GT_HOST", "0.0.0.0")

ROWS = list(csv.DictReader(open(MAN, encoding="utf-8"))) if os.path.isfile(MAN) else []
for r in ROWS:
    G = set(filter(None, r["gt_visible"].split(";")))
    W = set(filter(None, r["W_found"].split(";"))) & G
    K = set(filter(None, r["K_found"].split(";"))) & G
    M = set(filter(None, r["M_found"].split(";"))) & G
    r["only_W"] = ";".join(sorted(W - (K | M)))
    r["only_E"] = ";".join(sorted((K | M) - W))
    r["none"] = ";".join(sorted(G - (W | K | M)))
MET_D = json.load(open(MET)) if os.path.isfile(MET) else {}
BAGS = sorted({r["bag"] for r in ROWS})
print(f"{len(ROWS)}프레임 적재")

PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>YOLO-World vs YOLOE</title>
<style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#171a1f;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
select,button{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px 12px;cursor:pointer}
main{padding:10px 14px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.cap{display:flex;margin-top:6px;color:var(--dim);font-size:12px;text-align:center}
.cap>div{flex:1}
.muted{color:var(--dim);font-size:12px}
.note{background:#181c22;border-left:3px solid #4c9aff;padding:8px 12px;margin:8px 0;border-radius:0 6px 6px 0;font-size:13px}
.w{color:#8cf;font-weight:700}.e{color:var(--ok);font-weight:700}.n{color:var(--bad);font-weight:700}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style>
<header><div class=hrow>
 <b>원본 | YOLO-World | YOLOE multi-K | YOLOE montage</b>
 <span class=tag id=name></span><span class=muted id=pos></span><span class=muted id=sum></span>
 <select id=f>
  <option value=all selected>전체</option>
  <option value=onlyW>YOLO-World만 찾음</option>
  <option value=onlyE>YOLOE만 찾음</option>
  <option value=none>셋 다 놓침</option>
 </select>
 <select id=fb></select>
 <button id=prev>← 이전</button><button id=next>다음 →</button>
</div></header>
<main>
 <div class=note id=diff></div>
 <img id=img alt="">
 <div class=cap><div>원본</div><div>YOLO-World (텍스트)</div><div>YOLOE multi-K (뷰별 슬롯)</div><div>YOLOE montage (뷰 통합)</div></div>
 <div class=muted style="margin-top:8px">박스=검출 후보(게이트 이전) · <kbd>←</kbd><kbd>→</kbd> 이동</div>
</main>
<script>
let all=[],view=[],i=0; const $=s=>document.querySelector(s);
function cur(){return all[view[i]];}
function render(){
 if(!view.length){$('#name').textContent='해당 없음';$('#img').removeAttribute('src');$('#diff').innerHTML='';return;}
 if(i<0)i=0; if(i>=view.length)i=view.length-1;
 const r=cur(); $('#name').textContent=r.name; $('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${view[i]}`;
 let h=`<b>GT 보임:</b> ${r.gt_visible||'—'}<br>`+
  `<span class=w>YOLO-World:</span> ${r.W_found||'—'}<br>`+
  `<span class=e>YOLOE multi-K:</span> ${r.K_found||'—'}<br>`+
  `<span class=e>YOLOE montage:</span> ${r.M_found||'—'}`;
 if(r.only_W) h+=`<br><span class=w>YOLO-World만 찾음: ${r.only_W}</span>`;
 if(r.only_E) h+=`<br><span class=e>YOLOE만 찾음: ${r.only_E}</span>`;
 if(r.none) h+=`<br><span class=n>셋 다 놓침: ${r.none}</span>`;
 $('#diff').innerHTML=h;}
function applyFilter(){const f=$('#f').value,b=$('#fb').value;
 view=all.map((r,n)=>n).filter(n=>{const r=all[n];
  if(b!=='all'&&r.bag!==b)return false;
  if(f==='onlyW')return !!r.only_W; if(f==='onlyE')return !!r.only_E;
  if(f==='none')return !!r.none; return true;});
 i=0;render();}
$('#prev').onclick=()=>{i=Math.max(0,i-1);render();};
$('#next').onclick=()=>{i=Math.min(view.length-1,i+1);render();};
$('#f').onchange=applyFilter; $('#fb').onchange=applyFilter;
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;
 if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();i=Math.min(view.length-1,i+1);render();}
 if(e.key==='ArrowLeft'){e.preventDefault();i=Math.max(0,i-1);render();}});
fetch('/api/data').then(r=>r.json()).then(d=>{all=d.rows;
 $('#fb').innerHTML='<option value=all>전체 bag</option>'+d.bags.map(b=>`<option>${b}</option>`).join('');
 const m=d.met.recall||{};
 $('#sum').innerHTML=`박스-존재 recall &nbsp; <span class=w>YW ${m.W}</span> &nbsp; <span class=e>multiK ${m.K}</span> &nbsp; <span class=e>montage ${m.M}</span>`;
 applyFilter();});
</script></html>"""


class H(http.server.BaseHTTPRequestHandler):
    def _s(self, c, ct, b):
        self.send_response(c); self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        p = urllib.parse.urlparse(self.path).path
        if p == "/favicon.ico": return self._s(204, "image/x-icon", b"")
        if p == "/healthz": return self._s(200, "text/plain", f"ok rows={len(ROWS)}".encode())
        if p in ("/", "/index.html"): return self._s(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        if p == "/api/data":
            return self._s(200, "application/json; charset=utf-8",
                json.dumps({"rows": ROWS, "bags": BAGS, "met": MET_D}, ensure_ascii=False).encode())
        if p.startswith("/img/"):
            try: fp = os.path.join(VIZ, ROWS[int(p[5:])]["image"])
            except (ValueError, IndexError): return self._s(404, "text/plain", b"no")
            if not os.path.isfile(fp): return self._s(404, "text/plain", b"missing")
            with open(fp, "rb") as f: return self._s(200, "image/png", f.read())
        return self._s(404, "text/plain", b"no")

    def log_message(self, *a): pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


if __name__ == "__main__":
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n  종료 : Ctrl+C\n")
    with S((HOST, PORT), H) as httpd:
        httpd.serve_forever()
