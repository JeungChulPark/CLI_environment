#!/usr/bin/env python3
"""serve_boxviz.py — 프롬프트 A/B bbox 비교 뷰어 (표준 라이브러리만).

화면: [원본 | A 현행 프롬프트 | B CLIP-v3 프롬프트]
  굵은 실선 = ACCEPT(최종 검출, 클래스명 표시) · 얇고 어두운 선 = 게이트 탈락
필터: 전체 / 결과가 달라진 프레임만 / B에서 잃은 것 / B에서 얻은 것 / bag별
실행: ~/miniconda3/envs/sam_yolo/bin/python .../serve_boxviz.py  → http://127.0.0.1:8771
"""
import csv, http.server, json, os, socketserver, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase27_clip_prompt")
VIZ = os.path.join(OUT, "boxviz")
MAN = os.path.join(OUT, "csv", "box_compare_manifest.csv")
MET = os.path.join(OUT, "metrics", "prompt_v3_ab.json")
PORT = int(os.environ.get("GT_PORT", "8771")); HOST = os.environ.get("GT_HOST", "0.0.0.0")

ROWS = list(csv.DictReader(open(MAN, encoding="utf-8")))
for r in ROWS:
    A = set(filter(None, r["A_accept"].split(";"))); B = set(filter(None, r["B_accept"].split(";")))
    G = set(filter(None, r["gt_visible"].split(";")))
    r["lost"] = ";".join(sorted((A - B) & G))     # B가 잃은 참검출
    r["gained"] = ";".join(sorted((B - A) & G))   # B가 얻은 참검출
    r["fp_lost"] = ";".join(sorted((A - B) - G))  # B가 없앤 오검출
    r["fp_gained"] = ";".join(sorted((B - A) - G))
MET_D = json.load(open(MET)) if os.path.isfile(MET) else {}
BAGS = sorted({r["bag"] for r in ROWS})
print(f"{len(ROWS)}프레임 적재 · 변화 {sum(int(r['changed']) for r in ROWS)}")

PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>프롬프트 A/B bbox 비교</title>
<style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b;--wn:#ffb44c}
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
.lost{color:var(--bad);font-weight:700}.gain{color:var(--ok);font-weight:700}
.note{background:#181c22;border-left:3px solid #4c9aff;padding:8px 12px;margin:8px 0;border-radius:0 6px 6px 0;font-size:13px}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style>
<header><div class=hrow>
 <b>원본 | A 현행 프롬프트 | B CLIP-v3</b>
 <span class=tag id=name></span><span class=muted id=pos></span>
 <span class=muted id=summary></span>
 <select id=f>
  <option value=changed selected>결과 달라진 프레임만</option>
  <option value=lost>B가 잃은 검출(TP↓)</option>
  <option value=gained>B가 얻은 검출(TP↑)</option>
  <option value=fplost>B가 없앤 오검출(FP↓)</option>
  <option value=all>전체</option>
 </select>
 <select id=fb></select>
 <button id=prev>← 이전</button><button id=next>다음 →</button>
</div></header>
<main>
 <div class=note id=diff></div>
 <img id=img alt="">
 <div class=cap><div>원본</div><div>A · 현행 프롬프트</div><div>B · CLIP-v3 프롬프트</div></div>
 <div class=muted style="margin-top:8px">굵은 실선+라벨 = 최종 ACCEPT · 얇고 어두운 선 = 후보였으나 게이트 탈락 · <kbd>←</kbd><kbd>→</kbd> 이동</div>
</main>
<script>
let all=[],view=[],i=0,MET={}; const $=s=>document.querySelector(s);
function cur(){return all[view[i]];}
function render(){
 if(!view.length){$('#name').textContent='해당 없음';$('#img').removeAttribute('src');$('#diff').innerHTML='';return;}
 if(i<0)i=0; if(i>=view.length)i=view.length-1;
 const r=cur();
 $('#name').textContent=r.name; $('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${view[i]}`;
 let h=`<b>GT 보임:</b> ${r.gt_visible||'—'}<br><b>A 검출:</b> ${r.A_accept||'—'}<br><b>B 검출:</b> ${r.B_accept||'—'}`;
 if(r.lost) h+=`<br><span class=lost>B가 놓친 참검출: ${r.lost}</span>`;
 if(r.gained) h+=`<br><span class=gain>B가 새로 찾은 것: ${r.gained}</span>`;
 if(r.fp_lost) h+=`<br><span class=gain>B가 없앤 오검출: ${r.fp_lost}</span>`;
 if(r.fp_gained) h+=`<br><span class=lost>B의 새 오검출: ${r.fp_gained}</span>`;
 $('#diff').innerHTML=h;}
function applyFilter(){const f=$('#f').value,b=$('#fb').value;
 view=all.map((r,n)=>n).filter(n=>{const r=all[n];
  if(b!=='all'&&r.bag!==b)return false;
  if(f==='changed')return r.changed==='1';
  if(f==='lost')return !!r.lost;
  if(f==='gained')return !!r.gained;
  if(f==='fplost')return !!r.fp_lost;
  return true;});
 i=0; render();}
$('#prev').onclick=()=>{i=Math.max(0,i-1);render();};
$('#next').onclick=()=>{i=Math.min(view.length-1,i+1);render();};
$('#f').onchange=applyFilter; $('#fb').onchange=applyFilter;
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;
 if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();i=Math.min(view.length-1,i+1);render();}
 if(e.key==='ArrowLeft'){e.preventDefault();i=Math.max(0,i-1);render();}});
fetch('/api/data').then(r=>r.json()).then(d=>{all=d.rows;MET=d.met;
 $('#fb').innerHTML='<option value=all>전체 bag</option>'+d.bags.map(b=>`<option>${b}</option>`).join('');
 const A=MET.A||{},B=MET.B||{};
 $('#summary').innerHTML=`A: TP ${A.TP} FP ${A.FP} F1 ${A.f1} &nbsp;|&nbsp; B: TP ${B.TP} FP ${B.FP} F1 ${B.f1} `+
  `&nbsp;<span class=lost>ΔTP ${B.TP-A.TP}</span> <span class=gain>ΔFP ${B.FP-A.FP}</span>`;
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
        if p in ("/", "/index.html"):
            return self._s(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
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
