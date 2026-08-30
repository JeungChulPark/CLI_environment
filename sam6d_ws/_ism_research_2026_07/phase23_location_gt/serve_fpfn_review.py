#!/usr/bin/env python3
"""serve_fpfn_review.py — FP/FN 검증 웹 뷰어 (표준 라이브러리만).

그룹에 따라 질문과 선택지가 바뀐다.
  fp_check : "이 프레임에 이 객체가 실제로 있습니까?"   A / B / C / U
  fn_check : "이 객체가 실제로 보입니까? 어느 정도로?"  1 / 2 / 3 / 4 / U
저장: fp_fn_verification_task.csv 의 verdict / notes 열 (원자적 교체).

실행:
  ~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase23_location_gt/serve_fpfn_review.py
  브라우저: http://127.0.0.1:8767
"""
import csv, http.server, json, os, shutil, socketserver, sys, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase23_location_gt")
CSV_PATH = os.path.join(OUT, "fp_fn_verification_task.csv")
BAK = CSV_PATH + ".bak"
PORT = int(os.environ.get("GT_PORT", "8767"))
HOST = os.environ.get("GT_HOST", "0.0.0.0")

SETS = {
    "fp_check": [("A", "A", "없다 → FP 맞음(오검출)"),
                 ("B", "B", "있고, 빨간 박스가 그 위 → GT오류(사실 TP)"),
                 ("C", "C", "있지만 빨간 박스는 딴 곳"),
                 ("U", "U", "불확실")],
    "fn_check": [("1", "1", "온전히 보임(대부분 노출)"),
                 ("2", "2", "일부 보임(절반쯤/부분 가림)"),
                 ("3", "3", "극히 일부(아주 작거나 심한 가림)"),
                 ("4", "4", "안 보인다 → GT오류(사실 TN)"),
                 ("U", "U", "불확실")],
}
QUESTION = {"fp_check": "이 프레임에 이 객체가 <b>실제로 있습니까?</b>",
            "fn_check": "이 객체가 <b>실제로 보입니까? 어느 정도로?</b>"}

if not os.path.isfile(CSV_PATH):
    raise SystemExit(f"[err] {CSV_PATH} 없음 — build_fp_fn_verification_task.py 먼저 실행")
if not os.path.exists(BAK):
    shutil.copy2(CSV_PATH, BAK)
with open(CSV_PATH, newline="", encoding="utf-8") as f:
    rd = csv.DictReader(f); FIELDS = list(rd.fieldnames); ROWS = list(rd)
for r in ROWS:
    r.setdefault("verdict", ""); r.setdefault("notes", "")
print(f"{len(ROWS)}건 적재 (완료 {sum(1 for r in ROWS if r['verdict'].strip())})")


def save():
    tmp = CSV_PATH + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader(); w.writerows(ROWS)
    os.replace(tmp, CSV_PATH)


PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>FP/FN 검증</title>
<style>
:root{--bg:#14161a;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--wn:#ffb44c;--bad:#ff6b6b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#1b1e24;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-radius:4px;padding:2px 7px}
.tag.fp{background:#3a1f1f;border-color:#7a3a3a;color:#ffb3b3}
.tag.fn{background:#3a2f17;border-color:#7a5f2f;color:#ffd9a0}
.bar{height:6px;background:#262b33;border-radius:3px;overflow:hidden;flex:1;min-width:140px}
.bar>i{display:block;height:100%;background:var(--ok);width:0}
main{max-width:1700px;margin:0 auto;padding:12px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:9px;margin-top:12px}
button.c{background:#1f242c;border:2px solid var(--line);color:var(--fg);border-radius:8px;
  padding:13px 10px;cursor:pointer;font-size:14px;text-align:left}
button.c.on{border-color:var(--ok);background:#173324}
button.c small{display:block;color:var(--dim);font-size:12px;margin-top:3px}
.nav{display:flex;gap:8px;align-items:center;margin-top:12px;flex-wrap:wrap}
button.nv{background:#262b33;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:9px 16px;cursor:pointer}
input[type=text]{background:#1f242c;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:8px 10px}
select{background:#1f242c;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px}
.muted{color:var(--dim);font-size:12px}
kbd{font:11px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
.cls{font-size:16px;font-weight:700;color:#ffe08a}
</style>
<header><div class=hrow>
 <b id=q></b>
 <span class=tag id=cid></span><span class=tag id=grp></span>
 <span class=cls id=cls></span><span class=muted id=loc></span><span class=muted id=pos></span>
 <div class=bar><i id=prog></i></div><span id=done>0</span><span class=muted>/<span id=tot>0</span></span>
 <select id=filter>
  <option value=todo selected>미완료만</option>
  <option value=all>전체</option>
  <option value=fp_check>FP 검증 (86)</option>
  <option value=fn_check>FN 검증 (313)</option>
 </select>
</div></header>
<main>
 <img id=img alt="">
 <div class=grid id=grid></div>
 <div class=nav>
  <button class=nv id=prev><kbd>←</kbd> 이전</button>
  <button class=nv id=next>다음 <kbd>Space</kbd></button>
  <input type=text id=note placeholder="메모 (선택)" style="flex:1;min-width:220px">
  <span class=muted id=meta></span>
 </div>
 <div class=muted style="margin-top:8px">
  선택 즉시 저장되고 다음 미완료로 이동합니다. 같은 값을 다시 누르면 해제. <kbd>Space</kbd>=다음 <kbd>←</kbd>=이전
 </div>
</main>
<script>
const SETS=__SETS__, Q=__Q__; let rows=[],view=[],i=0; const $=s=>document.querySelector(s);
const isDone=r=>(r.verdict||'').trim()!=='';
function cur(){return rows[view[i]];}
function render(){
 if(!view.length){$('#cid').textContent='해당 없음';$('#img').removeAttribute('src');$('#grid').innerHTML='';return;}
 if(i<0)i=0; if(i>=view.length)i=view.length-1;
 const r=cur(), set=SETS[r.group]||[];
 $('#q').innerHTML=Q[r.group]||'';
 $('#cid').textContent=r.case_id;
 $('#grp').textContent=r.group; $('#grp').className='tag '+(r.group==='fp_check'?'fp':'fn');
 $('#cls').textContent=r.class_name;
 $('#loc').textContent=`${r.bag_name} f${r.frame_id}`+(r.has_candidate==='1'?'':' · 후보없음');
 $('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${view[i]}`; $('#note').value=r.notes||'';
 $('#meta').textContent=(r.root_cause?`거절사유 ${r.root_cause} · `:'')+`sem ${r.sem} / appe ${r.appe} / hsv ${r.hsv}`;
 $('#grid').innerHTML=set.map(c=>`<button class=c data-v="${c[0]}"><kbd>${c[1]}</kbd> ${c[0]}<small>${c[2]}</small></button>`).join('');
 document.querySelectorAll('button.c').forEach(b=>{
   b.onclick=()=>pick(b.dataset.v);
   b.classList.toggle('on',b.dataset.v===(r.verdict||'').trim().toUpperCase());});
 const d=rows.filter(isDone).length;
 $('#done').textContent=d; $('#tot').textContent=rows.length;
 $('#prog').style.width=(100*d/rows.length)+'%';}
function commit(){const r=cur();
 fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({index:view[i],verdict:r.verdict,notes:r.notes})});}
function pick(v){const r=cur();
 if((r.verdict||'').trim().toUpperCase()===v){r.verdict='';}else{r.verdict=v;}
 commit(); render(); if((r.verdict||'').trim()) setTimeout(()=>step(1),110);}
function step(d){let j=i+d;
 while(j>=0&&j<view.length&&isDone(rows[view[j]])) j+=d;
 if(j<0||j>=view.length) j=Math.max(0,Math.min(view.length-1,i+d)); i=j; render();}
function applyFilter(){const f=$('#filter').value;
 view=rows.map((r,n)=>n).filter(n=>{const r=rows[n];
  if(f==='todo')return !isDone(r); if(f==='all')return true; return r.group===f;});
 i=Math.max(0,view.findIndex(n=>!isDone(rows[n]))); render();}
$('#prev').onclick=()=>step(-1); $('#next').onclick=()=>step(1);
$('#filter').onchange=applyFilter;
$('#note').oninput=e=>{cur().notes=e.target.value; commit();};
addEventListener('keydown',e=>{ if(e.target.tagName==='INPUT')return;
 const r=cur(); if(!r)return; const set=SETS[r.group]||[];
 const k=e.key.toUpperCase();
 const a=set.find(c=>c[1].toUpperCase()===k||c[0].toUpperCase()===k);
 if(a){e.preventDefault();pick(a[0]);return;}
 if(e.key===' '||e.key==='ArrowRight'){e.preventDefault();step(1);}
 if(e.key==='Backspace'||e.key==='ArrowLeft'){e.preventDefault();step(-1);}});
fetch('/api/rows').then(r=>r.json()).then(d=>{rows=d;applyFilter();});
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
            html = (PAGE.replace("__SETS__", json.dumps(SETS, ensure_ascii=False))
                        .replace("__Q__", json.dumps(QUESTION, ensure_ascii=False)))
            return self._s(200, "text/html; charset=utf-8", html.encode("utf-8"))
        if p == "/api/rows":
            return self._s(200, "application/json; charset=utf-8",
                           json.dumps(ROWS, ensure_ascii=False).encode("utf-8"))
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
        for k in ("verdict", "notes"):
            if k in d:
                r[k] = str(d[k])
        save()
        return self._s(200, "application/json",
                       json.dumps({"done": sum(1 for x in ROWS if x["verdict"].strip())}).encode())

    def log_message(self, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        miss = [r["case_id"] for r in ROWS
                if not os.path.isfile(os.path.join(OUT, r["image_path"]))]
        print(f"  행 {len(ROWS)} · 완료 {sum(1 for r in ROWS if r['verdict'].strip())} · 이미지 누락 {len(miss)}")
        raise SystemExit(0 if not miss else 1)
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n  저장 : {CSV_PATH}\n  종료 : Ctrl+C\n")
    with S((HOST, PORT), H) as httpd:
        httpd.serve_forever()
