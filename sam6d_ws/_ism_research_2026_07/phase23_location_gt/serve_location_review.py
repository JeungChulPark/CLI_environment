#!/usr/bin/env python3
"""serve_location_review.py — 위치 검증 웹 뷰어 (표준 라이브러리만).

질문 하나: "빨간 박스가 오른쪽 TARGET template 과 같은 물체 위에 있습니까?"
BBox 를 새로 그리지 않는다. Y / N / U 한 번 클릭(또는 키보드)로 즉시 저장.
저장 위치: location_verification_task.csv 의 box_on_target / notes 열 (원자적 교체).

실행:
  ~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase23_location_gt/serve_location_review.py
  브라우저: http://127.0.0.1:8766   (VS Code 원격이면 포트 8766 자동 포워딩)
"""
import csv, http.server, json, os, shutil, socketserver, sys, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase23_location_gt")
CSV_PATH = os.path.join(OUT, "location_verification_task.csv")
BAK = CSV_PATH + ".bak"
PORT = int(os.environ.get("GT_PORT", "8766"))
HOST = os.environ.get("GT_HOST", "0.0.0.0")

CHOICES = [("Y", "1", "맞다 — 대상 객체 위"),
           ("N", "2", "아니다 — 다른 물체"),
           ("U", "3", "불확실")]

if not os.path.isfile(CSV_PATH):
    raise SystemExit(f"[err] {CSV_PATH} 없음 — build_location_verification_task.py 먼저 실행")
if not os.path.exists(BAK):
    shutil.copy2(CSV_PATH, BAK)
with open(CSV_PATH, newline="", encoding="utf-8") as f:
    rd = csv.DictReader(f); FIELDS = list(rd.fieldnames); ROWS = list(rd)
for r in ROWS:
    r.setdefault("box_on_target", ""); r.setdefault("notes", "")
_done = sum(1 for r in ROWS if r["box_on_target"].strip())
print(f"{len(ROWS)}건 적재 (완료 {_done})")


def save():
    tmp = CSV_PATH + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader(); w.writerows(ROWS)
    os.replace(tmp, CSV_PATH)


PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>위치 검증</title>
<style>
:root{--bg:#14161a;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--acc:#4c9aff;--ok:#3ddc84;--wn:#ffb44c;--bad:#ff6b6b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#1b1e24;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-radius:4px;padding:2px 7px}
.tag.g{background:#17321f;border-color:#2f6b45;color:#9fe6bd}
.bar{height:6px;background:#262b33;border-radius:3px;overflow:hidden;flex:1;min-width:140px}
.bar>i{display:block;height:100%;background:var(--ok);width:0}
main{max-width:1500px;margin:0 auto;padding:12px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;margin-top:12px}
button.c{background:#1f242c;border:2px solid var(--line);color:var(--fg);border-radius:8px;
  padding:14px 10px;cursor:pointer;font-size:15px;text-align:left}
button.c.on[data-v=Y]{border-color:var(--ok);background:#173324}
button.c.on[data-v=N]{border-color:var(--bad);background:#3a1f1f}
button.c.on[data-v=U]{border-color:var(--wn);background:#3a2f17}
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
 <b>빨간 박스가 오른쪽 <span style="color:#9fe6bd">TARGET template</span> 과 같은 물체 위에 있습니까?</b>
 <span class=tag id=cid></span><span class="tag g" id=grp></span>
 <span class=cls id=cls></span><span class=muted id=loc></span><span class=muted id=pos></span>
 <div class=bar><i id=prog></i></div><span id=done>0</span><span class=muted>/<span id=tot>0</span></span>
 <select id=filter>
  <option value=todo selected>미완료만</option>
  <option value=all>전체</option>
  <option value=phase1c_TP>phase1c_TP (TP 신뢰도)</option>
  <option value=rescue_recovered>rescue_recovered (28)</option>
  <option value=rerank_recovered>rerank_recovered (5)</option>
  <option value=N>N 표시한 것만 다시보기</option>
 </select>
</div></header>
<main>
 <img id=img alt="">
 <div class=grid id=grid></div>
 <div class=nav>
  <button class=nv id=prev><kbd>←</kbd> 이전</button>
  <button class=nv id=next>다음 <kbd>Space</kbd></button>
  <input type=text id=note placeholder="메모 (선택) — 예: 갈색 택배박스, 일부만 걸침" style="flex:1;min-width:220px">
  <span class=muted id=meta></span>
 </div>
 <div class=muted style="margin-top:8px">
  단축키 <kbd>1</kbd>/<kbd>Y</kbd>=맞다 · <kbd>2</kbd>/<kbd>N</kbd>=다른물체 · <kbd>3</kbd>/<kbd>U</kbd>=불확실 ·
  <kbd>Space</kbd>=다음 · <kbd>←</kbd>=이전 · 같은 값을 다시 누르면 선택 해제. 클릭 즉시 CSV 저장됩니다.
 </div>
</main>
<script>
const C=__C__; let rows=[],view=[],i=0; const $=s=>document.querySelector(s);
const isDone=r=>(r.box_on_target||'').trim()!=='';
function cur(){return rows[view[i]];}
function build(){
 $('#grid').innerHTML=C.map(c=>`<button class=c data-v="${c[0]}"><kbd>${c[1]}</kbd> ${c[0]} — ${c[2]}</button>`).join('');
 document.querySelectorAll('button.c').forEach(b=>b.onclick=()=>pick(b.dataset.v));}
function render(){
 if(!view.length){$('#cid').textContent='해당 없음';$('#img').removeAttribute('src');return;}
 if(i<0)i=0; if(i>=view.length)i=view.length-1;
 const r=cur();
 $('#cid').textContent=r.case_id; $('#grp').textContent=r.group;
 $('#cls').textContent=r.class_name;
 $('#loc').textContent=`${r.bag_name} f${r.frame_id} · area=${r.bbox_area}`;
 $('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${view[i]}`; $('#note').value=r.notes||'';
 $('#meta').textContent=`sem ${r.sem} / appe ${r.appe} / hsv ${r.hsv}`;
 document.querySelectorAll('button.c').forEach(b=>b.classList.toggle('on',b.dataset.v===(r.box_on_target||'').trim().toUpperCase()));
 const d=rows.filter(isDone).length;
 $('#done').textContent=d; $('#tot').textContent=rows.length;
 $('#prog').style.width=(100*d/rows.length)+'%';}
function commit(){const r=cur();
 fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({index:view[i],box_on_target:r.box_on_target,notes:r.notes})});}
function pick(v){const r=cur();
 if((r.box_on_target||'').trim().toUpperCase()===v){r.box_on_target='';}else{r.box_on_target=v;}
 commit(); render(); if((r.box_on_target||'').trim()) setTimeout(()=>step(1),110);}
function step(d){let j=i+d;
 while(j>=0&&j<view.length&&isDone(rows[view[j]])) j+=d;
 if(j<0||j>=view.length) j=Math.max(0,Math.min(view.length-1,i+d)); i=j; render();}
function applyFilter(){const f=$('#filter').value;
 view=rows.map((r,n)=>n).filter(n=>{const r=rows[n];
  if(f==='todo')return !isDone(r);
  if(f==='all')return true;
  if(f==='N')return (r.box_on_target||'').trim().toUpperCase()==='N';
  return r.group===f;});
 i=Math.max(0,view.findIndex(n=>!isDone(rows[n]))); render();}
$('#prev').onclick=()=>step(-1); $('#next').onclick=()=>step(1);
$('#filter').onchange=applyFilter;
$('#note').oninput=e=>{cur().notes=e.target.value; commit();};
addEventListener('keydown',e=>{ if(e.target.tagName==='INPUT')return;
 const k=e.key.toUpperCase();
 const a=C.find(c=>c[1]===k||c[0]===k);
 if(a){e.preventDefault();pick(a[0]);return;}
 if(e.key===' '||e.key==='ArrowRight'){e.preventDefault();step(1);}
 if(e.key==='Backspace'||e.key==='ArrowLeft'){e.preventDefault();step(-1);}});
build(); fetch('/api/rows').then(r=>r.json()).then(d=>{rows=d;applyFilter();});
</script></html>"""


class H(http.server.BaseHTTPRequestHandler):
    # protocol_version 은 기본값(HTTP/1.0) 유지 — 이전에 정상 동작한 serve_review.py 와 동일.
    # HTTP/1.1 keep-alive 로 바꾸면 브라우저가 연결을 물고 있어 로딩이 멈춰 보일 수 있음.

    def _s(self, code, ct, body):
        self.send_response(code); self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        p = urllib.parse.urlparse(self.path).path
        if p == "/favicon.ico":
            return self._s(204, "image/x-icon", b"")
        if p == "/healthz":
            return self._s(200, "text/plain; charset=utf-8",
                           f"ok rows={len(ROWS)}".encode("utf-8"))
        if p in ("/", "/index.html"):
            html = PAGE.replace("__C__", json.dumps(CHOICES, ensure_ascii=False))
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
        for k in ("box_on_target", "notes"):
            if k in d:
                r[k] = str(d[k])
        save()
        return self._s(200, "application/json",
                       json.dumps({"done": sum(1 for x in ROWS if x["box_on_target"].strip())}).encode())

    def log_message(self, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


def selftest():
    """서버를 띄우지 않고 데이터/이미지 무결성만 점검."""
    miss = [r["case_id"] for r in ROWS
            if not os.path.isfile(r["image_path"] if os.path.isabs(r["image_path"])
                                  else os.path.join(OUT, r["image_path"]))]
    print(f"  행 수        : {len(ROWS)}")
    print(f"  완료         : {sum(1 for r in ROWS if r['box_on_target'].strip())}")
    print(f"  이미지 누락  : {len(miss)}" + (f"  예: {miss[:3]}" if miss else "  (모두 존재)"))
    print(f"  CSV          : {CSV_PATH}")
    print("  -> 이상 없으면 서버를 그냥 실행하세요 (인자 없이).")
    return 0 if not miss else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        raise SystemExit(selftest())
    print(f"\n  주소 : http://127.0.0.1:{PORT}")
    print(f"  (VS Code 원격이면 PORTS 탭에서 {PORT} 자동/수동 포워딩 후 위 주소 열기)")
    print(f"  저장 : {CSV_PATH}")
    print(f"  백업 : {BAK}\n  종료 : Ctrl+C\n")
    try:
        with S((HOST, PORT), H) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        print(f"\n종료. {sum(1 for x in ROWS if x['box_on_target'].strip())}/{len(ROWS)} 저장됨.")
