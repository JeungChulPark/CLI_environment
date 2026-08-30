#!/usr/bin/env python3
"""serve_sample.py — 복구 사례 30건 표본 검수 뷰어 (표준 라이브러리만).

박스 하나당 질문 하나: "이 주황 상자 안에 실제로 있는 것은?"
BBox 를 새로 그리지 않는다. 자세한 규칙은 USER_REVIEW_GUIDE.md 참조.
저장은 color_review_queue.csv 의 review_label / reviewed 열에 즉시 반영(원자적 교체).
"""
import csv, http.server, json, os, shutil, socketserver, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "sample_review_queue.csv")
BAK = CSV_PATH + ".bak"
PORT = int(os.environ.get("GT_PORT", "8765"))
HOST = os.environ.get("GT_HOST", "0.0.0.0")

CHOICES = [("correct", "1"), ("partial", "2")]
SPECIAL = [("wrong_object", "W", "초록 상자에 다른 물건이 있음"),
           ("background", "B", "초록 상자가 배경만 잡음"),
           ("unsure", "U", "판단 불가")]
FIELDS = ["case_id", "dataset", "frame_id", "object", "config", "source_prompt", "conf",
          "iou_with_expected", "expected_from_frame_dist", "found_bbox", "image_path",
          "review_label", "reviewed", "note"]

if not os.path.exists(BAK):
    shutil.copy2(CSV_PATH, BAK)
ROWS = list(csv.DictReader(open(CSV_PATH, newline="", encoding="utf-8")))
print(f"{len(ROWS)}건 적재 (완료 {sum(1 for r in ROWS if r['reviewed']=='yes')}, "
      f"우선순위1 {sum(1 for r in ROWS if r['priority']=='1')})")


def save():
    tmp = CSV_PATH + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader(); w.writerows(ROWS)
    os.replace(tmp, CSV_PATH)


PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>색 검증 검수</title>
<style>
:root{--bg:#14161a;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--acc:#4c9aff;--ok:#3ddc84;--wn:#ffb44c}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#1b1e24;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-radius:4px;padding:2px 7px}
.tag.p1{background:#3a2f17;border-color:#6b552f;color:#f0d48f}
.bar{height:6px;background:#262b33;border-radius:3px;overflow:hidden;flex:1;min-width:140px}
.bar>i{display:block;height:100%;background:var(--ok);width:0}
main{max-width:1180px;margin:0 auto;padding:12px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:7px;margin-top:12px}
.sp{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:7px}
button.c{background:#1f242c;border:2px solid var(--line);color:var(--fg);border-radius:8px;
  padding:10px 8px;cursor:pointer;font-size:13px;text-align:left}
button.c.on{border-color:var(--acc);background:#1c2e47}
button.c.sp.on{border-color:var(--wn);background:#3a2f17}
button.c small{display:block;color:var(--dim);font-size:11px}
.nav{display:flex;gap:8px;align-items:center;margin-top:12px;flex-wrap:wrap}
button.nv{background:#262b33;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:9px 16px;cursor:pointer}
input[type=text]{background:#1f242c;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px 9px}
select{background:#1f242c;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px}
.muted{color:var(--dim);font-size:12px}
kbd{font:11px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
.rv{margin-top:10px;font-size:12px;color:var(--dim)}
</style>
<header><div class=hrow>
 <b>초록 상자 안에 target 객체가 실제로 있습니까?</b>
 <span class=tag id=cid></span><span class=tag id=pri></span>
 <span class=muted id=loc></span><span class=muted id=pos></span>
 <div class=bar><i id=prog></i></div><span id=done>0</span><span class=muted>/<span id=tot>0</span></span>
 <select id=filter><option value=all selected>전체 30</option><option value=todo>미완료만</option></select>
</div></header>
<main>
 <img id=img alt="">
 <div class=muted id=rsn style="margin-top:8px"></div>
 <div class=grid id=grid></div>
 <div class=sp id=sp></div>
 <div class=nav>
  <button class=nv id=prev><kbd>←</kbd> 이전</button>
  <button class=nv id=next>다음 <kbd>Space</kbd></button>
  <input type=text id=note placeholder="메모 (선택)" style="flex:1;min-width:160px">
 </div>
 <div class=rv><label><input type=checkbox id=reveal> 모델 판정 보기 (먼저 보면 편향)</label>
  <span id=meta class=muted></span></div>
</main>
<script>
const C=__C__, S=__S__; let rows=[],view=[],i=0; const $=s=>document.querySelector(s);
function cur(){return rows[view[i]];}
function build(){
 $('#grid').innerHTML=C.map(c=>`<button class=c data-v="${c[0]}"><kbd>${c[1]}</kbd> ${c[0]}</button>`).join('');
 $('#sp').innerHTML=S.map(c=>`<button class="c sp" data-v="${c[0]}"><kbd>${c[1]}</kbd> ${c[0]}<small>${c[2]}</small></button>`).join('');
 document.querySelectorAll('button.c').forEach(b=>b.onclick=()=>pick(b.dataset.v));}
function render(){
 if(!view.length){$('#cid').textContent='해당 없음';return;}
 if(i<0)i=0; if(i>=view.length)i=view.length-1;
 const r=cur();
 $('#cid').textContent=r.case_id;
 $('#pri').textContent=r.config; $('#pri').className='tag p1';
 $('#loc').textContent=`${r.dataset} f${r.frame_id} · target=${r.object} · conf ${r.conf} · IoU ${r.iou_with_expected}`;
 $('#pos').textContent=`${i+1} / ${view.length}`;
 $('#rsn').textContent=`복구 설정 ${r.config} · 프롬프트 '${r.source_prompt}' · 기대위치는 ${r.expected_from_frame_dist}프레임 떨어진 곳에서 가져온 근사입니다`;
 $('#img').src=`/img/${view[i]}`; $('#note').value=r.note||'';
 document.querySelectorAll('button.c').forEach(b=>b.classList.toggle('on',b.dataset.v===r.review_label));
 const d=rows.filter(x=>x.reviewed==='yes').length;
 $('#done').textContent=d; $('#tot').textContent=rows.length;
 $('#prog').style.width=(100*d/rows.length)+'%';
 $('#meta').textContent=$('#reveal').checked
   ? ` config=${r.config} conf=${r.conf} iou=${r.iou_with_expected}`:'';}
function commit(){const r=cur();
 fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({index:view[i],review_label:r.review_label,reviewed:r.reviewed,note:r.note})});}
function pick(v){const r=cur();
 if(r.review_label===v){r.review_label='';r.reviewed='no';}else{r.review_label=v;r.reviewed='yes';}
 commit(); render(); if(r.reviewed==='yes') setTimeout(()=>step(1),110);}
function step(d){let j=i+d;
 while(j>=0&&j<view.length&&rows[view[j]].reviewed==='yes') j+=d;
 if(j<0||j>=view.length) j=Math.max(0,Math.min(view.length-1,i+d)); i=j; render();}
function applyFilter(){const f=$('#filter').value;
 view=rows.map((r,n)=>n).filter(n=>{const r=rows[n];
  if(f==='todo')return r.reviewed!=='yes'; return true;});
 i=Math.max(0,view.findIndex(n=>rows[n].reviewed!=='yes')); render();}
$('#prev').onclick=()=>step(-1); $('#next').onclick=()=>step(1);
$('#filter').onchange=applyFilter; $('#reveal').onchange=render;
$('#note').oninput=e=>{cur().note=e.target.value; commit();};
addEventListener('keydown',e=>{ if(e.target.tagName==='INPUT')return;
 const k=e.key.toUpperCase();
 const a=C.find(c=>c[1]===k)||S.find(c=>c[1]===k);
 if(a){e.preventDefault();pick(a[0]);return;}
 if(e.key===' '||e.key==='ArrowRight'){e.preventDefault();step(1);}
 if(e.key==='Backspace'||e.key==='ArrowLeft'){e.preventDefault();step(-1);}});
build(); fetch('/api/rows').then(r=>r.json()).then(d=>{rows=d;applyFilter();});
</script></html>"""


class H(http.server.BaseHTTPRequestHandler):
    def _s(self, code, ct, body):
        self.send_response(code); self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        p = urllib.parse.urlparse(self.path).path
        if p in ("/", "/index.html"):
            html = (PAGE.replace("__C__", json.dumps(CHOICES, ensure_ascii=False))
                        .replace("__S__", json.dumps(SPECIAL, ensure_ascii=False)))
            return self._s(200, "text/html; charset=utf-8", html.encode("utf-8"))
        if p == "/api/rows":
            return self._s(200, "application/json; charset=utf-8",
                           json.dumps(ROWS, ensure_ascii=False).encode("utf-8"))
        if p.startswith("/img/"):
            try:
                fp = ROWS[int(p[5:])]["image_path"]
            except (ValueError, IndexError):
                return self._s(404, "text/plain", b"no")
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
        for k in ("review_label", "reviewed", "note"):
            if k in d:
                r[k] = str(d[k])
        save()
        return self._s(200, "application/json",
                       json.dumps({"done": sum(1 for x in ROWS if x["reviewed"] == "yes")}).encode())

    def log_message(self, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


if __name__ == "__main__":
    import socket
    print(f"\n  주소 : http://127.0.0.1:{PORT}")
    if HOST != "127.0.0.1":
        try:
            print(f"         원격: http://{socket.gethostbyname(socket.gethostname())}:{PORT}")
        except Exception:
            pass
    print(f"  저장 : {CSV_PATH}\n  종료 : Ctrl+C\n")
    try:
        with S((HOST, PORT), H) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        print(f"\n종료. {sum(1 for x in ROWS if x['reviewed']=='yes')}/{len(ROWS)} 저장됨.")
