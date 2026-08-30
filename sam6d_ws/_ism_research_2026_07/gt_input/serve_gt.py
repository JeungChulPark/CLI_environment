#!/usr/bin/env python3
"""serve_gt.py — 가시성 GT 입력 뷰어 (로컬 전용, 표준 라이브러리만 사용).

실행:
    python3 serve_gt.py
    → 브라우저에서  http://127.0.0.1:8765

무엇을 하는가
  · user_visibility_gt.csv 를 읽어 프레임을 한 장씩 보여준다
  · 숫자키/클릭으로 보이는 객체를 토글하면 **즉시 CSV 에 저장**한다
  · 껐다 켜도 이어서 작업할 수 있다 (user_reviewed=yes 인 행은 건너뛰기 가능)
  · 현재 모델 판정은 **기본 숨김** — 먼저 보면 확증 편향이 생겨 측정이 무효가 된다

안전장치
  · 첫 실행 시 user_visibility_gt.csv.bak 백업 생성
  · 저장은 임시파일 → os.replace 원자적 교체 (중간에 죽어도 CSV 가 깨지지 않음)
  · 127.0.0.1 에만 바인딩 (외부 노출 없음)

의존성 없음. numpy/opencv/flask 모두 불필요하다.
"""
import csv, http.server, json, os, shutil, socketserver, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "user_visibility_gt.csv")
BAK_PATH = CSV_PATH + ".bak"
OBJDIR = os.path.join(HERE, "objthumbs")
PORT = int(os.environ.get("GT_PORT", "8765"))
# 기본은 127.0.0.1 (외부 노출 없음).
# VS Code Remote SSH 등에서 포트 포워딩이 안 될 때만 GT_HOST=0.0.0.0 으로 열어 쓴다.
HOST = os.environ.get("GT_HOST", "127.0.0.1")

OBJECTS = [
    ("Bear", "1", "갈색 곰인형"),
    ("Rabbit", "2", "흰색 토끼인형"),
    ("Dinosaur", "3", "연두색 공룡인형"),
    ("milk", "4", "우유팩"),
    ("choco_hazelnut_high", "5", "초코하임 갈색 상자"),
    ("Febreze_high", "6", "페브리즈 파란 분무기"),
    ("Mugcup_high", "7", "분홍 머그컵"),
    ("saffron", "8", "샤프란 세제통"),
    ("Sauce_high", "9", "소스병"),
    ("Sikhye_high", "0", "식혜 금색 캔"),
]
NAMES = [o[0] for o in OBJECTS]
# 사용자 파일은 7열만 유지한다. 모델 판정 등 메타는 frame_meta.csv 로 분리돼 있다
# (같은 파일에 두면 먼저 눈에 들어와 확증 편향이 생긴다 — split_gt_csv.py 참조).
FIELDS = ["priority", "dataset_name", "frame_id", "image_path",
          "visible_objects", "user_reviewed", "notes"]
META_PATH = os.path.join(HERE, "frame_meta.csv")


def load():
    with open(CSV_PATH, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    meta = {}
    if os.path.isfile(META_PATH):
        with open(META_PATH, newline="", encoding="utf-8") as f:
            for m in csv.DictReader(f):
                meta[(m["dataset_name"], m["frame_id"])] = m
    for r in rows:
        m = meta.get((r["dataset_name"], r["frame_id"]), {})
        r["_strata"] = m.get("strata", "")
        r["_current_ism_accepted"] = m.get("current_ism_accepted", "")
        r["_n_yolo_candidate_objects"] = m.get("n_yolo_candidate_objects", "")
    return rows


def save(rows):
    """임시파일에 쓰고 원자적으로 교체한다."""
    tmp = CSV_PATH + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, CSV_PATH)


if not os.path.exists(BAK_PATH):
    shutil.copy2(CSV_PATH, BAK_PATH)
    print(f"백업 생성: {os.path.basename(BAK_PATH)}")

ROWS = load()
print(f"{len(ROWS)} 행 적재  (완료 {sum(1 for r in ROWS if r['user_reviewed'] == 'yes')})")

PAGE = """<!doctype html><html lang=ko><meta charset=utf-8>
<title>가시성 GT 입력</title>
<style>
:root{--bg:#14161a;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--acc:#4c9aff;--ok:#3ddc84;--warn:#ffb44c}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#1b1e24;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:14px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-radius:4px;padding:2px 7px}
.tag.p1{background:#173a2a;border-color:#2f6b4c;color:#8ff0b8}
.tag.p2{background:#3a2f17;border-color:#6b552f;color:#f0d48f}
.bar{height:6px;background:#262b33;border-radius:3px;overflow:hidden;flex:1;min-width:160px}
.bar>i{display:block;height:100%;background:var(--ok);width:0}
main{max-width:1180px;margin:0 auto;padding:12px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
#img.zoom{width:auto;max-width:none}
.grid{display:grid;grid-template-columns:repeat(5,1fr);gap:8px;margin-top:12px}
button.obj{display:flex;align-items:center;gap:8px;background:#1f242c;border:2px solid var(--line);
  color:var(--fg);border-radius:8px;padding:7px 9px;cursor:pointer;text-align:left;font-size:13px}
button.obj:hover{border-color:#3d444f}
button.obj.on{border-color:var(--acc);background:#1c2e47}
button.obj img{width:42px;height:42px;border-radius:5px;object-fit:cover;flex:none}
button.obj .k{font:11px ui-monospace,monospace;color:var(--dim);border:1px solid var(--line);
  border-radius:3px;padding:0 4px;flex:none}
button.obj .nm{font-weight:600;line-height:1.15}
button.obj .ds{font-size:11px;color:var(--dim)}
.sp{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px}
button.spc{background:#1f242c;border:2px solid var(--line);color:var(--fg);border-radius:8px;
  padding:11px;cursor:pointer;font-size:14px;font-weight:600}
button.spc.on{border-color:var(--warn);background:#3a2f17}
.nav{display:flex;gap:8px;align-items:center;margin-top:12px;flex-wrap:wrap}
button.nv{background:#262b33;border:1px solid var(--line);color:var(--fg);border-radius:6px;
  padding:9px 16px;cursor:pointer;font-size:14px}
button.nv.pri{background:var(--acc);border-color:var(--acc);color:#0b1220;font-weight:700}
select,input[type=text]{background:#1f242c;color:var(--fg);border:1px solid var(--line);
  border-radius:6px;padding:7px 9px;font-size:13px}
.sel{margin-top:10px;padding:9px 11px;background:#1b1e24;border:1px solid var(--line);border-radius:6px;
  font:13px ui-monospace,monospace;min-height:38px}
.sel b{color:var(--ok)}
.muted{color:var(--dim);font-size:12px}
.reveal{margin-top:10px;font-size:12px;color:var(--dim)}
.reveal code{background:#262b33;padding:1px 5px;border-radius:3px}
kbd{font:11px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);
  border-bottom-width:2px;border-radius:3px;padding:1px 5px}
#done{color:var(--ok);font-weight:700}
</style>
<header>
  <div class=hrow>
    <b id=loc>—</b>
    <span class=tag id=strata></span>
    <span class=muted id=pos></span>
    <div class=bar><i id=prog></i></div>
    <span id=done>0</span><span class=muted>/<span id=tot>0</span> 완료</span>
    <select id=filter>
      <option value=all>전체 339</option>
      <option value=p1 selected>우선순위 1 — 균등층 (107)</option>
      <option value=p12>우선순위 1+2 (170)</option>
      <option value=todo>미완료만</option>
    </select>
    <label class=muted><input type=checkbox id=skip checked> 완료 건너뛰기</label>
    <label class=muted><input type=checkbox id=zoom> 원본 크기</label>
  </div>
</header>
<main>
  <img id=img alt="">
  <div class=sel id=sel></div>
  <div class=grid id=grid></div>
  <div class=sp>
    <button class=spc id=bnone><kbd>N</kbd> &nbsp;none — 객체 없음</button>
    <button class=spc id=bunsure><kbd>U</kbd> &nbsp;unsure — 판단 불가</button>
  </div>
  <div class=nav>
    <button class=nv id=prev><kbd>←</kbd> 이전</button>
    <button class="nv pri" id=next>다음 <kbd>Space</kbd></button>
    <input type=text id=notes placeholder="메모 (선택)" style="flex:1;min-width:180px">
    <span class=muted><kbd>1</kbd>–<kbd>0</kbd> 객체 토글 · <kbd>Esc</kbd> 전체 해제</span>
  </div>
  <div class=reveal>
    <label><input type=checkbox id=reveal> 현재 모델 판정 보기</label>
    <span id=ism class=muted></span>
    <div class=muted style="margin-top:4px">
      ⚠ 균등층(우선순위 1)은 통계의 분모입니다. 모델 답을 먼저 보고 판정하면 측정이 무효가 됩니다.
    </div>
  </div>
</main>
<script>
const OBJ=__OBJECTS__;
let rows=[],view=[],i=0;
const $=s=>document.querySelector(s);

function buildGrid(){
  $('#grid').innerHTML=OBJ.map((o,n)=>
    `<button class=obj data-n="${o[0]}"><span class=k>${o[1]}</span>
     <img src="/obj/${encodeURIComponent(o[0])}" alt="">
     <span><span class=nm>${o[0]}</span><br><span class=ds>${o[2]}</span></span></button>`).join('');
  document.querySelectorAll('button.obj').forEach(b=>
    b.onclick=()=>toggle(b.dataset.n));
}
function cur(){return rows[view[i]];}
function parseSel(r){const v=(r.visible_objects||'').trim();
  return v===''?[]:v.split(';').map(s=>s.trim()).filter(Boolean);}

function render(){
  if(!view.length){$('#loc').textContent='해당 조건의 프레임이 없습니다';$('#img').src='';return;}
  if(i<0)i=0; if(i>=view.length)i=view.length-1;
  const r=cur(), s=parseSel(r);
  $('#loc').textContent=`${r.dataset_name}  f${r.frame_id}`;
  const st=$('#strata'); st.textContent=`P${r.priority}·${r._strata}`;
  st.className='tag'+(r.priority==='1'?' p1':r.priority==='2'?' p2':'');
  $('#pos').textContent=`${i+1} / ${view.length}`;
  $('#img').src=`/img/${view[i]}`;
  $('#notes').value=r.notes||'';
  document.querySelectorAll('button.obj').forEach(b=>
    b.classList.toggle('on',s.includes(b.dataset.n)));
  $('#bnone').classList.toggle('on',s.includes('none'));
  $('#bunsure').classList.toggle('on',s.includes('unsure'));
  $('#sel').innerHTML = s.length? '선택: <b>'+s.join('</b>, <b>')+'</b>'
    : '<span class=muted>선택 없음 — 객체가 안 보이면 <b>N</b>(none), 판단이 어려우면 <b>U</b>(unsure)</span>';
  const d=rows.filter(x=>x.user_reviewed==='yes').length;
  $('#done').textContent=d; $('#tot').textContent=rows.length;
  $('#prog').style.width=(100*d/rows.length)+'%';
  $('#ism').textContent=$('#reveal').checked
    ? ` → 모델: ${r._current_ism_accepted}  (후보 보유 객체 ${r._n_yolo_candidate_objects}개)` : '';
}
function commit(){
  const r=cur();
  fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({index:view[i],visible_objects:r.visible_objects,
      user_reviewed:r.user_reviewed,notes:r.notes})});
}
function toggle(n){
  const r=cur(); let s=parseSel(r);
  s=s.filter(x=>x!=='none'&&x!=='unsure');
  s = s.includes(n) ? s.filter(x=>x!==n) : s.concat([n]);
  s.sort((a,b)=>OBJ.findIndex(o=>o[0]===a)-OBJ.findIndex(o=>o[0]===b));
  r.visible_objects=s.join(';'); r.user_reviewed='yes';
  commit(); render();
}
function special(n){
  const r=cur(); const s=parseSel(r);
  r.visible_objects = s.length===1&&s[0]===n ? '' : n;
  r.user_reviewed = r.visible_objects===''?'no':'yes';
  commit(); render();
}
function clearAll(){const r=cur(); r.visible_objects=''; r.user_reviewed='no'; commit(); render();}
function step(d){
  const skip=$('#skip').checked;
  let j=i+d;
  while(skip && j>=0 && j<view.length && rows[view[j]].user_reviewed==='yes') j+=d;
  if(j<0||j>=view.length){
    // 건너뛰다 끝에 닿으면 건너뛰기 없이 한 칸만 이동
    j=Math.max(0,Math.min(view.length-1,i+d));
  }
  i=j; render();
}
function applyFilter(){
  const f=$('#filter').value;
  view=rows.map((r,n)=>n).filter(n=>{
    const r=rows[n];
    if(f==='p1')return r.priority==='1';
    if(f==='p12')return r.priority==='1'||r.priority==='2';
    if(f==='todo')return r.user_reviewed!=='yes';
    return true;});
  i=0;
  const k=view.findIndex(n=>rows[n].user_reviewed!=='yes');
  if(k>=0) i=k;                       // 미완료 첫 항목에서 시작
  render();
}
$('#prev').onclick=()=>step(-1);
$('#next').onclick=()=>step(1);
$('#bnone').onclick=()=>special('none');
$('#bunsure').onclick=()=>special('unsure');
$('#filter').onchange=applyFilter;
$('#skip').onchange=()=>{};
$('#zoom').onchange=e=>$('#img').classList.toggle('zoom',e.target.checked);
$('#reveal').onchange=render;
$('#notes').oninput=e=>{const r=cur(); r.notes=e.target.value; commit();};
addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT'&&e.target.type==='text')return;
  const k=e.key;
  const hit=OBJ.find(o=>o[1]===k);
  if(hit){e.preventDefault();toggle(hit[0]);return;}
  if(k==='n'||k==='N'){e.preventDefault();special('none');}
  else if(k==='u'||k==='U'){e.preventDefault();special('unsure');}
  else if(k===' '||k==='ArrowRight'){e.preventDefault();step(1);}
  else if(k==='Backspace'||k==='ArrowLeft'){e.preventDefault();step(-1);}
  else if(k==='Escape'){e.preventDefault();clearAll();}
});
buildGrid();
fetch('/api/rows').then(r=>r.json()).then(d=>{rows=d;applyFilter();});
</script></html>"""


class H(http.server.BaseHTTPRequestHandler):
    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = urllib.parse.urlparse(self.path).path
        if p in ("/", "/index.html"):
            html = PAGE.replace("__OBJECTS__", json.dumps(OBJECTS, ensure_ascii=False))
            return self._send(200, "text/html; charset=utf-8", html.encode("utf-8"))
        if p == "/api/rows":
            return self._send(200, "application/json; charset=utf-8",
                              json.dumps(ROWS, ensure_ascii=False).encode("utf-8"))
        if p.startswith("/img/"):
            try:
                idx = int(p[5:])
                fp = ROWS[idx]["image_path"]
            except (ValueError, IndexError, KeyError):
                return self._send(404, "text/plain", b"no")
            if not os.path.isfile(fp):
                return self._send(404, "text/plain", b"missing")
            with open(fp, "rb") as f:
                return self._send(200, "image/png", f.read())
        if p.startswith("/obj/"):
            name = urllib.parse.unquote(p[5:])
            if name not in NAMES:                      # 경로 조작 방지
                return self._send(404, "text/plain", b"no")
            fp = os.path.join(OBJDIR, name + ".png")
            if not os.path.isfile(fp):
                return self._send(404, "text/plain", b"missing")
            with open(fp, "rb") as f:
                return self._send(200, "image/png", f.read())
        return self._send(404, "text/plain", b"no")

    def do_POST(self):
        if urllib.parse.urlparse(self.path).path != "/api/save":
            return self._send(404, "text/plain", b"no")
        n = int(self.headers.get("Content-Length", "0"))
        try:
            d = json.loads(self.rfile.read(n) or b"{}")
            r = ROWS[int(d["index"])]
        except Exception:
            return self._send(400, "text/plain", b"bad")
        for k in ("visible_objects", "user_reviewed", "notes"):
            if k in d:
                r[k] = str(d[k])
        save(ROWS)
        done = sum(1 for x in ROWS if x["user_reviewed"] == "yes")
        return self._send(200, "application/json", json.dumps({"done": done}).encode())

    def log_message(self, *a):
        pass                                            # 요청 로그 억제


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    print(f"\n  주소   : http://127.0.0.1:{PORT}")
    if HOST != "127.0.0.1":
        import socket
        try:
            print(f"           원격 접속용: http://{socket.gethostbyname(socket.gethostname())}:{PORT}")
        except Exception:
            pass
        print(f"  ⚠ {HOST} 로 열려 있어 같은 네트워크의 다른 PC 에서도 접근할 수 있습니다.")
    print(f"  저장   : {CSV_PATH}")
    print(f"  백업   : {BAK_PATH}")
    print("  종료   : Ctrl+C\n")
    try:
        with S((HOST, PORT), H) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        d = sum(1 for x in ROWS if x["user_reviewed"] == "yes")
        print(f"\n종료. 완료 {d}/{len(ROWS)} 행이 CSV 에 저장돼 있습니다.")
