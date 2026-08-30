#!/usr/bin/env python3
"""serve_triage.py — 최종 FN 사례가 '정말 검출 가능했는가' 를 판정하는 뷰어 (표준 라이브러리만).

왜 필요한가
  가시성 GT 작성자(ldh)의 진술: "대부분 프레임에서 객체의 정말 미세한 일부만 보였고,
  탐지하기 힘든 부분은 분별력이 없는 경우가 있을 것."
  → 지금의 recall(64.3%)은 '아주 조금만 보인 경우'까지 분모에 넣은 **하한**이다.
  → FN 사례만 다시 보고 "검출될 만큼 충분히 보였나" 를 답하면 하한이 좁혀진다.

무엇을 묻는가 (한 프레임당 한 번)
  D  잘 보임 — 검출됐어야 함        → 진짜 실패 (모델 책임)
  B  아주 일부만 보임 — 검출 어려움  → 분모에서 제외 가능 (난이도 문제)
  N  사실 안 보임 — 라벨 오류        → GT 수정 (내가 잘못 적었음)
  S  판단 보류

우선순위
  P1 proposal miss (후보조차 없었음)  ← 가장 값어치 큼. 이것만 답해도 핵심 질문이 풀린다.
  P2 후보는 있었으나 선택/게이트 탈락

화면에는 그 프레임의 **해당 객체 후보 박스**를 겹쳐 그려 판단을 돕는다.
후보가 없으면(P1) 박스가 없으므로 "이 위치에 있었어야 한다" 를 눈으로 찾아야 한다.

실행:  python3 serve_triage.py    (기본 포트 8766, GT_HOST=0.0.0.0 지원)
저장:  triage_answers.csv 에 즉시 기록 (원자적 교체). 껐다 켜도 이어서 진행.
"""
import csv, http.server, io, json, os, socketserver, struct, urllib.parse, zlib

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
FUS = os.path.join(RSRCH, "ism_fusion_research")
ANS_PATH = os.path.join(HERE, "triage_answers.csv")
PORT = int(os.environ.get("GT_PORT", "8766"))
HOST = os.environ.get("GT_HOST", "0.0.0.0")

FIELDS = ["dataset", "frame_id", "object", "stage", "priority",
          "verdict", "note"]
VERDICTS = [("D", "잘 보임 — 검출됐어야 함", "#3ddc84"),
            ("B", "아주 일부만 보임 — 검출 어려움", "#ffb44c"),
            ("N", "사실 안 보임 — 내 라벨 오류", "#ff6b6b"),
            ("S", "판단 보류", "#9aa0a6")]

# ---------------------------------------------------------------- 대상 적재
src = os.path.join(FUS, "results", "gt_triage_queue.csv")
QUEUE = []
for r in csv.DictReader(open(src, encoding="utf-8")):
    QUEUE.append({"dataset": r["dataset"], "frame_id": int(r["frame_id"]),
                  "object": r["object"], "stage": r["stage"],
                  "image_path": r["image_path"],
                  "priority": 1 if r["stage"] != "Y5_select_or_gate" else 2,
                  "raw_max_conf": r.get("raw_max_conf", ""),
                  "post_max_conf": r.get("post_max_conf", "")})
QUEUE.sort(key=lambda x: (x["priority"], x["dataset"], x["frame_id"], x["object"]))

# 후보 박스 좌표 (해당 객체의 운영 후보)
BOXES = {}
for ds in sorted({q["dataset"] for q in QUEUE}):
    coord = {}
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
        coord[r["uid"]] = (int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        BOXES.setdefault((ds, int(r["frame_id"]), r["object"]), []).append(
            {"box": coord[r["uid"]], "conf": float(r["yolo_conf"]),
             "sem": float(r["sem_top5"]), "appe": float(r["appe11_clstop1"])})

# 기존 답변 이어받기
ANS = {}
if os.path.isfile(ANS_PATH):
    for r in csv.DictReader(open(ANS_PATH, encoding="utf-8")):
        ANS[(r["dataset"], int(r["frame_id"]), r["object"])] = r
for q in QUEUE:
    k = (q["dataset"], q["frame_id"], q["object"])
    a = ANS.get(k, {})
    q["verdict"] = a.get("verdict", "")
    q["note"] = a.get("note", "")
print(f"triage 대상 {len(QUEUE)}  (P1 proposal miss "
      f"{sum(1 for q in QUEUE if q['priority']==1)} / P2 선택·게이트 "
      f"{sum(1 for q in QUEUE if q['priority']==2)})  이미 답변 {sum(1 for q in QUEUE if q['verdict'])}")


def save():
    tmp = ANS_PATH + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        w.writerows([q for q in QUEUE if q["verdict"]])
    os.replace(tmp, ANS_PATH)


# ---------------------------------------------------------------- PNG 위에 박스 그리기
def draw_boxes_png(path, boxes):
    """PNG 를 디코드하지 않고 표시하기 위해, 박스는 브라우저(CSS)에서 그린다.
    여기서는 원본 크기만 알려주면 된다."""
    with open(path, "rb") as f:
        head = f.read(33)
    if head[:8] != b"\x89PNG\r\n\x1a\n":
        return None, None
    w, h = struct.unpack(">II", head[16:24])
    return w, h


PAGE = """<!doctype html><html lang=ko><meta charset=utf-8>
<title>FN 재검수 (triage)</title>
<style>
:root{--bg:#14161a;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--acc:#4c9aff;--ok:#3ddc84}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#1b1e24;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:14px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-radius:4px;padding:2px 7px}
.tag.p1{background:#3a2f17;border-color:#6b552f;color:#f0d48f}
.obj{font-size:19px;font-weight:800;color:var(--acc)}
.bar{height:6px;background:#262b33;border-radius:3px;overflow:hidden;flex:1;min-width:140px}
.bar>i{display:block;height:100%;background:var(--ok);width:0}
main{max-width:1180px;margin:0 auto;padding:12px}
.wrap{position:relative;display:inline-block;width:100%}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.bx{position:absolute;border:2px solid #ff4d4f;box-shadow:0 0 0 1px #000 inset}
.bx>span{position:absolute;top:-19px;left:-2px;background:#ff4d4f;color:#000;font:11px ui-monospace,monospace;padding:0 4px;white-space:nowrap}
.vs{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px}
button.v{background:#1f242c;border:2px solid var(--line);color:var(--fg);border-radius:8px;
  padding:14px 10px;cursor:pointer;font-size:14px;font-weight:600;text-align:left}
button.v small{display:block;font-weight:400;color:var(--dim);font-size:11px;margin-top:3px}
button.v.on{background:#1c2e47}
.nav{display:flex;gap:8px;align-items:center;margin-top:12px;flex-wrap:wrap}
button.nv{background:#262b33;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:9px 16px;cursor:pointer}
select,input[type=text]{background:#1f242c;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:7px 9px;font-size:13px}
.muted{color:var(--dim);font-size:12px}
kbd{font:11px ui-monospace,monospace;background:#262b33;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
.hint{margin-top:8px;padding:8px 11px;background:#1b1e24;border:1px solid var(--line);border-radius:6px;font-size:12px;color:var(--dim)}
</style>
<header><div class=hrow>
  <span class=muted>이 프레임에서</span> <span class=obj id=obj>—</span>
  <span class=muted>은(는) 검출될 만큼 충분히 보였습니까?</span>
  <span class=tag id=stage></span><span class=tag id=loc></span>
  <span class=muted id=pos></span>
  <div class=bar><i id=prog></i></div><span id=done>0</span><span class=muted>/<span id=tot>0</span></span>
  <select id=filter>
    <option value=p1 selected>P1 proposal miss 만</option>
    <option value=all>전체</option>
    <option value=todo>미답변만</option>
  </select>
</div></header>
<main>
  <div class=wrap><img id=img alt=""><div id=ov></div></div>
  <div class=hint id=hint></div>
  <div class=vs id=vs></div>
  <div class=nav>
    <button class=nv id=prev><kbd>←</kbd> 이전</button>
    <button class=nv id=next>다음 <kbd>Space</kbd></button>
    <input type=text id=note placeholder="메모 (선택)" style="flex:1;min-width:180px">
  </div>
</main>
<script>
const V=__V__; let rows=[],view=[],i=0; const $=s=>document.querySelector(s);
function cur(){return rows[view[i]];}
function build(){$('#vs').innerHTML=V.map(v=>
  `<button class=v data-v="${v[0]}" style="border-color:${v[2]}33">
   <kbd>${v[0]}</kbd> ${v[1].split(' — ')[0]}<small>${v[1].split(' — ')[1]||''}</small></button>`).join('');
  document.querySelectorAll('button.v').forEach(b=>b.onclick=()=>pick(b.dataset.v));}
function render(){
  if(!view.length){$('#obj').textContent='해당 없음';return;}
  if(i<0)i=0; if(i>=view.length)i=view.length-1;
  const r=cur();
  $('#obj').textContent=r.object;
  $('#stage').textContent=r.stage; $('#stage').className='tag'+(r.priority===1?' p1':'');
  $('#loc').textContent=`${r.dataset} f${r.frame_id}`;
  $('#pos').textContent=`${i+1} / ${view.length}`;
  $('#note').value=r.note||'';
  const img=$('#img'); img.src=`/img/${view[i]}`;
  img.onload=()=>{
    const sx=img.clientWidth/r.iw, sy=img.clientHeight/r.ih;
    $('#ov').innerHTML=(r.boxes||[]).map(b=>{
      const [x1,y1,x2,y2]=b.box;
      return `<div class=bx style="left:${x1*sx}px;top:${y1*sy}px;width:${(x2-x1)*sx}px;height:${(y2-y1)*sy}px">
        <span>conf ${b.conf.toFixed(2)} · sem ${b.sem.toFixed(2)} · appe ${b.appe.toFixed(2)}</span></div>`;}).join('');
  };
  $('#hint').textContent = r.priority===1
    ? `후보 박스가 하나도 없었습니다 (YOLO raw 최대 conf ${r.raw_max_conf||'?'} / 후처리 후 ${r.post_max_conf||'?'}). 화면에서 ${r.object} 을(를) 직접 찾아보십시오.`
    : `후보 박스는 있었으나(빨간 상자) semantic/appearance 게이트에서 탈락했습니다.`;
  document.querySelectorAll('button.v').forEach(b=>b.classList.toggle('on',b.dataset.v===r.verdict));
  const d=rows.filter(x=>x.verdict).length;
  $('#done').textContent=d; $('#tot').textContent=rows.length;
  $('#prog').style.width=(100*d/rows.length)+'%';
}
function commit(){const r=cur();
  fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({index:view[i],verdict:r.verdict,note:r.note})});}
function pick(v){const r=cur(); r.verdict=(r.verdict===v?'':v); commit(); render();
  if(r.verdict) setTimeout(()=>step(1),120);}
function step(d){let j=i+d;
  while(j>=0&&j<view.length&&rows[view[j]].verdict) j+=d;
  if(j<0||j>=view.length) j=Math.max(0,Math.min(view.length-1,i+d));
  i=j; render();}
function applyFilter(){const f=$('#filter').value;
  view=rows.map((r,n)=>n).filter(n=>{const r=rows[n];
    if(f==='p1')return r.priority===1; if(f==='todo')return !r.verdict; return true;});
  i=Math.max(0,view.findIndex(n=>!rows[n].verdict)); render();}
$('#prev').onclick=()=>step(-1); $('#next').onclick=()=>step(1);
$('#filter').onchange=applyFilter;
$('#note').oninput=e=>{cur().note=e.target.value; commit();};
addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT')return;
  const k=e.key.toUpperCase();
  if(V.some(v=>v[0]===k)){e.preventDefault();pick(k);return;}
  if(e.key===' '||e.key==='ArrowRight'){e.preventDefault();step(1);}
  if(e.key==='Backspace'||e.key==='ArrowLeft'){e.preventDefault();step(-1);}});
build(); fetch('/api/rows').then(r=>r.json()).then(d=>{rows=d;applyFilter();});
</script></html>"""


class H(http.server.BaseHTTPRequestHandler):
    def _s(self, code, ctype, body):
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = urllib.parse.urlparse(self.path).path
        if p in ("/", "/index.html"):
            return self._s(200, "text/html; charset=utf-8",
                           PAGE.replace("__V__", json.dumps(VERDICTS, ensure_ascii=False))
                           .encode("utf-8"))
        if p == "/api/rows":
            out = []
            for q in QUEUE:
                iw, ih = draw_boxes_png(q["image_path"], None)
                out.append({**{k: q[k] for k in ("dataset", "frame_id", "object", "stage",
                                                 "priority", "verdict", "note",
                                                 "raw_max_conf", "post_max_conf")},
                            "iw": iw or 848, "ih": ih or 480,
                            "boxes": BOXES.get((q["dataset"], q["frame_id"], q["object"]), [])})
            return self._s(200, "application/json; charset=utf-8",
                           json.dumps(out, ensure_ascii=False).encode("utf-8"))
        if p.startswith("/img/"):
            try:
                fp = QUEUE[int(p[5:])]["image_path"]
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
            d = json.loads(self.rfile.read(n) or b"{}")
            q = QUEUE[int(d["index"])]
        except Exception:
            return self._s(400, "text/plain", b"bad")
        q["verdict"] = str(d.get("verdict", ""))
        q["note"] = str(d.get("note", ""))
        save()
        return self._s(200, "application/json",
                       json.dumps({"done": sum(1 for x in QUEUE if x["verdict"])}).encode())

    def log_message(self, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    import socket
    print(f"\n  주소 : http://127.0.0.1:{PORT}")
    if HOST != "127.0.0.1":
        try:
            print(f"         원격 접속용: http://{socket.gethostbyname(socket.gethostname())}:{PORT}")
        except Exception:
            pass
    print(f"  저장 : {ANS_PATH}")
    print("  종료 : Ctrl+C\n")
    try:
        with S((HOST, PORT), H) as httpd:
            httpd.serve_forever()
    except KeyboardInterrupt:
        print(f"\n종료. 답변 {sum(1 for x in QUEUE if x['verdict'])}/{len(QUEUE)} 저장됨.")
