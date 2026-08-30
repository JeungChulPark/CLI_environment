#!/usr/bin/env python3
"""serve_triptych.py — [원본|객체인식|포즈추정] 3분할 이미지 갤러리 뷰어 (표준 라이브러리만).

라벨링이 아니라 **열람용**. 좌우 키로 넘기며 보고, 눈에 띄는 건 메모만 남긴다.
메모는 outputs/phase24_localization/triptych_notes.csv 에 즉시 저장(원자적 교체).

실행:
  ~/miniconda3/envs/sam_yolo/bin/python _ism_research_2026_07/phase24_localization/serve_triptych.py
  브라우저: http://127.0.0.1:8768
"""
import csv, glob, http.server, json, os, socketserver, sys, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
ROOT = os.path.join(REPO, "outputs", "phase24_localization", "triptych")
NOTES = os.path.join(REPO, "outputs", "phase24_localization", "triptych_notes.csv")
PORT = int(os.environ.get("GT_PORT", "8768"))
HOST = os.environ.get("GT_HOST", "0.0.0.0")

ITEMS = []
for sub in ("gt", "pem"):
    for p in sorted(glob.glob(os.path.join(ROOT, sub, "*.png"))):
        name = os.path.basename(p)[:-4]
        bag = "_".join(name.split("_")[:-1]); frame = name.split("_")[-1]
        ITEMS.append({"path": p, "name": name, "group": sub, "bag": bag, "frame": frame, "note": ""})
if not ITEMS:
    raise SystemExit(f"[err] 이미지 없음: {ROOT} — make_frame_triptych.py 먼저 실행")

if os.path.isfile(NOTES):
    prev = {r["name"]: r.get("note", "") for r in csv.DictReader(open(NOTES, encoding="utf-8"))}
    for it in ITEMS:
        it["note"] = prev.get(it["name"], "")
BAGS = sorted({i["bag"] for i in ITEMS})
print(f"{len(ITEMS)}장 적재 (GT {sum(1 for i in ITEMS if i['group']=='gt')} / 기타 "
      f"{sum(1 for i in ITEMS if i['group']=='pem')}) · bag {len(BAGS)}종")


def save():
    tmp = NOTES + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f); w.writerow(["name", "group", "bag", "frame", "note"])
        for i in ITEMS:
            if i["note"].strip():
                w.writerow([i["name"], i["group"], i["bag"], i["frame"], i["note"]])
    os.replace(tmp, NOTES)


PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>원본 | 객체인식 | 포즈추정</title>
<style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;background:#171a1f;border-bottom:1px solid var(--line);padding:8px 14px;z-index:5}
.hrow{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.tag{font:12px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-radius:4px;padding:2px 8px}
.tag.gt{background:#17321f;border-color:#2f6b45;color:#9fe6bd}
select,input[type=text]{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 9px}
button{background:#232830;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:7px 14px;cursor:pointer}
main{padding:10px 14px}
#img{display:block;width:100%;border:1px solid var(--line);border-radius:6px;background:#000}
.cap{display:flex;gap:0;margin-top:6px;color:var(--dim);font-size:12px;text-align:center}
.cap>div{flex:1}
.muted{color:var(--dim);font-size:12px}
kbd{font:11px ui-monospace,monospace;background:#232830;border:1px solid var(--line);border-bottom-width:2px;border-radius:3px;padding:1px 5px}
</style>
<header><div class=hrow>
 <b>원본 | 객체인식 | 포즈추정</b>
 <span class=tag id=name></span><span class=tag id=grp></span>
 <span class=muted id=pos></span>
 <select id=fgrp><option value=all>전체</option><option value=gt selected>GT 프레임만</option><option value=pem>그 외</option></select>
 <select id=fbag></select>
 <button id=prev><kbd>←</kbd> 이전</button><button id=next>다음 <kbd>→</kbd></button>
 <input type=text id=jump placeholder="이동: 번호 또는 이름" style="width:190px">
 <input type=text id=note placeholder="메모 (자동 저장)" style="flex:1;min-width:200px">
</div></header>
<main>
 <img id=img alt="">
 <div class=cap><div>원본</div><div>객체 인식</div><div>포즈 추정</div></div>
 <div class=muted style="margin-top:8px"><kbd>←</kbd><kbd>→</kbd> 이동 · <kbd>Home</kbd>/<kbd>End</kbd> 처음/끝 · 메모는 입력 즉시 저장 · 메모 있는 항목은 목록에 ● 표시</div>
</main>
<script>
let all=[],view=[],i=0; const $=s=>document.querySelector(s);
function cur(){return all[view[i]];}
function render(){
 if(!view.length){$('#name').textContent='해당 없음';$('#img').removeAttribute('src');return;}
 if(i<0)i=0; if(i>=view.length)i=view.length-1;
 const r=cur();
 $('#name').textContent=r.name;
 $('#grp').textContent=r.group==='gt'?'GT':'PEM'; $('#grp').className='tag'+(r.group==='gt'?' gt':'');
 $('#pos').textContent=`${i+1} / ${view.length}`;
 $('#img').src=`/img/${view[i]}`;
 $('#note').value=r.note||'';}
function applyFilter(){
 const g=$('#fgrp').value, b=$('#fbag').value;
 view=all.map((r,n)=>n).filter(n=>{const r=all[n];
  if(g!=='all'&&r.group!==g)return false;
  if(b!=='all'&&r.bag!==b)return false; return true;});
 i=0; render();}
function step(d){i=Math.max(0,Math.min(view.length-1,i+d)); render();}
$('#prev').onclick=()=>step(-1); $('#next').onclick=()=>step(1);
$('#fgrp').onchange=applyFilter; $('#fbag').onchange=applyFilter;
$('#note').oninput=e=>{const r=cur(); r.note=e.target.value;
 fetch('/api/note',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({index:view[i],note:r.note})});};
$('#jump').onchange=e=>{const v=e.target.value.trim();
 let k=-1;
 if(/^\\d+$/.test(v)) k=parseInt(v)-1;
 else k=view.findIndex(n=>all[n].name.includes(v));
 if(k>=0&&k<view.length){i=k;render();} e.target.value='';};
addEventListener('keydown',e=>{ if(e.target.tagName==='INPUT')return;
 if(e.key==='ArrowRight'||e.key===' '){e.preventDefault();step(1);}
 if(e.key==='ArrowLeft'){e.preventDefault();step(-1);}
 if(e.key==='Home'){i=0;render();} if(e.key==='End'){i=view.length-1;render();}});
fetch('/api/list').then(r=>r.json()).then(d=>{
 all=d.items;
 $('#fbag').innerHTML='<option value=all>전체 bag</option>'+d.bags.map(b=>`<option value="${b}">${b}</option>`).join('');
 applyFilter();});
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
            return self._s(200, "text/plain; charset=utf-8", f"ok items={len(ITEMS)}".encode())
        if p in ("/", "/index.html"):
            return self._s(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        if p == "/api/list":
            items = [{k: v for k, v in i.items() if k != "path"} for i in ITEMS]
            return self._s(200, "application/json; charset=utf-8",
                           json.dumps({"items": items, "bags": BAGS}, ensure_ascii=False).encode())
        if p.startswith("/img/"):
            try:
                fp = ITEMS[int(p[5:])]["path"]
            except (ValueError, IndexError):
                return self._s(404, "text/plain", b"no")
            if not os.path.isfile(fp):
                return self._s(404, "text/plain", b"missing")
            with open(fp, "rb") as f:
                return self._s(200, "image/png", f.read())
        return self._s(404, "text/plain", b"no")

    def do_POST(self):
        if urllib.parse.urlparse(self.path).path != "/api/note":
            return self._s(404, "text/plain", b"no")
        n = int(self.headers.get("Content-Length", "0"))
        try:
            d = json.loads(self.rfile.read(n) or b"{}")
            ITEMS[int(d["index"])]["note"] = str(d.get("note", ""))
        except Exception:
            return self._s(400, "text/plain", b"bad")
        save()
        return self._s(200, "application/json", b'{"ok":1}')

    def log_message(self, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        miss = [i["name"] for i in ITEMS if not os.path.isfile(i["path"])]
        print(f"  이미지 {len(ITEMS)} · 누락 {len(miss)}")
        raise SystemExit(0 if not miss else 1)
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n  메모 : {NOTES}\n  종료 : Ctrl+C\n")
    with S((HOST, PORT), H) as httpd:
        httpd.serve_forever()
