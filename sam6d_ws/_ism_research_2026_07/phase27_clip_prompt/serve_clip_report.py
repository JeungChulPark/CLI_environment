#!/usr/bin/env python3
"""serve_clip_report.py — CLIP 프롬프트 판별력 분석 결과 웹 뷰어 (표준 라이브러리만).

탭 4개
  1) 현행 프롬프트 margin      — 어떤 프롬프트가 위험한가
  2) 유사도 행렬(히트맵)        — 프롬프트 × 객체
  3) CLIP 예측 vs 사람 확인 혼동 — 가설이 맞았나
  4) 후보 문구 랭킹 / v3 선택   — margin 최대 자동 선택 결과

실행: ~/miniconda3/envs/sam_yolo/bin/python .../serve_clip_report.py   → http://127.0.0.1:8770
"""
import csv, glob, http.server, json, os, socketserver, sys, urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase27_clip_prompt")
CSVD = os.path.join(OUT, "csv")
PORT = int(os.environ.get("GT_PORT", "8770"))
HOST = os.environ.get("GT_HOST", "0.0.0.0")


def rd(name):
    p = os.path.join(CSVD, name)
    return list(csv.DictReader(open(p, encoding="utf-8"))) if os.path.isfile(p) else []


DATA = {
    "margins": rd("current_prompt_margins.csv"),
    "matrix": rd("similarity_matrix.csv"),
    "confusion": rd("confusion_prediction.csv"),
    "ranking": rd("candidate_ranking.csv"),
    "v3": rd("prompts_v3_selected.csv"),
}
print("적재:", {k: len(v) for k, v in DATA.items()})

PAGE = """<!doctype html><html lang=ko><meta charset=utf-8><title>CLIP 프롬프트 분석</title>
<style>
:root{--bg:#0f1114;--fg:#e8eaed;--dim:#9aa0a6;--line:#2a2e35;--ok:#3ddc84;--bad:#ff6b6b;--wn:#ffb44c}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.6 -apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
header{background:#171a1f;border-bottom:1px solid var(--line);padding:10px 16px;position:sticky;top:0;z-index:5}
h2{margin:0 0 4px}
.tabs{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
.tabs button{background:#232830;border:1px solid var(--line);color:var(--fg);border-radius:6px;padding:7px 14px;cursor:pointer}
.tabs button.on{background:#1c2e47;border-color:#4c9aff}
main{padding:14px 16px;max-width:1500px;margin:0 auto}
table{border-collapse:collapse;width:100%;font-size:13px}
th,td{border:1px solid var(--line);padding:6px 8px;text-align:left}
th{background:#1c2027;position:sticky;top:0}
tr:hover td{background:#181c22}
.neg{color:var(--bad);font-weight:700}
.pos{color:var(--ok)}
.warn{color:var(--wn)}
.muted{color:var(--dim);font-size:12px}
.note{background:#181c22;border-left:3px solid #4c9aff;padding:10px 14px;margin:10px 0;border-radius:0 6px 6px 0}
.cell{text-align:center;font:12px ui-monospace,monospace}
.bar{display:inline-block;height:10px;background:#4c9aff;border-radius:2px;vertical-align:middle}
select{background:#1c2027;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 9px}
</style>
<header>
 <h2>CLIP 프롬프트 판별력 분석 <span class=muted>encoder = clip:ViT-B/32 (YOLO-World 동일) · 사람 GT 미사용, PLY 렌더 42뷰만</span></h2>
 <div class=tabs>
  <button data-t=m class=on>1. 현행 margin</button>
  <button data-t=x>2. 유사도 행렬</button>
  <button data-t=c>3. 가설 검증</button>
  <button data-t=r>4. 후보 랭킹 / v3</button>
 </div>
</header>
<main id=body></main>
<script>
let D={};
const $=s=>document.querySelector(s);
function esc(s){return String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}
function tbl(rows,cols,fmt){
 if(!rows.length) return '<p class=muted>데이터 없음</p>';
 let h='<table><tr>'+cols.map(c=>`<th>${c[1]}</th>`).join('')+'</tr>';
 for(const r of rows){h+='<tr>'+cols.map(c=>`<td>${fmt?fmt(c[0],r):esc(r[c[0]])}</td>`).join('')+'</tr>';}
 return h+'</table>';}

function viewM(){
 const f=(k,r)=>{
  if(k==='margin'){const v=parseFloat(r.margin);
   return `<span class="${v<0?'neg':(v<0.035?'warn':'pos')}">${v>=0?'+':''}${v.toFixed(4)}</span>`;}
  if(k==='rank_of_self') return r[k]==='1'?'1':`<span class=neg>${r[k]} ⚠</span>`;
  return esc(r[k]);};
 return `<div class=note><b>margin = sim(문구, 자기 렌더) − sim(문구, 가장 가까운 다른 객체 렌더)</b><br>
  작을수록 그 프롬프트는 다른 객체와 헷갈립니다. <b>음수면 자기 객체보다 남을 더 잘 가리킵니다.</b></div>`+
 tbl(D.margins,[['object','객체'],['prompt','현행 프롬프트'],['sim_self','self'],
  ['top_confusable','가장 헷갈리는 상대'],['sim_other','other'],['margin','margin'],
  ['rank_of_self','self 순위'],['second_confusable','2순위 상대']],f);}

function viewX(){
 if(!D.matrix.length) return '<p class=muted>없음</p>';
 const cols=Object.keys(D.matrix[0]).slice(1);
 let vals=[]; D.matrix.forEach(r=>cols.forEach(c=>vals.push(parseFloat(r[c]))));
 const mn=Math.min(...vals), mx=Math.max(...vals);
 let h=`<div class=note>행 = 프롬프트, 열 = 객체 렌더. <b>대각선이 그 행의 최대여야 정상</b>입니다.
  진할수록 유사도가 높습니다.</div><table><tr><th>프롬프트 \\ 객체</th>`+cols.map(c=>`<th>${c.replace('_high','')}</th>`).join('')+'</tr>';
 D.matrix.forEach((r,i)=>{
  h+=`<tr><th>${r['prompt_of'].replace('_high','')}</th>`;
  cols.forEach((c,j)=>{const v=parseFloat(r[c]); const t=(v-mn)/(mx-mn+1e-9);
   const diag=(i===j); const rowmax=Math.max(...cols.map(k=>parseFloat(r[k])));
   const isMax=Math.abs(v-rowmax)<1e-9;
   h+=`<td class=cell style="background:rgba(76,154,255,${(0.08+0.75*t).toFixed(2)});
     ${diag?'outline:2px solid #3ddc84;':''}${(isMax&&!diag)?'color:#ff6b6b;font-weight:700;':''}">${v.toFixed(3)}</td>`;});
  h+='</tr>';});
 return h+'</table><p class=muted>초록 테두리 = 대각(자기 객체) · 빨간 글씨 = 그 행의 최대인데 대각이 아님(오매칭)</p>';}

function viewC(){
 const f=(k,r)=>k==='match'?(r.match==='1'?'<span class=pos>MATCH</span>':'<span class=muted>—</span>')
   :(k==='margin'?(parseFloat(r.margin)<0?`<span class=neg>${r.margin}</span>`:r.margin):esc(r[k]));
 const n=D.confusion.filter(r=>r.human_observed).length, h=D.confusion.filter(r=>r.match==='1').length;
 return `<div class=note><b>가설:</b> CLIP 상에서 가까운 쌍이 실제 YOLO 오인식으로 이어진다.<br>
  <b>결과: ${h}/${n} 일치</b> — 부분적으로만 맞습니다. CLIP 전역 유사도는 YOLO-World 의
  region-text 정렬과 다르므로 <b>예측력이 제한적</b>입니다(아래 해석 참고).</div>`+
 tbl(D.confusion,[['object','객체'],['clip_predicts','CLIP 예측 혼동상대'],
  ['human_observed','사람 확인 혼동상대'],['count','실제 건수'],['margin','margin'],['match','일치']],f);}

function viewR(){
 const objs=[...new Set(D.ranking.map(r=>r.object))];
 let h=`<div class=note>사전 등록된 후보 문구 187개를 <b>margin 최대</b> 기준으로 자동 선택했습니다.
  사람 GT 를 쓰지 않고 <b>PLY 렌더만</b> 사용 → 신규 객체도 동일 절차로 자동화 가능(INV-01 준수).</div>`;
 h+='<h3>v3 자동 선택 결과</h3>'+tbl(D.v3,[['object','객체'],['current','현행'],['v3_selected','v3 자동선택'],
   ['margin_current','현행 margin'],['margin_v3','v3 margin'],['gain','개선']],
   (k,r)=>k==='gain'?`<span class=pos>${r.gain}</span>`:esc(r[k]));
 h+=`<h3 style="margin-top:18px">객체별 후보 랭킹 <select id=selobj>`+
   objs.map(o=>`<option>${o}</option>`).join('')+'</select></h3><div id=rank></div>';
 setTimeout(()=>{const s=$('#selobj'); const draw=()=>{
   const rows=D.ranking.filter(r=>r.object===s.value).slice(0,15);
   $('#rank').innerHTML=tbl(rows,[['rank','#'],['candidate','후보 문구'],['sim_self','self'],
     ['sim_other_max','other'],['margin','margin'],['closest_other','가장 가까운 타객체'],['is_current','현행']],
     (k,r)=>k==='is_current'?(r.is_current==='1'?'<span class=warn>← 현행</span>':'')
      :(k==='margin'?`<span class=${parseFloat(r.margin)<0?'neg':'pos'}>${r.margin}</span>`:esc(r[k])));};
   s.onchange=draw; draw();},0);
 return h;}

const V={m:viewM,x:viewX,c:viewC,r:viewR};
function show(t){document.querySelectorAll('.tabs button').forEach(b=>b.classList.toggle('on',b.dataset.t===t));
 $('#body').innerHTML=V[t]();}
document.querySelectorAll('.tabs button').forEach(b=>b.onclick=()=>show(b.dataset.t));
fetch('/api/data').then(r=>r.json()).then(d=>{D=d;show('m');});
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
            return self._s(200, "text/plain", f"ok {sum(len(v) for v in DATA.values())}".encode())
        if p in ("/", "/index.html"):
            return self._s(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        if p == "/api/data":
            return self._s(200, "application/json; charset=utf-8",
                           json.dumps(DATA, ensure_ascii=False).encode("utf-8"))
        return self._s(404, "text/plain", b"no")

    def log_message(self, *a):
        pass


class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True


if __name__ == "__main__":
    print(f"\n  주소 : http://127.0.0.1:{PORT}\n  종료 : Ctrl+C\n")
    with S((HOST, PORT), H) as httpd:
        httpd.serve_forever()
