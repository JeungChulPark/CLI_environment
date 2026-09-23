#!/usr/bin/env python3
"""live_view.py — watch RTAB-Map build the map in real time, in a browser on 127.0.0.1.

rtab_stream's live mode is a TCP client: it connects out, announces itself, waits for a "start",
then streams newline-delimited JSON — one "pose" per frame at the replay rate and a "kf_update"
carrying the whole node graph whenever the map changes (plus once a second). This script is the
other end: it listens for that connection, relays every message to the browser over SSE, and
serves the viewer page.

Three odometry sources, chosen on the page and run one at a time (they would otherwise share the
CPU during a real-time measurement, which is the thing being measured):

    base   RTAB-Map's own visual odometry      Odom/Strategy 0 = F2M
    imu    the same VO + xsens orientation     --gyro RIG.json --imu DIR   (src/imu_source.h)
    lidar  no visual odometry; KISS-ICP poses  --rig RIG.json --ext-odom   (src/rtab_aux.h)

A finished run keeps its trajectory on the canvas, faded, so the next one draws over it.

    python3 live_view.py --session <dataset>/SLAM [--rig RIG.json] [--imu-dir DIR]
                         [--ext-odom kiss/kiss_tum.txt] [--port 8972] [--no-odom-cov]

Open http://127.0.0.1:<port>/. Nothing is written to disk.
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import socket
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

RUN = os.path.expanduser("~/objpose/run_slam.sh")

clients: list[queue.Queue] = []
clients_lock = threading.Lock()
state: dict = {"running": None, "proc": None, "srv": None}

# What each run has produced so far, so a browser that connects late — or reloads — gets the whole
# picture instead of only what arrives after it attached.
VARIANTS = ("base", "imu", "lidar")
hist: dict = {v: {"traj": [], "nodes": [], "frames": 0, "dropped": 0, "n": 0, "ms": 0.0}
              for v in VARIANTS}
hist_lock = threading.Lock()


def note(variant: str, msg: dict) -> None:
    h = hist.get(variant)
    if h is None:
        return
    with hist_lock:
        if msg["type"] == "pose":
            h["frames"] += 1
            h["dropped"] = msg.get("dropped", h["dropped"])
            h["ms"] = msg.get("track_ms", h["ms"])
            T = msg.get("T_wc")
            if T:
                h["traj"].append([round(T[3], 4), round(T[7], 4), round(T[11], 4)])
        elif msg["type"] == "kf_update":
            h["nodes"] = [[round(k[4], 4), round(k[8], 4), round(k[12], 4)] for k in msg["kfs"]]
            h["n"] = msg["n"]


def snapshot() -> dict:
    with hist_lock:
        return {"type": "restore",
                "runs": {v: {k: (list(x) if isinstance(x, list) else x) for k, x in h.items()}
                         for v, h in hist.items() if h["frames"] or h["nodes"]}}


def broadcast(obj: dict) -> None:
    line = json.dumps(obj, separators=(",", ":"))
    with clients_lock:
        dead = []
        for q in clients:
            try:
                q.put_nowait(line)
            except queue.Full:
                dead.append(q)      # a tab that stopped reading; drop it rather than stall SLAM
        for q in dead:
            clients.remove(q)


def build_cmd(a, variant: str, tcp_port: int, rate: float):
    cmd = [RUN, "--slam", "rtabmap", "--session", a.session, "--time-source", "frame",
           "--mode", "live", "--connect", f"127.0.0.1:{tcp_port}", "--rate", str(rate)]
    if variant == "imu":
        if not (a.rig and a.imu_dir):
            return None
        cmd += ["--gyro", a.rig, "--imu", a.imu_dir]
    elif variant == "lidar":
        if not (a.rig and a.ext_odom):
            return None
        cmd += ["--rig", a.rig, "--ext-odom", a.ext_odom]
    elif variant != "base":
        return None
    if a.features:
        cmd += ["--features", str(a.features)]
    if a.no_odom_cov:
        cmd += ["--no-odom-cov"]
    for p in a.param:
        cmd += ["--param", p]
    return cmd


def relay(conn: socket.socket, variant: str, rate: float) -> None:
    f = conn.makefile("rwb")
    try:
        hello = json.loads(f.readline().decode())
        broadcast(dict(hello, type="hello", variant=variant))
        f.write((json.dumps({"type": "start", "t0_ns": hello["first_ns"], "rate": rate}) + "\n").encode())
        f.flush()
        for raw in f:
            if not raw.strip():
                continue
            try:
                msg = json.loads(raw.decode())
            except json.JSONDecodeError:
                continue
            msg["variant"] = variant
            note(variant, msg)
            broadcast(msg)
    except (OSError, ValueError, KeyError) as e:
        broadcast({"type": "error", "variant": variant, "message": f"{type(e).__name__}: {e}"})
    finally:
        try:
            conn.close()
        except OSError:
            pass


def serve_slam(a, variant: str, rate: float, tcp_port: int) -> None:
    cmd = build_cmd(a, variant, tcp_port, rate)
    if cmd is None:
        broadcast({"type": "error", "variant": variant, "message": "필요한 입력이 설정되지 않음"})
        state["running"] = None
        broadcast({"type": "closed", "variant": variant})
        return
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", tcp_port))
    srv.listen(1)
    state["srv"] = srv
    broadcast({"type": "launch", "variant": variant, "cmd": " ".join(cmd)})
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    state["proc"] = proc

    def pump():
        for ln in proc.stderr:
            ln = ln.rstrip()
            if ln:
                print(f"  [{variant}]", ln, flush=True)
                broadcast({"type": "log", "variant": variant, "line": ln})
    threading.Thread(target=pump, daemon=True).start()

    srv.settimeout(40)
    try:
        conn, _ = srv.accept()
    except socket.timeout:
        broadcast({"type": "error", "variant": variant, "message": "rtab_stream이 40초 안에 접속하지 않음"})
        proc.kill()
    else:
        relay(conn, variant, rate)
    finally:
        state["srv"] = None
        try:
            srv.close()
        except OSError:
            pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
    state["proc"] = None
    state["running"] = None
    broadcast({"type": "closed", "variant": variant})


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RTAB-Map 실시간 3종 · 260901_cbnu_bigeightcircle</title>
<style>
:root{
  --paper:#EDF1F6; --surface:#FFFFFF; --surface-2:#F6F9FC; --line:#C6D2DE; --line-soft:#DDE5EC;
  --ink:#11202F; --muted:#5A6D80;
  --base:#2a78d6; --imu:#eb6834; --lidar:#1baf7a; --lost:#d03b3b;
  --sans:"Pretendard Variable",Pretendard,"Apple SD Gothic Neo","Noto Sans KR",-apple-system,Arial,sans-serif;
  --mono:ui-monospace,SFMono-Regular,Menlo,Consolas,"D2Coding",monospace;
}
@media (prefers-color-scheme:dark){:root{
  --paper:#0D1620; --surface:#15202C; --surface-2:#1A2734; --line:#28394A; --line-soft:#223142;
  --ink:#E3EBF3; --muted:#90A4B6; --base:#3987e5; --imu:#d95926; --lidar:#199e70; --lost:#e66767;}}
*{box-sizing:border-box}
body{margin:0;background:var(--paper);color:var(--ink);font-family:var(--sans);font-size:14px}
.wrap{max-width:1240px;margin:0 auto;padding:22px 16px 60px}
h1{font-size:22px;margin:0 0 4px;font-weight:700;letter-spacing:-.01em}
.sub{color:var(--muted);font-size:13px;margin:0 0 16px;max-width:80ch}
.bar{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-bottom:12px}
button{font-family:var(--sans);font-size:14px;font-weight:600;padding:8px 16px;border-radius:6px;
  border:1px solid var(--line);background:var(--surface);color:var(--ink);cursor:pointer}
button[disabled]{opacity:.4;cursor:default}
button.v{border-left-width:5px}
#b-base{border-left-color:var(--base)} #b-imu{border-left-color:var(--imu)} #b-lidar{border-left-color:var(--lidar)}
button.on{background:var(--ink);color:var(--surface)}
.pill{font-family:var(--mono);font-size:12px;padding:5px 10px;border-radius:99px;background:var(--surface-2);
  border:1px solid var(--line-soft);color:var(--muted)}
.pill b{color:var(--ink);font-weight:600}
.pill.ok b{color:var(--lidar)} .pill.lost b{color:var(--lost)}
.grid{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:16px}
@media (max-width:900px){.grid{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--surface);border:1px solid var(--line);border-radius:8px;padding:12px}
canvas{display:block;width:100%;height:auto;border-radius:5px}
#map{cursor:grab;touch-action:none} #map.drag{cursor:grabbing}
.viewbar{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin-bottom:8px}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:5px;overflow:hidden}
.seg button{font-size:12.5px;font-weight:500;padding:4px 12px;border:0;border-radius:0;background:var(--surface);color:var(--muted)}
.seg button[aria-pressed="true"]{background:var(--ink);color:var(--surface);font-weight:600}
.hint{font-size:11.5px;color:var(--muted);font-family:var(--mono)}
table{border-collapse:collapse;width:100%;font-size:13px}
td,th{padding:5px 4px;border-bottom:1px solid var(--line-soft);text-align:left}
th{font-size:11px;letter-spacing:.06em;text-transform:uppercase;color:var(--muted);font-weight:600}
td.n,th.n{text-align:right;font-family:var(--mono);font-variant-numeric:tabular-nums}
.sw{display:inline-block;width:9px;height:9px;border-radius:2px;margin-right:6px;vertical-align:middle}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:12px;color:var(--muted);margin-top:8px}
.legend i{display:inline-block;width:14px;height:0;border-top:3px solid;margin-right:5px;vertical-align:middle}
.smalls{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin-top:12px}
@media (max-width:760px){.smalls{grid-template-columns:minmax(0,1fr)}}
.smalls figure{margin:0}
.smalls canvas{border:1px solid var(--line-soft);background:var(--surface-2)}
.smalls figcaption{font-size:12px;color:var(--muted);margin-top:5px;font-family:var(--mono)}
.smalls figcaption b{color:var(--ink)}
#log{font-family:var(--mono);font-size:11px;color:var(--muted);height:110px;overflow-y:auto;
  background:var(--surface-2);border:1px solid var(--line-soft);border-radius:5px;padding:8px;margin-top:14px;white-space:pre-wrap}
</style></head><body><div class="wrap">
<h1>RTAB-Map 실시간 · 오도메트리 3종</h1>
<p class="sub">260901_cbnu_bigeightcircle · 9445 프레임 29.98 Hz. 매퍼 설정은 세 실행이 동일하고 오도메트리 소스만 다릅니다. 실시간 측정이라 <b>한 번에 하나만</b> 돌립니다 — 끝난 실행의 궤적은 흐리게 남아 다음 실행과 겹쳐 보입니다.</p>
<div class="bar">
  <button id="b-base"  class="v">1 · 기본 (VO)</button>
  <button id="b-imu"   class="v">2 · IMU + VO</button>
  <button id="b-lidar" class="v">3 · LiDAR 오도메트리</button>
  <button id="b-stop">정지</button>
  <span class="seg" style="margin-left:6px">
    <button id="r-1" type="button" aria-pressed="true">1.0×</button>
    <button id="r-2" type="button" aria-pressed="false">2×</button>
    <button id="r-4" type="button" aria-pressed="false">4×</button>
  </span>
  <span class="pill">상태 <b id="st">대기</b></span>
  <span class="pill">경과 <b id="el">0.0</b> s</span>
  <span class="pill">프레임 <b id="fr">0</b>/9445</span>
</div>
<div class="grid">
  <div class="card">
    <div class="viewbar">
      <span class="seg"><button id="v-iso" type="button" aria-pressed="true">비스듬히</button><button id="v-top" type="button" aria-pressed="false">위에서</button><button id="v-side" type="button" aria-pressed="false">옆에서</button></span>
      <span class="hint">드래그 = 회전 · 휠 = 확대 · shift+드래그 = 이동</span>
    </div>
    <canvas id="map" width="1500" height="1050"></canvas>
    <div class="legend">
      <span><i style="border-color:var(--base)"></i>기본 (VO)</span>
      <span><i style="border-color:var(--imu)"></i>IMU + VO</span>
      <span><i style="border-color:var(--lidar)"></i>LiDAR 오도메트리</span>
      <span>점 = 그래프 노드 · 시야뿔 = 현재 카메라 · 바닥 격자 1 m</span>
    </div>
    <div class="smalls">
      <figure><canvas id="s-base"  width="620" height="480"></canvas><figcaption id="c-base">기본 (VO)</figcaption></figure>
      <figure><canvas id="s-imu"   width="620" height="480"></canvas><figcaption id="c-imu">IMU + VO</figcaption></figure>
      <figure><canvas id="s-lidar" width="620" height="480"></canvas><figcaption id="c-lidar">LiDAR 오도메트리</figcaption></figure>
    </div>
    <p class="hint" style="margin-top:6px">세 패널은 위 3D 뷰와 <b>같은 시점·같은 축척</b>입니다 — 회전·확대가 함께 적용됩니다.</p>
  </div>
  <div>
    <div class="card">
      <table><thead><tr><th>실행</th><th class="n">프레임</th><th class="n">드롭</th><th class="n">노드</th><th class="n">ms</th></tr></thead>
      <tbody id="runs"></tbody></table>
    </div>
    <div class="card" style="margin-top:12px"><table id="stats"><tbody></tbody></table></div>
    <div class="card" style="margin-top:12px">
      <canvas id="tm" width="600" height="190"></canvas>
      <div class="legend"><span>현재 실행의 프레임 처리 시간 (ms) · 빨간선 = 33.3 ms 예산</span></div>
    </div>
  </div>
</div>
<div id="log"></div>
</div>
<script>
var VS=['base','imu','lidar'], COL={base:'--base',imu:'--imu',lidar:'--lidar'};
var NAME={base:'기본 (VO)', imu:'IMU + VO', lidar:'LiDAR 오도메트리'};
function blank(){ return {traj:[],nodes:[],cur:null,frames:0,dropped:0,n:0,ms:0,t0:null,ok:0,el:0}; }
var R={}; VS.forEach(function(v){ R[v]=blank(); });
var active=null, rate=1.0, tms=[], lastStat={};
var az=-0.62, el=0.42, dist=14, tgt=[0,0,0], userMoved=false, fpx=900;
function css(v){ return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }
function q(id){ return document.getElementById(id); }
function log(s){ var e=q('log'); e.textContent+=s+"\n"; e.scrollTop=e.scrollHeight; }

// optical (x right, y down, z forward) -> display (x right, y UP, z forward)
function D3(x,y,z){ return [x,-y,z]; }
function sub(a,b){ return [a[0]-b[0],a[1]-b[1],a[2]-b[2]]; }
function add(a,b){ return [a[0]+b[0],a[1]+b[1],a[2]+b[2]]; }
function mul(a,s){ return [a[0]*s,a[1]*s,a[2]*s]; }
var VIEW={W:1500,H:1050}, fpxc=900;
function proj(p){
  var d=sub(p,tgt), ca=Math.cos(az), sa=Math.sin(az);
  var ex= d[0]*ca + d[2]*sa, ez=-d[0]*sa + d[2]*ca;
  var ce=Math.cos(el), se=Math.sin(el);
  var ey= d[1]*ce - ez*se, e2= d[1]*se + ez*ce, zc= e2 + dist;
  if(zc<0.05) return null;
  var s=fpxc/zc;
  return [VIEW.W/2 + ex*s, VIEW.H/2 - ey*s, zc];
}
function seg(g,a,b){ var A=proj(a), B=proj(b); if(!A||!B) return; g.moveTo(A[0],A[1]); g.lineTo(B[0],B[1]); }

function autofit(){
  if(userMoved) return;
  var pts=[]; VS.forEach(function(v){ pts=pts.concat(R[v].traj, R[v].nodes); });
  if(pts.length<2) return;
  var lo=[1e9,1e9,1e9], hi=[-1e9,-1e9,-1e9];
  pts.forEach(function(p){ for(var k=0;k<3;k++){ if(p[k]<lo[k])lo[k]=p[k]; if(p[k]>hi[k])hi[k]=p[k]; } });
  tgt=[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2];
  dist=Math.max(hi[0]-lo[0], hi[2]-lo[2], hi[1]-lo[1], 2)*1.6+3;
}

function scene(g,W,H,list,solo){
  VIEW.W=W; VIEW.H=H; fpxc=900*Math.min(W,H)/1050;
  g.clearRect(0,0,W,H);
  var gy=0,n=0; list.forEach(function(v){ R[v].traj.forEach(function(p){ gy+=p[1]; n++; }); });
  gy=n?gy/n:0;
  var cxg=Math.round(tgt[0]), czg=Math.round(tgt[2]), Rr=Math.ceil(dist*0.9);
  g.strokeStyle=css('--line-soft'); g.lineWidth=1.5; g.beginPath();
  for(var i=-Rr;i<=Rr;i++){
    seg(g,[cxg+i,gy,czg-Rr],[cxg+i,gy,czg+Rr]);
    seg(g,[cxg-Rr,gy,czg+i],[cxg+Rr,gy,czg+i]);
  }
  g.stroke();

  list.forEach(function(v){
    var r=R[v]; if(!r.traj.length && !r.nodes.length) return;
    var isA=solo?true:(v===active), col=css(COL[v]);
    g.globalAlpha=isA?1:0.30;
    var pn=[]; r.nodes.forEach(function(p){ var s=proj(p); if(s) pn.push(s); });
    pn.sort(function(a,b){ return b[2]-a[2]; });
    g.fillStyle=col;
    pn.forEach(function(s){ var rad=Math.max(2,7*(dist/s[2])*0.55);
      g.beginPath(); g.arc(s[0],s[1],rad,0,6.2832); g.fill(); });
    if(r.traj.length>1){
      g.strokeStyle=col; g.lineWidth=isA?3:2; g.lineJoin='round'; g.beginPath();
      var started=false;
      for(var k=0;k<r.traj.length;k++){
        var s=proj(r.traj[k]);
        if(!s){ started=false; continue; }
        if(started) g.lineTo(s[0],s[1]); else { g.moveTo(s[0],s[1]); started=true; }
      }
      g.stroke();
    }
    if(isA && r.cur && (!solo || v===active)){
      var P=r.cur.p, f=0.75, w=0.42, h=0.30;
      var corner=[[w,h],[-w,h],[-w,-h],[w,-h]].map(function(uv){
        return add(P, add(mul(r.cur.ez,f), add(mul(r.cur.ex,uv[0]), mul(r.cur.ey,uv[1])))); });
      g.strokeStyle=col; g.lineWidth=3; g.beginPath();
      for(var a=0;a<4;a++){ seg(g,P,corner[a]); seg(g,corner[a],corner[(a+1)%4]); }
      g.stroke();
      g.lineWidth=4; g.beginPath(); seg(g,P,add(P,mul(r.cur.ez,f*1.5))); g.stroke();
      var s0=proj(P);
      if(s0){ g.fillStyle=col; g.beginPath(); g.arc(s0[0],s0[1],8,0,6.2832); g.fill();
              g.strokeStyle=css('--surface'); g.lineWidth=3; g.stroke(); }
      g.strokeStyle=css('--muted'); g.lineWidth=1.5; g.setLineDash([5,5]);
      g.beginPath(); seg(g,P,[P[0],gy,P[2]]); g.stroke(); g.setLineDash([]);
    }
    g.globalAlpha=1;
  });
}

function draw(){
  autofit();
  var c=q('map');
  if(VS.some(function(v){ return R[v].traj.length||R[v].nodes.length; }))
    scene(c.getContext('2d'), c.width, c.height, VS, false);
  else c.getContext('2d').clearRect(0,0,c.width,c.height);
  VS.forEach(function(v){
    var sc=q('s-'+v);
    scene(sc.getContext('2d'), sc.width, sc.height, [v], true);
    var r=R[v];
    q('c-'+v).innerHTML='<b>'+NAME[v]+'</b> · 프레임 '+r.frames+' · 드롭 '+r.dropped+' · 노드 '+r.n
      +(r.ms?' · '+r.ms.toFixed(1)+' ms':'');
  });

  var t=q('tm'), tg=t.getContext('2d'), TW=t.width, TH=t.height;
  tg.clearRect(0,0,TW,TH);
  var mx=Math.max(40, Math.max.apply(null,tms.concat([40])));
  tg.strokeStyle=css('--lost'); tg.lineWidth=2; tg.setLineDash([6,5]);
  var yb=TH-33.3/mx*TH; tg.beginPath(); tg.moveTo(0,yb); tg.lineTo(TW,yb); tg.stroke(); tg.setLineDash([]);
  tg.strokeStyle=active?css(COL[active]):css('--muted'); tg.lineWidth=2; tg.beginPath();
  tms.forEach(function(v,i){ var x=i/Math.max(1,tms.length-1)*TW, y=TH-v/mx*TH; i?tg.lineTo(x,y):tg.moveTo(x,y); });
  tg.stroke();
}

function runsTable(){
  var tb=q('runs'); tb.innerHTML='';
  VS.forEach(function(v){
    var r=R[v], tr=document.createElement('tr');
    tr.innerHTML='<td><span class="sw" style="background:'+css(COL[v])+'"></span>'+NAME[v]+(v===active?' ●':'')+'</td>'
      +'<td class="n">'+r.frames+'</td><td class="n">'+r.dropped+'</td>'
      +'<td class="n">'+r.n+'</td><td class="n">'+(r.ms?r.ms.toFixed(1):'—')+'</td>';
    tb.appendChild(tr);
  });
}
function stats(){
  var m=lastStat, rows=[['프레임 처리',(m.track_ms||0).toFixed(1)+' ms'],
    ['  오도메트리',(m.odom_ms||0).toFixed(1)+' ms'],['  매퍼',(m.map_ms||0).toFixed(1)+' ms'],
    ['  이미지 준비',(m.prep_ms||0).toFixed(1)+' ms'],['특징점',m.features||0],['인라이어',m.inliers||0],
    ['참조 노드',m.ref_kf==null?'—':m.ref_kf]];
  var tb=q('stats').querySelector('tbody'); tb.innerHTML='';
  rows.forEach(function(r){ var tr=document.createElement('tr');
    tr.innerHTML='<td>'+r[0]+'</td><td class="n">'+r[1]+'</td>'; tb.appendChild(tr); });
}

function onPose(v,m){
  var r=R[v];
  if(r.t0===null) r.t0=m.t_ns;
  r.frames++; r.dropped=m.dropped; r.el=(m.t_ns-r.t0)/1e9; r.ms=m.track_ms;
  if(m.state==='OK') r.ok++;
  if(v===active){
    q('el').textContent=r.el.toFixed(1);
    q('fr').textContent=m.frame_idx+1;
    var s=q('st'); s.textContent=m.state;
    s.parentElement.className='pill '+(m.state==='OK'?'ok':'lost');
    tms.push(m.track_ms); if(tms.length>400) tms.shift();
    lastStat=m;
  }
  if(m.T_wc){
    var T=m.T_wc;
    r.traj.push(D3(T[3],T[7],T[11]));
    r.cur={p:D3(T[3],T[7],T[11]),ex:D3(T[0],T[4],T[8]),ey:D3(T[1],T[5],T[9]),ez:D3(T[2],T[6],T[10])};
  }
  draw(); stats(); runsTable();
}

function setBusy(b){
  VS.forEach(function(v){ q('b-'+v).disabled=b; q('b-'+v).classList.toggle('on', b && active===v); });
  q('b-stop').disabled=!b;
}
function start(v){
  active=v; tms=[]; lastStat={}; R[v]=blank();
  setBusy(true); q('st').textContent='초기화'; q('el').textContent='0.0'; q('fr').textContent='0';
  fetch('start?v='+v+'&rate='+rate).then(function(r){return r.json();})
    .then(function(j){ if(j.error){ log('오류: '+j.error); setBusy(false); } });
  runsTable(); draw();
}

var es=new EventSource('events');
es.onmessage=function(ev){
  var m=JSON.parse(ev.data), v=m.variant;
  if(m.type==='restore'){
    Object.keys(m.runs).forEach(function(v){
      var h=m.runs[v], r=blank();
      r.traj=h.traj.map(function(p){ return D3(p[0],p[1],p[2]); });
      r.nodes=(h.nodes||[]).map(function(p){ return D3(p[0],p[1],p[2]); });
      r.frames=h.frames; r.dropped=h.dropped; r.n=h.n; r.ms=h.ms;
      R[v]=r;
    });
    draw(); runsTable();
    var k=Object.keys(m.runs).length;
    if(k) log('이전 실행 '+k+'개 복원');
  }
  else if(m.type==='launch'){ log('['+v+'] '+m.cmd.replace(/^\S*run_slam\.sh/,'run_slam.sh')); }
  else if(m.type==='hello'){ log('['+v+'] 연결 · rtabmap '+(m.rtabmap_version||'')+' · '+m.n_frames+' frames'); }
  else if(m.type==='log'){ if(/done:|loops|covariance|gravity|imu:|external/.test(m.line)) log('['+v+'] '+m.line.replace('[rtab_stream] ','')); }
  else if(m.type==='pose'){ onPose(v,m); }
  else if(m.type==='kf_update'){ R[v].nodes=m.kfs.map(function(k){return D3(k[4],k[8],k[12]);}); R[v].n=m.n; draw(); runsTable(); }
  else if(m.type==='end'){ log('['+v+'] 완료 · '+m.processed+' 프레임, 드롭 '+m.dropped); }
  else if(m.type==='closed'){ if(active===v) q('st').textContent='완료'; setBusy(false); draw(); runsTable(); }
  else if(m.type==='error'){ log('['+v+'] 오류: '+m.message); setBusy(false); }
};

VS.forEach(function(v){ q('b-'+v).addEventListener('click',function(){ start(v); }); });
q('b-stop').addEventListener('click',function(){ fetch('stop'); });
q('b-stop').disabled=true;
[['r-1',1],['r-2',2],['r-4',4]].forEach(function(p){
  q(p[0]).addEventListener('click',function(){
    rate=p[1]; ['r-1','r-2','r-4'].forEach(function(k){ q(k).setAttribute('aria-pressed', k===p[0]?'true':'false'); });
  });
});

(function(){
  var c=q('map'), drag=null;
  c.addEventListener('pointerdown',function(e){ drag={x:e.clientX,y:e.clientY,shift:e.shiftKey};
    userMoved=true; c.classList.add('drag'); c.setPointerCapture(e.pointerId); });
  c.addEventListener('pointermove',function(e){
    if(!drag) return;
    var dx=e.clientX-drag.x, dy=e.clientY-drag.y; drag.x=e.clientX; drag.y=e.clientY;
    if(drag.shift){
      // Pan along the CAMERA's own right/up axes, not world x/y. proj() decomposes an offset d as
      //   right = (ca, 0, sa)          up = (se*sa, ce, -se*ca)          depth = (-ce*sa, se, ce*ca)
      // Both right and up are perpendicular to depth, so moving the target along them leaves zc —
      // and therefore the apparent scale — untouched. Sliding along world y instead changes zc by
      // -sin(el)*dy, which is why panning used to zoom whenever the view was tilted.
      var ca=Math.cos(az), sa=Math.sin(az), ce=Math.cos(el), se=Math.sin(el);
      var rect=c.getBoundingClientRect();
      var k=dist/fpxc*(VIEW.W/Math.max(1,rect.width));   // world units per CSS pixel at the target
      tgt=[tgt[0] - dx*k*ca + dy*k*(se*sa),
           tgt[1]           + dy*k*ce,
           tgt[2] - dx*k*sa + dy*k*(-se*ca)];
    }
    else { az-=dx*0.006; el=Math.max(-1.45,Math.min(1.45, el+dy*0.006)); }
    draw();
  });
  function up(){ if(drag){ drag=null; c.classList.remove('drag'); } }
  c.addEventListener('pointerup',up); c.addEventListener('pointercancel',up);
  c.addEventListener('wheel',function(e){ e.preventDefault(); userMoved=true;
    dist=Math.max(1.5,Math.min(200,dist*(e.deltaY>0?1.12:0.89))); draw(); },{passive:false});
  function preset(id,a,e2){ q(id).addEventListener('click',function(){
    ['v-iso','v-top','v-side'].forEach(function(k){ q(k).setAttribute('aria-pressed',k===id?'true':'false'); });
    az=a; el=e2; userMoved=false; draw(); }); }
  preset('v-iso',-0.62,0.42); preset('v-top',0,1.44); preset('v-side',0,0.03);
})();
runsTable();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    args = None
    tcp_port = 8082

    def log_message(self, *a):
        pass

    def _json(self, obj):
        out = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(out)))
        self.end_headers()
        self.wfile.write(out)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/start":
            qs = parse_qs(u.query)
            variant = qs.get("v", ["base"])[0]
            rate = float(qs.get("rate", ["1.0"])[0])
            if state["running"]:
                self._json({"error": f"{state['running']} 실행 중 — 먼저 정지하세요"})
                return
            state["running"] = variant
            with hist_lock:
                hist[variant] = {"traj": [], "nodes": [], "frames": 0, "dropped": 0, "n": 0, "ms": 0.0}
            threading.Thread(target=serve_slam, args=(self.args, variant, rate, self.tcp_port),
                             daemon=True).start()
            self._json({"ok": True, "variant": variant, "rate": rate})
        elif u.path == "/stop":
            p = state.get("proc")
            if p and p.poll() is None:
                p.terminate()
            sv = state.get("srv")          # unblock a serve_slam still waiting in accept()
            if sv is not None:
                try:
                    sv.close()
                except OSError:
                    pass
            self._json({"ok": True})
        elif u.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            q: queue.Queue = queue.Queue(maxsize=60000)
            q.put(json.dumps(snapshot(), separators=(",", ":")))
            with clients_lock:
                clients.append(q)
            try:
                while True:
                    try:
                        self.wfile.write(f"data: {q.get(timeout=15)}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with clients_lock:
                    if q in clients:
                        clients.remove(q)
        else:
            self.send_error(404)


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, help="<dataset>/SLAM")
    ap.add_argument("--rig", default=str(here.parent / "bigeight" / "rig_260901_bigeight.json"))
    ap.add_argument("--imu-dir", dest="imu_dir", default="")
    ap.add_argument("--ext-odom", dest="ext_odom", default=str(here / "kiss" / "kiss_tum.txt"))
    ap.add_argument("--features", type=int, default=0)
    ap.add_argument("--no-odom-cov", dest="no_odom_cov", action="store_true")
    ap.add_argument("--param", action="append", default=[])
    ap.add_argument("--load", default="", help="directory with base.sse / imu.sse / lidar.sse to preload")
    ap.add_argument("--port", type=int, default=8972)
    ap.add_argument("--tcp-port", type=int, default=8082)
    a = ap.parse_args()
    if not a.imu_dir:
        a.imu_dir = str(Path(a.session).parent / "imu")
    a.rig = str(Path(a.rig).resolve()) if Path(a.rig).exists() else ""
    a.ext_odom = str(Path(a.ext_odom).resolve()) if Path(a.ext_odom).exists() else ""
    if not Path(a.imu_dir).is_dir():
        a.imu_dir = ""
    if a.load:
        for v in VARIANTS:
            f = Path(a.load) / f"{v}.sse"
            if not f.is_file():
                continue
            n = 0
            for ln in f.read_text(errors="replace").splitlines():
                if not ln.startswith("data: "):
                    continue
                try:
                    m = json.loads(ln[6:])
                except json.JSONDecodeError:
                    continue
                if m.get("type") in ("pose", "kf_update"):
                    note(v, m)
                    n += 1
            print(f"  loaded    {v}: {n} messages from {f.name}")
    Handler.args = a
    Handler.tcp_port = a.tcp_port
    print(f"live viewer:  http://127.0.0.1:{a.port}/")
    print(f"  session   {a.session}")
    print(f"  rig       {a.rig or '(없음 — imu/lidar 비활성)'}")
    print(f"  imu       {a.imu_dir or '(없음)'}")
    print(f"  ext-odom  {a.ext_odom or '(없음)'}", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    srv.daemon_threads = True
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        p = state.get("proc")
        if p and p.poll() is None:
            p.kill()
        print("\nbye")


if __name__ == "__main__":
    main()
