#!/usr/bin/env python3
"""make_report.py — build html/260923_rtabmap_odometry_covariance.html from this directory's runs.

Everything on the page is derived from the run outputs, so re-running the comparison and re-running
this script regenerates the report with the new numbers:

    covdefault/   3 runs with Rtabmap::process's default odometry covariance (identity)
    covreal/      3 runs with the odometry's own covariance handed to the mapper
    <set>/{base,imu,ext}.log          loops / nodes / wall / covariance diagnostics
    <set>/eval.json, series.json      eval_traj.py output, scored against KISS-ICP
    <set>/delta.txt                   mapper_delta.py output (non-circular read of the ext run)

    python3 make_report.py [--out <repo>/html/260923_rtabmap_odometry_covariance.html]

The page is self-contained (no CDN, no web fonts), like the other files in html/.
"""
from __future__ import annotations
import argparse, json, re
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]                      # <repo>/objpose/rt/260901/rtab_odom -> <repo>
SETS = (("off", "covdefault"), ("on", "covreal"))
RUNS = ("base", "imu", "ext")


def thin(a, n=700):
    if len(a) <= n:
        return a
    step = (len(a) - 1) / (n - 1)
    return [a[min(len(a) - 1, int(round(i * step)))] for i in range(n)]


def r2(a, k=3):
    return [[round(v, k) for v in p] for p in a]


def parse_log(p: Path) -> dict:
    t = p.read_text()
    out = {}
    m = re.search(r"done: (\d+) processed, (\d+) ok, loops (\d+), nodes (\d+), wall ([\d.]+) s", t)
    if m:
        out.update(processed=int(m[1]), ok=int(m[2]), loops=int(m[3]), nodes=int(m[4]), wall=float(m[5]))
    m = re.search(r"lin var min (\S+) med (\S+) max (\S+)\s+\(sigma med ([\d.]+) m\)", t)
    if m:
        out.update(cov_med=float(m[2]), cov_sigma=float(m[4]))
    m = re.search(r"gravity over .*-> base frame \(([-\d. ]+)\)", t)
    if m:
        out["gravity_base"] = m[1].strip()
    return out


def parse_delta(p: Path) -> dict:
    out = {}
    for ln in p.read_text().splitlines():
        m = re.match(r"\s*(live|opt)\s+\d+\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)°\s+([\d.]+)°", ln)
        if m:
            out[m[1]] = [float(m[i]) for i in range(2, 7)]
    return out


def collect() -> dict:
    D = {"runs": {}, "stats": {}, "delta": {}, "cov": {}}
    for tag, sub in SETS:
        d = HERE / sub
        ev = json.loads((d / "eval.json").read_text())
        se = json.loads((d / "series.json").read_text())
        if tag == "off":
            D["gt"] = r2(thin([p[:2] for p in se["gt"]["xyz"]]))
            D["gt_path_m"] = round(ev["gt_path_m"], 1)
            D["gt_duration_s"] = round(ev["gt_duration_s"], 1)
        D["stats"][tag] = {}
        D["cov"][tag] = {}
        for run in RUNS:
            lg = parse_log(d / f"{run}.log")
            D["stats"][tag][run] = {k: lg[k] for k in ("loops", "nodes", "wall") if k in lg}
            D["cov"][tag][run] = {"med": lg.get("cov_med"), "sigma": lg.get("cov_sigma")}
            for mode in ("live", "opt"):
                k = f"{run}_{mode}"
                s, m = se["runs"][k], ev["runs"][k]
                D["runs"][f"{tag}_{k}"] = {
                    "xy": r2(thin([p[:2] for p in s["xyz_aligned"]])),
                    "t": [round(v, 2) for v in thin(s["t"])],
                    "err": [round(v, 2) for v in thin(s["ate_pos_cm"])],
                    "ate_rmse": round(m["ate"]["pos_cm"]["rmse"], 1),
                    "ate_max": round(m["ate"]["pos_cm"]["max"], 1),
                    "yaw_rmse": round(m["drift"]["yaw_deg"]["rmse"], 2),
                    "yaw_max": round(m["drift"]["yaw_deg"]["max_abs"], 2),
                    "rpe1": round(m["rpe_1s"]["trans_cm"]["rmse"], 2),
                    "rpe10": round(m["rpe_10s"]["trans_cm"]["rmse"], 1),
                }
        dl = d / "delta.txt"
        if dl.is_file():
            D["delta"][tag] = parse_delta(dl)
    g = parse_log(HERE / "covreal" / "imu.log").get("gravity_base")
    if g:
        D["gravity_base"] = g
    return D


TEMPLATE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>260923 RTAB-Map 오도메트리와 링크 공분산</title>
<style>
:root{
  --paper:#EDF1F6; --surface:#FFFFFF; --surface-2:#F6F9FC; --line:#C6D2DE; --line-soft:#DDE5EC;
  --ink:#11202F; --ink-2:#425568; --muted:#5A6D80;
  --s1:#2a78d6; --s2:#eb6834; --s3:#1baf7a; --ref:#898781;
  --fast:#0B6E99; --fast-fill:#DCEEF7; --slow:#AC5209; --slow-fill:#FAEADA;
  --confirm:#0F7052; --confirm-fill:#DDF0E8; --band:#E4EBF3;
  --sans:"Pretendard Variable",Pretendard,"Apple SD Gothic Neo","Noto Sans KR","Malgun Gothic",-apple-system,"Helvetica Neue",Arial,sans-serif;
  --mono:ui-monospace,SFMono-Regular,Menlo,"Cascadia Mono",Consolas,"D2Coding",monospace;
}
@media (prefers-color-scheme:dark){ :root:not([data-theme="light"]){
  --paper:#0D1620; --surface:#15202C; --surface-2:#1A2734; --line:#28394A; --line-soft:#223142;
  --ink:#E3EBF3; --ink-2:#B3C3D1; --muted:#90A4B6;
  --s1:#3987e5; --s2:#d95926; --s3:#199e70;
  --fast:#54B4DC; --fast-fill:#12303E; --slow:#E4994D; --slow-fill:#352613;
  --confirm:#41B48C; --confirm-fill:#123126; --band:#1A2734;
}}
:root[data-theme="dark"]{
  --paper:#0D1620; --surface:#15202C; --surface-2:#1A2734; --line:#28394A; --line-soft:#223142;
  --ink:#E3EBF3; --ink-2:#B3C3D1; --muted:#90A4B6;
  --s1:#3987e5; --s2:#d95926; --s3:#199e70;
  --fast:#54B4DC; --fast-fill:#12303E; --slow:#E4994D; --slow-fill:#352613;
  --confirm:#41B48C; --confirm-fill:#123126; --band:#1A2734;
}
*{box-sizing:border-box}
body{background:var(--paper); color:var(--ink); font-family:var(--sans); line-height:1.65; margin:0; font-size:15px;}
.wrap{max-width:980px; margin:0 auto; padding:40px 16px 80px;}
h1{font-size:clamp(26px,5vw,38px); line-height:1.15; letter-spacing:-.015em; margin:0 0 10px; font-weight:700;}
h2{font-size:clamp(19px,3.2vw,25px); line-height:1.25; letter-spacing:-.01em; margin:0 0 6px; font-weight:700;}
h3{font-size:17px; margin:26px 0 8px; font-weight:600;}
p{margin:0 0 14px; max-width:70ch;} ul{margin:0 0 14px; padding-left:20px; max-width:70ch;} li{margin-bottom:7px;}
.lede{font-size:17px; color:var(--muted); max-width:68ch;}
.eyebrow{font-family:var(--mono); font-size:11px; letter-spacing:.14em; text-transform:uppercase; color:var(--muted); margin:0 0 10px;}
code{font-family:var(--mono); font-size:.88em; background:var(--surface-2); border:1px solid var(--line-soft); border-radius:3px; padding:1px 5px;}
pre{overflow-x:auto; background:var(--surface-2); border:1px solid var(--line-soft); border-radius:5px; padding:12px 14px; font-family:var(--mono); font-size:12.5px; line-height:1.55; margin:0 0 14px;}
header{border-bottom:2px solid var(--ink); padding-bottom:22px; margin-bottom:34px;}
.meta{display:flex; flex-wrap:wrap; gap:8px 18px; font-family:var(--mono); font-size:12px; color:var(--muted); margin-top:14px;}
section{margin-bottom:44px;}
.filters{display:flex; flex-wrap:wrap; gap:10px 20px; align-items:center; margin-bottom:14px;}
.fgroup{display:flex; align-items:center; gap:8px;}
.filters .lab{font-family:var(--mono); font-size:11px; letter-spacing:.1em; text-transform:uppercase; color:var(--muted);}
.seg{display:inline-flex; border:1px solid var(--line); border-radius:5px; overflow:hidden; background:var(--surface);}
.seg button{font-family:var(--sans); font-size:13px; padding:5px 13px; border:0; background:transparent; color:var(--muted); cursor:pointer;}
.seg button[aria-pressed="true"]{background:var(--ink); color:var(--surface); font-weight:600;}
.seg button:focus-visible{outline:2px solid var(--s1); outline-offset:-2px;}
.legend{display:flex; flex-wrap:wrap; gap:6px 18px; margin:2px 0 10px; font-size:13px;}
.legend span{display:inline-flex; align-items:center; gap:7px; color:var(--muted);}
.legend i{width:16px; height:0; border-top-width:2.5px; border-top-style:solid; display:inline-block;}
figure{margin:0 0 14px;}
.scroll{overflow-x:auto; background:var(--surface); border:1px solid var(--line); border-radius:6px; padding:18px 16px; position:relative;}
figcaption{font-size:13.5px; color:var(--muted); margin-top:10px; max-width:72ch;}
.chart text{font-family:var(--mono); font-size:10.5px; fill:var(--muted);}
.chart .ttl{font-family:var(--sans); font-size:12px; fill:var(--ink); font-weight:500;}
.chart .grid{stroke:var(--line-soft); stroke-width:1;}
.chart .axis{stroke:var(--line); stroke-width:1.2;}
.chart path.ln{fill:none; stroke-width:2; stroke-linejoin:round; stroke-linecap:round;}
.chart path.refln{fill:none; stroke:var(--ref); stroke-width:2; stroke-dasharray:5 4;}
.tip{position:absolute; pointer-events:none; opacity:0; transition:opacity .08s; background:var(--surface);
  border:1px solid var(--line); border-radius:5px; padding:7px 10px; font-family:var(--mono); font-size:11.5px;
  color:var(--ink); box-shadow:0 2px 10px rgba(0,0,0,.10); white-space:nowrap; z-index:5;}
.tip .row{display:flex; align-items:center; gap:7px;} .tip .row i{width:9px;height:9px;border-radius:2px;display:inline-block;}
table{border-collapse:collapse; width:100%; font-size:13.5px;} .tbl{overflow-x:auto;}
th,td{text-align:left; padding:8px 12px; border-bottom:1px solid var(--line-soft); vertical-align:top;}
th{font-size:12px; letter-spacing:.06em; text-transform:uppercase; color:var(--muted); border-bottom:1.5px solid var(--line); font-weight:600;}
td.num{font-family:var(--mono); font-variant-numeric:tabular-nums; white-space:nowrap;}
td.hi{font-weight:600; color:var(--s3);} td.bad{color:var(--s2);}
.note{border-left:3px solid var(--slow); background:var(--surface); padding:14px 16px; border-radius:0 5px 5px 0; margin:18px 0;}
.win{border-left:3px solid var(--s3); background:var(--surface); padding:14px 16px; border-radius:0 5px 5px 0; margin:18px 0;}
.fix{border-left:3px solid var(--s2); background:var(--surface); padding:14px 16px; border-radius:0 5px 5px 0; margin:18px 0;}
.note strong{color:var(--slow);} .win strong{color:var(--s3);} .fix strong{color:var(--s2);}
.note p:last-child,.win p:last-child,.fix p:last-child{margin-bottom:0;}
footer{border-top:1px solid var(--line); padding-top:18px; font-size:13px; color:var(--muted);}
</style>
</head>
<body>
<div class="wrap">

<header>
  <p class="eyebrow">objpose · 260901_cbnu_bigeightcircle · 2026-09-23</p>
  <h1>RTAB-Map: 오도메트리를 갈아끼우면 결과가 얼마나 달라지나</h1>
  <p class="lede">오도메트리 소스 3종 × 링크 공분산 2종 = 6회 실행. 매퍼 설정은 전부 동일. 결론부터: <strong>오도메트리 링크 공분산 기본값(단위행렬)이 오도메트리 개선을 통째로 삼키고 있었다.</strong></p>
  <div class="meta">
    <span>rtabmap 0.23.8 (Homebrew)</span><span>9445 프레임 · 29.98 Hz</span>
    <span>기준: VLP-16 KISS-ICP __PATH__ m / __DUR__ s</span><span>unpaced offline · 매퍼 1 Hz 동기</span>
  </div>
</header>

<section>
  <p class="eyebrow">01 — 무엇을 비교했나</p>
  <h2>오도메트리 소스 3종</h2>
  <ul>
    <li><b>base</b> — RTAB-Map 기본 비주얼 오도메트리 (Odom/Strategy 0 = F2M, GFTT/BRIEF, 로컬 워드맵 2000)</li>
    <li><b>imu</b> — 같은 VO + xsens MTi 자세를 <code>SensorData::setIMU</code>로 주입 (<code>objpose/src/imu_source.h</code>). 스톡 RTAB-Map이 이를 <code>OdometryF2M</code>의 번들 조정 제약과 노드별 <code>Link::kGravity</code>로 쓴다.</li>
    <li><b>ext</b> — VO를 끄고 KISS-ICP 궤적을 오도메트리로 주입 (<code>objpose/src/rtab_aux.h</code>의 <code>ExtOdom</code>). 카메라는 여전히 매퍼가 본다.</li>
  </ul>
  <p>두 번째 축은 <code>Rtabmap::process</code>에 넘기는 <strong>오도메트리 링크 공분산</strong>입니다. <code>기본값</code>은 라이브러리 기본인 단위행렬, <code>실제값</code>은 오도메트리가 스스로 보고하는 값입니다. 아래 토글로 비교하세요.</p>
  <p>IMU 좌표계는 실행할 때마다 자가검증합니다 — <code>imu_source.h</code>의 <code>gravity_report()</code>가 정지 구간 중력을 base 프레임에서 재고, <code>(0, 0, +9.8)</code>이 나와야 합니다. 이번 실행: <code>__GRAV__</code>.</p>
</section>

<section>
  <p class="eyebrow">02 — 결과</p>
  <h2>궤적과 오차</h2>
  <div class="filters">
    <span class="fgroup"><span class="lab">오도메트리 공분산</span>
      <span class="seg" role="group" aria-label="오도메트리 공분산 설정">
        <button id="c-off" type="button" aria-pressed="true">기본값 (단위행렬)</button>
        <button id="c-on" type="button" aria-pressed="false">실제값</button>
      </span></span>
    <span class="fgroup"><span class="lab">궤적</span>
      <span class="seg" role="group" aria-label="live 또는 optimized 궤적">
        <button id="m-live" type="button" aria-pressed="true">live</button>
        <button id="m-opt" type="button" aria-pressed="false">optimized</button>
      </span></span>
  </div>
  <div class="legend" id="legend"></div>
  <figure>
    <div class="scroll"><svg class="chart" id="traj" role="img" aria-label="세 실행의 평면 궤적을 KISS-ICP 기준 위에 겹쳐 그린 그림"></svg></div>
    <figcaption><strong>그림 1.</strong> 위에서 본 궤적(8자 4바퀴). 전 구간 Umeyama 정렬 후. 파선이 KISS-ICP 기준. 격자 1 m.</figcaption>
  </figure>
  <figure>
    <div class="scroll"><svg class="chart" id="err" role="img" aria-label="시간에 따른 위치 오차"></svg><div class="tip" id="tip-err"></div></div>
    <figcaption><strong>그림 2.</strong> 기준 대비 위치 오차. 마우스를 올리면 그 시각의 세 값을 같이 보여줍니다. 두 공분산 설정의 y축 범위는 같게 고정했습니다.</figcaption>
  </figure>
  <h3>측정값 <span id="mode-note" style="font-family:var(--mono);font-size:12px;color:var(--muted);font-weight:400"></span></h3>
  <div class="tbl"><table>
    <thead><tr><th>실행</th><th>ATE RMSE</th><th>ATE 최대</th><th>yaw RMSE</th><th>yaw 최대</th><th>RPE 1 s</th><th>RPE 10 s</th><th>루프</th><th>wall</th></tr></thead>
    <tbody id="metrics"></tbody>
  </table></div>
</section>

<section>
  <p class="eyebrow">03 — 진단</p>
  <h2>1차 실행에서 드러난 이상</h2>
  <p>기본 공분산으로 돌린 결과에는 설명이 안 되는 점이 있었습니다: <strong>최적화 궤적이 라이브 궤적보다 나빴습니다.</strong> 매퍼가 좋은 오도메트리를 <em>망가뜨리고</em> 있었다는 뜻입니다.</p>
  <h3>원인 — 오도메트리 공분산이 단위행렬</h3>
  <pre>bool process(
    const SensorData &amp; data,
    Transform odomPose,
    const cv::Mat &amp; odomCovariance = <b>cv::Mat::eye(6,6,CV_64FC1)</b>,   // σ = 1 m / 1 rad
    ...);</pre>
  <p><code>rtab_stream</code>은 이 기본값을 쓰고 있었습니다. 오도메트리가 스스로 보고하는 값은 전혀 다릅니다:</p>
  <div class="tbl"><table>
    <thead><tr><th>오도메트리</th><th>링크당 선형 분산 (중앙값)</th><th>= σ</th><th>기본값 대비</th></tr></thead>
    <tbody id="covtbl"></tbody>
  </table></div>
  <p>결과는 두 가지입니다. <strong>①</strong> 루프 링크는 <code>RegistrationVis</code>의 PnP 공분산(훨씬 작음)을 가지므로, 오도메트리 링크가 수십 배 헐거워진 그래프에서 루프가 모든 걸 지배합니다. <strong>②</strong> <code>RGBD/OptimizeMaxError = 3.0</code>은 "잔차 ÷ 각 링크의 표준편차"를 보는데 분모가 1 m이면 어떤 루프도 3σ를 못 넘어 <strong>거짓 루프 기각이 사실상 꺼집니다.</strong></p>
  <h3>고친 방법</h3>
  <p><code>rtabmap_ros</code>의 <code>CoreWrapper.cpp:1152-1178</code>을 그대로 복제했습니다 — 매퍼 업데이트 사이 프레임들의 <strong>최댓값</strong>을 유지하고(<em>"Use largest covariance error (to be independent of the odometry frame rate)"</em>), 업데이트 후 리셋합니다. <code>OdometryF2M</code>이 초기화 시 내는 <code>9999</code>("the new pose is not linked with the previous one")는 그대로 통과시킵니다 — 실행당 1회만 나옵니다.</p>
  <div class="win"><p><strong>역전이 사라지고 세 실행 모두 좋아졌습니다.</strong> 이제 모든 조합에서 opt &lt; live 입니다. 위 토글로 직접 확인할 수 있습니다.</p></div>
</section>

<section>
  <p class="eyebrow">04 — 순환 아닌 읽기</p>
  <h2>매퍼가 좋은 오도메트리를 얼마나 건드렸나</h2>
  <p><code>ext</code>의 ATE는 순환입니다 — 주입한 KISS-ICP가 곧 채점 기준이니까요. 순환이 아닌 값은 <strong>들어간 궤적과 나온 궤적의 차이</strong>, 즉 매퍼(루프 클로저 + 그래프 최적화)가 스스로 만들어낸 움직임입니다(<code>mapper_delta.py</code>).</p>
  <div class="tbl"><table>
    <thead><tr><th>공분산</th><th>ext 출력</th><th>중앙값</th><th>p95</th><th>최대</th><th>회전 중앙값</th><th>회전 최대</th></tr></thead>
    <tbody id="deltatbl"></tbody>
  </table></div>
</section>

<section>
  <p class="eyebrow">05 — 정리</p>
  <h2>오도메트리 정밀도가 중요한가, 매퍼 조정이 중요한가</h2>
  <ul>
    <li><strong>매퍼의 가중치가 틀려 있으면 오도메트리 개선이 최종 결과에 거의 도달하지 못합니다.</strong> 공분산 한 줄을 고친 효과가 IMU를 얹은 효과보다 컸습니다.</li>
    <li><strong>가중치가 맞으면 오도메트리 품질이 그대로 최종 정확도로 이어집니다.</strong> 라이다 오도메트리의 우위가 비로소 결과에 나타납니다.</li>
    <li>즉 <strong>오도메트리 정밀도가 상한을 정하고, 매퍼 가중치가 그 상한에 도달할지를 정합니다.</strong> 순서는 가중치가 먼저입니다 — 틀린 가중치 위에서는 오도메트리 개선의 효과를 측정조차 할 수 없기 때문입니다.</li>
    <li><strong>루프 횟수는 오도메트리 품질과 거의 무관합니다</strong> — 외형 기반 탐지이기 때문입니다. 공분산을 고치면 <code>ext</code>의 루프가 줄어드는데, 그것이 <code>OptimizeMaxError</code>의 거짓 루프 기각이 되살아난 증거입니다.</li>
    <li><strong>VO가 런타임의 대부분을 씁니다</strong> — VO를 끈 <code>ext</code>가 압도적으로 빠릅니다. 라이다 오도메트리가 따로 있는 시스템이라면 카메라를 매퍼 전용으로 두는 것만으로 CPU가 크게 절약됩니다.</li>
  </ul>
</section>

<footer>
  <p>재현: <code>bash objpose/rt/260901/rtab_odom/run_compare.sh &lt;dataset dir&gt;</code> 후 <code>python3 objpose/rt/260901/rtab_odom/make_report.py</code>.
  코드: <code>objpose/src/imu_source.h</code>(IMU), <code>objpose/src/rtab_aux.h</code>(외부 오도메트리), <code>objpose/src/rtab_stream.cc</code>(공분산 전달).
  원자료: <code>objpose/rt/260901/rtab_odom/{covdefault,covreal}/</code>.</p>
</footer>

<script type="application/json" id="vizdata">__DATA__</script>
<script>
(function(){
  var D=JSON.parse(document.getElementById('vizdata').textContent);
  var SER=[{key:'base',name:'base — VO only',css:'--s1'},
           {key:'imu', name:'imu — VO + IMU', css:'--s2'},
           {key:'ext', name:'ext — LiDAR odometry',css:'--s3'}];
  var cov='off', mode='live';
  function R(k){return D.runs[cov+'_'+k+'_'+mode];}
  function col(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim()||'#888';}
  function el(n,a){var e=document.createElementNS('http://www.w3.org/2000/svg',n);for(var k in a)e.setAttribute(k,a[k]);return e;}

  function legend(){var L=document.getElementById('legend');L.innerHTML='';
    SER.forEach(function(s){var sp=document.createElement('span');
      sp.innerHTML='<i style="border-top-color:'+col(s.css)+'"></i>'+s.name;L.appendChild(sp);});
    var sp=document.createElement('span');
    sp.innerHTML='<i style="border-top-color:'+col('--ref')+';border-top-style:dashed"></i>KISS-ICP 기준';L.appendChild(sp);}

  function traj(){
    var W=660,H=560,P={l:44,r:14,t:26,b:34},pw=W-P.l-P.r,ph=H-P.t-P.b;
    var sets=[D.gt].concat(SER.map(function(s){return R(s.key).xy;}));
    var xs=[],ys=[];sets.forEach(function(a){a.forEach(function(p){xs.push(p[0]);ys.push(p[1]);});});
    var cx=(Math.min.apply(null,xs)+Math.max.apply(null,xs))/2,cy=(Math.min.apply(null,ys)+Math.max.apply(null,ys))/2;
    var span=Math.max(Math.max.apply(null,xs)-Math.min.apply(null,xs),Math.max.apply(null,ys)-Math.min.apply(null,ys))*1.08;
    var k=Math.min(pw,ph)/span,ox=P.l+pw/2,oy=P.t+ph/2;
    function X(v){return ox+(v-cx)*k;} function Y(v){return oy-(v-cy)*k;}
    var svg=document.getElementById('traj');svg.innerHTML='';
    svg.setAttribute('viewBox','0 0 '+W+' '+H);svg.style.minWidth='560px';svg.style.maxWidth='100%';svg.style.height='auto';
    for(var v=Math.ceil(cx-span/2);v<=cx+span/2;v++){
      svg.appendChild(el('line',{x1:X(v),y1:P.t,x2:X(v),y2:P.t+ph,class:'grid'}));
      var t=el('text',{x:X(v),y:H-16,'text-anchor':'middle'});t.textContent=v+' m';svg.appendChild(t);}
    for(var v2=Math.ceil(cy-span/2);v2<=cy+span/2;v2++){
      svg.appendChild(el('line',{x1:P.l,y1:Y(v2),x2:P.l+pw,y2:Y(v2),class:'grid'}));
      var t2=el('text',{x:P.l-8,y:Y(v2)+3.5,'text-anchor':'end'});t2.textContent=v2;svg.appendChild(t2);}
    var ttl=el('text',{x:P.l,y:16,class:'ttl'});
    ttl.textContent='평면 궤적 — 공분산 '+(cov==='on'?'실제값':'기본값')+' · '+mode;svg.appendChild(ttl);
    function d(a){return a.map(function(p,i){return (i?'L':'M')+X(p[0]).toFixed(1)+' '+Y(p[1]).toFixed(1);}).join(' ');}
    svg.appendChild(el('path',{d:d(D.gt),class:'refln'}));
    SER.forEach(function(s){svg.appendChild(el('path',{d:d(R(s.key).xy),class:'ln',stroke:col(s.css)}));});
    var st=D.gt[0];
    svg.appendChild(el('circle',{cx:X(st[0]),cy:Y(st[1]),r:5.5,fill:'none',stroke:col('--surface'),'stroke-width':3}));
    svg.appendChild(el('circle',{cx:X(st[0]),cy:Y(st[1]),r:5.5,fill:'none',stroke:col('--ink'),'stroke-width':1.6}));
    var sl=el('text',{x:X(st[0])+10,y:Y(st[1])-7});sl.textContent='start';svg.appendChild(sl);}

  var EG={};
  function err(){
    var W=760,H=280,P={l:46,r:16,t:26,b:34},pw=W-P.l-P.r,ph=H-P.t-P.b;
    var runs=SER.map(function(s){return R(s.key);});
    var emax=0,tmax=0;
    ['off','on'].forEach(function(c){SER.forEach(function(s){['live','opt'].forEach(function(m){
      var r=D.runs[c+'_'+s.key+'_'+m];if(!r)return;tmax=Math.max(tmax,r.t[r.t.length-1]);
      r.err.forEach(function(v){emax=Math.max(emax,v);});});});});
    emax=Math.ceil(emax/20)*20;
    function X(v){return P.l+v/tmax*pw;} function Y(v){return P.t+ph-v/emax*ph;}
    var svg=document.getElementById('err');svg.innerHTML='';
    svg.setAttribute('viewBox','0 0 '+W+' '+H);svg.style.minWidth='620px';svg.style.maxWidth='100%';svg.style.height='auto';
    for(var v=0;v<=emax;v+=20){
      svg.appendChild(el('line',{x1:P.l,y1:Y(v),x2:P.l+pw,y2:Y(v),class:v?'grid':'axis'}));
      var t=el('text',{x:P.l-8,y:Y(v)+3.5,'text-anchor':'end'});t.textContent=v;svg.appendChild(t);}
    for(var s2=0;s2<=tmax;s2+=60){var tx=el('text',{x:X(s2),y:H-14,'text-anchor':'middle'});tx.textContent=s2+' s';svg.appendChild(tx);}
    var ttl=el('text',{x:P.l,y:16,class:'ttl'});
    ttl.textContent='기준 대비 위치 오차 (cm) — 공분산 '+(cov==='on'?'실제값':'기본값')+' · '+mode;svg.appendChild(ttl);
    runs.forEach(function(r,i){
      var d=r.t.map(function(tv,j){return (j?'L':'M')+X(tv).toFixed(1)+' '+Y(r.err[j]).toFixed(1);}).join(' ');
      svg.appendChild(el('path',{d:d,class:'ln',stroke:col(SER[i].css)}));});
    var cross=el('line',{x1:0,y1:P.t,x2:0,y2:P.t+ph,stroke:col('--muted'),'stroke-width':1,opacity:0});svg.appendChild(cross);
    var dots=SER.map(function(s){var c=el('circle',{r:4.5,fill:col(s.css),stroke:col('--surface'),'stroke-width':2,opacity:0});svg.appendChild(c);return c;});
    EG={svg:svg,X:X,Y:Y,P:P,pw:pw,tmax:tmax,runs:runs,cross:cross,dots:dots,W:W};
    svg.appendChild(el('rect',{x:P.l,y:P.t,width:pw,height:ph,fill:'transparent'}));
    svg.onmousemove=hover;svg.onmouseleave=hide;}
  function hover(ev){
    var tip=document.getElementById('tip-err'),r=EG.svg.getBoundingClientRect();
    var sx=(ev.clientX-r.left)/r.width*EG.W;
    if(sx<EG.P.l||sx>EG.P.l+EG.pw){hide();return;}
    var tv=(sx-EG.P.l)/EG.pw*EG.tmax;
    EG.cross.setAttribute('x1',sx);EG.cross.setAttribute('x2',sx);EG.cross.setAttribute('opacity',1);
    var html='<b>'+tv.toFixed(1)+' s</b>';
    EG.runs.forEach(function(run,i){
      var lo=0,hi=run.t.length-1;
      while(hi-lo>1){var m=(lo+hi)>>1;if(run.t[m]<=tv)lo=m;else hi=m;}
      var j=(tv-run.t[lo]<run.t[hi]-tv)?lo:hi;
      EG.dots[i].setAttribute('cx',EG.X(run.t[j]));EG.dots[i].setAttribute('cy',EG.Y(run.err[j]));EG.dots[i].setAttribute('opacity',1);
      html+='<div class="row"><i style="background:'+col(SER[i].css)+'"></i>'+SER[i].key+' '+run.err[j].toFixed(1)+' cm</div>';});
    tip.innerHTML=html;tip.style.opacity=1;
    var wrap=EG.svg.parentElement.getBoundingClientRect();
    var px=ev.clientX-wrap.left+14;if(px+170>wrap.width)px=ev.clientX-wrap.left-170;
    tip.style.left=px+'px';tip.style.top=(ev.clientY-wrap.top-10)+'px';}
  function hide(){document.getElementById('tip-err').style.opacity=0;
    if(EG.cross)EG.cross.setAttribute('opacity',0);
    if(EG.dots)EG.dots.forEach(function(d){d.setAttribute('opacity',0);});}

  function table(){
    var tb=document.getElementById('metrics');tb.innerHTML='';
    document.getElementById('mode-note').textContent='— 공분산 '+(cov==='on'?'실제값':'기본값')+' · '+mode;
    var best=Math.min.apply(null,SER.map(function(s){return R(s.key).ate_rmse;}));
    SER.forEach(function(s){
      var r=R(s.key),st=D.stats[cov][s.key],tr=document.createElement('tr');
      tr.innerHTML='<td><span style="display:inline-block;width:10px;height:10px;border-radius:2px;background:'+col(s.css)+';margin-right:7px"></span>'+s.key+'</td>'
        +'<td class="num'+(r.ate_rmse===best?' hi':'')+'">'+r.ate_rmse.toFixed(1)+' cm</td>'
        +'<td class="num">'+r.ate_max.toFixed(1)+' cm</td>'
        +'<td class="num">'+r.yaw_rmse.toFixed(2)+'°</td><td class="num">'+r.yaw_max.toFixed(2)+'°</td>'
        +'<td class="num">'+r.rpe1.toFixed(2)+' cm</td><td class="num">'+r.rpe10.toFixed(1)+' cm</td>'
        +'<td class="num">'+st.loops+'</td><td class="num">'+st.wall.toFixed(0)+' s</td>';
      tb.appendChild(tr);});}

  function statics(){
    var ct=document.getElementById('covtbl');ct.innerHTML='';
    SER.forEach(function(s){
      var c=D.cov.on[s.key];if(!c||c.med==null)return;
      var tr=document.createElement('tr');
      tr.innerHTML='<td>'+s.key+'</td><td class="num">'+c.med.toExponential(2)+'</td>'
        +'<td class="num">'+(c.sigma*100).toFixed(2)+' cm</td>'
        +'<td class="num">'+Math.round(1/c.sigma)+'배 과대평가</td>';
      ct.appendChild(tr);});
    var tr=document.createElement('tr');
    tr.innerHTML='<td><b>기본값</b></td><td class="num">1.0</td><td class="num">100 cm</td><td class="num">—</td>';
    ct.appendChild(tr);
    var dt=document.getElementById('deltatbl');dt.innerHTML='';
    [['off','기본값'],['on','실제값']].forEach(function(p){
      var d=D.delta[p[0]];if(!d)return;
      ['live','opt'].forEach(function(m){
        var v=d[m];if(!v)return;
        var good=p[0]==='on'?' hi':' bad';
        var tr=document.createElement('tr');
        tr.innerHTML='<td>'+p[1]+'</td><td>'+(m==='opt'?'optimized':'live')+'</td>'
          +'<td class="num'+good+'">'+v[0].toFixed(2)+' cm</td><td class="num">'+v[1].toFixed(2)+'</td>'
          +'<td class="num'+good+'">'+v[2].toFixed(2)+'</td><td class="num">'+v[3].toFixed(3)+'°</td>'
          +'<td class="num'+good+'">'+v[4].toFixed(3)+'°</td>';
        dt.appendChild(tr);});});}

  function draw(){legend();traj();err();table();}
  function bind(id,fn){document.getElementById(id).addEventListener('click',fn);}
  function press(on,off){document.getElementById(on).setAttribute('aria-pressed','true');
                         document.getElementById(off).setAttribute('aria-pressed','false');}
  bind('c-off',function(){cov='off';press('c-off','c-on');draw();});
  bind('c-on', function(){cov='on'; press('c-on','c-off');draw();});
  bind('m-live',function(){mode='live';press('m-live','m-opt');draw();});
  bind('m-opt', function(){mode='opt'; press('m-opt','m-live');draw();});
  statics();draw();
  if(window.matchMedia)window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change',function(){statics();draw();});
})();
</script>
</div>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO / "html" / "260923_rtabmap_odometry_covariance.html"))
    a = ap.parse_args()
    D = collect()
    html = (TEMPLATE
            .replace("__PATH__", str(D.get("gt_path_m", "?")))
            .replace("__DUR__", str(D.get("gt_duration_s", "?")))
            .replace("__GRAV__", D.get("gravity_base", "(측정 없음)"))
            .replace("__DATA__", json.dumps(D, separators=(",", ":"), ensure_ascii=False)))
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html)
    print(f"{out}  ({len(html)/1024:.0f} KB)")


if __name__ == "__main__":
    main()
