#!/usr/bin/env python3
"""make_jitter_report.py — HTML report: LiDAR (KISS-ICP) position jitter on 260910_object, cause and live fix.

Reads the measured results (live runs v3/v4/v5, offline variant scores, ORB-SLAM3 final trajectory) and writes a
self-contained page in the style of CLI_environment/html/*.html:

    python objpose/lidar/make_jitter_report.py [--out html/260910_LiDAR_jitter_report.html]

A second file <out>.artifact.html (scratch, not for the repo) holds the same page without the <html>/<head>/<body> wrapper.
"""
from __future__ import annotations

import argparse
import html
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "integration"))
sys.path.insert(0, str(ROOT / "objpose" / "pc"))
sys.path.insert(0, str(HERE))
from analyze_hdl_wobble import plane_axes  # noqa: E402
from fusion import mat  # noqa: E402
from rig_offset import read_tum  # noqa: E402
from smooth_lidar_traj import accel, umeyama_sim3  # noqa: E402

OUT = ROOT / "objpose" / "output"


def load_run(name):
    S = json.loads((OUT / name / "summary.json").read_text())
    P = sorted((json.loads(l) for l in open(OUT / name / "slam_poses.jsonl")), key=lambda m: m["t_ns"])
    return S, P


# ── SVG helpers ─────────────────────────────────────────────────────────────
def esc(s):
    return html.escape(str(s))


def path(xs, ys):
    return "M" + " L".join(f"{x:.1f},{y:.1f}" for x, y in zip(xs, ys))


def fig_accel(t3, a3, t5, a5, spikes):
    W, H, L, R, T, B = 720, 250, 46, 14, 16, 34
    x = lambda t: L + (t / 160.0) * (W - L - R)          # noqa: E731
    y = lambda v: T + (1 - min(v, 3.0) / 3.0) * (H - T - B)  # noqa: E731
    g = []
    for v in (0, 1, 2):
        g.append(f'<line x1="{L}" x2="{W-R}" y1="{y(v):.1f}" y2="{y(v):.1f}" style="stroke:var(--line)" />'
                 f'<text x="{L-8}" y="{y(v)+4:.1f}" text-anchor="end" style="fill:var(--ink-3)" font-size="11">{v}</text>')
    for tv in range(0, 161, 20):
        g.append(f'<text x="{x(tv):.1f}" y="{H-12}" text-anchor="middle" style="fill:var(--ink-3)" font-size="11">{tv}</text>')
    g.append(f'<text x="{L}" y="{T+2}" dx="14" style="fill:var(--ink-3)" font-size="11">데이터 시각 [s] →</text>')
    g.append(f'<text x="{L-38}" y="{T+4}" style="fill:var(--ink-3)" font-size="11">cm</text>')
    for s in spikes:
        g.append(f'<line x1="{x(s):.1f}" x2="{x(s):.1f}" y1="{T}" y2="{H-B}" style="stroke:var(--fail);opacity:.18" />')
    g.append(f'<path d="{path([x(t) for t in t3], [y(v) for v in a3])}" fill="none" style="stroke:var(--ink-3)" stroke-width="1.2" />')
    g.append(f'<path d="{path([x(t) for t in t5], [y(v) for v in a5])}" fill="none" style="stroke:var(--lidar)" stroke-width="1.6" />')
    lx = W - R - 250
    g.append(f'<line x1="{lx}" x2="{lx+22}" y1="{T+8}" y2="{T+8}" style="stroke:var(--ink-3)" stroke-width="2" />'
             f'<text x="{lx+28}" y="{T+12}" style="fill:var(--ink-2)" font-size="12">기존 (v3)</text>'
             f'<line x1="{lx+110}" x2="{lx+132}" y1="{T+8}" y2="{T+8}" style="stroke:var(--lidar)" stroke-width="2.5" />'
             f'<text x="{lx+138}" y="{T+12}" style="fill:var(--ink-2)" font-size="12">적용 후 (v5)</text>')
    return (f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="스캔 간 가속 시계열, 기존과 적용 후">{"".join(g)}</svg>')


def fig_zoom(v3, v5, orb):
    allp = np.array(v3 + v5 + orb)
    lo, hi = allp.min(0), allp.max(0)
    ext = (hi - lo) * 1.08
    c = (lo + hi) / 2
    W, pad = 680, 30
    k = (W - 2 * pad) / float(ext[0])                       # px per metre, same on both axes
    H = int(ext[1] * k + 2 * pad + 70)                      # + room for the legend above the path
    span = (W - 2 * pad) / k
    sx = lambda p: pad + (p[0] - c[0]) * k + (W - 2 * pad) / 2            # noqa: E731
    sy = lambda p: H - pad - 20 - (p[1] - c[1]) * k - ext[1] * k / 2       # noqa: E731
    g = []
    g.append(f'<path d="{path([sx(p) for p in orb], [sy(p) for p in orb])}" fill="none" style="stroke:var(--orb)" stroke-width="2" />')
    g.append(f'<path d="{path([sx(p) for p in v3], [sy(p) for p in v3])}" fill="none" style="stroke:var(--ink-3)" stroke-width="1.3" />')
    for p in v3:
        g.append(f'<circle cx="{sx(p):.1f}" cy="{sy(p):.1f}" r="2.4" style="fill:var(--ink-3)" />')
    g.append(f'<path d="{path([sx(p) for p in v5], [sy(p) for p in v5])}" fill="none" style="stroke:var(--lidar)" stroke-width="2.2" />')
    for p in v5:
        g.append(f'<circle cx="{sx(p):.1f}" cy="{sy(p):.1f}" r="2.6" style="fill:var(--lidar)" />')
    # 5 cm scale bar
    px = 0.05 / span * (W - 2 * pad)
    g.append(f'<line x1="{pad}" x2="{pad+px:.1f}" y1="{H-12}" y2="{H-12}" style="stroke:var(--ink)" stroke-width="2" />'
             f'<text x="{pad+px+6:.1f}" y="{H-8}" style="fill:var(--ink-2)" font-size="12">5 cm</text>')
    g.append(f'<g font-size="12"><rect x="{W-178}" y="10" width="168" height="66" style="fill:var(--surface);stroke:var(--line)" />'
             f'<line x1="{W-168}" x2="{W-146}" y1="26" y2="26" style="stroke:var(--ink-3)" stroke-width="2" /><text x="{W-140}" y="30" style="fill:var(--ink-2)">라이다 기존 (v3)</text>'
             f'<line x1="{W-168}" x2="{W-146}" y1="44" y2="44" style="stroke:var(--lidar)" stroke-width="2.5" /><text x="{W-140}" y="48" style="fill:var(--ink-2)">라이다 적용 후 (v5)</text>'
             f'<line x1="{W-168}" x2="{W-146}" y1="62" y2="62" style="stroke:var(--orb)" stroke-width="2.5" /><text x="{W-140}" y="66" style="fill:var(--ink-2)">ORB-SLAM3 최종</text></g>')
    return f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="106-111.5 s 수평 궤적 확대">{"".join(g)}</svg>'


def fig_corr(rows):
    W, L, R, rowh, T = 720, 190, 40, 34, 10
    H = T + rowh * len(rows) + 30
    x = lambda v: L + (v + 0.2) / 1.2 * (W - L - R)   # noqa: E731
    g = [f'<line x1="{x(0):.1f}" x2="{x(0):.1f}" y1="{T}" y2="{H-26}" style="stroke:var(--ink-3)" />']
    for v in (-0.2, 0, 0.2, 0.4, 0.6, 0.8, 1.0):
        g.append(f'<text x="{x(v):.1f}" y="{H-8}" text-anchor="middle" style="fill:var(--ink-3)" font-size="11">{v:g}</text>')
    for i, (name, v, key) in enumerate(rows):
        yc = T + i * rowh + rowh / 2
        col = "var(--fail)" if key else "var(--ink-3)"
        x0, x1 = sorted((x(0), x(v)))
        g.append(f'<text x="{L-12}" y="{yc+4:.1f}" text-anchor="end" style="fill:var(--ink-2)" font-size="13">{esc(name)}</text>'
                 f'<rect x="{x0:.1f}" y="{yc-8:.1f}" width="{max(x1-x0, 2):.1f}" height="16" rx="2" style="fill:{col}"><title>{esc(name)}: {v:+.2f}</title></rect>'
                 f'<text x="{max(x1, x(0))+6:.1f}" y="{yc+4:.1f}" style="fill:var(--ink)" font-size="12" font-family="var(--mono)">{v:+.2f}</text>')
    return f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="튐 크기와 각 요인의 순위상관">{"".join(g)}</svg>'


def fig_dots(rows):
    W, L, R, rowh, T = 720, 230, 30, 32, 26
    H = T + rowh * len(rows) + 30
    xmax = 2.5
    x = lambda v: L + v / xmax * (W - L - R)   # noqa: E731
    g = []
    for v in (0, 0.5, 1.0, 1.5, 2.0, 2.5):
        g.append(f'<line x1="{x(v):.1f}" x2="{x(v):.1f}" y1="{T-6}" y2="{H-26}" style="stroke:var(--line)" />'
                 f'<text x="{x(v):.1f}" y="{H-8}" text-anchor="middle" style="fill:var(--ink-3)" font-size="11">{v:g} cm</text>')
    g.append(f'<g font-size="12"><circle cx="{L+4}" cy="10" r="5" style="fill:var(--surface);stroke:var(--accent)" stroke-width="2" />'
             f'<text x="{L+14}" y="14" style="fill:var(--ink-2)">전체 스캔</text>'
             f'<circle cx="{L+104}" cy="10" r="5" style="fill:var(--fail)" /><text x="{L+114}" y="14" style="fill:var(--ink-2)">튀던 스캔(상위 1 %)</text></g>')
    for i, (name, allv, spk, best) in enumerate(rows):
        yc = T + i * rowh + rowh / 2
        wt = "700" if best else "400"
        g.append(f'<text x="{L-12}" y="{yc+4:.1f}" text-anchor="end" style="fill:var(--ink)" font-size="13" font-weight="{wt}">{esc(name)}</text>'
                 f'<line x1="{x(allv):.1f}" x2="{x(spk):.1f}" y1="{yc:.1f}" y2="{yc:.1f}" style="stroke:var(--line)" stroke-width="3" />'
                 f'<circle cx="{x(allv):.1f}" cy="{yc:.1f}" r="5.5" style="fill:var(--surface);stroke:var(--accent)" stroke-width="2"><title>{esc(name)} 전체 {allv:.2f} cm</title></circle>'
                 f'<circle cx="{x(spk):.1f}" cy="{yc:.1f}" r="5.5" style="fill:var(--fail)"><title>{esc(name)} 튀던 스캔 {spk:.2f} cm</title></circle>')
    return f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="방식별 ORB 대비 고주파 차">{"".join(g)}</svg>'


def fig_budget(rows, delay):
    W, L, R, rowh, T = 720, 220, 30, 40, 14
    H = T + rowh * len(rows) + 34
    xmax = 0.5
    x = lambda v: L + v / xmax * (W - L - R)   # noqa: E731
    g = []
    for v in (0, 0.1, 0.2, 0.3, 0.4, 0.5):
        g.append(f'<text x="{x(v):.1f}" y="{H-10}" text-anchor="middle" style="fill:var(--ink-3)" font-size="11">{v:g} s</text>')
    for i, (name, segs, ok) in enumerate(rows):
        yc = T + i * rowh + rowh / 2
        acc = 0.0
        g.append(f'<text x="{L-12}" y="{yc+4:.1f}" text-anchor="end" style="fill:var(--ink)" font-size="13">{esc(name)}</text>')
        for label, v, style in segs:
            g.append(f'<rect x="{x(acc)+1:.1f}" y="{yc-11:.1f}" width="{max(x(acc+v)-x(acc)-2, 1):.1f}" height="22" rx="2" style="{style}"><title>{esc(label)} {v:.3f} s</title></rect>')
            if x(acc + v) - x(acc) > 54:
                g.append(f'<text x="{(x(acc)+x(acc+v))/2:.1f}" y="{yc+4:.1f}" text-anchor="middle" font-size="11" style="fill:var(--ink)">{esc(label)}</text>')
            acc += v
        mark, col = {True: ("✓", "var(--pass)"), False: ("✗", "var(--fail)"), None: ("△", "var(--warn)")}[ok]
        g.append(f'<text x="{x(acc)+8:.1f}" y="{yc+5:.1f}" font-size="14" font-weight="700" style="fill:{col}">{mark} {acc:.2f} s</text>')
    g.append(f'<line x1="{x(delay):.1f}" x2="{x(delay):.1f}" y1="{T-4}" y2="{H-26}" style="stroke:var(--warn)" stroke-width="2" stroke-dasharray="5 4" />'
             f'<text x="{x(delay)+5:.1f}" y="{T+6}" font-size="11" style="fill:var(--warn)">화면 표시 지연 {delay:g} s</text>')
    return f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="화면 보간에 필요한 포즈 도착 시간">{"".join(g)}</svg>'


CSS_SOURCE = ROOT / "html" / "SLAM_3method_status_report.html"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "html" / "260910_LiDAR_jitter_report.html"))
    a = ap.parse_args()

    css = re.search(r"<style>(.*?)</style>", CSS_SOURCE.read_text(), re.S).group(1)
    css += """
figure svg text{font-family:var(--sans)}
.two{display:grid;grid-template-columns:1fr 1fr;gap:14px}
@media(max-width:700px){.two{grid-template-columns:1fr}}
td.best{color:var(--lidar);font-weight:700}
.tablebox.wrapcells td,.tablebox.wrapcells th{white-space:normal}
.chip{white-space:nowrap}
.tablebox.wrapth th{white-space:normal;vertical-align:bottom}
"""

    S3, P3 = load_run("live_260910_lidar_v3")
    S4, P4 = load_run("live_260910_lidar_v4")
    S5, P5 = load_run("live_260910_lidar_v5")
    first_ns = 1789033551145180928                       # first velodyne scan (hello first_ns)
    rel = lambda t_ns: (np.asarray(t_ns) - first_ns) / 1e9   # noqa: E731
    t3 = rel([m["t_ns"] for m in P3]); T3 = np.array([mat(m["T_wc"]) for m in P3])
    t5 = rel([m["t_ns"] for m in P5]); T5 = np.array([mat(m["T_wc"]) for m in P5])
    a3, a5 = accel(T3[:, :3, 3]), accel(T5[:, :3, 3])
    spikes = t3[a3 > np.percentile(a3, 99)]

    # 106-111.5 s horizontal close-up, ORB-SLAM3 final carried into the LiDAR-camera world
    e1, e2, _ = plane_axes(T3[:, :3, 3])
    T_lidar_cam = np.linalg.inv(np.asarray(json.loads((ROOT / "objpose/rt/260910/T_cam_lidar_final.json").read_text())["T_cam_lidar"]))
    to, To = read_tum(ROOT / "objpose/rt/260910/slam_orb_tum.txt.optimized.txt")
    tor = to - first_ns / 1e9
    ok = (t3 > tor[0]) & (t3 < tor[-1])
    C3 = np.array([(M @ T_lidar_cam)[:3, 3] for M in T3])
    C5 = np.array([(M @ T_lidar_cam)[:3, 3] for M in T5])
    Po = np.array([np.interp(t3[ok], tor, To[:, j, 3]) for j in range(3)]).T
    s, Rm, tt = umeyama_sim3(Po, C3[ok])
    lo, hi = 106.0, 111.5
    xy = lambda P: [(float(p @ e1), float(p @ e2)) for p in P]   # noqa: E731
    zoom_v3 = xy(C3[(t3 > lo) & (t3 < hi)])
    zoom_v5 = xy(C5[(t5 > lo) & (t5 < hi)])
    m = (tor > lo) & (tor < hi)
    zoom_orb = xy(s * To[m, :3, 3] @ Rm.T + tt)

    off = json.loads((HERE / "eval_lidar_variants_260910.json").read_text())
    live = json.loads((OUT / "live_260910_lidar_v5/traj_eval/eval.json").read_text())
    cmp53 = json.loads((OUT / "compare_260910_lidar_v5_vs_v3.json").read_text())
    kv = json.loads((HERE / "kiss_variants_260910/summary_run1.json").read_text())
    kv.update(json.loads((HERE / "kiss_variants_260910/summary_iter.json").read_text()))

    def disp(S, name):
        t0 = S["t0_ns"]
        by = {}
        for l in open(OUT / name / "display_objects.jsonl"):
            x = json.loads(l)
            by.setdefault(x["t_ns"], x)
        ts = np.array(sorted(by))
        r = (ts - t0) / 1e9
        sel = (r >= 50) & (r < 160)
        return (100 * float(np.mean([by[t]["slam"] is None for t in ts[sel]])),
                float(np.mean([len(by[t]["objects"]) for t in ts[sel]])))
    d3, d4, d5 = disp(S3, "live_260910_lidar_v3"), disp(S4, "live_260910_lidar_v4"), disp(S5, "live_260910_lidar_v5")
    lag = lambda P, k: 1000 * float(np.median([p[k] for p in P if p.get(k) is not None]))   # noqa: E731
    lag3, lag4, lag5, ref5 = lag(P3, "recv_lag_s"), lag(P4, "recv_lag_s"), lag(P5, "recv_lag_s"), lag(P5, "refine_lag_s")

    def mem(S):
        from collections import Counter
        c = Counter(l["status"] for l in S["memory_landmarks"])
        names = {l["name"] for l in S["memory_landmarks"] if l["status"] in ("active", "lost", "remembered")}
        return c, len(names)
    m3, m4, m5 = mem(S3), mem(S4), mem(S5)

    base, fin, raw5 = live["base"], live["v5_live_final"], live["v5_live_raw"]
    runs = cmp53["runs"]
    obj_diff = [v["distance_cm"] for v in cmp53["objects_B_in_A_map"].values() if "distance_cm" in v]

    V = lambda k: off[k]   # noqa: E731
    variants = [
        ("기존 (등속 deskew)", "base", kv["base"]["ms_per_scan"], "0"),
        ("deskew 끔", "nodeskew", kv["nodeskew"]["ms_per_scan"], "0"),
        ("반복 deskew 3회", "iterdeskew", kv["iterdeskew"]["ms_per_scan"], "0"),
        ("반복 deskew 2회", "iter2", kv["iter2"]["ms_per_scan"], "0"),
        ("복셀 0.05 m", "voxel05", kv["voxel05"]["ms_per_scan"], "0"),
        ("기존 + 5 스캔 평균", "base+box5", kv["base"]["ms_per_scan"], "0.2 s"),
        ("반복 2회 + 5 스캔 평균", "iter2+box5", kv["iter2"]["ms_per_scan"], "0.2 s"),
    ]
    vrows = []
    for label, k, ms, look in variants:
        r = V(k)
        best = k == "iter2+box5"
        cls = ' class="num best"' if best else ' class="num"'
        vrows.append(f"<tr><td>{'<b>' + label + '</b>' if best else label}</td><td class=\"num\">{ms:.1f}</td><td class=\"num\">{look}</td>"
                     f"<td class=\"num\">{r['accel_cm']['p99']:.2f}</td>"
                     f"<td{cls}>{r['hf_vs_orb_cm']['rms']:.2f}</td><td{cls}>{r['hf_vs_orb_cm']['at_base_spikes_rms']:.2f}</td>"
                     f"<td class=\"num\">{r['hf_vs_orb_cm']['fast_turn_gt40dps_rms']:.2f}</td>"
                     f"<td class=\"num\">{r['lowfreq_vs_orb_rmse_cm']:.2f}</td>"
                     f"<td class=\"num\">{r['object_spread_cm']['median']:.2f} / {r['object_spread_cm']['p90']:.2f}</td></tr>")
    dots = fig_dots([(label, V(k)["hf_vs_orb_cm"]["rms"], V(k)["hf_vs_orb_cm"]["at_base_spikes_rms"], k == "iter2+box5")
                     for label, k, _, _ in variants])

    corr = fig_corr([("선가속 (속도 변화)", 0.70, True), ("요 각가속 (회전 변화)", 0.63, True), ("요 속도", 0.53, False),
                     ("이동 속도", 0.48, False), ("기하 퇴화 (제약 고유값 비)", -0.01, False)])
    budget = fig_budget([
        ("v3 기존 — 원시 포즈", [("도착", lag3 / 1000, "fill:var(--accent-soft);stroke:var(--accent)"),
                              ("다음 스캔", 0.1009, "fill:var(--surface-2);stroke:var(--line)")], True),
        ("v4 — 평균 포즈만 전송", [("PC 도착", lag4 / 1000, "fill:color-mix(in srgb,var(--fail) 16%,transparent);stroke:var(--fail)"),
                               ("다음 스캔", 0.1009, "fill:var(--surface-2);stroke:var(--line)")], False),
        ("v5 — 원시 포즈 즉시", [("도착", lag5 / 1000, "fill:var(--accent-soft);stroke:var(--accent)"),
                             ("다음 스캔", 0.1009, "fill:var(--surface-2);stroke:var(--line)")], True),
        ("v5 — 평균 포즈 교체", [("PC 도착", ref5 / 1000, "fill:color-mix(in srgb,var(--lidar) 16%,transparent);stroke:var(--lidar)"),
                             ("다음 스캔", 0.1009, "fill:var(--surface-2);stroke:var(--line)")], None),
    ], S5["display_delay_s"])

    first = {}
    for name, S in (("v3", S3), ("v4", S4), ("v5", S5)):
        f = {}
        for l in open(OUT / f"live_260910_lidar_{name}" / "display_objects.jsonl"):
            x = json.loads(l)
            for o in x["objects"]:
                f.setdefault(o["name"], (x["t_ns"] - first_ns) / 1e9)
        first[name] = f

    body = f"""
<header>
  <div class="wrap">
    <div class="eyebrow">260910_object · VLP-16 라이다 SLAM · 실시간 객체 위치 시스템</div>
    <h1>라이다 위치가 흔들린 이유와 실시간 보정 결과</h1>
    <p class="lead">Dataset 폴더의 260910_object 에서 라이다 SLAM(KISS-ICP) 위치가 순간적으로 1.5–2.7 cm 씩 튀었다.
      원인은 <b>등속 가정 스캔 왜곡 보정(deskew)</b>이었고, <b>반복 deskew + 5 스캔 평균</b>을 Mac 라이다 스트리머에 넣어
      실시간으로 다시 돌렸다. 첫 적용에서 생긴 물체 표시 끊김도 원인을 찾아 고쳤다.</p>
    <div class="stats">
      <div class="stat"><span class="v"><span style="color:var(--ink-3)">{base['accel_cm']['at_base_spikes_median']:.2f}</span> → <span style="color:var(--lidar)">{fin['accel_cm']['at_base_spikes_median']:.2f}</span></span><span class="k">튀던 순간의 스캔 간 가속 [cm]</span></div>
      <div class="stat"><span class="v"><span style="color:var(--ink-3)">{base['hf_vs_orb_cm']['rms']:.2f}</span> → <span style="color:var(--lidar)">{fin['hf_vs_orb_cm']['rms']:.2f}</span></span><span class="k">ORB-SLAM3 대비 고주파 차 [cm]</span></div>
      <div class="stat"><span class="v">{lag5:.0f} ms</span><span class="k">포즈 도착 지연 (화면 끊김 {d5[0]:.0f} %)</span></div>
      <div class="stat"><span class="v" style="color:var(--pass)">{m5[1]} = 8 종</span><span class="k">객체 메모리 (기존과 동일)</span></div>
    </div>
  </div>
</header>

<main class="wrap">

<div class="note"><b>대상 확인.</b> 질문은 hdl_graph_slam 이었지만 Dataset 폴더(<code>~/DeepLearning/Dataset/260910_object</code>)에서
돌린 라이다 SLAM 은 <b>KISS-ICP</b> 다. 이 데이터의 hdl_graph_slam 결과는 없고, PC·Mac 어디에도 빌드돼 있지 않다
(Mac <code>hdlgraphslam_ws</code> 는 소스만 있고 ROS 가 없다). 이 문서의 모든 수치는 KISS-ICP 기준이다.</div>

<div class="hr"></div>
<h2><span class="no">01</span>무엇이 흔들렸나</h2>
<p class="sub">평소 흔들림은 ORB-SLAM3 수준이다. 문제는 특정 순간에 몰린 스캔 단위의 튐이다.</p>

<p>1 s 이동평균에서 벗어난 수평 흔들림은 라이다 0.62 cm, ORB-SLAM3 최종 궤적 0.69 cm 로 비슷하다. 그런데 스캔 간 가속
(<code>|p[i+1] − 2p[i] + p[i−1]|</code>)을 보면 67–68 s, 102–103 s, 108–110 s 에 1.5–2.7 cm 짜리 튐이 몰려 있고, 같은 순간
ORB-SLAM3 는 매끄럽다. 붉은 세로선이 상위 1 % 스캔이다.</p>

<figure>
  <div class="cap-top"><b>그림 1</b> 스캔 간 가속, 실시간 실행 전체 (10 Hz)</div>
  {fig_accel(t3, a3, t5, a5, spikes)}
  <figcaption>회색 = 기존(v3), 보라 = 적용 후(v5). 튀던 순간의 가속 중앙값 <b>{base['accel_cm']['at_base_spikes_median']:.2f} → {fin['accel_cm']['at_base_spikes_median']:.2f} cm</b>,
  p99 <b>{base['accel_cm']['p99']:.2f} → {fin['accel_cm']['p99']:.2f} cm</b>. 3 cm 이상은 위쪽에서 잘랐다.</figcaption>
</figure>

<figure>
  <div class="cap-top"><b>그림 2</b> 108–110 s 구간 수평 궤적 확대 (106–111.5 s)</div>
  {fig_zoom(zoom_v3, zoom_v5, zoom_orb)}
  <figcaption>점 하나가 스캔 하나. 기존 라이다(회색)는 ORB-SLAM3(초록)가 매끄럽게 지나가는 곳에서 옆으로 튀고,
  적용 후(보라)는 ORB-SLAM3 모양을 따라간다. ORB 궤적은 전체 궤적 Sim(3) 정합으로 라이다 좌표에 옮겼다.</figcaption>
</figure>

<div class="hr"></div>
<h2><span class="no">02</span>원인</h2>
<p class="sub">가설을 하나씩 데이터로 지웠고, 남은 것은 “직전 스캔의 움직임으로 현재 스캔을 편다”는 가정이다.</p>

<div class="tablebox wrapcells"><table>
<thead><tr><th>가설</th><th>확인 방법</th><th>결과</th><th>판정</th></tr></thead>
<tbody>
<tr><td>기하 퇴화</td><td>점–평면 정보행렬의 수평 고유값 비</td><td>전체 중앙 0.73, 108–111 s 0.78 · 튐과 상관 −0.01</td><td><span class="chip pass">아님</span></td></tr>
<tr><td>점 수 부족 · 근거리 가림</td><td>스캔별 유효 점, 1 m 이내 비율</td><td>튀는 스캔도 2.80–2.83 만 점, 0–0.5 %</td><td><span class="chip pass">아님</span></td></tr>
<tr><td>스캔 누락 · 시간 필드</td><td>점별 time 범위, 방위 범위</td><td>모든 스캔 −99.5…+1.3 ms, 360°</td><td><span class="chip pass">아님</span></td></tr>
<tr><td>스탬프 기준 어긋남</td><td>deskew 기준점 시험</td><td>스캔 끝 기준 = 헤더 스탬프와 일치</td><td><span class="chip pass">아님</span></td></tr>
<tr><td>움직이는 물체</td><td>5 스캔 전 지도로 설명 안 되는 점 비율</td><td>튀는 스캔 8–16 %, 평소 9 %</td><td><span class="chip warn">약함</span></td></tr>
<tr class="bad"><td><b>등속 가정 deskew</b></td><td>운동 변화와의 상관, deskew 끔 실험</td><td>선가속 0.70 · 각가속 0.63, 끄면 튐 1.84 → 0.63 cm</td><td><span class="chip fail">원인</span></td></tr>
</tbody></table></div>

<figure>
  <div class="cap-top"><b>그림 3</b> 스캔 간 가속과 각 요인의 순위상관 (Spearman, 1,577 스캔)</div>
  {corr}
  <figcaption>튐은 빠르기(속도·요 속도)보다 <b>움직임이 바뀌는 정도</b>(선가속·각가속)를 더 따라간다. 튀는 스캔은 각가속 40–130 °/s² 인
  출발 · 정지 · 회전 전환 순간이다. 기하 퇴화와는 상관이 없다.</figcaption>
</figure>

<p>VLP-16 은 한 바퀴(0.1 s)를 도는 동안 점을 찍으므로, 움직이는 동안 찍힌 스캔은 휘어 있다. KISS-ICP 는 이를 펴기 위해
<b>직전 스캔 사이의 움직임이 이번 스캔에도 계속된다</b>고 보고 점을 옮긴다. 카트가 출발·정지하거나 회전 방향을 바꾸면
이 가정이 틀려 스캔이 잘못 펴지고, 정합이 1–2 cm 옆으로 밀린다. 다음 스캔에서 움직임이 다시 맞으면 제자리로 돌아와 “튐”으로 보인다.
IMU 가 없어 실제 움직임을 따로 알 수 없는 것이 근본 원인이다.</p>

<div class="hr"></div>
<h2><span class="no">03</span>오프라인 비교 — 무엇이 효과가 있나</h2>
<p class="sub">같은 bag 을 Mac 에서 방식별로 다시 돌리고, ORB-SLAM3 최종 궤적과 비교했다.</p>

<p><b>고주파 차</b>는 라이다와 ORB-SLAM3 궤적의 차에서 1 s 이동평균을 뺀 값이다. 라이다 잡음이 줄면 작아지고, 반대로 실제 움직임을
뭉개면 급회전 구간에서 커진다. <b>저주파 RMSE</b>는 드리프트·RT 수준의 차로, 스무딩으로는 줄지 않아야 정상이다.</p>

<figure>
  <div class="cap-top"><b>그림 4</b> 방식별 ORB-SLAM3 대비 고주파 차</div>
  {dots}
  <figcaption>deskew 를 끄면 튐은 사라지지만 저주파 차가 3.27 → 5.25 cm 로 나빠진다. 반복 deskew 는 원인을 줄이고,
  남은 스캔 잡음은 5 스캔 평균이 흡수한다.</figcaption>
</figure>

<div class="tablebox wrapth"><table>
<thead><tr><th>방식</th><th>ms/스캔</th><th>지연</th><th>가속 p99</th><th>고주파 전체</th><th>고주파 튐</th><th>고주파 급회전</th><th>저주파 RMSE</th><th>객체 흩어짐 중앙/p90</th></tr></thead>
<tbody>{''.join(vrows)}</tbody></table></div>
<p style="font-size:14px;color:var(--ink-2)">단위 cm (ms/스캔·지연 제외). 반복 deskew = ICP 뒤 이번 스캔에서 추정한 움직임으로 원본 스캔을 다시 펴고 재정합.
가우시안 평균(σ 2 스캔, 0.6 s)과 등가속 칼만 RTS 스무더도 시험했지만 5 스캔 평균보다 낫지 않거나 지연이 0.3 s 를 넘었다.</p>

<div class="hr"></div>
<h2><span class="no">04</span>실시간 적용</h2>
<p class="sub">Mac <code>lidar_stream.py</code> 에 반복 deskew 2회와 5 스캔 평균을 넣고, 같은 데이터·SAM-6D·RT 로 다시 실행했다.</p>

<h3>첫 시도(v4)에서 물체가 사라진 이유</h3>
<p>평균에는 뒤따르는 2 스캔이 필요해 포즈를 0.2 s 늦게 보냈다. 그런데 화면 프레임 하나의 위치를 보간하려면 그 프레임
<b>다음 스캔</b>(최대 +0.1 s)까지 도착해 있어야 한다. PC 도착 지연이 {lag4:.0f} ms 로 늘면서 표시 지연 0.3 s 를 넘었고,
<b>화면 프레임의 {d4[0]:.0f} % 에서 포즈가 없어 박스를 그리지 못했다</b>. SAM-6D 검출 수는 그대로였다
(Sikhye 첫 표시 {first['v3'].get('Sikhye_high', 0):.1f} → {first['v4'].get('Sikhye_high', 0):.1f} s, 메모리에서 Mugcup lost · Dinosaur 둘로 쪼개짐).</p>

<figure>
  <div class="cap-top"><b>그림 5</b> 화면 보간에 필요한 시간 = 포즈 도착 + 다음 스캔</div>
  {budget}
  <figcaption>✓ 화면이 이 포즈로 보간 · ✗ 늦어서 화면에 포즈 없음 · △ 늦으면 원시 포즈로 대신 보간. v5 는 원시 포즈를 바로 보내 화면이 기다리지 않게 하고, 평균 포즈는 0.2 s 뒤에 같은 시각의 원시 포즈를 교체한다.
  SAM-6D 결과는 약 2 s 뒤에 나오므로 객체 배치·메모리는 항상 평균 포즈를 쓴다.</figcaption>
</figure>

<pre><span class="c"># Mac → PC (ssh -R 17001), 스캔마다</span>
{{"type":"pose",        "t_ns":…, "T_wc":[원시 16], "track_ms":…, "send_lag_s":…}}
<span class="c"># 뒤따르는 2 스캔이 정합된 뒤 (0.2 s 후)</span>
{{"type":"pose_refine", "t_ns":…, "T_wc":[5 스캔 평균 16], "smooth_n":5}}
<span class="c"># PC hub: PoseBuffer.refine(t_ns, T_wc) — slam_poses.jsonl 의 T_wc = 평균, T_wc_raw = 원시</span></pre>

<h3>결과 (v5)</h3>
<div class="tablebox"><table>
<thead><tr><th>실시간 · 260910_object</th><th>v3 기존</th><th>v4 평균만 전송</th><th>v5 원시 + 보정 전송</th></tr></thead>
<tbody>
<tr><td>포즈 도착 지연 중앙</td><td class="num">{lag3:.0f} ms</td><td class="num">{lag4:.0f} ms</td><td class="num best">{lag5:.0f} ms <span style="color:var(--ink-3);font-weight:400">(보정 {ref5:.0f})</span></td></tr>
<tr><td>포즈 없는 화면 프레임 (50–160 s)</td><td class="num">{d3[0]:.0f} %</td><td class="num" style="color:var(--fail)">{d4[0]:.0f} %</td><td class="num best">{d5[0]:.0f} %</td></tr>
<tr><td>화면 객체 수 (50–160 s)</td><td class="num">{d3[1]:.2f}</td><td class="num">{d4[1]:.2f}</td><td class="num best">{d5[1]:.2f}</td></tr>
<tr><td>객체 메모리</td><td class="num">active {m3[0]['active']} · 종 {m3[1]}</td><td class="num">active {m4[0]['active']} · lost {m4[0].get('lost', 0)} · 분열</td><td class="num best">active {m5[0]['active']} · 종 {m5[1]}</td></tr>
<tr><td>스캔 간 가속 중앙 / p99</td><td class="num">{base['accel_cm']['median']:.2f} / {base['accel_cm']['p99']:.2f} cm</td><td class="num">—</td><td class="num best">{fin['accel_cm']['median']:.2f} / {fin['accel_cm']['p99']:.2f} cm</td></tr>
<tr><td>ORB 대비 고주파 차 전체 / 튐</td><td class="num">{base['hf_vs_orb_cm']['rms']:.2f} / {base['hf_vs_orb_cm']['at_base_spikes_rms']:.2f} cm</td><td class="num">—</td><td class="num best">{fin['hf_vs_orb_cm']['rms']:.2f} / {fin['hf_vs_orb_cm']['at_base_spikes_rms']:.2f} cm</td></tr>
<tr><td>급회전(&gt;40 °/s) 고주파 차</td><td class="num">{base['hf_vs_orb_cm']['fast_turn_gt40dps_rms']:.2f} cm</td><td class="num">—</td><td class="num best">{fin['hf_vs_orb_cm']['fast_turn_gt40dps_rms']:.2f} cm</td></tr>
<tr><td>관측 흩어짐 중앙 / p90</td><td class="num">{runs['B']['observation_spread_cm']['median']:.2f} / {runs['B']['observation_spread_cm']['p90']:.2f} cm</td><td class="num">—</td><td class="num">{runs['A']['observation_spread_cm']['median']:.2f} / {runs['A']['observation_spread_cm']['p90']:.2f} cm</td></tr>
<tr><td>보간 오차 ≤3 s</td><td class="num">{runs['B']['interp_prediction']['0-3s']['median_cm']:.2f} cm</td><td class="num">—</td><td class="num">{runs['A']['interp_prediction']['0-3s']['median_cm']:.2f} cm</td></tr>
<tr><td>라이다 처리 시간</td><td class="num">{runs['B']['track_ms_mean']:.1f} ms/스캔</td><td class="num">—</td><td class="num">{runs['A']['track_ms_mean']:.1f} ms/스캔</td></tr>
</tbody></table></div>

<p>v5 와 v3 의 객체 위치 차는 {min(obj_diff):.2f}–{max(obj_diff):.2f} cm, 궤적 차 RMSE {cmp53['trajectory_final_B_aligned_to_A_cm']['rmse']:.2f} cm 다.
<b>위치와 지도는 그대로 두고 흔들림만 없앴다.</b> 물체 첫 표시 시각도 v3 와 같다
(choco {first['v5'].get('choco_hazelnut_high', 0):.1f} s … Febreze {first['v5'].get('Febreze_high', 0):.1f} s). 첫 검출에서 박스가 뜨기까지
2–6 s 걸리는 것은 SAM-6D 1 회 약 2 s + 메모리 확정 때문이며 기존과 같다.</p>

<div class="hr"></div>
<h2><span class="no">05</span>남은 한계와 다음 단계</h2>
<div class="cards">
  <div class="card" style="--stripe:var(--warn)"><h4>저주파 차 3.3 cm</h4><div class="who">스무딩으로 줄지 않음</div>
    <ul><li>ORB 대비 드리프트 · RT 수준 차는 모든 방식에서 <b>3.25–3.29 cm</b></li><li>줄이려면 루프 클로저 백엔드(hdl_graph_slam 등) 필요 — 현재 미설치</li></ul></div>
  <div class="card" style="--stripe:var(--accent)"><h4>화면 궤적</h4><div class="who">표시 지연 0.3 s</div>
    <ul><li>보정 포즈는 {ref5:.0f} ms 에 도착 → 화면 보간은 평균 · 원시 포즈를 섞어 쓴다</li><li><code>--display-delay 0.45</code> 로 올리면 화면도 전부 평균 포즈</li></ul></div>
  <div class="card" style="--stripe:var(--lidar)"><h4>근본 해결</h4><div class="who">IMU</div>
    <ul><li>deskew 가 추정 대신 실제 회전을 쓰면 튐의 원인이 사라진다</li><li>이 데이터에는 IMU 토픽이 없다</li></ul></div>
</div>

<h3>실행</h3>
<pre>bash objpose/run.sh --slam lidar \\
  --sam-session /home/jucpark/DeepLearning/Dataset/260910_object/SAM \\
  --slam-session /home/jucpark/DeepLearning/Dataset/260910_object/lidar \\
  --mac-slam-session '~/Documents/DefenseMeta/Dataset/260910_object/lidar' \\
  --extrinsic objpose/rt/260910/X_lidar_sam.json
<span class="c"># 기존 방식: --slam-param deskew_passes=1 --slam-param smooth_scans=1</span></pre>

<h3>파일</h3>
<ul class="plain">
<li><code>objpose/lidar/lidar_stream.py</code> — Mac 라이다 스트리머 (반복 deskew, pose_refine)</li>
<li><code>objpose/pc/hub.py</code>, <code>objpose/pc/fusion.py</code> — pose_refine 수신, <code>PoseBuffer.refine</code></li>
<li><code>objpose/lidar/kiss_variants.py</code>, <code>eval_lidar_variants.py</code>, <code>smooth_lidar_traj.py</code> — 오프라인 비교</li>
<li><code>objpose/output/live_260910_lidar_v3 · v4 · v5</code> — 실시간 실행 로그, <code>compare_260910_lidar_v5_vs_v3.json</code></li>
</ul>
</main>

<footer><div class="wrap">2026-09-13 · PC 192.168.219.100 (SAM-6D, hub) · Mac 192.168.219.113 (KISS-ICP) · 데이터 Dataset/260910_object · objpose/lidar/make_jitter_report.py 로 생성</div></footer>
"""
    title = "260910 라이다 흔들림 보정"
    standalone = (f'<!doctype html>\n<html lang="ko">\n<head>\n<meta charset="utf-8">\n'
                  f'<meta name="viewport" content="width=device-width, initial-scale=1">\n<title>{title}</title>\n'
                  f'<style>{css}</style>\n</head>\n<body>\n{body}\n</body>\n</html>\n')
    out = Path(a.out)
    out.write_text(standalone)
    Path(str(out) + ".artifact.html").write_text(f"<title>{title}</title>\n<style>{css}</style>\n{body}\n")
    print(out, len(standalone) // 1024, "KB")


if __name__ == "__main__":
    main()
