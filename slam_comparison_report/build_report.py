#!/usr/bin/env python3
"""Build the ORB-SLAM3 vs RTAB-Map visual comparison report (md + pdf).

Run in `conda activate rtabmap`. Reads the normalised SLAM outputs + the
generated top-view PNGs and emits:
  slam_comparison_report/slam_visual_comparison.md
  slam_comparison_report/slam_visual_comparison.pdf
The PDF is produced with matplotlib's PdfPages (no pandoc/latex needed).
"""
from __future__ import annotations

import sqlite3
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.image as mpimg
import matplotlib.font_manager as fm
import numpy as np

# Korean font for PDF text (titles/table/analysis). Falls back silently.
for _fp in ("/usr/share/fonts/truetype/nanum/NanumGothic.ttf",):
    if Path(_fp).exists():
        fm.fontManager.addfont(_fp)
        plt.rcParams["font.family"] = fm.FontProperties(fname=_fp).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "slam_comparison_report"
ORB_OUT = ROOT / "orbslam_ws" / "output"
RTAB_OUT = ROOT / "rtabmap_ws" / "output"

DATASETS = [
    ("SLAM_one_lap", "1바퀴 주행"),
    ("SLAM_one_lap_back_and_forth", "1바퀴 + 왕복"),
    ("SLAM_forward_backward_repeat", "전후 반복 주행"),
    ("SLAM_three_laps", "3바퀴 주행"),
]


def load_traj(path: Path) -> np.ndarray:
    if not path.exists():
        return np.empty((0, 3), np.float32)
    rows = []
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.split()
        if len(p) >= 4:
            try:
                rows.append((float(p[1]), float(p[2]), float(p[3])))
            except ValueError:
                pass
    a = np.asarray(rows, np.float32) if rows else np.empty((0, 3), np.float32)
    return a[np.isfinite(a).all(axis=1)] if a.size else a


def pcd_points(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", errors="replace") as f:
        for line in f:
            t = line.split()
            if t and t[0].upper() == "POINTS":
                return int(t[1])
    return 0


def count_lines(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(1 for ln in path.read_text(errors="replace").splitlines() if ln.strip() and not ln.startswith("#"))


def path_len(a: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum()) if len(a) >= 2 else 0.0


def start_end_gap(a: np.ndarray) -> float:
    return float(np.linalg.norm(a[-1] - a[0])) if len(a) >= 2 else 0.0


def xz_extent(a: np.ndarray):
    if not len(a):
        return 0.0, 0.0
    xz = a[:, [0, 2]]
    span = xz.max(axis=0) - xz.min(axis=0)
    return float(span[0]), float(span[1])


def orb_metrics(ds: str) -> dict:
    d = ORB_OUT / ds
    traj = load_traj(d / "trajectory.txt")
    log = d / "orbslam3_launch.log"
    loops = 0
    if log.exists():
        loops = sum(1 for ln in log.read_text(errors="replace").splitlines() if "Loop detected" in ln)
    ex, ez = xz_extent(traj)
    return {
        "poses": len(traj), "keyframes": count_lines(d / "keyframes.txt"),
        "map_points": pcd_points(d / "map_points.pcd"),
        "length": path_len(traj), "gap": start_end_gap(traj),
        "ext_x": ex, "ext_z": ez, "loops": loops, "loop_label": f"{loops} (Loop detected)",
    }


def rtab_metrics(ds: str) -> dict:
    d = RTAB_OUT / ds
    traj = load_traj(d / "trajectory.txt")
    db = d / "rtabmap.db"
    nodes = links = glob_lc = prox_lc = 0
    if db.exists():
        con = sqlite3.connect(str(db))
        try:
            nodes = con.execute("select count(*) from Node").fetchone()[0]
            for t, c in con.execute("select type,count(*) from Link group by type"):
                links += c
                if t == 1:
                    glob_lc = c           # global loop closure
                elif t == 2:
                    prox_lc = c           # local-space proximity
        except sqlite3.Error:
            pass
        finally:
            con.close()
    ex, ez = xz_extent(traj)
    return {
        "poses": len(traj), "keyframes": count_lines(d / "keyframes.txt"),
        "map_points": pcd_points(d / "dense_map.pcd"),
        "length": path_len(traj), "gap": start_end_gap(traj),
        "ext_x": ex, "ext_z": ez, "nodes": nodes, "links": links,
        "loops": glob_lc, "prox": prox_lc,
        "loop_label": f"{glob_lc} global + {prox_lc} proximity",
    }


def analysis_text(ds: str, kor: str, o: dict, r: dict) -> list[str]:
    lines = []
    # start-end / drift
    closes = "주행이 시작점 부근으로 복귀하는 시퀀스" if "lap" in ds or "repeat" in ds else "시퀀스"
    lines.append(f"- **시작-종료점 오차(start-end gap):** ORB {o['gap']:.2f} m vs RTAB {r['gap']:.2f} m. "
                 f"{closes}에서 값이 작을수록 누적 drift가 작고 루프 정합이 좋다는 의미.")
    # drift / consistency
    lines.append(f"- **궤적 길이·범위:** ORB 길이 {o['length']:.1f} m (X {o['ext_x']:.1f}×Z {o['ext_z']:.1f} m), "
                 f"RTAB 길이 {r['length']:.1f} m (X {r['ext_x']:.1f}×Z {r['ext_z']:.1f} m). "
                 f"두 시스템의 경로 범위가 유사할수록 스케일·형태 일관성이 높음.")
    # loop closure
    lines.append(f"- **Loop closure:** ORB = {o['loop_label']}; RTAB = {r['loop_label']} "
                 f"(RTAB Link type1=global, type2=local-space). 루프 폐합이 잡히면 누적오차가 보정됨.")
    # keyframe distribution
    lines.append(f"- **KeyFrame 분포:** ORB {o['keyframes']}개(전체 trajectory {o['poses']}프레임 대비), "
                 f"RTAB {r['keyframes']}개(=그래프 노드 {r.get('nodes', r['keyframes'])}). "
                 f"ORB는 프레임 단위 궤적+선별 KF, RTAB는 노드=KF 구조.")
    # path shape
    lines.append(f"- **경로 형태 차이:** Top-View(X–Z)에서 두 궤적의 형태를 직접 비교. "
                 f"ORB는 dense per-frame 궤적이라 곡선이 매끄럽고, RTAB는 KF 그래프라 노드 단위.")
    # map quality
    ratio = (r['map_points'] / max(1, o['map_points']))
    lines.append(f"- **맵 생성 품질:** ORB sparse {o['map_points']:,} 점 vs RTAB dense {r['map_points']:,} 점 "
                 f"(약 {ratio:.0f}배). RTAB(RGB-D dense)가 구조 시각화에 유리, ORB는 특징점 기반 sparse.")
    return lines


def build_md(rows: list[dict]) -> str:
    md = ["# SLAM 시각 비교 보고서 — ORB-SLAM3 vs RTAB-Map",
          "",
          "RealSense RGB-D 4개 시퀀스를 ORB-SLAM3와 RTAB-Map으로 각각 실행한 결과의 "
          "Top-View(X–Z) 궤적·KeyFrame·맵 비교. 모든 결과는 실제 ros2 bag 재생 기반으로 새로 생성됨 "
          "(ORB=conda `orbslam3`, RTAB=conda `rtabmap`).",
          "",
          "## 전체 요약",
          "",
          "| 시퀀스 | 시스템 | poses | KF | map points | 길이(m) | start-end(m) | loop closure |",
          "|---|---|---:|---:|---:|---:|---:|---|"]
    for row in rows:
        ds, kor, o, r = row["ds"], row["kor"], row["orb"], row["rtab"]
        md.append(f"| {kor} | ORB-SLAM3 | {o['poses']} | {o['keyframes']} | {o['map_points']:,} | "
                  f"{o['length']:.1f} | {o['gap']:.2f} | {o['loop_label']} |")
        md.append(f"| {kor} | RTAB-Map | {r['poses']} | {r['keyframes']} | {r['map_points']:,} | "
                  f"{r['length']:.1f} | {r['gap']:.2f} | {r['loop_label']} |")
    md.append("")
    for row in rows:
        ds, kor, o, r = row["ds"], row["kor"], row["orb"], row["rtab"]
        md.append(f"## {kor} (`{ds}`)")
        md.append("")
        # images (relative paths from report dir)
        op = f"../orbslam_ws/output/{ds}/trajectory_topview_keyframes.png"
        rp = f"../rtabmap_ws/output/{ds}/trajectory_topview_keyframes.png"
        md.append(f"| ORB-SLAM3 | RTAB-Map |")
        md.append(f"|---|---|")
        md.append(f"| ![ORB {ds}]({op}) | ![RTAB {ds}]({rp}) |")
        md.append("")
        md.extend(analysis_text(ds, kor, o, r))
        md.append("")
    md.append("## 종합 결론")
    md.append("")
    md.append("- **맵 품질:** RTAB-Map은 RGB-D dense 클라우드(수십만~수백만 점)로 환경 구조를 풍부하게 "
              "재구성하는 반면, ORB-SLAM3는 특징점 기반 sparse 맵(1~1.5만 점)으로 위치추정에 최적화됨.")
    md.append("- **궤적 특성:** ORB-SLAM3는 per-frame 궤적이라 조밀하고 매끄럽고, RTAB-Map은 KeyFrame "
              "그래프 노드 단위 궤적임. 두 시스템 모두 4개 시퀀스에서 트래킹에 성공.")
    md.append("- **Loop closure / drift:** 표의 start-end gap과 loop closure 수로 누적오차 보정 정도를 비교 가능. "
              "값이 작고 폐합이 잡힐수록 drift가 억제됨.")
    md.append("")
    md.append("*생성: `slam_comparison_report/make_topviews.py` + `build_report.py` (conda `rtabmap`).*")
    return "\n".join(md) + "\n"


def build_pdf(rows: list[dict], pdf_path: Path) -> None:
    with PdfPages(pdf_path) as pdf:
        # title page
        fig = plt.figure(figsize=(11.69, 8.27))  # A4 landscape
        fig.text(0.5, 0.72, "SLAM 시각 비교 보고서", ha="center", fontsize=24, weight="bold")
        fig.text(0.5, 0.64, "ORB-SLAM3  vs  RTAB-Map", ha="center", fontsize=18)
        fig.text(0.5, 0.57, "RealSense RGB-D · 4 시퀀스 · Top-View(X–Z)", ha="center", fontsize=13)
        # summary table
        col = ["시퀀스", "시스템", "poses", "KF", "map pts", "len(m)", "gap(m)", "loop"]
        cells = []
        for row in rows:
            o, r = row["orb"], row["rtab"]
            cells.append([row["kor"], "ORB-SLAM3", o["poses"], o["keyframes"], f"{o['map_points']:,}",
                          f"{o['length']:.1f}", f"{o['gap']:.2f}", o["loop_label"]])
            cells.append(["", "RTAB-Map", r["poses"], r["keyframes"], f"{r['map_points']:,}",
                          f"{r['length']:.1f}", f"{r['gap']:.2f}", r["loop_label"]])
        ax = fig.add_axes([0.05, 0.06, 0.9, 0.42])
        ax.axis("off")
        tbl = ax.table(cellText=cells, colLabels=col, loc="center", cellLoc="center")
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(8)
        tbl.scale(1, 1.4)
        pdf.savefig(fig)
        plt.close(fig)

        # per-dataset pages
        for row in rows:
            ds, kor, o, r = row["ds"], row["kor"], row["orb"], row["rtab"]
            fig = plt.figure(figsize=(11.69, 8.27))
            fig.suptitle(f"{kor}  ({ds})", fontsize=15, weight="bold")
            for col_i, (label, root) in enumerate([("ORB-SLAM3", ORB_OUT), ("RTAB-Map", RTAB_OUT)]):
                img = root / ds / "trajectory_topview_keyframes.png"
                ax = fig.add_axes([0.02 + col_i * 0.5, 0.30, 0.46, 0.60])
                ax.axis("off")
                ax.set_title(label, fontsize=11)
                if img.exists():
                    ax.imshow(mpimg.imread(img))
            ax_t = fig.add_axes([0.05, 0.02, 0.9, 0.24])
            ax_t.axis("off")
            txt = "\n".join(ln.replace("**", "").replace("- ", "• ")
                            for ln in analysis_text(ds, kor, o, r))
            ax_t.text(0, 1, txt, va="top", ha="left", fontsize=8.2, wrap=True)
            pdf.savefig(fig)
            plt.close(fig)


def main() -> int:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    for ds, kor in DATASETS:
        rows.append({"ds": ds, "kor": kor, "orb": orb_metrics(ds), "rtab": rtab_metrics(ds)})
        o, r = rows[-1]["orb"], rows[-1]["rtab"]
        print(f"{kor:14s} ORB gap={o['gap']:.2f} len={o['length']:.1f} loops={o['loops']} | "
              f"RTAB gap={r['gap']:.2f} len={r['length']:.1f} lc={r['loops']}/{r['prox']}", flush=True)
    (REPORT_DIR / "slam_visual_comparison.md").write_text(build_md(rows), encoding="utf-8")
    build_pdf(rows, REPORT_DIR / "slam_visual_comparison.pdf")
    print(f"\nWrote {REPORT_DIR/'slam_visual_comparison.md'}")
    print(f"Wrote {REPORT_DIR/'slam_visual_comparison.pdf'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
