#!/usr/bin/env python3
"""map_jitter.py — 카메라가 움직이는 동안 지도에 저장된 객체의 자세가 얼마나 흔들리는지 잰다 (제안 방법, 저장된 인식 결과를 순서대로 다시 넣는다).

인식 결과가 한 장 들어올 때마다 지도의 등록 객체 자세를 기록해 시간에 따른 궤적을 만든다. 세계 좌표는 정답 좌표계(SLAM → 정답 정렬).
  관측별 흩어짐      그 프레임의 추정 자세를 그대로 세계 좌표로 옮긴 값(지도에 넣기 전)이 물체별 중앙 위치에서 벗어난 거리
  저장값의 흔들림    지도에 저장된 위치가 갱신 사이에 움직인 거리(한 번에 움직인 양), 그리고 마지막 위치에서 벗어난 거리
  방향              저장된 방향이 갱신 사이에 바뀐 각도, 5° 넘게 바뀐 횟수
BENCH_ROOT / BENCH_TABLE / BENCH_SLAM / BENCH_FPS 로 데이터셋을 바꾼다.
    <sam6d python> objpose/tools/analysis/map_jitter.py --run <proposed dir> --scenes 000054 --out <dir> [--plot]
"""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM   # noqa: E402
from build_memory_maps import PoseBuffer, Fusion, ObjectMemory, DepthCheckConfig, map_occlusion_kwargs, KEEP, T0_NS, STEP_NS, FPS   # noqa: E402


def ang(Ra, Rb):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def replay(scene, run, by_id):
    """run_memory 와 같은 설정으로 다시 넣으면서, 프레임마다 (시각, 물체별 원시 추정, 지도 저장값)을 남긴다."""
    sc = BM.YCBV / scene; ids, Twc_gt, gobj, K = BM.gt_scene(sc, by_id); n = len(ids); slam = BM.load_slam(scene, n)
    poses = PoseBuffer(max_gap_s=0.25)
    for i in sorted(slam): poses.add(T0_NS + i * STEP_NS, "OK", slam[i], map_id=0)
    mem = ObjectMemory(Fusion(np.eye(4), poses), K, (640, 480), depth_check=DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0),
                       assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None, **map_occlusion_kwargs(by_id.values()))
    ks = sorted(slam); M = sum(Twc_gt[k, :3, :3] @ slam[k][:3, :3].T for k in ks); U, _, Vt = np.linalg.svd(M)
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt; A = np.eye(4); A[:3, :3] = R; A[:3, 3] = np.mean([Twc_gt[k, :3, 3] - R @ slam[k][:3, 3] for k in ks], 0)
    frames = [json.loads(l) for l in (run / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]; frames.sort(key=lambda f: f.get("done_s", f["idx"]))
    raw, lm = {}, {}
    for f in frames:
        t = f["idx"] / FPS
        d = cv2.imread(str(sc / "depth" / f"{f['idx'] + 1:06d}.png"), cv2.IMREAD_UNCHANGED)
        depth = np.rint(d.astype(np.float32) * 0.1).astype(np.uint16)[::8, ::8] if d is not None else None
        if f["idx"] in slam:
            for det in f["dets"]:
                Tco = np.eye(4); Tco[:3, :3] = np.asarray(det["R"], float); Tco[:3, 3] = np.asarray(det["t_mm"], float) / 1000.0
                T = A @ slam[f["idx"]] @ Tco; raw.setdefault(det["object"], []).append((t, T[:3, 3].copy(), T[:3, :3].copy()))
        mem.add_frame(int(f["t_ns"]), [(x["object"], x["score"], x["R"], x["t_mm"]) for x in f["dets"]], depth=depth); mem.update()
        for m in mem.landmarks():
            if m["status"] not in KEEP: continue
            T = A @ np.asarray(m["T_w_obj"], float); lm.setdefault((m["object_id"], m["name"]), []).append((t, T[:3, 3].copy(), T[:3, :3].copy(), m["n_obs"]))
    return raw, lm, gobj, n / FPS, A


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True); ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--plot", action="store_true")
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    by_id = {o["obj_id"]: o["name"] for o in json.loads(BM.TABLE.read_text())["objects"]}
    scenes = a.scenes or sorted(p.name for p in a.run.iterdir() if p.is_dir() and (p / "sam6d_frames.jsonl").exists()); rows = []
    for s in scenes:
        raw, lm, gobj, dur, _A = replay(s, a.run, by_id); series = {}
        for (oid, nm), tr in lm.items():
            if len(tr) < 5: continue
            t = np.array([x[0] for x in tr]); p = np.array([x[1] for x in tr]); Rm = [x[2] for x in tr]
            upd = np.r_[True, np.diff([x[3] for x in tr]) > 0]                     # 관측이 새로 들어와 값이 갱신된 시점
            pu = p[upd]; Ru = [Rm[i] for i in np.where(upd)[0]]
            step = np.linalg.norm(np.diff(pu, axis=0), axis=1) * 1000 if len(pu) > 1 else np.zeros(1); dev = np.linalg.norm(p - p[-1], axis=1) * 1000
            rstep = np.array([ang(Ru[i], Ru[i + 1]) for i in range(len(Ru) - 1)]) if len(Ru) > 1 else np.zeros(1)
            rw = raw.get(nm, []); rp = np.array([x[1] for x in rw]) if rw else np.zeros((0, 3))
            rp = rp[np.linalg.norm(rp - p[-1], axis=1) < 0.30] if len(rp) else rp            # 같은 물체로 묶이는 범위(0.30 m)의 관측만
            rdev = np.linalg.norm(rp - np.median(rp, 0), axis=1) * 1000 if len(rp) else np.zeros(1)
            g = gobj.get(nm); gerr = (np.linalg.norm(p - g[:3, 3], axis=1) * 1000) if g is not None else None
            k10 = min(10, len(dev) - 1)
            rows.append({"scene": s, "object": nm, "object_id": int(oid), "n_obs": int(tr[-1][3]), "n_updates": int(upd.sum()), "seconds_in_map": float(t[-1] - t[0]),
                         "raw_scatter_mm_p50": float(np.median(rdev)), "raw_scatter_mm_p95": float(np.percentile(rdev, 95)),
                         "step_mm_p50": float(np.median(step)), "step_mm_p95": float(np.percentile(step, 95)), "step_mm_max": float(step.max()),
                         "dev_from_final_mm_p50": float(np.median(dev)), "dev_from_final_mm_max_after10": float(dev[k10:].max()),
                         "rot_step_deg_p50": float(np.median(rstep)), "rot_step_deg_p95": float(np.percentile(rstep, 95)), "rot_jumps_over5deg": int((rstep > 5).sum()),
                         "gt_err_mm_first": None if gerr is None else float(gerr[0]), "gt_err_mm_last": None if gerr is None else float(gerr[-1])})
            series[nm] = (t, p, rw, gerr)
        if a.plot and series:
            import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
            plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 11})
            fig, axs = plt.subplots(1, len(series), figsize=(4.4 * len(series), 3.6), sharey=True); axs = np.atleast_1d(axs)
            for ax, (nm, (t, p, rw, gerr)) in zip(axs, series.items()):
                if rw:
                    rt = np.array([x[0] for x in rw]); rp = np.array([x[1] for x in rw]); ok = np.linalg.norm(rp - p[-1], axis=1) < 0.30
                    ax.plot(rt[ok], np.linalg.norm(rp[ok] - p[-1], axis=1) * 1000, ".", color="#b0b0b0", ms=5, label="그 프레임의 추정값 (지도에 넣기 전)")
                ax.plot(t, np.linalg.norm(p - p[-1], axis=1) * 1000, "-", color="#1f4e79", lw=2.2, label="지도에 저장된 값")
                ax.set_title(nm.replace("ycbv_", "").replace("_", " "), fontsize=12); ax.set_xlabel("시간 (초)"); ax.set_xlim(0, dur); ax.grid(alpha=0.25); ax.spines[["top", "right"]].set_visible(False)
            axs[0].set_ylabel("마지막 저장 위치와의 거리 (mm)"); axs[0].set_ylim(0, 40); axs[0].legend(frameon=False, fontsize=9.5, loc="upper right")
            fig.tight_layout(); fig.savefig(a.out / f"jitter_{s}.png", dpi=160); plt.close(fig)
        print(s, len(series), flush=True)
    (a.out / "jitter.json").write_text(json.dumps(rows, ensure_ascii=False))
    for r in rows if len(scenes) == 1 else []:
        print(f"{r['object'][5:]:18s} 갱신 {r['n_updates']:3d}회 | 관측별 흩어짐 {r['raw_scatter_mm_p50']:.1f} / {r['raw_scatter_mm_p95']:.1f} mm | 저장값 한 번에 {r['step_mm_p50']:.2f} / {r['step_mm_p95']:.2f} / 최대 {r['step_mm_max']:.1f} mm | "
              f"마지막과의 차 {r['dev_from_final_mm_p50']:.1f}, 10회 뒤 최대 {r['dev_from_final_mm_max_after10']:.1f} mm | 방향 {r['rot_step_deg_p50']:.2f} / {r['rot_step_deg_p95']:.2f}°, 5° 넘는 변화 {r['rot_jumps_over5deg']}회 | 정답 오차 {r['gt_err_mm_first']:.1f} → {r['gt_err_mm_last']:.1f} mm")
    if rows:
        q = lambda k: float(np.median([r[k] for r in rows]))
        print(f"객체 {len(rows)}개 중앙값: 관측별 흩어짐 {q('raw_scatter_mm_p50'):.1f} mm (95% {q('raw_scatter_mm_p95'):.1f}) | 저장값 한 번에 {q('step_mm_p50'):.2f} mm (95% {q('step_mm_p95'):.2f}) | 10회 뒤 최대 이탈 {q('dev_from_final_mm_max_after10'):.1f} mm | 방향 {q('rot_step_deg_p50'):.2f}° (95% {q('rot_step_deg_p95'):.2f})")


if __name__ == "__main__":
    main()
