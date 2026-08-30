#!/usr/bin/env python3
"""검증 필수 시각화 3종 생성.

  1) True Negative 예시  : milk 없음 + pose 미출력 → "No Milk Detected / No Pose Output"
  2) False Positive 예시 : milk 없음 + pose 출력  → bbox+mask+axis + "False Positive: Milk Pose Estimated"
  3) Pose Stability grid  : only_Milk 연속 프레임 → frame_id/t/Δt/Δr/pose_score + axis overlay grid

입력
  --frame-results : build_validation_report.py 가 만든 frame_results.csv
  --frames-dir    : extract_bag_frames.py 가 만든 RGB <frame_id>.png 디렉터리
  --pem-root      : 프레임별 <frame_id>/detection_pem.json 루트 (output_dir/_run)
  --camera-json   : axis 투영용 cam_K (예: src/sam6d_ros/config/d455f_camera.json)
  --out           : 출력 디렉터리 (visualizations/...)
  --mode          : tn | fp | stability | all (기본 all)

주의: pose axis 는 detection_pem.json 의 R,t(mm 가정) + cam_K 로 best-effort 투영.
"""
import argparse
import csv
import glob
import json
import math
import os

import cv2
import numpy as np


# ── RLE (uncompressed, column-major) → mask ──
def rle_to_mask(seg):
    h, w = seg["size"]
    counts = seg["counts"]
    if isinstance(counts, str):
        try:
            from pycocotools import mask as cocomask
            return cocomask.decode(seg).astype(bool)
        except Exception:
            return None
    arr = np.zeros(h * w, dtype=np.uint8)
    idx = 0
    val = 0
    for c in counts:
        c = int(c)
        arr[idx:idx + c] = val
        idx += c
        val ^= 1
    return arr.reshape((h, w), order="F").astype(bool)


def load_cam_K(path):
    try:
        d = json.load(open(path))
    except Exception:
        return None
    K = None
    if "cam_K" in d:
        K = d["cam_K"]
    else:
        for v in d.values():
            if isinstance(v, dict) and "cam_K" in v:
                K = v["cam_K"]
                break
    if K is None:
        return None
    K = np.array(K, dtype=np.float64).reshape(3, 3)
    return K


def load_pem(pem_root, fid):
    p = os.path.join(pem_root, fid, "detection_pem.json")
    if not os.path.isfile(p):
        return None
    try:
        dets = json.load(open(p))
    except Exception:
        return None
    return dets if isinstance(dets, list) and dets else None


def best_det(dets):
    return max(range(len(dets)), key=lambda i: dets[i].get("score", -1.0))


def draw_axis(img, K, R, t, length=50.0):
    if K is None or R is None or t is None:
        return
    R = np.array(R, dtype=np.float64)
    if R.size == 9:
        R = R.reshape(3, 3)
    t = np.array(t, dtype=np.float64).reshape(3)
    rvec, _ = cv2.Rodrigues(R)
    pts = np.float64([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, length]])
    proj, _ = cv2.projectPoints(pts, rvec, t, K, None)
    proj = proj.reshape(-1, 2).astype(int)
    o = tuple(proj[0])
    cv2.line(img, o, tuple(proj[1]), (0, 0, 255), 2)   # X red
    cv2.line(img, o, tuple(proj[2]), (0, 255, 0), 2)   # Y green
    cv2.line(img, o, tuple(proj[3]), (255, 0, 0), 2)   # Z blue


def put_banner(img, text, color=(0, 0, 255), y=30):
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)


def put_lines(img, lines, x=10, y0=60, color=(255, 255, 255)):
    for i, ln in enumerate(lines):
        yy = y0 + i * 22
        cv2.putText(img, ln, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, ln, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def gate_lines(r):
    """depth/workspace gate 정보 overlay 라인(있으면)."""
    out = []
    dr = r.get("depth_gate_reason"); wr = r.get("workspace_gate_reason")
    if dr or wr:
        out.append(f"gate: depth={dr or '-'} ws={wr or '-'}")
        out.append(f"vr={r.get('depth_valid_ratio','-')} errMed={r.get('depth_error_median','-')} "
                   f"agree={r.get('depth_agreement_ratio','-')}")
        bg = r.get("final_publish_decision_before_gate"); ag = r.get("final_publish_decision_after_gate")
        if bg or ag:
            out.append(f"before={bg or '-'} after={ag or '-'}")
    return out


def read_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_rgb(frames_dir, fid):
    for cand in (f"{fid}.png", f"{int(fid):06d}.png" if str(fid).isdigit() else None):
        if cand:
            p = os.path.join(frames_dir, cand)
            if os.path.isfile(p):
                return cv2.imread(p, cv2.IMREAD_COLOR)
    return None


def even_sample(items, n):
    if len(items) <= n:
        return items
    step = len(items) / n
    return [items[int(i * step)] for i in range(n)]


def make_tn(rows, frames_dir, out, maxn):
    d = os.path.join(out, "true_negative_examples")
    os.makedirs(d, exist_ok=True)
    tn = [r for r in rows if r["decision_type"] == "TN"]
    cnt = 0
    for r in even_sample(tn, maxn):
        img = load_rgb(frames_dir, r["frame_id"])
        if img is None:
            continue
        put_banner(img, "No Milk Detected / No Pose Output", (0, 220, 0))
        put_lines(img, [f"frame {r['frame_id']}  decision={r['final_decision']}"] + gate_lines(r))
        cv2.imwrite(os.path.join(d, f"tn_{r['frame_id']}.png"), img)
        cnt += 1
    print(f"[TN] {cnt}/{len(tn)} saved → {d}")


def make_fn(rows, frames_dir, out, maxn):
    """False Negative: GT=milk 있음 + prediction=pose 없음 (decision_type=FN)."""
    d = os.path.join(out, "false_negative_examples")
    os.makedirs(d, exist_ok=True)
    fn = [r for r in rows if r["decision_type"] == "FN"]
    cnt = 0
    for r in even_sample(fn, maxn) if maxn else fn:
        img = load_rgb(frames_dir, r["frame_id"])
        if img is None:
            continue
        put_banner(img, "False Negative: Milk Present, No Pose Output", (0, 165, 255))
        put_lines(img, [f"frame {r['frame_id']}  GT=milk_visible  decision={r['final_decision']}"] + gate_lines(r))
        cv2.imwrite(os.path.join(d, f"fn_{r['frame_id']}.png"), img)
        cnt += 1
    print(f"[FN] {cnt}/{len(fn)} saved → {d}")


def make_fp(rows, frames_dir, pem_root, K, out, maxn):
    d = os.path.join(out, "false_positive_examples")
    os.makedirs(d, exist_ok=True)
    fp = [r for r in rows if r["decision_type"] == "FP"]
    cnt = 0
    for r in even_sample(fp, maxn) if maxn else fp:
        fid = r["frame_id"]
        img = load_rgb(frames_dir, fid)
        if img is None:
            continue
        dets = load_pem(pem_root, fid)
        if dets:
            i = best_det(dets)
            seg = dets[i].get("segmentation")
            if seg:
                m = rle_to_mask(seg)
                if m is not None and m.shape[:2] == img.shape[:2]:
                    overlay = img.copy()
                    overlay[m] = (0.5 * overlay[m] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
                    img = overlay
            bb = dets[i].get("bbox")
            if bb:
                x, y, w, h = [int(round(v)) for v in bb]
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            draw_axis(img, K, dets[i].get("R"), dets[i].get("t"))
        put_banner(img, "False Positive: Milk Pose Estimated", (0, 0, 255))
        ps = r.get("pose_score") or ""
        put_lines(img, [f"frame {fid}  pose_score={ps}  t={r.get('translation_stab_xyz', '')}"] + gate_lines(r))
        cv2.imwrite(os.path.join(d, f"fp_{fid}.png"), img)
        cnt += 1
    print(f"[FP] {cnt}/{len(fp)} saved → {d}")


def longest_tp_run(rows):
    best, cur = [], []
    for r in rows:
        if r["decision_type"] == "TP":
            cur.append(r)
            if len(cur) > len(best):
                best = cur[:]
        else:
            cur = []
    return best


def make_stability(rows, frames_dir, pem_root, K, out, seq_len, cols):
    d = os.path.join(out, "pose_stability_sequence")
    os.makedirs(d, exist_ok=True)
    run = longest_tp_run(rows)
    if not run:
        print("[STABILITY] TP 연속 구간 없음 — grid 생략")
        return
    seq = run[:seq_len]
    tiles = []
    for r in seq:
        fid = r["frame_id"]
        img = load_rgb(frames_dir, fid)
        if img is None:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
        dets = load_pem(pem_root, fid)
        if dets:
            i = best_det(dets)
            draw_axis(img, K, dets[i].get("R"), dets[i].get("t"))
        dt = r.get("delta_translation_stab") or "-"
        dr = r.get("delta_rotation_stab_deg") or "-"
        ps = r.get("pose_score") or "-"
        put_lines(img, [f"frame {fid}",
                        f"t={r.get('translation_stab_xyz','')}",
                        f"dt={dt}mm dR={dr}deg",
                        f"score={ps}"], y0=28, color=(0, 255, 255))
        tiles.append(img)

    h, w = tiles[0].shape[:2]
    tiles = [cv2.resize(t, (w, h)) for t in tiles]
    rows_n = math.ceil(len(tiles) / cols)
    blank = np.zeros((h, w, 3), dtype=np.uint8)
    while len(tiles) < rows_n * cols:
        tiles.append(blank)
    grid = np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows_n)])
    path = os.path.join(d, f"grid_{seq[0]['frame_id']}_{seq[-1]['frame_id']}.png")
    cv2.imwrite(path, grid)
    print(f"[STABILITY] {len(seq)} frames → {path}")


def _jload(s):
    try:
        return json.loads(s) if s else None
    except (ValueError, TypeError):
        return None


def make_stab_compare(rows, out, metrics_path=None):
    """FR-10: raw vs stabilized pose 시계열 비교 플롯 (variance 감소 시각화)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[STABCMP] matplotlib 없음 — 비교 플롯 생략")
        return
    pub = [r for r in rows if str(r.get("pose_output_exists")).lower() == "true"
           and _jload(r.get("translation_raw_xyz"))]
    if len(pub) < 2:
        print("[STABCMP] published 프레임 부족 — 생략")
        return
    d = os.path.join(out, "stabilization_compare")
    os.makedirs(d, exist_ok=True)

    idx = list(range(len(pub)))
    tr = [_jload(r["translation_raw_xyz"]) for r in pub]
    ts = [_jload(r["translation_stab_xyz"]) for r in pub]
    red = {}
    if metrics_path and os.path.isfile(metrics_path):
        try:
            sc = json.load(open(metrics_path)).get("stabilization_comparison", {})
            red = {"t": sc.get("translation_spread_mm", {}),
                   "r": sc.get("rotation_spread_deg", {})}
        except Exception:
            pass

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    for a, axis_name in enumerate("XYZ"):
        axes[a].plot(idx, [t[a] for t in tr], color="tab:red", alpha=0.6, label="raw")
        axes[a].plot(idx, [t[a] for t in ts], color="tab:blue", lw=2, label="stabilized")
        axes[a].set_ylabel(f"t_{axis_name} (mm)")
        axes[a].grid(True, alpha=0.3)
        axes[a].legend(loc="upper right", fontsize=8)
    # 4행: 연속 프레임 Δtranslation 비교
    dr = [_f(r.get("delta_translation_raw")) for r in pub]
    ds = [_f(r.get("delta_translation_stab")) for r in pub]
    axes[3].plot(idx, dr, color="tab:red", alpha=0.6, label="raw Δt")
    axes[3].plot(idx, ds, color="tab:blue", lw=2, label="stabilized Δt")
    axes[3].set_ylabel("Δtrans (mm)")
    axes[3].set_xlabel("published frame index")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right", fontsize=8)

    tt = red.get("t", {})
    rr = red.get("r", {})
    title = "Stabilization ON vs OFF (raw vs stabilized)"
    if tt:
        title += (f"\ntrans spread: raw={tt.get('raw')}mm → stab={tt.get('stabilized')}mm "
                  f"(reduction {tt.get('reduction_rate')})")
    if rr:
        title += (f" | rot spread: raw={rr.get('raw')}° → stab={rr.get('stabilized')}° "
                  f"(reduction {rr.get('reduction_rate')})")
    fig.suptitle(title, fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(d, "stabilization_compare.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"[STABCMP] → {path}")


def _f(v):
    try:
        return float(v) if v not in (None, "", "None") else None
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame-results", required=True)
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--pem-root", required=True)
    ap.add_argument("--camera-json", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mode", default="all",
                    choices=["tn", "fp", "fn", "stability", "stabcmp", "all"])
    ap.add_argument("--metrics", default=None, help="metrics.json (reduction rate 주석용)")
    ap.add_argument("--max-examples", type=int, default=6)
    ap.add_argument("--seq-len", type=int, default=12)
    ap.add_argument("--cols", type=int, default=4)
    args = ap.parse_args()

    rows = read_rows(args.frame_results)
    K = load_cam_K(args.camera_json)
    if K is None:
        print(f"[warn] cam_K 로드 실패: {args.camera_json} — axis overlay 생략")
    os.makedirs(args.out, exist_ok=True)

    if args.mode in ("tn", "all"):
        make_tn(rows, args.frames_dir, args.out, args.max_examples)
    if args.mode in ("fp", "all"):
        make_fp(rows, args.frames_dir, args.pem_root, K, args.out, args.max_examples)
    if args.mode in ("fn", "all"):
        make_fn(rows, args.frames_dir, args.out, args.max_examples)
    if args.mode in ("stability", "all"):
        make_stability(rows, args.frames_dir, args.pem_root, K, args.out, args.seq_len, args.cols)
    if args.mode in ("stabcmp", "all"):
        metrics = args.metrics or os.path.join(os.path.dirname(args.frame_results), "metrics.json")
        make_stab_compare(rows, args.out, metrics)


if __name__ == "__main__":
    main()
