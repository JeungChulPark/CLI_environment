#!/usr/bin/env python3
"""make_example_images.py — 발표·확인용 예시 이미지.

1) 한 번도 등록되지 않은 객체(인식 실패) 5종 — 객체가 가장 잘 보이는 프레임 1장씩.
   원본 영상에 GT 위치(초록 윤곽)와 이름을 표시하고, 오른쪽에 그 부분을 확대해 붙인다.
2) 000055 — 실시간 입력 SAM-6D 가 실제로 결과를 낸 프레임 몇 장: 원본 영상과, 같은 영상에 추정 포즈를 겹쳐 그린 것.
   추정 = 그 영상 자체에 대한 SAM-6D 결과(모델 표면 점을 투영한 반투명 실루엣 + 3D 박스 + 좌표축),
   GT = 초록 3D 박스. 라벨은 객체 이름과 GT 대비 위치 오차.

    python3 objpose/tools/ycbv_sam6d/make_example_images.py --out objpose/docs/figs_ycbv_examples
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]
YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test")
PTS = REPO / "sam6d_realtime" / "assets" / "model_points"
TABLE = REPO / "objpose" / "rt" / "bop" / "ycbv_objects.json"
RT = REPO / "objpose" / "output" / "ycbv_sam6d_realtime_260921"
NEVER = ["ycbv_pudding_box", "ycbv_gelatin_box", "ycbv_potted_meat_can", "ycbv_large_clamp", "ycbv_foam_brick"]
KO = {"ycbv_pudding_box": "푸딩 상자", "ycbv_gelatin_box": "젤라틴 상자", "ycbv_potted_meat_can": "통조림 햄(SPAM)",
      "ycbv_large_clamp": "클램프(대)", "ycbv_foam_brick": "스펀지 블록"}
# 객체별 색 (BGR) — 000055 의 6개
PALETTE = [(178, 111, 31), (30, 138, 224), (85, 138, 47), (42, 52, 184), (167, 94, 123), (75, 86, 140), (23, 160, 212)]
EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def se3(R, t):
    T = np.eye(4); T[:3, :3] = np.asarray(R, float).reshape(3, 3); T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def project(K, T, P):
    Q = P @ T[:3, :3].T + T[:3, 3]
    uv = (Q @ K.T)[:, :2] / Q[:, 2:3]
    return uv, Q[:, 2]


def corners(p):
    lo, hi = p.min(0), p.max(0)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def box(img, K, T, cor, color, w):
    uv, _ = project(K, T, cor)
    uv = np.round(uv).astype(int)
    for i, j in EDGES:
        cv2.line(img, tuple(uv[i]), tuple(uv[j]), color, w, cv2.LINE_AA)


def label(img, text, org, color, taken=None):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    x, y = int(np.clip(org[0], 2, img.shape[1] - tw - 6)), int(np.clip(org[1], th + 6, img.shape[0] - 34))
    if taken is not None:                      # 이미 놓인 라벨과 겹치면 아래로 비켜 놓는다
        for _ in range(20):
            r = (x - 3, y - th - 5, x + tw + 3, y + 4)
            if not any(r[0] < q[2] + 4 and q[0] < r[2] + 4 and r[1] < q[3] + 4 and q[1] < r[3] + 4 for q in taken):
                break
            y += th + 10
        taken.append((x - 3, y - th - 5, x + tw + 3, y + 4))
    cv2.rectangle(img, (x - 3, y - th - 5), (x + tw + 3, y + 4), (255, 255, 255), -1)
    cv2.rectangle(img, (x - 3, y - th - 5), (x + tw + 3, y + 4), color, 1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)


def gt_frames(sc, by_id):
    gt = json.loads((sc / "scene_gt.json").read_text()); info = json.loads((sc / "scene_gt_info.json").read_text())
    cam = json.loads((sc / "scene_camera.json").read_text())
    return gt, info, cam


def never_seen(out, by_id, id_of):
    for nm in NEVER:
        best = None
        for sc in sorted(YCBV.iterdir()):
            gt, info, cam = gt_frames(sc, by_id)
            for f, rows in gt.items():
                for j, r in enumerate(rows):
                    if r["obj_id"] == id_of[nm]:
                        gi = info[f][j]
                        score = gi["visib_fract"] * gi["px_count_visib"]
                        if best is None or score > best[0]:
                            best = (score, sc, f, gi["bbox_visib"], gi["visib_fract"])
        _, sc, f, (x, y, w, h), vis = best
        img = cv2.imread(str(sc / "rgb" / f"{int(f):06d}.png"))
        ov = img.copy()
        m = 14
        x0, y0, x1, y1 = max(0, x - m), max(0, y - m), min(640, x + w + m), min(480, y + h + m)
        cv2.rectangle(ov, (x0, y0), (x1, y1), (60, 200, 60), 3)
        label(ov, f"{nm.replace('ycbv_', '')}", (x0, y0 - 6), (60, 200, 60))
        crop = img[y0:y1, x0:x1]
        s = 480 / max(crop.shape[0], crop.shape[1] * 480 / 400)
        crop = cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
        panel = np.full((480, 400, 3), 255, np.uint8)
        ch, cw = crop.shape[:2]
        panel[(480 - ch) // 2:(480 - ch) // 2 + ch, (400 - cw) // 2:(400 - cw) // 2 + cw] = crop
        cv2.rectangle(panel, (0, 0), (399, 479), (60, 200, 60), 2)
        full = np.concatenate([ov, np.full((480, 10, 3), 255, np.uint8), panel], 1)
        cv2.imwrite(str(out / f"never_{nm.replace('ycbv_', '')}.jpg"), full, [cv2.IMWRITE_JPEG_QUALITY, 92])
        print(nm, sc.name, f, f"visib {vis:.2f}")


def scene_055(out, by_id, n_pick=4):
    s = "000055"
    sc = YCBV / s
    gt, info, cam = gt_frames(sc, by_id)
    ids = sorted(cam, key=int)
    K = np.array(cam[ids[0]]["cam_K"]).reshape(3, 3)
    frames = [json.loads(l) for l in (RT / s / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]
    names = sorted({by_id[r["obj_id"]] for v in gt.values() for r in v})
    col = {n: PALETTE[i % len(PALETTE)] for i, n in enumerate(names)}
    pts = {n: np.load(PTS / f"{n}.npy").astype(np.float64) for n in names}
    cor = {n: corners(pts[n]) for n in names}
    # 결과가 나온 프레임 중, 잡힌 객체가 많은 것 위주로 영상 전체에 고르게 n_pick 장
    cand = [f for f in frames if len(f["dets"]) >= 4]
    picks = [cand[int(i)] for i in np.linspace(0, len(cand) - 1, n_pick)]
    rows_meta = []
    for f in picks:
        k = f["idx"]; fid = ids[k]
        img = cv2.imread(str(sc / "rgb" / f"{int(fid):06d}.png"))
        cv2.imwrite(str(out / f"s055_{k:04d}_raw.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        ov = img.copy()
        g = {by_id[r["obj_id"]]: se3(r["cam_R_m2c"], r["cam_t_m2c"]) for j, r in enumerate(gt[fid])
             if info[fid][j]["visib_fract"] >= 0.1}
        # 추정 실루엣 (먼 것부터)
        dets = sorted(f["dets"], key=lambda d: -d["t_mm"][2])
        layer = ov.copy()
        for d in dets:
            T = se3(d["R"], d["t_mm"])
            uv, _ = project(K, T, pts[d["object"]])
            uv = np.round(uv).astype(int)
            ok = (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)
            mask = np.zeros((480, 640), np.uint8)
            mask[uv[ok, 1], uv[ok, 0]] = 255
            mask = cv2.morphologyEx(cv2.dilate(mask, np.ones((5, 5), np.uint8)), cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
            layer[mask > 0] = col[d["object"]]
        ov = cv2.addWeighted(layer, 0.45, ov, 0.55, 0)
        for nm, T in g.items():
            box(ov, K, T, cor[nm], (60, 200, 60), 3)
        meta, taken = [], []
        for d in dets:
            T = se3(d["R"], d["t_mm"])
            box(ov, K, T, cor[d["object"]], col[d["object"]], 2)
            o, _ = project(K, T, np.array([[0, 0, 0], [40, 0, 0], [0, 40, 0], [0, 0, 40]], float))
            o = np.round(o).astype(int)
            for a, c in zip(range(1, 4), [(40, 40, 230), (40, 200, 40), (230, 90, 40)]):
                cv2.arrowedLine(ov, tuple(o[0]), tuple(o[a]), c, 2, cv2.LINE_AA, tipLength=0.25)
            e = float(np.linalg.norm(T[:3, 3] - g[d["object"]][:3, 3])) if d["object"] in g else None
            uvb, _ = project(K, T, cor[d["object"]])
            label(ov, f"{d['object'].replace('ycbv_', '')}" + (f"  {e:.1f} mm" if e is not None else ""),
                  (uvb[:, 0].min(), uvb[:, 1].min() - 6), col[d["object"]], taken)
            meta.append({"obj": d["object"].replace("ycbv_", ""), "err_mm": e})
        cv2.rectangle(ov, (0, 452), (640, 480), (255, 255, 255), -1)
        cv2.putText(ov, f"000055 #{k + 1}  t={k / 30:.1f}s   green: GT   colour: SAM-6D",
                    (8, 471), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
        cv2.imwrite(str(out / f"s055_{k:04d}_overlay.jpg"), ov, [cv2.IMWRITE_JPEG_QUALITY, 92])
        missing = [n.replace("ycbv_", "") for n in g if n not in {d["object"] for d in f["dets"]}]
        rows_meta.append({"frame": k + 1, "t": round(k / 30, 2), "raw": f"s055_{k:04d}_raw.jpg",
                          "overlay": f"s055_{k:04d}_overlay.jpg", "dets": meta, "not_detected": missing,
                          "ms": f["ms"]["total"]})
        print("055", k + 1, meta, "missing", missing)
    (out / "s055_meta.json").write_text(json.dumps(rows_meta, ensure_ascii=False, indent=1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, type=Path)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    table = json.loads(TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    id_of = {v: k for k, v in by_id.items()}
    never_seen(a.out, by_id, id_of)
    scene_055(a.out, by_id)


if __name__ == "__main__":
    main()
