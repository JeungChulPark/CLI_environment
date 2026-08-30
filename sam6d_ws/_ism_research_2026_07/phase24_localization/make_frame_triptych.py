#!/usr/bin/env python3
"""make_frame_triptych.py — 프레임별 [원본 | 객체인식 | 포즈추정] 3분할 이미지 생성 (READ-ONLY).

한 장 = 같은 프레임의
  1) 원본 RGB
  2) 객체 인식 : ISM detection_<obj>.json 의 mask(RLE)+bbox 를 클래스색으로 오버레이
  3) 포즈 추정 : PEM detection_pem.json 의 R,t 로 CAD 정점을 투영해 오버레이 (+3D bbox)

글자는 최소화한다 — 클래스 이름만 작게. 점수/임계/프레임ID 등은 넣지 않는다
(프레임 정보는 파일명에 있음).

입력: outputs/pem_inputs/<ds>/frame_XXXXXX/{rgb.png,camera.json,detection_<obj>.json,
                                           pem_<obj>/sam6d_results/detection_pem.json}
출력: outputs/phase24_localization/triptych/<gt|pem>/<ds>_<frame>.png
"""
import argparse, csv, glob, json, os, sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
PEMIN = os.path.join(REPO, "outputs", "pem_inputs")
OUT = os.path.join(REPO, "outputs", "phase24_localization", "triptych")
GT_CSV = os.path.join(RSRCH, "gt_input", "user_visibility_gt.csv")
CADDIR = os.path.join(REPO, "data", "cad")

# 클래스 고정 색 (BGR)
COLORS = {
    "Bear": (60, 180, 250), "Rabbit": (250, 250, 250), "Dinosaur": (90, 220, 90),
    "milk": (240, 200, 120), "choco_hazelnut_high": (80, 90, 190),
    "Febreze_high": (250, 170, 80), "Mugcup_high": (200, 130, 250),
    "saffron": (170, 230, 250), "Sauce_high": (60, 140, 240), "Sikhye_high": (70, 220, 240),
}
DEF_COLOR = (200, 200, 200)
_MESH = {}


def rle_to_mask(seg):
    h, w = seg["size"]; counts = seg["counts"]
    if isinstance(counts, str):
        return None
    arr = np.zeros(h * w, np.uint8); idx = 0; val = 0
    for c in counts:
        c = int(c); arr[idx:idx + c] = val; idx += c; val ^= 1
    return arr.reshape((h, w), order="F").astype(bool)


def load_K(p):
    try:
        d = json.load(open(p))
    except Exception:
        return None
    K = d.get("cam_K")
    return np.array(K, np.float64).reshape(3, 3) if K else None


def cad_points(obj, n=3000):
    """CAD 표면 균일 샘플 (mm 단위 가정).

    ⚠정점(vertex) 무작위추출을 쓰면 안 된다: 일부 PLY 는 정점 밀도가 극단적으로 편향돼 있다.
    예) Milk.ply 는 16,302 정점 중 16,266(99.8%)이 뚜껑(z157~196)에 몰려 있고 몸통은 정점 0개
    (큰 삼각형 몇 장). 정점 추출 시 '뚜껑만' 그려져 포즈가 틀린 것처럼 보인다.
    sample_surface 는 면적 비례 균일 샘플이라 이 왜곡이 없다.
    """
    if obj in _MESH:
        return _MESH[obj]
    cands = glob.glob(os.path.join(CADDIR, obj, "*.ply"))
    if not cands:
        for alt in (obj.replace("_high", "_color_high"), obj.replace("_high", "")):
            cands = glob.glob(os.path.join(CADDIR, alt, "*.ply"))
            if cands:
                break
    pts = None
    if cands:
        try:
            import trimesh
            m = trimesh.load(cands[0], process=False)
            try:
                s, _ = trimesh.sample.sample_surface(m, n)      # 면적 비례 균일
                pts = np.asarray(s, np.float64)
            except Exception:                                    # 면이 없으면 정점 fallback
                v = np.asarray(m.vertices, np.float64)
                if v.size:
                    pts = v[np.random.RandomState(0).choice(len(v), min(n, len(v)), replace=False)]
        except Exception:
            pts = None
    _MESH[obj] = pts
    return pts


def project(pts, K, R, t):
    R = np.array(R, np.float64).reshape(3, 3)
    t = np.array(t, np.float64).reshape(3)
    X = (R @ pts.T).T + t                      # camera frame (mm)
    X = X[X[:, 2] > 1e-6]
    if not len(X):
        return None
    uv = (K @ X.T).T
    return uv[:, :2] / uv[:, 2:3]


def draw_label(img, x, y, text, color):
    cv2.putText(img, text, (x, max(12, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
    cv2.putText(img, text, (x, max(12, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def panel_detect(rgb, fdir):
    img = rgb.copy(); ov = img.copy(); any_ = False
    for p in sorted(glob.glob(os.path.join(fdir, "detection_*.json"))):
        obj = os.path.basename(p)[len("detection_"):-len(".json")]
        if obj == "pem":
            continue
        try:
            d = json.load(open(p))
        except Exception:
            continue
        d = d if isinstance(d, list) else [d]
        col = COLORS.get(obj, DEF_COLOR)
        for det in d:
            seg = det.get("segmentation")
            if seg:
                m = rle_to_mask(seg)
                if m is not None and m.shape[:2] == img.shape[:2]:
                    ov[m] = col; any_ = True
            b = det.get("bbox")
            if b:
                x, y, w, h = [int(v) for v in b]
                cv2.rectangle(img, (x, y), (x + w, y + h), col, 2)
                draw_label(img, x, y, obj.replace("_high", ""), col)
                any_ = True
    img = cv2.addWeighted(ov, 0.45, img, 0.55, 0)
    return img, any_


def panel_pose(rgb, fdir, K):
    img = rgb.copy(); any_ = False
    if K is None:
        return img, False
    for pem in sorted(glob.glob(os.path.join(fdir, "pem_*", "sam6d_results", "detection_pem.json"))):
        obj = os.path.basename(os.path.dirname(os.path.dirname(pem)))[len("pem_"):]
        try:
            d = json.load(open(pem))
        except Exception:
            continue
        d = d if isinstance(d, list) else [d]
        col = COLORS.get(obj, DEF_COLOR)
        pts = cad_points(obj)
        for det in d:
            R, t = det.get("R"), det.get("t")
            if R is None or t is None:
                continue
            if pts is not None:
                uv = project(pts, K, R, t)
                if uv is not None:
                    h, w = img.shape[:2]
                    uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)]
                    for u, v in uv.astype(int):
                        cv2.circle(img, (u, v), 1, col, -1)
                    if len(uv):
                        any_ = True
                        draw_label(img, int(uv[:, 0].min()), int(uv[:, 1].min()),
                                   obj.replace("_high", ""), col)
            else:   # CAD 없으면 좌표축만
                rv, _ = cv2.Rodrigues(np.array(R, np.float64).reshape(3, 3))
                ax = np.float64([[0, 0, 0], [40, 0, 0], [0, 40, 0], [0, 0, 40]])
                pr, _ = cv2.projectPoints(ax, rv, np.array(t, np.float64).reshape(3, 1), K, None)
                pr = pr.reshape(-1, 2).astype(int)
                for i, c in enumerate([(0, 0, 255), (0, 255, 0), (255, 0, 0)], start=1):
                    cv2.line(img, tuple(pr[0]), tuple(pr[i]), c, 2)
                any_ = True
    return img, any_


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=int, default=0, help="총 상한(0=전체)")
    ap.add_argument("--only", default="", help="특정 ds_frame 하나만 (테스트용)")
    a = ap.parse_args()

    gt = {}
    for r in csv.DictReader(open(GT_CSV, encoding="utf-8")):
        if r["user_reviewed"] == "yes":
            gt.setdefault(r["dataset_name"], set()).add(int(r["frame_id"]))

    frames = []
    for fdir in sorted(glob.glob(os.path.join(PEMIN, "*", "frame_*"))):
        if not os.path.isdir(fdir):
            continue
        ds = os.path.basename(os.path.dirname(fdir))
        try:
            fr = int(os.path.basename(fdir).split("_")[1])
        except Exception:
            continue
        if not os.path.isfile(os.path.join(fdir, "rgb.png")):
            continue
        has_pem = bool(glob.glob(os.path.join(fdir, "pem_*", "sam6d_results", "detection_pem.json")))
        if not has_pem:
            continue
        frames.append((ds, fr, fdir, fr in gt.get(ds, set())))
    # GT 프레임 우선 정렬
    frames.sort(key=lambda x: (not x[3], x[0], x[1]))
    if a.only:
        frames = [f for f in frames if f"{f[0]}_{f[1]:06d}" == a.only]
    if a.limit:
        frames = frames[:a.limit]

    os.makedirs(os.path.join(OUT, "gt"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "pem"), exist_ok=True)
    n_gt = n_pem = 0
    for ds, fr, fdir, isgt in frames:
        rgb = cv2.imread(os.path.join(fdir, "rgb.png"))
        if rgb is None:
            continue
        K = load_K(os.path.join(fdir, "camera.json"))
        det, ok1 = panel_detect(rgb, fdir)
        pose, ok2 = panel_pose(rgb, fdir, K)
        combo = np.hstack([rgb, det, pose])
        sub = "gt" if isgt else "pem"
        cv2.imwrite(os.path.join(OUT, sub, f"{ds}_{fr:06d}.png"), combo)
        n_gt += isgt; n_pem += (not isgt)
    print(f"=== triptych 생성 ===")
    print(f"  GT 프레임 : {n_gt}")
    print(f"  기타 PEM  : {n_pem}")
    print(f"  총 {n_gt+n_pem}장 -> {OUT}")


if __name__ == "__main__":
    main()
