#!/usr/bin/env python3
"""render_combo_overlay.py — ObChange 방문마다 실제 카메라 영상 위에 정답과 각 방법의 최종 지도 객체를 겹쳐 그린다(objlist_combo.py 의 objlist.json 사용).

  프레임   채점 대상 정답 물체가 가장 많이 보이는 프레임 하나 + 거기서 안 보인 물체가 가장 많이 보이는 프레임 하나
  칸       같은 프레임을 두 장으로: a = 정답 + 자세 추정 구성 7가지(4열 2행), b = 정답 + 제안 방법(E) + FM-Fusion · DualMap 의 원래 이름 / CAD 이름(3열 2행)
  객체     대상 물체 이름이 붙은 지도 객체 전부를 그 프레임에 투영한 윤곽. 자세를 저장하는 방법은 CAD 점, 나머지는 복원 점(깊이 영상과 3 cm 안에서 맞는 점만).
  색       초록 = 맞음, 노랑 = 중복(같은 물체가 한 번 더 올라갔거나 일부만 잡힘), 보라 = 다른 물체 자리(이름을 틀림), 빨강 = 정답 없는 자리
  이름표   맞음은 모두, 나머지는 화면에서 큰 순서로 --max-labels 개까지
    <sam6d python> objpose/tools/obchange/render_combo_overlay.py --objlist <json> --out <dir>
"""
import argparse, json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

OB = Path("/home/jucpark/DeepLearning/Datasets/ObChange/bop")
KO = {"002_master_chef_can": "커피 캔", "003_cracker_box": "크래커 상자", "004_sugar_box": "설탕 상자", "005_tomato_soup_can": "수프 캔", "008_pudding_box": "푸딩 상자", "009_gelatin_box": "젤라틴 상자",
      "019_pitcher_base": "피처", "024_bowl": "그릇", "025_mug": "머그컵", "040_large_marker": "마커", "061_foam_brick": "스펀지 벽돌"}
LAB = {"A": "A. 원본 SAM-6D", "B": "B. 화면 전체 분할+원본 판정+검증", "C_on": "C. 문구 상자+원본 판정+검증", "C_off": "C′. C에서 검증 끔", "D": "D. 제안 방법, 완화", "E_on": "E. 제안 방법, 배포", "E_off": "E′. E에서 검증 끔",
       "FM-Fusion": "FM-Fusion", "FM-Fusion + CAD": "FM-Fusion + CAD 이름", "DualMap": "DualMap", "DualMap + CAD": "DualMap + CAD 이름"}
ORDER = ["A", "B", "C_on", "C_off", "D", "E_on", "E_off", "FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"]
COL = {"맞음": (46, 125, 50), "중복": (214, 160, 0), "다른 물체 자리": (123, 31, 162), "정답 없는 자리": (198, 40, 40)}
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"; F = ImageFont.truetype(FB, 17); FT = ImageFont.truetype(FB, 21)


def cam_of(cam, k): return np.array(cam[k]["cam_K"]).reshape(3, 3), np.array(cam[k]["cam_R_w2c"]).reshape(3, 3), np.array(cam[k]["cam_t_w2c"]) / 1000


def project(P, K, R, t):
    X = np.asarray(P) @ R.T + t; z = X[:, 2]
    uv = np.c_[K[0, 0] * X[:, 0] / np.maximum(z, 1e-6) + K[0, 2], K[1, 1] * X[:, 1] / np.maximum(z, 1e-6) + K[1, 2]]
    return uv, z, (z > 0.2) & (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)


def pick_frames(scene, cen):
    cam = json.loads((OB / "test" / scene / "scene_camera.json").read_text()); keys = sorted(cam, key=int)[::5]
    def vis(k):
        uv, z, ok = project(cen, *cam_of(cam, k)); return {i for i in range(len(cen)) if ok[i] and 25 < uv[i, 0] < 615 and 25 < uv[i, 1] < 455}
    V = {k: vis(k) for k in keys}; k1 = max(keys, key=lambda k: len(V[k])); rest = set(range(len(cen))) - V[k1]
    k2 = max(keys, key=lambda k: (len(V[k] & rest), len(V[k]))) if rest else None
    return cam, [k1] + ([k2] if k2 is not None and V[k2] & rest else [])


def label(d, xy, text, col, taken):
    w = d.textlength(text, font=F); x = min(max(xy[0] - w / 2, 2), 638 - w); y = min(max(xy[1], 32), 456)
    for _ in range(6):                                   # 이름표끼리 겹치면 아래로 민다
        if all(not (x < bx + bw and bx < x + w and abs(y - by) < 21) for bx, by, bw in taken): break
        y = min(y + 21, 456)
    taken.append((x, y, w)); d.rectangle([x - 3, y - 1, x + w + 3, y + 21], fill=(255, 255, 255)); d.text((x, y - 2), text, fill=col, font=F)


def finish(rgb, ov, title, notes):
    out = Image.alpha_composite(Image.fromarray(rgb).convert("RGBA"), ov).convert("RGB"); d = ImageDraw.Draw(out); taken = []
    for xy, text, col in notes: label(d, xy, text, col, taken)
    w = d.textlength(title, font=FT); d.rectangle([0, 0, w + 14, 30], fill=(255, 255, 255)); d.text((7, -1), title, fill=(0, 0, 0), font=FT)
    return np.asarray(out)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--objlist", type=Path, required=True); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--max-labels", type=int, default=7); ap.add_argument("--scenes", nargs="*", default=None)
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True); index = {}
    for r in json.loads(a.objlist.read_text()):
        sc = r["scene"]
        if a.scenes and sc not in a.scenes: continue
        gts = [g for g in r["gt"] if g["center"]]; tg = [g for g in gts if g["target"]]
        if not tg: continue
        cam, keys = pick_frames(sc, [g["center"] for g in tg]); index[sc] = []
        for vi, k in enumerate(keys):
            K, R, t = cam_of(cam, k); rgb0 = cv2.imread(str(OB / "test" / sc / "rgb" / f"{int(k):06d}.png"))[:, :, ::-1].copy()
            dep = cv2.imread(str(OB / "test" / sc / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) * float(cam[k].get("depth_scale", 1.0)) / 1000.0
            uvt, _, okt = project([g["center"] for g in tg], K, R, t); crops = [None]
            if okt.any():                                   # 확대: 보이는 대상 물체를 둘러싼 4:3 영역
                x1, y1 = uvt[okt].min(0) - 70; x2, y2 = uvt[okt].max(0) + 70; cw = max(x2 - x1, (y2 - y1) * 4 / 3, 240); ch = cw * 3 / 4; cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                x0 = float(np.clip(cx - cw / 2, 0, 640 - cw)); y0 = float(np.clip(cy - ch / 2, 0, 480 - ch))
                if cw <= 460: crops.append((x0, y0, 640 / cw))
            for ci, crop in enumerate(crops):
                if crop is None: rgb = rgb0; tf = lambda q: q; inside = lambda q: np.ones(len(q), bool)
                else:
                    x0, y0, sc_ = crop; cw = 640 / sc_; rgb = cv2.resize(rgb0[int(y0):int(y0 + cw * 3 / 4), int(x0):int(x0 + cw)], (640, 480), interpolation=cv2.INTER_CUBIC)
                    tf = lambda q, x0=x0, y0=y0, sc_=sc_: (np.asarray(q) - [x0, y0]) * sc_; inside = lambda q: (q[:, 0] >= 0) & (q[:, 0] < 640) & (q[:, 1] >= 0) & (q[:, 1] < 480)
                ov = Image.new("RGBA", (640, 480), (0, 0, 0, 0)); d = ImageDraw.Draw(ov); notes = []
                uv, z, ok = project([g["center"] for g in gts], K, R, t); uvz = tf(uv); okz = ok & inside(uvz)
                for g, pp, v in zip(gts, uvz, okz):
                    if not v: continue
                    col = (0, 0, 0) if g["target"] else (120, 120, 120)
                    d.ellipse([pp[0] - 11, pp[1] - 11, pp[0] + 11, pp[1] + 11], outline=col + (255,), width=3); notes.append(((pp[0], pp[1] + 13), KO.get(g["name"], "대상 밖의 물체"), col))
                tiles = [finish(rgb, ov, f"정답 (대상 물체 {len(tg)}개)", notes)]
                for m in ORDER:
                    if m not in r["methods"]: continue
                    objs = [o for o in r["methods"][m]["objects"] if o["verdict"] != "이름 없음"]; ov = Image.new("RGBA", (640, 480), (0, 0, 0, 0)); d = ImageDraw.Draw(ov); drawn = []
                    for o in sorted(objs, key=lambda o: o["verdict"] == "맞음"):       # 맞음을 맨 위에 그린다
                        P = np.asarray(o["pts"], float)
                        if len(P) < 4: continue
                        q, zz, v = project(P, K, R, t)
                        if not v.any(): continue
                        ui, vi_ = q[v, 0].astype(int), q[v, 1].astype(int); dd = dep[vi_, ui]; tol = 0.10 if o["cad"] else 0.03      # 깊이 영상과 맞는 점만 = 그 프레임에서 가려지지 않은 부분
                        idx = np.where(v)[0][(dd <= 0) | (np.abs(dd - zz[v]) < tol)]
                        if len(idx) < max(4, (0.35 if o["cad"] else 0.15) * len(P)): continue
                        qz = tf(q[idx]); 
                        if inside(qz).sum() < 0.5 * len(idx): continue
                        col = COL[o["verdict"]]; hull = cv2.convexHull(qz.astype(np.float32)).reshape(-1, 2)
                        if len(hull) < 3: continue
                        d.polygon([tuple(pp) for pp in hull], fill=col + (60,)); d.line([tuple(pp) for pp in hull] + [tuple(hull[0])], fill=col + (255,), width=3)
                        drawn.append((o, float(cv2.contourArea(hull)), (float(qz[:, 0].mean()), float(qz[:, 1].max()) + 3)))
                    ok_ = [x for x in drawn if x[0]["verdict"] == "맞음"]; bad = sorted((x for x in drawn if x[0]["verdict"] != "맞음"), key=lambda x: -x[1])
                    notes = [(xy, KO.get(o["name"], o["name"]), COL[o["verdict"]]) for o, _, xy in ok_ + bad[: a.max_labels]]
                    n_ok = sum(o["verdict"] == "맞음" for o in objs)
                    tiles.append(finish(rgb, ov, f"{LAB[m]} · 맞음 {n_ok}, 잘못 등록 {len(objs) - n_ok}", notes))
                names = ["정답"] + [m for m in ORDER if m in r["methods"]]; T = dict(zip(names, tiles))
                for part, sel, ncol in (("a", ["정답", "A", "B", "C_on", "C_off", "D", "E_on", "E_off"], 4), ("b", ["정답", "E_on", "FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"], 3)):
                    tl = [T[x] for x in sel if x in T]
                    while len(tl) % ncol: tl.append(np.full((480, 640, 3), 255, np.uint8))
                    rows = [np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tl[i:i + ncol]], 1)[:, :-8] for i in range(0, len(tl), ncol)]
                    fig = np.concatenate([np.pad(x, ((0, 8), (0, 0), (0, 0)), constant_values=255) for x in rows], 0)[:-8]
                    name = f"overlay_{sc}_v{vi + 1}{'_zoom' if crop else ''}_{part}.jpg"; Image.fromarray(fig).save(a.out / name, quality=86); index[sc].append({"file": name, "frame": int(k), "view": vi + 1, "zoom": crop is not None, "part": part})
        print(sc, len(index[sc]), flush=True)
    (a.out / "index.json").write_text(json.dumps(index))


if __name__ == "__main__":
    main()
