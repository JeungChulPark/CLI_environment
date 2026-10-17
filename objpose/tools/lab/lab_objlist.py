#!/usr/bin/env python3
"""lab_objlist.py — 직접 녹화한 세션마다 각 방법의 최종 지도에 등록된 객체 목록을 뽑고, 실제 영상 위에 겹쳐 그린다(lab_combo_261010 용). 이 세션들은 정답 자세가 없다.

  objlist.json   세션 → 방법 → 객체 [{name, center, R(자세를 저장하는 방법만), n_obs, extra, pts}]
                 extra = 같은 종류의 객체가 여럿일 때, 관측 횟수(점군 지도는 점 수)가 가장 많은 하나를 뺀 나머지. 장면에 종류마다 물체가 하나뿐이므로 초과 등록이다.
  media/lab_<세션>_v<번호>_{a,b}.jpg
                 a = 실제 영상 + 자세 추정 구성 7가지, b = 실제 영상 + 제안 방법(E) + FM-Fusion · DualMap 의 원래 이름 / CAD 이름
                 프레임은 제안 방법(E) 지도의 객체가 많이 보이는 것부터 서로 다른 객체가 보이도록 --views 장까지 고른다.
                 윤곽 색: 초록 = 그 종류에서 관측이 가장 많은 객체, 빨강 = 같은 종류의 초과 등록. 그 프레임에서 가려진 객체는 그리지 않는다. (정답이 없어 초록이 옳은 자리인지는 그림으로 확인해야 한다)
    <sam6d python> objpose/tools/lab/lab_objlist.py --root objpose/output/lab_combo_261010 --out objpose/docs/combo_obchange_261009/media_lab
"""
import argparse, collections, json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[3]; BOP = Path.home() / "DeepLearning/Datasets/lab_sessions_bop/test"; PTS = REPO / "sam6d_realtime/assets/model_points"; TAG = "lab_rt_261010"
TAB = json.loads((REPO / "objpose/rt/bop/lab8_objects.json").read_text())["objects"]; NAMES = [o["name"] for o in TAB]; BY_YCB = {o["ycb"]: o["name"] for o in TAB}; BY_LABEL = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in TAB}
KO = {"milk": "우유", "choco_hazelnut_high": "초코 상자", "Febreze_high": "페브리즈", "Mugcup_high": "머그컵", "saffron": "세제 통", "Sikhye_high": "식혜 캔", "Bear": "곰 인형", "Dinosaur": "공룡 인형"}
POSE = ["A", "B", "C", "C_off", "D", "E", "E_off"]; ORDER = POSE + ["FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"]
LAB = {"A": "A. 원본 SAM-6D", "B": "B. 화면 전체 분할+원본 판정+검증", "C": "C. 문구 상자+원본 판정+검증", "C_off": "C′. C에서 검증 끔", "D": "D. 제안 방법, 완화", "E": "E. 제안 방법, 배포", "E_off": "E′. E에서 검증 끔",
       "FM-Fusion": "FM-Fusion", "FM-Fusion + CAD": "FM-Fusion + CAD 이름", "DualMap": "DualMap", "DualMap + CAD": "DualMap + CAD 이름"}
G, R_ = (46, 125, 50), (198, 40, 40); FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"; F = ImageFont.truetype(FB, 17); FT = ImageFont.truetype(FB, 21)
ap = argparse.ArgumentParser(); ap.add_argument("--root", type=Path, required=True); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--views", type=int, default=3); ap.add_argument("--max-labels", type=int, default=10); ap.add_argument("--only", nargs="*", default=None)
a = ap.parse_args(); O = a.root.resolve(); a.out.mkdir(parents=True, exist_ok=True); rng = np.random.default_rng(0)
MODEL = {n: np.load(PTS / f"{n}.npy") / 1000.0 for n in NAMES}


def read_ply_xyz(p):
    raw = p.read_bytes(); end = raw.index(b"end_header\n") + len(b"end_header\n"); hd = raw[:end].decode()
    n = int(next(l.split()[-1] for l in hd.splitlines() if l.startswith("element vertex")))
    dt = np.dtype([(pr[2], {"double": "<f8", "float": "<f4", "uchar": "u1"}[pr[1]]) for pr in (l.split() for l in hd.splitlines() if l.startswith("property"))])
    v = np.frombuffer(raw[end:end + n * dt.itemsize], dt); return np.stack([v["x"], v["y"], v["z"]], 1).astype(float)


def sub(P, n=160): return np.round(P[rng.choice(len(P), min(len(P), n), replace=False)], 3).tolist()


def load(m, s):
    if m in POSE:
        f = O / m / "maps" / f"{s}.json"
        if not f.exists(): return None
        return [{"name": o["name"], "center": o["center"], "R": o["R"], "n_obs": o["n_obs"], "weight": o["n_obs"], "cad": True, "pts": sub(MODEL[o["name"]] @ np.asarray(o["R"]).T + np.asarray(o["center"]))} for o in json.loads(f.read_text())]
    cad = m.endswith("+ CAD")
    if m.startswith("FM-Fusion"):
        sd = O / "fmfusion_rt" / s
        if not (sd / "instance_info.txt").exists(): return None
        ren = None
        if cad:
            f = O / "cad_rt/fmf_names.json"
            if not f.exists() or s not in json.loads(f.read_text()): return None
            ren = json.loads(f.read_text())[s]
        out = []
        for line in (sd / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip(): continue
            f = line.split(";"); Q = read_ply_xyz(sd / f"{int(f[0]):04d}.ply")
            if not len(Q): continue
            nm = BY_YCB.get(ren.get(str(int(f[0])))) if cad else BY_LABEL.get(f[1].split("(")[0])
            out.append({"name": nm, "center": Q.mean(0).tolist(), "R": None, "n_obs": int(f[2]) if f[2].strip().isdigit() else None, "weight": len(Q), "cad": False, "pts": sub(Q)})
        return out
    f = (O / "cad_rt/dualmap" if cad else O / "dualmap_rt/maps" / TAG) / f"{s}.json"
    if not f.exists(): return None
    return [{"name": BY_YCB.get(o["name"]), "center": o["center"], "R": None, "n_obs": o.get("n_obs"), "weight": o.get("n_pts", len(o["pts"])), "cad": False, "pts": sub(np.asarray(o["pts"], float))} for o in json.loads(f.read_text()) if len(o["pts"])]


def project(P, K, T):
    X = np.asarray(P) @ T[:3, :3].T + T[:3, 3]; z = X[:, 2]
    uv = np.c_[K[0, 0] * X[:, 0] / np.maximum(z, 1e-6) + K[0, 2], K[1, 1] * X[:, 1] / np.maximum(z, 1e-6) + K[1, 2]]
    return uv, z, (z > 0.2) & (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)


def quat_to_rot(q):
    x, y, z, w = q; return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def label(d, xy, text, col, taken):
    w = d.textlength(text, font=F); x = min(max(xy[0] - w / 2, 2), 638 - w); y = min(max(xy[1], 32), 456)
    for _ in range(8):
        if all(not (x < bx + bw and bx < x + w and abs(y - by) < 21) for bx, by, bw in taken): break
        y = min(y + 21, 456)
    taken.append((x, y, w)); d.rectangle([x - 3, y - 1, x + w + 3, y + 21], fill=(255, 255, 255)); d.text((x, y - 2), text, fill=col, font=F)


def finish(rgb, ov, title, notes):
    out = Image.alpha_composite(Image.fromarray(rgb).convert("RGBA"), ov).convert("RGB"); d = ImageDraw.Draw(out); taken = []
    for xy, text, col in notes: label(d, xy, text, col, taken)
    w = d.textlength(title, font=FT); d.rectangle([0, 0, w + 14, 30], fill=(255, 255, 255)); d.text((7, -1), title, fill=(0, 0, 0), font=FT)
    return np.asarray(out)


sessions = sorted(p.name for p in BOP.iterdir() if (p / "scene_camera.json").exists()); OUT = {}; index = {}
for s in sessions:
    rec = {}
    for m in ORDER:
        if a.only and m not in a.only: continue
        objs = load(m, s)
        if objs is None: continue
        best = {}
        for i, o in enumerate(objs):
            if o["name"] and (o["name"] not in best or o["weight"] > objs[best[o["name"]]]["weight"]): best[o["name"]] = i
        for i, o in enumerate(objs): o["extra"] = bool(o["name"]) and best[o["name"]] != i
        rec[m] = objs
    OUT[s] = rec
    # ---- 프레임 고르기: 제안 방법(E) 지도의 객체가 많이 보이는 프레임부터
    cam = json.loads((BOP / s / "scene_camera.json").read_text()); keys = sorted(cam, key=int); K = np.array(cam[keys[0]]["cam_K"]).reshape(3, 3); Tcw = {}
    for r in np.loadtxt(O / "slam" / f"{s}_frames.txt", ndmin=2):
        T = np.eye(4); T[:3, :3] = quat_to_rot(r[4:8]); T[:3, 3] = r[1:4]; Tcw[int(round(r[0] * 10))] = np.linalg.inv(T)
    ref = [o["center"] for o in rec.get("E", []) if not o["extra"]] or [o["center"] for m in rec for o in rec[m] if o["name"]][:20]
    cand = [i for i in sorted(Tcw) if i % 5 == 0 and i < len(keys)]; V = {}
    for i in cand:
        if not ref: V[i] = set(); continue
        uv, z, ok = project(ref, K, Tcw[i]); dp = cv2.imread(str(BOP / s / "depth" / f"{int(keys[i]):06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) * 0.0001
        V[i] = {j for j in range(len(ref)) if ok[j] and z[j] < 2.5 and 40 < uv[j, 0] < 600 and 40 < uv[j, 1] < 440 and abs(dp[int(uv[j, 1]), int(uv[j, 0])] - z[j]) < 0.15}
    picks = []; seen = set()
    for _ in range(a.views):
        rest = [i for i in cand if all(abs(i - p) > 150 for p in picks)]          # 서로 15초 이상 떨어진 시점
        if not rest: break
        i = max(rest, key=lambda i: (len(V[i] - seen), len(V[i])))
        if picks and not V[i] - seen and len(V[i]) == 0: break
        picks.append(i); seen |= V[i]
    index[s] = []
    for vi, i in enumerate(picks):
        rgb = cv2.imread(str(BOP / s / "rgb" / f"{int(keys[i]):06d}.png"))[:, :, ::-1].copy(); c = cam[keys[i]]
        dep = cv2.imread(str(BOP / s / "depth" / f"{int(keys[i]):06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) * float(c["depth_scale"]) / 1000.0; T = Tcw[i]; tiles = {"실제 영상": finish(rgb, Image.new("RGBA", (640, 480), (0, 0, 0, 0)), "실제 영상", [])}
        for m in rec:
            ov = Image.new("RGBA", (640, 480), (0, 0, 0, 0)); d = ImageDraw.Draw(ov); drawn = []; named = [o for o in rec[m] if o["name"]]
            for o in sorted(named, key=lambda o: not o["extra"]):
                P = np.asarray(o["pts"], float)
                if len(P) < 4: continue
                q, zz, v = project(P, K, T)
                if not v.any(): continue
                ui, vj = q[v, 0].astype(int), q[v, 1].astype(int); dd = dep[vj, ui]; tol = 0.10 if o["cad"] else 0.03        # 깊이 영상과 맞는 점만 = 그 프레임에서 가려지지 않은 부분
                idx = np.where(v)[0][(dd <= 0) | (np.abs(dd - zz[v]) < tol)]
                if len(idx) < max(4, (0.35 if o["cad"] else 0.15) * len(P)): continue
                col = R_ if o["extra"] else G; hull = cv2.convexHull(q[idx].astype(np.float32)).reshape(-1, 2)
                if len(hull) < 3: continue
                d.polygon([tuple(p) for p in hull], fill=col + (60,)); d.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=col + (255,), width=3)
                drawn.append((o, float(cv2.contourArea(hull)), (float(q[idx, 0].mean()), float(q[idx, 1].max()) + 3)))
            first = [x for x in drawn if not x[0]["extra"]]; ex = sorted((x for x in drawn if x[0]["extra"]), key=lambda x: -x[1])
            notes = [(xy, KO[o["name"]], R_ if o["extra"] else G) for o, _, xy in first + ex[: a.max_labels]]
            tiles[m] = finish(rgb, ov, f"{LAB[m]} · 등록 {len(named)}, 초과 {sum(o['extra'] for o in named)}", notes)
        for part, sel, ncol in (("a", ["실제 영상"] + POSE, 4), ("b", ["실제 영상", "E", "FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"], 3)):
            tl = [tiles[x] for x in sel if x in tiles]
            if len(tl) < 2: continue
            while len(tl) % ncol: tl.append(np.full((480, 640, 3), 255, np.uint8))
            rows = [np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tl[j:j + ncol]], 1)[:, :-8] for j in range(0, len(tl), ncol)]
            fig = np.concatenate([np.pad(x, ((0, 8), (0, 0), (0, 0)), constant_values=255) for x in rows], 0)[:-8]
            name = f"lab_{s}_v{vi + 1}_{part}.jpg"; Image.fromarray(fig).save(a.out / name, quality=86); index[s].append({"file": name, "frame": int(i), "view": vi + 1, "part": part, "t_s": i / 10.0})
    print(s, {m: (sum(bool(o["name"]) for o in v), sum(o["extra"] for o in v)) for m, v in rec.items()}, "frames", picks, flush=True)
(a.root / "objlist.json").write_text(json.dumps(OUT, ensure_ascii=False)); (a.out / "index.json").write_text(json.dumps(index))
