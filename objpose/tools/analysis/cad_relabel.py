#!/usr/bin/env python3
"""cad_relabel.py — 점군 지도(FM-Fusion, DualMap)의 객체 이름을 제안 방법의 CAD 템플릿 검증으로 다시 붙인다.
"비교 방법에도 CAD 를 주면 어떻게 되는가"를 보기 위한 실험. 객체의 점군(어디에 무엇이 한 덩어리로 있는가)은 그대로 두고 이름만 바꾼다.

  1) 영상을 실시간 재생으로 본다. 각 방법이 지도에 넣는 프레임(--gap, 예: 30 fps 의 10장마다)이 도착할 때마다 이름 판별을 한 번 한다.
     판별에 걸린 실제 시간만큼 시계를 보내고, 그 사이 도착한 프레임이 여럿이면 가장 최신 것만 쓴다(나머지는 버림). 장면 전체 프레임이 대상이다.
  2) 지도 객체의 점을 그 프레임에 투영해 2D 상자를 만든다(깊이 영상과 3 cm 안에서 맞는 점만 = 그 프레임에서 실제로 보이는 부분)
  3) 그 상자들을 후보로 제안 방법의 후보 검증(yolo_ism_object_n: DINOv2 의미 유사도 → MobileSAM 마스크 → 마스크 외형 유사도, 설정은 configs/yolo_ism_ycbv.yaml 그대로)에 넣는다.
     문구 검출기는 쓰지 않는다. 모든 물체가 모든 상자를 후보로 본다(top_k 제한만 풂).
  4) "이 상자는 이 물체"로 통과한 결과를 객체별로 모아, 가장 많이 나온 이름이 3번 이상이고 그 객체가 보인 프레임의 10 % 이상이면 그 이름을 붙인다. 아니면 이름 없음.
  --data ycb      eval_maps_f1.py --dump 의 장면 json 을 읽고 같은 형식으로 쓴다(BENCH_ROOT/BENCH_TABLE/BENCH_FPS 로 YCB-V/YCB-M 선택). 찾을 물체 = 그 장면의 정답 물체(제안 방법의 YCB 실행과 같음)
  --data obchange FM-Fusion 은 objpose/output/obchange_261005/fmfusion/<scene>/ 의 객체 전부, DualMap 은 maps_final/dualmap/<scene>.json. 찾을 물체 = 11종 전부.
                  결과 <out>/fmf_names.json (장면 → 객체 번호 → 이름), <out>/dualmap/<scene>.json → inst_iou_obchange.py 의 FMF_NAMES / DUALMAP_MAPS
  판별 횟수 · 버린 프레임 · 판별 시간은 <out>/stream_stats.json
    <sam6d python> objpose/tools/analysis/cad_relabel.py --data ycb --dump <dump dir> --out <new dump dir> --gap FM-Fusion=10 DualMap=10 [--scenes …]
"""
import argparse, collections, json, os, sys, time
from pathlib import Path
import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]; SAM6D = REPO / "sam6d_realtime"
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--data", choices=["ycb", "obchange", "lab"], required=True); ap.add_argument("--dump", type=Path, default=None); ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--gap", nargs="+", default=["FM-Fusion=10", "DualMap=10"], help="방법마다 지도에 넣는 프레임 간격"); ap.add_argument("--device", default="cuda:0")
ap.add_argument("--poses", type=Path, default=None, help="(lab) 카메라 자세 폴더 <세션>_frames.txt"); ap.add_argument("--fps", type=float, default=10.0)
ap.add_argument("--fmf-dir", type=Path, default=None, help="(obchange) FM-Fusion 결과 폴더"); ap.add_argument("--dualmap-maps", type=Path, default=None, help="(obchange) DualMap 지도 json 폴더")
a = ap.parse_args(); GAP = {k: int(v) for k, v in (x.split("=") for x in a.gap)}; a.out = a.out.resolve(); a.dump = a.dump.resolve() if a.dump else None; a.out.mkdir(parents=True, exist_ok=True)
OB = Path("/home/jucpark/DeepLearning/Datasets/lab_sessions_bop") if a.data == "lab" else Path("/home/jucpark/DeepLearning/Datasets/ObChange/bop"); OBO = REPO / "objpose/output/obchange_261005"; OBM = OBO / "maps_final"; STATS = []
table = json.loads((BM.TABLE if a.data == "ycb" else REPO / ("objpose/rt/bop/lab8_objects.json" if a.data == "lab" else "objpose/rt/bop/obchange_objects.json")).read_text())["objects"]
by_id = {o["obj_id"]: o["name"] for o in table}; ycb_of = {o["name"]: o["ycb"] for o in table}; name_of = {o["ycb"]: o["name"] for o in table}

sys.path.insert(0, str(SAM6D)); os.chdir(SAM6D)
import torch                                   # noqa: E402
import yolo_ism as yi                          # noqa: E402
import yolo_ism_object_n as o_n                # noqa: E402
defaults, all_objs = o_n.load_config(str(SAM6D / ("configs/yolo_ism_objects.yaml" if a.data == "lab" else "configs/yolo_ism_ycbv.yaml")))
model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, a.device)
segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), a.device); pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)


def boxes_of(objs_pts, K, Tcw, dep):
    """지도 객체마다 그 프레임에서 보이는 부분의 2D 상자(없으면 None)."""
    out = []
    H, W = dep.shape
    for P in objs_pts:
        if len(P) < 10: out.append(None); continue
        X = P @ Tcw[:3, :3].T + Tcw[:3, 3]; z = X[:, 2]; u = K[0, 0] * X[:, 0] / np.maximum(z, 1e-6) + K[0, 2]; v = K[1, 1] * X[:, 1] / np.maximum(z, 1e-6) + K[1, 2]
        ok = (z > 0.2) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
        if ok.sum() < 8: out.append(None); continue
        ui, vi = u[ok].astype(int), v[ok].astype(int); dd = dep[vi, ui]; keep = (dd <= 0) | (np.abs(dd - z[ok]) < 0.03)
        if keep.sum() < max(8, 0.15 * len(P)): out.append(None); continue
        uu, vv = u[ok][keep], v[ok][keep]
        x1, x2 = np.percentile(uu, [1, 99]); y1, y2 = np.percentile(vv, [1, 99])
        b = [int(max(0, x1 - 3)), int(max(0, y1 - 3)), int(min(W, x2 + 3)), int(min(H, y2 + 3))]
        out.append(b if (b[2] - b[0] >= 14 and b[3] - b[1] >= 14 and (b[2] - b[0]) * (b[3] - b[1]) < 0.5 * W * H) else None)
    return out


def iou(p, q):
    ix = max(0, min(p[2], q[2]) - max(p[0], q[0])); iy = max(0, min(p[3], q[3]) - max(p[1], q[1])); inter = ix * iy
    return inter / max((p[2] - p[0]) * (p[3] - p[1]) + (q[2] - q[0]) * (q[3] - q[1]) - inter, 1)


def relabel(scene, names, n_frames, fps, load, methods_pts):
    """names: 찾을 물체(ycbv_ 이름). load(k) → (bgr, depth_m, K, Tcw) or None. methods_pts: {방법: [점 배열]} → {방법: [(새 이름 or None, 득표, 보인 프레임 수)]}"""
    objs = o_n.prepare_objects([dict(o) for o in all_objs if o["name"] in names], model, a.device, rebuild=False)
    for o in objs: o["top_k"] = 200                                   # 상자에 순위가 없으므로 모든 상자를 후보로 본다
    prompts, groups = o_n.build_prompt_groups(objs); tsim = o_n.template_similarity(objs); out = {}

    def step(fr, pts, votes, seen):
        bgr, dep, K, Tcw = fr; rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB); norm = yi.normalize_rgb(rgb)
        bx = boxes_of(pts, K, Tcw, dep); idx = [i for i, b in enumerate(bx) if b is not None]
        for i in idx: seen[i] += 1
        if not idx: return
        cand = [(bx[i], 1.0) for i in idx]; pb = {pi: list(cand) for pi in groups}
        res = o_n.recognize_frame_auto(groups, pb, bgr, rgb, norm, model, a.device, segmentor, pool, tsim)
        for nm, r in res.items():
            if r.get("accepted") and r.get("box") is not None:
                j = max(idx, key=lambda i: iou(bx[i], r["box"]))
                if iou(bx[j], r["box"]) > 0.9: votes[j][nm] += 1

    for m, pts in methods_pts.items():
        cand = list(range(0, n_frames, GAP[m])); votes = [collections.Counter() for _ in pts]; seen = [0] * len(pts)
        fr = load(cand[0])
        if fr is not None: step(fr, pts, [collections.Counter() for _ in pts], [0] * len(pts))      # 첫 호출의 준비 시간은 재생 시간에 넣지 않는다
        t = 0.0; j = 0; done = 0; cost = []
        while j < len(cand):
            fr = load(cand[j]); t = max(t, cand[j] / fps)
            if fr is not None:
                t0 = time.perf_counter(); step(fr, pts, votes, seen); dt = time.perf_counter() - t0; t += dt; cost.append(dt); done += 1
            nj = j + 1
            while nj + 1 < len(cand) and cand[nj + 1] / fps <= t: nj += 1                             # 판별하는 동안 도착한 프레임은 최신 것만
            j = nj
        out[m] = []
        for c, s in zip(votes, seen):
            top = c.most_common(1)
            out[m].append((top[0][0] if top and top[0][1] >= 3 and top[0][1] >= 0.1 * s else None, top[0][1] if top else 0, s))
        STATS.append({"scene": scene, "method": m, "objects": len(pts), "frames": n_frames, "offered": len(cand), "processed": done, "ms_mean": 1000 * float(np.mean(cost)) if cost else None,
                      "ms_p95": 1000 * float(np.percentile(cost, 95)) if cost else None, "interval_ms": 1000 * GAP[m] / fps})
    return out


def kind(x, y): return "같음" if x == y else ("이름 없음→이름" if x is None else ("이름→이름 없음" if y is None else "다른 이름"))


def read_ply_xyz(p):
    raw = p.read_bytes(); end = raw.index(b"end_header\n") + len(b"end_header\n"); hd = raw[:end].decode()
    n = int(next(l.split()[-1] for l in hd.splitlines() if l.startswith("element vertex")))
    dt = np.dtype([(pr[2], {"double": "<f8", "float": "<f4", "uchar": "u1"}[pr[1]]) for pr in (l.split() for l in hd.splitlines() if l.startswith("property"))])
    v = np.frombuffer(raw[end:end + n * dt.itemsize], dt); return np.stack([v["x"], v["y"], v["z"]], 1).astype(float)


def main():
    if a.data == "ycb":
        for f in sorted(a.dump.glob("*.json")):
            if a.scenes and f.stem not in a.scenes: continue
            D = json.loads(f.read_text()); sd = BM.YCBV / f.stem; ids, Twc, gobj, K = BM.gt_scene(sd, by_id); cam = json.loads((sd / "scene_camera.json").read_text())

            def load(k):
                bgr = cv2.imread(str(sd / "rgb" / f"{int(ids[k]):06d}.png")); raw = cv2.imread(str(sd / "depth" / f"{int(ids[k]):06d}.png"), cv2.IMREAD_UNCHANGED)
                if bgr is None or raw is None: return None
                return bgr, raw.astype(np.float32) * float(cam[ids[k]].get("depth_scale", 1.0)) / 1000.0, np.array(cam[ids[k]]["cam_K"]).reshape(3, 3), np.linalg.inv(Twc[k])
            ms = [m for m in ("FM-Fusion", "DualMap") if m in D["methods"]]
            new = relabel(f.stem, {g["name"] for g in D["gt"]}, len(ids), BM.FPS, load, {m: [np.asarray(o["pts"], float) for o in D["methods"][m]] for m in ms}); ch = collections.Counter()
            for m in ms:
                for o, (nm, v, s) in zip(D["methods"][m], new[m]):
                    ch[(m, kind(o["name"], nm))] += 1; o["old_name"] = o["name"]; o["name"] = nm; o["kind"] = "unnamed" if nm is None else "match"; o["cad_votes"] = v; o["frames_seen"] = s
            (a.out / f.name).write_text(json.dumps(D, ensure_ascii=False))
            print(f.stem, dict(ch), [(x["method"], x["processed"], x["offered"], round(x["ms_mean"] or 0)) for x in STATS[-len(ms):]], flush=True)
            (a.out / "stream_stats.json").write_text(json.dumps(STATS, ensure_ascii=False))
    else:
        tab = table if a.data == "lab" else json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]; by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["ycb"] for o in tab}
        scenes = a.scenes or sorted(p.stem for p in (OBM / "dualmap").glob("*.json") if not p.stem.startswith("bigroom"))
        FMD = (a.fmf_dir or OBO / "fmfusion").resolve(); DMD = (a.dualmap_maps or OBM / "dualmap").resolve()
        fn = a.out / "fmf_names.json"; names_out = json.loads(fn.read_text()) if fn.exists() else {}; (a.out / "dualmap").mkdir(parents=True, exist_ok=True)
        for sc in scenes:
            cam = json.loads((OB / "test" / sc / "scene_camera.json").read_text()); ks = sorted(cam, key=int); PW = {}
            if a.poses:                                   # SLAM 자세(카메라→월드). 자세가 없는 프레임은 건너뛴다
                for r_ in np.loadtxt(a.poses / f"{sc}_frames.txt", ndmin=2):
                    T_ = np.eye(4); T_[:3, :3] = BM.quat_to_rot(r_[4:8]); T_[:3, 3] = r_[1:4]; PW[int(round(r_[0] * a.fps))] = np.linalg.inv(T_)

            def load(i):
                k = ks[i]; bgr = cv2.imread(str(OB / "test" / sc / "rgb" / f"{int(k):06d}.png")); raw = cv2.imread(str(OB / "test" / sc / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED)
                if bgr is None or raw is None: return None
                if a.poses:
                    if i not in PW: return None
                    T = PW[i]
                else:
                    T = np.eye(4); T[:3, :3] = np.array(cam[k]["cam_R_w2c"]).reshape(3, 3); T[:3, 3] = np.array(cam[k]["cam_t_w2c"]) / 1000.0
                return bgr, raw.astype(np.float32) * float(cam[k].get("depth_scale", 1.0)) / 1000.0, np.array(cam[k]["cam_K"]).reshape(3, 3), T
            fm = []
            for line in (FMD / sc / "instance_info.txt").read_text().splitlines():
                if line.startswith("#") or not line.strip(): continue
                f = line.split(";"); Q = read_ply_xyz(FMD / sc / f"{int(f[0]):04d}.ply")
                if len(Q): fm.append((str(int(f[0])), by_label.get(f[1].split("(")[0]), Q[:: max(1, len(Q) // 10000)]))
            J = json.loads((DMD / f"{sc}.json").read_text())
            new = relabel(sc, set(name_of.values()), len(ks), a.fps, load, {"FM-Fusion": [x[2] for x in fm], "DualMap": [np.asarray(o.get("pts", []), float).reshape(-1, 3) for o in J]}); ch = collections.Counter()
            names_out[sc] = {}
            for (i, old, _), (nm, v, s) in zip(fm, new["FM-Fusion"]):
                names_out[sc][i] = ycb_of[nm] if nm else None; ch[("FM-Fusion", kind(old, names_out[sc][i]))] += 1
            for o, (nm, v, s) in zip(J, new["DualMap"]):
                old = o["name"]; o["old_name"] = old; o["name"] = ycb_of[nm] if nm else None; o["cad_votes"] = v; o["frames_seen"] = s; ch[("DualMap", kind(old, o["name"]))] += 1
            (a.out / "dualmap" / f"{sc}.json").write_text(json.dumps(J)); fn.write_text(json.dumps(names_out))
            print(sc, dict(ch), [(x["method"], x["processed"], x["offered"], round(x["ms_mean"] or 0)) for x in STATS[-2:]], flush=True)
            (a.out / "stream_stats.json").write_text(json.dumps(STATS, ensure_ascii=False))


if __name__ == "__main__":
    main()
