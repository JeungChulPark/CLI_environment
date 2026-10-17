#!/usr/bin/env python3
"""rt_bop.py — six_rt.py 의 일반형: BOP 형식 데이터(--table 의 물체표: YCB-Video, YCB-M, ObChange)에 인식(live_six.py, LIVE_SIX)을 실시간 재생으로 돌리고 객체 지도를 실행 중에 만든다.

six_rt.py 와 다른 점: 물체표 · 재생 속도를 --table 에서 읽고(찾을 물체 = 표의 전부), 카메라 자세는 데이터셋의 정답 자세(scene_camera.json 의 cam_R_w2c)를 쓴다(--poses <폴더> 를 주면 <장면>_frames.txt).
출력 형식은 six_rt.py 와 같다: <out>/<장면>/{sam6d_frames.jsonl, sam6d/, pred/, run.json, online_timeline.jsonl}, <out>/maps/proposed/<장면>.json, <out>/../designs/c<구성>_<장면>.json
    LIVE_SIX=8 <sam6d python> objpose/tools/mapgen/rt_bop.py --table objpose/rt/bop/ycbv_objects.json --out <dir> --scenes 000048 … --online-map --save-pred
"""
import argparse, json, os, sys, time
from pathlib import Path
REPO = Path(__file__).resolve().parents[3]; SR = REPO / "sam6d_realtime"
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--scenes", nargs="+", required=True); ap.add_argument("--table", type=Path, required=True); ap.add_argument("--poses", default="gt"); ap.add_argument("--ism-config", default="configs/yolo_ism_ycbv.yaml"); ap.add_argument("--max-frames", type=int, default=0); ap.add_argument("--online-map", action="store_true"); ap.add_argument("--save-pred", action="store_true", help="처리한 프레임마다 통과한 물체의 마스크와 이름을 <out>/<방문>/pred/ 에 FM-Fusion 의 prediction 형식으로 남긴다(다른 지도 생성 프로그램의 입력)")
a = ap.parse_args(); a.out = a.out.resolve(); a.out.mkdir(parents=True, exist_ok=True)
TAB = json.loads(a.table.read_text()); a.fps = float(TAB["fps"]); ROOT = Path(TAB["scenes_dir"]); NAMES = [o["name"] for o in TAB["objects"]]; WORDS = {o["name"]: o["ycb"].split("_", 1)[1].replace("_", " ") for o in TAB["objects"]}; TAGS = ". ".join(WORDS.values())     # FM-Fusion 은 프레임의 문구 목록(tags)이 비어 있으면 죽는다 → 찾는 물체 이름 전부를 넣는다(GroundingDINO 에 문구 전부를 주던 것과 같다)
sys.path.insert(0, str(REPO / "objpose/tools/lab"))
import cv2, numpy as np, torch     # noqa: E402
import live_six as LS              # noqa: E402  (LIVE_SIX 환경변수로 구성을 고른다)
VERIFY = {"enabled": True, "mask_iou_min": 0.420998, "texture_min_score": 0.449562, "cluster_rotation_deg": 20, "cluster_translation_mm": 25, "cluster_min_occupancy": 0.5}
core = LS.LiveCore({"ism": {"config": a.ism_config, "objects": NAMES}, "runtime": {"device": "cuda:0", "det_score_thresh": 0.2, "verify": dict(VERIFY)}})
assert sorted(o["name"] for o in core.objs) == sorted(NAMES), [o["name"] for o in core.objs]
T0_NS = 1_600_000_000_000_000_000; STEP_NS = int(round(1e9 / a.fps))
if a.online_map:
    import threading
    os.environ.update(BENCH_ROOT=str(ROOT.parent), BENCH_TABLE=str(a.table.resolve()), BENCH_FPS=str(a.fps))
    for p in ("objpose/pc", "objpose/tools/obchange"): sys.path.insert(0, str(REPO / p))
    import proposed_maps as PM; from online_designs import OnlineDesigns      # noqa: E402,E702
    YCB_OF = {o["name"]: o["ycb"] for o in TAB["objects"]}; (a.out / "maps/proposed").mkdir(parents=True, exist_ok=True); (a.out.parent / "designs").mkdir(exist_ok=True)


def sync():
    if torch.cuda.is_available(): torch.cuda.synchronize()


def scalar(v): return isinstance(v, (int, float, str, bool)) or v is None


for s in a.scenes:
    d = ROOT / s; cam = json.loads((d / "scene_camera.json").read_text()); keys = sorted(cam, key=int); n = len(keys) if not a.max_frames else min(a.max_frames, len(keys)); out = a.out / s; (out / "sam6d").mkdir(parents=True, exist_ok=True)

    def load(k):
        c = cam[keys[k]]; bgr = cv2.imread(str(d / "rgb" / f"{int(keys[k]):06d}.png")); raw = cv2.imread(str(d / "depth" / f"{int(keys[k]):06d}.png"), cv2.IMREAD_UNCHANGED)
        return bgr, np.rint(raw.astype(np.float32) * float(c["depth_scale"])).astype(np.uint16), np.asarray(c["cam_K"], np.float64).reshape(3, 3)
    bgr, dep, K = load(0)
    for _ in range(2): core.process(bgr, dep, K, want_mask=False)
    if a.online_map:
        mem, poses = PM.new_memory(NAMES, K); od = OnlineDesigns(mem, K, (bgr.shape[1], bgr.shape[0])); stop = threading.Event(); th = threading.Thread(target=od.run, args=(stop,), daemon=True); th.start(); cam_T = {}; fed = 0
        if a.poses == "gt":
            for i, kk in enumerate(keys):
                Rw = np.array(cam[kk]["cam_R_w2c"]).reshape(3, 3); T = np.eye(4); T[:3, :3] = Rw.T; T[:3, 3] = -Rw.T @ (np.array(cam[kk]["cam_t_w2c"]) / 1000.0); cam_T[i] = T
        else:
            for q in np.loadtxt(Path(a.poses) / f"{s}_frames.txt", ndmin=2):
                T = np.eye(4); T[:3, :3] = PM.BM.quat_to_rot(q[4:8]); T[:3, 3] = q[1:4]; cam_T[int(round(q[0] * a.fps))] = T
    rows, dets, frs = [], [], []; t0 = time.perf_counter(); last = -1
    while True:
        now = time.perf_counter() - t0; k = int(now * a.fps)
        if k >= n: break
        if k <= last: time.sleep(max((last + 1) / a.fps - now, 0.0005)); continue
        last = k; bgr, dep, K = load(k); sync(); r, ms, nb, lab = core.process(bgr, dep, K, want_mask=a.save_pred); sync(); done = time.perf_counter() - t0; st = T0_NS + k * STEP_NS
        rows.append({"t_ns": st, "idx": k, "im_id": int(keys[k]), "arrive_s": round(k / a.fps, 4), "start_s": round(now, 4), "done_s": round(done, 4), "ms": ms, "dets": [{"object": x["object"], "score": float(x["score"]), "R": x["R"], "t_mm": x["t_mm"]} for x in r]})
        for x in r: dets.append({**{kk: vv for kk, vv in x.items() if scalar(vv) or kk in ("R", "t_mm", "bbox")}, "stamp_ns": st, "frame_seq": len(rows) - 1})
        diag = core.last_frame_diag
        if a.save_pred and r and lab is not None:    # frame-<프레임 번호>_mask.png (값 i = i 번째 물체), _label.json. 디스크 쓰기는 처리 시간에 들어간다
            (out / "pred").mkdir(exist_ok=True); cv2.imwrite(str(out / "pred" / f"frame-{k:06d}_mask.png"), lab.astype(np.uint8))
            (out / "pred" / f"frame-{k:06d}_label.json").write_text(json.dumps({"raw_tags": TAGS, "tags": TAGS, "mask": [{"value": 0, "label": "background"}] + [{"value": i + 1, "labels": {WORDS[x["object"]]: float(x["score"])}, "box": [float(v) for v in x["bbox"]], "object": x["object"]} for i, x in enumerate(r)]}))
        if a.online_map:                              # 지도는 지금까지 도착한 프레임의 카메라 자세만 안다
            while fed <= k:
                if fed in cam_T: poses.add(T0_NS + fed * STEP_NS, "OK", cam_T[fed], map_id=0)
                fed += 1
            mem.add_frame(st, [(x["object"], float(x["score"]), x["R"], x["t_mm"]) for x in r], depth=dep[::8, ::8]); mem.update()
            od.add(st, r, [{"object": c["object"], "bbox": c.get("bbox")} for c in diag.get("pem_candidates", [])], dep); rows[-1]["map_s"] = round(time.perf_counter() - t0, 4)
        frs.append({"stamp_ns": st, "frame_seq": len(rows) - 1, "n_accept": len(r), "n_boxes": int(nb or 0), "ms": ms, "objects": [x["object"] for x in r],
                    "diagnostics": {"pem_candidates": [{kk: vv for kk, vv in c.items() if scalar(vv) or kk == "bbox"} for c in diag.get("pem_candidates", [])], "pem_error": diag.get("pem_error")}})
    (out / "sam6d_frames.jsonl").write_text("\n".join(json.dumps(x) for x in rows) + "\n"); (out / "sam6d/detections.jsonl").write_text("".join(json.dumps(x) + "\n" for x in dets)); (out / "sam6d/frames.jsonl").write_text("".join(json.dumps(x) + "\n" for x in frs))
    el = rows[-1]["done_s"] if rows else 0.0
    if a.online_map:
        stop.set(); th.join(); mem.update(); R = od.final(); R["scene"] = s; R["config"] = LS.MODE
        (a.out / "maps/proposed" / f"{s}.json").write_text(json.dumps(PM.snapshot(mem, YCB_OF), indent=1)); (a.out.parent / "designs" / f"c{LS.MODE}_{s}.json").write_text(json.dumps(R, ensure_ascii=False))
        (out / "online_timeline.jsonl").write_text("".join(json.dumps(x) + "\n" for x in od.timeline)); print(f"[six_rt] 지도 갱신 {R['online']['passes']}회, 한 번 {R['online']['pass_ms']}, 반영 지연 {R['online']['lag_ms']}", flush=True)
    (out / "run.json").write_text(json.dumps({"scene": s, "mode": "realtime", "config": LS.MODE, "frames": n, "processed": len(rows), "detections": len(dets), "elapsed_s": round(el, 1),
                                              "ms_median": float(np.median([f["ms"]["total"] for f in frs])) if frs else None, "ms_mean": float(np.mean([f["ms"]["total"] for f in frs])) if frs else None, "objects": NAMES}))
    print(f"[six_rt] c{LS.MODE} {s}: {len(rows)}/{n} frames, {len(dets)} dets, {el:.0f}s", flush=True)
print("[six_rt] done", flush=True)
