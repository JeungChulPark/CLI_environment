#!/usr/bin/env python3
"""six_rt.py — ObChange 방문 영상에 여섯 구성(objpose/tools/lab/live_six.py, 실제 환경 실험과 같은 인식기)을 실시간 재생으로 돌린다. 물체는 ObChange 의 YCB 11종 전부.

영상은 10 fps 로 벽시계에 맞춰 도착하고, 앞 프레임의 처리가 끝난 순간 도착해 있는 최신 프레임 하나만 처리한다. 인식에는 카메라 자세를 쓰지 않는다.
--online-map 을 주면 객체 지도도 실행 중에 만든다: 처리한 프레임마다 지금 방식의 지도(ObjectMemory, proposed_maps.py 와 같은 설정, 카메라 자세 = 데이터셋 제공값)에 넣고,
  별도 스레드가 설계 조합 11가지(objpose/pc/online_designs.py)를 그때까지의 자료로 갱신한다. 끝나면 <out>/maps/proposed/<방문>.json, <out>/../designs/c<구성>_<방문>.json(갱신 시간 통계 포함), <out>/<방문>/online_timeline.jsonl 을 쓴다.
  <out>/<방문>/sam6d_frames.jsonl        지도 구성기(proposed_maps.py)가 읽는 형식
              /sam6d/detections.jsonl    검증을 통과한 자세마다 한 줄(검증 점수 포함)      ┐ 지도 설계 도구(map_designs.py)가 읽는 형식
              /sam6d/frames.jsonl        처리한 프레임마다 한 줄(후보와 탈락 이유 포함)    ┘ (hub 실행 기록과 같음)
              /run.json
    LIVE_SIX=<1~6> <sam6d python> objpose/tools/obchange/six_rt.py --out <dir> --scenes smallroom_s2 …
"""
import argparse, json, os, sys, time
from pathlib import Path
REPO = Path(__file__).resolve().parents[3]; SR = REPO / "sam6d_realtime"
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--scenes", nargs="+", required=True); ap.add_argument("--fps", type=float, default=10.0); ap.add_argument("--max-frames", type=int, default=0); ap.add_argument("--online-map", action="store_true"); ap.add_argument("--save-pred", action="store_true", help="처리한 프레임마다 통과한 물체의 마스크와 이름을 <out>/<방문>/pred/ 에 FM-Fusion 의 prediction 형식으로 남긴다(다른 지도 생성 프로그램의 입력)")
a = ap.parse_args(); a.out = a.out.resolve(); a.out.mkdir(parents=True, exist_ok=True)
TAB = json.loads((REPO / "objpose/rt/bop/obchange_objects.json").read_text()); ROOT = Path(TAB["scenes_dir"]); NAMES = [o["name"] for o in TAB["objects"]]; WORDS = {o["name"]: o["ycb"].split("_", 1)[1].replace("_", " ") for o in TAB["objects"]}; TAGS = ". ".join(WORDS.values())     # FM-Fusion 은 프레임의 문구 목록(tags)이 비어 있으면 죽는다 → 찾는 물체 이름 전부를 넣는다(GroundingDINO 에 문구 전부를 주던 것과 같다)
sys.path.insert(0, str(REPO / "objpose/tools/lab"))
import cv2, numpy as np, torch     # noqa: E402
import live_six as LS              # noqa: E402  (LIVE_SIX 환경변수로 구성을 고른다)
VERIFY = {"enabled": True, "mask_iou_min": 0.420998, "texture_min_score": 0.449562, "cluster_rotation_deg": 20, "cluster_translation_mm": 25, "cluster_min_occupancy": 0.5}
core = LS.LiveCore({"ism": {"config": "configs/yolo_ism_ycbv.yaml", "objects": NAMES}, "runtime": {"device": "cuda:0", "det_score_thresh": 0.2, "verify": dict(VERIFY)}})
assert sorted(o["name"] for o in core.objs) == sorted(NAMES), [o["name"] for o in core.objs]
T0_NS = 1_600_000_000_000_000_000; STEP_NS = int(round(1e9 / a.fps))
if a.online_map:
    import threading
    os.environ.update(BENCH_ROOT=str(ROOT.parent), BENCH_TABLE=str(REPO / "objpose/rt/bop/obchange_objects.json"), BENCH_FPS=str(a.fps))
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
        for q in np.loadtxt(PM.BM.YCBV.parent / "poses_tf" / f"{s}_frames.txt", ndmin=2):
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
