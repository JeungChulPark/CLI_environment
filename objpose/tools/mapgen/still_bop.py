#!/usr/bin/env python3
"""still_bop.py — 인식 구성(live_six.py, LIVE_SIX)의 6D 포즈 정확도를 정해진 프레임에서 잰다. 프레임마다 독립된 정지 영상으로 처리한다(시계 없음, 건너뛰는 프레임 없음).

rt_bop.py 와 같은 인식기 · 같은 검증 설정 · 같은 물체표(찾을 물체 = 표의 21종 전부)를 쓰고, 처리할 프레임만 --frames 의 목록으로 고정한다.
네 구성이 똑같은 프레임을 처리하므로 포즈 지표를 그대로 비교할 수 있다. 채점은 score_still.py.
    LIVE_SIX=8 <sam6d python> objpose/tools/mapgen/still_bop.py --table objpose/rt/bop/ycbv_objects.json --frames <장면별 im_id 목록.json> --out <dir>
출력: <out>/<장면>/sam6d_frames.jsonl (run_compare.py · rt_bop.py 와 같은 형식), <out>/<장면>/run.json
"""
import argparse, json, sys, time
from pathlib import Path
REPO = Path(__file__).resolve().parents[3]
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--table", type=Path, required=True); ap.add_argument("--frames", type=Path, required=True)
ap.add_argument("--ism-config", default="configs/yolo_ism_ycbv.yaml"); ap.add_argument("--scenes", nargs="*", default=None)
a = ap.parse_args(); a.out = a.out.resolve(); a.out.mkdir(parents=True, exist_ok=True)
TAB = json.loads(a.table.read_text()); ROOT = Path(TAB["scenes_dir"]); NAMES = [o["name"] for o in TAB["objects"]]; FR = json.loads(a.frames.read_text())
sys.path.insert(0, str(REPO / "objpose/tools/lab"))
import cv2, numpy as np, torch     # noqa: E402
import live_six as LS              # noqa: E402
VERIFY = {"enabled": True, "mask_iou_min": 0.420998, "texture_min_score": 0.449562, "cluster_rotation_deg": 20, "cluster_translation_mm": 25, "cluster_min_occupancy": 0.5}     # rt_bop.py 와 같다
core = LS.LiveCore({"ism": {"config": a.ism_config, "objects": NAMES}, "runtime": {"device": "cuda:0", "det_score_thresh": 0.2, "verify": dict(VERIFY)}})


def sync():
    if torch.cuda.is_available(): torch.cuda.synchronize()


for s in (a.scenes or sorted(FR)):
    out = a.out / s
    if (out / "run.json").exists(): continue                  # 끝난 장면은 건너뛴다(이어 돌리기)
    out.mkdir(parents=True, exist_ok=True); d = ROOT / s; cam = json.loads((d / "scene_camera.json").read_text())

    def load(i):
        c = cam[str(i)]; bgr = cv2.imread(str(d / "rgb" / f"{i:06d}.png")); raw = cv2.imread(str(d / "depth" / f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
        return bgr, np.rint(raw.astype(np.float32) * float(c["depth_scale"])).astype(np.uint16), np.asarray(c["cam_K"], np.float64).reshape(3, 3)
    ids = [int(i) for i in FR[s]]; bgr, dep, K = load(ids[0])
    for _ in range(2): core.process(bgr, dep, K, want_mask=False)
    rows = []; t0 = time.perf_counter()
    for k, i in enumerate(ids):
        bgr, dep, K = load(i); sync(); r, ms, nb, _ = core.process(bgr, dep, K, want_mask=False); sync()
        rows.append({"idx": k, "im_id": i, "ms": ms, "n_boxes": int(nb or 0), "dets": [{"object": x["object"], "score": float(x["score"]), "R": x["R"], "t_mm": x["t_mm"]} for x in r]})
    (out / "sam6d_frames.jsonl").write_text("".join(json.dumps(x) + "\n" for x in rows))
    (out / "run.json").write_text(json.dumps({"scene": s, "mode": "still", "config": LS.MODE, "frames": len(rows), "elapsed_s": round(time.perf_counter() - t0, 1),
                                              "ms_median": float(np.median([x["ms"]["total"] for x in rows])), "objects": NAMES, "verify": VERIFY}))
    print(f"[still] c{LS.MODE} {s}: {len(rows)} frames, {sum(len(x['dets']) for x in rows)} dets, {time.perf_counter() - t0:.0f}s", flush=True)
print("[still] done", flush=True)
