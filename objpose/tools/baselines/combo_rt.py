#!/usr/bin/env python3
"""combo_rt.py — 조합 실험(objpose/docs/ycb_combo_261008)의 한 장 평가 스크립트(run_orig.py, run_ours.py)를 고치지 않고 ObChange 영상에 실시간 재생으로 돌린다.

스크립트는 `data.load_targets()` 가 준 사진 목록을 차례로 돌며 `data.load_image()` 로 사진을 읽는다. 이 두 함수만 바꿔 끼운다.
  사진 목록   영상이 --fps 로 벽시계에 맞춰 도착한다고 보고, 앞 프레임의 추론이 끝난 순간 이미 도착한 프레임 가운데 가장 최신 것 하나를 준다(그 사이 프레임은 버림).
              찾을 물체는 ObChange 의 YCB 11종 전부(물체를 모르는 조건).
  사진 읽기   ObChange BOP 폴더(<scenes_dir>/<방문>/rgb, depth, scene_camera.json)
upstream 의 후보 1개 버그 우회(out_5090/run_fix1.py)는 그대로 거친다. 결과는 run_compare.py 와 같은 형식으로 쓴다:
  <out>/<방문>/sam6d_frames.jsonl (처리한 프레임마다 도착 · 시작 · 완료 시각과 자세), run.json, 원래 스크립트의 <out>/pred.json
    <sam6d python> objpose/tools/baselines/combo_rt.py --out <dir> --scenes office_s2 … -- run_ours.py --mode text --no-verify
"""
import argparse, json, os, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
Y = Path("/home/jucpark/DeepLearning/CLI_env_paper/paper_etrij_261003/src/ycbv"); W = "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper"
i = sys.argv.index("--"); ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True); ap.add_argument("--scenes", nargs="+", required=True); ap.add_argument("--fps", type=float, default=10.0); ap.add_argument("--max-frames", type=int, default=0)
a = ap.parse_args(sys.argv[1:i]); cmd = sys.argv[i + 1:]; a.out = a.out.resolve(); a.out.mkdir(parents=True, exist_ok=True)
TAB = json.loads((REPO / "objpose/rt/bop/obchange_objects.json").read_text()); ROOT = Path(TAB["scenes_dir"]); NAME = {o["obj_id"]: o["name"] for o in TAB["objects"]}; OIDS = sorted(NAME)
T0_NS = 1_600_000_000_000_000_000; STEP_NS = int(round(1e9 / a.fps))
os.environ.update(YCBV_DIR="/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video", YCBV_WORK=W, PREDEF_POSES=W + "/predefined_poses", YCBV_OUT=str(a.out), YCBV_RUN_LABEL="ObChange realtime, RTX 5090 Laptop",
                  DINOV2_VITL_DIR=str(REPO / "objpose/tools/baselines/ckpt_dinov2"),
                  FASTSAM_X="/home/jucpark/DeepLearning/CLI_environment_server/sam6d_ws/sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt")
os.chdir(Y); sys.path.insert(0, str(Y)); sys.path.insert(0, str(Y / "ladder"))
import cv2, numpy as np      # noqa: E402
import data as D             # noqa: E402

CAM = {s: json.loads((ROOT / s / "scene_camera.json").read_text()) for s in a.scenes}; KEYS = {s: sorted(CAM[s], key=int) for s in a.scenes}
LOG = []          # 처리한 프레임: [방문 번호, 프레임 번호(0부터), 도착, 시작, 완료]


class Items:
    """사진 목록 자리에 들어가는 실시간 재생기."""
    def __len__(self): return sum(len(KEYS[s]) for s in a.scenes)
    def __getitem__(self, sl):                      # items[:warmup] — 첫 방문의 앞 프레임. 시간에 넣지 않는다
        return [(0, k, OIDS) for k in range(len(KEYS[a.scenes[0]]))][sl]
    def __iter__(self):
        for si, s in enumerate(a.scenes):
            n = len(KEYS[s]) if not a.max_frames else min(a.max_frames, len(KEYS[s])); t0 = time.perf_counter(); last = -1
            while True:
                now = time.perf_counter() - t0; k = int(now * a.fps)
                if k >= n: break
                if k <= last: time.sleep(max((last + 1) / a.fps - now, 0.0005)); continue
                last = k; row = [si, k, k / a.fps, now, None]; LOG.append(row)
                yield (si, k, OIDS)
                row[4] = time.perf_counter() - t0
            print(f"[rt] {s}: {sum(1 for r in LOG if r[0] == si)}/{n} frames, {time.perf_counter() - t0:.0f}s", flush=True)


def load_image(si, k):
    s = a.scenes[si]; key = KEYS[s][k]; d = ROOT / s; c = CAM[s][key]
    bgr = cv2.imread(str(d / "rgb" / f"{int(key):06d}.png"), cv2.IMREAD_COLOR); raw = cv2.imread(str(d / "depth" / f"{int(key):06d}.png"), cv2.IMREAD_UNCHANGED)
    ds = float(c["depth_scale"]); return bgr, raw, raw.astype(np.float32) * ds, np.asarray(c["cam_K"], np.float64).reshape(3, 3), ds


D.load_targets = lambda: Items(); D.load_image = load_image
import runpy                  # noqa: E402
pred = a.out / "pred.json"; sys.argv = [str(Y / "out_5090/run_fix1.py"), cmd[0]] + cmd[1:] + ["--out", str(pred)]
t_all = time.time(); runpy.run_path(str(Y / "out_5090/run_fix1.py"), run_name="__main__")

P = json.loads(pred.read_text()); imgs = P["images"]; assert len(imgs) == len(LOG), (len(imgs), len(LOG))
per = {}
for (si, k, arr, st, dn), im in zip(LOG, imgs):
    assert (im["scene_id"], im["im_id"]) == (si, k)
    dets = [{"object": NAME[int(p["obj_id"])], "score": float(p["score"]), "R": p["R"], "t_mm": p["t_mm"]} for p in im["preds"] if int(p["obj_id"]) in NAME]
    per.setdefault(si, []).append({"t_ns": T0_NS + k * STEP_NS, "idx": k, "im_id": int(KEYS[a.scenes[si]][k]), "arrive_s": round(arr, 4), "start_s": round(st, 4), "done_s": round(dn if dn is not None else st, 4),
                                   "latency_s": round((dn if dn is not None else st) - arr, 4), "ms": im.get("stage_ms"), "dets": dets})
for si, rows in per.items():
    d = a.out / a.scenes[si]; d.mkdir(exist_ok=True); (d / "sam6d_frames.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    n = len(KEYS[a.scenes[si]]); el = rows[-1]["done_s"]
    (d / "run.json").write_text(json.dumps({"scene": a.scenes[si], "mode": "realtime", "frames": n, "processed": len(rows), "detections": sum(len(r["dets"]) for r in rows), "elapsed_s": round(el, 1),
                                            "hz": round(len(rows) / max(el, 1e-6), 3), "ms_median": float(np.median([1000 * (r["done_s"] - r["start_s"]) for r in rows])), "config": {"script": cmd, "objects": [NAME[o] for o in OIDS]}}))
print(f"[rt] done {len(per)} scenes, {len(LOG)} frames, {time.time() - t_all:.0f}s", flush=True)
