#!/usr/bin/env python3
"""lab_rt.py — 직접 녹화한 세션(extract_lab_bop.py 가 꺼낸 프레임)에 자세 추정 구성 일곱 가지를 실시간 재생으로 돌린다. ObChange 실험(combo_rt.py)의 실험실 물체 판.

구성 (조합 보고서의 A~E 와 같은 뜻. 물체는 실험실 8종, 인식기 설정은 sam6d_realtime/configs/yolo_ism_objects.yaml = 실시간 뷰어에서 쓰는 설정)
  A      원본 SAM-6D: 화면 전체 분할(FastSAM-x) + ViT-L + 원본 점수 0.2 + 원본 PEM, 검증 없음
  B      화면 전체 분할 + ViT-S + 원본 점수 0.5 + 자세 검증
  C      문구 상자(YOLO-World, 물체마다 상위 3개) + MobileSAM + ViT-S + 원본 점수 0.5 + 자세 검증      C_off = 검증 끔
  D      제안 방법, 완화 설정: 관문 기준 낮춤(의미 0.25, 색상 0.06) + 배정 대안
  E      제안 방법, 배포 설정(실시간 뷰어와 같음)                                                    E_off = 자세 검증 끔
A~C 의 원본 ISM/PEM 은 논문 저장소의 run_orig.py · realdata/live_cores.py 를 그대로 쓴다. B~E 의 자세 추정과 검증은 이 저장소의 Sam6DCore.
영상은 --fps 로 벽시계에 맞춰 도착하고, 앞 프레임의 추론이 끝난 순간 도착해 있는 최신 프레임 하나만 처리한다(첫 두 번의 준비 호출은 시간에 넣지 않음).
  <out>/<세션>/sam6d_frames.jsonl, run.json   (run_compare.py 와 같은 형식)
    <sam6d python> objpose/tools/lab/lab_rt.py --config E --out <dir> [--scenes …]
"""
import argparse, json, os, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]; SR = REPO / "sam6d_realtime"; PAPER = Path("/home/jucpark/DeepLearning/CLI_env_paper/paper_etrij_261003/src")
ap = argparse.ArgumentParser()
ap.add_argument("--config", required=True, choices=["A", "B", "C", "C_off", "D", "E", "E_off"]); ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--root", type=Path, default=Path.home() / "DeepLearning/Datasets/lab_sessions_bop/test"); ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--fps", type=float, default=10.0); ap.add_argument("--max-frames", type=int, default=0)
a = ap.parse_args(); a.out = a.out.resolve(); a.out.mkdir(parents=True, exist_ok=True); scenes = a.scenes or sorted(p.name for p in a.root.iterdir() if (p / "scene_camera.json").exists())
os.environ.update(SAM6D_REALTIME=str(SR), DINOV2_VITL_DIR=str(REPO / "objpose/tools/baselines/ckpt_dinov2"), YCBV_WORK="/home/jucpark/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper",
                  PREDEF_POSES="/home/jucpark/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper/predefined_poses",
                  FASTSAM_X="/home/jucpark/DeepLearning/CLI_environment_server/sam6d_ws/sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt")
VERIFY = {"enabled": True, "mask_iou_min": 0.420998, "texture_min_score": 0.449562, "cluster_rotation_deg": 20, "cluster_translation_mm": 25, "cluster_min_occupancy": 0.5}
CFG = {"ism": {"config": str(SR / "configs/yolo_ism_objects.yaml"), "objects": []}, "runtime": {"device": "cuda:0", "det_score_thresh": 0.2, "verify": None if a.config.endswith("_off") else dict(VERIFY)}}
import cv2, numpy as np, torch      # noqa: E402
sys.path.insert(0, str(PAPER / "ycbv")); sys.path.insert(0, str(PAPER / "ycbv/ladder")); sys.path.insert(0, str(PAPER / "realdata")); sys.path.insert(0, str(SR / "realtime")); sys.path.insert(0, str(SR))
import sam6d_core                   # noqa: E402
_Core = sam6d_core.Sam6DCore


class FastCore(_Core):               # B~E 공통: 실시간 뷰어의 속도 설정 중 PEM 점 수(1024). 마스크 재사용은 배포 경로(D, E)에만 켠다
    def __init__(self, *args, **kw):
        kw.setdefault("fine_npoint_override", 1024)
        if a.config in ("D", "E", "E_off"): kw.setdefault("mask_cache", {"enabled": True, "iou_thresh": 0.90, "max_age": 5, "interval": 30})
        super().__init__(*args, **kw)


sam6d_core.Sam6DCore = FastCore
if a.config == "A":
    os.environ.update(OBJPOSE_LIVE_MODE="orig", OBJPOSE_ORIG_DESC="dinov2_vitl14", OBJPOSE_ORIG_THRESH="0.2")
    import builtins, runpy          # run_orig 의 후보 1개 버그 우회(out_5090/run_fix1.py 와 같은 패치)를 import 때 건다
    _imp = builtins.__import__

    def imp(name, *aa, **kk):
        m = _imp(name, *aa, **kk)
        if name == "run_orig" and not getattr(m, "_fix1", False):
            m._fix1 = True; C = m.CustomDINOv2

            def pmp(self, masks, boxes):
                masks.unsqueeze_(1); return self.rgb_proposal_processor(masks, boxes).squeeze_(1)
            C.process_masks_proposals = pmp; I = m.Instance_Segmentation_Model; _cq = I.Calculate_the_query_translation

            def cq(self, proposal, *x, **k):
                if len(proposal) == 1:
                    out = _cq(self, torch.cat([proposal, proposal.clone()], 0), *x, **k)[:1]
                    if proposal.dim() == 4: proposal.squeeze_(1)
                    return out
                return _cq(self, proposal, *x, **k)
            I.Calculate_the_query_translation = cq
        return m
    builtins.__import__ = imp
    import live_cores as LC         # noqa: E402
    core = LC.LiveCore(CFG)
elif a.config in ("B", "C", "C_off"):
    os.environ.update(OBJPOSE_LIVE_MODE="hybrid", OBJPOSE_ORIG_DESC="dinov2_vits14", OBJPOSE_ORIG_THRESH="0.5")
    import builtins
    _imp = builtins.__import__

    def imp(name, *aa, **kk):
        m = _imp(name, *aa, **kk)
        if name == "run_orig" and not getattr(m, "_fix1", False):
            m._fix1 = True; C = m.CustomDINOv2

            def pmp(self, masks, boxes):
                masks.unsqueeze_(1); return self.rgb_proposal_processor(masks, boxes).squeeze_(1)
            C.process_masks_proposals = pmp; I = m.Instance_Segmentation_Model; _cq = I.Calculate_the_query_translation

            def cq(self, proposal, *x, **k):
                if len(proposal) == 1:
                    out = _cq(self, torch.cat([proposal, proposal.clone()], 0), *x, **k)[:1]
                    if proposal.dim() == 4: proposal.squeeze_(1)
                    return out
                return _cq(self, proposal, *x, **k)
            I.Calculate_the_query_translation = cq
        return m
    builtins.__import__ = imp
    import live_cores as LC         # noqa: E402
    core = LC.LiveCore(CFG)
    if a.config != "B":             # C: 마스크를 화면 전체 분할 대신 문구 상자(물체마다 점수 상위 3개) + MobileSAM 으로 만든다 (run_ours.py --orig-seg text_msam 과 같은 방식)
        import yolo_ism as yi
        from ultralytics import YOLO
        inner = core.core; msam = yi.build_segmentor(str(SR / "mobile_sam.pt"), "cuda:0"); prompts = list(inner.unique_prompts); dev = LC.RO.DEV

        def box_masks(rgb):
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR); r = YOLO.predict(inner.yolo, bgr, conf=0.02, verbose=False)[0]; top = {}
            if r.boxes is not None:
                for b, c, s in sorted(zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()), key=lambda x: -x[2]):
                    if len(top.setdefault(int(c), [])) < 3: top[int(c)].append(b)
            boxes = [b for bs in top.values() for b in bs]
            if not boxes: return None
            ms = yi.segment_boxes(msam, bgr, boxes, "cuda:0"); keep = [j for j, m in enumerate(ms) if m is not None and m.any()]
            if not keep: return None
            return {"masks": torch.from_numpy(np.stack([ms[j] for j in keep])).float().to(dev), "boxes": torch.tensor([boxes[j] for j in keep], dtype=torch.float32, device=dev)}
        core.ism.segmentor_model.generate_masks = box_masks
else:
    import verify_config as VC      # noqa: E402
    cwd = os.getcwd(); os.chdir(SR)
    core = sam6d_core.Sam6DCore(CFG["ism"]["config"], [], "cuda:0", 0.2, verify=CFG["runtime"]["verify"])
    os.chdir(cwd)
    if a.config == "D":
        for o in core.objs: o["similarity_threshold"] = 0.25; o["appe_v2_gate"] = 0.605; o["appe_gate"] = 0.605; o["hsv_gate_threshold"] = 0.06; o["hsv_gate_enabled"] = True; o["assignment_fallback"] = True
names = [o["name"] for o in core.objs]; assert "Sauce_high" not in names, names
print(f"[lab_rt] config {a.config}: {len(names)} objects {names}", flush=True)
T0_NS = 1_600_000_000_000_000_000; STEP_NS = int(round(1e9 / a.fps))


def sync():
    if torch.cuda.is_available(): torch.cuda.synchronize()


for s in scenes:
    d = a.root / s; cam = json.loads((d / "scene_camera.json").read_text()); keys = sorted(cam, key=int); n = len(keys) if not a.max_frames else min(a.max_frames, len(keys)); out = a.out / s; out.mkdir(exist_ok=True)

    def load(k):
        c = cam[keys[k]]; bgr = cv2.imread(str(d / "rgb" / f"{int(keys[k]):06d}.png")); raw = cv2.imread(str(d / "depth" / f"{int(keys[k]):06d}.png"), cv2.IMREAD_UNCHANGED)
        return bgr, np.rint(raw.astype(np.float32) * float(c["depth_scale"])).astype(np.uint16), np.asarray(c["cam_K"], np.float64).reshape(3, 3)
    bgr, dep, K = load(0)
    for _ in range(2): core.process(bgr, dep, K, want_mask=False)
    mc = getattr(core, "mask_cache", None)
    if mc is not None: mc.reset()
    rows = []; t0 = time.perf_counter(); last = -1
    while True:
        now = time.perf_counter() - t0; k = int(now * a.fps)
        if k >= n: break
        if k <= last: time.sleep(max((last + 1) / a.fps - now, 0.0005)); continue
        last = k; bgr, dep, K = load(k); sync(); r, ms, nb, _ = core.process(bgr, dep, K, want_mask=False); sync(); done = time.perf_counter() - t0
        rows.append({"t_ns": T0_NS + k * STEP_NS, "idx": k, "im_id": int(keys[k]), "arrive_s": round(k / a.fps, 4), "start_s": round(now, 4), "done_s": round(done, 4), "latency_s": round(done - k / a.fps, 4), "ms": ms, "n_boxes": int(nb or 0),
                     "dets": [{"object": x["object"], "score": float(x["score"]), "R": x["R"], "t_mm": x["t_mm"]} for x in r]})
    (out / "sam6d_frames.jsonl").write_text("\n".join(json.dumps(x) for x in rows) + "\n")
    el = rows[-1]["done_s"] if rows else 0.0
    (out / "run.json").write_text(json.dumps({"scene": s, "mode": "realtime", "frames": n, "processed": len(rows), "detections": sum(len(x["dets"]) for x in rows), "elapsed_s": round(el, 1), "hz": round(len(rows) / max(el, 1e-6), 3),
                                              "ms_median": float(np.median([1000 * (x["done_s"] - x["start_s"]) for x in rows])) if rows else None, "config": {"name": a.config, "objects": names}}))
    print(f"[lab_rt] {s}: {len(rows)}/{n} frames, {sum(len(x['dets']) for x in rows)} dets, {el:.0f}s", flush=True)
print("[lab_rt] done", flush=True)
