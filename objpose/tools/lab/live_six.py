#!/usr/bin/env python3
"""live_six.py — 실제 환경(노트북 SLAM + 이 PC 인식, objpose/pc/hub.py)에 끼워 넣는 인식기. 후보 생성 · 마스크 · 특징의 여섯 조합. 자세 추정과 자세 검증은 항상 켠다.

환경변수 LIVE_SIX (hub 를 띄우기 전에 OBJPOSE_RECOGNIZER=<이 파일> 과 함께 준다)
  1  화면 전체 분할(FastSAM-x)                         + ViT-L
  2  화면 전체 분할(FastSAM-x)                         + ViT-S
  3  YOLO-World 문구 상자 → 그 상자와 가장 잘 겹치는 FastSAM-x 마스크 + ViT-L
  4  같음                                               + ViT-S
  5  YOLO-World 문구 상자 → 상자 안을 MobileSAM 으로 분할 + ViT-L
  6  같음                                               + ViT-S
  7  제안 방법, 완화 설정: YOLO-World 문구 상자 + MobileSAM + ViT-S + 제안 관문(의미 기준 0.25 로 낮춤, 상자를 2 · 3위 물체에게도 넘기는 배정 대안)   — 조합 보고서의 D
  8  제안 방법, 배포 설정: 인식기 설정 파일 그대로(실시간 뷰어와 같은 제안 관문)                                                                — 조합 보고서의 E
     7 · 8 은 물체 판정이 원본 점수가 아니라 제안 관문이다. 자세 추정 · 자세 검증 · PEM 점 수는 1~6 과 같다. 색상 관문은 설정 파일에 켜져 있는 데이터(실험실 물체)에서만 쓰고, 7 은 그 기준을 0.06 으로 낮춘다.
  9  화면 전체 분할 + 제안 관문: FastSAM-x 가 낸 상자를 모든 물체의 후보로 넣고(YOLO-World 대신), 그 뒤는 8 과 같다(의미 · 외형 · 색상 관문, 상자 배정, 상자 안 마스크는 MobileSAM, ViT-S). 논문 저장소 live_cores.py 의 gateA
     2026-10-11 확정 구성과의 대응: ① 원본 = 2, ② 관문 비교 = 9, ③ 제안 방법 = 8, ④ 완화 = 7, ⑤ 총점 효과 = 6
공통: 물체 판정은 원본 SAM-6D 점수(의미 + 외형 + 기하, 물체마다 1위, 기준 0.5), 자세 추정(PEM, 점 1024개)과 자세 검증은 이 저장소의 Sam6DCore(실시간 뷰어와 같은 검증 기준).
      문구는 sam6d_realtime/configs/yolo_ism_objects.yaml, YOLO-World 상자는 점수 0.02 이상에서 문구마다 상위 3개.
원본 ISM 부분은 논문 저장소의 run_orig.py · realdata/live_cores.py(hybrid) 를 그대로 쓴다.
"""
import builtins, os, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]; SR = REPO / "sam6d_realtime"; PAPER = Path("/home/jucpark/DeepLearning/CLI_env_paper/paper_etrij_261003/src")
MODE = int(os.environ.get("LIVE_SIX", "6")); DESC = "dinov2_vitl14" if MODE in (1, 3, 5) else "dinov2_vits14"
os.environ.update(SAM6D_REALTIME=str(SR), DINOV2_VITL_DIR=str(REPO / "objpose/tools/baselines/ckpt_dinov2"), YCBV_WORK="/home/jucpark/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper",
                  PREDEF_POSES="/home/jucpark/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper/predefined_poses",
                  FASTSAM_X="/home/jucpark/DeepLearning/CLI_environment_server/sam6d_ws/sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/FastSAM/FastSAM-x.pt",
                  OBJPOSE_LIVE_MODE="gateA" if MODE == 9 else "hybrid", OBJPOSE_ORIG_DESC=DESC, OBJPOSE_ORIG_THRESH="0.5")
import cv2, numpy as np, torch      # noqa: E402
for p in (PAPER / "ycbv", PAPER / "ycbv/ladder", PAPER / "realdata", SR / "realtime", SR): sys.path.insert(0, str(p))
import sam6d_core                   # noqa: E402
_Core = sam6d_core.Sam6DCore


class _FastCore(_Core):              # 실시간 뷰어의 속도 설정 중 PEM 점 수(1024)만 쓴다. 마스크 재사용은 쓰지 않는다(여섯 구성이 같은 조건)
    def __init__(self, *args, **kw):
        kw.setdefault("fine_npoint_override", 1024); super().__init__(*args, **kw)


sam6d_core.Sam6DCore = _FastCore
_imp = builtins.__import__


def _hook(name, *aa, **kk):          # run_orig 의 "후보가 1개일 때" 버그 우회 (out_5090/run_fix1.py 와 같은 패치)
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


builtins.__import__ = _hook
import live_cores as LC             # noqa: E402


def _box_iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1])); inter = ix * iy
    return inter / max((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter, 1e-6)


def LiveCore(cfg):
    """sam6d_infer.py / six_rt.py 가 부르는 입구. 1~6 은 원본 판정(hybrid), 7 · 8 은 제안 관문(Sam6DCore 그대로)."""
    if MODE <= 6 or MODE == 9: return _Hybrid(cfg)
    import verify_config as VC
    ic = cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"); ic = ic if os.path.isabs(ic) else str(SR / ic); rt = cfg.get("runtime", {}); cwd = os.getcwd(); os.chdir(SR)
    core = sam6d_core.Sam6DCore(ic, cfg.get("ism", {}).get("objects", []), rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2), verify=rt.get("verify", VC.UNSET)); os.chdir(cwd)
    assert "Sauce_high" not in [o["name"] for o in core.objs]
    if MODE == 7:
        for o in core.objs:
            o["similarity_threshold"] = 0.25; o["relative_assignment_enabled"] = True; o["assignment_fallback"] = True      # 배정 대안은 상자 배정 경로 안에 있다
            if o.get("hsv_gate_enabled"): o["hsv_gate_threshold"] = 0.06
    print(f"[live_six] mode {MODE} (제안 관문{' 완화' if MODE == 7 else ''}) objects={len(core.objs)}", flush=True)
    return core


class _Hybrid(LC.LiveCore):
    def __init__(self, cfg):
        cfg = dict(cfg); cfg["ism"] = dict(cfg.get("ism", {})); cfg["ism"]["config"] = str(SR / cfg["ism"].get("config", "configs/yolo_ism_objects.yaml")) if not os.path.isabs(cfg["ism"].get("config", "")) else cfg["ism"]["config"]
        super().__init__(cfg)
        assert "Sauce_high" not in [o["name"] for o in self.objs]
        if 3 <= MODE <= 6:
            import yolo_ism as yi
            from ultralytics import YOLO
            inner = self.core; dev = LC.RO.DEV; full = self.ism.segmentor_model.generate_masks
            msam = yi.build_segmentor(str(SR / "mobile_sam.pt"), "cuda:0") if MODE >= 5 else None

            def text_boxes(bgr):
                r = YOLO.predict(inner.yolo, bgr, conf=0.02, verbose=False)[0]; top = {}
                if r.boxes is not None:
                    for b, c, s in sorted(zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist()), key=lambda x: -x[2]):
                        if len(top.setdefault(int(c), [])) < 3: top[int(c)].append(b)
                return [b for bs in top.values() for b in bs]

            def gen(rgb):
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR); boxes = text_boxes(bgr)
                if not boxes: return None
                if MODE >= 5:                      # 상자 안을 MobileSAM 으로
                    ms = yi.segment_boxes(msam, bgr, boxes, "cuda:0"); keep = [j for j, m in enumerate(ms) if m is not None and m.any()]
                    if not keep: return None
                    return {"masks": torch.from_numpy(np.stack([ms[j] for j in keep])).float().to(dev), "boxes": torch.tensor([boxes[j] for j in keep], dtype=torch.float32, device=dev)}
                props = full(rgb)                  # 상자마다 가장 잘 겹치는 FastSAM-x 마스크 하나(겹침 0.5 이상)
                if props is None or len(props["masks"]) == 0: return None
                fb = props["boxes"].tolist(); pick = []
                for b in boxes:
                    j = max(range(len(fb)), key=lambda j: _box_iou(b, fb[j]))
                    if _box_iou(b, fb[j]) >= 0.5 and j not in pick: pick.append(j)
                if not pick: return None
                idx = torch.tensor(pick, device=props["masks"].device); return {"masks": props["masks"][idx], "boxes": props["boxes"][idx]}
            self.ism.segmentor_model.generate_masks = gen
        print(f"[live_six] mode {MODE} ({DESC}) objects={len(self.objs)}", flush=True)
