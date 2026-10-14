"""후보 영역 단계 비교 (2026-10-07): 뒤 단계는 원본 판정 사슬(DINOv2 ViT-S → 원본 점수 → 물체별 1위 → PEM, 검증 없음)로 고정하고
후보 영역만 세 가지로 바꾼다.
  fastsam_mask   FastSAM-x 전체 분할 마스크 그대로 (= 사다리 L1)
  fastsam_box    FastSAM-x 전체 분할의 상자만 가져가 MobileSAM 으로 마스크를 다시 만듦 (A안의 후보 방식)
  yolo_box       YOLO-World 문구 상자(대상 물체, 상위 3개) + MobileSAM 마스크 (= 사다리 L2)
이미지마다 후보 상자 목록(cand_boxes)을 기록해 후보 재현율(정답 bbox_visib 와 IoU ≥ 0.5)을 따로 채점할 수 있게 한다.
    python ladder/run_candidates.py --cand fastsam_box [--limit N] [--thresh 0.2]
출력: $YCBV_OUT/pred_ladder_cand_<cand>.json  (evaluate.py --methods ladder_cand_<cand>)
"""
import argparse, json, os, sys, time
from pathlib import Path
import cv2, numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import data as D, paths as P, run_orig as RO            # noqa: E402
from run_ladder import YoloMobileSAM                     # noqa: E402


class FastSAMBoxMobileSAM:
    """FastSAM-x(원본 설정) 상자 → MobileSAM 마스크. 마스크 1개면 복제(원본 특징기가 2-D 로 줄이는 문제 회피)."""
    def __init__(self):
        sys.path.insert(0, str(P.SR))
        import yolo_ism as yi
        self.yi = yi
        self.fast = RO.FastSAMUpstream(P.FASTSAM_X, 640)
        self.seg = yi.build_segmentor(str(P.SR / "mobile_sam.pt"), "cuda:0")

    def activate(self, oids):
        pass

    @torch.no_grad()
    def generate_masks(self, image_rgb):
        d = self.fast.generate_masks(image_rgb)
        if d is None:
            return None
        h, w = image_rgb.shape[:2]
        boxes = [[max(0, int(b[0])), max(0, int(b[1])), min(w, int(b[2])), min(h, int(b[3]))] for b in d["boxes"].tolist()]
        bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        masks = self.yi.segment_boxes(self.seg, bgr, boxes, "cuda:0")
        keep = [(b, m) for b, m in zip(boxes, masks) if m is not None and m.any()]
        if not keep:
            return None
        if len(keep) == 1:
            keep = keep * 2
        return {"masks": torch.from_numpy(np.stack([m for _, m in keep])).float().to(RO.DEV),
                "boxes": torch.tensor([b for b, _ in keep], dtype=torch.float32, device=RO.DEV)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cand", choices=["fastsam_mask", "fastsam_box", "yolo_box"], required=True)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--thresh", type=float, default=0.2)
    a = ap.parse_args()
    # 판정 기준이 원본(0.2)과 다르면 파일·방법 이름에 _thNN 을 붙인다(run_ladder.py 와 같은 규칙)
    tag = a.cand + (f"_th{int(round(a.thresh * 10)):02d}" if abs(a.thresh - 0.2) > 1e-9 else "")
    out_path = P.OUT / f"pred_ladder_cand_{tag}.json"
    items = D.load_targets()
    if a.limit:
        items = items[:a.limit]
    np.random.seed(0); torch.manual_seed(0)
    seg = {"fastsam_mask": None, "fastsam_box": FastSAMBoxMobileSAM, "yolo_box": YoloMobileSAM}[a.cand]
    seg = seg() if seg else None
    ism, bank = RO.build_ism(model_name="dinov2_vits14", segmentor=seg)
    # 후보 상자 기록: 세그멘터 출력을 가로채서 마지막 상자 목록을 보관
    last = {"boxes": []}
    orig_gen = ism.segmentor_model.generate_masks
    def gen(img):
        d = orig_gen(img)
        last["boxes"] = [] if d is None else [[round(float(v)) for v in b] for b in d["boxes"].tolist()]
        return d
    ism.segmentor_model.generate_masks = gen
    net, tcfg, pobj = RO.build_pem()
    cache = {}

    def run(sid, iid, oids):
        bgr, raw, depth_mm, K, ds = D.load_image(sid, iid)
        RO.select_bank(ism, bank, oids, cache)
        if seg is not None:
            seg.activate(oids)
        T, o = RO.run_frame(ism, net, tcfg, pobj, oids, bgr, raw, depth_mm, K, ds, det_thresh=a.thresh)
        o["cand_boxes"] = list(last["boxes"])
        return T, o

    for sid, iid, oids in items[:a.warmup]:
        run(sid, iid, oids)
    res = D.header(f"ladder_cand_{tag}", {
        "pipeline": "candidate stage study: <cand> -> DINOv2 ViT-S -> upstream scoring top-1 -> PEM (no verification)",
        "candidates": a.cand, "descriptor": "dinov2_vits14", "det_score_thresh": a.thresh, "warmup_images": a.warmup})
    res["images"] = []
    for n, (sid, iid, oids) in enumerate(items):
        T, o = run(sid, iid, oids)
        for p in o["preds"]:
            p["time_ms"] = round(T["total"], 2)
        res["images"].append({"scene_id": sid, "im_id": iid, "targets": oids, "time_ms": round(T["total"], 2),
                              "stage_ms": {k: round(v, 2) for k, v in T.items()}, **o})
        if n % 50 == 0 or n == len(items) - 1:
            print(f"[{n + 1}/{len(items)}] {sid}/{iid} {T['total']:.0f} ms cand {o['n_fastsam']} top1 {len(o['ism_top1'])} out {len(o['preds'])}", flush=True)
    res["time_ms"] = D.stats([x["time_ms"] for x in res["images"]])
    res["stage_ms"] = {k: D.stats([x["stage_ms"].get(k, 0.0) for x in res["images"]]) for k in ("proposals", "descriptors", "matching", "pem")}
    res["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(res, open(out_path, "w"), indent=1, default=float)
    print("->", out_path, res["time_ms"])


if __name__ == "__main__":
    main()
