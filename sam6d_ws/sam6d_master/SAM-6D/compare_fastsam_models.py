"""
compare_fastsam_models.py

FastSAM-x.pt vs FastSAM-s.pt 속도/정확도 비교

실행:
  python compare_fastsam_models.py
"""

import os, sys, time
import numpy as np
import torch
from PIL import Image

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
ISM_DIR  = os.path.join(ROOT_DIR, "Instance_Segmentation_Model")
sys.path.insert(0, ISM_DIR)

import yaml
from model.utils import Detections

with open(os.path.join(ROOT_DIR, "pipeline_config.yaml"), encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

RGB_PATH      = cfg["rgb_path"]
DEPTH_PATH    = cfg["depth_path"]
CAM_PATH      = cfg["cam_path"]
CAD_PATH      = cfg["cad_path"]
TMPL_DIR      = cfg["template_dir"]
SEG_MODEL     = cfg.get("segmentor_model", "fastsam")
STAB_THRESH   = cfg.get("stability_score_thresh", 0.97)
MAX_PROPOSALS = 50
N_RUNS        = 3

CKPT_DIR = os.path.join(ISM_DIR, "checkpoints", "FastSAM")
MODELS = {
    "FastSAM-x": os.path.join(CKPT_DIR, "FastSAM-x.pt"),
    "FastSAM-s": os.path.join(CKPT_DIR, "FastSAM-s.pt"),
}

from run_batch_inference_fast import load_ism_model, load_ism_templates, _batch_input_data

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"device: {device}\n")

# 존재하는 모델만 실행
for name, path in list(MODELS.items()):
    if not os.path.isfile(path):
        print(f"[SKIP] {name}: 파일 없음 ({path})")
        del MODELS[name]

if len(MODELS) < 2:
    print("[경고] 비교할 모델이 1개뿐입니다. 단독 측정만 진행합니다.")


def cuda_sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def run_ism_timed(model, label: str) -> dict:
    timing = {}

    def _t(key, t_prev):
        cuda_sync()
        timing[key] = round(time.time() - t_prev, 4)
        return time.time()

    rgb = Image.open(RGB_PATH).convert("RGB")
    rgb_np = np.array(rgb)

    cuda_sync(); t = time.time()
    raw = model.segmentor_model.generate_masks(rgb_np)
    n_raw = len(raw["masks"]) if isinstance(raw, dict) else len(raw)
    detections = Detections(raw)
    if MAX_PROPOSALS > 0 and len(detections) > MAX_PROPOSALS:
        areas = (detections.boxes[:, 2] - detections.boxes[:, 0]) * \
                (detections.boxes[:, 3] - detections.boxes[:, 1])
        _, top_idx = torch.topk(areas, MAX_PROPOSALS)
        detections.filter(top_idx)
    n_after = len(detections)
    t = _t("mask_generation", t)

    cuda_sync(); t = time.time()
    query_desc, query_appe_desc = model.descriptor_model.forward(rgb_np, detections)
    t = _t("dinov2_feature", t)

    cuda_sync(); t = time.time()
    idx_sel, pred_idx, sem_score, best_tmpl = model.compute_semantic_score(query_desc)
    t = _t("semantic_score", t)

    detections.filter(idx_sel)
    query_appe_desc = query_appe_desc[idx_sel, :]

    cuda_sync(); t = time.time()
    appe_scores, ref_aux = model.compute_appearance_score(best_tmpl, pred_idx, query_appe_desc)
    t = _t("appearance_score", t)

    batch = _batch_input_data(DEPTH_PATH, CAM_PATH, device)
    image_uv = model.project_template_to_image(best_tmpl, pred_idx, batch, detections.masks)
    cuda_sync(); t = time.time()
    geo_score, vis_ratio = model.compute_geometric_score(
        image_uv, detections, query_appe_desc, ref_aux,
        visible_thred=model.visible_thred)
    _t("geometric_score", t)

    timing["proposals"] = f"{n_raw}->{n_after}"
    timing["passed"]    = len(idx_sel)
    timing["total"]     = round(sum(v for k, v in timing.items()
                                    if k not in ("proposals", "passed")), 4)
    return timing


STEPS = ["mask_generation", "dinov2_feature", "semantic_score",
         "appearance_score", "geometric_score", "total"]

all_results = {}

for name, ckpt_path in MODELS.items():
    print("=" * 60)
    print(f"  [{name}] 모델 로딩 중...")
    print("=" * 60)
    t0 = time.time()
    ism = load_ism_model(SEG_MODEL, STAB_THRESH, device,
                         dinov2_chunk_size=32,
                         fastsam_checkpoint=ckpt_path)
    ism = load_ism_templates(ism, TMPL_DIR, CAD_PATH, device)
    print(f"  로드 완료: {time.time()-t0:.1f}s")

    print(f"  warm-up...")
    run_ism_timed(ism, name)

    runs = []
    for i in range(N_RUNS):
        r = run_ism_timed(ism, name)
        runs.append(r)
        print(f"  run {i+1}/{N_RUNS}: total={r['total']:.3f}s  proposals={r['proposals']}  passed={r['passed']}")

    avg = {k: round(sum(r[k] for r in runs) / N_RUNS, 4)
           for k in STEPS}
    avg["proposals"] = runs[0]["proposals"]
    avg["passed"]    = runs[0]["passed"]
    all_results[name] = avg

    # 모델 메모리 해제
    del ism
    torch.cuda.empty_cache()
    print()

# 결과 비교 출력
print("=" * 72)
print(f"  비교 결과 ({N_RUNS}회 평균, CUDA sync 적용, max_proposals={MAX_PROPOSALS})")
print("=" * 72)

names = list(all_results.keys())
W = 14

header = f"  {'단계':<20}"
for n in names:
    header += f"  {n:>{W}}"
if len(names) == 2:
    header += f"  {'차이(x-s)':>{W}}"
print(header)
print(f"  {'-'*20}" + f"  {'-'*W}" * (len(names) + (1 if len(names)==2 else 0)))

for s in STEPS:
    row = f"  {s:<20}"
    vals = []
    for n in names:
        v = all_results[n].get(s, 0)
        vals.append(v)
        row += f"  {v:>{W}.4f}s"
    if len(names) == 2:
        diff = vals[0] - vals[1]
        row += f"  {diff:>+{W}.4f}s"
    print(row)

print(f"\n  proposals : " + "  |  ".join(f"{n}={all_results[n]['proposals']}" for n in names))
print(f"  passed    : " + "  |  ".join(f"{n}={all_results[n]['passed']}" for n in names))

if len(names) == 2:
    speedup = all_results[names[0]]["total"] / all_results[names[1]]["total"]
    mask_speedup = all_results[names[0]]["mask_generation"] / all_results[names[1]]["mask_generation"]
    print(f"\n  [요약]")
    print(f"  mask_generation 속도비 (x/s): {mask_speedup:.2f}x")
    print(f"  전체 ISM 속도비     (x/s): {speedup:.2f}x")
    winner = names[0] if all_results[names[0]]["total"] < all_results[names[1]]["total"] else names[1]
    print(f"  더 빠른 모델: {winner}")
