#!/usr/bin/env python3
"""detect_gsam_prompts.py — FM-Fusion 의 검출 단계(GroundingDINO → NMS → SAM)를 YCB-V 에 돌려 prediction 폴더를 만든다.

FM-Fusion 원래 스크립트(Grounded-Segment-Anything/run_ram_ground_sam.py, glennliu 판)의 함수를 그대로 쓰고,
문구만 바꾼다: RAM 태그 대신 제안 방법(YOLO-World)과 같은 장면별 물체 문구(configs/yolo_ism_ycbv.yaml 의 yolo_prompt,
장면에 주석된 물체만 — run_compare.py 와 같은 목록)를 한 문장으로 넣는다.
임계값은 원래 스크립트 기본값: box 0.35, text 0.25, NMS IoU 0.3. SAM ViT-H, 다중 마스크 끔.
라벨은 문구 → YCB 이름("red cracker box" → "cracker box")으로 바꿔 저장한다(채점기 eval_maps.py 와 같은 이름).

    ~/DeepLearning/third_party/gsam_venv/bin/python objpose/tools/baselines/fmfusion/detect_gsam_prompts.py --gap 10
"""
from __future__ import annotations

import argparse, json, os, sys, time, types
from pathlib import Path

import numpy as np
import torch
import torchvision
import yaml

REPO = Path(__file__).resolve().parents[4]
GSAM = Path.home() / "DeepLearning/third_party/Grounded-Segment-Anything"
sys.path[:0] = [str(GSAM / "segment_anything"), str(GSAM), str(GSAM / "GroundingDINO")]   # 저장소 안의 SAM(build_sam_hq 포함)
# 원래 스크립트는 RAM·litellm 을 모듈 첫머리에서 import 한다 — 이 실험은 RAM 을 쓰지 않으므로 빈 모듈로 막는다.
for m in ("litellm", "ram", "ram.models"):
    sys.modules[m] = types.ModuleType(m)
sys.modules["ram"].inference_ram = None
sys.modules["ram.models"].ram = None
import run_ram_ground_sam as R  # noqa: E402
from segment_anything import build_sam, SamPredictor  # noqa: E402

BOX_T, TEXT_T, IOU_T = 0.35, 0.25, 0.3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, default=Path.home() / "DeepLearning/Datasets/fmfusion_ycbv/scans")
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--gap", type=int, default=10)
    ap.add_argument("--out-name", default="prediction_gsam")
    ap.add_argument("--all-objects", action="store_true", help="표의 모든 물체 문구를 모든 장면에 넣는다(run_compare.py --all-objects 와 같은 목록)")
    ap.add_argument("--prompts", type=Path, default=None, help="{ycb 이름: 문구} json. 없으면 제안 방법과 같은 문구")
    ap.add_argument("--ism-config", type=Path, default=REPO / "sam6d_realtime/configs/yolo_ism_ycbv.yaml", help="물체 문구(yolo_prompt)를 읽을 인식기 설정")
    ap.add_argument("--realtime", type=float, default=0.0, help="영상 fps. 주면 --gap 프레임이 벽시계에 맞춰 도착한다고 보고, 검출이 끝난 순간 이미 도착한 것 중 최신 프레임만 처리한다(첫 프레임은 시간에 넣지 않음)")
    a = ap.parse_args()
    dev = "cuda"
    ism = yaml.safe_load(a.ism_config.read_text())
    prompt = {o["name"]: o["yolo_prompt"] for o in ism["objects"]}
    tab = json.loads(Path(os.environ.get("BENCH_TABLE", REPO / "objpose/rt/bop/ycbv_objects.json")).read_text())
    table, bop = tab["objects"], Path(tab["scenes_dir"])
    by_id = {o["obj_id"]: o["name"] for o in table}
    label = {o["name"]: o["ycb"].split("_", 1)[1].replace("_", " ") for o in table}
    if a.prompts:
        own = json.loads(a.prompts.read_text()); prompt = {o["name"]: own[o["ycb"]] for o in table}

    gd = R.load_model(str(GSAM / "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"),
                      str(GSAM / "checkpoints/groundingdino_swint_ogc.pth"), device=dev)
    sam = SamPredictor(build_sam(checkpoint=str(GSAM / "checkpoints/sam_vit_h_4b8939.pth")).to(dev))
    import cv2
    scenes = a.scenes or sorted(p.name for p in a.data.iterdir() if p.is_dir())
    for s in scenes:
        gt = json.loads((bop / s / "scene_gt.json").read_text())
        names = sorted(by_id.values()) if a.all_objects else sorted({by_id[r["obj_id"]] for v in gt.values() for r in v})
        p2l = {prompt[n]: label[n] for n in names}
        caption = ". ".join(prompt[n] for n in names)
        out = a.data / s / a.out_name
        out.mkdir(exist_ok=True)
        imgs = sorted((a.data / s / "color").glob("frame-*.*"))
        times, n_frames, n_boxes = [], 0, 0
        cand = [p for p in imgs if int(p.stem.split("-")[-1]) % a.gap == 0]; rt0 = None
        for ci, p in enumerate(cand):
            idx = int(p.stem.split("-")[-1])
            if a.realtime > 0 and rt0 is not None:
                now = time.time() - rt0
                if now < idx / a.realtime: time.sleep(idx / a.realtime - now)
                elif ci + 1 < len(cand) and int(cand[ci + 1].stem.split("-")[-1]) / a.realtime <= now: continue
            if a.realtime > 0 and rt0 is None and n_frames == 1: rt0 = time.time() - idx / a.realtime
            torch.cuda.synchronize(); t0 = time.time()
            image_pil, image_t = R.load_image(str(p))
            boxes, scores, _, phrases = R.get_grounding_output(gd, image_t, caption, BOX_T, TEXT_T, device=dev)
            t_gd = time.time() - t0
            n_frames += 1
            if boxes.size(0) == 0:
                times.append((t_gd, 0.0)); continue
            R.recover_bbox(boxes, image_pil.size)
            keep = torchvision.ops.nms(boxes.cpu(), scores, IOU_T).numpy().tolist()
            boxes = boxes.cpu()[keep]; phrases = [phrases[k] for k in keep]
            R.filter_large_bbox(boxes, phrases, image_pil.size, threshold=0.9)   # 원래 스크립트와 같이 호출(결과를 되돌려주지 않아 효과 없음)
            img = cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB)
            t1 = time.time()
            sam.set_image(img)
            tb = sam.transform.apply_boxes_torch(boxes, img.shape[:2]).to(dev)
            masks, _, _ = sam.predict_torch(point_coords=None, point_labels=None, boxes=tb, multimask_output=False)
            torch.cuda.synchronize()
            times.append((t_gd, time.time() - t1))
            labs = [{p2l.get(k, k): v for k, v in tm.items()} for tm in phrases]
            R.save_mask_data(str(out), p.stem, caption, caption, boxes, labs, masks)
            n_boxes += len(boxes)
        t = np.array(times)
        print(f"{s}: {n_frames} frames, {n_boxes} boxes, GroundingDINO {1000 * t[:, 0].mean():.0f} ms, SAM {1000 * t[:, 1].mean():.0f} ms  [{caption}]", flush=True)
        (out / "timing.json").write_text(json.dumps({"offered": len(cand), "realtime_fps": a.realtime, "frames": n_frames, "boxes": n_boxes, "gdino_ms": 1000 * t[:, 0].mean(),
                                                     "sam_ms": 1000 * t[:, 1].mean(), "caption": caption}))


if __name__ == "__main__":
    main()
