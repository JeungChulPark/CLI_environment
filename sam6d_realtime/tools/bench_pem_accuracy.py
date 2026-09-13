#!/usr/bin/env python3
"""bench_pem_accuracy.py — PEM 설정이 pose 정확도를 얼마나 해치는지 잰다.

왜 이렇게 재는가
----------------
이 기계에는 CAD(.ply)도 실제 라벨링된 RGB-D 프레임도 없다. 그래서 BOP 식 ADD 를
계산할 수 없다. 대신 **정답을 내가 만든다**:

  1. 42 장의 렌더 템플릿 중 한 장을 골라 그 마스크 픽셀의 xyz(객체 좌표계)를 꺼낸다.
  2. 거기에 **내가 정한 R, t** 를 곱해 "카메라에서 본 관측 점군"을 만든다.
  3. 그 관측과 (같은 객체의) 템플릿 특징을 PEM 에 넣는다.
  4. PEM 이 돌려준 pred_R, pred_t 를 내가 넣은 R, t 와 비교한다.

즉 **self-consistency 검증**이다. 실제 센서 노이즈·부분 가림·조명 변화가 없으므로
현장 정확도(ADD)와 같지 않다. 그러나 "coarse_npoint 를 196→96 으로 줄이면 포즈가
깨지는가" 라는 질문에는 답할 수 있다 — 그게 지금 가장 큰 미지수다.

관측을 만들 때 관측 뷰(query)와 템플릿 뷰(support)를 **다르게** 고를 수 있게 해 두었다
(--leave-out). 같은 뷰를 쓰면 문제가 너무 쉬워 설정 차이가 드러나지 않는다.

사용
----
    conda activate sam6d
    python tools/bench_pem_accuracy.py --trials 20
    python tools/bench_pem_accuracy.py --trials 20 --set coarse_npoint=96
    python tools/bench_pem_accuracy.py --trials 20 --precision bf16
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
PEM_DIR = REPO / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"
DEFAULT_TEMPLATES = (Path("/home/jucpark/DeepLearning/CLI_environment/sam6d_ws")
                     / "sam6d_master/SAM-6D/Data/custom/Milk_scaled_195mm/templates")


def _add_paths():
    for sub in ("", "provider", "utils", "model", "model/pointnet2"):
        p = str(PEM_DIR / sub) if sub else str(PEM_DIR)
        if p not in sys.path:
            sys.path.insert(0, p)


def build(cfg_overrides, device="cuda:0", load_ckpt=True):
    _add_paths()
    import gorilla, importlib
    cfg = gorilla.Config.fromfile(str(PEM_DIR / "config" / "base.yaml"))
    for k, v in (cfg_overrides or {}).items():
        node, parts = cfg.model, k.split(".")
        for p in parts[:-1]:
            node = node[p]
        node[parts[-1]] = v
    MODEL = importlib.import_module("pose_estimation_model")
    net = MODEL.Net(cfg.model).to(device).eval()
    if load_ckpt:
        ck = PEM_DIR / "checkpoints" / "sam-6d-pem-base.pth"
        gorilla.solver.load_checkpoint(model=net, filename=str(ck))
    return net, cfg


def random_pose(rng, z_range=(0.35, 0.75)):
    """임의 회전 + 카메라 앞 z 거리. 실제 장면의 물체 배치를 흉내낸다."""
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    t = np.array([rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08),
                  rng.uniform(*z_range)], dtype=np.float64)
    return R, t


def make_observation(tem_dir, view, R, t, cfg, device):
    """템플릿 한 장을 알려진 R,t 로 옮겨 '관측'을 만든다. 전처리는 _get_template 과 동일."""
    import cv2
    from data_utils import get_bbox, get_resize_rgb_choose, load_im
    import run_inference_custom as ric

    td = cfg.test_dataset
    N = int(cfg.model.fine_npoint)

    rgb = load_im(str(tem_dir / f"rgb_{view}.png")).astype(np.uint8)
    mask = load_im(str(tem_dir / f"mask_{view}.png")).astype(np.uint8) == 255
    xyz = np.load(tem_dir / f"xyz_{view}.npy").astype(np.float32) / 1000.0

    y1, y2, x1, x2 = get_bbox(mask)
    m = mask[y1:y2, x1:x2]
    crop = rgb[:, :, ::-1][y1:y2, x1:x2, :]
    if td.rgb_mask_flag:
        crop = crop * (m[:, :, None] > 0).astype(np.uint8)
    crop = cv2.resize(crop, (td.img_size, td.img_size), interpolation=cv2.INTER_LINEAR)
    crop = ric.rgb_transform(np.array(crop))

    choose = (m > 0).astype(np.float32).flatten().nonzero()[0]
    idx = np.random.choice(len(choose), N, replace=len(choose) < N)
    choose = choose[idx]
    pts_obj = xyz[y1:y2, x1:x2, :].reshape(-1, 3)[choose, :]        # 객체 좌표계 (m)
    pts_cam = (R @ pts_obj.T).T + t                                  # 관측 = 카메라 좌표계
    rgb_choose = get_resize_rgb_choose(choose, [y1, y2, x1, x2], td.img_size)

    return {
        "rgb": torch.FloatTensor(crop).unsqueeze(0).to(device),
        "rgb_choose": torch.IntTensor(rgb_choose).long().unsqueeze(0).to(device),
        "pts": torch.FloatTensor(pts_cam).unsqueeze(0).to(device),
        "score": torch.ones(1, device=device),
        "model": torch.FloatTensor(pts_obj[np.random.choice(
            len(pts_obj), int(td.n_sample_model_point),
            replace=len(pts_obj) < int(td.n_sample_model_point))]).unsqueeze(0).to(device),
        "K": torch.tensor([[572.4, 0, 325.3], [0, 573.6, 242.0], [0, 0, 1]],
                          dtype=torch.float32, device=device).unsqueeze(0),
    }


def bake_templates(net, tem_dir, cfg, device, exclude=None):
    """실시간 노드가 미리 구워 두는 (dense_po, dense_fo) 를 여기서 만든다."""
    import run_inference_custom as ric
    td = cfg.test_dataset
    views = [v for v in range(42) if v != exclude]
    all_tem, all_tem_pts, all_tem_choose = [], [], []
    for v in views[: int(td.n_template_view)]:
        tem, tem_choose, tem_pts = ric._get_template(str(tem_dir), td, v)
        all_tem.append(torch.FloatTensor(tem).unsqueeze(0).to(device))
        all_tem_choose.append(torch.IntTensor(tem_choose).long().unsqueeze(0).to(device))
        all_tem_pts.append(torch.FloatTensor(tem_pts).unsqueeze(0).to(device))
    with torch.inference_mode():
        dense_po, dense_fo = net.feature_extraction.get_obj_feats(
            all_tem, all_tem_pts, all_tem_choose)
    return dense_po, dense_fo


def rot_err_deg(R_gt, R_pr):
    c = (np.trace(R_gt.T @ R_pr) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--templates", default=str(DEFAULT_TEMPLATES))
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--precision", default="fp32", choices=["fp32", "fp16", "bf16"])
    ap.add_argument("--set", nargs="*", default=None, metavar="KEY=VAL")
    ap.add_argument("--leave-out", action="store_true", default=True,
                    help="관측에 쓴 뷰를 템플릿에서 제외 (기본 켜짐)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    tem_dir = Path(args.templates)
    assert tem_dir.is_dir(), f"템플릿 없음: {tem_dir}"

    ov = {}
    for kv in (args.set or []):
        k, v = kv.split("=", 1)
        ov[k] = int(v) if v.lstrip("-").isdigit() else v
    tag = "+".join(args.set) if args.set else "baseline"

    cwd = os.getcwd()
    os.chdir(PEM_DIR)          # ViT 가 checkpoints/ 를 상대경로로 연다
    try:
        net, cfg = build(ov)
    finally:
        os.chdir(cwd)

    rng = np.random.default_rng(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    rot_errs, tra_errs, times = [], [], []
    for i in range(args.trials):
        view = int(rng.integers(0, 42))
        R, t = random_pose(rng)
        inp = make_observation(tem_dir, view, R, t, cfg, "cuda:0")
        dense_po, dense_fo = bake_templates(
            net, tem_dir, cfg, "cuda:0", exclude=view if args.leave_out else None)
        inp["dense_po"], inp["dense_fo"] = dense_po, dense_fo

        torch.cuda.synchronize(); t0 = time.perf_counter()
        with torch.inference_mode():
            if args.precision == "fp32":
                out = net(dict(inp))
            else:
                dt = torch.float16 if args.precision == "fp16" else torch.bfloat16
                with torch.autocast(device_type="cuda", dtype=dt):
                    out = net(dict(inp))
        torch.cuda.synchronize(); times.append((time.perf_counter() - t0) * 1000)

        Rp = out["pred_R"][0].float().cpu().numpy()
        tp = out["pred_t"][0].float().cpu().numpy()
        rot_errs.append(rot_err_deg(R, Rp))
        tra_errs.append(float(np.linalg.norm(t - tp)) * 1000.0)   # mm

    r = np.array(rot_errs); tr = np.array(tra_errs); tm = np.array(times)
    print("=" * 70)
    print(f"PEM accuracy — {tag} · {args.precision} · {args.trials} trials "
          f"· leave_out={args.leave_out}")
    print(f"  object: {tem_dir.parent.name}")
    print("=" * 70)
    for nm, a, unit in (("rotation error", r, "deg"), ("translation error", tr, "mm")):
        print(f"  {nm:<20} median {np.median(a):8.2f}  mean {a.mean():8.2f}  "
              f"p90 {np.percentile(a,90):8.2f}  max {a.max():8.2f} {unit}")
    print(f"  {'latency':<20} median {np.median(tm):8.2f} ms")
    print(f"  {'rot < 5 deg':<20} {int((r<5).sum())}/{len(r)}"
          f"     {'trans < 20 mm':<16} {int((tr<20).sum())}/{len(tr)}")
    if args.out:
        Path(args.out).write_text(json.dumps(
            dict(tag=tag, precision=args.precision, trials=args.trials,
                 rot_deg=r.tolist(), tra_mm=tr.tolist(), ms=tm.tolist()), indent=1))
        print(f"  wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
