"""
run_batch_inference.py
폴더 내 모든 RGB 이미지에 대해 SAM-6D (ISM → PEM) 파이프라인을 순차 실행한다.

⚠️  이미지마다 subprocess를 생성해 모델을 재로드하므로 이미지당 ~20 s 소요.
    모델을 한 번만 로드하는 빠른 버전은 run_batch_inference_fast.py 를 사용:
      python run_batch_inference_fast.py [동일한 인자]

전제 조건:
  - Render/render_custom_templates.py 로 템플릿이 이미 생성되어 있어야 한다.
    (기본 위치: {template_dir}/templates/  또는  {output_dir}/templates/)
  - PLY 파일은 렌더링 외에도 추론 시 geometric score 계산에 직접 사용된다.

사용 예시 (단일 카메라 JSON):
  python run_batch_inference.py \
      --rgb_dir       /path/to/lmo/test/000002/rgb \
      --depth_dir     /path/to/lmo/test/000002/depth \
      --cam_path      /path/to/camera.json \
      --cad_path      /path/to/obj_000008.ply \
      --template_dir  /path/to/outputs/templates \
      --output_dir    /path/to/outputs/batch_results

사용 예시 (BOP scene_camera.json):
  python run_batch_inference.py \
      --rgb_dir            /path/to/lmo/test/000002/rgb \
      --depth_dir          /path/to/lmo/test/000002/depth \
      --scene_camera_path  /path/to/lmo/test/000002/scene_camera.json \
      --cad_path           /path/to/obj_000008.ply \
      --template_dir       /path/to/outputs/templates \
      --output_dir         /path/to/outputs/batch_results

결과 구조:
  output_dir/
    000001/
      sam6d_results/
        detection_ism.json
        detection_pem.json
        vis_ism.png
        vis_pem.png
    000002/
      ...
"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time


def parse_args():
    p = argparse.ArgumentParser(description="SAM-6D Batch Inference")
    p.add_argument("--rgb_dir",      required=True, help="RGB 이미지 폴더")
    p.add_argument("--depth_dir",    required=True, help="Depth 이미지 폴더 (RGB와 동일한 파일명)")
    p.add_argument("--cad_path",     required=True, help="CAD 모델 파일 (.ply, mm 단위) — 추론 시 포인트클라우드 샘플링에 사용")
    p.add_argument("--template_dir", required=True, help="렌더링된 템플릿 폴더 (rgb_*.png, mask_*.png, xyz_*.npy 포함)")
    p.add_argument("--output_dir",   required=True, help="결과 저장 루트 폴더 (이미지별 서브폴더 자동 생성)")

    cam_group = p.add_mutually_exclusive_group(required=True)
    cam_group.add_argument("--cam_path",          help="공유 카메라 JSON (camera.json)")
    cam_group.add_argument("--scene_camera_path", help="BOP scene_camera.json (이미지 ID별 카메라 파라미터)")

    p.add_argument("--segmentor_model",           default="sam", choices=["sam", "fastsam"])
    p.add_argument("--stability_score_thresh",    default=0.97, type=float)
    p.add_argument("--det_score_thresh",          default=0.2,  type=float)
    p.add_argument("--img_ext",                   default="png", help="이미지 확장자 (png/jpg)")
    return p.parse_args()


def get_cam_path_for_image(stem, scene_camera, tmp_dir):
    """BOP scene_camera.json 에서 해당 이미지의 카메라 파라미터를 임시 파일로 저장."""
    img_id = int(stem)
    cam = scene_camera.get(str(img_id)) or scene_camera.get(img_id)
    if cam is None:
        raise KeyError(f"scene_camera.json 에 image_id={img_id} 없음")
    tmp_path = os.path.join(tmp_dir, f"cam_{stem}.json")
    with open(tmp_path, "w") as f:
        json.dump(cam, f)
    return tmp_path


def run_ism(stem, img_output_dir, template_dir, cad_path, rgb_path, depth_path,
            cam_path, segmentor_model, stability_score_thresh):
    ism_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Instance_Segmentation_Model")
    cmd = [
        sys.executable,
        os.path.join(ism_dir, "run_inference_custom.py"),
        "--segmentor_model",           segmentor_model,
        "--output_dir",                img_output_dir,
        "--template_dir",              template_dir,
        "--cad_path",                  cad_path,
        "--rgb_path",                  rgb_path,
        "--depth_path",                depth_path,
        "--cam_path",                  cam_path,
        "--stability_score_thresh",    str(stability_score_thresh),
    ]
    result = subprocess.run(cmd, cwd=ism_dir)
    return result.returncode == 0


def run_pem(stem, img_output_dir, template_dir, cad_path, rgb_path, depth_path,
            cam_path, seg_path, det_score_thresh):
    pem_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Pose_Estimation_Model")
    cmd = [
        sys.executable,
        os.path.join(pem_dir, "run_inference_custom.py"),
        "--output_dir",       img_output_dir,
        "--template_dir",     template_dir,
        "--cad_path",         cad_path,
        "--rgb_path",         rgb_path,
        "--depth_path",       depth_path,
        "--cam_path",         cam_path,
        "--seg_path",         seg_path,
        "--det_score_thresh", str(det_score_thresh),
    ]
    result = subprocess.run(cmd, cwd=pem_dir)
    return result.returncode == 0


def main():
    args = parse_args()

    # 서브프로세스는 cwd가 달라지므로 모든 경로를 절대경로로 변환
    args.template_dir = os.path.abspath(args.template_dir)
    args.cad_path     = os.path.abspath(args.cad_path)
    args.rgb_dir      = os.path.abspath(args.rgb_dir)
    args.depth_dir    = os.path.abspath(args.depth_dir)
    args.output_dir   = os.path.abspath(args.output_dir)
    if args.cam_path:
        args.cam_path = os.path.abspath(args.cam_path)
    if args.scene_camera_path:
        args.scene_camera_path = os.path.abspath(args.scene_camera_path)

    # template_dir 존재 확인
    if not os.path.isdir(args.template_dir):
        print(f"[ERROR] template_dir 없음: {args.template_dir}")
        print("  Render/render_custom_templates.py 를 먼저 실행하여 템플릿을 생성하세요.")
        sys.exit(1)

    # BOP scene_camera.json 로드
    scene_camera = None
    if args.scene_camera_path:
        with open(args.scene_camera_path) as f:
            scene_camera = json.load(f)

    # RGB 이미지 목록 수집 (정렬)
    pattern = os.path.join(args.rgb_dir, f"*.{args.img_ext}")
    rgb_files = sorted(glob.glob(pattern))
    if not rgb_files:
        print(f"[ERROR] RGB 이미지 없음: {pattern}")
        sys.exit(1)

    print(f"[INFO] 처리할 이미지 수: {len(rgb_files)}")
    print(f"[INFO] 템플릿 위치: {args.template_dir}")
    print(f"[INFO] 결과 저장: {args.output_dir}")

    # 임시 카메라 JSON 폴더 (BOP 모드)
    tmp_cam_dir = os.path.join(args.output_dir, "_tmp_cam")
    if scene_camera:
        os.makedirs(tmp_cam_dir, exist_ok=True)

    success_count = 0
    fail_list = []

    for idx, rgb_path in enumerate(rgb_files):
        stem = os.path.splitext(os.path.basename(rgb_path))[0]
        depth_path = os.path.join(args.depth_dir, f"{stem}.{args.img_ext}")
        img_output_dir = os.path.join(args.output_dir, stem)
        seg_path = os.path.join(img_output_dir, "sam6d_results", "detection_ism.json")

        print(f"\n[{idx+1}/{len(rgb_files)}] {stem}")

        if not os.path.isfile(depth_path):
            print(f"  [SKIP] Depth 없음: {depth_path}")
            fail_list.append((stem, "depth missing"))
            continue

        # 카메라 경로 결정
        if scene_camera:
            try:
                cam_path = get_cam_path_for_image(stem, scene_camera, tmp_cam_dir)
            except (KeyError, ValueError) as e:
                print(f"  [SKIP] 카메라 정보 없음: {e}")
                fail_list.append((stem, str(e)))
                continue
        else:
            cam_path = args.cam_path

        os.makedirs(os.path.join(img_output_dir, "sam6d_results"), exist_ok=True)

        # ISM
        t0 = time.time()
        ok = run_ism(stem, img_output_dir, args.template_dir, args.cad_path,
                     rgb_path, depth_path, cam_path,
                     args.segmentor_model, args.stability_score_thresh)
        if not ok:
            print(f"  [FAIL] ISM ({time.time()-t0:.1f}s)")
            fail_list.append((stem, "ISM failed"))
            continue
        print(f"  [OK]   ISM ({time.time()-t0:.1f}s)  → {seg_path}")

        # PEM
        t0 = time.time()
        ok = run_pem(stem, img_output_dir, args.template_dir, args.cad_path,
                     rgb_path, depth_path, cam_path, seg_path, args.det_score_thresh)
        if not ok:
            print(f"  [FAIL] PEM ({time.time()-t0:.1f}s)")
            fail_list.append((stem, "PEM failed"))
            continue
        print(f"  [OK]   PEM ({time.time()-t0:.1f}s)  → {img_output_dir}/sam6d_results/vis_pem.png")

        success_count += 1

    print(f"\n{'='*50}")
    print(f"완료: {success_count}/{len(rgb_files)} 성공")
    if fail_list:
        print("실패 목록:")
        for s, r in fail_list:
            print(f"  - {s}: {r}")


if __name__ == "__main__":
    main()
