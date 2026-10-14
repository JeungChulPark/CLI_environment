# YCB-V single-image comparison (paper Table 1, YCB-V rows)

**Verification run, RTX 3080 Ti Laptop.** These numbers check the pipeline. The final paper numbers will be produced on the RTX 5090 machine with the same scripts. Results: `results.json`; per-image predictions: `out/pred_*.json`; BOP CSVs and bop_toolkit output: `out/bop/`.

Three methods run on the 900 images of the BOP YCB-V test list (`test_targets_bop19.json`). Each method runs over the target objects of the image only, and gives at most one answer per object (the highest score):

| key | method | script |
|---|---|---|
| `sam6d` | original SAM-6D: FastSAM-x + DINOv2 ViT-L/14, weighted-sum ISM, top-1 per object, PEM | `run_orig.py` |
| `ours_text` | our deployed recognizer (`Sam6DCore.process`), YOLO-World text prompts | `run_ours.py --mode text` |
| `ours_gtbox` | same, with the YOLO-World boxes replaced by the GT `bbox_obj` boxes of the targets | `run_ours.py --mode gtbox` |

Settings for YCB (paper): the consensus threshold (`verify.cluster_min_occupancy`) is 0.3 instead of 0.5, and the HSV hue correction is 0 for every object. All other thresholds are the `configs/yolo_ism_objects.yaml` defaults plus the deployed verify defaults (mask IoU ≥ 0.421, texture ≥ 0.450, 20°/25 mm). The prompts are listed in `paths.py`.

## Rerun (any machine)

Every path comes from `paths.py` and can be overridden with an environment variable:

| env var | meaning | default |
|---|---|---|
| `OBJPOSE_REPO` | CLI_environment checkout | 3 levels above this folder |
| `SAM6D_REALTIME` | our recognizer | `$OBJPOSE_REPO/sam6d_realtime` |
| `SAM6D_ORIG` | original SAM-6D code (ISM + PEM) | `$OBJPOSE_REPO/sam6d_ws/sam6d_master/SAM-6D` |
| `YCBV_DIR` | BOP `ycbv` folder (models, models_eval, test, test_targets_bop19.json, camera_uw.json) | `~/DeepLearning/Dataset/bop/ycbv` |
| `YCBV_WORK` | templates, caches, assets, venvs | `~/DeepLearning/Dataset/bop/ycbv_work` |
| `YCBV_OUT` | predictions + BOP output | `./out` |
| `FASTSAM_X` | `FastSAM-x.pt` | `/mnt/d/old/.../checkpoints/FastSAM/FastSAM-x.pt` |
| `DINOV2_VITL_DIR` | folder with `dinov2_vitl14_pretrain.pth` | `~/.cache/sam6d_orig_ckpt/dinov2` |
| `PEM_CKPT` | `sam-6d-pem-base.pth` | `$SAM6D_REALTIME/sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/` |
| `PREDEF_POSES` | SAM-6D `Instance_Segmentation_Model/utils/poses/predefined_poses` | `$YCBV_WORK/predefined_poses` |
| `BOP_TOOLKIT`, `BOP_PYTHON` | bop_toolkit clone and the python that has its deps | `$YCBV_WORK/bop_toolkit`, `$YCBV_WORK/bop_venv/bin/python` |
| `YCBV_RUN_LABEL` | label written into every output | `verification run, RTX 3080 Ti Laptop` |

Checkpoints already in the repo, used read-only: the YOLO-World `yolov8m-worldv2.pt`, `mobile_sam.pt`, DINOv2 ViT-S (`sam6d_master/.../checkpoints/dinov2`), `assets/clip/ViT-B-32.pt` and the PEM `mae_pretrain_vit_base.pth` (all under `sam6d_realtime`).

```bash
PY=/path/to/envs/sam6d/bin/python          # the env of the deployed recognizer
W=${YCBV_WORK:-~/DeepLearning/Dataset/bop/ycbv_work}; mkdir -p $W
cp -r <SAM-6D>/Instance_Segmentation_Model/utils/poses/predefined_poses $W/

# 1) 42-view templates (blenderproc in its own venv; it downloads Blender 4.2.1, CPU Cycles)
python3 -m venv $W/bproc_venv && $W/bproc_venv/bin/pip install blenderproc   # tested: 2.8.0
$W/bproc_venv/bin/blenderproc pip install trimesh opencv-python-headless --blender-install-path $W/blender
$W/bproc_venv/bin/blenderproc run render_templates_bproc.py --blender-install-path $W/blender \
    --models $YCBV_DIR/models --out $W/templates            # about 1.3 min per object on 20 CPU threads

# 2) recognizer config, HSV caches, DINOv2 caches, PEM assets (model points, template features + colours)
$PY build_assets.py

# 3) predictions (about 2 s/img for SAM-6D, about 0.5 s/img for ours on the 3080 Ti)
$PY run_orig.py
$PY run_ours.py --mode text
$PY run_ours.py --mode gtbox

# 4) metrics + official BOP AR
python3 -m venv $W/bop_venv && $W/bop_venv/bin/pip install git+https://github.com/thodan/bop_toolkit
git clone https://github.com/thodan/bop_toolkit $W/bop_toolkit
$PY evaluate.py              # --skip_bop for the fast metrics only
```

What `build_assets.py` writes to `$YCBV_WORK`:
- `ycbv_objects.yaml`: the deployed defaults copied verbatim, plus the 21 YCB objects.
- `features/ycbv_XX_{cls,appe,appe_b2-9,hsv}`: built by the repo's `tools/build_hsv_template_cache.py` and `yolo_ism_object_n.prepare_objects`.
- `core_repo/assets/{model_points,pem_templates,clip}`: the PEM assets `Sam6DCore` loads. `sam6d_core.REPO` is pointed at this folder, so the repo assets are not touched.

Rebuild an asset by deleting it and rerunning `build_assets.py` (`--force` rebuilds everything).

Notes for WSL: the VSD renderer (vispy, EGL) needs the system `libstdc++`. `evaluate.py` sets `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6` when that file exists, because the anaconda one is too old for Mesa's libLLVM.

## Protocol / metrics (`evaluate.py`)
- A pose is **correct** if ADD-S < 10 % of the object diameter (models_eval vertices and `models_info.json`).
- **Found**: visible GT targets with a correct answer, divided by visible GT targets. All 4123 targets have visib_fract ≥ 0.10.
- **Answers correct**: correct answers / all answers. **Wrong / image**: wrong answers / 900.
- **ADD-S AUC**: 0–10 cm, PoseCNN VOCap, pooled over all 4123 targets. A missing answer counts as an infinite error.
- **BOP AR**: the official `scripts/eval_bop19_pose.py` (VSD, MSSD, MSPD).
- **Time**: the median per-image time of the timed region. The timed region covers the whole pipeline from image to poses. It excludes disk reads and switching the per-image target set (YOLO-World `set_classes`, bank indexing). The first 3 images are run once beforehand as warm-up.

## Verification-run results (RTX 3080 Ti Laptop, 2026-10-04)

| | Found | Answers correct | Wrong / img | ADD-S AUC | BOP AR (VSD/MSSD/MSPD) | Time median |
|---|---|---|---|---|---|---|
| SAM-6D | 92.9 % | 93.7 % | 0.287 | 92.4 | 84.1 (82.2/87.0/83.0) | 1573 ms |
| Ours (text) | 61.4 % | 99.0 % | 0.028 | 60.3 | 56.2 (56.4/57.5/54.8) | 1235 ms |
| Ours (GT box) | 64.1 % | 99.1 % | 0.028 | 62.9 | 59.1 (58.9/60.4/57.9) | 1240 ms |
| Ours (GT box, label-locked)* | 78.0 % | 100.0 % | 0.000 | 76.4 | not run | invalid (shared GPU) |

\* Supplementary, not the protocol: `run_ours.py --mode gtbox_locked`. It turns off the per-box owner election (`relative_assignment_enabled`), so a GT box can only be claimed by its own object. In the protocol GT-box run, 937 of the 4123 target GT boxes were elected to a different target object (`diag_ism_gates.py` → `out/diag_ism_gates.json`). The earlier (5090) GT-box numbers, 80.9 / 100 / 0.00 / 79.0, match the label-locked behaviour.
