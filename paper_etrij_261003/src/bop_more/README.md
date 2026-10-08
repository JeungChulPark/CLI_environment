# More BOP datasets (LM-O, TUD-L, IC-BIN; T-LESS optional): single-image comparison

Copy of `src/ycbv/` generalised to any BOP dataset with public test ground truth. The method
scripts (`run_orig.py`, `run_ours.py`), the metrics (`evaluate.py`, `ladder/same_object_adds.py`)
and the asset pipeline (`render_templates_bproc.py`, `build_assets.py`) are the YCB-V ones;
only the dataset wiring in `paths.py` changed, so the numbers are comparable with the YCB-V rows.

Two methods are compared (the YCB-V paper rows that matter):

| key | method | command |
|---|---|---|
| `ladder_L0` | original SAM-6D "L0": FastSAM-x + DINOv2 ViT-L/14, weighted-sum ISM, top-1 per object, PEM | `run_orig.py --out $BOP_OUT/pred_ladder_L0.json` |
| `hybS05_cf` | 개선안 3 + 군집 먼저: original SAM-6D ISM front end (FastSAM-x masks, ViT-S/14, ISM score > 0.5) + our PEM + pose verification, cluster-first | `run_ours.py --mode text --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full --verify-cluster-first --out $BOP_OUT/pred_hybS05_cf.json` |

No text prompts are needed for either method (the YOLO-World branch is bypassed by `--orig-ism`);
`build_assets.py` still gives every object a unique placeholder prompt (`"<ds> object <id>"`).
As for YCB-V: HSV hue correction 0 for every object, consensus (`verify.cluster_min_occupancy`) 0.3.

## What differs from `src/ycbv/`

`paths.py`
- `BOP_DATASET` = `lmo | tudl | icbin | tless` (default `lmo`) selects
  `BOP_DS_DIR` (default `~/Dataset/bop/<ds>`), `BOP_WORK` (`~/Dataset/bop/<ds>_work`),
  `BOP_OUT` (`./out/<ds>`), `BOP_OURS_CFG` (`<work>/<ds>_objects.yaml`), `BOP_RUN_LABEL`.
- `OIDS` = object ids from `<ds>/models_eval/models_info.json` (LM-O: 1,5,6,8,9,10,11,12).
- `YCB_NAMES` / `PROMPTS` became dicts keyed by obj id; `obj_name(oid)` = `"<ds>_<id:02d>"`.
- The variable names `YCBV` (dataset dir), `WORK`, `OUT` are kept so the other scripts stay identical.
- `PREDEF_POSES` default: `~/Dataset/bop/ycbv_work/predefined_poses` (shared).

`run_orig.py`: `OIDS = P.OIDS`; the per-image bank selection uses `OIDS.index(o)` instead of `o-1`.
`run_ours.py`: object list from `P.OIDS`, prompt lookups by id. `build_assets.py`: ids from `P.OIDS`.
`evaluate.py`: models from `P.OIDS`, per-object keys `obj_name(oid)`, CSV `*_<ds>-test.csv`,
`bop_root/<ds>` symlink. `render_templates_bproc.py`: `--objs` required, CPU-only Cycles with
`--threads` (default 12) because the GPUs are shared.
`ladder/same_object_adds.py`: ids from `P.OIDS`; the ADD-S error of a target is the minimum over
its GT instances and only the best-scored answer per object counts (= `evaluate.py`; identical
for single-instance images, needed for IC-BIN where every target has several instances).

`data.py` is unchanged: depth = raw * `depth_scale` of `scene_camera.json` (1.0 for LM-O / TUD-L /
IC-BIN, 0.1 for YCB-V), so depth is in mm for every dataset.

## Protocol notes
- One answer per (image, object) as in YCB-V. IC-BIN targets have several instances of the
  same object per image (1786 instances for 200 targets): "found" counts a target as found if the
  single answer matches any of its instances; the official BOP AR, which expects `inst_count`
  answers per target, is therefore capped well below 100 for IC-BIN for both methods.
- TUD-L has 1 target object per image (3 scenes x 200 images), LM-O 8 targets in each of the
  200 images.

## Rerun
```bash
export BOP_DATASET=lmo; source env_bop_more.sh        # paths of the 4090 PC
$BPROC run render_templates_bproc.py --blender-install-path ~/blender --models $BOP_DS_DIR/models \
      --out $BOP_WORK/templates --objs 1,5,6,8,9,10,11,12 --threads 12
$SAM6D_PY build_assets.py
CUDA_VISIBLE_DEVICES=2 $SAM6D_PY run_orig.py --out $BOP_OUT/pred_ladder_L0.json
CUDA_VISIBLE_DEVICES=3 $SAM6D_PY run_ours.py --mode text --orig-ism 0.5 --orig-desc dinov2_vits14 \
      --orig-seg fastsam_full --verify-cluster-first --out $BOP_OUT/pred_hybS05_cf.json
ln -sf pred_hybS05_cf.json $BOP_OUT/pred_ladder_hybS05cf.json
$SAM6D_PY evaluate.py --methods ladder_L0,ladder_hybS05cf --results $BOP_OUT/results_${BOP_DATASET}_4090.json
$SAM6D_PY ladder/same_object_adds.py --preds pred_ladder_L0.json,pred_hybS05_cf.json --names L0_원본,개선안3_cf \
      --out $BOP_OUT/same_object_${BOP_DATASET}_4090.json
```
`run_ds.sh <ds>` (copied here) does all of this for one dataset (L0 on GPU2, ours on GPU3).
Results of the 2026-10-08 run: `out_4090/<ds>/` in the server copy of the paper folder, summary in
`bop_more_report.md`.
