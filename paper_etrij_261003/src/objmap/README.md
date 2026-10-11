# Object-map comparison (proposed vs FM-Fusion vs DualMap)

Step 1 of the object-map experiment: the GT maps of the YCB-V test sequences and the evaluator. Every method writes its final object map in one common format, and `evaluate.py` scores it with object-level TP/FP/FN/F1, 3D IoU and surface distance (mm).

## Files

| file | role |
|---|---|
| `common.py` | CAD loading and surface sampling, label → obj_id, map read/write |
| `ycbv_gt.py` | GT maps + common camera trajectories of the 12 YCB-V test sequences → `gt/` |
| `evaluate.py` | scores `out/<method>/ycbv/<scene>/` against `gt/` |
| `selftest.py` | checks the GT against depth and the evaluator against synthetic maps (log: `selftest.log`) |

```
python ycbv_gt.py                 # 1 min, writes gt/ycbv and gt/ycbv_vis
python selftest.py --scenes 48 49 50 51 52 53 54 55 56 57 58 59   # ends with SELFTEST PASS
python evaluate.py --method <name> [--clean] [--match iou|center] [--gt ycbv]
```

## GT (`gt/`)

Data: `~/DeepLearning/Dataset/bop/ycbv/full/test/000048..000059`, all frames (836–2401 per scene, 21 k in total), 55 objects.

- **Camera poses are re-fitted.** The shipped `cam_R_w2c/cam_t_w2c` disagree with the object GT poses by 4–10 mm median, up to 31 mm, while the object poses agree with each other as a rigid set. Each frame's camera is re-fitted to the object poses (alternating mean, 5 rounds): object residual < 0.01 mm and < 0.1°. Depth back-projected through the re-fitted cameras lies 1.5–1.9 mm (median) from the CAD surface, against 2.8–5.8 mm with the shipped cameras. **Every method gets `poses_tum.txt` / `cameras.json` as its camera trajectory.**
- `gt/ycbv/<scene>`: the full CAD surface (2 mm sample spacing) at the GT pose.
- `gt/ycbv_vis/<scene>` (**default**): only the CAD surface that was seen. A point is kept if, in at least one of every 5 frames, it projects inside the object's visible mask and lies within 1 cm of the measured depth. It keeps 42–98 % of the surface (the bottoms are never seen). A fusion mapper can only map the seen surface, so the full CAD would be unfair to it.
- Also per scene: `cameras.json` (K, per-frame T_wc in metres), `poses_tum.txt`, `visibility.json` (per-frame visib_fract), `fit.json`.

## Common map format (what each method must write)

`out/<method>/ycbv/<scene>/map.json` + `points/<k>.npy`; write it with `common.save_map(dir, objects)`.

```json
{"units": "m", "frame": "world", "objects": [
  {"id": 0, "label": "006_mustard_bottle", "points": "points/0.npy"},
  {"id": 1, "label": 14, "T_w_o": [[...4x4...]]}
]}
```

- `label`: obj_id (1–21), the YCB name (`006_mustard_bottle` or `mustard_bottle`), or the prompt text from `ycbv/paths.py`. Other labels (`table`, ...) are dropped before scoring.
- `points`: the object's points in the world frame of `cameras.json`, in metres. If an object only has `T_w_o`, the whole CAD surface is placed at that pose.
- The proposed method places CAD models, so report it two ways: (a) CAD at its estimated pose, (b) its observed points. The baselines only have (b).

## Scoring

- Predicted points are reduced to one per 2 mm voxel (same density for all methods).
- Pairing: same-class Hungarian on 3D IoU (1 cm voxels), kept if IoU ≥ 0.25. With `--match center`, kept if the centre distance is ≤ 5 cm.
- TP = kept pairs. FP = unpaired predictions (wrong labels included). FN = unpaired GT.
- On TP: mIoU, SD = (mean pred→GT + mean GT→pred NN distance)/2 in mm, acc/comp, centre error.
- `--clean` keeps the largest connected cluster of each predicted object, for every method alike. Report raw and `--clean` side by side.
- Frame level (supplement): every visible GT object (visib ≥ 0.1) of every frame counts TP if paired in the final map, else FN. FP = unpaired predictions whose centre lies in that view.

## What the self-test showed (scores against `ycbv_vis`, 55 objects)

| synthetic map | F1 | mIoU | SD mm | remark |
|---|---|---|---|---|
| GT copy | 1.000 | 1.000 | 0.0 | evaluator identity |
| other CAD samples + 2 mm noise | 1.000 | 0.65–0.74 | 1.8 | ceiling of surface-voxel IoU even when the pose is perfect |
| whole CAD at GT pose (method (a)) | 1.000 | 0.738 | 2.8 | unseen side lowers IoU |
| observed depth in GT masks, raw | 0.96 | ~0.49 | 2.9–11 | two objects lost to mask-edge points 0.8 m behind them |
| same, `--clean` | 1.000 | 0.555 | 2.7 | |
| perturbed (drop/move/relabel/fake) | as designed | | | counts asserted |

Read the IoU with this in mind: a perfectly placed fusion map reaches only ~0.55, and a correct CAD placement ~0.74. The IoU ≥ 0.25 pairing threshold is therefore loose enough for an exact map, but raw outlier clusters can still break it. That is why raw and `--clean` are reported together.

## Next

2. Adapters: proposed method (GT poses in, map out in this format), FM-Fusion, DualMap (YCB-V → their input layout, fixed prompt list).
3. ObChange-Kitchen: GT builder for its map-frame annotations, continuous map over the 6 visits with a snapshot per visit.
4. Final numbers on the RTX 5090 machine; this PC (RTX 3080 Ti Laptop) checks the pipeline.
