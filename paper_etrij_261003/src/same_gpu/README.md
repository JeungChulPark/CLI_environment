# Same-GPU timing: original SAM-6D vs. our recognizer

GPU: NVIDIA GeForce RTX 3080 Ti Laptop (16 GB). torch 2.7.1, env `sam6d`. No other GPU process ran (nvidia-smi showed 0 MiB before each run).

Frames: `Dataset/260915_eightcircle/SAM` (ConvSession, offset 2 ms, aligned depth, session K, 640x480). The 30 frames spaced evenly over 300..3500, plus frame 1808, give 31 frames. Warm-up used frames 300, 1808 and 3500, and then each frame was timed once. Frames were pre-loaded, and the timed region has no disk I/O, visualization or JSON writing. `torch.cuda.synchronize()` runs at every stage boundary.

Objects (8): milk, choco_hazelnut_high, Febreze_high, Mugcup_high, saffron, Sikhye_high, Bear, Dinosaur. Sauce_high is enabled in the deployed config, but it is not one of the paper's eight objects, so it was excluded from both pipelines.

## Results (run 1; run 2 repeat in parentheses)

| | (A) original SAM-6D | (B) ours |
|---|---|---|
| total ms, median / p90 | 2335.5 / 2632.4 (2268.6 / 2662.9) | 1431.7 / 1928.5 (1405.0 / 1916.8) |
| proposals | FastSAM-x 33.8 | YOLO-World 13.0 |
| descriptors / ISM | DINOv2 ViT-L 1593.1 + scoring/NMS 37.7 | DINOv2 ViT-S + MobileSAM + gates 87.0 |
| PEM | 672.5 (≈132 per object, one forward per object) | 1335.0 incl. verification (≈386 per accepted object) |
| outputs / frame (median, mean) | top-1 objects 6, 6.0; poses (ISM score > 0.2) 5, 5.0 | ISM-accepted 3, 3.4; verified poses 3, 2.3 (3 frames with 0) |
| GPU peak MiB (allocated / reserved) | 3392 / 6206 | 5977 / 14788 |

Variant A with DINOv2 ViT-S/14 (the local config value) took 1108.8 / 1221.8 ms. Its stages were proposals 34.5, descriptors 206.5, matching 31.9 and PEM 812.0, with 6 poses per frame.

## How A was run (`bench_orig.py`)
- The code is the `sam6d_ws` copy of SAM-6D. Against upstream GitHub, its PEM code is identical. Its ISM code differs only in no_grad→inference_mode, a faster RLE encoder (not used here) and a read-only side channel.
- The upstream defaults were restored in code, and no repo file was edited: FastSAM-x (from `/mnt/d/old/...`) and DINOv2 ViT-L/14 (downloaded to `~/.cache/sam6d_orig_ckpt/dinov2`).
- Flow: the upstream `test_step` multi-object path, which makes one ISM pass against an 8×42 template bank. Then top-1 per object, then PEM for each top-1 whose ISM score is above 0.2, with one forward per object using that object's templates.
- Adaptations:
  - The upstream ultralytics-8.0 `CustomYOLO` wrapper cannot be imported with ultralytics 8.4. FastSAM is therefore called through `YOLO.predict` with the wrapper's own overrides: iou 0.9, conf 0.25, max_det 200, imgsz 640, RGB input.
  - `pytorch_lightning`, `hydra` and `ruamel_yaml` are missing from the env and are stubbed. They provide only base classes and unused imports.
  - PEM inputs are built in memory with the same operations as the upstream file-based loader.
- Template poses come from the upstream `predefined_poses`. The CAD files come from `/mnt/d/old/CLI_environment/sam6d_ws/data/cad`.

## How B was run (`bench_ours.py`)
Production `Sam6DCore.process`, with the same construction as `real_stage/run_real_stage.py` (production mode, no diagnostics). It uses the run config `live_260915_eightcircle_orbslam3`, restricted to the 8 objects. "total" is the time around `process()`. The stages are the core's own synchronized ms.

## Files
`raw_*.json` contains per-frame timings and outputs. `results.json` is the summary, produced by `summarize.py`. `log_*.txt` are the console logs; there is no log file for A run 1, because it printed to the console.
