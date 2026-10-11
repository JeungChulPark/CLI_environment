#!/usr/bin/env python3
"""FoundationPose (NVlabs, model-based, RGB-D; register = rotation-grid hypotheses + refiner + scorer)
on the BOP YCB-V test set (BOP19 targets, 900 images) with the BOP default CNOS-FastSAM detections.

Per target object in each target image: CAD obj_XXXXXX.ply (mm -> m), RGB, depth (depth_scale 0.1 -> m,
zfar 1.5 m as in the official run_ycb_video.py), K from scene_camera.json, mask = highest-scoring CNOS
detection of that object (no detection -> no answer). Symmetries from models_info.json exactly as the
official BOP reader (symmetry_tfs_from_info, 5 deg). Object-outer loop like the official script; the
rotation grid is built once (official behaviour). Per-(image,object) wall time is GPU-synchronised and
covers est.register() only (CNOS detection, image loading and the one-off mesh preparation are excluded).

Resumable: results are appended to <out>/results.jsonl; the CSV/timing json are rebuilt at the end.
"""
import os, sys, json, time, argparse, logging
import numpy as np, cv2, imageio, trimesh, torch
from PIL import Image

REPO = os.path.expanduser('~/foundationpose/FoundationPose')
sys.path.insert(0, REPO); sys.path.insert(0, f'{REPO}/mycpp/build')
os.chdir(REPO)
from estimater import *            # noqa: FoundationPose, dr, wp, symmetry_tfs_from_info, set_seed
from pycocotools import mask as mask_utils
logging.getLogger().setLevel(logging.WARNING)   # Utils.set_logging_format() turns INFO spam on


def load_mesh(ycbv, oid):
  f = f'{ycbv}/models/obj_{oid:06d}.ply'
  mesh = trimesh.load(f, process=False)
  mesh.vertices *= 1e-3
  tex = f.replace('.ply', '.png')
  if os.path.exists(tex):
    im = Image.open(tex)
    uv = mesh.visual.uv
    mesh.visual = trimesh.visual.TextureVisuals(uv=uv, image=im, material=trimesh.visual.texture.SimpleMaterial(image=im))
  return mesh


def decode_rle(seg):
  counts, size = seg['counts'], seg['size']
  if isinstance(size, str):
    size = json.loads(size)
  h, w = int(size[0]), int(size[1])
  if isinstance(counts, str) and counts.lstrip().startswith('['):
    counts = json.loads(counts)
  if isinstance(counts, list):
    rle = mask_utils.frPyObjects({'counts': counts, 'size': [h, w]}, h, w)
  else:
    rle = {'counts': counts.encode() if isinstance(counts, str) else counts, 'size': [h, w]}
  return mask_utils.decode(rle).astype(bool)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--ycbv', default=os.path.expanduser('~/gigapose_root/datasets/ycbv'))
  ap.add_argument('--dets', default=os.path.expanduser('~/gigapose_root/datasets/default_detections/core19_model_based_unseen/cnos-fastsam/cnos-fastsam_ycbv-test_f4f2127c-6f59-447c-95b3-28e1e591f1a1.json'))
  ap.add_argument('--out', default=os.path.expanduser('~/foundationpose/out'))
  ap.add_argument('--limit', type=int, default=0, help='only the first N target images (smoke test)')
  ap.add_argument('--zfar', type=float, default=1.5)
  ap.add_argument('--iteration', type=int, default=5, help='refiner iterations (official default 5)')
  ap.add_argument('--name', default='foundationpose')
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=True)

  targets = json.load(open(f'{a.ycbv}/test_targets_bop19.json'))
  img_objs = {}
  for t in targets:
    img_objs.setdefault((t['scene_id'], t['im_id']), set()).add(t['obj_id'])
  images = sorted(img_objs)
  if a.limit:
    images = images[:a.limit]
  img_set = set(images)
  print(f'{len(images)} target images, {sum(len(img_objs[k]) for k in images)} (image,object) targets', flush=True)

  best_det = {}
  for d in json.load(open(a.dets)):
    k = (d['scene_id'], d['image_id'], d['category_id'])
    if k[:2] in img_set and (k not in best_det or d['score'] > best_det[k]['score']):
      best_det[k] = d
  cams = {}
  for sid in sorted({s for s, _ in images}):
    cams[sid] = json.load(open(f'{a.ycbv}/test/{sid:06d}/scene_camera.json'))
  info = json.load(open(f'{a.ycbv}/models/models_info.json'))

  res_path = f'{a.out}/results.jsonl'
  done = {}
  if os.path.exists(res_path):
    for line in open(res_path):
      r = json.loads(line)
      done[(r['scene_id'], r['im_id'], r['obj_id'])] = r
  print(f'{len(done)} results already done (resume)', flush=True)
  fout = open(res_path, 'a')

  wp.force_load(device='cuda')
  glctx = dr.RasterizeCudaContext()
  box = trimesh.primitives.Box(extents=np.ones(3), transform=np.eye(4))
  mesh_tmp = trimesh.Trimesh(vertices=box.vertices.copy(), faces=box.faces.copy(), process=False)  # trimesh 5: primitives are immutable
  est = FoundationPose(model_pts=mesh_tmp.vertices.copy(), model_normals=mesh_tmp.vertex_normals.copy(),
                       symmetry_tfs=None, mesh=mesh_tmp, scorer=None, refiner=None, glctx=glctx,
                       debug_dir=f'{a.out}/debug', debug=0)
  print(f'rot_grid {tuple(est.rot_grid.shape)}', flush=True)

  def load_img(sid, iid):
    base = f'{a.ycbv}/test/{sid:06d}'
    rgb = imageio.imread(f'{base}/rgb/{iid:06d}.png')
    if rgb.ndim == 2:
      rgb = np.tile(rgb[..., None], (1, 1, 3))
    rgb = rgb[..., :3]
    cam = cams[sid][str(iid)]
    depth = cv2.imread(f'{base}/depth/{iid:06d}.png', -1).astype(np.float64) * 1e-3 * cam['depth_scale']
    depth[depth < 0.001] = 0
    depth[depth > a.zfar] = 0
    K = np.array(cam['cam_K'], dtype=np.float64).reshape(3, 3)
    return rgb, depth, K

  warmed = False
  n_nomask = n_degenerate = 0
  t_start = time.time()
  for oid in range(1, 22):
    todo = [(s, i) for (s, i) in images if oid in img_objs[(s, i)] and (s, i, oid) not in done]
    todo_with_mask = [(s, i) for (s, i) in todo if (s, i, oid) in best_det]
    n_nomask += len(todo) - len(todo_with_mask)
    if not todo_with_mask:
      continue
    mesh = load_mesh(a.ycbv, oid)
    sym = symmetry_tfs_from_info(info[str(oid)], rot_angle_discrete=5)
    est.reset_object(model_pts=mesh.vertices.copy(), model_normals=mesh.vertex_normals.copy(), symmetry_tfs=sym, mesh=mesh)
    print(f'obj {oid}: {len(todo_with_mask)} images (mesh {len(mesh.vertices)} v, sym {len(sym)}, diameter {est.diameter:.3f} m)', flush=True)
    for n, (sid, iid) in enumerate(todo_with_mask):
      rgb, depth, K = load_img(sid, iid)
      det = best_det[(sid, iid, oid)]
      mask = decode_rle(det['segmentation'])
      if mask.shape != depth.shape:
        raise RuntimeError(f'mask {mask.shape} vs depth {depth.shape}')
      if not warmed:   # JIT (warp kernels, nvdiffrast, cudnn) is not timed
        est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, ob_id=oid, iteration=a.iteration)
        warmed = True
      est.scores = None
      torch.cuda.synchronize(); t0 = time.perf_counter()
      pose = est.register(K=K, rgb=rgb, depth=depth, ob_mask=mask, ob_id=oid, iteration=a.iteration)
      torch.cuda.synchronize(); dt = time.perf_counter() - t0
      if est.scores is None:      # <4 valid masked depth pixels: register() returns a translation-only guess
        n_degenerate += 1
        score = 0.0
      else:
        score = float(est.scores[0])
      r = {'scene_id': sid, 'im_id': iid, 'obj_id': oid, 'score': score,
           'R': [float(v) for v in pose[:3, :3].reshape(-1)], 't': [float(v) * 1000.0 for v in pose[:3, 3]],
           'time': dt, 'det_score': float(det['score'])}
      done[(sid, iid, oid)] = r
      fout.write(json.dumps(r) + '\n'); fout.flush()
      if n % 25 == 0:
        el = time.time() - t_start
        print(f'  obj {oid} {n + 1}/{len(todo_with_mask)} scene {sid} im {iid} dt {dt:.3f}s score {score:.3f} | total done {len(done)} elapsed {el / 60:.1f} min', flush=True)
  fout.close()
  print(f'done: {len(done)} results; targets without a CNOS mask: {n_nomask}; degenerate masks: {n_degenerate}', flush=True)

  # ---- BOP CSV + timing json (per image time = sum over its objects; BOP convention)
  per_img = {}
  for (sid, iid, oid), r in done.items():
    if (sid, iid) in img_set:
      per_img.setdefault((sid, iid), {})[oid] = r
  csv_path = f'{a.out}/{a.name}_ycbv-test.csv'
  with open(csv_path, 'w') as f:
    f.write('scene_id,im_id,obj_id,score,R,t,time\n')
    for (sid, iid) in sorted(per_img):
      t_img = sum(r['time'] for r in per_img[(sid, iid)].values())
      for oid in sorted(per_img[(sid, iid)]):
        r = per_img[(sid, iid)][oid]
        f.write(f"{sid},{iid},{oid},{r['score']:.6f},{' '.join(f'{v:.8f}' for v in r['R'])},"
                f"{' '.join(f'{v:.5f}' for v in r['t'])},{t_img:.6f}\n")
  img_times = [sum(r['time'] for r in v.values()) for v in per_img.values()]
  obj_times = [r['time'] for v in per_img.values() for r in v.values()]
  timing = {
    'method': 'FoundationPose model-based register (hypotheses + refiner x%d + scorer), CNOS-FastSAM masks' % a.iteration,
    'gpu': torch.cuda.get_device_name(0), 'torch': torch.__version__, 'zfar_m': a.zfar, 'refiner_iterations': a.iteration,
    'note': 'time = est.register() wall time, torch.cuda.synchronize() before/after; excludes CNOS detection (~0.19 s/img), '
            'image/mask loading and the one-off per-object mesh preparation; first call (JIT warm-up) not timed. '
            'per image time = sum over its target objects.',
    'images': len(per_img), 'image_object_results': len(obj_times), 'targets_without_mask': n_nomask,
    'degenerate_masks': n_degenerate,
    'per_image_s': {'median': float(np.median(img_times)), 'mean': float(np.mean(img_times)),
                    'p90': float(np.percentile(img_times, 90)), 'max': float(np.max(img_times))},
    'per_object_s': {'median': float(np.median(obj_times)), 'mean': float(np.mean(obj_times)),
                     'p90': float(np.percentile(obj_times, 90)), 'max': float(np.max(obj_times))},
    'images_detail': [{'scene_id': sid, 'im_id': iid, 'n_obj': len(v), 'time_s': sum(r['time'] for r in v.values()),
                       'per_obj_s': {str(o): r['time'] for o, r in sorted(v.items())}}
                      for (sid, iid), v in sorted(per_img.items())],
  }
  json.dump(timing, open(f'{a.out}/{a.name}_ycbv-test_timing.json', 'w'), indent=1)
  print(f'wrote {csv_path} ({len(obj_times)} rows) and timing json; per-image median {timing["per_image_s"]["median"]:.3f}s, '
        f'per-object median {timing["per_object_s"]["median"]:.3f}s', flush=True)


if __name__ == '__main__':
  set_seed(0)
  main()
