"""geometry-only PLYs (vertices + faces) of the lab CAD for bop_toolkit (its loader needs texture_uv when a texture is named)"""
import trimesh, numpy as np
from pathlib import Path
src = Path('objpose/output/pgt_260915_inlong_middle/models'); R = Path('objpose/tools/baselines/bop_root/real')
for d in ('models', 'models_eval'):
    for f in list((R / d).iterdir()):
        if f.suffix in ('.png', '.ply'):
            f.unlink()
    for k in range(1, 10):
        m = trimesh.load(src / f'obj_{k:06d}.ply', process=False)
        trimesh.Trimesh(np.asarray(m.vertices), np.asarray(m.faces), process=False).export(R / d / f'obj_{k:06d}.ply', encoding='binary')
print('ok')
