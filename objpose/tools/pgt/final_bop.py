"""CSV + official BOP scoring of the tuned test runs (tag estcmpT)"""
import json, os, shutil, subprocess, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_bop_eval as M
REPO = M.REPO; BL = M.BL; RES = M.RES; EST = M.EST
t = json.loads((REPO / 'objpose/rt/bop/ycbm_objects.json').read_text()); n2i = {o['name']: o['obj_id'] for o in t['objects']}
for m in ('proposed', 'gtbox'):
    M.csv_from_runs(EST / f'ycbm_{m}T', n2i, RES / f'{m}-estcmpT_ycbm-test.csv')
    shutil.copy(next((EST / f'ycbv_{m}T').glob('bop_*_ycbv.csv')), RES / f'{m}-estcmpT_ycbv-test.csv')
env = dict(os.environ, PYTHONPATH=str(BL / 'bop_toolkit'), PATH=f"{Path.home()}/anaconda3/envs/bop/bin:" + os.environ['PATH'],
           BOP_PATH=str(BL / 'bop_root'), BOP_RESULTS_PATH=str(RES), BOP_EVAL_PATH=str(BL / 'bop_eval'))
for ds in ('ycbv', 'ycbm'):
    for m in ('proposed', 'gtbox'):
        f = f'{m}-estcmpT_{ds}-test.csv'
        log = subprocess.run([f"{Path.home()}/anaconda3/envs/bop/bin/python", 'scripts/eval_bop19_pose.py', f'--result_filenames={f}',
                              '--renderer_type=vispy', '--num_workers=8'], cwd=BL / 'bop_toolkit', env=env, capture_output=True, text=True)
        print(f, [l for l in log.stdout.splitlines() + log.stderr.splitlines() if 'bop19_average_recall:' in l][-1:], flush=True)
