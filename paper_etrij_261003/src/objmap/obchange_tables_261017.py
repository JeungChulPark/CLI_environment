"""Doc 29 section 5 tables: ObChange rooms other than Kitchen, under two conditions.
  default   out/obchange_scores_<room>.json    10 fps BOP frames; baselines with their published offline settings
  realtime  out30/obchange_scores_<room>.json  30 fps raw frames; every method on the same real-time stream
Prints markdown: per-room ctr5cm F1 (per visit / continuous) with the removed-objects-left count, the pooled
totals over the rooms that have scores, and (realtime) how many frames each method took.

    python obchange_tables_261017.py [--rooms office livingarea smallroom bigroom]
"""
import argparse, glob, json, re
from pathlib import Path

HERE = Path(__file__).resolve().parent
W30 = Path.home() / 'DeepLearning/Datasets/objmap_obchange30'
METHODS = [('ours_obs', 'ours_obs'), ('ours_cad', 'ours_cad'), ('fmfusion', 'FM-Fusion'),
           ('dualmap', 'DualMap'), ('dualmap_bgr', 'dualmap_bgr')]
NAME = {'office': 'Office', 'livingarea': 'LivingArea', 'smallroom': 'SmallRoom', 'bigroom': 'BigRoom'}
LAST = {'office': 6, 'livingarea': 6, 'smallroom': 6, 'bigroom': 7}


def load(cond, room):
    p = (HERE / ('out' if cond == 'default' else 'out30')) / f'obchange_scores_{room}.json'
    return json.load(open(p)) if p.exists() else None


def f1(tp, fp, fn):
    return 2 * tp / (2 * tp + fp + fn) if tp else 0.0


def table(cond, rooms):
    have = [r for r in rooms if load(cond, r)]
    out = ['| 방법 | ' + ' | '.join(NAME[r] for r in have) + ' | **합산** |', '|---' * (len(have) + 2) + '|']
    for key, label in METHODS:
        cells, pool = [], {'v': [0, 0, 0], 'a': [0, 0, 0], 'rm': 0, 'st': 0}
        for r in have:
            s = load(cond, r)
            v = s.get(f'{key}|obchange_{r}|ctr5cm'); a = s.get(f'{key}|obchange_{r}_all|ctr5cm')
            if not v or not a:
                cells.append('—'); continue
            rv = a['revisit']
            cells.append(f"{v['f1']:.3f} / {a['f1']:.3f} ({rv['removed_still_in_map']}/{rv['removed']})")
            for k, x in (('v', v), ('a', a)):
                pool[k][0] += x['tp']; pool[k][1] += x['fp']; pool[k][2] += x['fn']
            pool['rm'] += rv['removed']; pool['st'] += rv['removed_still_in_map']
        tot = f"{f1(*pool['v']):.3f} / {f1(*pool['a']):.3f} ({pool['st']}/{pool['rm']})"
        out.append(f'| {label} | ' + ' | '.join(cells) + f' | {tot} |')
    return have, '\n'.join(out)


def rates(rooms):
    rows = ['| 방(30 fps 프레임) | 우리 방법 | FM-Fusion | DualMap | dualmap_bgr |', '|---|---|---|---|---|']
    for r in rooms:
        L = LAST[r]
        sp = HERE / 'out30/ours_runs' / f'obchange_{r}_all_{L:06d}' / 'summary.json'
        rd = W30 / 'fmfusion/scans' / f'obchange_{r}_all_{L:06d}' / 'realtime_detect.json'
        if not sp.exists():
            continue
        s = json.load(open(sp)); n = None
        cells = [f"{s['sam6d_frames']} ({s['sam6d_frames'] / s['duration_s']:.1f} Hz)"]
        if rd.exists():
            d = json.load(open(rd)); n = d['n_frames']
            cells.append(f"{len(d['taken'])} ({len(d['taken']) / (n / 30):.1f} Hz, {d['ms_per_frame']:.0f} ms)")
        else:
            cells.append('—')
        for m in ('dualmap', 'dualmap_bgr'):
            lg = W30 / 'dualmap_runs' / m / f'ob{r[0]}a{L:06d}.log'
            mm = re.search(r'(?:REALTIME|\[RealTime\][^:]*:) processed (\d+) of (\d+)', lg.read_text(errors='ignore')) if lg.exists() else None
            cells.append(f'{mm.group(1)} ({int(mm.group(1)) / (int(mm.group(2)) / 30):.1f} Hz)' if mm else '—')
            n = n or (int(mm.group(2)) if mm else None)
        rows.append(f"| {NAME[r]} ({n if n else '?'}) | " + ' | '.join(cells) + ' |')
    return '\n'.join(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rooms', nargs='*', default=['office', 'livingarea', 'smallroom', 'bigroom'])
    a = ap.parse_args()
    for cond in ('realtime', 'default'):
        have, t = table(cond, a.rooms)
        print(f'### {cond}: {", ".join(NAME[r] for r in have)}\n\n{t}\n')
    print('### realtime frames taken (continuous run of each room)\n\n' + rates(a.rooms))


if __name__ == '__main__':
    main()
