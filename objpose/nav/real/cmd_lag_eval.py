#!/usr/bin/env python3
"""cmd_lag_eval.py <cmd_lag_*.csv.gz> — delay from each stage of the command chain to the next and to the wheels.
Delay = shift (0..1.5 s) that best lines up one signal's turn rate (and speed) with the next one, by cross-correlation on a
common 10 ms grid; also message rates and how often the controller's output reached the wheels unchanged."""
import gzip, sys, numpy as np
rows = []
try:                                                    # the recorder is killed with the stack: the gzip has no end marker
    for l in gzip.open(sys.argv[1], "rt"):
        rows.append(l.rstrip().split(","))
except EOFError:
    pass
rows = [r for r in rows[1:] if len(r) == 5]
by = {}
for t, topic, st, vx, wz in rows:
    by.setdefault(topic, []).append((float(t), float(vx), float(wz)))
for k in by:
    by[k] = np.array(by[k])
t0 = min(a[0, 0] for a in by.values()); t1 = max(a[-1, 0] for a in by.values()); g = np.arange(t0, t1, 0.01)
print(f"{(t1 - t0):.0f} s recorded")
for k, a in by.items():
    print(f"  {k:22s} {len(a):6d} msgs, {len(a) / max(t1 - t0, 1e-3):5.1f} Hz, moving {100 * np.mean(np.abs(a[:, 1]) + np.abs(a[:, 2]) > 0.02):3.0f}% of them")
def on_grid(k, col):
    a = by[k]; return np.interp(g, a[:, 0], a[:, col])
def lag(a, b, col):
    x, y = on_grid(a, col), on_grid(b, col)
    act = np.abs(x) > 0.02
    if act.sum() < 200: return None
    best = None
    for s in range(0, 151):                         # b lags a by s * 10 ms
        xs, ys = x[:len(x) - s], y[s:]
        c = np.corrcoef(xs, ys)[0, 1] if xs.std() > 1e-6 and ys.std() > 1e-6 else -1
        if best is None or c > best[1]: best = (s * 10, c)
    return best
chain = [k for k in ("cmd_vel_nav", "cmd_vel", "platform/cmd_vel", "platform/cmd_vel_out", "odom") if k in by]
for col, name in ((2, "turn rate"), (1, "speed")):
    print(f"delay by {name} (best-matching shift, correlation):")
    for a, b in zip(chain, chain[1:]):
        r = lag(a, b, col); print(f"  {a:22s} -> {b:22s} " + ("too little motion" if r is None else f"{r[0]:4d} ms  (r = {r[1]:.2f})"))
    r = lag(chain[0], "odom", col) if "odom" in by and chain[0] != "odom" else None
    print(f"  {chain[0]:22s} -> {'odom (wheels)':22s} " + ("too little motion" if r is None else f"{r[0]:4d} ms  (r = {r[1]:.2f})"))
