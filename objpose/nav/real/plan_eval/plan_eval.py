# Compare global-planner settings on the saved room map: for fixed start/goal pairs (free cells, >= 1.5 m apart),
# ask the running planner_server for a path and measure its clearance to the nearest occupied map cell.
#   python3 plan_eval.py <grid.yaml> <out.json> [n_pairs]
import sys, json, math, random, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from scipy.ndimage import distance_transform_edt
import os


def read_grid(yaml_path):
    meta = {}
    for line in open(yaml_path):
        if ':' in line: k, v = line.split(':', 1); meta[k.strip()] = v.strip()
    res = float(meta['resolution']); ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    with open(os.path.join(os.path.dirname(yaml_path), meta['image']), 'rb') as f:
        f.readline(); w, h = map(int, f.readline().split()); f.readline()
        a = np.frombuffer(f.read(w * h), np.uint8).reshape(h, w)[::-1]
    return a >= 250, res, ox, oy

grid_yaml, out = sys.argv[1], sys.argv[2]; N = int(sys.argv[3]) if len(sys.argv) > 3 else 20
free, res, ox, oy = read_grid(grid_yaml)
meta = open(grid_yaml).read()
pgm = os.path.join(os.path.dirname(grid_yaml), [l.split(':', 1)[1].strip() for l in meta.splitlines() if l.startswith('image')][0])
with open(pgm, 'rb') as f:
    f.readline(); w, h = map(int, f.readline().split()); f.readline()
    occ = (np.frombuffer(f.read(w * h), np.uint8).reshape(h, w)[::-1] == 0)
dist = distance_transform_edt(~occ) * res                      # metres to the nearest occupied cell
def clr(x, y):
    i, j = int((y - oy) / res), int((x - ox) / res)
    return float(dist[i, j]) if 0 <= i < dist.shape[0] and 0 <= j < dist.shape[1] else 0.0
random.seed(7)
ys, xs = np.nonzero(free & (dist > 0.7))                       # starts/goals the robot could actually stand on
pts = [(ox + (x + 0.5) * res, oy + (y + 0.5) * res) for y, x in zip(ys, xs)]
pairs = []
while len(pairs) < N:
    a, b = random.sample(pts, 2)
    if math.dist(a, b) >= 1.5: pairs.append((a, b))
rclpy.init(); n = Node('plan_eval'); ac = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
assert ac.wait_for_server(timeout_sec=30), 'planner not up'
rows = []
for a, b in pairs:
    g = ComputePathToPose.Goal(); g.use_start = True; g.planner_id = 'GridBased'
    for ps, (x, y) in ((g.start, a), (g.goal, b)):
        ps.header.frame_id = 'map'; ps.pose.position.x, ps.pose.position.y = x, y; ps.pose.orientation.w = 1.0
    tic = time.time(); f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f); hdl = f.result()
    r = hdl.get_result_async(); rclpy.spin_until_future_complete(n, r); dt = time.time() - tic
    poses = r.result().result.path.poses
    if not poses: rows.append(dict(ok=False)); continue
    xy = np.array([[p.pose.position.x, p.pose.position.y] for p in poses])
    c = np.array([clr(x, y) for x, y in xy]); L = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    rows.append(dict(ok=True, length=round(L, 2), straight=round(math.dist(a, b), 2), min_clear=round(float(c.min()), 3),
                     mean_clear=round(float(c.mean()), 3), p10_clear=round(float(np.percentile(c, 10)), 3), ms=round(dt * 1000)))
ok = [r for r in rows if r['ok']]
summ = dict(pairs=len(rows), planned=len(ok),
            min_clear_median=round(float(np.median([r['min_clear'] for r in ok])), 3),
            min_clear_worst=round(min(r['min_clear'] for r in ok), 3),
            mean_clear=round(float(np.mean([r['mean_clear'] for r in ok])), 3),
            length_ratio=round(float(np.mean([r['length'] / r['straight'] for r in ok])), 3),
            ms_median=round(float(np.median([r['ms'] for r in ok]))))
json.dump(dict(summary=summ, rows=rows), open(out, 'w'), indent=1); print(json.dumps(summ))
