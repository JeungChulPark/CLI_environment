# How does each global planner bring the robot INTO a goal pose? For start/goal poses (x, y, yaw deg) ask the running
# planner_server and measure the last 1.5 m of the path against the goal's own heading line.
#   python3 approach_eval.py <out.json> "<sx,sy,syaw>;<gx,gy,gyaw>" ...
import sys, json, math, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
rclpy.init(); n = Node('approach_eval'); ac = ActionClient(n, ComputePathToPose, '/compute_path_to_pose')
assert ac.wait_for_server(timeout_sec=40), 'planner not up'
rows = []
for spec in sys.argv[2:]:
    s, g = [[float(v) for v in p.split(',')] for p in spec.split(';')]
    goal = ComputePathToPose.Goal(); goal.use_start = True; goal.planner_id = 'GridBased'
    for ps, (x, y, yaw) in ((goal.start, s), (goal.goal, g)):
        ps.header.frame_id = 'map'; ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
    tic = time.time(); f = ac.send_goal_async(goal); rclpy.spin_until_future_complete(n, f); h = f.result()
    r = h.get_result_async(); rclpy.spin_until_future_complete(n, r); ms = (time.time() - tic) * 1000
    P = r.result().result.path.poses
    if len(P) < 3: rows.append(dict(spec=spec, ok=False, ms=round(ms))); continue
    xy = np.array([[p.pose.position.x, p.pose.position.y] for p in P])
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]; L = d[-1]
    last = xy[d >= L - 1.5]
    gh = np.array([math.cos(math.radians(g[2])), math.sin(math.radians(g[2]))])
    lat = (last - np.array(g[:2])) @ np.array([-gh[1], gh[0]])            # sideways offset from the goal's heading line
    k = max(len(xy) - 6, 0); v = xy[-1] - xy[k]; ph = math.degrees(math.atan2(v[1], v[0]))
    dh = (ph - g[2] + 180) % 360 - 180
    # direction changes (forward <-> reverse) show up as sharp turns of the path tangent
    t = np.diff(xy, axis=0); t = t[np.linalg.norm(t, axis=1) > 1e-4]; a = np.degrees(np.arctan2(t[:, 1], t[:, 0]))
    cusps = int((np.abs((np.diff(a) + 180) % 360 - 180) > 120).sum())
    rows.append(dict(spec=spec, ok=True, ms=round(ms), length=round(float(L), 2), end_miss_cm=round(float(np.linalg.norm(xy[-1] - g[:2])) * 100, 1),
                     sideways_in_last_1p5m_cm=round(float(np.abs(lat).max()) * 100, 1), arrival_heading_vs_goal_deg=round(dh, 1), reversals=cusps,
                     path=[[round(float(x), 3), round(float(y), 3)] for x, y in xy[::max(1, len(xy) // 150)]]))
json.dump(rows, open(sys.argv[1], 'w'))
for r in rows: print({k: v for k, v in r.items() if k != 'path'})
