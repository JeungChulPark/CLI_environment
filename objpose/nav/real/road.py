"""road.py — drive along the path the robot was driven by hand while mapping (the "road"), instead of a fresh plan.

The planner draws its own line through the room (it keeps away from walls and takes short cuts), the controller cuts
corners of that line, and the robot comes out in an S. The hand-driven path is already a smooth line through free
space, so for a goal near it the robot is sent ALONG it (nav2 FollowPath = the controller only, MPPI still avoids live
obstacles). No direction: the road is used either way, the shorter way round on a closed loop.

  Road(points)                         points = [[x, y, ...], ...] in the map frame (mapping-run poses)
  road.path_to(x, y, yaw, gx, gy, gyaw) -> [[x, y, yaw], ...] or None (start or goal too far from the road)
numpy only (runs on the laptop).
"""
import math
import numpy as np

STEP = 0.05            # m between road points
SMOOTH = 0.25          # m half-window of the moving average (hand driving wobbles)
GOAL_OFF = 0.9         # a goal farther than this from the road is not a road goal
START_OFF = 1.5        # nor is a start farther than this
TURN_COST = 1.0        # m added to a way round that starts against the robot's heading (it has to turn about first)


def _resample(p, step):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(p, axis=0).T))]
    s = np.arange(0, d[-1], step)
    return np.column_stack([np.interp(s, d, p[:, 0]), np.interp(s, d, p[:, 1])])


class Road:
    def __init__(self, points):
        p = np.asarray([q[:2] for q in points], float)
        keep = [0]
        for i in range(1, len(p)):                       # standing still / turning on the spot add nothing
            if np.hypot(*(p[i] - p[keep[-1]])) >= 0.03:
                keep.append(i)
        p = _resample(p[keep], STEP)
        self.length = float(len(p) * STEP)
        self.closed = self.length > 3.0 and np.hypot(*(p[0] - p[-1])) < 1.0
        if self.closed:                                  # close the gap so the loop has no seam
            p = np.vstack([p, _resample(np.vstack([p[-1], p[0]]), STEP)[1:]])
        k = max(1, int(SMOOTH / STEP))
        if self.closed:
            q = np.vstack([p[-k:], p, p[:k]]); c = np.cumsum(np.vstack([np.zeros((1, 2)), q]), 0)
            p = (c[2 * k + 1:] - c[:-2 * k - 1]) / (2 * k + 1)
        else:
            q = np.vstack([np.repeat(p[:1], k, 0), p, np.repeat(p[-1:], k, 0)]); c = np.cumsum(np.vstack([np.zeros((1, 2)), q]), 0)
            p = (c[2 * k + 1:] - c[:-2 * k - 1]) / (2 * k + 1)
        self.p = p; self.n = len(p)

    def nearest(self, x, y):
        d = np.hypot(self.p[:, 0] - x, self.p[:, 1] - y); i = int(d.argmin()); return i, float(d[i])

    def _arc(self, i, j, step):
        if not self.closed:
            return list(range(i, j + step, step)) if (j - i) * step >= 0 else []
        out, k = [i], i
        while k != j:
            k = (k + step) % self.n; out.append(k)
        return out

    def path_to(self, x, y, yaw, gx, gy, gyaw):
        i, ds = self.nearest(x, y); j, dg = self.nearest(gx, gy)
        if ds > START_OFF or dg > GOAL_OFF:
            return None
        ways = [self._arc(i, j, 1), self._arc(i, j, -1)] if self.closed else [self._arc(i, j, 1 if j >= i else -1)]
        best = None
        for idx in ways:
            if len(idx) < 2:
                continue
            q = self.p[idx]; L = len(idx) * STEP
            t = q[min(len(q) - 1, 6)] - q[0]                # direction the road leaves in
            if abs(math.atan2(math.sin(math.atan2(t[1], t[0]) - yaw), math.cos(math.atan2(t[1], t[0]) - yaw))) > math.pi / 2:
                L += TURN_COST
            if best is None or L < best[0]:
                best = (L, q)
        if best is None:                                  # start and goal at the same place of the road
            q = np.array([[x, y], [gx, gy]])
        else:
            q = best[1]
            # join the road a little ahead (not behind the robot), leave it at the point nearest the goal
            q = np.vstack([[x, y], q[min(10, len(q) - 1):], [gx, gy]])        # join 0.5 m ahead: a gentle entry
        if len(q) > 2:
            q = _resample(q, STEP)
            if np.hypot(q[-1, 0] - gx, q[-1, 1] - gy) > 0.01:
                q = np.vstack([q, [gx, gy]])
        h = np.arctan2(np.diff(q[:, 1]), np.diff(q[:, 0])); h = np.r_[h, gyaw]
        return [[float(a), float(b), float(c)] for (a, b), c in zip(q, h)]
