# Offline test of localizer_core on the mapping bag: replays /velodyne_points with the wheel/EKF odometry as the
# prediction, and compares every pose with the hdl_graph_slam reference recorded in the same bag
# (map->hdl_odom from /hdl/tf composed with /odom), i.e. the trajectory the map itself was built from.
# Also runs global relocalization (no prior) at a few instants.
#   python3 offline_loc_test.py <bag dir> <map.pcd> <grid.yaml> [--every 2] [--reloc 0.25,0.5,0.75]
import argparse, bisect, math, time, sys, os, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs_py import point_cloud2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from localizer_core import MapLocalizer, Tracker, yaw_of


def T_of(p, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.x, p.y, p.z]; return T


def nearest(ts, vals, t):
    i = bisect.bisect_left(ts, t)
    if i == 0: return vals[0]
    if i >= len(ts): return vals[-1]
    return vals[i] if ts[i] - t < t - ts[i - 1] else vals[i - 1]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bag'); ap.add_argument('pcd'); ap.add_argument('grid')
    ap.add_argument('--every', type=int, default=2); ap.add_argument('--reloc', default='0.2,0.4,0.6,0.8')
    a = ap.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag), rosbag2_py.ConverterOptions('', ''))
    scans, wheel_t, wheel_T, hodom_t, hodom_T, mo_t, mo_T = [], [], [], [], [], [], []
    k = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/velodyne_points':
            k += 1
            if k % a.every: continue
            m = deserialize_message(data, PointCloud2)
            p = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.astype(np.float64)))
        elif topic == '/a200_0881/platform/odom/filtered':
            m = deserialize_message(data, Odometry); wheel_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            wheel_T.append(T_of(m.pose.pose.position, m.pose.pose.orientation))
        elif topic == '/odom':
            m = deserialize_message(data, Odometry); hodom_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            hodom_T.append(T_of(m.pose.pose.position, m.pose.pose.orientation))
        elif topic == '/hdl/tf':
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'hdl_odom':
                    mo_t.append(t * 1e-9); mo_T.append(T_of(tr.transform.translation, tr.transform.rotation))
    print(f'scans used {len(scans)} (every {a.every}), wheel {len(wheel_T)}, hdl odom {len(hodom_T)}, map->hdl_odom {len(mo_T)}')

    def ref(t):
        Tmo = nearest(mo_t, mo_T, t) if mo_T else np.eye(4)
        return Tmo @ nearest(hodom_t, hodom_T, t)

    loc = MapLocalizer(a.pcd, a.grid)
    # ---- tracking from the mapping start pose (map origin), with the same Tracker as the live node ----
    trk = Tracker(loc); trk.set_pose(ref(scans[0][0]))
    errs, yerrs, events, dts = [], [], [], []
    for i, (t, p) in enumerate(scans):
        w = nearest(wheel_t, wheel_T, t)
        tic = time.time(); T, ev = trk.step(loc.scan_to_base(p), w); dts.append(time.time() - tic)
        if ev: events.append((round(t - scans[0][0], 1), ev))
        R = ref(t); errs.append(math.hypot(T[0, 3] - R[0, 3], T[1, 3] - R[1, 3]))
        yerrs.append(abs(math.degrees((yaw_of(T) - yaw_of(R) + math.pi) % (2 * math.pi) - math.pi)))
        if i % 200 == 0:
            print(f'  t+{t - scans[0][0]:6.1f}s loc ({T[0,3]:+.2f},{T[1,3]:+.2f},{math.degrees(yaw_of(T)):+6.1f}) ref ({R[0,3]:+.2f},{R[1,3]:+.2f}) '
                  f'err {errs[-1]:.3f} m {yerrs[-1]:.2f} deg')
    fails = sum(1 for _ in events)
    for e_ in events[:20]: print('  event', e_)
    e = np.array(errs); ye = np.array(yerrs)
    print(f'TRACKING: {len(scans)} scans, rejected {trk.n_reject}, lost events {fails}, pos err median {np.median(e):.3f} p95 {np.percentile(e,95):.3f} max {e.max():.3f} m; '
          f'yaw err median {np.median(ye):.2f} max {ye.max():.2f} deg; {1000*np.mean(dts):.1f} ms/scan')
    # ---- global relocalization, no prior ----
    for frac in [float(f) for f in a.reloc.split(',') if f]:
        t, p = scans[int(frac * (len(scans) - 1))]; R = ref(t)
        tic = time.time(); Tg, ratio, nc = loc.relocalize(loc.scan_to_base(p)); dt = time.time() - tic
        print(f'RELOC at {frac:.0%}: err {math.hypot(Tg[0,3]-R[0,3], Tg[1,3]-R[1,3]):.3f} m, '
              f'{abs(math.degrees((yaw_of(Tg)-yaw_of(R)+math.pi)%(2*math.pi)-math.pi)):.2f} deg, fit {ratio:.2f}, {nc} candidates, {dt:.1f} s')


if __name__ == '__main__':
    main()
