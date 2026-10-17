# map_localizer.py — live localization of the real Husky against the fixed global map (localizer_core.py).
# Replaces hdl_graph_slam + hdl_map_relay.py in navigation mode: the map never changes, the robot's pose in it is
# found by GICP of each VLP-16 scan, predicted with the Clearpath EKF odometry between scans.
#   in : /velodyne_points (velodyne frame), <ns>/tf odom->base_link (EKF), <ns>/initialpose (RViz "2D Pose Estimate")
#   out: map->odom on <ns>/tf at 20 Hz (stamped now + 0.1 s, like AMCL), <ns>/localization/pose, <ns>/localization/status
# Start: INIT=global (default: search the whole map), INIT=origin (robot at the mapping start pose), INIT="x,y,yaw_deg".
# Lost: after LOST_N rejected scans in a row -> local search (1.5 m around the last pose), then global.
#   env: MAP_PCD, MAP_GRID, NS (/a200_0881), INIT, LIDAR (x,y,z,yaw rad; default 0,0,1.215,1.2171), LOST_N (5)
import os, sys, math, json, threading, numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
import tf2_ros
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from localizer_core import MapLocalizer, Tracker, se2, yaw_of

NS = os.environ.get('NS', '/a200_0881')
HERE = os.path.dirname(os.path.abspath(__file__))
MAP_PCD = os.environ.get('MAP_PCD', os.path.join(HERE, 'maps/globalmap_261002.pcd'))
MAP_GRID = os.environ.get('MAP_GRID', os.path.join(HERE, 'maps/globalmap_261002.yaml'))
LIDAR = tuple(float(v) for v in os.environ.get('LIDAR', '0,0,1.215,1.2171').split(','))
INIT = os.environ.get('INIT', 'global'); LOST_N = int(os.environ.get('LOST_N', 5))


def T_of(tr, rot):
    x, y, z, w = rot.x, rot.y, rot.z, rot.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [tr.x, tr.y, tr.z]; return T


def quat_of(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


class Localizer(Node):
    def __init__(self):
        super().__init__('map_localizer')
        self.loc = MapLocalizer(MAP_PCD, MAP_GRID, lidar_xyz_yaw=LIDAR)
        self.trk = Tracker(self.loc, lost_n=LOST_N)
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.T_map_odom = None; self.last = None; self.fails = 0; self.lock = threading.Lock()
        self.latest = None; self.busy = False; self.init_req = None; self.n_ok = 0
        self.create_subscription(PointCloud2, '/velodyne_points', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, NS + '/initialpose', self.on_initialpose, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, NS + '/localization/pose', 10)
        self.status_pub = self.create_publisher(String, NS + '/localization/status', 10)
        self.create_timer(0.05, self.publish_tf)
        threading.Thread(target=self.worker, daemon=True).start()
        if INIT == 'origin': self.trk.set_pose(se2(0.0, 0.0, 0.0))
        elif INIT not in ('global', ''):
            x, y, yd = (float(v) for v in INIT.split(',')); self.trk.set_pose(se2(x, y, math.radians(yd)))
        else: self.trk.set_global()
        self.status('ready', map=os.path.basename(MAP_PCD), map_points=len(self.loc.map_pts), init=INIT)

    def status(self, msg, **kw):
        self.get_logger().info(msg + (' ' + json.dumps(kw) if kw else ''))
        self.status_pub.publish(String(data=json.dumps(dict(msg=msg, **kw))))

    def on_scan(self, m):
        # direct view of x/y/z float32 (read_points_numpy on 28k points x 10 Hz cost most of a core)
        off = {f.name: f.offset for f in m.fields}
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3, 'offsets': [off['x'], off['y'], off['z']],
                       'itemsize': m.point_step})
        a = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
        p = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
        self.latest = (Time.from_msg(m.header.stamp), p[np.isfinite(p).all(1)])

    def on_initialpose(self, m):
        p = m.pose.pose; T = T_of(p.position, p.orientation)
        self.trk.set_pose(se2(T[0, 3], T[1, 3], yaw_of(T))); self.status('initial pose from RViz', x=round(T[0, 3], 2), y=round(T[1, 3], 2))

    def odom_base(self, stamp):
        for t in (stamp, Time()):
            try:
                r = self.buf.lookup_transform('odom', 'base_link', t, timeout=Duration(seconds=0.05)).transform
                return T_of(r.translation, r.rotation)
            except Exception:
                continue
        return None

    def worker(self):
        import time
        while rclpy.ok():
            if self.latest is None: time.sleep(0.01); continue
            stamp, pts = self.latest; self.latest = None
            self.n_scan = getattr(self, 'n_scan', 0) + 1
            if self.n_scan <= 5 or len(pts) < 8000: continue      # start-up / partial scans: no fix from those (a full one has ~28k points)
            T_ob = self.odom_base(stamp)
            if T_ob is None: continue
            if self.trk.request is not None and self.trk.request[0] == 'global': self.status('global relocalization ...')
            T, ev = self.trk.step(self.loc.scan_to_base(pts), T_ob)
            if T is None: continue
            with self.lock: self.T_map_odom = None if self.trk.T_map_odom is None else self.trk.T_map_odom.copy()
            if ev: self.status(ev.pop('msg'), x=round(T[0, 3], 2), y=round(T[1, 3], 2), yaw_deg=round(math.degrees(yaw_of(T)), 1), **ev)
            if self.trk.fails == 0:
                self.n_ok += 1
                if self.n_ok % 100 == 0: self.status('tracking', x=round(T[0, 3], 2), y=round(T[1, 3], 2), rejected=self.trk.n_reject)
                self.publish_pose(stamp, T, dict(err=0.02))

    def publish_pose(self, stamp, T, info):
        m = PoseWithCovarianceStamped(); m.header.stamp = stamp.to_msg(); m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = T[:3, 3]
        q = quat_of(T[:3, :3]); m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
        s = 0.05 + info['err'];  m.pose.covariance[0] = m.pose.covariance[7] = s * s; m.pose.covariance[35] = (0.02 + info['err']) ** 2
        self.pose_pub.publish(m)

    def publish_tf(self):
        with self.lock: T = None if self.T_map_odom is None else self.T_map_odom.copy()
        if T is None: return
        out = TransformStamped(); out.header.stamp = (self.get_clock().now() + Duration(seconds=0.1)).to_msg()
        out.header.frame_id, out.child_frame_id = 'map', 'odom'
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = T[:3, 3]
        q = quat_of(T[:3, :3]); out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q
        self.br.sendTransform(out)


def main():
    rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static'])
    rclpy.spin(Localizer())


if __name__ == '__main__':
    main()
