# Robot-tree map->odom from hdl_graph_slam, 20 Hz, stamped now (+0.1 s) like AMCL.
#   hdl (in /hdl/tf):  map -> hdl_odom (graph, keyframe-stamped)   /odom: hdl_odom -> base_link (scan matching, t)
#   robot (/a200_0881/tf): odom -> base_link (EKF)
#   map->odom = map->hdl_odom * hdl_odom->base_link(t) * inverse(odom->base_link(t))
import os, numpy as np, rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf2_ros

NS = '/a200_0881'
SIM_TIME = os.environ.get('USE_SIM_TIME', 'true') == 'true'   # real robot: USE_SIM_TIME=false


def mat(t, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]; return m


def quat(r):
    # rotation matrix -> (x, y, z, w)
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


class Relay(Node):
    def __init__(self):
        super().__init__('hdl_map_relay', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=SIM_TIME)])
        self.map_hdlodom = np.eye(4); self.hdl_odom = None
        self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)   # remapped to the robot tree
        self.create_subscription(TFMessage, '/hdl/tf', self.on_hdl_tf, 50)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.05, self.tick)

    def on_hdl_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'hdl_odom':
                self.map_hdlodom = mat(t.transform.translation, t.transform.rotation)

    def on_odom(self, m):
        p = m.pose.pose; self.hdl_odom = (Time.from_msg(m.header.stamp), mat(p.position, p.orientation))

    def tick(self):
        if self.hdl_odom is None: return
        t, hdl_base = self.hdl_odom
        try:
            tf = self.buf.lookup_transform('odom', 'base_link', t, timeout=Duration(seconds=0.0))
        except Exception:
            try: tf = self.buf.lookup_transform('odom', 'base_link', Time())
            except Exception: return
        odom_base = mat(tf.transform.translation, tf.transform.rotation)
        m = self.map_hdlodom @ hdl_base @ np.linalg.inv(odom_base)
        out = TransformStamped()
        out.header.stamp = (self.get_clock().now() + Duration(seconds=0.1)).to_msg()
        out.header.frame_id, out.child_frame_id = 'map', 'odom'
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = m[:3, 3]
        q = quat(m[:3, :3])
        out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q
        self.br.sendTransform(out)


rclpy.init(args=['--ros-args', '-r', f'/tf:={NS}/tf', '-r', f'/tf_static:={NS}/tf_static'])
rclpy.spin(Relay())
