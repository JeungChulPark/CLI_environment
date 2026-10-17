# 2D occupancy grid for Nav2 from hdl_graph_slam's map cloud (/hdl_graph_slam/map_points, map frame):
# floor points -> free, points in the robot's height band -> occupied, rest unknown. Published as /a200_0881/map.
# map frame = base_link at the first keyframe; the Husky base_link is 0.13 m above the ground, so the floor is at z = FLOOR.
import os, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid

# floor rings from the VLP-16 land up to ~0.15 m off the nominal floor, so obstacles start at 0.35 m
SIM_TIME = os.environ.get('USE_SIM_TIME', 'true') == 'true'   # real robot: USE_SIM_TIME=false
RES, FLOOR, FLOOR_TOL, BAND, MIN_PTS = 0.1, -0.13, 0.2, (0.35, 1.8), 3


class Grid(Node):
    def __init__(self):
        super().__init__('hdl_grid', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=SIM_TIME)])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/a200_0881/map', latched)
        self.create_subscription(PointCloud2, '/hdl_graph_slam/map_points', self.on_cloud,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        # hdl publishes map points only after the robot has moved; until then give Nav2 an all-unknown map
        # (its costmaps wait for a map before planning).
        self.have_cloud = False
        self.create_timer(1.0, self.initial_map)

    def initial_map(self):
        if self.have_cloud or self.get_clock().now().nanoseconds == 0: return
        msg = OccupancyGrid(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'map'
        msg.info.resolution = RES; msg.info.width = msg.info.height = 400
        msg.info.origin.position.x = msg.info.origin.position.y = -20.0; msg.info.origin.orientation.w = 1.0
        msg.data = [-1] * (400 * 400); self.pub.publish(msg); self.have_cloud = True
        self.get_logger().info('initial all-unknown 40x40 m map published')

    def on_cloud(self, m):
        p = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        if len(p) == 0: return
        self.have_cloud = True
        z = p[:, 2] - FLOOR
        free = p[np.abs(z) < FLOOR_TOL]; occ = p[(z > BAND[0]) & (z < BAND[1])]
        lo = np.floor(p[:, :2].min(0) / RES) * RES - 1.0; hi = p[:, :2].max(0) + 1.0
        w, h = (np.ceil((hi - lo) / RES)).astype(int)
        g = np.full((h, w), -1, np.int8)
        ij = ((free[:, :2] - lo) / RES).astype(int); g[ij[:, 1], ij[:, 0]] = 0
        # occupied only where >= MIN_PTS map points fall in the height band (drops isolated noise points)
        ij = ((occ[:, :2] - lo) / RES).astype(int); cnt = np.zeros((h, w), np.int32)
        np.add.at(cnt, (ij[:, 1], ij[:, 0]), 1); g[cnt >= MIN_PTS] = 100
        msg = OccupancyGrid(); msg.header.stamp = m.header.stamp; msg.header.frame_id = 'map'
        msg.info.resolution = RES; msg.info.width, msg.info.height = int(w), int(h)
        msg.info.origin.position.x, msg.info.origin.position.y = float(lo[0]), float(lo[1])
        msg.info.origin.orientation.w = 1.0; msg.data = g.ravel().tolist()
        self.pub.publish(msg)
        self.get_logger().info(f'grid {w}x{h} from {len(p)} pts (occ {len(occ)}, free {len(free)})')


rclpy.init(); rclpy.spin(Grid())
