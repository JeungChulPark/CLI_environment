# Send one NavigateToPose goal in the map frame and report the result. usage: python3 goal_test.py x y [timeout_s]
import sys, time, math, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
ns = '/a200_0881'
x, y = float(sys.argv[1]), float(sys.argv[2]); T = float(sys.argv[3]) if len(sys.argv) > 3 else 120
rclpy.init(); n = Node('goal_test'); od = {}
n.create_subscription(Odometry, ns + '/platform/odom', lambda m: od.update(p=m.pose.pose.position), qos_profile_sensor_data)
ac = ActionClient(n, NavigateToPose, ns + '/navigate_to_pose')
assert ac.wait_for_server(timeout_sec=20), 'navigate_to_pose server not available'
g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; g.pose.pose.position.x = x; g.pose.pose.position.y = y
g.pose.pose.orientation.w = 1.0
fb = {}
fut = ac.send_goal_async(g, feedback_callback=lambda f: fb.update(d=f.feedback.distance_remaining))
rclpy.spin_until_future_complete(n, fut); h = fut.result()
if not h.accepted: print('goal rejected'); sys.exit(1)
t0 = time.time(); res = h.get_result_async()
while not res.done() and time.time() - t0 < T: rclpy.spin_once(n, timeout_sec=0.2)
status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.result().status, res.result().status) if res.done() else 'TIMEOUT'
p = od.get('p'); print(f"goal ({x},{y}) -> {status} in {time.time()-t0:.1f}s wall, odom=({p.x:.2f},{p.y:.2f}), last remaining={fb.get('d', float('nan')):.2f} m")
