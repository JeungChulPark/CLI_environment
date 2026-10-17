#!/usr/bin/env python3
"""make_nav2_params.py — nav2_bringup default params adapted for Husky A200 + hdl_graph_slam (no AMCL).
  python3 make_nav2_params.py out.yaml [--cmd-vel /cmd_vel]"""
import sys, yaml
src = "/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"
out = sys.argv[1]
P = yaml.safe_load(open(src))
FOOT = "[[0.50, 0.35], [0.50, -0.35], [-0.50, -0.35], [-0.50, 0.35]]"   # Husky A200 ~0.99 x 0.67 m
P.pop("amcl", None)
for cm in ("local_costmap", "global_costmap"):
    p = P[cm][cm]["ros__parameters"]
    p.pop("robot_radius", None)
    p["footprint"] = FOOT
    p["robot_base_frame"] = "base_link"
    obst = {"plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True, "observation_sources": "scan",
            "scan": {"topic": "/scan", "data_type": "LaserScan", "clearing": True, "marking": True,
                     "max_obstacle_height": 2.0, "min_obstacle_height": -2.0,
                     "raytrace_max_range": 12.0, "obstacle_max_range": 10.0}}
    if cm == "local_costmap":
        p["plugins"] = ["obstacle_layer", "inflation_layer"]
        p.pop("voxel_layer", None)
        p["width"] = p["height"] = 6
    p["obstacle_layer"] = obst
    p["inflation_layer"]["inflation_radius"] = 0.8
    p["inflation_layer"]["cost_scaling_factor"] = 3.0
cm = P["collision_monitor"]["ros__parameters"]
cm["base_frame_id"] = "base_link"
for k in ("FootprintApproach",):
    if k in cm: cm[k]["footprint_topic"] = "/local_costmap/published_footprint"
P["collision_monitor"]["ros__parameters"]["scan"]["topic"] = "/scan"
# Husky A200 limits (clearpath default max 1.0 m/s); keep indoor-safe speeds
vs = P["velocity_smoother"]["ros__parameters"]
vs["max_velocity"] = [0.5, 0.0, 1.0]; vs["min_velocity"] = [-0.3, 0.0, -1.0]
mppi = P["controller_server"]["ros__parameters"]["FollowPath"]
mppi["vx_max"] = 0.5; mppi["vx_min"] = -0.3; mppi["wz_max"] = 1.0
yaml.safe_dump(P, open(out, "w"), sort_keys=False)
print("wrote", out)
