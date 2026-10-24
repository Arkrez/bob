"""Webots: send Bob to a goal on the landing of the east staircase (3 steps of 3 cm) and print
what he does every 3 s: position, body height (it rises as he climbs) and the planner status.

    scripts/sim_ros.sh python3 /workspace/rl/stairs_test.py
    scripts/sim_ros.sh python3 /workspace/rl/stairs_test.py north   (4 steps of 4.5 cm)

Needs the sim in navigation mode with the planner (scripts/start_sim.sh --nav planner).
"""

import sys, time, numpy as np, rclpy
# (start x, y, heading), goal x, y: in front of a staircase, goal on its landing
CASES = {"east": ((3.4, 0.55, -1.5708), (3.35, -1.05)), "north": ((0.8, 1.95, 1.5708), (0.8, 3.0))}
(sx, sy, syaw), (gx, gy) = CASES[sys.argv[1] if len(sys.argv) > 1 else "east"]
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("stairs_trace"); st = {"status": "", "pos": (0, 0, 0)}
n.create_subscription(String, "/nav/status", lambda m: st.__setitem__("status", m.data), 5)
n.create_subscription(PointStamped, "/sim/position", lambda m: st.__setitem__("pos", (m.point.x, m.point.y, m.point.z)), 5)
place = n.create_publisher(PointStamped, "/place", 5); goal = n.create_publisher(PointStamped, "/goal", 5)
def spin(sec):
    end = time.time() + sec
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
spin(1.5)
for _ in range(2):
    p = PointStamped(); p.header.frame_id = "world"; p.point.x, p.point.y, p.point.z = sx, sy, syaw; place.publish(p); spin(1.0)
g = PointStamped(); g.header.frame_id = "world"; g.point.x, g.point.y = gx, gy; goal.publish(g)
for i in range(30):
    spin(3.0)
    x, y, z = st["pos"]
    print(f"{3*i+3:3d}s ({x:+.2f},{y:+.2f}) body z {z:.3f} | {st['status'][:90]}", flush=True)
    if "reached" in st["status"]: break
