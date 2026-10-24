"""Webots: send Bob to goals behind each wall of the obstacle world (and one on open floor) and
report whether he gets there, how long it takes, and how close he comes to anything.

    scripts/sim_ros.sh python3 /workspace/rl/wall_test.py

Needs the sim in navigation mode (scripts/start_sim.sh --nav planner).
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

GOALS = [("behind wall_east", 2.6, -1.2), ("behind wall_west", -1.25, 2.3), ("behind wall_south", -1.6, -2.8),
         ("beside box_a", 2.25, 1.8), ("beside box_b", -2.4, 0.0), ("open floor", 0.5, 0.5)]
TIMEOUT = 120.0
START = (0.0, 0.0)   # each goal starts here (bridge /place), upright, facing +x, memory cleared


def main():
    rclpy.init()
    n = rclpy.create_node("wall_test")
    st = {"status": "", "min": 9.0, "pos": (0.0, 0.0)}
    n.create_subscription(String, "/nav/status", lambda m: st.__setitem__("status", m.data), 5)
    n.create_subscription(PointStamped, "/sim/position", lambda m: st.__setitem__("pos", (m.point.x, m.point.y)), 5)

    def imu(m):
        q = m.orientation
        st["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    n.create_subscription(Imu, "/imu", imu, 5)

    def scan(m):
        # nearest return, ignoring the target post itself
        r = np.array(m.ranges)
        a = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r)
        if st.get("goal") is not None and "yaw" in st:
            dx, dy = st["goal"][0] - st["pos"][0], st["goal"][1] - st["pos"][1]
            c, s_ = math.cos(st["yaw"]), math.sin(st["yaw"])
            gx, gy = c * dx + s_ * dy, -s_ * dx + c * dy
            px, py = r * np.cos(a), r * np.sin(a)
            ok &= np.hypot(px - gx, py - gy) > 0.12
        if ok.any():
            st["min"] = min(st["min"], float(r[ok].min()))
    n.create_subscription(LaserScan, "/scan", scan, 5)
    goal = n.create_publisher(PointStamped, "/goal", 5)
    place = n.create_publisher(PointStamped, "/place", 5)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    spin(2)
    results = []
    for name, gx, gy in GOALS:
        p = PointStamped()
        p.header.frame_id = "world"
        p.point.x, p.point.y = START
        for _ in range(2):   # the first message can go out before the bridge is listening
            place.publish(p)
            spin(1.0)
        g = PointStamped()
        g.header.frame_id = "world"
        g.point.x, g.point.y = gx, gy
        goal.publish(g)
        st["goal"] = (gx, gy)
        spin(1.0)
        st["status"], st["min"] = "", 9.0
        start, t0 = st["pos"], time.time()
        while "reached" not in st["status"] and time.time() - t0 < TIMEOUT:
            spin(0.2)
        took = time.time() - t0
        results.append(took < TIMEOUT)
        print(f"{name:18s} from ({start[0]:+.1f},{start[1]:+.1f}): "
              f"{'reached in %4.1f s' % took if took < TIMEOUT else 'NOT reached in %.0f s' % TIMEOUT} | "
              f"closest to anything {st['min']:.2f} m (lidar)", flush=True)
    print(f"reached {sum(results)}/{len(results)}")


if __name__ == "__main__":
    main()
