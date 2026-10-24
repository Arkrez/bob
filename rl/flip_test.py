"""Webots: flip Bob over (180, 150 and 90 degrees) and time how long he takes to walk again.

    scripts/sim_ros.sh python3 /workspace/rl/flip_test.py

Needs the sim in navigation mode (the bridge then subscribes /flip) and rl/policies/rocky_recover.npz.
"""

import math, time, rclpy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String
rclpy.init(); n = rclpy.create_node("flip_test"); st = {"up": 1.0, "status": ""}
def imu(m):
    q = m.orientation; st["up"] = 1 - 2 * (q.x * q.x + q.y * q.y)
n.create_subscription(Imu, "/imu", imu, 5)
n.create_subscription(String, "/nav/status", lambda m: st.__setitem__("status", m.data), 5)
flip = n.create_publisher(Float64, "/flip", 5)
def spin(sec):
    end = time.time() + sec
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
spin(2)
for roll in (180.0, 150.0, 90.0):
    flip.publish(Float64(data=roll)); spin(1.0)
    t0, low, back = time.time(), st["up"], None
    while time.time() - t0 < 25:
        spin(0.2); low = min(low, st["up"])
        if back is None and st["up"] > 0.95 and "righting" not in st["status"] and time.time() - t0 > 2: back = time.time() - t0
    print(f"flip {roll:.0f} deg: lowest up {low:+.2f} | {'walking again after %.1f s' % back if back else 'NOT back up in 25 s'} | status now: {st['status'][:60]}", flush=True)
