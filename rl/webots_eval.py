#!/usr/bin/env python3
"""Measure a walking policy in Webots, using the same tests as MuJoCo (flat ground, no shoves).

Needs the sim running with a policy:  scripts/start_sim.sh --policy rl/policies/rocky_walk.npz
Then, from the repo root:              scripts/sim_ros.sh python3 /workspace/rl/webots_eval.py

Sends each command on /cmd_vel and measures the result from /sim/position (ground-truth GPS)
and /imu: speed in the robot's own frame, turn rate, and whether it tipped over. Rates are per
second of simulation time (/clock), so they don't depend on how fast Webots runs.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import Imu

TESTS = [("forward", (0.15, 0.0, 0.0)), ("run", (0.30, 0.0, 0.0)), ("backward", (-0.10, 0.0, 0.0)),
         ("turn in place", (0.0, 0.0, 0.4)), ("sidestep", (0.0, 0.08, 0.0))]
SECONDS = 10.0
SETTLE = 3.0      # ignore the first seconds of each command
REST = 3.0        # stand still between tests


class Eval(Node):
    def __init__(self):
        super().__init__("bob_webots_eval")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.samples = []      # (sim t, x, y, yaw, upright)
        self.pos = None
        self.sim_time = None
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(PointStamped, "/sim/position", self.on_pos, 10)
        self.create_subscription(Imu, "/imu", self.on_imu, 10)

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_pos(self, msg):
        self.pos = (msg.point.x, msg.point.y)

    def on_imu(self, msg):
        if self.pos is None or self.sim_time is None:
            return
        q = msg.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # body z axis' vertical component: below 0.5 means tipped past 60 degrees
        upright = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.samples.append((self.sim_time, *self.pos, yaw, upright))

    def drive(self, cmd, seconds):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = (float(c) for c in cmd)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def measure(self, cmd):
        self.samples.clear()
        self.drive(cmd, SECONDS)
        if not self.samples:
            return None
        start = self.samples[0][0]
        s = np.array([r for r in self.samples if r[0] - start > SETTLE])
        if len(s) < 10:
            return None
        t, x, y, yaw, upright = s.T
        yaw_u = np.unwrap(yaw)
        dt = t[-1] - t[0]
        # world displacement rotated into the robot's frame at each step
        dx, dy = np.diff(x), np.diff(y)
        mid_yaw = yaw_u[:-1]
        fwd = np.sum(dx * np.cos(mid_yaw) + dy * np.sin(mid_yaw)) / dt
        side = np.sum(-dx * np.sin(mid_yaw) + dy * np.cos(mid_yaw)) / dt
        return fwd, side, (yaw_u[-1] - yaw_u[0]) / dt, bool(np.min(upright) < 0.5)


def main():
    rclpy.init()
    node = Eval()
    for _ in range(10):   # topic discovery can take a few seconds
        node.drive((0, 0, 0), 1.0)
        if node.samples:
            break
    if not node.samples:
        print("No /imu + /sim/position data: is the sim running with Rocky-Bob and the bridge connected?")
        return
    print(f"{'test':15s} {'command':>22s}   {'forward m/s':>11s} {'side m/s':>9s} {'turn rad/s':>10s}  fell")
    for name, cmd in TESTS:
        r = node.measure(cmd)
        node.drive((0, 0, 0), REST)
        if r is None:
            print(f"{name:15s} no data")
            continue
        fwd, side, turn, fell = r
        print(f"{name:15s} vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f}   {fwd:+11.3f} {side:+9.3f} {turn:+10.2f}  {'YES' if fell else 'no'}",
              flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
