#!/usr/bin/env python3
"""Drive Bob with a trained walking policy over ROS 2 (Webots now, the real robot later).

    python3 rl/ros_policy_node.py rl/runs/walk/policy.npz

Subscribes: /imu (sensor_msgs/Imu), /joint_states, /cmd_vel (geometry_msgs/Twist: linear.x,
            linear.y in m/s, angular.z in rad/s; times out to standing still after 1 s)
Publishes:  /leg_position_controller/commands (12 joint targets, rad) at 50 Hz

Needs only numpy + rclpy. On hardware without joint encoders, pass --open-loop to feed the
last commanded targets in place of measured joint positions.

With --nav rl/policies/rocky_nav.npz it walks to the camera target instead of following
/cmd_vel: camera_target_node.py publishes /target detections (PointStamped, base_link), and the
navigator turns the latest one (plus whether it's visible and how old it is, as in training)
into walker commands every 0.2 s. It stops when the target is within reach.
"""

import argparse
import pathlib
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camera  # noqa: E402
import nav_spec  # noqa: E402
from policy import NumpyPolicy  # noqa: E402
from spec import (CONTROL_DT, JOINT_NAMES, STAND, action_to_targets,  # noqa: E402
                  advance_phase, observation, projected_gravity)

COMMAND_TIMEOUT = 1.0


class PolicyNode(Node):
    def __init__(self, policy_path, open_loop, nav_path=None):
        super().__init__("bob_walk_policy")
        self.nav = None
        if nav_path:   # "scripted" = hand-written turn-then-go navigator; else a trained .npz
            self.nav = "scripted" if nav_path == "scripted" else NumpyPolicy(nav_path)
        self.target, self.target_time = np.zeros(2), -1e9
        self.nav_cmd_norm = np.zeros(3)
        self.policy = NumpyPolicy(policy_path)
        self.open_loop = open_loop
        self.gravity = None
        self.gyro = np.zeros(3)
        self.joints = None
        self.command = np.zeros(3)
        self.command_time = 0.0
        self.last_action = np.zeros(12)
        self.targets = STAND.copy()
        self.t0 = None
        self.phase = 0.0          # gait clock, advanced at the command's stride rhythm

        self.create_subscription(Imu, "/imu", self.on_imu, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        if self.nav is not None:
            self.create_subscription(PointStamped, "/target", self.on_target, 10)
            self.create_timer(nav_spec.NAV_DT, self.navigate)
        else:
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/leg_position_controller/commands", 10)
        self.create_timer(CONTROL_DT, self.tick)
        self.get_logger().info(f"walking policy loaded from {policy_path}; waiting for /imu and /joint_states")

    def on_imu(self, msg):
        o = msg.orientation
        self.gravity = projected_gravity((o.w, o.x, o.y, o.z))
        a = msg.angular_velocity
        self.gyro = np.array([a.x, a.y, a.z])

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in JOINT_NAMES):
            self.joints = np.array([pos[n] for n in JOINT_NAMES])

    def on_cmd(self, msg):
        self.command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.command_time = time.monotonic()

    def on_target(self, msg):
        self.target = np.array([msg.point.x, msg.point.y])
        self.target_time = time.monotonic()

    def navigate(self):
        """Navigator step: camera reading -> walker command (same inputs as the training camera model)."""
        if self.gravity is None:
            return
        age = min(time.monotonic() - self.target_time, camera.MAX_AGE)
        if age <= camera.VISIBLE_FOR and np.linalg.norm(self.target) < nav_spec.GOAL_RADIUS:
            self.command, self.command_time = np.zeros(3), time.monotonic()
            return
        visible = float(age <= camera.VISIBLE_FOR)
        estimate = self.target if self.target_time > -1e9 else np.zeros(2)
        obs = nav_spec.nav_observation(estimate, visible, age / camera.MAX_AGE, self.nav_cmd_norm,
                                       self.gyro[2], self.gravity)
        if self.nav == "scripted":
            cmd = nav_spec.scripted_command(obs)
            self.nav_cmd_norm = nav_spec.command_to_action(cmd)
        else:
            self.nav_cmd_norm = self.nav(obs)
            cmd = nav_spec.action_to_command(self.nav_cmd_norm)
        self.command, self.command_time = cmd, time.monotonic()

    def tick(self):
        if self.gravity is None or (self.joints is None and not self.open_loop):
            return
        if self.t0 is None:
            self.t0 = time.monotonic()
            self.get_logger().info("sensors live: policy running (send /cmd_vel to walk)")
        command = self.command if time.monotonic() - self.command_time < COMMAND_TIMEOUT else np.zeros(3)
        joints = self.targets if self.open_loop else self.joints
        obs = observation(self.gravity, self.gyro, command, joints, self.last_action, self.phase)
        self.phase = advance_phase(self.phase, command)
        action = self.policy(obs)
        self.targets = action_to_targets(action)
        self.last_action = action
        self.pub.publish(Float64MultiArray(data=[float(v) for v in self.targets]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policy", help="path to policy.npz")
    p.add_argument("--open-loop", action="store_true", help="no joint feedback (real MG995s)")
    p.add_argument("--nav", help="'scripted' or a navigator .npz: walk to the camera target (/target) instead of /cmd_vel")
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = PolicyNode(args.policy, args.open_loop, args.nav)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
