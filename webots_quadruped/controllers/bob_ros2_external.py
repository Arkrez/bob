#!/usr/bin/env python3
"""ROS 2 -> Webots external-controller bridge for Bob.

Consumes the existing quadruped_control JointGroupPositionController topic:
  /leg_position_controller/commands, Float64MultiArray, 12 radians
Ordering: leg0 J1/J3/J4, leg1, leg2, leg3.
Webots is the sole motor backend in this simulation mode; no ROS2-control
controller_manager is required.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from controller import Robot


NAMES = tuple(
    f"joint{joint}_leg{leg}"
    for leg in range(4)
    for joint in (1, 3, 4)
)
LIMIT = math.pi / 2


class BobBridge(Node):
    def __init__(self, robot):
        super().__init__("bob_webots_bridge")
        self.robot = robot
        self.step_ms = max(1, int(robot.getBasicTimeStep()))
        self.motors = []
        self.sensors = []
        for name in NAMES:
            motor = robot.getDevice(name)
            sensor = robot.getDevice(name + "_sensor")
            if motor is None or sensor is None:
                raise RuntimeError(f"Missing Webots device for {name}")
            sensor.enable(self.step_ms)
            motor.setVelocity(min(2.618, motor.getMaxVelocity()))
            self.motors.append(motor)
            self.sensors.append(sensor)

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, "/leg_position_controller/commands",
            self.command_callback, 10
        )
        self.target = None
        self.have_command = False
        self.get_logger().info(
            "Bob Webots bridge ready: 12 actuators, "
            "/leg_position_controller/commands -> Webots motors"
        )

    def command_callback(self, msg):
        if len(msg.data) != len(NAMES):
            self.get_logger().error(
                f"Expected {len(NAMES)} joint radians, got {len(msg.data)}"
            )
            return
        if not all(math.isfinite(v) and -LIMIT <= v <= LIMIT for v in msg.data):
            self.get_logger().error("Rejected non-finite or out-of-range joint target")
            return
        self.target = tuple(float(v) for v in msg.data)
        self.have_command = True

    def update(self):
        if self.have_command:
            for motor, target in zip(self.motors, self.target):
                motor.setPosition(target)
        else:
            # Hold the CAD starting pose until the first ROS command arrives.
            for motor, sensor in zip(self.motors, self.sensors):
                motor.setPosition(sensor.getValue())
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(NAMES)
        msg.position = [sensor.getValue() for sensor in self.sensors]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        robot = Robot()
        node = BobBridge(robot)
        while rclpy.ok():
            # Drain pending callbacks before issuing the next motor targets.
            rclpy.spin_once(node, timeout_sec=0.0)
            if robot.step(node.step_ms) == -1:
                break
            node.update()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Bob Webots bridge failed: {exc}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
