#!/usr/bin/env python3
"""Simple standing-in-place joint motion demo through the existing /set_angles API.

This is a small movement smoke test, NOT a stable walking gait.
No hardware servo writes should be enabled while running this test.
"""
import math
import time

import rclpy
from std_msgs.msg import String
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("bob_joint_motion_demo")
    pub = node.create_publisher(String, "/set_angles", 10)
    try:
        start = time.monotonic()
        while rclpy.ok():
            phase = (time.monotonic() - start) * 1.2
            for leg in range(4):
                group_offset = 0 if leg in (0, 3) else math.pi
                swing = 15 * math.sin(phase + group_offset)
                knee = -15 + 10 * math.sin(phase + group_offset + math.pi / 2)
                msg = String()
                # Legacy API leg,J1,J3,J4 (degrees).
                msg.data = f"{leg},0,{swing:.2f},{knee:.2f}"
                pub.publish(msg)
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
