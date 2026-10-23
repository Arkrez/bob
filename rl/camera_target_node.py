#!/usr/bin/env python3
"""Pi camera -> /target: find an ArUco marker and publish where it is relative to the robot.

Runs on the Raspberry Pi 5 with a Camera Module 3 behind the rock's front window:
    python3 rl/camera_target_node.py --marker-size 0.08

Publishes /target (geometry_msgs/PointStamped, frame "base_link": x forward, y left, metres)
for every frame in which the marker is found. ros_policy_node.py --nav turns those into the
navigator's camera inputs (last estimate, visible, age), exactly like the training camera model.

Print a marker from OpenCV's DICT_4X4_50 dictionary (id 0 by default), measure the black
square's side for --marker-size, and mount it on a post about as high as the camera
(~0.25 m when Bob stands). For accurate distances, calibrate the camera once and pass
--calibration calib.npz (with arrays K and dist); otherwise nominal Module 3 optics are used.

Untested on hardware: written against Picamera2 and OpenCV >= 4.7 (cv2.aruco.ArucoDetector).
"""

import argparse
import pathlib
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from camera import CAMERA_X  # noqa: E402

WIDTH, HEIGHT = 640, 360
# Camera Module 3 nominal optics: 4.74 mm lens, 6.45 mm wide sensor -> focal length in pixels
FX = 4.74 / 6.45 * WIDTH


class CameraTarget(Node):
    def __init__(self, marker_id, marker_size, calibration):
        super().__init__("bob_camera_target")
        import cv2
        from picamera2 import Picamera2

        self.cv2 = cv2
        self.marker_id, self.size = marker_id, marker_size
        if calibration:
            c = np.load(calibration)
            self.K, self.dist = c["K"], c["dist"]
        else:
            self.K = np.array([[FX, 0, WIDTH / 2], [0, FX, HEIGHT / 2], [0, 0, 1]])
            self.dist = np.zeros(5)
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                                cv2.aruco.DetectorParameters())
        s = marker_size / 2
        self.corners_3d = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

        self.cam = Picamera2()
        self.cam.configure(self.cam.create_video_configuration(main={"size": (WIDTH, HEIGHT), "format": "RGB888"}))
        self.cam.start()
        self.pub = self.create_publisher(PointStamped, "/target", 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"looking for ArUco id {marker_id} ({marker_size * 100:.0f} cm)")

    def tick(self):
        cv2 = self.cv2
        stamp = self.get_clock().now().to_msg()
        gray = cv2.cvtColor(self.cam.capture_array(), cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return
        for c, i in zip(corners, ids.ravel()):
            if i != self.marker_id:
                continue
            ok, _, tvec = cv2.solvePnP(self.corners_3d, c.reshape(4, 2), self.K, self.dist,
                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return
            x_right, _, z_forward = tvec.ravel()   # OpenCV camera frame: x right, y down, z forward
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = "base_link"
            msg.point.x = float(CAMERA_X + z_forward)
            msg.point.y = float(-x_right)
            self.pub.publish(msg)
            return


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--marker-id", type=int, default=0)
    p.add_argument("--marker-size", type=float, default=0.08, help="black square side, metres")
    p.add_argument("--calibration", help="npz with K (3x3) and dist from a checkerboard calibration")
    args, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = CameraTarget(args.marker_id, args.marker_size, args.calibration)
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
