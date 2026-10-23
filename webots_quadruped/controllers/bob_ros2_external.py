#!/usr/bin/env python3
"""ROS 2 -> Webots external-controller bridge for Bob.

Consumes the existing quadruped_control JointGroupPositionController topic:
  /leg_position_controller/commands, Float64MultiArray, 12 radians
Ordering: leg0 J1/J3/J4, leg1, leg2, leg3.
Webots is the sole motor backend in this simulation mode; no ROS2-control
controller_manager is required.

Policy mode (BOB_POLICY=/path/to/policy.npz): the bridge runs a trained walking
policy itself, in lockstep with simulation time, so it sees exactly what it saw in
MuJoCo training: a 50 Hz control step and gait clock in sim seconds, however fast
Webots runs. It also drives the motors with the MuJoCo MG995 servo model
(torque = KP*(target-angle) - KV*speed, clamped to stall) instead of Webots'
own position controller. Drive it with /cmd_vel (linear.x/y m/s, angular.z rad/s).

Navigation mode (also BOB_NAV=scripted, or a trained /path/to/nav.npz): the robot walks to the DEF TARGET marker in
the world. A Pi-camera model (rl/camera.py: field of view, range, noise, latency) decides when
it can see the marker, and the navigator turns that into walker commands every 0.2 s of sim
time, replacing /cmd_vel. Drag the marker in Webots, or publish /goal (PointStamped, world
frame) to move it. /nav/status reports what it is doing.
"""

import math
import os
import pathlib
import sys

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Image, Imu, JointState
from controller import Robot, Supervisor


NAMES = tuple(
    f"joint{joint}_leg{leg}"
    for leg in range(4)
    for joint in (1, 3, 4)
)
LIMIT = math.pi / 2
COMMAND_TIMEOUT = 1.0   # s of sim time without /cmd_vel before the policy is told to stand


class PolicyDriver:
    """Runs a numpy walking policy against the bridge's own devices, in sim time."""

    def __init__(self, path, step_ms):
        import numpy as np
        sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "rl"))
        import spec
        from policy import NumpyPolicy

        self.np, self.spec = np, spec
        self.policy = NumpyPolicy(path)
        self.dt = step_ms / 1000.0
        self.decimation = round(spec.CONTROL_DT / self.dt)
        if abs(self.decimation * self.dt - spec.CONTROL_DT) > 1e-9:
            raise RuntimeError(f"basicTimeStep {step_ms} ms does not divide the {spec.CONTROL_DT * 1000:.0f} ms "
                               "policy step; use 2, 4, 5, 10 or 20 ms")
        self.steps = 0
        self.phase = 0.0          # gait clock, advanced at the command's stride rhythm (spec.gait_period)
        self.last_action = np.zeros(12)
        self.targets = None
        self.prev_q = None
        self.command = np.zeros(3)
        self.command_time = -1e9

    def set_command(self, msg, now):
        self.command = self.np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self.command_time = now

    def torques(self, q, quat_xyzw, gyro, now):
        np, spec = self.np, self.spec
        q = np.asarray(q)
        qd = np.zeros(12) if self.prev_q is None else (q - self.prev_q) / self.dt
        self.prev_q = q
        if self.steps % self.decimation == 0:
            x, y, z, w = quat_xyzw
            command = self.command if now - self.command_time < COMMAND_TIMEOUT else np.zeros(3)
            obs = spec.observation(spec.projected_gravity((w, x, y, z)), gyro, command, q,
                                   self.last_action, self.phase)
            self.phase = spec.advance_phase(self.phase, command)
            action = self.policy(obs)
            self.targets = spec.action_to_targets(action)
            self.last_action = action
        self.steps += 1
        tau = spec.SERVO_KP * (self.targets - q) - spec.SERVO_KV * qd
        return np.clip(tau, -spec.STALL_TORQUE, spec.STALL_TORQUE)


class NavDriver:
    """Camera model + navigator: turns the target marker's position into walker commands."""

    def __init__(self, path, supervisor):
        import numpy as np
        import camera
        import nav_spec
        from policy import NumpyPolicy

        self.np, self.nav = np, nav_spec
        # "scripted" = the hand-written turn-then-go navigator; otherwise a trained .npz
        self.policy = nav_spec.scripted_command if path == "scripted" else None
        self.trained = None if path == "scripted" else NumpyPolicy(path)
        self.camera = camera.CameraModel(np.random.default_rng())
        self.robot = supervisor.getSelf()
        self.target = supervisor.getFromDef("TARGET")
        if self.target is None:
            raise RuntimeError("BOB_NAV needs a 'DEF TARGET' node in the world (rocky_bob.wbt has one)")
        self.next_decision = 0.0
        self.cmd_norm = np.zeros(3)
        self.command = np.zeros(3)
        self.status = "starting"

    def move_target(self, x, y):
        self.target.getField("translation").setSFVec3f([x, y, 0.0])
        self.camera.reset()

    def relative_target(self):
        np = self.np
        p = self.robot.getPosition()
        R = np.array(self.robot.getOrientation()).reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        quat = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
        return self.nav.goal_in_body(self.target.getPosition()[:2], p[:2], quat)

    def update(self, now, gyro_z, gravity):
        """Call every sim step; returns the walker command (vx, vy, wz)."""
        np, nav = self.np, self.nav
        rel = self.relative_target()
        self.camera.update(rel, now)
        if now >= self.next_decision:
            self.next_decision = now + nav.NAV_DT
            if np.linalg.norm(rel) < nav.GOAL_RADIUS:
                self.command, self.cmd_norm = np.zeros(3), 2 * (0 - nav.CMD_LOW) / (nav.CMD_HIGH - nav.CMD_LOW) - 1
                self.status = "reached target"
            else:
                estimate, visible, age = self.camera.reading(now)
                obs = nav.nav_observation(estimate, visible, age, self.cmd_norm, gyro_z, gravity)
                if self.trained is not None:
                    self.cmd_norm = self.trained(obs)
                    self.command = nav.action_to_command(self.cmd_norm)
                else:
                    self.command = self.policy(obs)
                    self.cmd_norm = nav.command_to_action(self.command)
                self.status = (f"{'tracking' if visible else 'searching'} target "
                               f"{np.linalg.norm(rel):.2f} m away; cmd vx={self.command[0]:+.2f} "
                               f"vy={self.command[1]:+.2f} wz={self.command[2]:+.2f}")
        return self.command


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
        # Simulation time, for anything that must not depend on how fast Webots happens to run.
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)

        # Optional IMU (Rocky-Bob has one; the older CAD model does not).
        self.imu = robot.getDevice("imu")
        self.gyro = robot.getDevice("gyro")
        self.imu_publisher = None
        if self.imu is not None and self.gyro is not None:
            self.imu.enable(self.step_ms)
            self.gyro.enable(self.step_ms)
            self.imu_publisher = self.create_publisher(Imu, "/imu", 10)

        # Optional simulation-only ground truth position, for judging gaits.
        self.gps = robot.getDevice("gps")
        self.gps_publisher = None
        if self.gps is not None:
            self.gps.enable(self.step_ms)
            self.gps_publisher = self.create_publisher(PointStamped, "/sim/position", 10)

        # Optional front camera (Rocky-Bob's Pi camera). Webots only renders a camera that a
        # controller has enabled; frames go to /camera/image_raw at ~10 Hz.
        self.camera = robot.getDevice("camera")
        self.camera_publisher = None
        if self.camera is not None:
            period = self.step_ms * max(1, round(100 / self.step_ms))
            self.camera.enable(period)
            self.camera_period = period / 1000.0
            self.camera_next = 0.0
            self.camera_publisher = self.create_publisher(Image, "/camera/image_raw", 2)

        self.policy = None
        policy_path = os.environ.get("BOB_POLICY", "").strip()
        if policy_path:
            if self.imu_publisher is None:
                raise RuntimeError("BOB_POLICY needs a robot with 'imu' and 'gyro' devices (Rocky-Bob)")
            self.policy = PolicyDriver(policy_path, self.step_ms)
            self.nav = None
            nav_path = os.environ.get("BOB_NAV", "").strip()
            if nav_path:
                self.nav = NavDriver(nav_path, robot)
                self.create_subscription(PointStamped, "/goal", self.goal_callback, 10)
                self.nav_status = self.create_publisher(String, "/nav/status", 10)
                self.get_logger().info(f"navigation mode: {nav_path} walks to the TARGET marker (camera model)")
            else:
                self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
            self.get_logger().info(
                f"Bob Webots bridge ready: 12 actuators driven by {policy_path} in lockstep with sim time "
                f"(policy every {self.policy.decimation} x {self.step_ms} ms), MG995 servo model; send /cmd_vel"
            )
        else:
            self.subscription = self.create_subscription(
                Float64MultiArray, "/leg_position_controller/commands",
                self.command_callback, 10
            )
            self.get_logger().info(
                "Bob Webots bridge ready: 12 actuators, "
                "/leg_position_controller/commands -> Webots motors"
            )
        self.target = None
        self.have_command = False

    def goal_callback(self, msg):
        self.nav.move_target(msg.point.x, msg.point.y)

    def cmd_vel_callback(self, msg):
        self.policy.set_command(msg, self.robot.getTime())

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
        positions = [sensor.getValue() for sensor in self.sensors]
        if self.policy is not None:
            if self.nav is not None:
                q = self.imu.getQuaternion()
                gravity = self.policy.spec.projected_gravity((q[3], q[0], q[1], q[2]))
                cmd = self.nav.update(self.robot.getTime(), self.gyro.getValues()[2], gravity)
                self.policy.command, self.policy.command_time = cmd, self.robot.getTime()
                if self.policy.steps % 50 == 0:
                    self.nav_status.publish(String(data=self.nav.status))
            tau = self.policy.torques(positions, self.imu.getQuaternion(), self.gyro.getValues(),
                                      self.robot.getTime())
            for motor, t in zip(self.motors, tau):
                motor.setTorque(float(t))
        elif self.have_command:
            for motor, target in zip(self.motors, self.target):
                motor.setPosition(target)
        else:
            # Hold the CAD starting pose until the first ROS command arrives.
            for motor, position in zip(self.motors, positions):
                motor.setPosition(position)
        sim = Clock()
        t = self.robot.getTime()
        sim.clock.sec, sim.clock.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1_000_000_000
        self.clock_publisher.publish(sim)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(NAMES)
        msg.position = positions
        self.publisher.publish(msg)

        if self.imu_publisher is not None:
            imu = Imu()
            imu.header.stamp = msg.header.stamp
            imu.header.frame_id = "base_link"
            x, y, z, w = self.imu.getQuaternion()
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = x, y, z, w
            gx, gy, gz = self.gyro.getValues()
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gx, gy, gz
            self.imu_publisher.publish(imu)

        if self.camera_publisher is not None and self.robot.getTime() >= self.camera_next:
            self.camera_next = self.robot.getTime() + self.camera_period
            try:
                frame = self.camera.getImage()
            except ValueError:   # no frame rendered yet (the first camera period after enabling)
                frame = None
            if frame:
                img = Image()
                img.header.stamp = msg.header.stamp
                img.header.frame_id = "camera"
                img.width, img.height = self.camera.getWidth(), self.camera.getHeight()
                img.encoding, img.step = "bgra8", 4 * img.width
                img.data = frame
                self.camera_publisher.publish(img)

        if self.gps_publisher is not None:
            p = PointStamped()
            p.header.stamp = msg.header.stamp
            p.header.frame_id = "world"
            p.point.x, p.point.y, p.point.z = self.gps.getValues()
            self.gps_publisher.publish(p)


def main():
    rclpy.init()
    node = None
    try:
        # navigation reads the target marker's position, which needs supervisor rights
        robot = Supervisor() if os.environ.get("BOB_NAV", "").strip() else Robot()
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
