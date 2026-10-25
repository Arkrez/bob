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
Webots runs. It also drives the motors with the MuJoCo servo model from rl/spec.py
(torque = KP*(target-angle) - KV*speed, clamped to stall) instead of Webots'
own position controller. Drive it with /cmd_vel (linear.x/y m/s, angular.z rad/s). If the
policy file changes, the bridge swaps the new one in (checked every 5 s of sim time), so it can
follow a training run's best snapshot (start_sim.sh --follow-training).

Navigation mode (also BOB_NAV=scripted, or a trained /path/to/nav.npz): the robot walks to the DEF TARGET marker in
the world. A Pi-camera model (rl/camera.py: field of view, range, noise, latency) decides when
it can see the marker, and the navigator turns that into walker commands every 0.2 s of sim
time, replacing /cmd_vel. Drag the marker in Webots, or publish /goal (PointStamped, world
frame) to move it. /nav/status reports what it is doing.

Lidar: if the robot has a "lidar" device (Rocky-Bob's RPLIDAR C1 on the lid), its 360-degree
scan goes to /scan (sensor_msgs/LaserScan, frame "laser", angles counter-clockwise from -pi) at
~10 Hz, like sllidar_ros2 on the real robot. rl/scan_viewer.py shows it as a live point cloud.

Self-righting: with rl/policies/rocky_recover.npz (or BOB_RECOVER) present, a robot tipped past
60 degrees for 0.3 s hands over to self-righting (rl/recover_spec.py: the learned roll-over, then
a leg-by-leg tidy-up into the standing pose) and back to walking when it's done; navigation pauses. In navigation mode, publish
/flip (std_msgs/Float64, roll in degrees, 0 = 180) to turn him over and watch.
"""

import math
import os
import pathlib
import sys

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, Float64MultiArray, String
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from controller import Robot, Supervisor
import numpy as np


NAMES = tuple(
    f"joint{joint}_leg{leg}"
    for leg in range(4)
    for joint in (1, 3, 4)
)
LIMIT = math.pi / 2
LOOK_TIME = 0.4     # s to stand and look after the goal moves (a camera frame plus its latency, and some)
RELOAD_EVERY = 250   # policy steps (5 s of sim time) between checks for a changed policy file
COMMAND_TIMEOUT = 1.0   # s of sim time without /cmd_vel before the policy is told to stand
# A second robot (the chase demo's runner) runs its own bridge with BOB_NS=bob2: its topics are
# /bob2/..., it walks to the BOB_TARGET marker, and with BOB_KNOWS_GOAL=1 it knows where that is
# without having to see it (the runner flees to spots it picks, often behind it).
NS = os.environ.get("BOB_NS", "").strip("/")
TARGET_DEF = os.environ.get("BOB_TARGET", "TARGET")
KNOWS_GOAL = os.environ.get("BOB_KNOWS_GOAL") == "1"


class PolicyDriver:
    """Runs a numpy walking policy against the bridge's own devices, in sim time."""

    def __init__(self, path, step_ms):
        import numpy as np
        sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "rl"))
        import spec
        from policy import NumpyPolicy

        self.np, self.spec, self.NumpyPolicy = np, spec, NumpyPolicy
        self.path = path
        self.policy = NumpyPolicy(path)
        self.mtime = os.stat(path).st_mtime
        # self-righting (rl/recover_env.py): takes over when he tips past 60 degrees
        self.recover_path = os.environ.get("BOB_RECOVER", "").strip() or str(
            pathlib.Path(__file__).resolve().parents[2] / "rl/policies/rocky_recover.npz")
        self.recover, self.recover_mtime = None, None
        self.load_recover()
        self.mode = "walk"
        self.tipped_for = 0.0
        self.recovery = None
        self.floor = np.zeros(spec.TOF_ZONES)   # floor heights ahead (rl/tof.py), set by the bridge
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

    def load_recover(self):
        try:
            mtime = os.stat(self.recover_path).st_mtime
        except OSError:
            return
        if mtime != self.recover_mtime:
            try:
                self.recover, self.recover_mtime = self.NumpyPolicy(self.recover_path), mtime
                print(f"[bob_webots_bridge] self-righting policy {self.recover_path}", flush=True)
            except Exception as e:
                print(f"[bob_webots_bridge] self-righting policy not loaded: {e}", flush=True)

    def update_mode(self, gravity, gyro, q):
        """Walk, or right himself (rl/recover_spec.py: learned roll-over, then the leg-by-leg tidy-up).
        Switch after 0.3 s tipped past 60 deg; back to walking once the tidy-up has finished."""
        dt = self.spec.CONTROL_DT
        if self.mode == "walk":
            self.tipped_for = self.tipped_for + dt if -gravity[2] < 0.5 else 0.0
            if self.tipped_for > 0.3 and self.recover is not None:
                from recover_spec import Recovery
                self.mode, self.recovery = "recover", Recovery(self.recover)
                self.recovery.reset(q)
                print("[bob_webots_bridge] tipped over: self-righting", flush=True)
        elif self.recovery.done():
            self.mode, self.tipped_for = "walk", 0.0
            self.phase, self.last_action = 0.0, self.np.zeros(12)
            print("[bob_webots_bridge] back on his feet: walking", flush=True)

    def reload_if_changed(self):
        """Swap in the policy file's new contents (e.g. keep_best.py's best.npz improving during training)."""
        try:
            mtime = os.stat(self.path).st_mtime
            if mtime != self.mtime:
                self.policy, self.mtime = self.NumpyPolicy(self.path), mtime
                print(f"[bob_webots_bridge] reloaded walker {self.path}", flush=True)
        except Exception as e:   # mid-write or missing: keep the current walker, try again later
            print(f"[bob_webots_bridge] walker reload skipped: {e}", flush=True)

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
            floor = self.floor if self.policy.inputs > spec.OBS_SIZE else None   # floor-sensing walkers
            obs = spec.observation(spec.projected_gravity((w, x, y, z)), gyro, command, q,
                                   self.last_action, self.phase, floor)
            self.phase = spec.advance_phase(self.phase, command)
            if self.steps % (self.decimation * RELOAD_EVERY) == 0:
                self.reload_if_changed()
                self.load_recover()
            gravity = spec.projected_gravity((w, x, y, z))
            self.update_mode(gravity, gyro, q)
            if self.mode == "recover":
                self.targets = self.recovery.step(gravity, np.asarray(gyro), q)
            else:
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

        import lidar

        self.np, self.nav, self.lidar = np, nav_spec, lidar
        # "scripted" = the hand-written turn-then-go navigator (with lidar obstacle avoidance when the
        # robot has a lidar); otherwise a trained .npz, camera-only or lidar (told apart by input size)
        # "planner" (memory + free-path steering, the default with a lidar), "scripted", or a trained .npz
        hand_written = path in ("scripted", "planner")
        self.policy = (nav_spec.PlannerNavigator() if path == "planner" else
                       nav_spec.ScriptedNavigator() if path == "scripted" else None)
        self.trained = None if hand_written else NumpyPolicy(path)
        self.path, self.NumpyPolicy = path, NumpyPolicy
        self.mtime = None if hand_written else os.stat(path).st_mtime
        self.decisions = 0
        self.bins = None            # latest lidar scan (lidar.N_BINS ranges), set by the bridge
        self.camera = camera.CameraModel(np.random.default_rng())
        self.robot = supervisor.getSelf()
        self.target = supervisor.getFromDef(TARGET_DEF)
        if self.target is None:
            raise RuntimeError(f"BOB_NAV needs a 'DEF {TARGET_DEF}' node in the world (rocky_bob.wbt has one)")
        self.next_decision = 0.0
        self.look_until = 0.0
        self.moved = False
        self.cmd_norm = np.zeros(3)
        self.command = np.zeros(3)
        self.status = "starting"

    def move_target(self, x, y):
        self.target.getField("translation").setSFVec3f([x, y, 0.0])
        self.camera.reset()
        if hasattr(self.policy, "target"):   # the planner's memory of the old target is now wrong
            self.policy.target, self.policy.seen = None, set()
        self.moved = True   # stand and look first: with no sighting yet he'd spin off at search speed

    def relative_target(self):
        np = self.np
        p = self.robot.getPosition()
        R = np.array(self.robot.getOrientation()).reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        quat = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
        return self.nav.goal_in_body(self.target.getPosition()[:2], p[:2], quat)

    def blocked(self, rel):
        """Is the target hidden behind a wall? Judged from the lidar, as in training (nav_env.py)."""
        if self.bins is None:
            return False
        np = self.np
        k = int(np.argmin(np.abs(np.angle(np.exp(1j * (self.lidar.BIN_ANGLES - np.arctan2(rel[1], rel[0])))))))
        return bool(self.bins[k] < np.linalg.norm(rel) - 0.25)

    def update(self, now, gyro_z, gravity):
        """Call every sim step; returns the walker command (vx, vy, wz)."""
        np, nav = self.np, self.nav
        rel = self.relative_target()
        self.camera.update(rel, now, self.blocked(rel))
        if KNOWS_GOAL:
            self.camera.estimate, self.camera.seen_at = np.asarray(rel, float).copy(), now
        if self.moved:
            self.moved, self.look_until = False, now + LOOK_TIME
        if now < self.look_until and self.camera.reading(now)[1] == 0:
            self.command, self.status = np.zeros(3), "looking for the new target"
            return self.command
        if now >= self.next_decision:
            self.next_decision = now + nav.NAV_DT
            if np.linalg.norm(rel) < nav.GOAL_RADIUS:
                self.command, self.cmd_norm = np.zeros(3), 2 * (0 - nav.CMD_LOW) / (nav.CMD_HIGH - nav.CMD_LOW) - 1
                self.status = "reached target"
            else:
                estimate, visible, age = self.camera.reading(now)
                wants_lidar = self.trained is None or self.trained.mean.shape[0] == nav.NAV_OBS_LIDAR
                if wants_lidar and self.path != "scripted" and self.bins is None:
                    self.command, self.status = np.zeros(3), "waiting for the first lidar scan"
                    return self.command
                obs = nav.nav_observation(estimate, visible, age, self.cmd_norm, gyro_z, gravity,
                                          lidar=self.bins if wants_lidar else None,
                                          floor=getattr(self, "floor", None) if self.path == "planner" else None)
                self.decisions += 1
                if self.trained is not None and self.decisions % 25 == 0:   # every 5 s: a newer navigator?
                    try:
                        mtime = os.stat(self.path).st_mtime
                        if mtime != self.mtime:
                            self.trained, self.mtime = self.NumpyPolicy(self.path), mtime
                            print(f"[bob_webots_bridge] reloaded navigator {self.path}", flush=True)
                    except Exception as e:
                        print(f"[bob_webots_bridge] navigator reload skipped: {e}", flush=True)
                if self.trained is not None and wants_lidar:
                    # lidar navigators drive while the target is in sight, the scripted search finds it
                    if getattr(self, "hybrid", None) is None or self.hybrid.policy is not self.trained:
                        self.hybrid = nav.HybridNavigator(self.trained)
                    self.command = self.hybrid(obs)
                    self.cmd_norm = nav.command_to_action(self.command)
                elif self.trained is not None:
                    self.cmd_norm = self.trained(obs)
                    self.command = nav.action_to_command(self.cmd_norm)
                else:
                    self.command = self.policy(obs)
                    self.cmd_norm = nav.command_to_action(self.command)
                    if hasattr(self.policy, "status"):   # the planner says what it's doing
                        self.status = f"{self.policy.status}; cmd vx={self.command[0]:+.2f} wz={self.command[2]:+.2f}"
                        return self.command
                self.status = (f"{'tracking' if visible else 'searching'} target "
                               f"{np.linalg.norm(rel):.2f} m away; cmd vx={self.command[0]:+.2f} "
                               f"vy={self.command[1]:+.2f} wz={self.command[2]:+.2f}")
        return self.command


class BobBridge(Node):
    def __init__(self, robot):
        super().__init__("bob_webots_bridge", **({"namespace": "/" + NS} if NS else {}))
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

        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        # Simulation time, for anything that must not depend on how fast Webots happens to run.
        self.clock_publisher = self.create_publisher(Clock, "clock", 10)

        # Optional IMU (Rocky-Bob has one; the older CAD model does not).
        self.imu = robot.getDevice("imu")
        self.gyro = robot.getDevice("gyro")
        self.imu_publisher = None
        if self.imu is not None and self.gyro is not None:
            self.imu.enable(self.step_ms)
            self.gyro.enable(self.step_ms)
            self.imu_publisher = self.create_publisher(Imu, "imu", 10)

        # Optional simulation-only ground truth position, for judging gaits.
        self.gps = robot.getDevice("gps")
        self.gps_publisher = None
        if self.gps is not None:
            self.gps.enable(self.step_ms)
            self.gps_publisher = self.create_publisher(PointStamped, "sim/position", 10)

        # Optional front camera (Rocky-Bob's Pi camera). Webots only renders a camera that a
        # controller has enabled; frames go to /camera/image_raw at ~10 Hz.
        self.camera = robot.getDevice("camera")
        self.camera_publisher = None
        if self.camera is not None:
            period = self.step_ms * max(1, round(100 / self.step_ms))
            self.camera.enable(period)
            self.camera_period = period / 1000.0
            self.camera_next = 0.0
            self.camera_publisher = self.create_publisher(Image, "camera/image_raw", 2)

        # floor depth sensor in the nose (rl/tof.py): 8 x 8 distances -> floor heights ahead, ~15 Hz
        self.tof = robot.getDevice("tof")
        self.tof_publisher = None
        if self.tof is not None:
            self.tof.enable(self.step_ms * max(1, round(66 / self.step_ms)))
            self.tof_publisher = self.create_publisher(Float64MultiArray, "tof/heights", 5)
            self.tof_points_publisher = self.create_publisher(Float64MultiArray, "tof/points", 5)

        self.lidar = robot.getDevice("lidar")
        self.scan_publisher = None
        if self.lidar is not None:
            period = self.step_ms * max(1, round(100 / self.step_ms))
            self.lidar.enable(period)
            self.lidar.enablePointCloud()   # View > Optional Rendering > Show Lidar Point Cloud draws it in Webots
            self.scan_period = period / 1000.0
            self.scan_next = 0.0
            self.scan_publisher = self.create_publisher(LaserScan, "scan", 5)

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
                self.create_subscription(PointStamped, "goal", self.goal_callback, 10)
                self.create_subscription(Float64, "flip", self.flip_callback, 10)
                self.create_subscription(PointStamped, "place", self.place_callback, 10)
                # manual override: /cmd_vel drives him for COMMAND_TIMEOUT s past the last message,
                # instead of the navigator (the climbing demos walk him straight at a tall step)
                self.manual_until = -1.0
                # track: move the target without the 'new target' reset (the chase demo moves it every step)
                self.create_subscription(PointStamped, "track", self.track_callback, 10)
                # prize: move the chase demo's green PRIZE marker, if the world has one
                self.prize = robot.getFromDef("PRIZE")
                if self.prize is not None:
                    self.create_subscription(PointStamped, "prize", self.prize_callback, 10)
                self.create_subscription(Twist, "cmd_vel", self.manual_callback, 10)
                self.nav_status = self.create_publisher(String, "nav/status", 10)
                self.target_pub = self.create_publisher(PointStamped, "sim/target", 10)
                if not NS:   # one bridge handles the keyboard and the labels
                    self.setup_demo_keys(robot)
                self.get_logger().info(f"navigation mode: {nav_path} walks to the TARGET marker (camera model)")
            else:
                self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
            self.get_logger().info(
                f"Bob Webots bridge ready: 12 actuators driven by {policy_path} in lockstep with sim time "
                f"(policy every {self.policy.decimation} x {self.step_ms} ms), {self.policy.spec.SERVO_NAME} servo model"
            )
        else:
            self.subscription = self.create_subscription(
                Float64MultiArray, "leg_position_controller/commands",
                self.command_callback, 10
            )
            self.get_logger().info(
                "Bob Webots bridge ready: 12 actuators, "
                "/leg_position_controller/commands -> Webots motors"
            )
        self.target = None
        self.have_command = False

    def flip_callback(self, msg):
        """Turn him over (roll by `data` degrees, default 180) to test self-righting."""
        node = self.robot.getSelf()
        p = node.getPosition()
        roll = math.radians(msg.data or 180.0)
        node.getField("translation").setSFVec3f([p[0], p[1], p[2] + 0.25])
        node.getField("rotation").setSFRotation([1, 0, 0, roll])
        node.resetPhysics()
        self.get_logger().info(f"flipped {math.degrees(roll):.0f} deg")

    def place_callback(self, msg):
        """Put him upright at world (x, y), heading point.z (rad, 0 = +x), and clear the navigator's
        memory (for tests)."""
        node = self.robot.getSelf()
        node.getField("translation").setSFVec3f([msg.point.x, msg.point.y, 0.25])
        node.getField("rotation").setSFRotation([0, 0, 1, msg.point.z])
        node.resetPhysics()
        nav = self.nav
        if nav is not None and nav.path in ("planner", "scripted"):
            import nav_spec
            nav.policy = nav_spec.PlannerNavigator() if nav.path == "planner" else nav_spec.ScriptedNavigator()
        self.get_logger().info(f"placed at ({msg.point.x:.2f}, {msg.point.y:.2f})")

    def goal_callback(self, msg):
        self.nav.move_target(msg.point.x, msg.point.y)

    def prize_callback(self, msg):
        self.prize.getField("translation").setSFVec3f([msg.point.x, msg.point.y, 0.0])

    def track_callback(self, msg):
        self.nav.target.getField("translation").setSFVec3f([msg.point.x, msg.point.y, 0.0])

    def manual_callback(self, msg):
        self.policy.set_command(msg, self.robot.getTime())
        self.manual_until = self.robot.getTime() + COMMAND_TIMEOUT

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

    # ---------------------------------------------------------------- demos from the Webots window
    DEMO_KEYS = {"1": "tour", "2": "stairs", "3": "flip", "4": "walls", "5": "tall", "6": "ledge", "7": "chase", "0": "all"}

    def setup_demo_keys(self, robot):
        """Click the 3D view, then press 1-4 (or 0 for all) to run a demo (rl/demo.py), S to stop it."""
        self.keyboard = robot.getKeyboard()
        self.keyboard.enable(self.step_ms)
        self.demo_proc, self.demo_name, self.label_next = None, "", 0.0
        self.show_label(0, "Demos: click here, then press  1 tour  2 stairs  3 flip  4 walls  5 tall  6 ledge  7 chase  0 all  S stop", 0.01, 0.01)

    def show_label(self, i, text, x, y):
        self.robot.setLabel(i, text, x, y, 0.07, 0xFFFFFF, 0.2, "Arial")

    def poll_demo_keys(self):
        key = self.keyboard.getKey()
        if key > 0:
            c = chr(key & 0xFF).upper()
            running = self.demo_proc is not None and self.demo_proc.poll() is None
            if c == "S" and running:
                self.demo_proc.terminate()
                self.demo_name = ""
            elif c in self.DEMO_KEYS and not running:
                import subprocess
                name = self.DEMO_KEYS[c]
                demo = str(pathlib.Path(__file__).resolve().parents[2] / "rl/demo.py")
                self.demo_proc = subprocess.Popen([sys.executable, demo] + ([] if name == "all" else [name]))
                self.demo_name = name
        if self.robot.getTime() >= self.label_next:
            self.label_next = self.robot.getTime() + 0.5
            if self.demo_proc is not None and self.demo_proc.poll() is not None:
                self.demo_name = ""
            now = f"demo: {self.demo_name}   " if self.demo_name else ""
            status = "righting himself" if self.policy is not None and self.policy.mode == "recover" else self.nav.status
            self.show_label(1, f"{now}{status.split(';')[0]}", 0.01, 0.06)

    def update(self):
        positions = [sensor.getValue() for sensor in self.sensors]
        if self.policy is not None:
            if self.nav is not None:
                q = self.imu.getQuaternion()
                gravity = self.policy.spec.projected_gravity((q[3], q[0], q[1], q[2]))
                cmd = self.nav.update(self.robot.getTime(), self.gyro.getValues()[2], gravity)
                manual = self.robot.getTime() < self.manual_until
                if manual:
                    self.nav.status = "manual drive (/cmd_vel)"
                else:
                    self.policy.command, self.policy.command_time = cmd, self.robot.getTime()
                if self.policy.steps % 50 == 0:
                    status = "righting himself" if self.policy.mode == "recover" else self.nav.status
                    self.nav_status.publish(String(data=status))
                    t = PointStamped()
                    t.header.frame_id = "world"
                    t.point.x, t.point.y, _ = self.nav.target.getPosition()
                    self.target_pub.publish(t)
                if hasattr(self, "keyboard"):
                    self.poll_demo_keys()
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

        if self.tof_publisher is not None and self.imu is not None:
            try:
                ranges = self.tof.getRangeImage()
            except (ValueError, SystemError):
                ranges = None
            if ranges:
                import tof as tofm
                qx, qy, qz, qw = self.imu.getQuaternion()
                g = self.policy.spec.projected_gravity((qw, qx, qy, qz)) if self.policy is not None else None
                if g is not None:
                    heights = tofm.heights(tofm.from_depth(ranges), g)   # RangeFinder gives depth, not distance
                    self.policy.floor = heights
                    if getattr(self, "nav", None) is not None:
                        self.nav.floor = tofm.encode(tofm.from_depth(ranges))   # the planner wants distances
                    if self.policy.steps % 10 == 0:
                        self.tof_publisher.publish(Float64MultiArray(data=[float(h) for h in heights]))
                        # where each zone hit (body frame x, y) and the floor height there, for scan_viewer.py
                        pts = tofm.points(tofm.from_depth(ranges))
                        flat = [float(v) for xy, h in zip(pts[:, :2], heights) if np.isfinite(xy[0]) for v in (*xy, h)]
                        self.tof_points_publisher.publish(Float64MultiArray(data=flat))

        if self.scan_publisher is not None and self.robot.getTime() >= self.scan_next:
            self.scan_next = self.robot.getTime() + self.scan_period
            try:
                ranges = self.lidar.getRangeImage()
            except (ValueError, SystemError):   # no scan rendered yet (the first period after enabling)
                ranges = None
            if ranges:
                # Webots sweeps left to right (clockwise from above); LaserScan wants counter-clockwise
                scan = LaserScan()
                scan.header.stamp = msg.header.stamp
                scan.header.frame_id = "laser"
                fov = self.lidar.getFov()
                scan.angle_increment = fov / len(ranges)
                scan.angle_min = -fov / 2
                scan.angle_max = scan.angle_min + scan.angle_increment * (len(ranges) - 1)
                scan.scan_time = self.scan_period
                scan.time_increment = self.scan_period / len(ranges)
                scan.range_min, scan.range_max = self.lidar.getMinRange(), self.lidar.getMaxRange()
                scan.ranges = [float(r) for r in reversed(ranges)]
                self.scan_publisher.publish(scan)
                if getattr(self, "nav", None) is not None:
                    self.nav.bins = self.nav.lidar.scan_bins(scan.ranges, scan.angle_min, scan.angle_increment)

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
