"""What the policy sees and does, shared by training (env.py) and deployment (ros_policy_node.py).

Numpy only, so it also runs in the ROS container and on the Raspberry Pi.
Joint order: leg0 J1,J3,J4, leg1 ..., leg3.
"""

import numpy as np

# Standing pose (rad), chosen so the full action range stays inside the soft joint limits:
# a policy that can push a joint onto a stop learns to lean on it and grinds the servo.
STAND = np.tile([0.0, 0.75, 1.05], 4)
# rad per unit action: yaw, hip, knee. Yaw was 0.35: wide hip swings turn the body further per step
# (rescale_actions.py converts walkers trained with the old scale).
# A unit of action reaches TARGET_LIMIT from the standing pose: the whole range the servos allow (less
# the 10 % guard), nothing narrower. (Walkers were trained on narrower bands, 0.35-0.60 / 0.45 / 0.28 rad,
# which capped foot lift at 4.6 cm; rescale_actions.py converts them.) Legs collide with the body and
# each other in the sim, so the rest is up to the robot's shape.
_FULL = 0.9 * 1.5708   # = TARGET_LIMIT
ACTION_SCALE = np.tile([_FULL, _FULL + 0.75, _FULL + 1.05], 4)   # yaw, hip, knee (STAND is 0, 0.75, 1.05)
JOINT_LIMIT = 1.5708
# Targets stop short of the servo end stops: pushing into a stop grinds a hobby servo's gears.
TARGET_LIMIT = 0.9 * JOINT_LIMIT
CONTROL_DT = 0.02                                # 50 Hz
PHYSICS_DT = 0.004                               # s; MuJoCo and Webots both step physics at this

# Servo model, used by MuJoCo (make_mjcf.py), the Webots bridge and the URDF/PROTO (build.py):
# torque = KP * (target - angle) - KV * speed, clamped to the stall torque.
# HobbyPark 25 kg coreless digital servo at 7.4 V (2S LiPo): 25 kg.cm, ~0.13 s/60 deg.
# Seller specs vary; the earlier MG995 was KP 8, KV 0.15, stall 0.94 N m, ~5.2 rad/s.
SERVO_NAME = "HobbyPark 25 kg coreless"
STALL_TORQUE = 2.45       # N m (25 kg.cm)
SERVO_FREE_SPEED = 8.0    # rad/s (0.13 s per 60 deg)
SERVO_KP = 20.0           # N m / rad: saturates at stall torque ~7 deg from target
SERVO_KV = STALL_TORQUE / SERVO_FREE_SPEED   # N m s / rad: no torque left at free speed
JOINT_DAMPING = 0.02      # N m s / rad
JOINT_FRICTION = 0.02     # N m
ARMATURE = 0.003          # kg m^2, reflected rotor + gearbox inertia (MuJoCo only)
FOOT_FRICTION = 0.9
# Gait clock: one stride per GAIT_PERIOD when walking, speeding up to RUN_PERIOD at RUN_SPEED.
# The rhythm follows the commanded speed, so "run" is just a faster /cmd_vel.
GAIT_PERIOD = 0.6                                # s per stride, walking (and standing/turning)
RUN_PERIOD = 0.33                                # s per stride at full running speed
WALK_SPEED = 0.18                                # m/s: above this the stride starts to quicken
RUN_SPEED = 0.8                                  # m/s: top commanded speed (above what the servos allow)
TURN_PERIOD = 0.8                                # s per stride when turning in place: big, unhurried steps
OBS_SIZE = 3 + 3 + 3 + 12 + 12 + 2
TOF_ZONES = 64                                   # floor depth sensor (tof.py): heights of the floor ahead
OBS_SIZE_TOF = OBS_SIZE + TOF_ZONES             # walkers that see the floor; older ones use the first OBS_SIZE
JOINT_NAMES = [f"joint{j}_leg{l}" for l in range(4) for j in (1, 3, 4)]


def quat_matrix(q):
    """Rotation matrix (body -> world) of unit quaternion q (w, x, y, z)."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_rotate_inverse(q, v):
    """Rotate world vector v into the frame of unit quaternion q (w, x, y, z)."""
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return R.T @ v


def projected_gravity(q):
    return quat_rotate_inverse(q, np.array([0.0, 0.0, -1.0]))


def gait_period(command):
    """Stride period (s) for a command: walking rhythm up to WALK_SPEED, quickening to RUN_PERIOD."""
    speed = float(np.hypot(command[0], command[1]))
    if speed < 0.02 and abs(command[2]) > 0.1:   # turning in place
        return TURN_PERIOD
    run = np.clip((speed - WALK_SPEED) / (RUN_SPEED - WALK_SPEED), 0.0, 1.0)
    return GAIT_PERIOD + run * (RUN_PERIOD - GAIT_PERIOD)


def advance_phase(phase, command, dt=CONTROL_DT):
    """Gait clock phase (rad) after dt seconds at the rhythm for this command."""
    return (phase + 2 * np.pi * dt / gait_period(command)) % (2 * np.pi)


def observation(gravity, gyro, command, joints, last_action, phase, floor=None):
    """gravity: unit gravity in the body frame; gyro: rad/s body frame; command: vx, vy, wz;
    phase: gait clock (rad), advanced every control step with advance_phase().
    floor: optional 64 floor heights ahead (m, tof.heights), appended scaled x10."""
    obs = [gravity, np.asarray(gyro) * 0.25, np.asarray(command) * [2.0, 2.0, 1.0],
           np.asarray(joints) - STAND, last_action, [np.sin(phase), np.cos(phase)]]
    if floor is not None:
        obs.append(np.asarray(floor) * 10.0)
    return np.concatenate(obs).astype(np.float32)


RECOVERY_MAX_RATE = 3.0   # rad/s: self-righting joint targets move no faster than this (smooth, no flinging)


def slew(previous, target, dt=CONTROL_DT, rate=RECOVERY_MAX_RATE):
    """Move joint targets toward `target` by at most rate * dt per step."""
    step = rate * dt
    return np.asarray(previous) + np.clip(np.asarray(target) - previous, -step, step)


def recovery_targets(action):
    """Self-righting (recover_env.py) uses nearly the whole joint range: rolling a rock over needs
    big leg swings, far outside the walking band round STAND. Same limits as walking (TARGET_LIMIT)."""
    return np.clip(np.asarray(action, dtype=float), -1.0, 1.0) * TARGET_LIMIT


def action_to_targets(action):
    action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
    return np.clip(STAND + ACTION_SCALE * action, -TARGET_LIMIT, TARGET_LIMIT)
