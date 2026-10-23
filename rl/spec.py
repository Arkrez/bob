"""What the policy sees and does, shared by training (env.py) and deployment (ros_policy_node.py).

Numpy only, so it also runs in the ROS container and on the Raspberry Pi.
Joint order: leg0 J1,J3,J4, leg1 ..., leg3.
"""

import numpy as np

# Standing pose (rad), chosen so the full action range stays inside the soft joint limits:
# a policy that can push a joint onto a stop learns to lean on it and grinds the servo.
STAND = np.tile([0.0, 0.75, 1.05], 4)
ACTION_SCALE = np.tile([0.35, 0.45, 0.28], 4)    # rad per unit action: yaw, hip, knee
JOINT_LIMIT = 1.5708
# Targets stop short of the servo end stops: pushing into a stop grinds an MG995's gears.
TARGET_LIMIT = 0.9 * JOINT_LIMIT
CONTROL_DT = 0.02                                # 50 Hz
PHYSICS_DT = 0.004                               # s; MuJoCo and Webots both step physics at this

# MG995 servo model, used by MuJoCo (make_mjcf.py) and reproduced in Webots (the bridge):
# torque = KP * (target - angle) - KV * speed, clamped to the stall torque.
SERVO_KP = 8.0            # N m / rad: saturates at stall torque ~7 deg from target
SERVO_KV = 0.15           # N m s / rad: caps free speed near the 5.2 rad/s datasheet value
STALL_TORQUE = 0.94       # N m
JOINT_DAMPING = 0.02      # N m s / rad
JOINT_FRICTION = 0.02     # N m
ARMATURE = 0.003          # kg m^2, reflected rotor + gearbox inertia (MuJoCo only)
FOOT_FRICTION = 0.9
# Gait clock: one stride per GAIT_PERIOD when walking, speeding up to RUN_PERIOD at RUN_SPEED.
# The rhythm follows the commanded speed, so "run" is just a faster /cmd_vel.
GAIT_PERIOD = 0.6                                # s per stride, walking (and standing/turning)
RUN_PERIOD = 0.4                                 # s per stride at full running speed
WALK_SPEED = 0.18                                # m/s: above this the stride starts to quicken
RUN_SPEED = 0.35                                 # m/s: top commanded speed
TURN_PERIOD = 1.0                                # s per stride when turning in place: fewer, bigger steps
OBS_SIZE = 3 + 3 + 3 + 12 + 12 + 2
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


def observation(gravity, gyro, command, joints, last_action, phase):
    """gravity: unit gravity in the body frame; gyro: rad/s body frame; command: vx, vy, wz;
    phase: gait clock (rad), advanced every control step with advance_phase()."""
    return np.concatenate([gravity, np.asarray(gyro) * 0.25, np.asarray(command) * [2.0, 2.0, 1.0],
                           np.asarray(joints) - STAND, last_action,
                           [np.sin(phase), np.cos(phase)]]).astype(np.float32)


def action_to_targets(action):
    action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
    return np.clip(STAND + ACTION_SCALE * action, -TARGET_LIMIT, TARGET_LIMIT)
