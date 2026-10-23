"""What the navigator sees and does, shared by training (nav_env.py), the Webots bridge and the robot.

numpy only. Frames: body frame x forward, y left, metres.
"""

import numpy as np

NAV_DT = 0.2                                   # navigator decision period (s)
GOAL_RADIUS = 0.3                              # m from the body centre: the marker is at the rock's nose
# Command limits: what the walker was trained on (vx, vy m/s, yaw rate rad/s).
CMD_LOW = np.array([-0.12, -0.08, -0.5])
CMD_HIGH = np.array([0.35, 0.08, 0.5])        # up to running speed (spec.RUN_SPEED)
NAV_OBS = 14


def goal_in_body(goal_xy, base_xy, quat):
    """Goal relative to the robot, in its heading frame (x forward, y left). quat is (w, x, y, z)."""
    w, x, y, z = quat
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    d = np.asarray(goal_xy, dtype=float) - np.asarray(base_xy, dtype=float)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])


def nav_observation(estimate, visible, age, last_cmd, yaw_rate, gravity):
    """estimate: last camera detection of the target (body frame, m); age 0..1; last_cmd normalised."""
    dist = float(np.linalg.norm(estimate))
    bearing = np.arctan2(estimate[1], estimate[0]) if dist > 0 else 0.0
    return np.concatenate([np.clip(estimate, -3, 3), [min(dist, 3.0), np.cos(bearing), np.sin(bearing)],
                           [visible, age], last_cmd, [yaw_rate], gravity]).astype(np.float32)


# Scripted navigator (the default): spin to find the target, turn in place to face it, then go
# straight, running when it's far. With the walk/run walker this beats the trained navigators.
SEARCH_TURN = 0.4        # rad/s while the target isn't in view
FACE_TOLERANCE = 0.25    # rad: turn in place until the target is within this of straight ahead
RUN_DISTANCE = 1.0       # m: run when further than this, walk when closer
RUN_CMD, WALK_CMD = 0.35, 0.15
STEER_GAIN, STEER_MAX = 1.0, 0.2


def scripted_command(obs):
    """Velocity command (vx, vy, wz) from a navigator observation (see nav_observation)."""
    x, y, dist, c, s, visible, age = obs[:7]
    if age > 0.5 or dist == 0:   # no sighting for 1.5 s (age is scaled 0..1 over camera.MAX_AGE = 3 s)
        return np.array([0.0, 0.0, SEARCH_TURN if y >= 0 else -SEARCH_TURN])
    bearing = np.arctan2(s, c)
    if abs(bearing) > FACE_TOLERANCE:
        return np.array([0.0, 0.0, 0.5 * np.sign(bearing)])
    vx = RUN_CMD if dist > RUN_DISTANCE else WALK_CMD
    return np.array([vx, 0.0, float(np.clip(STEER_GAIN * bearing, -STEER_MAX, STEER_MAX))])


class ScriptedNavigator:
    """scripted_command plus a search that explores: if a full turn goes by without a
    sighting (the target may be out of camera range), walk forward a little and look again.
    Call it every NAV_DT with the navigator observation."""

    SPIN_BEFORE_EXPLORE = 2 * np.pi / 0.3       # s: about one full turn at the walker's real turn rate
    EXPLORE_TIME = 4.0                           # s of running forward (~1.4 m) before looking again

    def __init__(self):
        self.lost_for = 0.0
        self.exploring_for = 0.0

    def __call__(self, obs):
        searching = obs[6] > 0.5 or obs[2] == 0       # same "lost" test as scripted_command
        if not searching:
            self.lost_for = self.exploring_for = 0.0
            return scripted_command(obs)
        if self.exploring_for > 0:
            self.exploring_for -= NAV_DT
            return np.array([RUN_CMD, 0.0, 0.0])
        self.lost_for += NAV_DT
        if self.lost_for > self.SPIN_BEFORE_EXPLORE:
            self.lost_for, self.exploring_for = 0.0, self.EXPLORE_TIME
        return scripted_command(obs)


def command_to_action(cmd):
    return 2 * (np.asarray(cmd, dtype=float) - CMD_LOW) / (CMD_HIGH - CMD_LOW) - 1


def action_to_command(action):
    a = np.clip(np.asarray(action, dtype=float), -1, 1)
    return CMD_LOW + (a + 1) / 2 * (CMD_HIGH - CMD_LOW)
