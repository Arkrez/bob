"""Self-righting: get back on his feet after a fall, without leaning on the lidar.

Each episode drops Bob on his back (half the time), on his side, or at a random angle, from a
random leg pose, and lets him settle. Then the recovery policy has RECOVER_SECONDS to roll over
and stand up. It sees what the walker sees (spec.observation: gravity, gyro, joints, last action;
the command and gait clock are zero) and sets joint targets over nearly the whole range
(spec.recovery_targets), because rolling a rock over takes far bigger swings than walking.

Upside down he rests on the RPLIDAR C1, the highest point of the robot. Rewards:
  + upright (body up-vector), then standing height and the standing pose once upright,
    and a bonus for every second spent fully recovered
  - every second the lidar is on the ground (more than being upright earns), force on it (per
    newton), more for hard knocks, and a cost per new touch: get off it quickly and gently, and
    never use it as a lever
  - body spin (steeply past SPIN_OK: a slow, controlled roll, not a fling), hard landings of the
    rock on the ground, joint speed, servo torque and jerky actions (gentle on the servos)
Joint targets also move no faster than spec.RECOVERY_MAX_RATE, here and on the robot.
The Webots bridge and the robot switch to this policy when the walker tips past 60 degrees, and
back once he's up and steady (see bob_ros2_external.py).
"""

import mujoco
import numpy as np

from env import RockyBobWalk
from spec import CONTROL_DT, STAND, TARGET_LIMIT, observation, projected_gravity, recovery_targets, slew

RECOVER_SECONDS = 8.0
SETTLE_SECONDS = 1.0
# pose: an exp kernel alone is flat far from STAND (after rolling over, legs can be far off and
# got no pull back at all), so a linear term pulls at any distance and the kernel is wide
REWARD = {"upright": 2.0, "height": 2.0, "pose": 1.0, "pose_lin": -0.15, "recovered": 3.0,
          # the lidar is fragile: every second on the ground costs more than being upright earns
          "lidar_ground": -4.0, "lidar_force": -0.3, "lidar_knock": -0.5, "lidar_touch": -4.0,
          "spin": -0.05, "joint_speed": -0.002, "torque": -0.02, "action_rate": -0.5,
          # smooth and controlled: rolling faster than a gentle SPIN_OK costs steeply, and so do hard landings
          "spin_fast": -1.0, "impact": -0.05}
SPIN_OK = 2.0         # rad/s of body rotation that's still a controlled roll
IMPACT_OK = 30.0      # N of rock-on-ground contact force that's still a gentle landing
LIDAR_KNOCK = 30.0    # N: force on the lidar beyond this counts as a knock
UPRIGHT = 0.95        # up-vector z for "upright"


class RockyBobRecover(RockyBobWalk):
    def __init__(self, randomize=True, terrain_level=None, **kw):
        super().__init__(randomize=randomize, episode_seconds=RECOVER_SECONDS, command=(0, 0, 0),
                         terrain_level=0.0 if terrain_level is None else terrain_level, pushes=False,
                         floor_sensor=False, **kw)
        m = self.model
        self.lidar_geom = m.geom("lidar").id
        self.body_geoms = set(self.base_geom)
        self.seconds = RECOVER_SECONDS
        self.stand_height = None
        self._force = np.zeros(6)

    def _body_force(self):
        """Total contact force (N) of the rock (and lidar) on the ground: the impacts of rolling over."""
        d, total = self.data, 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in self.body_geoms or c.geom2 in self.body_geoms:
                mujoco.mj_contactForce(self.model, d, i, self._force)
                total += abs(self._force[0])
        return total

    def _lidar_force(self):
        d, total = self.data, 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if self.lidar_geom in (c.geom1, c.geom2):
                mujoco.mj_contactForce(self.model, d, i, self._force)
                total += abs(self._force[0])
        return total

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        m, d, r = self.model, self.data, self.np_random
        if self.stand_height is None:
            self.stand_height = float(d.qpos[2] - self.ground(*d.qpos[:2]))
        kind = r.random()
        if kind < 0.5:      # on his back, give or take
            roll, pitch = np.pi + r.uniform(-0.5, 0.5), r.uniform(-0.4, 0.4)
        elif kind < 0.75:   # on a side
            roll, pitch = r.choice([-1, 1]) * np.pi / 2 + r.uniform(-0.4, 0.4), r.uniform(-0.4, 0.4)
        else:
            roll, pitch = r.uniform(-np.pi, np.pi), r.uniform(-np.pi / 2, np.pi / 2)
        yaw = r.uniform(-np.pi, np.pi)
        q = np.zeros(4)
        mujoco.mju_euler2Quat(q, np.array([roll, pitch, yaw]), "xyz")
        d.qpos[3:7] = q
        d.qpos[7:] = r.uniform(-1, 1, 12) * TARGET_LIMIT * 0.8
        d.qpos[2] = self.ground(*d.qpos[:2]) + 0.35
        d.qvel[:] = 0
        d.ctrl[:] = d.qpos[7:]
        for _ in range(int(SETTLE_SECONDS / m.opt.timestep)):   # fall and settle, servos holding the pose
            mujoco.mj_step(m, d)
        self.t, self.steps = 0.0, 0
        self.last_action = d.qpos[7:] / TARGET_LIMIT
        self.targets = d.qpos[7:].copy()
        self.prev_action = self.last_action.copy()
        self.was_on_lidar = self._lidar_force() > 0
        self.recovered_for = 0.0
        return self._recover_obs(), {}

    def _recover_obs(self):
        gravity = projected_gravity(self._sense("orientation", 4))
        gyro = self._sense("gyro", 3)
        joints = self.data.qpos[7:].copy()
        if self.randomize:
            gravity += self.np_random.normal(0, 0.03, 3)
            gyro += self.np_random.normal(0, 0.1, 3)
            joints += self.np_random.normal(0, 0.02, 12)
        return observation(gravity, gyro, np.zeros(3), joints, self.last_action, 0.0)

    def step(self, action):
        m, d = self.model, self.data
        action = np.clip(np.asarray(action, dtype=float), -1, 1)
        self.targets = slew(self.targets, recovery_targets(action))   # joints move no faster than RECOVERY_MAX_RATE
        d.ctrl[:] = self.targets
        torque = force = knock = 0.0
        impact = spin = 0.0
        for _ in range(self.frame_skip):
            mujoco.mj_step(m, d)
            torque += np.sum(d.actuator_force ** 2)
            f = self._lidar_force()
            force += f
            knock = max(knock, f - LIDAR_KNOCK)
            impact = max(impact, self._body_force())
            spin = max(spin, float(np.linalg.norm(d.qvel[3:6])))
        torque /= self.frame_skip
        force /= self.frame_skip
        self.t += CONTROL_DT
        self.steps += 1

        up = -projected_gravity(d.qpos[3:7])[2]         # 1 upright, -1 upside down
        upright = (up + 1) / 2
        height = float(d.qpos[2] - self.ground(*d.qpos[:2]))
        pose_err = float(np.sum((d.qpos[7:] - STAND) ** 2))
        is_up = up > UPRIGHT
        stood = is_up and abs(height - self.stand_height) < 0.03 and pose_err < 0.3
        self.recovered_for = self.recovered_for + CONTROL_DT if stood else 0.0
        on_lidar = force > 0
        terms = {
            "upright": upright,
            "height": np.exp(-((height - self.stand_height) / 0.03) ** 2) if is_up else 0.0,
            "pose": np.exp(-pose_err / 4.0) if is_up else 0.0,
            "pose_lin": pose_err if is_up else 0.0,
            "recovered": float(stood),
            "lidar_ground": float(force > 0),
            "lidar_force": force,
            "lidar_knock": max(knock, 0.0),
            "lidar_touch": float(on_lidar and not self.was_on_lidar) / CONTROL_DT,
            "spin": float(np.sum(d.qvel[3:6] ** 2)),
            "joint_speed": float(np.sum(d.qvel[6:] ** 2)),
            "torque": torque,
            "action_rate": float(np.sum((action - self.last_action) ** 2)),
            "spin_fast": max(spin - SPIN_OK, 0.0) ** 2,
            "impact": max(impact - IMPACT_OK, 0.0),
        }
        self.was_on_lidar = on_lidar
        reward = CONTROL_DT * sum(REWARD[k] * v for k, v in terms.items())
        self.prev_action, self.last_action = self.last_action, action
        truncated = self.t >= self.seconds
        info = {"terms": terms, "up": up, "height": height, "stood": stood, "recovered_for": self.recovered_for,
                "lidar_force": force, "impact": impact, "spin_rate": spin, "terrain_level": self.level, "vel": np.zeros(3), "yaw_rate": 0.0,
                "torque": float(np.abs(d.actuator_force).max())}
        return self._recover_obs(), float(reward), False, truncated, info
