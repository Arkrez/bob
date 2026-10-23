"""Goal-reaching on top of the trained walker: a small "navigator" policy learns to steer.

Two levels, the standard way navigation is layered on legged robots:
  walker    (policies/rocky_walk.npz, frozen): 50 Hz, turns velocity commands into joint targets
  navigator (this env, trained with PPO):        5 Hz, turns "where is the goal" into velocity commands

Each episode drops random goals 0.5-2.5 m away; reaching one (within GOAL_RADIUS) scores a
bonus and spawns the next. Rewards: progress toward the goal, a reach bonus, facing the goal
while walking forward, and small penalties for sideways commands, jerky commands and time.
The walker keeps its domain randomisation, rough terrain and shoves, so the navigator learns
to correct the walker's drift instead of trusting it.

Observations come from the Pi camera model (camera.py), exactly what the robot will have:
  last detected target x, y (m, body frame), its distance and bearing cos/sin, visible now (0/1),
  age of that detection, last command (3), yaw rate, gravity (3)
Goals often start behind or beside the robot, so it has to learn to turn and look for them.
The reward uses the true distance (that's fine: rewards are only needed in training).
"""

import pathlib

import gymnasium as gym
import numpy as np

from camera import CameraModel
from env import RockyBobWalk
from nav_spec import (CMD_HIGH, CMD_LOW, GOAL_RADIUS, NAV_DT, NAV_OBS, action_to_command,  # noqa: F401
                      goal_in_body, nav_observation)
from policy import NumpyPolicy
from spec import CONTROL_DT, projected_gravity

HERE = pathlib.Path(__file__).resolve().parent
WALKER = HERE / "policies" / "rocky_walk.npz"
WALK_STEPS = int(round(NAV_DT / CONTROL_DT))   # walker steps per navigator step
GOAL_RANGE = (0.5, 2.5)                        # m from the robot
FIELD = 5.0                                    # keep goals inside the terrain (half-size, m)
REWARD = {"progress": 10.0, "reach": 10.0, "facing": 0.3, "sideways": -2.0,
          "cmd_rate": -0.5, "time": -0.1, "fall": -20.0,
          # turn first, then walk straight: moving forward while not facing the target, and
          # turning while moving forward, both cost more than the progress they buy
          "misaligned_move": -10.0, "arc": -5.0}
# Gentle steering while walking is free: the walker drifts, and without small corrections the
# target slides out of the camera's view. Only turning faster than this while walking counts as arcing.
ARC_DEADZONE = 0.15   # rad/s


class RockyBobNavigate(gym.Env):
    def __init__(self, episode_seconds=40.0, randomize=True, terrain=(0.0, 0.6), walker=WALKER):
        self.walk = RockyBobWalk(randomize=randomize, episode_seconds=1e9, command=(0, 0, 0),
                                 terrain_level=0.0, pushes=randomize)
        self.walker = NumpyPolicy(walker)
        self.terrain = terrain
        self.max_steps = int(episode_seconds / NAV_DT)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (NAV_OBS,), np.float32)

    def _new_goal(self):
        base = self.walk.data.qpos[:2]
        for _ in range(20):
            r = self.np_random.uniform(*GOAL_RANGE)
            a = self.np_random.uniform(-np.pi, np.pi)
            g = base + r * np.array([np.cos(a), np.sin(a)])
            if np.all(np.abs(g) < FIELD):
                break
        self.goal = g

    def _goal_body(self):
        d = self.walk.data
        return goal_in_body(self.goal, d.qpos[:2], d.qpos[3:7])

    def _obs(self):
        d = self.walk.data
        gravity = projected_gravity(d.qpos[3:7])
        estimate, visible, age = self.camera.reading(self.walk.t)
        return nav_observation(estimate, visible, age, self.cmd_norm, self.walk._sense("gyro", 3)[2], gravity)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        level = float(self.np_random.uniform(*self.terrain))
        self.walk.fixed_level = self.walk.level = level
        self.walk_obs, _ = self.walk.reset(seed=int(self.np_random.integers(1 << 30)))
        self.steps, self.reached = 0, 0
        self.cmd = np.zeros(3)
        self.cmd_norm = np.zeros(3)
        self.camera = CameraModel(self.np_random)
        self._new_goal()
        self.camera.update(self._goal_body(), self.walk.t)
        self.dist = float(np.linalg.norm(self._goal_body()))
        return self._obs(), {}

    def step(self, action):
        cmd = action_to_command(action)
        cmd_norm = np.clip(np.asarray(action, dtype=float), -1, 1)
        self.walk.command = cmd
        fell = False
        for _ in range(WALK_STEPS):
            self.walk_obs, _, fell, _, _ = self.walk.step(self.walker(self.walk_obs))
            self.camera.update(self._goal_body(), self.walk.t)
            if fell:
                break
        self.steps += 1

        goal_body = self._goal_body()
        dist = float(np.linalg.norm(goal_body))
        bearing = np.arctan2(goal_body[1], goal_body[0])
        terms = {
            "progress": self.dist - dist,
            "reach": 0.0,
            "facing": np.cos(bearing) * max(cmd[0], 0.0) / CMD_HIGH[0],   # walking forward, facing the goal
            "sideways": abs(cmd[1]) / CMD_HIGH[1] * NAV_DT,
            "misaligned_move": max(cmd[0], 0.0) / CMD_HIGH[0] * (1 - np.cos(bearing)),
            "arc": max(abs(cmd[2]) - ARC_DEADZONE, 0.0) / (CMD_HIGH[2] - ARC_DEADZONE)
                   * max(cmd[0], 0.0) / CMD_HIGH[0] * NAV_DT,
            "cmd_rate": float(np.sum((cmd_norm - self.cmd_norm) ** 2)) * NAV_DT,
            "time": NAV_DT,
            "fall": float(fell),
        }
        if dist < GOAL_RADIUS:
            terms["reach"] = 1.0
            self.reached += 1
            self._new_goal()
            self.camera.reset()   # a new target: nothing known about it yet
            dist = float(np.linalg.norm(self._goal_body()))
        self.dist = dist
        self.cmd, self.cmd_norm = cmd, cmd_norm
        reward = sum(REWARD[k] * v for k, v in terms.items())
        truncated = self.steps >= self.max_steps
        info = {"terms": terms, "reached": self.reached, "dist": dist, "cmd": cmd.copy(),
                "visible": self.camera.reading(self.walk.t)[1]}
        return self._obs(), float(reward), bool(fell), truncated, info

    def render(self):
        return self.walk.render()
