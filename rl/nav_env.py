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

With obstacles=True (the lidar navigator), each episode lays out a random set of walls and boxes
(nav_spec.OBSTACLES) round the robot, the observation adds the lidar (lidar.py, 36 sectors), and
walls hide the target from the camera. Bumping into anything costs a lot: a big penalty for each
new bump, a steady one while touching, a small one for getting close, and one for being stuck.
"""

import pathlib

import gymnasium as gym
import numpy as np

import mujoco

from camera import CameraModel
from env import RockyBobWalk
from lidar import BIN_ANGLES, LidarModel
from nav_spec import (CMD_HIGH, CMD_LOW, GOAL_RADIUS, NAV_DT, NAV_OBS, NAV_OBS_FLOOR, NAV_OBS_LIDAR, OBSTACLES,  # noqa: F401
                      action_to_command, goal_in_body, nav_observation)
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
          "misaligned_move": -10.0, "arc": -5.0,
          # obstacles: each new bump costs about four goals' worth; pressing on costs more every second
          "bump": -40.0, "touching": -20.0, "near": -4.0, "stuck": -3.0}
NEAR = 0.45          # m: lidar returns closer than this start to cost ("near")
STUCK_WINDOW = 10    # navigator steps (2 s): moved less than STUCK_MOVE and turned less than STUCK_TURN
STUCK_MOVE, STUCK_TURN = 0.04, 0.15
LAYOUT_HALF = 3.5    # m: obstacles go within this of the start
CLEAR_START = 0.7    # m: and no closer than this to the robot
GOAL_CLEARANCE = 0.4 # m: goals are at least this far from any obstacle
# Gentle steering while walking is free: the walker drifts, and without small corrections the
# target slides out of the camera's view. Only turning faster than this while walking counts as arcing.
ARC_DEADZONE = 0.15   # rad/s


class RockyBobNavigate(gym.Env):
    def __init__(self, episode_seconds=40.0, randomize=True, terrain=(0.0, 0.6), walker=WALKER, obstacles=False,
                 goals="random"):
        """goals (with obstacles): "random"; "near", 0.3-0.5 m from an obstacle; "hidden", behind
        one (no line of sight from the robot when it appears); "mixed", a third of each."""
        self.goal_mode = goals
        self.walk = RockyBobWalk(randomize=randomize, episode_seconds=1e9, command=(0, 0, 0),
                                 terrain_level=0.0, pushes=randomize)
        self.walker = NumpyPolicy(walker)
        self.terrain = terrain
        self.max_steps = int(episode_seconds / NAV_DT)
        self.obstacles = obstacles
        m = self.walk.model
        self.obstacle_bodies = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"obstacle{i}") for i in range(len(OBSTACLES))]
        self.obstacle_geoms = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"obstacle{i}")
                                        for i in range(len(OBSTACLES))])
        self.mocap = [int(m.body_mocapid[b]) for b in self.obstacle_bodies]
        self.post_mocap = int(m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "goal_post")])
        self.parked = self.walk.data.mocap_pos.copy()
        self.layout = []          # (x, y, yaw, half x, half y) of the obstacles in use
        self.lidar = None
        self.bins = None
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)
        n_obs = NAV_OBS_FLOOR if obstacles else NAV_OBS
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (n_obs,), np.float32)

    # ---------------------------------------------------------------- obstacles
    def _clearance(self, p):
        """Distance (m) from world point p to the nearest obstacle in the layout (inf if none)."""
        best = np.inf
        for x, y, yaw, hx, hy in self.layout:
            c, s = np.cos(yaw), np.sin(yaw)
            dx, dy = p[0] - x, p[1] - y
            lx, ly = abs(c * dx + s * dy), abs(-s * dx + c * dy)
            best = min(best, float(np.hypot(max(lx - hx, 0.0), max(ly - hy, 0.0))))
        return best

    def _lay_out(self):
        d, r = self.walk.data, self.np_random
        d.mocap_pos[:] = self.parked
        self.layout = []
        if not self.obstacles:
            return
        start = d.qpos[:2].copy()
        walls = [i for i, h in enumerate(OBSTACLES) if h[1] < 0.05]
        boxes = [i for i, h in enumerate(OBSTACLES) if h[1] >= 0.05]
        chosen = list(r.choice(walls, r.integers(1, len(walls) + 1), replace=False)) + \
                 list(r.choice(boxes, r.integers(1, len(boxes) + 1), replace=False))
        for i in chosen:
            hx, hy, hz = OBSTACLES[i]
            for _ in range(30):
                x, y = start + r.uniform(-LAYOUT_HALF, LAYOUT_HALF, 2)
                yaw = r.uniform(-np.pi, np.pi)
                self.layout.append((x, y, yaw, hx, hy))
                if self._clearance(start) >= CLEAR_START:
                    break
                self.layout.pop()
            else:
                continue
            k = self.mocap[i]
            d.mocap_pos[k] = [x, y, self.walk.ground(x, y) + hz - 0.03]
            d.mocap_quat[k] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]

    def _touching(self):
        d = self.walk.data
        if not self.layout or d.ncon == 0:
            return False
        return bool(np.isin(d.contact.geom[:d.ncon], self.obstacle_geoms).any())

    def _blocked(self):
        """Is the target hidden behind something? Judged from the lidar, as the bridge does in Webots."""
        if self.bins is None:
            return False
        g = self._goal_body()
        k = int(np.argmin(np.abs(np.angle(np.exp(1j * (BIN_ANGLES - np.arctan2(g[1], g[0])))))))
        return bool(self.bins[k] < np.linalg.norm(g) - 0.25)

    def _line_blocked(self, p, q):
        """Does the straight line p -> q pass through an obstacle? (sampled every 5 cm)"""
        n = max(2, int(np.linalg.norm(q - p) / 0.05))
        return any(self._clearance(p + (q - p) * t) < 0.02 for t in np.linspace(0.1, 0.9, n))

    def _new_goal(self):
        base = self.walk.data.qpos[:2].copy()
        mode = self.goal_mode
        if mode == "mixed":
            mode = ("random", "near", "hidden")[int(self.np_random.integers(3))]
        if not self.layout:
            mode = "random"
        for _ in range(200):
            r = self.np_random.uniform(*GOAL_RANGE) if mode == "random" else self.np_random.uniform(1.0, 3.0)
            a = self.np_random.uniform(-np.pi, np.pi)
            g = base + r * np.array([np.cos(a), np.sin(a)])
            if not np.all(np.abs(g) < FIELD):
                continue
            c = self._clearance(g)
            if mode == "near" and 0.3 <= c <= 0.5:
                break
            if mode == "hidden" and c >= GOAL_CLEARANCE and self._line_blocked(base, g):
                break
            if mode == "random" and c >= GOAL_CLEARANCE:
                break
        self.goal = g
        d = self.walk.data
        d.mocap_pos[self.post_mocap] = [g[0], g[1], self.walk.ground(g[0], g[1])]

    def _goal_body(self):
        d = self.walk.data
        return goal_in_body(self.goal, d.qpos[:2], d.qpos[3:7])

    def _obs(self):
        d = self.walk.data
        gravity = projected_gravity(d.qpos[3:7])
        estimate, visible, age = self.camera.reading(self.walk.t)
        return nav_observation(estimate, visible, age, self.cmd_norm, self.walk._sense("gyro", 3)[2], gravity,
                               lidar=self.bins if self.obstacles else None,
                               floor=self.walk.tof.last_dist if self.obstacles and self.walk.tof is not None else None)

    def _scan(self):
        if self.obstacles:
            mujoco.mj_kinematics(self.walk.model, self.walk.data)
            self.bins = self.lidar.scan(self.walk.data)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        level = float(self.np_random.uniform(*self.terrain))
        self.walk.fixed_level = self.walk.level = level
        self.walk_obs, _ = self.walk.reset(seed=int(self.np_random.integers(1 << 30)))
        self.steps, self.reached = 0, 0
        self.cmd = np.zeros(3)
        self.cmd_norm = np.zeros(3)
        self.camera = CameraModel(self.np_random)
        if self.obstacles and self.lidar is None:
            self.lidar = LidarModel(self.walk.model, self.np_random)
        self._lay_out()
        self._new_goal()
        self._scan()
        self.camera.update(self._goal_body(), self.walk.t, self._blocked())
        self.dist = float(np.linalg.norm(self._goal_body()))
        self.was_touching, self.bumps = False, 0
        self.track = [(self.walk.data.qpos[:2].copy(), 0.0)]
        self.yaw_total = 0.0
        return self._obs(), {}

    def step(self, action):
        cmd = action_to_command(action)
        cmd_norm = np.clip(np.asarray(action, dtype=float), -1, 1)
        self.walk.command = cmd
        fell, touching = False, False
        blocked = self._blocked()
        for _ in range(WALK_STEPS):
            self.walk_obs, _, fell, _, info = self.walk.step(self.walker(self.walk_obs))
            self.camera.update(self._goal_body(), self.walk.t, blocked)
            self.yaw_total += info["yaw_rate"] * CONTROL_DT
            touching = touching or self._touching()
            if fell:
                break
        self.steps += 1
        self._scan()
        bump = touching and not self.was_touching
        self.was_touching = touching
        self.bumps += int(bump)
        self.track.append((self.walk.data.qpos[:2].copy(), self.yaw_total))
        self.track = self.track[-(STUCK_WINDOW + 1):]
        moved = float(np.linalg.norm(self.track[-1][0] - self.track[0][0]))
        turned = abs(self.track[-1][1] - self.track[0][1])
        stuck = len(self.track) > STUCK_WINDOW and moved < STUCK_MOVE and turned < STUCK_TURN
        nearest = float(np.min(self.bins)) if self.bins is not None else np.inf

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
            "bump": float(bump),
            "touching": float(touching) * NAV_DT,
            "near": max(NEAR - nearest, 0.0) / NEAR * NAV_DT,
            "stuck": float(stuck) * NAV_DT,
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
                "visible": self.camera.reading(self.walk.t)[1], "bump": bump, "touching": touching,
                "stuck": stuck, "bumps": self.bumps}
        return self._obs(), float(reward), bool(fell), truncated, info

    def render(self):
        return self.walk.render()
