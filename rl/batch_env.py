"""All the walking robots in one process: physics on a thread pool, rewards as array math.

train.py used to run 12 worker processes of 4 robots each (vec.py). Every control step then waits
for the slowest worker, and on an M2 Max four of them run on the slower efficiency cores; each
robot's reward was also ~0.35 ms of per-robot Python. Here:

- each robot keeps its own RockyBobWalk (its MjModel with randomised masses and gains, its MjData,
  its terrain and random generator), which still does everything once per episode: reset,
  randomisation, terrain, curriculum, command sampling;
- the physics of all robots steps on a thread pool (MuJoCo releases the GIL in mj_step), so a slow
  core just takes fewer robots;
- everything per control step after the physics (rewards, contacts, gait bookkeeping, the floor
  sensor, observations) is computed for all robots at once with numpy.

The reward is RockyBobWalk.step's, term for term; test_batch_env.py checks them against it robot by
robot. It's a Stable-Baselines3 VecEnv, with Monitor-style episode info, for train.py --batched.
"""

import time
from concurrent.futures import ThreadPoolExecutor

import mujoco
import numpy as np
from stable_baselines3.common.vec_env import VecEnv

import env as E
import tof
from env import RockyBobWalk
from spec import (ACTION_SCALE, CONTROL_DT, GAIT_PERIOD, OBS_SIZE, RUN_PERIOD, RUN_SPEED, STAND, TARGET_LIMIT,
                  TURN_PERIOD, WALK_SPEED)

# per-episode state RockyBobWalk.step keeps, copied into the batch arrays after every reset
_VECTORS = ("last_action", "prev_action", "air_time", "last_land_xy", "last_land_level", "stance_time",
            "swing_peak", "last_peak", "vel_avg", "prev_world_vel", "anchor", "command")
_SCALARS = ("t", "phase", "steps", "target_height", "next_push")


def quat_to_matrix(q):
    """(N, 4) unit quaternions (w, x, y, z) -> (N, 3, 3) body -> world rotations (spec.quat_matrix)."""
    w, x, y, z = q.T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def gait_periods(command):
    """spec.gait_period for (N, 3) commands."""
    speed = np.hypot(command[:, 0], command[:, 1])
    run = np.clip((speed - WALK_SPEED) / (RUN_SPEED - WALK_SPEED), 0.0, 1.0)
    period = GAIT_PERIOD + run * (RUN_PERIOD - GAIT_PERIOD)
    return np.where((speed < 0.02) & (np.abs(command[:, 2]) > 0.1), TURN_PERIOD, period)


class Fields:
    """Every robot's height field, stacked, with terrain.heights_at and tof.HeightfieldRays for many robots."""

    def __init__(self, n, nrow, ncol, half_x, half_y, max_height):
        self.shape, self.half, self.max_height = (nrow, ncol), (half_x, half_y), max_height
        self.data = np.zeros((n, nrow, ncol))
        self.cell = 2 * half_x / (ncol - 1)
        b = tof.HeightfieldRays.BLOCK
        self.blocks = (-(-nrow // b), -(-ncol // b))
        self.block_max = np.zeros((n,) + self.blocks)
        self.low = np.zeros(n)

    def set(self, i, field):
        self.data[i] = field
        b = tof.HeightfieldRays.BLOCK
        pad = np.full((self.blocks[0] * b, self.blocks[1] * b), -np.inf)
        pad[:self.shape[0], :self.shape[1]] = field
        self.block_max[i] = pad.reshape(self.blocks[0], b, self.blocks[1], b).max(axis=(1, 3)) * self.max_height
        self.low[i] = field.min() * self.max_height

    def _cells(self, x, y):
        nrow, ncol = self.shape
        cx = np.clip((x + self.half[0]) / (2 * self.half[0]) * (ncol - 1), 0, ncol - 1.001)
        cy = np.clip((y + self.half[1]) / (2 * self.half[1]) * (nrow - 1), 0, nrow - 1.001)
        c0, r0 = cx.astype(np.intp), cy.astype(np.intp)
        return c0, r0, cx - c0, cy - r0

    def heights(self, robot, x, y):
        """terrain.heights_at (bilinear) for points of robots `robot` (same-shaped index array)."""
        c0, r0, fx, fy = self._cells(x, y)
        f = self.data
        z = (f[robot, r0, c0] * (1 - fx) * (1 - fy) + f[robot, r0, c0 + 1] * fx * (1 - fy)
             + f[robot, r0 + 1, c0] * (1 - fx) * fy + f[robot, r0 + 1, c0 + 1] * fx * fy)
        return z * self.max_height

    def surface(self, robot, x, y):
        """MuJoCo's own surface (two triangles per cell), as tof.HeightfieldRays._heights."""
        c0, r0, fx, fy = self._cells(x, y)
        f = self.data
        h00, h11, h01, h10 = f[robot, r0, c0], f[robot, r0 + 1, c0 + 1], f[robot, r0, c0 + 1], f[robot, r0 + 1, c0]
        z = np.where(fx >= fy, h00 + (h01 - h00) * fx + (h11 - h01) * fy, h00 + (h10 - h00) * fy + (h11 - h10) * fx)
        return z * self.max_height

    def _high(self, i, p, reach):
        nrow, ncol = self.shape
        b = tof.HeightfieldRays.BLOCK

        def idx(v, half, n):
            return int(np.clip((v + half) / (2 * half) * (n - 1), 0, n - 1)) // b
        c0, c1 = idx(p[0] - reach, self.half[0], ncol), idx(p[0] + reach, self.half[0], ncol)
        r0, r1 = idx(p[1] - reach, self.half[1], nrow), idx(p[1] + reach, self.half[1], nrow)
        return float(self.block_max[i, r0:r1 + 1, c0:c1 + 1].max())

    def cast(self, robots, origins, dirs, max_range):
        """tof.HeightfieldRays.cast for several robots at once: origins (k, 3), dirs (k, n, 3) -> (k, n)."""
        k, n = dirs.shape[:2]
        rob = np.repeat(robots, n)
        o = np.repeat(origins, n, axis=0)
        d = dirs.reshape(-1, 3)
        out = np.full(k * n, -1.0)
        high = np.repeat([self._high(i, p, max_range) for i, p in zip(robots, origins)], n)
        low = self.low[rob]
        dz = d[:, 2]
        rise = o[:, 2] - high
        down = dz < -1e-9
        safe = np.where(down, -dz, 1.0)
        t0 = np.where(down, np.maximum(0.0, rise / safe), 0.0)
        t1 = np.where(down, np.minimum(max_range, (o[:, 2] - low + 0.01) / safe), max_range)
        t1 = np.where(~down & (rise > 0), -1.0, t1)
        live = t1 > t0
        if not live.any():
            return out.reshape(k, n)
        hs = np.hypot(d[:, 0], d[:, 1])
        step = np.minimum(tof.HeightfieldRays.FRAC * self.cell / np.maximum(hs, 1e-6), 0.02)
        cnt = np.where(live, np.ceil((t1 - t0) / step).astype(np.intp) + 1, 0)
        ray = np.repeat(np.arange(k * n), cnt)
        start = np.concatenate(([0], np.cumsum(cnt)[:-1]))
        j = np.arange(len(ray)) - start[ray]
        t = np.minimum(t0[ray] + j * step[ray], t1[ray])
        gap = o[ray, 2] + t * dz[ray] - self.surface(rob[ray], o[ray, 0] + t * d[ray, 0], o[ray, 1] + t * d[ray, 1])
        below = np.flatnonzero(gap <= 0)
        if len(below):
            r = ray[below]
            first = np.concatenate(([True], r[1:] != r[:-1]))
            i, r = below[first], r[first]
            prev = np.maximum(i - 1, start[r])
            g0, g1 = gap[prev], gap[i]
            f = np.where(prev < i, g0 / np.maximum(g0 - g1, 1e-12), 0.0)
            out[r] = t[prev] + f * (t[i] - t[prev])
        return out.reshape(k, n)


class BatchedWalk(VecEnv):
    def __init__(self, n, threads=8, stream=False, env_kwargs=None):
        self.envs = [RockyBobWalk(stream=stream and i == 0, **(env_kwargs or {})) for i in range(n)]
        e0 = self.envs[0]
        super().__init__(n, e0.observation_space, e0.action_space)
        self.n = n
        self.pool = ThreadPoolExecutor(threads)
        self.chunks = [list(range(k, n, threads * 4)) for k in range(min(n, threads * 4))]
        self.frame_skip = e0.frame_skip
        self.feet = np.array(e0.feet)
        self.foot_radius = e0.foot_radius
        self.floor_sensor = e0.floor_sensor
        self.sensor = e0.sensor
        self.fields = Fields(n, *e0.field_shape, *e0.field_half, e0.field_max)
        self.rng = np.random.default_rng()
        self.actions = np.zeros((n, 12))
        # physics outputs per substep, filled by the worker threads
        self.force = np.zeros((n, self.frame_skip, 12))
        self.jvel = np.zeros((n, self.frame_skip, 12))
        # per-robot state (RockyBobWalk's step state), rows refreshed from the env on every reset
        for k in _SCALARS:
            setattr(self, k, np.zeros(n))
        self.last_land_xy = np.zeros((n, 4, 2))
        self.last_action, self.prev_action = np.zeros((n, 12)), np.zeros((n, 12))
        self.air_time, self.stance_time, self.swing_peak = np.zeros((n, 4)), np.zeros((n, 4)), np.zeros((n, 4))
        self.last_land_level, self.last_peak = np.zeros((n, 4)), np.zeros((n, 4))
        self.vel_avg, self.prev_world_vel, self.command = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 3))
        self.anchor = np.zeros((n, 2))
        self.stair_practice = np.zeros(n, bool)
        self.counts_climb = np.zeros(n, bool)   # practice other than a plain flight: tall_level tracks height
        self.walked, self.asked, self.climbed, self.fell = (np.zeros(n) for _ in range(4))
        self.penalty = np.ones(n)
        self.tof_next = np.zeros(n)
        self.tof_last = np.full((n, tof.ZONES ** 2), np.inf)
        self.ep_return, self.ep_len, self.ep_start = np.zeros(n), np.zeros(n, int), np.full(n, time.time())
        self.obs = np.zeros((n,) + self.observation_space.shape, np.float32)
        self._vel6 = [np.zeros(6) for _ in range(n)]

    # ------------------------------------------------------------ state <-> envs
    def _load(self, i):
        """Copy robot i's freshly reset episode state from its env into the batch arrays."""
        e = self.envs[i]
        for k in _VECTORS:
            getattr(self, k)[i] = getattr(e, k)
        for k in _SCALARS:
            getattr(self, k)[i] = getattr(e, k)
        self.stair_practice[i] = e.stair_practice
        self.counts_climb[i] = e.practice is not None and e.practice[0] != "stairs"
        self.walked[i] = self.asked[i] = self.climbed[i] = 0.0
        self.fell[i] = False
        self.penalty[i] = e.penalty_scale
        self.fields.set(i, e.field)
        if e.tof is not None:
            self.tof_next[i], self.tof_last[i] = e.tof.next, e.tof.last
        self.ep_return[i], self.ep_len[i], self.ep_start[i] = 0.0, 0, time.time()

    def _reset_one(self, i, seed=None):
        e = self.envs[i]
        e._fell, e._walked, e._asked, e._climbed = bool(self.fell[i]), self.walked[i], self.asked[i], self.climbed[i]
        obs, _ = e.reset(seed=seed)
        self._load(i)
        return obs

    # ------------------------------------------------------------ VecEnv API
    def reset(self):
        for i in range(self.n):
            seed = self._seeds[i] if self._seeds else None
            self.obs[i] = self._reset_one(i, seed)
        if self._seeds and self._seeds[0] is not None:
            self.rng = np.random.default_rng(self._seeds[0] + 10_000_019)
        self._reset_seeds()
        return self.obs.copy()

    def step_async(self, actions):
        self.actions = np.clip(np.asarray(actions, dtype=float), -1, 1)

    def _physics(self, idx):
        fs = self.frame_skip
        for i in idx:
            e = self.envs[i]
            m, d = e.model, e.data
            for s in range(fs):
                mujoco.mj_step(m, d)
                self.force[i, s] = d.actuator_force
                self.jvel[i, s] = d.qvel[6:]

    def step_wait(self):
        n, a = self.n, self.actions
        targets = np.clip(STAND + ACTION_SCALE * a, -TARGET_LIMIT, TARGET_LIMIT)
        for i, e in enumerate(self.envs):
            e.data.ctrl[:] = targets[i]
        list(self.pool.map(self._physics, self.chunks))

        f = self.force
        power = np.abs(f * self.jvel).sum((1, 2)) / self.frame_skip
        torque_sq = (f ** 2).sum((1, 2)) / self.frame_skip
        over_torque = np.maximum(np.abs(f) - E.SOFT_TORQUE, 0.0).sum((1, 2)) / self.frame_skip
        self.t += CONTROL_DT
        self.phase = (self.phase + 2 * np.pi * CONTROL_DT / gait_periods(self.command)) % (2 * np.pi)
        self.steps += 1

        # shoves and new commands (rare): each robot's own generator, as RockyBobWalk does
        for i in np.flatnonzero(self.t >= self.next_push):
            e = self.envs[i]
            if not e.pushes:
                continue
            e.data.qvel[:2] += e.np_random.uniform(-E.PUSH_SPEED, E.PUSH_SPEED, 2)
            e.data.qvel[5] += e.np_random.uniform(-E.PUSH_SPIN, E.PUSH_SPIN)
            self.next_push[i] = self.t[i] + e.np_random.uniform(*E.PUSH_EVERY)
        for i in np.flatnonzero(self.steps % 250 == 0):
            e = self.envs[i]
            if e.fixed_command is None and not self.counts_climb[i]:
                self.command[i] = e._sample_command()
                self.anchor[i] = e.data.qpos[:2]

        # gather what the rewards need
        qpos = np.empty((n, 19))
        qvel = np.empty((n, 18))
        qacc = np.empty((n, 12))
        quat = np.empty((n, 4))
        gyro = np.empty((n, 3))
        feet = np.empty((n, 4, 3))
        foot_speed = np.empty((n, 4))
        touching = np.zeros((n, 4), bool)
        leg_hit, base_hit, lidar_hit = np.zeros(n, bool), np.zeros(n, bool), np.zeros(n, bool)
        so, sg = self.sensor["orientation"], self.sensor["gyro"]
        for i, e in enumerate(self.envs):
            m, d = e.model, e.data
            qpos[i], qvel[i], qacc[i] = d.qpos, d.qvel, d.qacc[6:]
            quat[i], gyro[i] = d.sensordata[so:so + 4], d.sensordata[sg:sg + 3]
            feet[i] = d.geom_xpos[self.feet]
            touching[i], leg_hit[i], base_hit[i] = e._foot_contacts()
            lidar_hit[i] = e.lidar_hit
            v = self._vel6[i]
            for k, g in enumerate(self.feet):
                mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, g, v, 0)
                foot_speed[i, k] = np.hypot(v[3], v[4])

        R = quat_to_matrix(quat)
        gravity = -R[:, 2, :]                                    # = -(R.T)[:, :, 2]
        vel = np.einsum("nji,nj->ni", R, qvel[:, :3])           # R.T @ v
        ang = np.einsum("nji,nj->ni", R, qvel[:, 3:6])
        body_acc = (qvel[:, :3] - self.prev_world_vel) / CONTROL_DT
        self.prev_world_vel = qvel[:, :3].copy()
        rate = CONTROL_DT / E.TRACK_TAU
        self.vel_avg += rate * (np.stack([vel[:, 0], vel[:, 1], ang[:, 2]], 1) - self.vel_avg)
        cmd = self.command
        asked = np.linalg.norm(cmd[:, :2], axis=1)
        along = np.einsum("ni,ni->n", vel[:, :2], cmd[:, :2])
        self.walked += np.where(asked > 0, np.maximum(along / np.maximum(asked, 1e-12), 0.0) * CONTROL_DT, 0.0)
        self.asked += asked * CONTROL_DT

        rob = np.arange(n)
        px = np.concatenate([feet[:, :, 0], qpos[:, :1]], 1)
        py = np.concatenate([feet[:, :, 1], qpos[:, 1:2]], 1)
        ground = self.fields.heights(np.repeat(rob, 5).reshape(n, 5), px, py)
        self.climbed = np.where(self.counts_climb, np.maximum(self.climbed, ground[:, 4]), self.climbed)

        first_contact = touching & (self.air_time > 0)
        turning = (np.abs(cmd[:, 0]) < 0.02) & (np.abs(cmd[:, 1]) < 0.02) & (np.abs(cmd[:, 2]) > 0.1)
        moving = np.linalg.norm(cmd, axis=1) > 0.05
        feet_air = np.sum((self.air_time - 0.2) * first_contact, 1) * moving
        air_before = self.air_time.copy()
        self.air_time = np.where(touching, 0.0, self.air_time + CONTROL_DT)
        self.stance_time = np.where(touching, self.stance_time + CONTROL_DT, 0.0)
        clearance = feet[:, :, 2] - self.foot_radius - ground[:, :4]
        foot_low = (np.maximum(E.FOOT_CLEARANCE - clearance, 0.0) / E.FOOT_CLEARANCE) ** 2
        land_xy = feet[:, :, :2]
        new_level = first_contact & (np.abs(ground[:, :4] - self.last_land_level) > E.STEP_LEVEL)
        retouch = first_contact & (np.hypot(*(land_xy - self.last_land_xy).transpose(2, 0, 1)) < E.RETOUCH_DIST)
        self.last_land_xy = np.where(first_contact[:, :, None], land_xy, self.last_land_xy)
        self.last_land_level = np.where(first_contact, ground[:, :4], self.last_land_level)
        self.swing_peak = np.where(touching, self.swing_peak, np.maximum(self.swing_peak, clearance))
        real_step = first_contact & (air_before >= E.BOUNCE_AIR)
        bounce = first_contact & (air_before < E.BOUNCE_AIR)
        tap = real_step & (self.swing_peak < E.TAP_HEIGHT)
        swing_height = np.where(moving, np.sum(np.exp(-((self.swing_peak - E.SWING_HEIGHT) / E.SWING_SIGMA) ** 2)
                                               * real_step, 1), 0.0)
        self.last_peak = np.where(real_step, self.swing_peak, self.last_peak)
        complete = ~np.isnan(self.last_peak).any(1)
        with np.errstate(invalid="ignore"):
            spread = np.std(self.last_peak, 1)
        swing_spread = np.where(complete & moving, spread * np.sum(real_step, 1), 0.0)
        self.swing_peak = np.where(touching, 0.0, self.swing_peak)

        # gait schedule (RockyBobWalk._gait_score / expected_contact)
        standing = np.linalg.norm(cmd, axis=1) < 0.05
        ph = np.sin(self.phase)[:, None]
        expected = np.where(standing[:, None], True, np.where(E.TROT_PAIR == 0, ph >= 0, ph < 0))
        swing = ~expected
        gait = np.where(expected.all(1), touching.mean(1),
                        np.sum(~touching & swing, 1) / np.maximum(swing.sum(1), 1))

        # stumbles (RockyBobWalk._stumbles)
        reach = self.foot_radius + 0.02
        ring = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        rx = feet[:, :, 0, None] + reach[None, :, None] * np.cos(ring)
        ry = feet[:, :, 1, None] + reach[None, :, None] * np.sin(ring)
        around = self.fields.heights(np.broadcast_to(rob[:, None, None], rx.shape), rx, ry)
        stumble = touching & (around.max(2) - ground[:, :4] > E.STUMBLE_RISE)

        anchor_d2 = np.sum((qpos[:, :2] - self.anchor) ** 2, 1)
        a2 = asked ** 2
        prog = np.einsum("ni,ni->n", self.vel_avg[:, :2], cmd[:, :2])
        act_d = a - self.last_action
        with np.errstate(divide="ignore", invalid="ignore"):
            run_progress = np.where(asked > WALK_SPEED, np.clip(prog / a2, 0.0, 1.0), 0.0)
            turn_progress = np.where(turning, np.clip(self.vel_avg[:, 2] / cmd[:, 2], 0.0, 1.0), 0.0)
        terms = {
            "track_lin": np.exp(-np.sum((cmd[:, :2] - self.vel_avg[:, :2]) ** 2, 1) / (0.02 + 0.06 * a2)),
            "run_progress": run_progress,
            "track_yaw": np.exp(-(cmd[:, 2] - self.vel_avg[:, 2]) ** 2 / 0.1),
            "lin_z": vel[:, 2] ** 2,
            "ang_xy": np.sum(ang[:, :2] ** 2, 1),
            "tilt": np.sum(gravity[:, :2] ** 2, 1),
            "height": (qpos[:, 2] - ground[:, 4] - self.target_height) ** 2,
            "power": power,
            "torque": torque_sq,
            "action_rate": np.sum(act_d ** 2, 1),
            "action_smooth": np.sum((a - 2 * self.last_action + self.prev_action) ** 2, 1),
            "joint_acc": np.sum(qacc ** 2, 1),
            "joint_vel": np.sum(qvel[:, 6:] ** 2, 1),
            "joint_limit": np.sum(np.maximum(np.abs(qpos[:, 7:]) - E.SOFT_JOINT_LIMIT, 0.0), 1),
            "torque_limit": over_torque,
            "feet_air": feet_air,
            "gait": gait,
            "turn_progress": turn_progress,
            "turn_anchor": np.where(turning, np.exp(-anchor_d2 / E.TURN_ANCHOR_SIGMA), 0.0),
            "turn_footfall": np.where(turning, first_contact.sum(1), 0.0),
            "turn_drift": np.where(turning, np.hypot(qvel[:, 0], qvel[:, 1]), 0.0),
            "turn_slip": np.where(turning, np.sum(foot_speed * touching, 1), 0.0),
            "turn_planted": np.where(turning, np.sum(np.maximum(self.stance_time - E.TURN_MAX_STANCE, 0.0), 1), 0.0),
            "turn_offset": np.where(turning, np.sqrt(anchor_d2), 0.0),
            "swing_height": swing_height,
            "swing_spread": swing_spread,
            "foot_bounce": bounce.sum(1).astype(float),
            "tap": np.where(moving, tap.sum(1), 0.0),
            "stair_step": new_level.sum(1).astype(float),
            "retouch": np.where(asked > 0.1, retouch.sum(1), 0.0),
            "stair_progress": np.where(self.stair_practice & (asked > 0.05),
                                       np.clip(prog / np.maximum(a2, 1e-4), -1.0, 1.0), 0.0),
            "stair_jitter": np.where(self.stair_practice, np.sum(act_d ** 2, 1), 0.0),
            "steady": np.exp(-(ang[:, 0] ** 2 + ang[:, 1] ** 2) / E.STEADY_SIGMA),
            "smooth_ride": np.exp(-np.sum(body_acc ** 2, 1) / E.RIDE_SIGMA),
            "slip": np.sum(foot_speed * touching, 1),
            "stumble": stumble.sum(1).astype(float),
            "foot_clearance": np.sum(foot_low * np.tanh(5 * foot_speed) * ~touching, 1),
            "leg_contact": leg_hit.astype(float),
            "belly": base_hit.astype(float),
            "alive": np.ones(n),
        }
        reward = np.zeros(n)
        for k, v in terms.items():
            reward += E.REWARD[k] * v * (self.penalty if k in E.SHAPING else 1.0)
        reward *= CONTROL_DT
        self.terms = terms
        self.prev_action = self.last_action
        self.last_action = a.copy()

        belly_ok = np.array([base_hit[i] and self.envs[i]._belly_ok() for i in range(n)])
        terminated = (gravity[:, 2] > -0.5) | lidar_hit | (base_hit & ~belly_ok)
        truncated = self.steps >= self.envs[0].max_steps
        self.fell = np.logical_or(self.fell, terminated)

        e0 = self.envs[0]
        if e0._stream is not None:
            e0.command, e0.t = cmd[0].copy(), self.t[0]
            packet = np.concatenate([qpos[0], cmd[0], vel[0, :2], [ang[0, 2], reward[0], float(terminated[0])]])
            try:
                e0._stream.sendto(np.asarray(packet, "<f8").tobytes(), ("127.0.0.1", E.STREAM_PORT))
            except OSError:
                pass

        self.obs = self._observe(quat, gyro, qpos)
        self.ep_return += reward
        self.ep_len += 1
        done = terminated | truncated
        infos = [{"terrain_level": e.level, "tall_level": e.tall_level} for e in self.envs]
        for i in np.flatnonzero(done):
            info = infos[i]
            info["terminal_observation"] = self.obs[i].copy()
            info["TimeLimit.truncated"] = bool(truncated[i] and not terminated[i])
            info["episode"] = {"r": float(self.ep_return[i]), "l": int(self.ep_len[i]),
                               "t": round(time.time() - self.ep_start[i], 6)}
            self.obs[i] = self._reset_one(i)
            info["terrain_level"], info["tall_level"] = self.envs[i].level, self.envs[i].tall_level
        return self.obs.copy(), reward.astype(np.float32), done, infos

    def _observe(self, quat, gyro, qpos):
        """spec.observation for every robot (with RockyBobWalk's sensor noise), plus the floor sensor."""
        n = self.n
        R = quat_to_matrix(quat)
        grav_true = -R[:, 2, :]
        gravity, gyro, joints = grav_true.copy(), gyro.copy(), qpos[:, 7:].copy()
        if self.envs[0].randomize:
            gravity += self.rng.normal(0, 0.03, (n, 3))
            gyro += self.rng.normal(0, 0.1, (n, 3))
            joints += self.rng.normal(0, 0.02, (n, 12))
        parts = [gravity, gyro * 0.25, self.command * [2.0, 2.0, 1.0], joints - STAND, self.last_action,
                 np.sin(self.phase)[:, None], np.cos(self.phase)[:, None]]
        if self.floor_sensor:
            parts.append(tof.heights_batch(self._scan(), grav_true) * 10.0)
        return np.concatenate(parts, 1).astype(np.float32)

    def _scan(self):
        """tof.ToFModel.update for every robot: rescan the robots whose 15 Hz sensor is due."""
        due = np.flatnonzero(self.t >= self.tof_next)
        if len(due):
            self.tof_next[due] = self.t[due] + tof.PERIOD
            fast, slow = [], []
            for i in due:
                (fast if self.envs[i].tof._terrain_only(self.envs[i].data) else slow).append(i)
            dist = {}
            if fast:
                sites = [self.envs[i].tof.site for i in fast]
                origins = np.array([self.envs[i].data.site_xpos[s] for i, s in zip(fast, sites)])
                mats = np.array([self.envs[i].data.site_xmat[s].reshape(3, 3) for i, s in zip(fast, sites)])
                rays = self.fields.cast(np.array(fast), origins, np.einsum("zj,kij->kzi", tof.DIRS, mats), tof.MAX_RANGE)
                dist.update(zip(fast, rays))
            for i in slow:   # something besides the terrain in range: MuJoCo's own ray casting
                dist[i] = self._mujoco_rays(self.envs[i])
            k = len(due)
            d = np.stack([dist[i] for i in due])
            d = np.where(d < 0, np.inf, d)
            d = d + self.rng.normal(0, 1, d.shape) * (tof.RANGE_NOISE[0] + tof.RANGE_NOISE[1] * np.minimum(d, tof.MAX_RANGE))
            d[self.rng.random((k, d.shape[1])) < tof.DROPOUT] = np.inf
            self.tof_last[due] = d
        return self.tof_last

    @staticmethod
    def _mujoco_rays(e):
        t = e.tof
        R = e.data.site_xmat[t.site].reshape(3, 3)
        t.mujoco.mj_multiRay(t.model, e.data, e.data.site_xpos[t.site].copy(), (tof.DIRS @ R.T).ravel(),
                             np.array([1, 1, 0, 1, 1, 1], np.uint8), 1, t.body, t.geomid, t.dist, len(tof.DIRS), tof.MAX_RANGE)
        return t.dist.copy()

    def close(self):
        self.pool.shutdown()
        for e in self.envs:
            e.close()

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        out = [getattr(self.envs[i], method_name)(*method_args, **method_kwargs) for i in self._idx(indices)]
        if method_name == "set_penalty_scale":
            self.penalty[:] = [e.penalty_scale for e in self.envs]
        return out

    def get_attr(self, attr_name, indices=None):
        return [getattr(self.envs[i], attr_name) for i in self._idx(indices)]

    def set_attr(self, attr_name, value, indices=None):
        for i in self._idx(indices):
            setattr(self.envs[i], attr_name, value)

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False for _ in self._idx(indices)]

    def _idx(self, indices):
        if indices is None:
            return range(self.n)
        return [indices] if isinstance(indices, int) else indices


def _serve(remote, parent, n, threads, stream):
    parent.close()
    b = BatchedWalk(n, threads=threads, stream=stream)
    try:
        while True:
            cmd, data = remote.recv()
            if cmd == "step":
                b.step_async(data)
                remote.send(b.step_wait())
            elif cmd == "reset":
                remote.send(b.reset())
            elif cmd == "seed":
                b.seed(data)
                remote.send(None)
            elif cmd == "env_method":
                name, args, kwargs, idx = data
                remote.send(b.env_method(name, *args, indices=idx, **kwargs))
            elif cmd == "get_attr":
                remote.send(b.get_attr(*data))
            elif cmd == "set_attr":
                b.set_attr(*data)
                remote.send(None)
            elif cmd == "close":
                b.close()
                remote.close()
                break
    except KeyboardInterrupt:
        pass


class ProcessBatches(VecEnv):
    """Several worker processes, each running a BatchedWalk: the per-step Python work of a
    BatchedWalk runs on one core, so a few of them side by side use more of the machine."""

    def __init__(self, n, workers=4, threads=2, stream=False):
        import multiprocessing as mp

        ctx = mp.get_context("forkserver")
        sizes = [n // workers + (w < n % workers) for w in range(workers)]
        self.slices, start = [], 0
        for s in sizes:
            self.slices.append(slice(start, start + s))
            start += s
        self.remotes, self.procs = [], []
        for w, s in enumerate(sizes):
            remote, child = ctx.Pipe()
            p = ctx.Process(target=_serve, args=(child, remote, s, threads, stream and w == 0), daemon=True)
            p.start()
            child.close()
            self.remotes.append(remote)
            self.procs.append(p)
        e = RockyBobWalk()
        super().__init__(n, e.observation_space, e.action_space)
        e.close()

    def _all(self, cmd, per_worker):
        for r, d in zip(self.remotes, per_worker):
            r.send((cmd, d))
        return [r.recv() for r in self.remotes]

    def reset(self):
        if self._seeds and self._seeds[0] is not None:
            self._all("seed", [self._seeds[s.start] for s in self.slices])
        self._reset_seeds()
        return np.concatenate(self._all("reset", [None] * len(self.remotes)))

    def step_async(self, actions):
        for r, s in zip(self.remotes, self.slices):
            r.send(("step", actions[s]))

    def step_wait(self):
        res = [r.recv() for r in self.remotes]
        obs, rew, done, infos = zip(*res)
        return np.concatenate(obs), np.concatenate(rew), np.concatenate(done), [i for part in infos for i in part]

    def _route(self, indices):
        idx = range(self.num_envs) if indices is None else ([indices] if isinstance(indices, int) else list(indices))
        return [[i - s.start for i in idx if s.start <= i < s.stop] for s in self.slices]

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        parts = self._all("env_method", [(method_name, method_args, method_kwargs, ix) for ix in self._route(indices)])
        return [x for p in parts for x in p]

    def get_attr(self, attr_name, indices=None):
        return [x for p in self._all("get_attr", [(attr_name, ix) for ix in self._route(indices)]) for x in p]

    def set_attr(self, attr_name, value, indices=None):
        self._all("set_attr", [(attr_name, value, ix) for ix in self._route(indices)])

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * sum(len(ix) for ix in self._route(indices))

    def close(self):
        for r in self.remotes:
            r.send(("close", None))
        for p in self.procs:
            p.join()
