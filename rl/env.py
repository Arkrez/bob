"""Gymnasium environment: teach Rocky-Bob to walk on velocity commands.

The policy runs at 50 Hz and outputs 12 joint-target offsets around the standing pose.
Its observations are chosen so the same policy can run on the real robot:
  projected gravity (3)  and gyro (3)       -> from an IMU on the body
  velocity command (3)                       -> vx, vy (m/s), yaw rate (rad/s)
  joint positions (12)                       -> hobby servos don't report position; on hardware
                                                feed the last commanded targets instead
  last action (12)
  gait clock sin/cos (2)
The critic gets nothing extra, so there is no hidden "privileged" information.

Domain randomisation (mass, friction, servo strength, sensor noise) keeps the policy from
depending on exact simulator details. Rough terrain with low blocks (terrain.py, on a curriculum:
harder after a good episode, easier after a fall) and regular shoves teach it dynamic balance.
The policy is blind: it feels the ground only through its legs and IMU, like the real robot.
Joint order everywhere: leg0 J1,J3,J4, leg1 ..., leg3 (same as the Webots bridge).
"""

import pathlib
import socket
import struct

import gymnasium as gym
import mujoco
import numpy as np

import terrain
from spec import quat_matrix
from spec import advance_phase
from spec import STALL_TORQUE, WALK_SPEED
import tof
from spec import (CONTROL_DT, GAIT_PERIOD, JOINT_LIMIT, OBS_SIZE, OBS_SIZE_TOF, STAND, action_to_targets, observation,
                  projected_gravity, quat_rotate_inverse)

MODEL = pathlib.Path(__file__).resolve().parent / "rocky_bob.xml"
STREAM_PORT = 9871   # watch.py --live listens here

SOFT_JOINT_LIMIT = 0.85 * JOINT_LIMIT   # rad; penalised beyond this
SOFT_TORQUE = 0.75 * STALL_TORQUE      # N m; penalised above 75 % of stall
FOOT_CLEARANCE = 0.03                  # m of swing-foot height above the local ground
EASIER_SHARE = 0.5                     # share of training episodes on terrain easier than the robot's level
STAIR_PRACTICE = 0.4                   # share of training episodes that start walking at a flight of stairs
STUMBLE_RISE = 0.02                    # m: a step this tall right beside a planted foot means it's at a riser
# Penalties that shape *how* it walks; train.py ramps them in (penalty_scale 0.25 -> 1) so the
# robot learns to walk before it learns to walk politely, instead of finding that freezing is cheapest.
SHAPING = {"power", "torque", "action_rate", "action_smooth", "joint_acc", "joint_vel",
           "joint_limit", "torque_limit", "slip", "foot_clearance", "stumble"}
TURN_ANCHOR_SIGMA = 0.05 ** 2   # m^2: the pivot bonus falls to 1/e at 5 cm of drift
TURN_MAX_STANCE = 0.75           # s: a foot planted longer than this while turning is a pivot leg
SWING_HEIGHT = 0.045        # m: target peak foot clearance of every step
SWING_SIGMA = 0.012         # m: swing_height bonus falls to 1/e this far from the target
BOUNCE_AIR = 0.08           # s: a touchdown after less air time than this is a bounce
TAP_HEIGHT = 0.015          # m: a step that never lifted this far clear is a tap, not a step
STEP_LEVEL = 0.015          # m: a landing this much higher or lower than the foot's last one is onto a new step
RETOUCH_DIST = 0.06         # m: landing closer than this to where the foot last landed is a re-touch
STEADY_SIGMA = 0.1          # (rad/s)^2: roll^2 + pitch^2 rate at which "steady" falls to 1/e
RIDE_SIGMA = 1.0            # (m/s^2)^2: body acceleration (per control step) where "smooth_ride" falls to 1/e
PUSH_EVERY = (2.0, 5.0)                # s between shoves
PUSH_SPEED, PUSH_SPIN = 0.5, 0.6       # m/s sideways and rad/s yaw added by a shove
# Command mix: each kind of motion is practised on its own as well as blended, and speeds stay
# within what the robot can actually do (~0.18 m/s), so it can track every command it gets.
COMMAND_MIX = [            # (share, vx range, vy range, wz range)
    (0.08, (0, 0), (0, 0), (0, 0)),                    # stand still
    (0.22, (-0.12, 0.18), (0, 0), (0, 0)),             # walk forward / back
    (0.18, (0, 0), (0, 0), (-1.0, 1.0)),               # turn in place
    (0.12, (0, 0), (-0.10, 0.10), (0, 0)),             # sidestep
    (0.20, (-0.10, 0.18), (-0.08, 0.08), (-0.4, 0.4)),  # mixed, walking
    (0.20, (0.18, 0.80), (0, 0), (-0.15, 0.15)),       # run: faster stride (spec.gait_period)
]
# Trot schedule on the gait clock: diagonal pairs (leg0 back-right + leg2 front-left, leg1 + leg3)
# alternate; all feet planted when told to stand (Walk These Ways-style contact schedule).
TROT_PAIR = np.array([0, 1, 0, 1])
# Speed and turn rate are scored on a ~1-stride moving average: a trot rocks the body every
# stride, and scoring the instantaneous rates made freezing (perfectly still) look better than walking.
TRACK_TAU = GAIT_PERIOD
REWARD = {
    "track_lin": 3.0, "track_yaw": 3.0, "gait": 1.5,
    # run commands: pays in proportion to how much of the commanded speed is reached, so every
    # extra bit of speed is worth the same (track_lin alone flattens out far from the target)
    "run_progress": 2.0,
    # "chicken head": bonus for a steady body (and camera) while the legs do the work
    "steady": 1.0, "smooth_ride": 1.0,
    # turning in place: pivot over the spot where the turn began, in as few steps as possible
    # ...and as fast as asked: paid per share of the commanded turn rate reached (linear, like run_progress)
    "turn_progress": 2.0,
    "turn_anchor": 3.0, "turn_footfall": -5.0, "turn_drift": -6.0, "turn_slip": -1.0,
    # no pivoting on one planted leg: every foot keeps stepping during a turn in place
    "turn_planted": -2.0,
    # distance off the pivot, linear: a signal at any drift (the anchor bonus alone is flat far out)
    "turn_offset": -10.0,
    # clean, even steps everywhere (pivots too): each step should peak at SWING_HEIGHT, all four legs
    # alike, and land once - a foot that taps down again after a brief lift is a bounce, not a step
    "swing_height": 5.0, "swing_spread": -50.0, "foot_bounce": -5.0,   # -10 made him step less (slower) on stairs
    # decisive climbing: no nervous taps (a touchdown after a tiny lift), keep moving forward on
    # stair practice (walking back counts against), and no jittering while at it
    "tap": -2.0, "stair_progress": 3.0, "stair_jitter": -0.5,
    # stairs, quickly and cleanly: a bonus for every landing on a new step (up or down), so faster
    # climbs earn more; a cost for putting a foot down again where it last landed (each leg touches
    # each step once, no hesitant re-placing)
    "stair_step": 1.5, "retouch": -1.5,
    "lin_z": -2.0, "ang_xy": -0.05, "tilt": -2.0, "height": -20.0,
    "power": -0.02, "torque": -0.15,
    # stepping, not dragging: reward air time, penalise feet that slide on the ground or move
    # while lower than FOOT_CLEARANCE above the terrain beneath them (Isaac Lab foot_clearance)
    "feet_air": 1.0, "slip": -0.5, "foot_clearance": -0.4,
    # stairs: a foot pushing mostly sideways is hitting a riser (legged_gym's feet_stumble), so lift higher
    "stumble": -0.5,
    # smoothness (legged_gym-style): 1st and 2nd difference of actions, joint acceleration and
    # speed. Each is ~5 % of a good walk's reward, so jitter costs more than it gains.
    "action_rate": -0.5, "action_smooth": -0.5, "joint_acc": -5e-6, "joint_vel": -0.005,
    # servo protection (legged_gym-style soft limits): stay clear of end stops and stall torque
    "joint_limit": -3.0, "torque_limit": -5.0,
    "leg_contact": -0.5, "alive": 0.1,
}


class RockyBobWalk(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": int(1 / CONTROL_DT)}

    def __init__(self, render_mode=None, randomize=True, episode_seconds=20.0, command=None, stream=False,
                 terrain_level=None, pushes=None, floor_sensor=True):
        """terrain_level: None = curriculum (training), or a fixed 0 (flat) .. 1 (roughest).
        pushes: shove the robot every few seconds (defaults to `randomize`)."""
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.data = mujoco.MjData(self.model)
        self.frame_skip = int(round(CONTROL_DT / self.model.opt.timestep))
        self.max_steps = int(episode_seconds / CONTROL_DT)
        self.randomize = randomize
        self.fixed_command = None if command is None else np.asarray(command, dtype=float)
        self.render_mode = render_mode
        self._renderer = None
        self._stream = None
        if stream:   # send each state to a local viewer; dropped silently if nobody listens
            self._stream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._stream.setblocking(False)

        m = self.model
        self.feet = [m.geom(f"leg{i}_foot").id for i in range(4)]
        self.floor = m.geom("floor").id
        self.base_geom = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body("base_link").id
                          and m.geom_contype[g] != 0]
        self.leg_geoms = set(g for g in range(m.ngeom) if m.geom_contype[g] == 4) - set(self.feet)
        # what each geom is, for classifying floor contacts without a Python loop:
        # 0 = other, 1..4 = foot index + 1, -1 = body, -2 = other leg part
        self.geom_role = np.zeros(m.ngeom, int)
        self.geom_role[list(self.leg_geoms)] = -2
        self.geom_role[self.base_geom] = -1
        self.geom_role[self.feet] = np.arange(1, 5)
        self.foot_radius = m.geom_size[self.feet, 0]
        self.sensor = {n: m.sensor(n).adr[0] for n in ("orientation", "gyro", "velocity")}
        self.base_mass = m.body_mass.copy()
        self.base_friction = m.geom_friction.copy()
        self.base_kp = m.actuator_gainprm[:, 0].copy()
        self.base_force = m.actuator_forcerange.copy()
        self._vel6 = np.zeros(6)
        self.pushes = randomize if pushes is None else pushes
        self.fixed_level = terrain_level
        self.level = 0.0 if terrain_level is None else float(terrain_level)
        self.field_shape = (int(m.hfield_nrow[0]), int(m.hfield_ncol[0]))
        self.field_half = float(m.hfield_size[0][0]), float(m.hfield_size[0][1])
        self.field_max = float(m.hfield_size[0][2])
        self.field = np.zeros(self.field_shape)
        self._episodes = 0
        self._terrain_dirty = False
        self.penalty_scale = 1.0

        self.action_space = gym.spaces.Box(-1.0, 1.0, (12,), np.float32)
        self.floor_sensor = floor_sensor
        self.tof = None
        self.floor_heights = np.zeros(tof.ZONES ** 2)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBS_SIZE_TOF if floor_sensor else OBS_SIZE,), np.float32)

    # ---------------------------------------------------------------- helpers
    def _sense(self, name, n):
        a = self.sensor[name]
        return self.data.sensordata[a:a + n].copy()

    def _sample_command(self):
        if self.fixed_command is not None:
            return self.fixed_command.copy()
        r = self.np_random
        kind = r.choice(len(COMMAND_MIX), p=[c[0] for c in COMMAND_MIX])
        cmd = np.array([r.uniform(*rng) for rng in COMMAND_MIX[kind][1:]])
        if kind == 2:   # turning in place: keep it a real turn, not a twitch
            cmd[2] = np.sign(cmd[2] or 1) * max(abs(cmd[2]), 0.3)
        if kind == 3:
            cmd[1] = np.sign(cmd[1] or 1) * max(abs(cmd[1]), 0.04)
        return cmd

    def _gait_score(self, touching):
        """Moving: share of swing-phase feet actually in the air (freezing scores 0).
        Standing: share of feet on the ground."""
        expected = self.expected_contact()
        if expected.all():
            return float(np.mean(touching))
        swing = ~expected
        return float(np.mean(~touching[swing]))

    def expected_contact(self):
        """Which feet the trot schedule wants on the ground right now."""
        if np.linalg.norm(self.command) < 0.05:
            return np.ones(4, bool)
        phase = np.sin(self.phase)
        return np.where(TROT_PAIR == 0, phase >= 0, phase < 0)

    def _randomize_physics(self):
        m = self.model
        m.body_mass[:] = self.base_mass
        m.geom_friction[:] = self.base_friction
        m.actuator_gainprm[:, 0] = self.base_kp
        m.actuator_biasprm[:, 1] = -self.base_kp
        m.actuator_forcerange[:] = self.base_force
        if not self.randomize:
            return
        r = self.np_random
        m.body_mass[1] *= r.uniform(0.8, 1.3)            # body: PSU / battery uncertainty
        m.body_mass[2:] *= r.uniform(0.9, 1.1, m.nbody - 2)
        m.geom_friction[:, 0] = self.base_friction[:, 0] * r.uniform(0.6, 1.3)
        kp = self.base_kp * r.uniform(0.8, 1.2, m.nu)
        m.actuator_gainprm[:, 0] = kp
        m.actuator_biasprm[:, 1] = -kp
        m.actuator_forcerange[:] = self.base_force * r.uniform(0.85, 1.05, (m.nu, 1))

    def ground(self, x, y):
        """Terrain height (m) under world point (x, y)."""
        return terrain.height_at(self.field, x, y, *self.field_shape, *self.field_half, self.field_max)

    def set_penalty_scale(self, scale):
        self.penalty_scale = float(scale)

    def _update_curriculum(self):
        """Harder terrain after walking most of a real command; easier after a fall or a stall."""
        if self.fixed_level is not None or self._episodes == 0:
            return
        if self._fell or (self._asked > 0.5 and self._walked < 0.3 * self._asked):
            self.level -= 0.1
        elif self._asked > 0.5 and self._walked >= 0.6 * self._asked:
            self.level += 0.1
        self.level = float(np.clip(self.level, 0.0, 1.0))

    def _stumbles(self, touching):
        """Touching feet pressed against a riser: ground more than STUMBLE_RISE higher right next to
        the foot than under it. (By geometry: MuJoCo's sphere-heightfield contacts have tilted normals
        even on flat ground, so contact forces can't tell a riser from the floor.)"""
        if not touching.any():
            return np.zeros(4, bool)
        d = self.data
        px, py = d.geom_xpos[self.feet, 0], d.geom_xpos[self.feet, 1]
        reach = self.foot_radius + 0.02
        ring = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        xs = (px[:, None] + reach[:, None] * np.cos(ring)).ravel()
        ys = (py[:, None] + reach[:, None] * np.sin(ring)).ravel()
        around = terrain.heights_at(self.field, xs, ys, *self.field_shape, *self.field_half, self.field_max).reshape(4, -1)
        under = terrain.heights_at(self.field, px, py, *self.field_shape, *self.field_half, self.field_max)
        return touching & (around.max(1) - under > STUMBLE_RISE)

    def _foot_contacts(self):
        touching = np.zeros(4, bool)
        n = self.data.ncon
        if n == 0:
            return touching, False, False
        g1, g2 = self.data.contact.geom1[:n], self.data.contact.geom2[:n]
        other = np.where(g1 == self.floor, g2, np.where(g2 == self.floor, g1, -1))
        roles = self.geom_role[other[other >= 0]]
        touching[roles[roles > 0] - 1] = True
        return touching, bool(np.any(roles == -2)), bool(np.any(roles == -1))

    def _obs(self):
        gravity = projected_gravity(self._sense("orientation", 4))
        gyro = self._sense("gyro", 3)
        joints = self.data.qpos[7:].copy()
        if self.randomize:
            gravity += self.np_random.normal(0, 0.03, 3)
            gyro += self.np_random.normal(0, 0.1, 3)
            joints += self.np_random.normal(0, 0.02, 12)
        floor = None
        if self.floor_sensor:
            if self.tof is None:
                self.tof = tof.ToFModel(self.model, self.np_random)
            g = projected_gravity(self._sense("orientation", 4))
            floor = tof.heights(self.tof.update(self.data, self.t), g)
            self.floor_heights = floor
        return observation(gravity, gyro, self.command, joints, self.last_action, self.phase, floor)

    # ---------------------------------------------------------------- gym API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._randomize_physics()
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        self._update_curriculum()
        yaw = self.np_random.uniform(-np.pi, np.pi) if self.randomize else 0.0
        # stair practice: in training, some episodes put a flight of stairs straight ahead and walk him at it
        self.stair_practice = (self.fixed_level is None and self.fixed_command is None
                               and self.np_random.random() < STAIR_PRACTICE)
        # Train on a mix: half the episodes at this robot's curriculum level, half at an easier random one.
        # With every episode at the top level, training soon happened almost only on the hardest clutter
        # and the walker grew slow and cautious everywhere (worse on flat ground and plain stairs).
        level = self.level
        if self.fixed_level is None and self.np_random.random() < EASIER_SHARE:
            level = self.np_random.uniform(0.0, self.level)
        self.field = terrain.generate(self.np_random, level, *self.field_shape, *self.field_half, self.field_max,
                                      ahead_yaw=yaw if self.stair_practice else None)
        m.hfield_data[:] = self.field.ravel()
        self._terrain_dirty = True
        noise = self.np_random.uniform(-0.05, 0.05, 12) if self.randomize else 0
        d.qpos[7:] = STAND + noise
        d.ctrl[:] = d.qpos[7:]
        d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        mujoco.mj_forward(m, d)
        low = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.feet)
        d.qpos[2] -= low - 0.002
        mujoco.mj_forward(m, d)
        self.t = 0.0
        self.phase = 0.0
        self.steps = 0
        if self.tof is not None:
            self.tof.next = 0.0
        self.command = self._sample_command()
        if self.stair_practice:
            self.command = np.array([self.np_random.uniform(0.15, 0.3), 0.0, 0.0])
        self.anchor = d.qpos[:2].copy()   # pivot point for turn-in-place commands
        self.last_action = np.zeros(12)
        self.prev_action = np.zeros(12)
        self.air_time = np.zeros(4)
        self.last_land_xy = d.geom_xpos[self.feet, :2].copy()
        self.last_land_level = np.full(4, np.nan)
        self.stance_time = np.zeros(4)
        self.swing_peak = np.zeros(4)         # highest clearance of each foot's current swing
        self.last_peak = np.full(4, np.nan)   # peak of each foot's last completed step
        self.vel_avg = np.zeros(3)   # vx, vy, yaw rate, averaged over ~one stride
        self.prev_world_vel = d.qvel[:3].copy()
        self.target_height = d.qpos[2]          # spawn pad is flat, so this is height above ground
        self.next_push = self.np_random.uniform(*PUSH_EVERY)
        self._episodes += 1
        self._fell, self._walked, self._asked = False, 0.0, 0.0
        return self._obs(), {}

    def step(self, action):
        m, d = self.model, self.data
        action = np.clip(np.asarray(action, dtype=float), -1, 1)
        d.ctrl[:] = action_to_targets(action)
        power = torque_sq = over_torque = 0.0
        for _ in range(self.frame_skip):
            mujoco.mj_step(m, d)
            power += np.abs(d.actuator_force * d.qvel[6:]).sum()
            torque_sq += np.sum(d.actuator_force ** 2)   # static strain heats a hobby servo too
            over_torque += np.sum(np.maximum(np.abs(d.actuator_force) - SOFT_TORQUE, 0.0))
        power /= self.frame_skip
        torque_sq /= self.frame_skip
        over_torque /= self.frame_skip
        self.t += CONTROL_DT
        self.phase = advance_phase(self.phase, self.command)
        self.steps += 1

        if self.pushes and self.t >= self.next_push:   # shove: sideways and a twist
            d.qvel[:2] += self.np_random.uniform(-PUSH_SPEED, PUSH_SPEED, 2)
            d.qvel[5] += self.np_random.uniform(-PUSH_SPIN, PUSH_SPIN)
            self.next_push = self.t + self.np_random.uniform(*PUSH_EVERY)
        if self.fixed_command is None and self.steps % 250 == 0:
            self.command = self._sample_command()
            self.anchor = d.qpos[:2].copy()

        q = self._sense("orientation", 4)
        Rt = quat_matrix(q).T                               # world -> body
        gravity = -Rt[:, 2]
        vel = Rt @ d.qvel[:3]                               # base linear velocity, body frame
        ang = Rt @ d.qvel[3:6]                              # base angular velocity, body frame
        touching, leg_hit, base_hit = self._foot_contacts()
        body_acc = (d.qvel[:3] - self.prev_world_vel) / CONTROL_DT   # averaged over the control step
        self.prev_world_vel = d.qvel[:3].copy()
        a = CONTROL_DT / TRACK_TAU
        self.vel_avg += a * (np.array([vel[0], vel[1], ang[2]]) - self.vel_avg)
        asked = np.linalg.norm(self.command[:2])
        if asked > 0:   # progress along the command only: jiggling and shoves don't count
            self._walked += max(float(vel[:2] @ self.command[:2]) / asked, 0.0) * CONTROL_DT
        self._asked += asked * CONTROL_DT

        first_contact = touching & (self.air_time > 0)
        turning = (abs(self.command[0]) < 0.02 and abs(self.command[1]) < 0.02 and abs(self.command[2]) > 0.1)
        moving = np.linalg.norm(self.command) > 0.05
        feet_air = np.sum((self.air_time - 0.2) * first_contact) * moving
        air_before = self.air_time.copy()
        self.air_time = np.where(touching, 0.0, self.air_time + CONTROL_DT)
        self.stance_time = np.where(touching, self.stance_time + CONTROL_DT, 0.0)
        foot_speed = np.zeros(4)
        for k, g in enumerate(self.feet):
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, g, self._vel6, 0)
            foot_speed[k] = np.hypot(self._vel6[3], self._vel6[4])
        # terrain under the four feet and the body in one lookup
        px = np.append(d.geom_xpos[self.feet, 0], d.qpos[0])
        py = np.append(d.geom_xpos[self.feet, 1], d.qpos[1])
        ground = terrain.heights_at(self.field, px, py, *self.field_shape, *self.field_half, self.field_max)
        clearance = d.geom_xpos[self.feet, 2] - self.foot_radius - ground[:4]
        foot_low = (np.maximum(FOOT_CLEARANCE - clearance, 0.0) / FOOT_CLEARANCE) ** 2
        # landings: onto a new step level, or back down where the foot last landed
        land_xy = d.geom_xpos[self.feet, :2]
        new_level = first_contact & (np.abs(ground[:4] - self.last_land_level) > STEP_LEVEL)
        retouch = first_contact & (np.hypot(*(land_xy - self.last_land_xy).T) < RETOUCH_DIST)
        self.last_land_xy = np.where(first_contact[:, None], land_xy, self.last_land_xy)
        self.last_land_level = np.where(first_contact, ground[:4], self.last_land_level)
        # step quality, scored once per touchdown (events, like turn_footfall)
        self.swing_peak = np.where(touching, self.swing_peak, np.maximum(self.swing_peak, clearance))
        real_step = first_contact & (air_before >= BOUNCE_AIR)
        bounce = first_contact & (air_before < BOUNCE_AIR)
        tap = real_step & (self.swing_peak < TAP_HEIGHT)
        swing_height = float(np.sum(np.exp(-((self.swing_peak - SWING_HEIGHT) / SWING_SIGMA) ** 2) * real_step)) if moving else 0.0
        self.last_peak = np.where(real_step, self.swing_peak, self.last_peak)
        complete = not np.isnan(self.last_peak).any()
        swing_spread = float(np.std(self.last_peak)) * float(np.sum(real_step)) if complete and moving else 0.0
        self.swing_peak = np.where(touching, 0.0, self.swing_peak)

        terms = {
            # tolerance grows a little with commanded speed
            "track_lin": np.exp(-np.sum((self.command[:2] - self.vel_avg[:2]) ** 2)
                                / (0.02 + 0.06 * float(self.command[:2] @ self.command[:2]))),
            "run_progress": (float(np.clip((self.vel_avg[:2] @ self.command[:2]) / (asked ** 2), 0.0, 1.0))
                             if asked > WALK_SPEED else 0.0),
            "track_yaw": np.exp(-(self.command[2] - self.vel_avg[2]) ** 2 / 0.1),
            "lin_z": vel[2] ** 2,
            "ang_xy": np.sum(ang[:2] ** 2),
            "tilt": np.sum(gravity[:2] ** 2),
            "height": (d.qpos[2] - ground[4] - self.target_height) ** 2,
            "power": power,
            "torque": torque_sq,
            "action_rate": np.sum((action - self.last_action) ** 2),
            "action_smooth": np.sum((action - 2 * self.last_action + self.prev_action) ** 2),
            "joint_acc": np.sum(d.qacc[6:] ** 2),
            "joint_vel": np.sum(d.qvel[6:] ** 2),
            "joint_limit": np.sum(np.maximum(np.abs(d.qpos[7:]) - SOFT_JOINT_LIMIT, 0.0)),
            "torque_limit": over_torque,
            "feet_air": feet_air,
            "gait": self._gait_score(touching),   # turning in place uses the slower TURN_PERIOD stride
            "turn_progress": float(np.clip(self.vel_avg[2] / self.command[2], 0.0, 1.0)) if turning else 0.0,
            "turn_anchor": np.exp(-float(np.sum((d.qpos[:2] - self.anchor) ** 2)) / TURN_ANCHOR_SIGMA) if turning else 0.0,
            # an event, not a rate: with the CONTROL_DT scaling each footfall costs 5 * 0.02 = 0.1
            "turn_footfall": float(np.sum(first_contact)) if turning else 0.0,
            "turn_drift": float(np.hypot(d.qvel[0], d.qvel[1])) if turning else 0.0,   # body sliding off the pivot
            "turn_slip": float(np.sum(foot_speed * touching)) if turning else 0.0,   # feet dragged round in a pivot
            "turn_planted": float(np.sum(np.maximum(self.stance_time - TURN_MAX_STANCE, 0.0))) if turning else 0.0,
            "turn_offset": float(np.linalg.norm(d.qpos[:2] - self.anchor)) if turning else 0.0,
            "swing_height": swing_height,
            "swing_spread": swing_spread,
            "foot_bounce": float(np.sum(bounce)),
            "tap": float(np.sum(tap)) if moving else 0.0,
            "stair_step": float(np.sum(new_level)),
            "retouch": float(np.sum(retouch)) if asked > 0.1 else 0.0,
            "stair_progress": (float(np.clip((self.vel_avg[:2] @ self.command[:2]) / max(asked ** 2, 1e-4), -1.0, 1.0))
                               if self.stair_practice and asked > 0.05 else 0.0),
            "stair_jitter": float(np.sum((action - self.last_action) ** 2)) if self.stair_practice else 0.0,
            "steady": np.exp(-(ang[0] ** 2 + ang[1] ** 2) / STEADY_SIGMA),
            "smooth_ride": np.exp(-float(body_acc @ body_acc) / RIDE_SIGMA),
            "slip": np.sum(foot_speed * touching),
            "stumble": float(np.sum(self._stumbles(touching))),
            "foot_clearance": np.sum(foot_low * np.tanh(5 * foot_speed) * ~touching),
            "leg_contact": float(leg_hit),
            "alive": 1.0,
        }
        reward = sum(REWARD[k] * v * (self.penalty_scale if k in SHAPING else 1.0)
                     for k, v in terms.items()) * CONTROL_DT
        self.prev_action = self.last_action
        self.last_action = action

        terminated = bool(gravity[2] > -0.5 or base_hit)    # tipped past 60 deg or belly on the ground
        truncated = self.steps >= self.max_steps
        self._fell = self._fell or terminated
        if self._stream is not None:
            packet = np.concatenate([d.qpos, self.command, vel[:2], [ang[2], reward, float(terminated)]])
            try:
                self._stream.sendto(struct.pack(f"{packet.size}d", *packet), ("127.0.0.1", STREAM_PORT))
            except OSError:
                pass
        info = {"terms": terms, "command": self.command.copy(), "vel": vel.copy(), "yaw_rate": float(ang[2]),
                "torque": float(np.abs(d.actuator_force).max()), "terrain_level": self.level}
        return self._obs(), float(reward), terminated, truncated, info

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 720, 1280)
        if self._terrain_dirty:
            mujoco.mjr_uploadHField(self.model, self._renderer._mjr_context, 0)
            self._terrain_dirty = False
        self._renderer.update_scene(self.data, camera="track")
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        if self._stream is not None:
            self._stream.close()
            self._stream = None
