"""What the navigator sees and does, shared by training (nav_env.py), the Webots bridge and the robot.

numpy only. Frames: body frame x forward, y left, metres.
"""

import numpy as np

from lidar import NAV_RANGE

NAV_DT = 0.2                                   # navigator decision period (s)
GOAL_RADIUS = 0.3                              # m from the body centre: the marker is at the rock's nose
# Command limits: what the walker was trained on (vx, vy m/s, yaw rate rad/s).
# Turn rate up to 1.0 rad/s since the fast-turn walker (navigators before nav_lidar were trained at 0.5).
CMD_LOW = np.array([-0.12, -0.08, -1.0])
CMD_HIGH = np.array([0.35, 0.08, 1.0])        # up to running speed (spec.RUN_SPEED)
NAV_OBS = 14                                   # camera-only navigators (rocky_nav*.npz)
LIDAR_SECTORS = 36                             # 10 degree lidar sectors for the navigator
NAV_OBS_LIDAR = NAV_OBS + LIDAR_SECTORS
FLOOR_ZONES = 64                               # nose sensor distances (tof.py; 0 = no return)
NAV_OBS_FLOOR = NAV_OBS_LIDAR + FLOOR_ZONES

# Obstacle pool for training in MuJoCo (make_mjcf.py builds them as movable bodies, nav_env.py lays
# them out each episode): box half-sizes (x, y, z), m. All reach above the lidar's scan plane.
OBSTACLES = [(1.0, 0.03, 0.30)] * 3 + [(0.5, 0.03, 0.30)] * 3 + [(0.15, 0.15, 0.25)] * 5 + [(0.30, 0.20, 0.25)] * 3


def goal_in_body(goal_xy, base_xy, quat):
    """Goal relative to the robot, in its heading frame (x forward, y left). quat is (w, x, y, z)."""
    w, x, y, z = quat
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    d = np.asarray(goal_xy, dtype=float) - np.asarray(base_xy, dtype=float)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])


def nav_observation(estimate, visible, age, last_cmd, yaw_rate, gravity, lidar=None, floor=None):
    """estimate: last camera detection of the target (body frame, m); age 0..1; last_cmd normalised.
    lidar: optional lidar.N_BINS ranges (m); appended as LIDAR_SECTORS min-pooled sectors scaled 0..1."""
    dist = float(np.linalg.norm(estimate))
    bearing = np.arctan2(estimate[1], estimate[0]) if dist > 0 else 0.0
    obs = [np.clip(estimate, -3, 3), [min(dist, 3.0), np.cos(bearing), np.sin(bearing)],
           [visible, age], last_cmd, [yaw_rate], gravity]
    if lidar is not None:
        obs.append(lidar_sectors(lidar) / NAV_RANGE)
        if floor is not None:
            obs.append(np.asarray(floor, dtype=float))   # nose sensor distances, metres (tof.encode)
    return np.concatenate(obs).astype(np.float32)


def lidar_sectors(bins):
    """lidar.N_BINS ranges -> LIDAR_SECTORS (nearest in each), metres, CCW from straight behind."""
    b = np.minimum(np.asarray(bins, dtype=float), NAV_RANGE)
    return b.reshape(LIDAR_SECTORS, -1).min(1)


SECTOR_ANGLES = -np.pi + (np.arange(LIDAR_SECTORS) + 0.5) * 2 * np.pi / LIDAR_SECTORS


# Scripted navigator (the default): spin to find the target, turn in place to face it, then go
# straight, running when it's far. With the walk/run walker this beats the trained navigators.
SEARCH_TURN = 0.8        # rad/s while the target isn't in view
FACE_TOLERANCE = 0.25    # rad: turn in place until the target is within this of straight ahead
FACE_TURN, FACE_GAIN = (0.4, 1.0), 2.0   # rad/s: turn fast when far off, slowing as it lines up
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
        return np.array([0.0, 0.0, float(np.clip(FACE_GAIN * abs(bearing), *FACE_TURN)) * np.sign(bearing)])
    vx = RUN_CMD if dist > RUN_DISTANCE else WALK_CMD
    return np.array([vx, 0.0, float(np.clip(STEER_GAIN * bearing, -STEER_MAX, STEER_MAX))])


# Obstacle avoidance from the lidar (when the observation has it)
BLOCKED = 0.55           # m: something this close ahead (+-35 deg) stops a forward walk
CLEAR = 0.85             # m: turn until the way ahead (+-30 deg) is this clear...
DETOUR_TIME = 1.5        # s: ...then walk on for this long before heading for the target again
SIDE_NEAR = 0.35         # m: steer gently away from things this close to either side
AVOID_TURN = 0.8         # rad/s
# Stuck: walking at a target in sight that gets no closer, e.g. against a low step the lidar
# can't see (it only scans ~36 cm up). Then turn away ~90 degrees and walk on before trying again.
STUCK_TIME = 2.4         # s of walking at the target with...
STUCK_GAIN = 0.08        # ...less than this much (m) of progress
ESCAPE_TURN_TIME = 1.6   # s of turning at AVOID_TURN (~75 degrees)


# Safety layer: the legs reach ~0.35 m from the centre (the lidar), so judging walls by what's
# straight ahead isn't enough. Every command is checked against the lidar first.
FOOTPRINT = 0.31         # m: half-size of the square his body and feet stay inside...
FOOT_RADIUS = 0.38       # ...with its corners rounded off at this radius (the feet, at the diagonals,
                         # reach 0.27-0.35 m), so turning in place doesn't swing big corners out
SAFE_MARGIN = 0.08       # m kept between that shape and anything the lidar sees (walker drift)
PREDICT = 0.6            # s ahead (navigator period, walker response, a little slack)


def _margin(points, vx, wz, vy=0.0):
    """Smallest gap (m) between obstacle points (body frame) and the footprint over the next
    PREDICT s of (vx, vy, wz): checked along the way, since a turn swings the corners out mid-turn."""
    return min(_margin_at(points, vx, wz, t, vy) for t in (0.0, 0.25, 0.5, 0.75, 1.0))


def _margin_at(points, vx, wz, frac, vy=0.0):
    """Gap (m) between obstacle points (body frame) and the footprint (a square with rounded
    corners) after frac * PREDICT s. A point clears the shape if it clears the square or the circle."""
    if len(points) == 0:
        return np.inf
    a = wz * PREDICT * frac
    if abs(wz) > 1e-6:
        r = vx / wz
        dx, dy = r * np.sin(a), r * (1 - np.cos(a))
    else:
        dx, dy = vx * PREDICT * frac, 0.0
    dy += vy * PREDICT * frac
    rel = points - [dx, dy]
    c, s = np.cos(a), np.sin(a)
    lx, ly = c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]
    square = np.hypot(np.maximum(np.abs(lx) - FOOTPRINT, 0), np.maximum(np.abs(ly) - FOOTPRINT, 0))
    square = np.where((np.abs(lx) < FOOTPRINT) & (np.abs(ly) < FOOTPRINT), -1.0, square)
    circle = np.hypot(lx, ly) - FOOT_RADIUS
    return float(np.min(np.maximum(square, circle)))


def _real_speed(vx):
    """Forward speed the walker really reaches for a command (measured; ~20 % going backwards)."""
    return float(np.interp(vx, SPEED_CMD, SPEED_REAL)) if vx > 0 else 0.2 * vx


def safe_command(cmd, sectors):
    """The command itself if the footprint stays clear for PREDICT s, else the closest safe one."""
    if sectors is None:
        return cmd
    real = _real_speed   # predict with what the walker actually does (it barely walks backwards)
    near = sectors < NAV_RANGE - 1e-3
    pts = np.stack([sectors[near] * np.cos(SECTOR_ANGLES[near]), sectors[near] * np.sin(SECTOR_ANGLES[near])], 1)
    vx, vy, wz = cmd
    now = _margin_at(pts, 0.0, 0.0, 0.0)
    left = np.sum(np.minimum(sectors[(SECTOR_ANGLES > 0) & (SECTOR_ANGLES < np.pi)], 2))
    right = np.sum(np.minimum(sectors[(SECTOR_ANGLES < 0) & (SECTOR_ANGLES > -np.pi)], 2))
    open_side = 1.0 if left >= right else -1.0
    options = [(vx, wz), (0.5 * vx, wz)] + ([(0.0, wz)] if abs(wz) > 0.1 else []) + \
              [(0.0, AVOID_TURN * open_side), (0.0, -AVOID_TURN * open_side), (-0.10, 0.0), (0.0, 0.0)]
    if now >= SAFE_MARGIN:
        for cvx, cwz in options:
            if _margin_at(pts, real(cvx), cwz, 1.0) >= SAFE_MARGIN and _margin(pts, real(cvx), cwz) >= SAFE_MARGIN:
                return np.array([cvx, vy if cvx == vx else 0.0, cwz])
        return np.zeros(3)
    # already too close: whatever gets him out fastest (stepping sideways away, backing off, or
    # turning the corners away). Standing still isn't safe either: the walker drifts.
    escapes = [(0.0, 0.08, 0.0), (0.0, -0.08, 0.0), (0.15, 0.0, 0.0), (-0.10, 0.0, 0.0),
               (0.0, 0.0, 0.6), (0.0, 0.0, -0.6), (0.15, 0.0, 0.5), (0.15, 0.0, -0.5)]
    gains = [_margin_at(pts, real(cvx), cwz, 1.0, 0.8 * cvy) for cvx, cvy, cwz in escapes]
    best = int(np.argmax(gains))
    return np.array(escapes[best]) if gains[best] > now + 0.005 else np.zeros(3)


def mask_post(sectors, x, y):
    """Hide the target post (at body-frame x, y) from obstacle checks, but only at its own
    distance: a wall right behind a goal must stay visible."""
    dist = float(np.hypot(x, y))
    if dist <= 0:
        return sectors
    r = sectors.copy()
    near = np.abs(np.angle(np.exp(1j * (SECTOR_ANGLES - np.arctan2(y, x))))) < np.radians(12)
    r[near & (r > dist - 0.25) & (r < dist + 0.15)] = NAV_RANGE
    return r


class StuckEscape:
    """Shared by the scripted and hybrid navigators: notices a forward walk that isn't getting
    the (visible) target any closer, and overrides the command with a turn-away and a detour."""

    def __init__(self):
        self.history = []            # target distances while walking at it
        self.turn_for = self.walk_for = 0.0
        self.dir = 1.0

    def __call__(self, obs, cmd, sectors=None):
        if self.turn_for > 0:
            self.turn_for -= NAV_DT
            return np.array([0.0, 0.0, AVOID_TURN * self.dir])
        if self.walk_for > 0:
            self.walk_for -= NAV_DT
            return np.array([WALK_CMD, 0.0, 0.0])
        visible, dist = obs[5] > 0.5, obs[2]
        if cmd[0] > 0.05 and visible and dist > 0:
            self.history.append(dist)
        else:
            self.history = []
        n = int(round(STUCK_TIME / NAV_DT))
        if len(self.history) >= n and self.history[-n] - self.history[-1] < STUCK_GAIN:
            self.history = []
            if sectors is not None:   # toward the more open side
                left = sectors[(SECTOR_ANGLES > 0) & (SECTOR_ANGLES < np.pi / 2)]
                right = sectors[(SECTOR_ANGLES < 0) & (SECTOR_ANGLES > -np.pi / 2)]
                self.dir = 1.0 if np.sum(np.minimum(left, 2)) >= np.sum(np.minimum(right, 2)) else -1.0
            else:
                self.dir = -self.dir
            self.turn_for, self.walk_for = ESCAPE_TURN_TIME, DETOUR_TIME
            return np.array([0.0, 0.0, AVOID_TURN * self.dir])
        return cmd


class ScriptedNavigator:
    """scripted_command plus a search that explores, and, with lidar, obstacle avoidance.

    Search: if a full turn goes by without a sighting (the target may be out of camera range),
    walk forward a little and look again. Avoidance: when a wall or box blocks the way, turn in
    place toward the more open side until the way ahead is clear, walk on for DETOUR_TIME, then
    go back to the target. Call it every NAV_DT with the navigator observation."""

    SPIN_BEFORE_EXPLORE = 2 * np.pi / 0.65      # s: about one full turn at the walker's real turn rate
    EXPLORE_TIME = 4.0                           # s of running forward (~1.4 m) before looking again

    def __init__(self):
        self.lost_for = 0.0
        self.exploring_for = 0.0
        self.avoid_dir = 0.0        # +1 turning left round an obstacle, -1 right, 0 not avoiding
        self.detour_for = 0.0
        self.escape = StuckEscape()

    def _sectors(self, obs):
        """Lidar sector ranges (m), with the target post masked out, or None without lidar."""
        if len(obs) < NAV_OBS_LIDAR:
            return None
        r = np.array(obs[NAV_OBS:NAV_OBS + LIDAR_SECTORS], dtype=float) * NAV_RANGE
        return mask_post(r, obs[0], obs[1])

    def _go(self, obs):
        if obs[6] > 0.5 or obs[2] == 0:      # same "lost" test as scripted_command
            if self.exploring_for > 0:
                self.exploring_for -= NAV_DT
                return np.array([RUN_CMD, 0.0, 0.0])
            self.lost_for += NAV_DT
            if self.lost_for > self.SPIN_BEFORE_EXPLORE:
                self.lost_for, self.exploring_for = 0.0, self.EXPLORE_TIME
        else:
            self.lost_for = self.exploring_for = 0.0
        return scripted_command(obs)

    def __call__(self, obs):
        r = self._sectors(obs)
        return safe_command(self.escape(obs, self._command(obs, r), r), r)

    def _command(self, obs, r):
        if r is None:
            return self._go(obs)
        ang = np.abs(SECTOR_ANGLES)
        ahead = r[ang < np.radians(35)].min()
        left = r[(SECTOR_ANGLES > 0) & (SECTOR_ANGLES < np.pi / 2)]
        right = r[(SECTOR_ANGLES < 0) & (SECTOR_ANGLES > -np.pi / 2)]
        if self.avoid_dir:
            if r[ang < np.radians(30)].min() < CLEAR:
                return np.array([0.0, 0.0, AVOID_TURN * self.avoid_dir])
            self.avoid_dir, self.detour_for = 0.0, DETOUR_TIME
        cmd = self._go(obs) if self.detour_for <= 0 else np.array([WALK_CMD, 0.0, 0.0])
        self.detour_for -= NAV_DT
        if cmd[0] > 0 and ahead < BLOCKED:
            self.avoid_dir = 1.0 if np.sum(np.minimum(left, 2.0)) >= np.sum(np.minimum(right, 2.0)) else -1.0
            self.detour_for = 0.0
            return np.array([0.0, 0.0, AVOID_TURN * self.avoid_dir])
        if cmd[0] > 0:   # brushing past something: ease away from it
            side_l = r[(SECTOR_ANGLES > np.radians(35)) & (SECTOR_ANGLES < np.radians(100))].min()
            side_r = r[(SECTOR_ANGLES < -np.radians(35)) & (SECTOR_ANGLES > -np.radians(100))].min()
            if min(side_l, side_r) < SIDE_NEAR:
                cmd = cmd.copy()
                cmd[2] += -0.3 if side_l < side_r else 0.3
        return cmd


class HybridNavigator:
    """A trained navigator while the target is in sight (or was seen in the last 1.5 s), and the
    scripted search when it's lost: the trained ones never learned to explore (in training the
    target nearly always comes into view within a turn), so behind a wall they just spin. The
    scripted search turns once, then walks off round obstacles and looks again.
    policy: obs -> normalised action (a NumpyPolicy). Returns velocity commands, like ScriptedNavigator."""

    def __init__(self, policy):
        self.policy, self.search, self.escape = policy, ScriptedNavigator(), StuckEscape()

    def __call__(self, obs):
        lost = obs[6] > 0.5 or obs[2] == 0          # same "lost" test as scripted_command
        search_cmd = self.search(obs)               # always called, so its search keeps its place
        if lost:
            return search_cmd                       # already through the safety layer
        r = self.search._sectors(obs)
        return safe_command(self.escape(obs, action_to_command(self.policy(obs)), r), r)


def command_to_action(cmd):
    return 2 * (np.asarray(cmd, dtype=float) - CMD_LOW) / (CMD_HIGH - CMD_LOW) - 1


def action_to_command(action):
    a = np.clip(np.asarray(action, dtype=float), -1, 1)
    return CMD_LOW + (a + 1) / 2 * (CMD_HIGH - CMD_LOW)


# ---------------------------------------------------------------- planner (the default with a lidar)
# Real speed of the walker for each forward command (measured, policies/rocky_walk.npz), for dead reckoning
SPEED_CMD = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.35]
SPEED_REAL = [0.0, 0.012, 0.094, 0.249, 0.293, 0.316, 0.329]   # walk_rounds 128M; Webots is ~10-25 % slower
HEADINGS = -np.pi + np.arange(72) * 2 * np.pi / 72
LANE = FOOTPRINT + SAFE_MARGIN + 0.02   # m: corridor half-width, a bit wider than the safety layer allows
FACE = 0.3                    # rad: turn in place first if the way to go is further off than this
LOOK_SPIN = 2 * np.pi / 0.75  # s: one full look-around at SEARCH_TURN (he turns ~0.75 rad/s)
EXPLORE_LEG = 1.5             # m to walk before looking around again
FORGET_AFTER = 30.0           # s without a sighting before the remembered target is dropped
ARRIVED = 0.15                # m: by memory alone, this close counts as arrived
CLIMB_TIME = 4.0              # s of slow straight walking to try a step the lidar can't see
VISIT_CELL = 0.5              # m: grid for "been here" while exploring
EDGE_BONUS = 3.0              # explore toward wall ends (worth 3 m of open space)
SEARCH_COVERAGE = False       # search by what the camera hasn't looked at yet: measured worse (0.7 vs 0.9 hidden goals/min), so full spins
SEEN_CELL = 0.25              # m: grid of floor the camera has already looked over
CAM_HALF_FOV = np.radians(30) # a little inside the camera's 66 degrees
CAM_REACH = 4.0               # m the camera is trusted to spot the marker at (it can do ~5.9)
USE_FLOOR = True              # use the nose floor sensor (off: lidar only, for comparison)
CLIMBABLE = 0.08              # m: single jumps up to this he may try to climb (full joint range: feet lift ~7 cm;
                              # neighbouring zones often straddle two 4.5 cm risers); bigger are walls
CLIFF = -0.12                 # m: single drops deeper than this are avoided like walls (-0.08 walled off the
                              # sides of a narrow staircase: its landing is 9 cm up)
STEEP = 0.4                   # rise per metre between neighbouring zones that counts as a face, not a staircase
                              # (0.25 walled off 4.5 cm stairs: two risers in one 0.3 m zone gap is 0.3)
WALL_JUMP = 0.3               # m: a jump this big is a wall however far apart the zones are (0.12 walled off a
                              # staircase seen from afar: its landing is 18 cm up; steepness already catches walls)
STEP_SEEN = 0.025             # m: a single rise this big ahead (in 2+ zones): slow down, take it straight (0.035 measured worse)
FLOOR_MEMORY = 0.0            # s to remember ledges/drops the floor sensor saw. Off: with dead-reckoned positions it
                              # measured worse (60 s: 5.4 -> 3.5 goals/min on flat floor; 8 s: 4.4)
FLOOR_MEMORY_REACH = 1.5      # m: only near ones
FLOOR_CELL = 0.1              # m grid for that memory
REMEMBER_LOW = 120.0          # s to remember a step he couldn't climb (the lidar can't see it)


def clear_length(points, heading, reach):
    """How far (m, up to reach) his footprint can travel along heading before something is in the way."""
    if len(points) == 0:
        return reach
    u = np.array([np.cos(heading), np.sin(heading)])
    along = points @ u
    side = np.abs(points @ np.array([-u[1], u[0]]))
    hit = (side < LANE) & (along > -0.05)
    if not hit.any():
        return reach
    return float(np.clip(along[hit].min() - FOOTPRINT - SAFE_MARGIN, 0.0, reach))


class PlannerNavigator:
    """Remember where the target is, and take the free way round to it.

    - Memory: every sighting is stored in an odometry frame (gyro for heading, the walker's real
      speed for distance), so a target hidden by a wall is still known, and its bearing stays
      right as he moves.
    - Steering: of 72 headings, take the free one nearest the target (his whole footprint must fit
      along it), which leads round the end of a wall instead of into it. Turn in place first if
      it's far off, then walk or run with gentle steering.
    - Search: with no target known, look all round, then walk off towards open space he hasn't
      visited yet, and look again.
    - Stuck (the scan stops changing while walking): if the way ahead looks clear, it's probably
      a step or stairs below the lidar's scan plane, so first try to climb it slowly; then turn away.
    - Every command goes through the lidar safety layer (safe_command).
    """

    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.target = None            # (x, y) in the odometry frame
        self.unseen = 0.0
        self.heading = 0.0
        self.mode, self.timer, self.walked = "look", LOOK_SPIN, 0.0
        self.visited = {}
        self.scans = []
        self.last_cmd = np.zeros(3)
        self.status = "starting"
        self.low = []                 # (x, y, forget at) of steps he couldn't climb, odometry frame
        self.clock = 0.0
        self.seen = set()             # floor cells the camera has looked over (odometry frame)
        self.dists = []               # distance to the visible target while walking at it (progress check)
        self.floor_mem = {}           # (i, j) cell -> forget at: ledges and drops the floor sensor saw
        self.climb_from = None        # target distance when a climb attempt began
        self.straight_at = False      # heading straight for the target (not going round)

    # ---------------------------------------------------------------- odometry and memory
    def _odometry(self, obs):
        self.yaw += float(obs[10]) * NAV_DT
        vx = float(self.last_cmd[0])
        speed = np.interp(vx, SPEED_CMD, SPEED_REAL) if vx > 0 else 0.2 * vx
        self.x += speed * np.cos(self.yaw) * NAV_DT
        self.y += speed * np.sin(self.yaw) * NAV_DT
        cell = (int(np.floor(self.x / VISIT_CELL)), int(np.floor(self.y / VISIT_CELL)))
        self.visited[cell] = self.visited.get(cell, 0) + 1

    def _to_world(self, bx, by):
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return np.array([self.x + c * bx - s * by, self.y + s * bx + c * by])

    def _to_body(self, p):
        dx, dy = p[0] - self.x, p[1] - self.y
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return np.array([c * dx + s * dy, -s * dx + c * dy])

    def _remember(self, obs):
        fresh = obs[5] > 0.5 and obs[6] * 3.0 < NAV_DT + 1e-6 and obs[2] > 0   # a new sighting this step
        if fresh:
            self.target, self.unseen = self._to_world(obs[0], obs[1]), 0.0
        else:
            self.unseen += NAV_DT
        if self.target is not None:
            rel = self._to_body(self.target)
            # "arrived" by memory alone only when practically on it: dead reckoning runs long when the
            # walker is slower than measured (Webots), and dropping it early sends him searching
            if self.unseen > FORGET_AFTER or (np.hypot(*rel) < ARRIVED and not fresh):
                self.target = None    # reached (or long gone): look for the next one
                self.seen = set()     # ...which could be anywhere, including places already looked at

    def _mark_low(self):
        """Remember an unclimbable step just ahead as a short invisible wall across his path."""
        for side in np.linspace(-0.4, 0.4, 5):
            self.low.append((*self._to_world(FOOTPRINT + 0.05, side), self.clock + REMEMBER_LOW))

    def _with_low(self, sectors):
        """Lidar sectors with the remembered steps added, as if the lidar could see them."""
        self.low = [o for o in self.low if o[2] > self.clock]
        r = sectors.copy()
        for x, y, _ in self.low:
            b = self._to_body((x, y))
            d = float(np.hypot(*b))
            if d < NAV_RANGE:
                k = int(np.argmin(np.abs(np.angle(np.exp(1j * (SECTOR_ANGLES - np.arctan2(b[1], b[0])))))))
                r[k] = min(r[k], d)
        return r

    def _edges(self, sectors, clear):
        """1 for headings that pass close by a wall end (the scan jumps from near to far between
        neighbouring sectors: unseen space behind it, where a hidden target would be), else 0."""
        jump = np.abs(np.diff(np.r_[sectors, sectors[0]])) > 1.0
        near = np.minimum(sectors, np.r_[sectors[1:], sectors[0]]) < 2.5
        edge_angles = (SECTOR_ANGLES + np.pi / LIDAR_SECTORS)[jump & near]
        out = np.zeros(len(HEADINGS))
        for a in edge_angles:
            d = np.abs(np.angle(np.exp(1j * (HEADINGS - a))))
            out = np.maximum(out, (d < np.radians(25)) * (clear >= 0.8))
        return out

    def _cell(self, bx, by):
        w = self._to_world(bx, by)
        return int(np.floor(w[0] / SEEN_CELL)), int(np.floor(w[1] / SEEN_CELL))

    def _look(self, sectors):
        """Mark what the camera covers now: its field of view, out to the nearest obstacle."""
        for a in np.linspace(-CAM_HALF_FOV, CAM_HALF_FOV, 7):
            k = int(np.argmin(np.abs(np.angle(np.exp(1j * (SECTOR_ANGLES - a))))))
            reach = min(sectors[k], CAM_REACH)
            for d in np.arange(0.3, reach + 1e-6, SEEN_CELL):
                self.seen.add(self._cell(d * np.cos(a), d * np.sin(a)))

    def _unseen(self, heading, sectors, near=0.5, far=CAM_REACH, half=np.radians(35)):
        """Floor cells within view of heading (out to the nearest obstacle) not yet looked over."""
        n = 0
        for a in heading + np.linspace(-half, half, 7):
            k = int(np.argmin(np.abs(np.angle(np.exp(1j * (SECTOR_ANGLES - a))))))
            reach = min(sectors[k], far)
            for d in np.arange(near, reach + 1e-6, SEEN_CELL * 2):
                n += self._cell(d * np.cos(a), d * np.sin(a)) not in self.seen
        return n

    def _floor(self, obs, target=None):
        """From the nose sensor: points (body frame) of steps too high to climb or drops too deep, as
        if the lidar could see them, and whether a climbable step lies just ahead."""
        if len(obs) < NAV_OBS_FLOOR or not USE_FLOOR:
            return np.zeros((0, 2)), False
        import tof
        d = np.asarray(obs[NAV_OBS_LIDAR:NAV_OBS_FLOOR], dtype=float)
        d = np.where(d > 0, d, np.inf)                    # 0 = no return
        h = tof.heights(d, obs[11:14])                    # floor height where each zone hit
        xy = tof.points(d)[:, :2]                         # and where that is (body frame)
        seen = np.isfinite(xy[:, 0])
        xy = np.where(seen[:, None], xy, 0.0)
        # judge each step, not the total height: the jump from the zone just nearer in the same column
        # (the nearest row against his own floor). A staircase is many small climbable jumps even if its
        # top is 10 cm up; a wall or kerb face is one big jump; a drop off a ledge one big negative one.
        H = np.where(seen, h, np.nan).reshape(8, 8)
        nearer = np.vstack([H[1:], np.zeros((1, 8))])          # row r's nearer neighbour is row r + 1
        nearer = np.where(np.isnan(nearer), 0.0, nearer)
        jump = (H - nearer).ravel()
        jump = np.where(np.isnan(jump), 0.0, jump)
        # and how steep: neighbouring zones can land a tread or two apart, so two 3 cm steps can read as
        # one 6 cm jump. A wall or ledge face rises within a short way; stairs rise gradually.
        XY = xy.reshape(8, 8, 2)
        near_xy = np.concatenate([XY[1:], np.repeat(tof.POS[None, None, :2], 8, 1)], 0)
        gap = np.maximum(np.hypot(*(XY - near_xy).transpose(2, 0, 1)).ravel(), 0.02)
        steep = np.abs(jump) / gap > STEEP
        blocked = seen & ((jump > WALL_JUMP) | (jump < -WALL_JUMP) | (steep & ((jump > CLIMBABLE) | (jump < CLIFF))))
        if target is not None:   # the target's own post is not a wall
            blocked &= np.hypot(xy[:, 0] - target[0], xy[:, 1] - target[1]) > 0.3   # remembered position drifts
        ahead = seen & (xy[:, 0] < 0.9) & (np.abs(xy[:, 1]) < 0.3)
        # a step only where nothing further out in that column is a wall (a wall's foot isn't a step)
        col_ok = ~np.tile((blocked.reshape(8, 8) & (jump.reshape(8, 8) > CLIMBABLE)).any(0), 8)
        step = int(np.sum(ahead & col_ok & (jump > STEP_SEEN) & (jump <= CLIMBABLE))) >= 2   # not one noisy zone
        self._floor_free = xy[seen & ~blocked]
        return xy[blocked], step

    def _remember_floor(self, blocked_xy):
        """Keep the floor sensor's ledges and drops in the odometry frame, so they still count once he
        has turned away (the sensor only looks ahead, the lidar never sees them); forget cells it now
        sees clear, and old ones."""
        if FLOOR_MEMORY <= 0:
            return np.asarray(blocked_xy).reshape(-1, 2)
        for x, y in getattr(self, "_floor_free", np.zeros((0, 2))):
            w = self._to_world(x, y)
            self.floor_mem.pop((int(np.floor(w[0] / FLOOR_CELL)), int(np.floor(w[1] / FLOOR_CELL))), None)
        for x, y in blocked_xy:
            if np.hypot(x, y) > FLOOR_MEMORY_REACH:
                continue
            w = self._to_world(x, y)
            self.floor_mem[(int(np.floor(w[0] / FLOOR_CELL)), int(np.floor(w[1] / FLOOR_CELL)))] = self.clock + FLOOR_MEMORY
        self.floor_mem = {c: t for c, t in self.floor_mem.items() if t > self.clock}
        out = []
        for (i, j) in self.floor_mem:
            b = self._to_body(((i + 0.5) * FLOOR_CELL, (j + 0.5) * FLOOR_CELL))
            if np.hypot(*b) < NAV_RANGE:
                out.append(b)
        return np.array(out).reshape(-1, 2)

    def _visited_along(self, heading, length=2.0):
        n = 0
        for d in np.arange(0.5, length + 1e-6, 0.5):
            cell = (int(np.floor((self.x + d * np.cos(self.yaw + heading)) / VISIT_CELL)),
                    int(np.floor((self.y + d * np.sin(self.yaw + heading)) / VISIT_CELL)))
            n += min(self.visited.get(cell, 0), 20)
        return n / 20.0

    # ---------------------------------------------------------------- steering
    def _drive_to(self, heading, dist, pts):
        """Turn toward heading (body frame), then walk/run along it."""
        if abs(heading) > FACE:
            return np.array([0.0, 0.0, float(np.clip(FACE_GAIN * abs(heading), *FACE_TURN)) * np.sign(heading)])
        ahead = clear_length(pts, heading, 3.0)
        vx = RUN_CMD if dist > RUN_DISTANCE and ahead > 1.2 else WALK_CMD
        return np.array([vx, 0.0, float(np.clip(1.2 * heading, -0.3, 0.3))])

    def _pick_heading(self, pts, want, need):
        """Free heading nearest `want` (or the most open if none), with a little hysteresis."""
        clear = np.array([clear_length(pts, h, need) for h in HEADINGS])
        free = clear >= need - 1e-6
        diff = np.abs(np.angle(np.exp(1j * (HEADINGS - want))))
        stick = np.abs(np.angle(np.exp(1j * (HEADINGS - self.heading))))
        if free.any():
            cost = np.where(free, diff + 0.25 * stick, np.inf)
            return float(HEADINGS[int(np.argmin(cost))]), True
        return float(HEADINGS[int(np.argmax(clear - 0.1 * diff))]), False

    def _no_progress(self, obs, cmd):
        """Walking at a target he can see, and it's getting no closer (e.g. pushing against a step).
        More reliable than the scan when he wobbles in place, which keeps the scan changing."""
        if cmd[0] > 0.05 and obs[5] > 0.5 and obs[2] > 0 and self.straight_at:
            self.dists.append(float(obs[2]))
        else:
            self.dists = []
        n = int(round(STUCK_TIME / NAV_DT))
        if len(self.dists) >= n and self.dists[-n] - self.dists[-1] < STUCK_GAIN:
            self.dists = []
            return True
        return False

    def _stuck(self, sectors, cmd):
        """Walking, but the scan isn't changing: he isn't actually moving."""
        if cmd[0] <= 0.05:
            self.scans = []
            return False
        self.scans.append(sectors.copy())
        n = int(round(STUCK_TIME / NAV_DT))
        if len(self.scans) < n:
            return False
        old, new = self.scans[-n], self.scans[-1]
        self.scans = self.scans[-n:]
        seen = (old < NAV_RANGE - 0.01) & (new < NAV_RANGE - 0.01)
        return seen.sum() >= 3 and float(np.median(np.abs(new[seen] - old[seen]))) < 0.03

    # ---------------------------------------------------------------- main
    def __call__(self, obs):
        self.clock += NAV_DT
        self._odometry(obs)
        self._remember(obs)
        raw = self._with_low(np.array(obs[NAV_OBS:NAV_OBS + LIDAR_SECTORS], dtype=float) * NAV_RANGE)
        rel = self._to_body(self.target) if self.target is not None else None
        sectors = mask_post(raw, *rel) if rel is not None else raw
        self._look(raw)
        floor_pts, step_ahead = self._floor(obs, rel)
        floor_pts = self._remember_floor(floor_pts)   # what it sees now, and has seen recently
        if rel is not None and len(floor_pts):      # never wall off the target itself
            floor_pts = floor_pts[np.hypot(floor_pts[:, 0] - rel[0], floor_pts[:, 1] - rel[1]) > 0.3]
        for x, y in floor_pts:   # steps too high and drops, into the sectors, like the remembered ones
            d = float(np.hypot(x, y))
            k = int(np.argmin(np.abs(np.angle(np.exp(1j * (SECTOR_ANGLES - np.arctan2(y, x)))))))
            sectors[k] = min(sectors[k], d)
        near = sectors < NAV_RANGE - 1e-3
        pts = np.stack([sectors[near] * np.cos(SECTOR_ANGLES[near]), sectors[near] * np.sin(SECTOR_ANGLES[near])], 1)

        cmd = None
        if self.mode in ("climb", "escape_turn", "escape_walk"):
            self.timer -= NAV_DT
            if self.mode == "climb":
                cmd = np.array([WALK_CMD, 0.0, 0.0])
                if self.timer <= 0:
                    closer = (self.climb_from is not None and obs[5] > 0.5 and obs[2] > 0
                              and obs[2] < self.climb_from - 0.03)
                    if self._stuck(sectors, cmd) or not closer:   # couldn't get up it: remember it, go round
                        self._mark_low()
                        self.mode, self.timer = "escape_turn", ESCAPE_TURN_TIME
                        self.status = "step too high: going round"
                    else:
                        self.mode = "go"
            elif self.mode == "escape_turn":
                cmd = np.array([0.0, 0.0, AVOID_TURN * self.escape_dir])
                if self.timer <= 0:
                    self.mode, self.timer = "escape_walk", DETOUR_TIME
            else:
                cmd = np.array([WALK_CMD, 0.0, 0.0])
                if self.timer <= 0:
                    self.mode = "go"
        if cmd is None and rel is not None:                          # go to the remembered target
            self.mode = "go"
            dist = float(np.hypot(*rel))
            want = float(np.arctan2(rel[1], rel[0]))
            self.heading, free = self._pick_heading(pts, want, min(dist, 1.0))
            cmd = self._drive_to(self.heading, dist, pts)
            self.straight_at = abs(np.angle(np.exp(1j * (self.heading - want)))) < 0.1
            self.status = (f"{'tracking' if obs[5] > 0.5 else 'remembering'} target {dist:.2f} m away"
                           + ("" if abs(self.heading - want) < 0.1 else ", going round"))
        elif cmd is None:                                            # search
            if self.mode != "explore" and self.mode != "look":
                self.mode, self.timer = "look", LOOK_SPIN
            if self.mode == "look":
                # turn to look wherever hasn't been seen yet, nearest first; when all round is seen, move on
                self.timer -= NAV_DT
                unseen = np.array([self._unseen(h, sectors, half=np.radians(15)) for h in HEADINGS[::3]])
                self.status = "looking for the target"
                if not SEARCH_COVERAGE:
                    cmd = np.array([0.0, 0.0, SEARCH_TURN])
                elif unseen.max() > 2 and self.timer > 0:   # turn toward the side with more left to see
                    left = sum(unseen[(HEADINGS[::3] > 0.2)]) >= sum(unseen[(HEADINGS[::3] < -0.2)])
                    cmd = np.array([0.0, 0.0, SEARCH_TURN * (1 if left else -1)])
                else:
                    self.timer = 0
                if self.timer <= 0:
                    clear = np.array([clear_length(pts, h, 3.0) for h in HEADINGS])
                    score = (np.minimum(clear, 3.0) - 1.5 * np.array([self._visited_along(h) for h in HEADINGS])
                             + EDGE_BONUS * self._edges(sectors, clear)
                             + (0.15 * np.array([self._unseen(h, sectors) for h in HEADINGS]) if SEARCH_COVERAGE else 0))
                    self.heading = float(HEADINGS[int(np.argmax(score))])
                    self.mode, self.walked = "explore", 0.0
                    cmd = self._drive_to(self.heading, 3.0, pts)
            else:
                self.status = "exploring"
                self.heading, free = self._pick_heading(pts, self.heading, 0.6)
                cmd = self._drive_to(self.heading, 3.0, pts)
                self.walked += np.interp(max(cmd[0], 0), SPEED_CMD, SPEED_REAL) * NAV_DT
                if self.walked > EXPLORE_LEG or not free:
                    self.mode, self.timer = "look", LOOK_SPIN
                self.heading -= float(obs[10]) * NAV_DT      # keep the explore heading fixed in the world
        if step_ahead:   # climbing a step the sensor says he can manage: slow progress is not being stuck
            self.dists = []
        if self.mode in ("go", "explore") and (self._stuck(sectors, cmd) or (not step_ahead and self._no_progress(obs, cmd))):
            self.scans, self.dists = [], []
            self.climb_from = float(obs[2]) if obs[5] > 0.5 and obs[2] > 0 else None
            open_ahead = clear_length(pts, 0.0, 0.8) >= 0.8
            left = np.sum(np.minimum(sectors[(SECTOR_ANGLES > 0) & (SECTOR_ANGLES < np.pi / 2)], 2))
            right = np.sum(np.minimum(sectors[(SECTOR_ANGLES < 0) & (SECTOR_ANGLES > -np.pi / 2)], 2))
            self.escape_dir = 1.0 if left >= right else -1.0
            if open_ahead:      # nothing the lidar can see: a step or stairs, try to climb
                self.mode, self.timer = "climb", CLIMB_TIME
                self.status = "climbing a step"
                cmd = np.array([WALK_CMD, 0.0, 0.0])
            else:
                self.mode, self.timer = "escape_turn", ESCAPE_TURN_TIME
        if step_ahead and cmd[0] > WALK_CMD:   # a climbable step coming up: take it at walking pace, straight
            cmd = np.array([WALK_CMD, 0.0, float(np.clip(cmd[2], -0.15, 0.15))])
            self.status = "step ahead: climbing it"
        cmd = safe_command(cmd, sectors)
        self.last_cmd = cmd
        return cmd
