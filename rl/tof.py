"""The floor depth sensor in Bob's nose (SparkFun Qwiic ToF Imager, VL53L5CX): what the walker and the
planner get from it, the same in MuJoCo, Webots and on the robot.

The sensor gives 8 x 8 distances over ~45 x 45 deg, looking TILT down from the bottom of the nose
(cad/rocky_bob/body.py: tof_point). Each zone's distance becomes the HEIGHT of the floor where that
zone lands, relative to the floor he's standing on (using the IMU's gravity to undo body tilt):
flat floor reads ~0, a 4 cm step +0.04, a drop -0.04. That's what a step looks like to him, and it
doesn't depend on how far away it is. Rows run from the far edge of the view (row 0) to the near
edge, columns from left to right.

numpy only (mujoco for ToFModel).
"""

import numpy as np

POS = np.array([0.1022, 0.0, 0.013])   # m, body frame (cad/rocky_bob/body.py tof_point)
TILT = np.radians(24.0)                # looks this far below horizontal
FOV = np.radians(45.0)                 # per axis (63 deg diagonal)
ZONES = 8
MIN_RANGE, MAX_RANGE = 0.02, 4.0
HEIGHT = 0.203                          # m: sensor above the floor when standing (base 0.19 m + 13 mm)
PERIOD = 0.066                          # s: 15 Hz at 8 x 8
RANGE_NOISE = (0.008, 0.01)             # sigma = a + b * distance (m)
DROPOUT = 0.03                          # chance a zone returns nothing
CLIP = 0.25                             # m: heights clipped to +-this

# Zone directions like a camera's pixels (the real sensor's zones and Webots' RangeFinder both are):
# evenly spaced on an image plane 1 m ahead, rows top (far) to bottom (near), columns left to right.
_t = np.tan(FOV / 2) * (2 * (np.arange(ZONES) + 0.5) / ZONES - 1)
_V, _U = np.meshgrid(_t, _t, indexing="ij")                 # v down the image, u to the right
_s = np.stack([np.ones_like(_U), -_U, -_V], -1).reshape(-1, 3)
STRETCH = np.linalg.norm(_s, axis=1)                         # radial distance / depth along the axis
_s = _s / STRETCH[:, None]
_c, _n = np.cos(TILT), np.sin(TILT)
DIRS = np.stack([_c * _s[:, 0] + _n * _s[:, 2], _s[:, 1], -_n * _s[:, 0] + _c * _s[:, 2]], -1)   # pitched down TILT


def from_depth(depth):
    """Webots RangeFinder values (depth along the sensor's axis) -> distances along each zone's ray."""
    return np.asarray(depth, dtype=float) * STRETCH


def heights(dist, gravity):
    """64 distances (m, inf/0 = no return) and the body-frame unit gravity -> 64 floor heights (m)."""
    d = np.asarray(dist, dtype=float)
    up = -np.asarray(gravity, dtype=float)
    up = up / (np.linalg.norm(up) + 1e-9)
    ok = np.isfinite(d) & (d > MIN_RANGE) & (d < MAX_RANGE)
    h = HEIGHT + POS @ up + np.where(ok, d, 0.0) * (DIRS @ up)
    return np.clip(np.where(ok, h, 0.0), -CLIP, CLIP)   # no return: assume floor


def heights_batch(dist, gravity):
    """heights() for many robots: dist (N, 64), gravity (N, 3) -> (N, 64)."""
    d = np.asarray(dist, dtype=float)
    up = -np.asarray(gravity, dtype=float)
    up = up / (np.linalg.norm(up, axis=1, keepdims=True) + 1e-9)
    ok = np.isfinite(d) & (d > MIN_RANGE) & (d < MAX_RANGE)
    h = HEIGHT + (up @ POS)[:, None] + np.where(ok, d, 0.0) * (up @ DIRS.T)
    return np.clip(np.where(ok, h, 0.0), -CLIP, CLIP)


def encode(dist):
    """Distances for an observation: "no return" (inf) as 0, everything clipped to MAX_RANGE."""
    d = np.asarray(dist, dtype=float)
    return np.where(np.isfinite(d) & (d > MIN_RANGE), np.minimum(d, MAX_RANGE), 0.0)


def points(dist):
    """64 hit points (m, body frame) for the planner; nan where nothing came back."""
    d = np.asarray(dist, dtype=float)
    ok = np.isfinite(d) & (d > MIN_RANGE) & (d < MAX_RANGE)
    return np.where(ok[:, None], POS + d[:, None] * DIRS, np.nan)


class HeightfieldRays:
    """Casts rays at a height field (terrain.py's, values 0..1 scaled by max_height) without MuJoCo.

    MuJoCo's ray-heightfield test was ~20% of a training step: a long, shallow ray (the sensor's top
    rows) checks every cell under its whole footprint. This samples each ray every half cell instead,
    only over the stretch where it is between the highest and lowest terrain nearby, and places the
    hit by linear interpolation between the last sample above the ground and the first one below it.
    Heights between grid points use MuJoCo's own two triangles per cell, so the surface is the same.
    """

    FRAC = 0.5   # sample spacing along a ray, in cells (horizontally)
    BLOCK = 40   # cells per side of the blocks whose maximum height bounds each ray

    def __init__(self, nrow, ncol, half_x, half_y, max_height):
        self.shape, self.half, self.max_height = (nrow, ncol), (half_x, half_y), max_height
        self.cell = 2 * half_x / (ncol - 1)
        self.set_field(np.zeros((nrow, ncol)))

    def set_field(self, field):
        self.field = field
        b = self.BLOCK
        nr, nc = -(-self.shape[0] // b), -(-self.shape[1] // b)
        pad = np.full((nr * b, nc * b), -np.inf)
        pad[:self.shape[0], :self.shape[1]] = field
        self.block_max = pad.reshape(nr, b, nc, b).max(axis=(1, 3)) * self.max_height
        self.low = float(field.min()) * self.max_height

    def _heights(self, x, y):
        nrow, ncol = self.shape
        cx = np.clip((x + self.half[0]) / (2 * self.half[0]) * (ncol - 1), 0, ncol - 1.001)
        cy = np.clip((y + self.half[1]) / (2 * self.half[1]) * (nrow - 1), 0, nrow - 1.001)
        c0, r0 = cx.astype(np.intp), cy.astype(np.intp)
        fx, fy = cx - c0, cy - r0
        f = self.field
        h00, h11 = f[r0, c0], f[r0 + 1, c0 + 1]
        # MuJoCo's two triangles per cell, split along the (0,0)-(1,1) diagonal (matches mj_ray exactly)
        upper = fx >= fy
        h01, h10 = f[r0, c0 + 1], f[r0 + 1, c0]
        z = np.where(upper, h00 + (h01 - h00) * fx + (h11 - h01) * fy,
                     h00 + (h10 - h00) * fy + (h11 - h10) * fx)
        return z * self.max_height

    def _high(self, p, reach):
        """Highest terrain within `reach` of p (by block)."""
        nrow, ncol = self.shape

        def idx(v, half, n):
            return int(np.clip((v + half) / (2 * half) * (n - 1), 0, n - 1)) // self.BLOCK
        c0, c1 = idx(p[0] - reach, self.half[0], ncol), idx(p[0] + reach, self.half[0], ncol)
        r0, r1 = idx(p[1] - reach, self.half[1], nrow), idx(p[1] + reach, self.half[1], nrow)
        return float(self.block_max[r0:r1 + 1, c0:c1 + 1].max())

    def cast(self, origin, dirs, max_range):
        """Distances along unit `dirs` from `origin` to the terrain, -1 where it's not hit within max_range."""
        n = len(dirs)
        out = np.full(n, -1.0)
        high = self._high(origin, max_range)
        dz = dirs[:, 2]
        rise = origin[2] - high                       # >0: the origin is above all the terrain nearby
        down = dz < -1e-9
        safe = np.where(down, -dz, 1.0)
        t0 = np.where(down, np.maximum(0.0, rise / safe), 0.0)
        # (1 cm below the lowest ground, so a ray that ends exactly on it still counts as a hit)
        t1 = np.where(down, np.minimum(max_range, (origin[2] - self.low + 0.01) / safe), max_range)
        t1 = np.where(~down & (rise > 0), -1.0, t1)  # level or upward rays above every surface miss
        live = t1 > t0
        if not live.any():
            return out
        hs = np.hypot(dirs[:, 0], dirs[:, 1])
        step = np.minimum(self.FRAC * self.cell / np.maximum(hs, 1e-6), 0.02)
        k = np.where(live, np.ceil((t1 - t0) / step).astype(np.intp) + 1, 0)
        ray = np.repeat(np.arange(n), k)
        start = np.concatenate(([0], np.cumsum(k)[:-1]))
        j = np.arange(len(ray)) - start[ray]
        t = np.minimum(t0[ray] + j * step[ray], t1[ray])
        px = origin[0] + t * dirs[ray, 0]
        py = origin[1] + t * dirs[ray, 1]
        gap = origin[2] + t * dz[ray] - self._heights(px, py)   # ray height above the ground
        below = np.flatnonzero(gap <= 0)
        if len(below) == 0:
            return out
        r = ray[below]
        first = np.concatenate(([True], r[1:] != r[:-1]))
        i, r = below[first], r[first]
        prev = np.maximum(i - 1, start[r])
        g0, g1 = gap[prev], gap[i]
        f = np.where(prev < i, g0 / np.maximum(g0 - g1, 1e-12), 0.0)
        out[r] = t[prev] + f * (t[i] - t[prev])
        return out


class ToFModel:
    """Casts the 64 zones in a MuJoCo scene from the tof site, ignoring the robot's body.

    terrain: a HeightfieldRays for the scene's height field, and field: a function returning the
    height field (0..1) currently loaded in the model. While nothing but the terrain is within the
    sensor's range (the beams never reach the robot's own legs), rays are cast against that instead
    of through MuJoCo, which is several times faster. When anything else comes in range (the
    navigator's walls, boxes and goal post), the scan goes through MuJoCo."""

    def __init__(self, model, rng, site="tof", body="base_link", terrain=None, field=None):
        import mujoco

        self.mujoco, self.model, self.rng = mujoco, model, rng
        self.site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        self.geomid = np.zeros(len(DIRS), np.int32)
        self.dist = np.zeros(len(DIRS))
        self.last = np.full(len(DIRS), np.inf)
        self.last_dist = np.zeros(len(DIRS))   # same, with "no return" as 0 (for observations)
        self.next = 0.0
        robot = model.body_rootid[self.body]
        self.others = np.array([g for g in range(model.ngeom)
                                if model.body_rootid[model.geom_bodyid[g]] != robot and model.geom_group[g] != 2
                                and int(model.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_HFIELD)], int)
        self.terrain, self.field = (terrain, field) if terrain is not None and field is not None else (None, None)

    def _terrain_only(self, data):
        if self.terrain is None:
            return False
        if len(self.others):
            gap = np.linalg.norm(data.geom_xpos[self.others] - data.site_xpos[self.site], axis=1)
            if (gap < MAX_RANGE + self.model.geom_rbound[self.others]).any():
                return False
        field = self.field()
        if field is not self.terrain.field:
            self.terrain.set_field(field)
        return True

    def scan(self, data):
        R = data.site_xmat[self.site].reshape(3, 3)
        if self._terrain_only(data):
            self.dist = self.terrain.cast(data.site_xpos[self.site], DIRS @ R.T, MAX_RANGE)
        else:
            self.dist = np.zeros(len(DIRS))
            vec = (DIRS @ R.T).ravel()
            # geomgroup: leave out visual meshes (group 2), as the real sensor's beams pass the rock's own shell
            self.mujoco.mj_multiRay(self.model, data, data.site_xpos[self.site].copy(), vec,
                                    np.array([1, 1, 0, 1, 1, 1], np.uint8), 1, self.body, self.geomid, self.dist,
                                    len(DIRS), MAX_RANGE)
        d = np.where(self.dist < 0, np.inf, self.dist)
        d = d + self.rng.normal(0, 1, d.shape) * (RANGE_NOISE[0] + RANGE_NOISE[1] * np.minimum(d, MAX_RANGE))
        d[self.rng.random(len(d)) < DROPOUT] = np.inf
        return d

    def update(self, data, now):
        """Call every control step; returns the latest 64 distances (refreshed at 15 Hz)."""
        if now >= self.next:
            self.next = now + PERIOD
            self.last = self.scan(data)
            self.last_dist = np.where(np.isfinite(self.last), np.minimum(self.last, MAX_RANGE), 0.0)
        return self.last
