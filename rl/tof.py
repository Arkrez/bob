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


def encode(dist):
    """Distances for an observation: "no return" (inf) as 0, everything clipped to MAX_RANGE."""
    d = np.asarray(dist, dtype=float)
    return np.where(np.isfinite(d) & (d > MIN_RANGE), np.minimum(d, MAX_RANGE), 0.0)


def points(dist):
    """64 hit points (m, body frame) for the planner; nan where nothing came back."""
    d = np.asarray(dist, dtype=float)
    ok = np.isfinite(d) & (d > MIN_RANGE) & (d < MAX_RANGE)
    return np.where(ok[:, None], POS + d[:, None] * DIRS, np.nan)


class ToFModel:
    """Casts the 64 zones in a MuJoCo scene from the tof site, ignoring the robot's body."""

    def __init__(self, model, rng, site="tof", body="base_link"):
        import mujoco

        self.mujoco, self.model, self.rng = mujoco, model, rng
        self.site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        self.geomid = np.zeros(len(DIRS), np.int32)
        self.dist = np.zeros(len(DIRS))
        self.last = np.full(len(DIRS), np.inf)
        self.last_dist = np.zeros(len(DIRS))   # same, with "no return" as 0 (for observations)
        self.next = 0.0

    def scan(self, data):
        R = data.site_xmat[self.site].reshape(3, 3)
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
