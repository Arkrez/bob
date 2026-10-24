"""RPLIDAR C1 on Rocky-Bob's lid, as a navigator would see it (MuJoCo, Webots and the real robot).

The real C1 spins at 10 Hz and returns ~500 points a turn, 0.05-12 m, about +-3 cm. On the robot
sllidar_ros2 publishes them on /scan; in Webots the bridge does the same from a Lidar device. For
learning, every scan is reduced to N_BINS sectors (min-pooled: the nearest thing in each sector),
counter-clockwise from straight behind, clipped to MAX_RANGE and scaled to 0..1. scan_bins() does
that for a ROS LaserScan; LidarModel casts the same rays in MuJoCo with mj_multiRay.

numpy only (mujoco for LidarModel). Frames: body frame x forward, y left, metres.
"""

import numpy as np

LIDAR_Z = 0.166           # scan plane above the base (cad/rocky_bob/body.py LIDAR_SCAN_Z)
SCAN_PERIOD = 0.1         # s per revolution (10 Hz)
MIN_RANGE, MAX_RANGE = 0.05, 12.0
NAV_RANGE = 4.0           # bins are clipped here: indoor navigation doesn't need further
N_BINS = 72               # 5 degree sectors
RAYS_PER_BIN = 3          # MuJoCo rays cast per sector (min-pooled)
RANGE_NOISE = 0.015       # m, 1 sigma
DROPOUT = 0.02            # chance a sector returns nothing (dark or shiny surfaces)

BIN_ANGLES = -np.pi + (np.arange(N_BINS) + 0.5) * 2 * np.pi / N_BINS   # sector centres, CCW


def scan_bins(ranges, angle_min, angle_increment):
    """ROS LaserScan arrays -> N_BINS min-pooled ranges (m, MAX hits clipped to NAV_RANGE)."""
    r = np.asarray(ranges, dtype=float)
    r = np.where(np.isfinite(r) & (r >= MIN_RANGE), r, np.inf)
    a = np.mod(angle_min + angle_increment * np.arange(len(r)) + np.pi, 2 * np.pi)
    idx = np.minimum((a / (2 * np.pi) * N_BINS).astype(int), N_BINS - 1)
    out = np.full(N_BINS, np.inf)
    np.minimum.at(out, idx, r)
    return np.minimum(out, NAV_RANGE)


def normalise(bins):
    return np.asarray(bins) / NAV_RANGE


class LidarModel:
    """Casts the C1's rays in a MuJoCo scene, ignoring the robot's own body."""

    def __init__(self, model, rng, site="lidar", body="base_link"):
        import mujoco

        self.mujoco, self.model, self.rng = mujoco, model, rng
        self.site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site)
        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        n = N_BINS * RAYS_PER_BIN
        self.angles = -np.pi + (np.arange(n) + 0.5) * 2 * np.pi / n
        self.local = np.stack([np.cos(self.angles), np.sin(self.angles), np.zeros(n)], 1)
        self.geomid = np.zeros(n, np.int32)
        self.dist = np.zeros(n)
        self.bins = np.full(N_BINS, NAV_RANGE)
        self.next_scan = 0.0

    def scan(self, data):
        """One revolution now: N_BINS ranges in metres (clipped to NAV_RANGE)."""
        R = data.site_xmat[self.site].reshape(3, 3)
        # the scan plane turns and tilts with the body, like the real head
        vec = (self.local @ R.T).ravel()
        self.mujoco.mj_multiRay(self.model, data, data.site_xpos[self.site].copy(), vec, None, 1,
                                self.body, self.geomid, self.dist, len(self.angles), MAX_RANGE)
        d = np.where(self.dist < 0, np.inf, self.dist)   # -1: no hit
        d = d + self.rng.normal(0, RANGE_NOISE, d.shape)
        d = np.where(d < MIN_RANGE, np.inf, d)
        bins = d.reshape(N_BINS, RAYS_PER_BIN).min(1)
        bins[self.rng.random(N_BINS) < DROPOUT] = np.inf
        return np.minimum(bins, NAV_RANGE)

    def update(self, data, now):
        """Call every control step; refreshes self.bins at the C1's 10 Hz."""
        if now >= self.next_scan:
            self.next_scan = now + SCAN_PERIOD
            self.bins = self.scan(data)
        return self.bins
