"""Pi camera target sensing, as the navigator sees it (shared by nav_env.py and the Webots bridge).

On the real robot, a Pi Camera Module 3 on the front of the rock detects a printed ArUco marker
on a post (rl/camera_target_node.py). OpenCV returns the marker's position relative to the camera.
Here that is modelled geometrically, so the navigator trains on what the camera will really give it:
  - the target is only seen inside the horizontal field of view and between MIN_RANGE and MAX_RANGE
  - about 10 detections a second, ~0.1 s late, and sometimes missed
  - noisy distance (worse further away) and bearing
Between sightings the navigator only has the last estimate and how old it is, so it has to learn to
turn and look when the target isn't in view.

numpy only. Frames: body frame x forward, y left, metres.
"""

import numpy as np

# Pi Camera Module 3 (standard lens): 66 x 41 deg. Wide: 102 deg; Module v2: 62.2 deg.
HFOV_DEG = 66.0
CAMERA_X = 0.109          # m ahead of the body origin (front face of the rock, 70 mm up)
# How far a marker can be detected depends on how many pixels it covers: ArUco needs roughly
# MIN_MARKER_PX across. Range = focal length (px) * marker size / MIN_MARKER_PX.
IMAGE_WIDTH = 1280        # px, as captured by camera_target_node.py
MARKER_SIZE = 0.15        # m, black square of the printed marker
MIN_MARKER_PX = 25
FOCAL_PX = (IMAGE_WIDTH / 2) / np.tan(np.radians(HFOV_DEG / 2))
MIN_RANGE, MAX_RANGE = 0.15, FOCAL_PX * MARKER_SIZE / MIN_MARKER_PX   # ~5.9 m with the defaults
FRAME_PERIOD = 0.1        # s between detections
LATENCY = 0.1             # s from exposure to result
MISS_RATE = 0.1           # chance a frame fails to detect a visible marker
RANGE_NOISE = (0.02, 0.03)   # sigma = a + b * distance (m)
BEARING_NOISE_DEG = 1.5
VISIBLE_FOR = 0.3         # s: "visible" while the last detection is younger than this
MAX_AGE = 3.0             # s: ages are clipped here


class CameraModel:
    def __init__(self, rng):
        self.rng = rng
        self.reset()

    def reset(self):
        self.estimate = np.zeros(2)     # last detected target position, body frame at detection
        self.seen_at = -1e9
        self.next_frame = 0.0
        self.pending = []               # (ready_time, measurement)

    def in_view(self, rel):
        """rel: true target position relative to the body origin (body frame)."""
        cam = np.asarray(rel) - [CAMERA_X, 0.0]
        dist = float(np.hypot(*cam))
        bearing = np.degrees(np.arctan2(cam[1], cam[0]))
        return MIN_RANGE <= dist <= MAX_RANGE and abs(bearing) <= HFOV_DEG / 2

    def update(self, rel, now):
        """Call every control step with the true relative target position and the time."""
        if now >= self.next_frame:
            self.next_frame = now + FRAME_PERIOD
            if self.in_view(rel) and self.rng.random() > MISS_RATE:
                cam = np.asarray(rel) - [CAMERA_X, 0.0]
                dist = float(np.hypot(*cam))
                bearing = np.arctan2(cam[1], cam[0])
                dist += self.rng.normal(0, RANGE_NOISE[0] + RANGE_NOISE[1] * dist)
                bearing += np.radians(self.rng.normal(0, BEARING_NOISE_DEG))
                meas = np.array([CAMERA_X + dist * np.cos(bearing), dist * np.sin(bearing)])
                self.pending.append((now + LATENCY, meas))
        while self.pending and self.pending[0][0] <= now:
            _, self.estimate = self.pending.pop(0)
            self.seen_at = now

    def reading(self, now):
        """(estimate x, y, visible 0/1, age 0..1) for the navigator."""
        age = min(now - self.seen_at, MAX_AGE)
        visible = float(age <= VISIBLE_FOR)
        return self.estimate.copy(), visible, age / MAX_AGE
