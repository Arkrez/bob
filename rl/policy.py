"""Numpy-only runner for an exported policy (runs/<name>/policy.npz).

No torch needed, so it runs in the ROS container and on the Raspberry Pi.
"""

import numpy as np


class NumpyPolicy:
    def __init__(self, path):
        z = np.load(path)
        n = sum(1 for k in z.files if k.startswith("w"))
        self.layers = [(z[f"w{i}"], z[f"b{i}"]) for i in range(n)]
        self.mean, self.std = z["obs_mean"], np.sqrt(z["obs_var"] + 1e-8)
        self.clip = float(z["clip_obs"])

    def __call__(self, obs):
        x = np.clip((np.asarray(obs, np.float32) - self.mean) / self.std, -self.clip, self.clip)
        for i, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if i < len(self.layers) - 1:
                x = np.where(x > 0, x, np.expm1(x))   # ELU
        return np.clip(x, -1.0, 1.0)
