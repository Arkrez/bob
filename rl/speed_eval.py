"""Top-speed test: command increasing forward speeds and measure what the walker actually reaches.

    .venv/bin/python speed_eval.py policies/rocky_walk.npz

Flat ground, randomised physics, no shoves; 6 seeds x 8 s per speed (first 2 s ignored).
"""

import argparse

import numpy as np

from env import RockyBobWalk
from policy import NumpyPolicy

SPEEDS = (0.15, 0.30, 0.45, 0.60, 0.80)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policies", nargs="+")
    p.add_argument("--seeds", type=int, default=6)
    args = p.parse_args()
    for path in args.policies:
        policy = NumpyPolicy(path)
        cells, best = [], 0.0
        for v in SPEEDS:
            speeds, torque, falls = [], [], 0
            for s in range(args.seeds):
                env = RockyBobWalk(randomize=True, episode_seconds=8, command=(v, 0, 0), terrain_level=0.0, pushes=False)
                obs, _ = env.reset(seed=1200 + s)
                vel = []
                for i in range(400):
                    obs, _, fell, _, info = env.step(policy(obs))
                    torque.append(info["torque"])
                    if i >= 100:
                        vel.append(info["vel"][0])
                    if fell:
                        falls += 1
                        break
                speeds.append(np.mean(vel) if vel else 0.0)
            achieved = float(np.mean(speeds))
            best = max(best, achieved)
            cells.append(f"cmd {v:.2f} -> {achieved:.3f} m/s (torque95 {np.percentile(torque, 95):.2f}, falls {falls})")
        print(f"{path.split('/')[-1]}: top speed {best:.3f} m/s | " + " | ".join(cells), flush=True)


if __name__ == "__main__":
    main()
