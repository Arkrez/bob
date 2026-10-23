"""Step quality: how high each foot lifts, how even the four legs are, and how often feet bounce.

    .venv/bin/python step_eval.py policies/rocky_walk.npz

Per command (walk, run, pivot), 4 seeds x 8 s, randomised physics, flat ground. A "step" is a
touchdown after at least env.BOUNCE_AIR seconds in the air; a quicker re-touch is a bounce.
"""

import argparse

import numpy as np

from env import BOUNCE_AIR, SWING_HEIGHT, RockyBobWalk
from policy import NumpyPolicy

TESTS = [("walk", (0.15, 0, 0)), ("run", (0.45, 0, 0)), ("pivot", (0, 0, 0.4))]


def measure(policy, cmd, seeds=4, secs=8.0):
    per_leg, bounces, seconds = [[] for _ in range(4)], 0, 0.0
    for s in range(seeds):
        env = RockyBobWalk(randomize=True, episode_seconds=secs, command=cmd, terrain_level=0.0, pushes=False)
        obs, _ = env.reset(seed=1300 + s)
        before = env._foot_contacts()[0]
        for i in range(int(secs / 0.02)):
            air = env.air_time.copy()
            peak = env.swing_peak.copy()
            obs, _, fell, _, _ = env.step(policy(obs))
            touching = env._foot_contacts()[0]
            if i >= 75:   # skip the first 1.5 s
                down = touching & ~before
                for k in np.flatnonzero(down):
                    if air[k] + 0.02 >= BOUNCE_AIR:
                        per_leg[k].append(max(peak[k], env.last_peak[k]))
                    else:
                        bounces += 1
                seconds += 0.02
            before = touching
            if fell:
                break
    means = np.array([np.mean(p) if p else np.nan for p in per_leg]) * 100
    return means, bounces / max(seconds, 1e-6), sum(len(p) for p in per_leg) / max(seconds, 1e-6)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policies", nargs="+")
    args = p.parse_args()
    for path in args.policies:
        policy = NumpyPolicy(path)
        cells = []
        for name, cmd in TESTS:
            means, bounce_rate, step_rate = measure(policy, cmd)
            cells.append(f"{name}: step height {np.nanmean(means):.1f} cm (legs {'/'.join(f'{m:.1f}' for m in means)}, "
                         f"spread {np.nanstd(means):.1f}) | {step_rate:.1f} steps/s, {bounce_rate:.1f} bounces/s")
        print(f"{path.split('/')[-1]} (target {SWING_HEIGHT * 100:.1f} cm):\n  " + "\n  ".join(cells), flush=True)


if __name__ == "__main__":
    main()
