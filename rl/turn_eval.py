"""How a walker turns in place: footfalls per radian turned, body drift off the pivot, turn rate.

    .venv/bin/python turn_eval.py policies/rocky_walk.npz

Each run turns in place for 10 s (both directions, randomised physics, flat and medium terrain,
no shoves). Drift is how far the body centre ends up from where the turn started; "max drift" is
the furthest it wandered on the way.
"""

import argparse

import numpy as np

from env import RockyBobWalk
from policy import NumpyPolicy
from spec import CONTROL_DT


def turn(policy, wz, level, seed, seconds=10.0):
    env = RockyBobWalk(randomize=True, episode_seconds=seconds, command=(0.0, 0.0, wz),
                       terrain_level=level, pushes=False)
    obs, _ = env.reset(seed=seed)
    start = env.data.qpos[:2].copy()
    footfalls, yaw, max_drift, fell = 0, 0.0, 0.0, False
    touching_before = env._foot_contacts()[0]
    for _ in range(int(seconds / CONTROL_DT)):
        obs, _, fell, trunc, info = env.step(policy(obs))
        touching = env._foot_contacts()[0]
        footfalls += int(np.sum(touching & ~touching_before))
        touching_before = touching
        yaw += info["yaw_rate"] * CONTROL_DT
        max_drift = max(max_drift, float(np.linalg.norm(env.data.qpos[:2] - start)))
        if fell or trunc:
            break
    drift = float(np.linalg.norm(env.data.qpos[:2] - start))
    return abs(yaw), footfalls, drift, max_drift, fell


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policies", nargs="+")
    p.add_argument("--wz", type=float, nargs="+", default=[0.4], help="commanded turn rates (rad/s)")
    args = p.parse_args()
    for path in args.policies:
        policy = NumpyPolicy(path)
        for w in args.wz:
            rows = [turn(policy, wz, level, seed) for wz in (w, -w) for level in (0.0, 0.3) for seed in range(3)]
            yaw, steps, drift, max_drift, fell = (np.array(c, dtype=float) for c in zip(*rows))
            print(f"{path.split('/')[-1]}: turn rate {np.mean(yaw) / 10:.2f} rad/s (cmd {w:.2f}) | "
                  f"{np.sum(steps) / np.sum(yaw):.1f} footfalls per radian | drift {100 * np.mean(drift):.1f} cm "
                  f"(max {100 * np.mean(max_drift):.1f} cm) | falls {int(np.sum(fell))}/{len(rows)}", flush=True)


if __name__ == "__main__":
    main()
