"""Score a self-righting policy: how often and how fast he gets up, and how hard he leans on the lidar.

    .venv/bin/python recover_eval.py policies/rocky_recover.npz

24 falls (fixed seeds: on his back, side or anywhere, randomised physics), run the way the robot
runs it (recover_spec.Recovery: the learned roll-over, then the scripted tidy-up). Recovered =
upright and in the standing pose, ready to walk, within 12 s.
"""

import argparse

import numpy as np

from policy import NumpyPolicy
from recover_env import RockyBobRecover
from recover_spec import Recovery
from spec import STAND, TARGET_LIMIT, projected_gravity


def evaluate(path, runs=24, seconds=12.0):
    policy = NumpyPolicy(str(path))
    env = RockyBobRecover()
    env.seconds = seconds
    ok, times, lidar_s, peak, spins, hits = [], [], [], [], [], []
    for s in range(runs):
        env.reset(seed=4000 + s)
        rec = Recovery(policy)
        rec.reset(env.data.qpos[7:])
        t_up, on, top, spin, hit = None, 0.0, 0.0, 0.0, 0.0
        while True:
            gravity = projected_gravity(env._sense("orientation", 4))
            targets = rec.step(gravity, env._sense("gyro", 3), env.data.qpos[7:].copy())
            _, _, _, trunc, info = env.step(targets / TARGET_LIMIT)
            on += 0.02 * (info["lidar_force"] > 0)
            top = max(top, info["lidar_force"])
            if t_up is None:   # only while righting himself (from when he was let go)
                spin, hit = max(spin, info["spin_rate"]), max(hit, info["impact"])
            ready = rec.done() and info["up"] > 0.95 and float(np.sum((env.data.qpos[7:] - STAND) ** 2)) < 0.3
            if ready and t_up is None:
                t_up = env.t
            if trunc:
                break
        ok.append(t_up is not None)
        if t_up is not None:
            times.append(t_up)
        lidar_s.append(on)
        peak.append(top)
        spins.append(spin)
        hits.append(hit)
    return {"success": float(np.mean(ok)), "time": float(np.median(times)) if times else float("nan"),
            "lidar_s": float(np.mean(lidar_s)), "lidar_peak": float(np.median(peak)),
            "spin": float(np.median(spins)), "spin_max": float(np.max(spins)),
            "impact": float(np.median(hits)), "impact_max": float(np.max(hits))}


def line(path, r):
    return (f"{path}: recovered {r['success']:.0%} | time to stand median {r['time']:.1f} s | "
            f"lidar on the ground {r['lidar_s']:.1f} s per fall | lidar peak force median {r['lidar_peak']:.0f} N | "
            f"body spin peak {r['spin']:.1f} rad/s (worst {r['spin_max']:.1f}) | body impact peak {r['impact']:.0f} N "
            f"(worst {r['impact_max']:.0f})")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policies", nargs="+")
    args = p.parse_args()
    for path in args.policies:
        print(line(path.split("/")[-1], evaluate(path)), flush=True)


if __name__ == "__main__":
    main()
