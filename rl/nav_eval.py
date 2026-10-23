"""Score a goal-reaching navigator (or the scripted baseline) on random goals, with the camera model.

    .venv/bin/python nav_eval.py policies/rocky_nav.npz
    .venv/bin/python nav_eval.py baseline
    .venv/bin/python nav_eval.py policies/rocky_nav.npz --video nav.mp4

Each run is 60 s on medium terrain with shoves and randomised physics; goals keep appearing
0.5-2.5 m away in random directions. Reports goals per minute, time to each goal, path
efficiency (straight-line distance / distance walked), sideways command use and falls.
"""

import argparse

import numpy as np

from camera import CAMERA_X
from nav_env import RockyBobNavigate
from nav_spec import NAV_DT, ScriptedNavigator, command_to_action
from policy import NumpyPolicy

TERRAIN = (0.3, 0.3)


def baseline():
    """The scripted navigator, with its explore-when-lost search (nav_spec.ScriptedNavigator)."""
    return ScriptedNavigator()


def run(nav, seed, seconds=60, frames=None):
    env = RockyBobNavigate(episode_seconds=seconds, terrain=TERRAIN)
    obs, _ = env.reset(seed=seed)
    scripted = baseline() if nav == "baseline" else None
    times, effs, sideways, arcing, fell = [], [], [], [], False
    t_goal, path, start = 0.0, 0.0, env.walk.data.qpos[:2].copy()
    straight = float(np.linalg.norm(env.goal - start))
    prev = start.copy()
    done = False
    while not done:
        action = command_to_action(scripted(obs)) if scripted is not None else nav(obs)
        reached_before = env.reached
        obs, _, fell, trunc, info = env.step(action)
        pos = env.walk.data.qpos[:2].copy()
        path += float(np.linalg.norm(pos - prev))
        prev = pos
        t_goal += NAV_DT
        sideways.append(abs(info["cmd"][1]))
        arcing.append(abs(info["cmd"][2]) > 0.15 and info["cmd"][0] > 0.05)
        if frames is not None:
            frames.append(env.render())
        if env.reached > reached_before:
            times.append(t_goal)
            effs.append(straight / max(path, 1e-6))
            t_goal, path = 0.0, 0.0
            straight = float(np.linalg.norm(env.goal - pos))
        done = fell or trunc
    return env.reached, times, effs, float(np.mean(sideways)), fell, float(np.mean(arcing))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policy", help="navigator .npz, or 'baseline'")
    p.add_argument("--runs", type=int, default=6)
    p.add_argument("--video", help="record the first run to this mp4")
    args = p.parse_args()
    nav = "baseline" if args.policy == "baseline" else NumpyPolicy(args.policy)

    goals, times, effs, side, falls, arcs = [], [], [], [], 0, []
    frames = [] if args.video else None
    for i in range(args.runs):
        g, t, e, s, f, a = run(nav, seed=500 + i, frames=frames if i == 0 else None)
        goals.append(g); times += t; effs += e; side.append(s); falls += int(f); arcs.append(a)
    print(f"{args.policy}: {np.mean(goals):.1f} goals/min (range {min(goals)}-{max(goals)}) | "
          f"time to goal median {np.median(times) if times else float('nan'):.1f} s | "
          f"path efficiency {np.mean(effs) if effs else float('nan'):.0%} | "
          f"sideways cmd {np.mean(side):.3f} m/s | arcing {np.mean(arcs):.0%} | falls {falls}/{args.runs}")
    if args.video:
        import imageio
        imageio.mimsave(args.video, frames, fps=int(1 / NAV_DT), macro_block_size=8)
        print(f"wrote {args.video}")


if __name__ == "__main__":
    main()
