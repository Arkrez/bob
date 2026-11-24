"""Check batch_env.BatchedWalk against RockyBobWalk, robot by robot.

Before every batched step, each robot's exact state (physics, episode bookkeeping, random generator)
is copied into a plain RockyBobWalk, which takes the same action with its own step(). Every reward
term must match. Observations match too, apart from the sensor noise, which BatchedWalk draws
from one shared generator.

    .venv/bin/python test_batch_env.py            # 6 robots, 1500 steps (30 s each)
"""

import argparse
import copy

import numpy as np

import batch_env
from batch_env import _SCALARS, _VECTORS, BatchedWalk
from policy import NumpyPolicy
from spec import OBS_SIZE


def reference(b, i):
    """A RockyBobWalk in exactly robot i's state."""
    e = b.envs[i]
    r = copy.copy(e)
    r.data = copy.deepcopy(e.data)
    r.np_random = copy.deepcopy(e.np_random)
    r._stream = None
    for k in _VECTORS:
        setattr(r, k, getattr(b, k)[i].copy())
    for k in _SCALARS:
        v = getattr(b, k)[i]
        setattr(r, k, int(v) if k == "steps" else float(v))
    r.penalty_scale = float(b.penalty[i])
    r._fell, r._walked, r._asked, r._climbed = bool(b.fell[i]), b.walked[i], b.asked[i], b.climbed[i]
    if e.tof is not None:
        r.tof = copy.copy(e.tof)
        r.tof.rng = r.np_random
        r.tof.next, r.tof.last = float(b.tof_next[i]), b.tof_last[i].copy()
        r.tof.field = lambda: r.field
    return r


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robots", type=int, default=6)
    p.add_argument("--steps", type=int, default=1500)
    p.add_argument("--noise", type=float, default=0.15, help="action noise on top of the walker (more = more falls)")
    args = p.parse_args()
    pol = NumpyPolicy("policies/rocky_walk.npz")
    b = BatchedWalk(args.robots, threads=4)
    b.seed(7)
    obs = b.reset()
    b.env_method("set_penalty_scale", 0.6)
    rng = np.random.default_rng(0)
    worst = {}
    reward_err = obs_err = 0.0
    resets = compared = falls = 0
    for step in range(args.steps):
        act = np.array([pol(o) for o in obs]) + rng.normal(0, args.noise, (args.robots, 12))
        act = np.clip(act, -1, 1)
        # the references step first: they share each robot's MjModel, which a reset re-randomises
        refs = [reference(b, i) for i in range(args.robots)]
        results = [r.step(act[i]) for i, r in enumerate(refs)]
        obs, rew, done, infos = b.step(act)
        for i, (o_r, rew_r, term_r, trunc_r, info_r) in enumerate(results):
            reward_err = max(reward_err, abs(rew_r - rew[i]) / max(1.0, abs(rew_r)))
            for k, v in info_r["terms"].items():
                worst[k] = max(worst.get(k, 0.0), abs(v - b.terms[k][i]))
            assert bool(term_r or trunc_r) == bool(done[i]), (step, i, term_r, trunc_r, done[i])
            if done[i]:
                resets += 1
                falls += bool(term_r)
                o_r_cmp, o_b_cmp = o_r, infos[i]["terminal_observation"]
            else:
                o_r_cmp, o_b_cmp = o_r, obs[i]
            # command, last action and gait clock are exact; gravity, gyro, joints, floor carry noise
            exact = slice(6, 9), slice(21, 33), slice(33, 35)
            for s in exact:
                obs_err = max(obs_err, float(np.abs(o_r_cmp[s] - o_b_cmp[s]).max()))
            compared += 1
    print(f"{compared} robot-steps compared, {resets} episode ends ({falls} falls)")
    print(f"largest reward difference: {reward_err:.2e} (relative)")
    print(f"largest exact-observation difference: {obs_err:.2e}")
    bad = {k: v for k, v in worst.items() if v > 1e-6}
    print("reward terms off by more than 1e-6:", bad or "none")
    b.close()
    assert reward_err < 1e-6 and obs_err < 1e-5 and not bad


if __name__ == "__main__":
    main()
