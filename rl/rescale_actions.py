"""Change a joint's action scale (spec.ACTION_SCALE) without changing what a trained walker does.

    .venv/bin/python rescale_actions.py runs/walk_hp4/checkpoints/ppo_43999296_steps.zip \\
        --joint 0 --old 0.35 --new 0.6 --out runs/turn_start/start.zip

Joint targets are STAND + ACTION_SCALE * action, so widening a joint's range would make an old
policy swing it further. This scales the policy's output rows (and exploration noise) for that
joint by old/new, and the observation normaliser's statistics for the matching last-action inputs
the same way, so the network sees exactly the inputs it was trained on and sets the same targets.
Fine-tune from the result (train.py --resume) to use the wider range.

Writes <out>.zip, its ppo_vecnormalize pkl (as train.py --resume expects: vecnormalize.pkl next to
it) and policy.npz. An exported policy.npz can be converted on its own too (in place with --out =
the same file):
    .venv/bin/python rescale_actions.py policies/rocky_walk.npz --old 0.35 --new 0.6 --out policies/rocky_walk.npz
"""

import argparse
import pathlib
import pickle

import numpy as np
import torch
from stable_baselines3 import PPO

from spec import OBS_SIZE, OBS_SIZE_TOF
from train import export_numpy

LAST_ACTION = 3 + 3 + 3 + 12   # observation index of last_action[0] (see spec.observation)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("checkpoint")
    p.add_argument("--joint", type=int, default=0, help="0 yaw, 1 hip, 2 knee (all four legs)")
    p.add_argument("--old", type=float, required=True)
    p.add_argument("--new", type=float, required=True)
    p.add_argument("--out", required=True)
    args = p.parse_args()

    k = args.old / args.new
    rows = [args.joint + 3 * leg for leg in range(4)]
    idx = [LAST_ACTION + r for r in rows]
    ckpt = pathlib.Path(args.checkpoint)
    if ckpt.suffix == ".npz":
        z = dict(np.load(ckpt))
        last = sum(1 for key in z if key.startswith("w")) - 1
        z[f"w{last}"][rows] *= k
        z[f"b{last}"][rows] *= k
        z["obs_mean"][idx] *= k
        z["obs_var"][idx] *= k * k
        np.savez(args.out, **z)
        print(f"scaled joint {args.joint} outputs by {k:.3f}: wrote {args.out}")
        return
    stats = ckpt.with_name(ckpt.stem.replace("_", "_vecnormalize_", 1) + ".pkl")
    if not stats.exists():
        stats = ckpt.with_name("vecnormalize.pkl")
    with open(stats, "rb") as f:
        vecnorm = pickle.load(f)
    model = PPO.load(str(ckpt), device="cpu")

    with torch.no_grad():
        model.policy.action_net.weight[rows] *= k
        model.policy.action_net.bias[rows] *= k
        model.policy.log_std[rows] += float(np.log(k))
    assert vecnorm.obs_rms.mean.shape[0] in (OBS_SIZE, OBS_SIZE_TOF)
    vecnorm.obs_rms.mean[idx] *= k
    vecnorm.obs_rms.var[idx] *= k * k

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(out))
    # VecNormalize pickles itself without these; loaded on its own (no env) they are missing
    vecnorm.__dict__.setdefault("class_attributes", {})
    vecnorm.__dict__.setdefault("returns", None)
    with open(out.with_name("vecnormalize.pkl"), "wb") as f:
        pickle.dump(vecnorm, f)
    export_numpy(model, vecnorm, out.with_name("policy.npz"))
    print(f"scaled joint {args.joint} outputs by {k:.3f}: wrote {out}, vecnormalize.pkl, policy.npz")


if __name__ == "__main__":
    main()
