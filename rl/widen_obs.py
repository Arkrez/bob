"""Give a trained walker new observation inputs (the floor sensor) without changing what it does.

    .venv/bin/python widen_obs.py runs/rounds/champion/walker.zip --out runs/walk_tof/start.zip
    .venv/bin/python widen_obs.py runs/rounds/champion/nav.zip --to 114 --out runs/nav_chase_start/latest.zip   (a navigator)

The first layer of the policy and value networks gets extra input columns, set to zero, so the
network ignores the new inputs until training teaches it to use them. The observation normaliser
gets matching entries (mean 0, variance 1). Writes <out>.zip, vecnormalize.pkl next to it and
policy.npz.
"""

import argparse
import pathlib
import pickle

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO

from spec import OBS_SIZE, OBS_SIZE_TOF
from train import export_numpy


def widen(layer, extra):
    new = torch.nn.Linear(layer.in_features + extra, layer.out_features)
    with torch.no_grad():
        new.weight.zero_()
        new.weight[:, :layer.in_features] = layer.weight
        new.bias.copy_(layer.bias)
    return new


def main():
    p = argparse.ArgumentParser()
    p.add_argument("checkpoint")
    p.add_argument("--vecnorm", help="its vecnormalize .pkl (default: next to it)")
    p.add_argument("--out", required=True)
    p.add_argument("--to", type=int, help="inputs to widen to (default: a walker, OBS_SIZE -> OBS_SIZE_TOF)")
    args = p.parse_args()
    ckpt = pathlib.Path(args.checkpoint)
    stats = pathlib.Path(args.vecnorm) if args.vecnorm else ckpt.with_name("vecnormalize.pkl")
    if not stats.exists():
        stats = ckpt.with_name(ckpt.stem + "_vecnormalize.pkl")
    with open(stats, "rb") as f:
        vecnorm = pickle.load(f)
    model = PPO.load(str(ckpt), device="cpu")
    old = model.observation_space.shape[0]
    size = args.to or OBS_SIZE_TOF
    if not args.to:
        assert old == OBS_SIZE, f"expected a {OBS_SIZE}-input walker, got {old}"
    extra = size - old
    pol = model.policy
    for net in (pol.mlp_extractor.policy_net, pol.mlp_extractor.value_net):
        net[0] = widen(net[0], extra)
    space = gym.spaces.Box(-np.inf, np.inf, (size,), np.float32)
    model.observation_space = pol.observation_space = space
    pol.optimizer = pol.optimizer_class(pol.parameters(), lr=pol.optimizer.param_groups[0]["lr"], **pol.optimizer_kwargs)
    rms = vecnorm.obs_rms
    rms.mean = np.concatenate([rms.mean, np.zeros(extra)])
    rms.var = np.concatenate([rms.var, np.ones(extra)])
    vecnorm.observation_space = space
    vecnorm.__dict__.setdefault("class_attributes", {})
    vecnorm.__dict__.setdefault("returns", None)
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(out))
    with open(out.with_name("vecnormalize.pkl"), "wb") as f:
        pickle.dump(vecnorm, f)
    export_numpy(model, vecnorm, out.with_name("policy.npz"))
    print(f"widened {old} -> {size} inputs: {out}")


if __name__ == "__main__":
    main()
