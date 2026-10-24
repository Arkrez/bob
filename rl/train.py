"""Train a walking policy for Rocky-Bob with PPO.

    .venv/bin/python train.py                      # 30M steps, all CPU cores
    .venv/bin/python train.py --steps 2e6 --name quick
    .venv/bin/python train.py --resume runs/walk/latest.zip

Watch progress:  .venv/bin/tensorboard --logdir runs
Each run writes runs/<name>/: checkpoints, latest.zip, vecnormalize.pkl, policy.npz
(policy.npz is the numpy-only export used by the Webots/ROS runner), and every 1M steps
progress/: a policy snapshot, a short video and a row in progress.md (see progress.py).
"""

import argparse
import os
import pathlib
import shutil
import subprocess
import sys

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import VecNormalize

from env import RockyBobWalk
from recover_env import RockyBobRecover
from vec import BatchedSubprocVecEnv

HERE = pathlib.Path(__file__).resolve().parent
NICE = ["nice", "-n", "10"] if shutil.which("nice") else []   # background scoring at low priority (not on Windows)


def export_numpy(model, vecnorm, path):
    """Save the actor MLP + observation normalisation as plain numpy arrays."""
    policy = model.policy
    layers = [m for m in policy.mlp_extractor.policy_net if isinstance(m, torch.nn.Linear)]
    arrays = {}
    for i, layer in enumerate(layers + [policy.action_net]):
        arrays[f"w{i}"] = layer.weight.detach().cpu().numpy()
        arrays[f"b{i}"] = layer.bias.detach().cpu().numpy()
    arrays["obs_mean"] = vecnorm.obs_rms.mean.astype(np.float32)
    arrays["obs_var"] = vecnorm.obs_rms.var.astype(np.float32)
    arrays["clip_obs"] = np.array(vecnorm.clip_obs, np.float32)
    arrays["activation"] = np.array("elu")
    np.savez(path, **arrays)


class TerrainLogger(BaseCallback):
    """Log the mean terrain-curriculum level of finished episodes to TensorBoard."""

    def _on_step(self):
        levels = [i["terrain_level"] for i in self.locals["infos"] if "episode" in i and "terrain_level" in i]
        if levels:
            self.logger.record_mean("curriculum/terrain_level", float(np.mean(levels)))
        return True


class PenaltyRamp(BaseCallback):
    """Ramp the shaping penalties (env.SHAPING) from `start` to full strength over `steps`,
    counted from the start of this run (so a fine-tune can explore new motions before the
    penalties bite)."""

    def __init__(self, start=0.25, steps=10e6):
        super().__init__()
        self.start, self.steps, self.last, self.t0 = start, steps, None, 0

    def _on_training_start(self):
        self.t0 = self.num_timesteps

    def _on_step(self):
        done = (self.num_timesteps - self.t0) / self.steps
        scale = round(self.start + (1 - self.start) * min(1.0, done), 2)
        if scale != self.last:
            self.training_env.env_method("set_penalty_scale", scale)
            self.last = scale
        self.logger.record("curriculum/penalty_scale", scale)
        return True


class SaveLatest(BaseCallback):
    def __init__(self, run_dir, every, scorer="progress.py"):
        super().__init__()
        self.run_dir, self.every, self.scorer = run_dir, every, scorer

    def _on_step(self):
        if self.n_calls % self.every == 0:
            self.save()
        return True

    def _on_training_end(self):
        self.save()

    def save(self):
        self.model.save(self.run_dir / "latest.zip")
        venv = self.model.get_env()
        venv.save(str(self.run_dir / "vecnormalize.pkl"))
        export_numpy(self.model, venv, self.run_dir / "policy.npz")
        self.report()

    def report(self):
        """Snapshot the policy and score/record it in the background (see progress.py)."""
        progress = self.run_dir / "progress"
        progress.mkdir(exist_ok=True)
        snap = progress / f"policy_{round(self.model.num_timesteps / 1e6):02d}M.npz"
        if snap.exists():
            return
        shutil.copy(self.run_dir / "policy.npz", snap)
        # the full checkpoint too, so a snapshot keep_best.py picks can be trained on from (train_rounds.py)
        shutil.copy(self.run_dir / "latest.zip", snap.with_suffix(".zip"))
        shutil.copy(self.run_dir / "vecnormalize.pkl", snap.with_name(snap.stem.replace("_", "_vecnormalize_", 1) + ".pkl"))
        with open(progress / "progress.log", "a") as log:
            subprocess.Popen([*NICE, sys.executable, str(HERE / self.scorer), str(snap)],
                             stdout=log, stderr=subprocess.STDOUT, cwd=HERE)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=float, default=30e6)
    p.add_argument("--envs", type=int, default=os.cpu_count(), help="worker processes")
    p.add_argument("--per-worker", type=int, default=4,
                   help="robots per worker process (batching cuts the per-step messaging overhead)")
    p.add_argument("--name", default="walk")
    p.add_argument("--task", choices=("walk", "recover"), default="walk",
                   help="walk (env.py), or recover: self-righting after a fall (recover_env.py)")
    p.add_argument("--resume", help="path to a .zip to keep training")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--explore", type=float, help="with --resume: reset the action noise std to this")
    p.add_argument("--lr", type=float, help="with --resume: learning rate (gentler fine-tunes, e.g. 1e-4)")
    p.add_argument("--target-kl", type=float, help="with --resume: stop each update past this KL (e.g. 0.01)")
    p.add_argument("--obs-stats", help="with --resume: policy.npz whose input scaling matches the checkpoint "
                   "(checkpoints made before vecnormalize was saved with them)")
    p.add_argument("--penalty-start", type=float, default=0.25,
                   help="shaping penalties start at this fraction and ramp to full (1 = no ramp)")
    args = p.parse_args()

    run_dir = HERE / "runs" / args.name
    run_dir.mkdir(parents=True, exist_ok=True)
    # env 0 streams its state so `watch.py --live` can show a robot that is actually training
    def make(rank):
        if args.task == "recover":
            return lambda: Monitor(RockyBobRecover())
        return lambda: Monitor(RockyBobWalk(stream=(rank == 0)))
    robots = args.envs * args.per_worker
    venv = BatchedSubprocVecEnv([make(i) for i in range(robots)], workers=args.envs)
    venv.seed(args.seed)
    n_steps = max(64, 24576 // robots)   # keep ~24.6k samples per update whatever the robot count

    if args.resume:
        stats = pathlib.Path(args.resume).with_name("vecnormalize.pkl")
        ckpt = pathlib.Path(args.resume)   # CheckpointCallback saves ppo_<n>_steps.zip + ppo_vecnormalize_<n>_steps.pkl
        own = ckpt.with_name(ckpt.stem.replace("_", "_vecnormalize_", 1) + ".pkl")
        venv = VecNormalize.load(str(own if own.exists() else stats), venv)
        if args.obs_stats:   # the network only works with the input scaling it was trained with
            z = np.load(args.obs_stats)
            venv.obs_rms.mean, venv.obs_rms.var = z["obs_mean"].astype(np.float64), z["obs_var"].astype(np.float64)
        model = PPO.load(args.resume, env=venv, device="cpu", custom_objects={"n_steps": n_steps})
        if args.lr:   # a good policy needs smaller steps: big updates can knock it off its gait for good
            model.learning_rate = args.lr
            model._setup_lr_schedule()
        if args.target_kl:
            model.target_kl = args.target_kl
        if args.explore:   # a converged policy has almost no noise left to discover new motions with
            model.policy.log_std.data.fill_(float(np.log(args.explore)))
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.99)
        model = PPO(
            "MlpPolicy", venv, device="cpu", seed=args.seed, verbose=1,
            n_steps=n_steps, batch_size=8192, n_epochs=5, learning_rate=3e-4,
            gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.003,
            max_grad_norm=1.0, target_kl=0.03,
            policy_kwargs=dict(net_arch=dict(pi=[256, 256, 128], vf=[256, 256, 128]),
                               activation_fn=torch.nn.ELU, log_std_init=-1.0),
            tensorboard_log=str(HERE / "runs"),
        )

    every = max(1, int(1e6 // robots))
    model.learn(
        total_timesteps=int(args.steps),
        tb_log_name=args.name,
        reset_num_timesteps=not args.resume,
        callback=[CheckpointCallback(every * 5, str(run_dir / "checkpoints"), name_prefix="ppo", save_vecnormalize=True),
                  SaveLatest(run_dir, every, scorer="recover_eval.py" if args.task == "recover" else "progress.py"),
                  TerrainLogger(),
                  PenaltyRamp(start=args.penalty_start, steps=8e6 if args.resume else 10e6)],
    )
    print(f"done: {run_dir}")


if __name__ == "__main__":
    main()
