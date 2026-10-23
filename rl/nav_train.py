"""Train the goal-reaching navigator (nav_env.py) with PPO, on top of the frozen walker.

    .venv/bin/python nav_train.py                 # 3M navigator steps (~10 days of robot time)
    .venv/bin/python nav_train.py --steps 1e6 --name nav_quick

Writes runs/<name>/: latest.zip, vecnormalize.pkl, policy.npz (numpy navigator) and, every
250k steps, a row in progress.md from nav_eval.py (goals per minute, time to goal, path
efficiency, sideways use, falls). The scripted baseline is the first row for comparison.
"""

import argparse
import os
import pathlib
import subprocess
import sys

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

from nav_env import RockyBobNavigate
from train import export_numpy

HERE = pathlib.Path(__file__).resolve().parent


class SaveAndScore(BaseCallback):
    def __init__(self, run_dir, every):
        super().__init__()
        self.run_dir, self.every, self.next = run_dir, every, every

    def _on_training_start(self):
        self.next = self.num_timesteps + self.every   # counts from where a resumed run starts

    def _on_step(self):
        if self.num_timesteps >= self.next:
            self.next += self.every
            self.save()
        return True

    def _on_training_end(self):
        self.save()

    def save(self):
        self.model.save(self.run_dir / "latest.zip")
        venv = self.model.get_env()
        venv.save(str(self.run_dir / "vecnormalize.pkl"))
        snap = self.run_dir / f"nav_{self.num_timesteps / 1e6:.2f}M.npz"
        export_numpy(self.model, venv, snap)
        export_numpy(self.model, venv, self.run_dir / "policy.npz")
        with open(self.run_dir / "progress.md", "a") as log:
            subprocess.Popen(["nice", "-n", "10", sys.executable, str(HERE / "nav_eval.py"), str(snap)],
                             stdout=log, stderr=subprocess.STDOUT, cwd=HERE)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=float, default=3e6)
    p.add_argument("--envs", type=int, default=os.cpu_count())
    p.add_argument("--name", default="nav")
    p.add_argument("--resume", help="path to a latest.zip to keep training (vecnormalize.pkl beside it)")
    args = p.parse_args()

    run_dir = HERE / "runs" / args.name
    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / "progress.md", "w") as log:
        log.write("# Navigator progress (nav_eval.py: 6 x 60 s, medium terrain, shoves, camera model)\n\n")
        log.flush()
        subprocess.run([sys.executable, str(HERE / "nav_eval.py"), "baseline"], stdout=log, cwd=HERE, check=True)

    venv = SubprocVecEnv([lambda: Monitor(RockyBobNavigate()) for _ in range(args.envs)])
    venv.seed(0)
    if args.resume:
        venv = VecNormalize.load(str(pathlib.Path(args.resume).with_name("vecnormalize.pkl")), venv)
        model = PPO.load(args.resume, env=venv, device="cpu")
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.98)
        model = PPO(
            "MlpPolicy", venv, device="cpu", seed=0, verbose=1,
            n_steps=512, batch_size=2048, n_epochs=8, learning_rate=3e-4,
            gamma=0.98, gae_lambda=0.95, clip_range=0.2, ent_coef=0.005, max_grad_norm=1.0,
            policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128]), activation_fn=torch.nn.ELU,
                               log_std_init=-0.5),
            tensorboard_log=str(HERE / "runs"),
        )
    model.learn(total_timesteps=int(args.steps), tb_log_name=args.name, reset_num_timesteps=not args.resume,
                callback=SaveAndScore(run_dir, every=250_000))
    print(f"done: {run_dir}")


if __name__ == "__main__":
    main()
