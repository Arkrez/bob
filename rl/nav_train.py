"""Train the goal-reaching navigator (nav_env.py) with PPO, on top of the frozen walker.

    .venv/bin/python nav_train.py                 # 3M navigator steps (~10 days of robot time)
    .venv/bin/python nav_train.py --steps 1e6 --name nav_quick
    .venv/bin/python nav_train.py --obstacles --imitate 300000 --name nav_lidar   # lidar navigator

Writes runs/<name>/: latest.zip, vecnormalize.pkl, policy.npz (numpy navigator) and, every
250k steps, a row in progress.md from nav_eval.py (goals per minute, time to goal, path
efficiency, sideways use, falls). The scripted baseline is the first row for comparison.

--obstacles trains among random walls and boxes with the lidar in the observation, and big
penalties for bumping (nav_env.py). --imitate N first copies the scripted navigator (with its lidar
obstacle avoidance) over N steps of its own driving, so PPO starts from something that already
works and only has to improve on it.
"""

import argparse
import os
import pathlib
import shutil
import subprocess
import sys

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

import numpy as np

from nav_env import WALKER, RockyBobNavigate
from nav_spec import ScriptedNavigator, command_to_action
from train import export_numpy

HERE = pathlib.Path(__file__).resolve().parent


def imitate(model, venv, make_env, steps, log):
    """Behaviour cloning: drive with the scripted navigator, fit the policy's mean action to it,
    and set the observation normaliser from what was seen."""
    rng = np.random.default_rng(0)
    obs_all, act_all = [], []
    env, nav, obs = None, None, None
    while len(obs_all) < steps:
        if env is None:
            env, nav = make_env(), ScriptedNavigator()
            obs, _ = env.reset(seed=int(rng.integers(1 << 30)))
        act = np.clip(command_to_action(nav(obs)), -1, 1)
        obs_all.append(obs); act_all.append(act)
        # drive with a little noise so it also sees how to recover from slightly-off commands
        obs, _, fell, trunc, _ = env.step(np.clip(act + rng.normal(0, 0.15, 3), -1, 1))
        if fell or trunc:
            env = None
    X, Y = np.array(obs_all, np.float32), np.array(act_all, np.float32)
    venv.obs_rms.mean, venv.obs_rms.var, venv.obs_rms.count = X.mean(0), X.var(0) + 1e-6, len(X)
    Xn = torch.tensor(np.clip((X - X.mean(0)) / np.sqrt(X.var(0) + 1e-6), -10, 10))
    Yt = torch.tensor(Y)
    pol = model.policy
    opt = torch.optim.Adam(list(pol.mlp_extractor.policy_net.parameters()) + list(pol.action_net.parameters()), lr=1e-3)
    for epoch in range(30):
        perm = torch.randperm(len(Xn))
        total = 0.0
        for i in range(0, len(Xn), 1024):
            idx = perm[i:i + 1024]
            mean = pol.action_net(pol.mlp_extractor.policy_net(Xn[idx]))
            loss = ((mean - Yt[idx]) ** 2).mean()
            opt.zero_grad(); loss.backward(); opt.step()
            total += loss.item() * len(idx)
        log.write(f"imitation epoch {epoch}: mse {total / len(Xn):.4f}\n"); log.flush()


class SaveAndScore(BaseCallback):
    def __init__(self, run_dir, every, eval_args=()):
        super().__init__()
        self.run_dir, self.every, self.next = run_dir, every, every
        self.eval_args = list(eval_args)

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
        # the full checkpoint too, so train_rounds.py can continue from whichever snapshot wins
        shutil.copy(self.run_dir / "latest.zip", snap.with_suffix(".zip"))
        shutil.copy(self.run_dir / "vecnormalize.pkl", snap.with_name(snap.stem + "_vecnormalize.pkl"))
        with open(self.run_dir / "progress.md", "a") as log:
            subprocess.Popen(["nice", "-n", "10", sys.executable, str(HERE / "nav_eval.py"), str(snap), *self.eval_args],
                             stdout=log, stderr=subprocess.STDOUT, cwd=HERE)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--steps", type=float, default=3e6)
    p.add_argument("--envs", type=int, default=os.cpu_count())
    p.add_argument("--name", default="nav")
    p.add_argument("--resume", help="path to a latest.zip to keep training (vecnormalize.pkl beside it)")
    p.add_argument("--obstacles", action="store_true", help="walls and boxes, lidar observations, bump penalties")
    p.add_argument("--walker", default=str(WALKER), help="frozen walker .npz")
    p.add_argument("--imitate", type=int, default=0, help="first copy the scripted navigator over this many steps")
    args = p.parse_args()
    eval_args = ["--obstacles"] if args.obstacles else []

    def make_env():
        return RockyBobNavigate(obstacles=args.obstacles, walker=args.walker)

    run_dir = HERE / "runs" / args.name
    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / "progress.md", "w") as log:
        log.write("# Navigator progress (nav_eval.py: 6 x 60 s, medium terrain, shoves, camera model)\n\n")
        log.flush()
        for nav in (["blind", "baseline"] if args.obstacles else ["baseline"]):
            subprocess.run([sys.executable, str(HERE / "nav_eval.py"), nav, *eval_args], stdout=log, cwd=HERE, check=True)

    venv = SubprocVecEnv([lambda: Monitor(make_env()) for _ in range(args.envs)])
    venv.seed(0)
    if args.resume:
        venv = VecNormalize.load(str(pathlib.Path(args.resume).with_name("vecnormalize.pkl")), venv)
        model = PPO.load(args.resume, env=venv, device="cpu")
    else:
        venv = VecNormalize(venv, norm_obs=True, norm_reward=True, clip_obs=10.0, gamma=0.98)
        model = PPO(
            "MlpPolicy", venv, device="cpu", seed=0, verbose=1,
            n_steps=512, batch_size=2048, n_epochs=8, learning_rate=1e-4 if args.imitate else 3e-4,
            target_kl=0.02 if args.imitate else None,
            gamma=0.98, gae_lambda=0.95, clip_range=0.2, ent_coef=0.005, max_grad_norm=1.0,
            policy_kwargs=dict(net_arch=dict(pi=[256, 256] if args.obstacles else [128, 128],
                                             vf=[256, 256] if args.obstacles else [128, 128]),
                               activation_fn=torch.nn.ELU, log_std_init=-1.2 if args.imitate else -0.5),
            tensorboard_log=str(HERE / "runs"),
        )
        if args.imitate:
            with open(run_dir / "progress.md", "a") as log:
                imitate(model, venv, make_env, args.imitate, log)
            export_numpy(model, venv, run_dir / "nav_imitated.npz")
            with open(run_dir / "progress.md", "a") as log:
                subprocess.run([sys.executable, str(HERE / "nav_eval.py"), str(run_dir / "nav_imitated.npz"), *eval_args],
                               stdout=log, cwd=HERE, check=True)
    model.learn(total_timesteps=int(args.steps), tb_log_name=args.name, reset_num_timesteps=not args.resume,
                callback=SaveAndScore(run_dir, every=250_000, eval_args=eval_args))
    print(f"done: {run_dir}")


if __name__ == "__main__":
    main()
