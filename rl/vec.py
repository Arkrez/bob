"""Batched subprocess vector env: each worker process runs several environments.

Stable-Baselines3's SubprocVecEnv runs one environment per process and exchanges a message with
every process on every control step. For Rocky-Bob that round trip (~1.8 ms for 12 processes)
costs far more than the 0.33 ms physics step, so the cores sit half idle. Here each worker steps
its own group of environments (a DummyVecEnv, which also handles auto-reset and episode info) and
sends back the whole group in one message, so the overhead is shared by many robots.
"""

import multiprocessing as mp

import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv, VecEnv
from stable_baselines3.common.vec_env.base_vec_env import CloudpickleWrapper


def _worker(remote, parent, env_fns):
    parent.close()
    venv = DummyVecEnv(env_fns.var)
    try:
        while True:
            cmd, data = remote.recv()
            if cmd == "step":
                remote.send(venv.step(data))
            elif cmd == "reset":
                remote.send(venv.reset())
            elif cmd == "seed":
                venv.seed(data)
                remote.send(None)
            elif cmd == "env_method":
                name, args, kwargs = data
                remote.send(venv.env_method(name, *args, **kwargs))
            elif cmd == "get_attr":
                remote.send(venv.get_attr(data))
            elif cmd == "set_attr":
                venv.set_attr(*data)
                remote.send(None)
            elif cmd == "is_wrapped":
                remote.send(venv.env_is_wrapped(data))
            elif cmd == "close":
                venv.close()
                remote.close()
                break
    except KeyboardInterrupt:
        pass


class BatchedSubprocVecEnv(VecEnv):
    def __init__(self, env_fns, workers):
        n = len(env_fns)
        workers = min(workers, n)
        self.groups = [list(g) for g in np.array_split(np.arange(n), workers)]
        # forkserver on macOS/Linux; Windows only has spawn
        ctx = mp.get_context("forkserver" if "forkserver" in mp.get_all_start_methods() else "spawn")
        self.remotes, self.procs = [], []
        for g in self.groups:
            remote, child = ctx.Pipe()
            fns = CloudpickleWrapper([env_fns[i] for i in g])   # lambdas need cloudpickle
            p = ctx.Process(target=_worker, args=(child, remote, fns), daemon=True)
            p.start()
            child.close()
            self.remotes.append(remote)
            self.procs.append(p)
        self.remotes[0].send(("get_attr", "observation_space"))
        obs_space = self.remotes[0].recv()[0]
        self.remotes[0].send(("get_attr", "action_space"))
        act_space = self.remotes[0].recv()[0]
        super().__init__(n, obs_space, act_space)
        self._actions = None

    def _gather(self, results):
        obs = np.concatenate([r[0] for r in results])
        rews = np.concatenate([r[1] for r in results])
        dones = np.concatenate([r[2] for r in results])
        infos = [i for r in results for i in r[3]]
        return obs, rews, dones, infos

    def step_async(self, actions):
        for remote, g in zip(self.remotes, self.groups):
            remote.send(("step", actions[g]))

    def step_wait(self):
        return self._gather([remote.recv() for remote in self.remotes])

    def reset(self):
        for remote in self.remotes:
            remote.send(("reset", None))
        return np.concatenate([remote.recv() for remote in self.remotes])

    def seed(self, seed=None):
        for k, remote in enumerate(self.remotes):
            remote.send(("seed", None if seed is None else seed + int(self.groups[k][0])))
        return [remote.recv() for remote in self.remotes]

    def close(self):
        for remote in self.remotes:
            remote.send(("close", None))
        for p in self.procs:
            p.join()

    def _call(self, cmd, data, indices):
        indices = range(self.num_envs) if indices is None else ([indices] if isinstance(indices, int) else indices)
        wanted = set(indices)
        out = []
        for remote, g in zip(self.remotes, self.groups):
            remote.send((cmd, data))
        for remote, g in zip(self.remotes, self.groups):
            res = remote.recv()
            if res is not None:
                out += [r for i, r in zip(g, res) if i in wanted]
        return out

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        return self._call("env_method", (method_name, method_args, method_kwargs), indices)

    def get_attr(self, attr_name, indices=None):
        return self._call("get_attr", attr_name, indices)

    def set_attr(self, attr_name, value, indices=None):
        self._call("set_attr", (attr_name, value), indices)

    def env_is_wrapped(self, wrapper_class, indices=None):
        return self._call("is_wrapped", wrapper_class, indices)
