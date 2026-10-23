"""3D views of training (macOS needs mjpython for the window).

    .venv/bin/mjpython watch.py --live           # a robot that is training right now
    .venv/bin/mjpython watch.py runs/walk        # the newest saved policy, reloaded as training saves
    .venv/bin/mjpython watch.py --nav scripted   # walk/run to random targets with the camera (or a nav .npz)

Saved-policy mode walks on rough terrain (--terrain, default 0.7) with shoves.
--live: train.py's first environment streams every state it simulates (the ground is drawn flat). Training runs ~15x
faster than real time, so the stream is buffered and shown as real-time clips; when the view
falls more than a few seconds behind it jumps to the newest state. Use --fast to show the
newest state every frame instead (fast-forward).

Policy mode cycles forward, turn, sidestep, back and stop commands and resets after a fall.
Close the window to quit.
"""

import argparse
import collections
import pathlib
import socket
import struct
import time

import mujoco.viewer
import numpy as np

from env import STREAM_PORT, RockyBobWalk
from policy import NumpyPolicy
from spec import CONTROL_DT

COMMANDS = [("forward", (0.2, 0.0, 0.0)), ("turn left", (0.0, 0.0, 0.5)), ("sidestep", (0.0, 0.1, 0.0)),
            ("back", (-0.1, 0.0, 0.0)), ("turn right", (0.0, 0.0, -0.5)), ("stand", (0.0, 0.0, 0.0))]
SECONDS_PER_COMMAND = 4.0
MAX_LAG_STEPS = int(3.0 / CONTROL_DT)


def overlay(viewer, *lines):
    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, lines[0], "\n".join(lines[1:])))


def track(viewer, env):
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = env.model.body("base_link").id
    viewer.cam.distance, viewer.cam.elevation = 1.2, -20


def live(fast):
    env = RockyBobWalk(randomize=False)
    env.reset(seed=0)
    nq = env.model.nq
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", STREAM_PORT))
    sock.setblocking(False)
    buf = collections.deque()
    episodes, last_rx = 0, 0.0
    print(f"listening for training on udp {STREAM_PORT} (start or resume train.py)", flush=True)

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        track(viewer, env)
        while viewer.is_running():
            t0 = time.time()
            while True:
                try:
                    raw = sock.recv(4096)
                except BlockingIOError:
                    break
                buf.append(np.array(struct.unpack(f"{len(raw) // 8}d", raw)))
                last_rx = t0
            if fast and buf:
                state = buf[-1]
                buf.clear()
            else:
                if len(buf) > MAX_LAG_STEPS:
                    for _ in range(len(buf) - MAX_LAG_STEPS // 3):
                        buf.popleft()
                state = buf.popleft() if buf else None
            if state is not None:
                env.data.qpos[:] = state[:nq]
                mujoco.mj_forward(env.model, env.data)
                cmd, vel, yaw, reward, fell = state[nq:nq + 3], state[nq + 3:nq + 5], state[nq + 5], state[nq + 6], state[nq + 7]
                episodes += int(fell)
                overlay(viewer, "LIVE TRAINING (env 0)" + ("  fast-forward" if fast else ""),
                        f"cmd    vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f}",
                        f"actual vx={vel[0]:+.2f} vy={vel[1]:+.2f} wz={yaw:+.2f}",
                        f"step reward {reward:+.3f}   falls seen {episodes}")
            elif t0 - last_rx > 2:
                overlay(viewer, "waiting for training...", "start or resume train.py")
            viewer.sync()
            time.sleep(max(0.0, CONTROL_DT - (time.time() - t0)))


def saved_policy(run, level):
    path = pathlib.Path(run) / "policy.npz"
    while not path.exists():
        print(f"waiting for {path} (first save is after ~1M steps)...", flush=True)
        time.sleep(10)
    env = RockyBobWalk(randomize=False, episode_seconds=1e6, command=(0, 0, 0), terrain_level=level, pushes=level > 0)
    obs, _ = env.reset(seed=0)
    policy, stamp = NumpyPolicy(path), path.stat().st_mtime
    print(f"loaded policy ({time.ctime(stamp)})", flush=True)

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        track(viewer, env)
        step = 0
        while viewer.is_running():
            t0 = time.time()
            if path.stat().st_mtime != stamp:
                time.sleep(0.5)   # let the save finish
                policy, stamp = NumpyPolicy(path), path.stat().st_mtime
                print(f"reloaded newer policy ({time.ctime(stamp)})", flush=True)
                obs, _ = env.reset(seed=step)
                viewer.update_hfield(0)
            name, cmd = COMMANDS[int(step * CONTROL_DT / SECONDS_PER_COMMAND) % len(COMMANDS)]
            env.command = np.array(cmd)
            obs, _, fell, _, info = env.step(policy(obs))
            if fell:
                print("fell over: resetting", flush=True)
                obs, _ = env.reset(seed=step)
                viewer.update_hfield(0)
            overlay(viewer, f"saved policy: {name}",
                    f"cmd    vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} wz={cmd[2]:+.2f}",
                    f"actual vx={info['vel'][0]:+.2f} vy={info['vel'][1]:+.2f} wz={info['yaw_rate']:+.2f}")
            viewer.sync()
            step += 1
            time.sleep(max(0.0, CONTROL_DT - (time.time() - t0)))


def navigate(path, level):
    """Goal-seeking demo: the navigator finds and walks to random targets (orange posts)."""
    from nav_env import RockyBobNavigate
    from nav_spec import NAV_DT, ScriptedNavigator, command_to_action
    if path == "scripted":
        scripted = ScriptedNavigator()
        nav = lambda obs: command_to_action(scripted(obs))  # noqa: E731
    else:
        nav = NumpyPolicy(path)
    env = RockyBobNavigate(episode_seconds=1e6, randomize=False, terrain=(level, level))
    obs, _ = env.reset(seed=0)
    with mujoco.viewer.launch_passive(env.walk.model, env.walk.data) as viewer:
        track(viewer, env.walk)
        viewer.update_hfield(0)
        while viewer.is_running():
            t0 = time.time()
            reached = env.reached
            obs, _, fell, _, info = env.step(nav(obs))
            if fell:
                obs, _ = env.reset()
                viewer.update_hfield(0)
            with viewer.lock():
                viewer.user_scn.ngeom = 0
                g = viewer.user_scn.geoms[0]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CYLINDER, [0.015, 0.015, 0.14], 
                                    [env.goal[0], env.goal[1], 0.14], np.eye(3).ravel(), [0.95, 0.45, 0.1, 1])
                viewer.user_scn.ngeom = 1
            seen = "sees target" if info["visible"] else "searching"
            overlay(viewer, f"navigator: {seen}", f"target {info['dist']:.2f} m away",
                    f"cmd vx={info['cmd'][0]:+.2f} vy={info['cmd'][1]:+.2f} wz={info['cmd'][2]:+.2f}",
                    f"targets reached {env.reached}")
            if env.reached > reached:
                print(f"reached target {env.reached}", flush=True)
            viewer.sync()
            time.sleep(max(0.0, NAV_DT - (time.time() - t0)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run", nargs="?", help="run directory for saved-policy mode, e.g. runs/walk")
    p.add_argument("--live", action="store_true", help="show a robot that is training right now")
    p.add_argument("--nav", help="'scripted' or a navigator .npz: walk to random targets using the camera model")
    p.add_argument("--fast", action="store_true", help="with --live: show the newest state (fast-forward)")
    p.add_argument("--terrain", type=float, default=0.7, help="saved-policy mode: 0 flat .. 1 roughest, with shoves")
    args, _ = p.parse_known_args()   # tolerate macOS app flags such as -ApplePersistenceIgnoreState
    if args.nav:
        navigate(args.nav, args.terrain if args.terrain != 0.7 else 0.3)
    elif args.live:
        live(args.fast)
    elif args.run:
        saved_policy(args.run, args.terrain)
    else:
        p.error("give a run directory or --live")


if __name__ == "__main__":
    main()
