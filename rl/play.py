"""Watch or record a trained Rocky-Bob policy in MuJoCo.

    .venv/bin/python play.py runs/walk --video walk.mp4          # record 10 s to a file
    .venv/bin/python play.py runs/walk --video walk.mp4 --cmd 0.2 0 0
    .venv/bin/mjpython play.py runs/walk --viewer                  # live 3D window (macOS needs mjpython)

Uses runs/<name>/policy.npz, the same numpy policy the Webots runner loads, so what you
see here is exactly what gets deployed. Prints the commanded vs achieved speed.
"""

import argparse
import pathlib
import sys
import time

import numpy as np

from env import RockyBobWalk
from spec import CONTROL_DT
from policy import NumpyPolicy


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run", help="run directory containing policy.npz")
    p.add_argument("--video", help="write an mp4 here")
    p.add_argument("--viewer", action="store_true")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--terrain", type=float, default=0.0, help="0 flat .. 1 roughest (adds shoves)")
    p.add_argument("--cmd", type=float, nargs=3, default=[0.2, 0.0, 0.0], metavar=("VX", "VY", "WZ"))
    args = p.parse_args()

    policy = NumpyPolicy(pathlib.Path(args.run) / "policy.npz")
    env = RockyBobWalk(randomize=False, episode_seconds=args.seconds, command=args.cmd,
                       terrain_level=args.terrain, pushes=args.terrain > 0)
    obs, _ = env.reset(seed=0)

    frames, speeds, yaw, torque = [], [], [], []
    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(env.model, env.data)

    done, falls = False, 0
    while not done:
        t0 = time.time()
        obs, _, term, trunc, info = env.step(policy(obs))
        speeds.append(info["vel"][:2])
        yaw.append(info["yaw_rate"])
        torque.append(info["torque"])
        if term:
            falls += 1
        done = term or trunc
        if args.video and env.steps % 2 == 0:   # 25 fps
            frames.append(env.render())
        if viewer is not None:
            viewer.sync()
            time.sleep(max(0.0, CONTROL_DT - (time.time() - t0)))
            if not viewer.is_running():
                break

    settled = len(speeds) // 4
    v = np.mean(speeds[settled:], axis=0)
    print(f"command vx={args.cmd[0]:.2f} vy={args.cmd[1]:.2f} wz={args.cmd[2]:.2f} | "
          f"achieved vx={v[0]:.3f} vy={v[1]:.3f} wz={np.mean(yaw[settled:]):.2f} | "
          f"peak servo torque {np.percentile(torque, 95):.2f} N m (95th pct) | lasted {env.steps * CONTROL_DT:.1f} s"
          f"{' (fell)' if falls else ''}")
    if args.video:
        import imageio
        imageio.mimsave(args.video, frames, fps=25, macro_block_size=8)
        print(f"wrote {args.video}")
    env.close()
    return 0 if not falls else 1


if __name__ == "__main__":
    sys.exit(main())
