"""Score a saved policy and record a short clip: one row in the progress table per snapshot.

    .venv/bin/python progress.py runs/walk/progress/policy_07M.npz

train.py runs this automatically every 1M steps (in the background, so training doesn't wait).
Writes next to the policy: step_07M.mp4 and a row in progress.md / progress.csv.
"""

import argparse
import pathlib

import numpy as np

from env import RockyBobWalk
from policy import NumpyPolicy

# (name, command, terrain level, shoves): flat tests stay comparable across runs; the last one is
# the roughest terrain with shoves
TESTS = [("forward", (0.15, 0.0, 0.0), 0.0, False), ("turn", (0.0, 0.0, 0.4), 0.0, False),
         ("sidestep", (0.0, 0.08, 0.0), 0.0, False), ("rough", (0.15, 0.0, 0.0), 1.0, True),
         ("run", (0.30, 0.0, 0.0), 0.0, False)]
# Consistency: forward 0.15 m/s on medium terrain with shoves and randomised physics, several seeds.
CONSISTENCY_SEEDS = 4
SECONDS = 6.0


def run(policy, cmd, frames, level=0.0, pushes=False, randomize=False, seed=0):
    env = RockyBobWalk(randomize=randomize, episode_seconds=SECONDS, command=cmd, terrain_level=level, pushes=pushes)
    obs, _ = env.reset(seed=seed)
    vel, yaw, torque, joints, drag, trot, steady, ride, fell = [], [], [], [], [], [], [], [], False
    done = False
    while not done:
        obs, _, fell, trunc, info = env.step(policy(obs))
        joints.append(env.data.qpos[7:].copy())
        vel.append(info["vel"][:2])
        yaw.append(info["yaw_rate"])
        torque.append(info["torque"])
        drag.append(info["terms"]["foot_clearance"])
        trot.append(info["terms"]["gait"])
        steady.append(info["terms"]["steady"])
        ride.append(info["terms"]["smooth_ride"])
        if frames is not None and env.steps % 2 == 0:
            frames.append(env.render())
        done = fell or trunc
    env.close()
    settled = len(vel) // 3
    q = np.array(joints[settled:])
    dq = np.diff(q, axis=0)
    seconds = len(dq) * env.model.opt.timestep * env.frame_skip
    reversals = float(np.mean(np.sum(np.diff(np.sign(dq), axis=0) != 0, axis=0)) / seconds)
    distance = float(np.linalg.norm(np.mean(vel[settled:], axis=0)) * seconds)
    travel = float(np.sum(np.abs(dq)) / max(distance, 1e-6))
    return (np.mean(vel[settled:], axis=0), float(np.mean(yaw[settled:])), float(np.percentile(torque, 95)), fell,
            reversals, travel, float(np.mean(drag[settled:])), float(np.mean(trot[settled:])),
            float(np.mean(steady[settled:])), float(np.mean(ride[settled:])))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("policy", help="a policy_XXM.npz snapshot")
    args = p.parse_args()
    path = pathlib.Path(args.policy)
    tag = path.stem.replace("policy_", "")
    policy = NumpyPolicy(path)

    frames, results = [], {}
    for name, cmd, level, pushes in TESTS:
        results[name] = run(policy, cmd, frames, level, pushes)
    spread = [run(policy, (0.15, 0.0, 0.0), None, 0.5, True, randomize=True, seed=s)
              for s in range(1, CONSISTENCY_SEEDS + 1)]
    spread_v = [r[0][0] for r in spread]
    spread_falls = sum(int(r[3]) for r in spread)

    import imageio
    imageio.mimsave(path.with_name(f"step_{tag}.mp4"), frames, fps=25, macro_block_size=8)

    fwd, turn, side, rough, run_ = (results[k] for k in ("forward", "turn", "sidestep", "rough", "run"))
    falls = sum(int(r[3]) for r in results.values())
    torque = max(r[2] for r in results.values())
    row = (f"| {tag} | {fwd[0][0]:.3f} / 0.15 | {run_[0][0]:.3f} / 0.30 | {turn[1]:+.2f} / 0.40 | {side[0][1]:+.3f} / 0.08 | "
           f"{falls} | {torque:.2f} | {fwd[4]:.1f} | {fwd[5]:.0f} | {fwd[6]:.2f} | "
           f"{rough[0][0]:.3f} / 0.15{' FELL' if rough[3] else ''} | "
           f"{np.mean(spread_v):.3f} (worst {min(spread_v):.3f}){f' {spread_falls} fell' if spread_falls else ''} | "
           f"{100 * fwd[7]:.0f}% | {fwd[8]:.2f} | {fwd[9]:.2f} | [video](step_{tag}.mp4) |")
    md = path.with_name("progress.md")
    if not md.exists():
        md.write_text("# Training progress\n\nEach row is a 6 s test per command, without randomisation. "
                      "Achieved / commanded.\n\n"
                      "Smoothness is from the forward test: joint direction reversals per second "
                      "(a clean trot here is ~3; jitter pushes it up) and joint travel per metre walked. "
                      "Foot drag: mean penalty for feet moving below 3 cm (0 = clean steps, 4 = every foot dragging). "
                      "Rough: forward on the roughest terrain with shoves. Consistency: forward 0.15 m/s on medium "
                      "terrain with shoves and randomised physics, 4 seeds, mean and worst. Trot: share of time the "
                      "feet match the diagonal trot schedule (forward test). Steady / smooth ride: the stable-body "
                      "bonuses in the forward test, 1 = perfectly still body.\n\n"
                      "| steps | forward m/s | run m/s | turn rad/s | sidestep m/s | falls | servo torque N m (95th pct, stall 0.94) "
                      "| reversals /s | joint travel rad/m | foot drag | rough m/s | consistency m/s | trot | steady | smooth ride | clip |\n"
                      "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
    with md.open("a") as f:
        f.write(row + "\n")
    csv = path.with_name("progress.csv")
    if not csv.exists():
        csv.write_text("steps,forward,run,turn,sidestep,falls,torque95,reversals_per_s,travel_rad_per_m,foot_drag,"
                       "rough_forward,rough_fell,consistency_mean,consistency_worst,consistency_falls,trot,steady,smooth_ride\n")
    with csv.open("a") as f:
        f.write(f"{tag},{fwd[0][0]:.4f},{run_[0][0]:.4f},{turn[1]:.4f},{side[0][1]:.4f},{falls},{torque:.3f},{fwd[4]:.2f},{fwd[5]:.1f},{fwd[6]:.3f},{rough[0][0]:.4f},{int(rough[3])},"
                f"{np.mean(spread_v):.4f},{min(spread_v):.4f},{spread_falls},{fwd[7]:.3f},{fwd[8]:.3f},{fwd[9]:.3f}\n")
    print(row, flush=True)


if __name__ == "__main__":
    main()
