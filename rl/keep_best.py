"""Keep the best walker from a long training run: score every 1M-step snapshot, keep the winner.

    .venv/bin/python keep_best.py runs/walk_turn2 --baseline policies/rocky_walk.npz

Long fine-tunes drift: a run can get better at what it is rewarded for and quietly worse at
something else (walk_turn turned best after 4M steps, then lost running speed). This watches
runs/<name>/progress/ for the policy_<n>M.npz snapshots train.py writes, scores each one, and
copies it to runs/<name>/best.npz only if it beats the best so far, the baseline included.

The score adds up what we want from the walker, each scaled so the installed walker is ~1:
turn rate and footfalls per radian at a 1.0 rad/s command, walking and top speed, step height,
pivot bounces, how far he drifts sideways walking straight or slides turning in place, and how
often he gets over a staircase (STAIR_CASES: 8 kinds, 2 tries each) and over taller steps and
ledges (TALL_CASES: 7-10 cm stairs, 15-30 cm ledges), and how fast he locates a hidden goal
with the planner driving him (find_time).
A snapshot that falls, or loses more than 10% of the baseline's walking or top
speed, is never kept, however well it turns. Every result goes to runs/<name>/best.md.
Webots can follow it live (scripts/start_sim.sh --follow-training runs/<name>). To make the winner
the default: cp runs/<name>/best.npz policies/rocky_walk.npz
"""

import argparse
import os
import pathlib
import re
import shutil
import time

import numpy as np

from env import RockyBobWalk
from policy import NumpyPolicy
from spec import quat_matrix
from step_eval import measure
from turn_eval import turn

# the installed walker (walk_turn 48M) on these tests, so each term is ~1 for it
REFERENCE = {"turn": 0.72, "footfalls": 8.4, "walk": 0.198, "top": 0.283, "step": 3.4, "bounce": 1.2,
             "drift": 35.0, "slide": 4.0,
             "find": 14.7}   # s, the 178M walker with the planner (keep_best.find_time)


def speeds(policy, seeds=3):
    """Mean forward speed reached at a 0.15 m/s command, best mean over the faster ones, and falls."""
    out, falls = {}, 0
    for v in (0.15, 0.30, 0.45):
        vel = []
        for s in range(seeds):
            env = RockyBobWalk(randomize=True, episode_seconds=8, command=(v, 0, 0), terrain_level=0.0, pushes=False)
            obs, _ = env.reset(seed=1200 + s)
            for i in range(400):
                obs, _, fell, _, info = env.step(policy(obs))
                if i >= 100:
                    vel.append(info["vel"][0])
                if fell:
                    falls += 1
                    break
        out[v] = float(np.mean(vel)) if vel else 0.0
    return out[0.15], max(out.values()), falls


def sideways(policy, cmd, secs, seeds=6):
    """Mean |sideways travel| (cm) relative to the starting heading: curving or crabbing off a
    straight walk, or sliding off the spot while turning in place."""
    out = []
    for s in range(seeds):
        env = RockyBobWalk(randomize=True, episode_seconds=secs + 1, command=cmd, terrain_level=0.0, pushes=False)
        obs, _ = env.reset(seed=900 + s)
        p0 = env.data.qpos[:2].copy()
        left = quat_matrix(env.data.qpos[3:7])[:2, 1].copy()
        for _ in range(int(secs / 0.02)):
            obs, *_ = env.step(policy(obs))
        out.append((env.data.qpos[:2] - p0) @ left)
    return 100 * abs(float(np.mean(out)))


# Stair test cases: (kind, riser heights (m), tread (m), angle of the stairs to his path (rad)).
STAIR_CASES = [("pyramid", [0.03] * 3, 0.32, 0.0), ("pyramid", [0.04] * 3, 0.32, 0.0), ("pyramid", [0.05] * 3, 0.32, 0.0),
               ("flight", [0.04] * 3, 0.40, 0.0), ("flight", [0.04] * 4, 0.25, 0.0),
               ("flight", [0.03, 0.05, 0.035], 0.32, 0.0), ("flight", [0.04] * 3, 0.32, np.radians(30)),
               ("kerb", [0.05], 0.6, 0.0)]
# Taller ones: stairs his belly can clear with his legs straightened, and ledges up to 30 cm, above
# his hips, where he may belly-climb (belly contact is allowed over a ledge above BELLY_LEDGE).
TALL_CASES = [("flight", [0.07] * 2, 0.35, 0.0), ("flight", [0.10] * 2, 0.40, 0.0),
              ("flight", [0.06, 0.09, 0.08], 0.40, 0.0), ("kerb", [0.15], 0.6, 0.0),
              ("kerb", [0.20], 0.6, 0.0), ("kerb", [0.30], 0.8, 0.0)]
BELLY_LEDGE = 0.10


def stair_field(env, kind, rises, tread, angle, p0, ahead):
    """Height field with one staircase across his path, starting 0.6 m ahead. Returns it and the
    distance along his path he must reach to be over it."""
    nr, nc = env.field_shape
    hx, hy = env.field_half
    X, Y = np.meshgrid(np.linspace(-hx, hx, nc), np.linspace(-hy, hy, nr))
    yaw = np.arctan2(ahead[1], ahead[0]) + angle
    co, si = np.cos(yaw), np.sin(yaw)
    start = p0 + 0.6 * ahead
    u = (X - start[0]) * co + (Y - start[1]) * si
    v = -(X - start[0]) * si + (Y - start[1]) * co
    tops = np.r_[0.0, np.cumsum(rises)]
    n = len(rises)
    if kind == "pyramid":
        landing = 0.5
        half = landing + n * tread
        cheb = np.maximum(np.abs(u - half), np.abs(v))
        k = np.clip(np.floor((half - cheb) / tread), -1, n - 1).astype(int)
        h, length = tops[k + 1], 2 * half
    elif kind == "flight":   # up, landing, down
        landing = 0.6
        up = tops[np.clip(np.floor(u / tread) + 1, 0, n).astype(int)]
        far = n * tread + landing
        down = tops[n - np.clip(np.floor((u - far) / tread) + 1, 0, n).astype(int)]
        length = far + n * tread
        h = np.where(u > far, down, up) * ((u >= 0) & (u < length) & (np.abs(v) < 1.5))
    else:                    # kerb: one step up, a strip, one step down
        length = tread
        h = rises[0] * ((u >= 0) & (u < length) & (np.abs(v) < 1.5))
    return np.asarray(h, float) / env.field_max, 0.6 + length / max(np.cos(angle), 0.5) + 0.3


def stairs(policy, seeds=2, cases=STAIR_CASES):
    """Across STAIR_CASES (pyramids of 3-5 cm, deep and tight treads, uneven steps, a 30 degree
    approach and a kerb): the share of runs that get right over, and how hesitant he is on the way:
    taps (touchdowns after a tiny lift) and seconds spent moving backwards, per run."""
    ok, taps, back, times = [], [], [], []
    for kind, rises, tread, angle in cases:
        for s in range(seeds):
            env = RockyBobWalk(randomize=True, episode_seconds=30, command=(0.2, 0, 0), terrain_level=0.0, pushes=False)
            obs, _ = env.reset(seed=1500 + s)
            p0 = env.data.qpos[:2].copy()
            ahead = quat_matrix(env.data.qpos[3:7])[:2, 0]
            ahead = ahead / np.linalg.norm(ahead)
            env.field, far = stair_field(env, kind, rises, tread, angle, p0, ahead)
            env.model.hfield_data[:] = env.field.ravel()
            if kind == "kerb" and rises[0] > BELLY_LEDGE:
                env.belly_zone = (p0, ahead, 0.6 - 0.35, 0.6 + tread + 0.35)
            done, t, b = False, 0.0, 0.0
            for step in range(int(30 / 0.02)):
                obs, _, fell, _, info = env.step(policy(obs))
                t += info["terms"]["tap"]
                b += 0.02 * (info["vel"][0] < -0.02)
                if fell or (env.data.qpos[:2] - p0) @ ahead > far:
                    done = not fell
                    break
            ok.append(done)
            taps.append(t)
            back.append(b)
            if done:
                times.append((step + 1) * 0.02)
    return float(np.mean(ok)), float(np.mean(taps)), float(np.mean(back)), float(np.mean(times)) if times else 30.0


def find_time(path, runs=4, seconds=45):
    """Mean seconds until a new hidden goal (behind a wall) is first in view, with the planner
    navigator driving this walker."""
    from nav_eval import run
    finds = []
    for i in range(runs):
        finds += run("planner", seed=700 + i, seconds=seconds, obstacles=True, goals="hidden", walker=str(path))[-1]
    return float(np.mean(finds)) if finds else float(seconds)


def evaluate(path):
    policy = NumpyPolicy(str(path))
    rows = [turn(policy, wz, level, seed) for wz in (1.0, -1.0) for level in (0.0, 0.3) for seed in range(2)]
    yaw, steps, _, _, fell = (np.array(c, dtype=float) for c in zip(*rows))
    walk, top, speed_falls = speeds(policy)
    walk_steps, _, _ = measure(policy, (0.15, 0, 0), seeds=2)
    _, pivot_bounce, _ = measure(policy, (0, 0, 0.6), seeds=2)
    return {"turn": float(np.mean(yaw) / 10), "footfalls": float(np.sum(steps) / max(np.sum(yaw), 1e-6)),
            "walk": walk, "top": top, "step": float(np.nanmean(walk_steps)), "bounce": float(pivot_bounce),
            "falls": int(np.sum(fell)) + speed_falls,
            "drift": sideways(policy, (0.15, 0, 0), 6.0),
            "slide": 0.5 * (sideways(policy, (0, 0, 0.6), 3.0) + sideways(policy, (0, 0, -0.6), 3.0)),
            **dict(zip(("stairs", "stair_taps", "stair_back", "stair_time"), stairs(policy))),
            "tall": stairs(policy, cases=TALL_CASES)[0], "find": find_time(path)}


def publish(src, dst):
    """Copy atomically: the Webots bridge may be reading dst to follow the run."""
    tmp = dst.with_suffix(".tmp.npz")
    shutil.copy(src, tmp)
    os.replace(tmp, dst)


def score(r):
    R = REFERENCE
    return (2.0 * r["turn"] / R["turn"] + 1.5 * R["footfalls"] / max(r["footfalls"], 1e-6)
            + r["walk"] / R["walk"] + 1.5 * r["top"] / R["top"] + r["step"] / R["step"]
            - 0.5 * r["bounce"] / R["bounce"]
            - 0.5 * r["drift"] / R["drift"] - 0.5 * r["slide"] / R["slide"]
            + 3.0 * r["stairs"] - 0.3 * min(r["stair_taps"] / 10, 1) - 0.3 * min(r["stair_back"] / 3, 1)
            + 0.5 * (1 - min(r["stair_time"] / 30, 1))
            + 2.0 * r.get("tall", 0.0) + R["find"] / max(r.get("find", 45.0), 1.0)) / 9.5


def acceptable(r, base):
    # walking: commanded 0.15 m/s, so tracking it closely is fine even if the champion overshoots it
    walk_ok = r["walk"] >= min(0.9 * base["walk"], 0.9 * 0.15)
    return r["falls"] == 0 and walk_ok and r["top"] >= 0.9 * base["top"]


def line(name, r, s, note=""):
    return (f"| {name} | {r['turn']:.2f} | {r['footfalls']:.1f} | {r['walk']:.3f} | {r['top']:.3f} | "
            f"{r['step']:.1f} | {r['bounce']:.1f} | {r['drift']:.0f} | {r['slide']:.1f} | {r['stairs']:.0%} | {r['stair_time']:.0f} | {r['stair_taps']:.1f} | {r['stair_back']:.1f} | {r.get('tall', 0):.0%} | {r.get('find', float('nan')):.1f} | {r['falls']} | {s:.3f} | {note} |")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run", help="runs/<name>")
    p.add_argument("--baseline", default="policies/rocky_walk.npz")
    p.add_argument("--poll", type=float, default=60.0)
    args = p.parse_args()
    run = pathlib.Path(args.run)
    report = run / "best.md"
    best_path = run / "best.npz"

    base = evaluate(args.baseline)
    best = score(base)
    publish(args.baseline, best_path)
    with open(report, "w") as f:
        f.write("| snapshot | turn rad/s (cmd 1.0) | footfalls/rad | walk 0.15 | top speed | step cm | pivot bounces/s | walk drift cm | turn slide cm | stairs | stair s | taps/run | back s/run | tall | find s | falls | score | |\n")
        f.write("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n")
        f.write(line(f"baseline ({pathlib.Path(args.baseline).name})", base, best, "kept") + "\n")
    print(f"baseline score {best:.3f}", flush=True)

    seen = set()
    while True:
        snaps = sorted(run.glob("progress/policy_*M.npz"), key=lambda q: int(re.findall(r"(\d+)M", q.name)[-1]))
        new = [q for q in snaps if q not in seen]
        for q in new:
            seen.add(q)
            time.sleep(2)   # let the writer finish
            r = evaluate(q)
            s = score(r)
            note = ""
            if not acceptable(r, base):
                note = "rejected (fell or slower)"
            elif s > best:
                best, note = s, "**new best**"
                publish(q, best_path)
            with open(report, "a") as f:
                f.write(line(q.stem.replace("policy_", ""), r, s, note) + "\n")
            print(f"{q.name}: score {s:.3f} {note}", flush=True)
        if not new:
            log = run.parent / f"{run.name}_train.log"   # train.py's log, as started below
            if log.exists() and "done:" in log.read_text()[-2000:]:
                return
            time.sleep(args.poll)


if __name__ == "__main__":
    main()
