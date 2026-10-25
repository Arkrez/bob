"""Train Bob some more, on demand, keeping only what makes him better.

    .venv/bin/python train_rounds.py --hours 3      (scripts/train_more.sh starts it in the background)
    .venv/bin/python train_rounds.py --rounds 2
    touch runs/rounds/STOP                          finish the current round, then stop early

It runs rounds until the budget is used (it never stops mid-round), then exits. The champions and
the round counter live in runs/rounds/, so each session carries on where the last one stopped.

Rounds:
  walker round (every round): continue the champion walker for WALK_STEPS on the full curriculum
      (rough ground, blocks, stairs, step-up platforms, shoves). keep_best.py scores every 1M-step
      snapshot against the champion (turning, speed, steps, drift, stairs); if one beats it, it
      becomes the champion and is installed as policies/rocky_walk.npz. A running Webots bridge
      reloads that file by itself, so the sim always walks with the latest champion.
  self-righting round (every RECOVER_EVERY rounds): continue the champion self-righting policy
      (recover_env.py) for RECOVER_STEPS; recover_eval.py scores each snapshot (getting up, how
      fast, and how little it leans on the lidar); a winner is installed as policies/rocky_recover.npz.
  navigator round (every NAV_EVERY rounds): continue the champion lidar navigator for NAV_STEPS
      among random walls and boxes, on top of the current walker. Its snapshots and the champion
      are scored with nav_eval.py --obstacles (goals/min, minus bumps, falls and the seconds it takes
      to locate each new goal); a winner is
      installed as policies/rocky_nav_lidar.npz.
Each round's leftovers (losing snapshots, videos, checkpoints) are deleted to save disk; the score
tables stay. Old champions are kept in policies/history/ (the last HISTORY of each). Everything
is logged to runs/rounds/log.md. A crashed round is logged and skipped.
A walker round takes about 30 minutes, a navigator round about 30, a self-righting round about 15.
"""

import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
RUNS = HERE / "runs"
STATE_DIR = RUNS / "rounds"
STATE = STATE_DIR / "state.json"
LOG = STATE_DIR / "log.md"
STOP = STATE_DIR / "STOP"
POLICIES = HERE / "policies"
HISTORY = 10
PY = sys.executable

WALK_STEPS = 10e6
WALK_LR, WALK_KL = 3e-5, 0.005  # 1e-4 learned the floor sensor in 1M steps, then collapsed; keep it gentle
RESET_AFTER = 4         # rounds without a new champion before training restarts from the champion
NAV_EVERY = 2
RECOVER_EVERY = 3
RECOVER_STEPS = 5e6
NAV_STEPS = 1e6
NAV_EVAL_RUNS = 20
MIN_FREE_GB = 8         # pause (and say so) rather than fill the disk


def log(msg):
    line = f"- {time.strftime('%Y-%m-%d %H:%M')} {msg}"
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(line + "\n")


def load_state():
    if STATE.exists():
        return json.loads(STATE.read_text())
    return {"round": 0, "walker": None, "nav": None, "recover": None}


def save_state(state):
    tmp = STATE.with_suffix(".tmp")
    tmp.write_text(json.dumps(state, indent=2))
    os.replace(tmp, STATE)


def install(src, dst, kind):
    """Atomically replace dst (the Webots bridge may be reading it) and archive the old one."""
    dst = pathlib.Path(dst)
    hist = POLICIES / "history"
    hist.mkdir(exist_ok=True)
    if dst.exists():
        shutil.copy(dst, hist / f"{dst.stem}_{time.strftime('%Y%m%d_%H%M')}.npz")
        old = sorted(hist.glob(f"{dst.stem}_*.npz"))
        for f in old[:-HISTORY]:
            f.unlink()
    tmp = dst.with_suffix(".tmp.npz")
    shutil.copy(src, tmp)
    os.replace(tmp, dst)
    log(f"installed new {kind} champion -> policies/{dst.name}")


def free_gb():
    return shutil.disk_usage(HERE).free / 1e9


def wait_for_disk():
    while free_gb() < MIN_FREE_GB:
        log(f"only {free_gb():.1f} GB free: paused until there is {MIN_FREE_GB} GB")
        time.sleep(1800)


def other_training():
    """Another training job (started by hand) already using the CPU? (psutil: works on Windows too)"""
    import psutil
    found = []
    for p in psutil.process_iter(["pid", "cmdline"]):
        cmd = " ".join(p.info["cmdline"] or [])
        if p.info["pid"] != os.getpid() and re.search(r"(nav_)?train\.py", cmd) and "train_rounds" not in cmd:
            found.append(p.info["pid"])
    return found


def keep_awake():
    """Windows: stop the PC sleeping while training (macOS uses caffeinate in train_more.sh)."""
    if os.name == "nt":
        import ctypes
        ES_CONTINUOUS, ES_SYSTEM_REQUIRED = 0x80000000, 0x00000001
        ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)


def champion_dir():
    d = STATE_DIR / "champion"
    d.mkdir(parents=True, exist_ok=True)
    return d


# ---------------------------------------------------------------- walker
def walker_round(state, k):
    champ = state["walker"]
    name = f"round_w{k:04d}"
    run = RUNS / name
    run.mkdir(parents=True, exist_ok=True)
    # Train on from the frontier (where the last round ended), so new skills like stairs can build up
    # over several rounds even while they cost speed; fall back to the champion after a dry spell.
    front = state.get("frontier")
    if front is None or state.get("dry_rounds", 0) >= RESET_AFTER:
        front, state["dry_rounds"] = {"zip": champ["zip"], "vecnorm": champ["vecnorm"], "from": champ["from"]}, 0
    shutil.copy(front["zip"], run / "start.zip")
    shutil.copy(front["vecnorm"], run / "vecnormalize.pkl")
    log(f"round {k}: walker training {WALK_STEPS / 1e6:.0f}M steps from {front['from']} ({name})")
    with open(RUNS / f"{name}_train.log", "w") as out:
        train = subprocess.Popen([PY, "train.py", "--name", name, "--resume", str(run / "start.zip"),
                                  "--steps", str(WALK_STEPS), "--lr", str(WALK_LR), "--target-kl", str(WALK_KL),
                                  "--penalty-start", "1.0", "--seed", str(k)], cwd=HERE, stdout=out, stderr=subprocess.STDOUT)
    time.sleep(5)
    with open(RUNS / f"{name}_best.log", "w") as out:
        watch = subprocess.Popen([PY, "keep_best.py", str(run), "--baseline", champ["npz"]], cwd=HERE,
                                 stdout=out, stderr=subprocess.STDOUT)
    code = train.wait()
    try:
        watch.wait(timeout=3600)
    except subprocess.TimeoutExpired:
        watch.kill()
    if code != 0:
        log(f"round {k}: walker training exited with {code}; see runs/{name}_train.log")
        return
    best_md = (run / "best.md").read_text() if (run / "best.md").exists() else ""
    winners = re.findall(r"^\| (\d+)M \|.*\*\*new best\*\*", best_md, re.M)
    d = champion_dir()
    shutil.copy(run / "latest.zip", d / "frontier.zip")
    shutil.copy(run / "vecnormalize.pkl", d / "frontier_vecnormalize.pkl")
    state["frontier"] = {"zip": str(d / "frontier.zip"), "vecnorm": str(d / "frontier_vecnormalize.pkl"),
                         "from": f"{name} (end)"}
    if not winners:
        state["dry_rounds"] = state.get("dry_rounds", 0) + 1
        log(f"round {k}: no snapshot beat the champion (scores in runs/{name}/best.md); "
            f"{state['dry_rounds']} round(s) without one")
    else:
        n = winners[-1]
        state["dry_rounds"] = 0
        snap = run / "progress" / f"policy_{n}M"
        for suffix, key in ((".zip", "zip"), (".npz", "npz")):
            shutil.copy(snap.with_suffix(suffix), d / f"walker{suffix}")
        shutil.copy(snap.with_name(f"policy_vecnormalize_{n}M.pkl"), d / "walker_vecnormalize.pkl")
        score = re.search(rf"^\| {n}M \|.*\| ([\d.]+) \| \*\*new best", best_md, re.M)
        state["walker"] = {"zip": str(d / "walker.zip"), "vecnorm": str(d / "walker_vecnormalize.pkl"),
                           "npz": str(d / "walker.npz"), "from": f"{name} {n}M",
                           "score": float(score.group(1)) if score else None}
        log(f"round {k}: new walker champion {name} {n}M (score {state['walker']['score']})")
        install(d / "walker.npz", POLICIES / "rocky_walk.npz", "walker")
    prune(run)


def prune(run):
    """Delete a finished round's bulk, keep its tables and logs."""
    for pattern in ("progress/*.mp4", "progress/*.zip", "progress/*.pkl", "progress/policy_*.npz",
                    "checkpoints", "latest.zip", "start.zip", "best*.npz", "nav_*.npz", "nav_*.zip", "nav_*.pkl"):
        for f in run.glob(pattern):
            shutil.rmtree(f) if f.is_dir() else f.unlink()


# ---------------------------------------------------------------- navigator
def nav_score(npz):
    out = subprocess.run([PY, "nav_eval.py", str(npz), "--obstacles", "--hybrid", "--runs", str(NAV_EVAL_RUNS)],
                         cwd=HERE, capture_output=True, text=True).stdout
    m = re.search(r"([\d.]+) goals/min.*time to find mean ([\d.]+|nan) s.*falls (\d+)/\d+.*bumps ([\d.]+)/min", out)
    if not m:
        return None, out.strip()[-300:]
    goals, find, falls, bumps = float(m.group(1)), float(m.group(2)), int(m.group(3)), float(m.group(4))
    find = 30.0 if find != find else find   # nan: never saw a goal
    return (goals - 2.0 * bumps - 0.5 * falls - 0.1 * find,   # 10 s to find a goal costs one goal/min
            f"{goals} goals/min, finds in {find:.1f} s, {bumps} bumps/min, {falls} falls")


def nav_round(state, k):
    champ = state["nav"]
    name = f"round_n{k:04d}"
    run = RUNS / name
    run.mkdir(parents=True, exist_ok=True)
    shutil.copy(champ["zip"], run / "start.zip")
    shutil.copy(champ["vecnorm"], run / "vecnormalize.pkl")
    log(f"round {k}: navigator training {NAV_STEPS / 1e6:.1f}M steps ({name})")
    with open(RUNS / f"{name}_train.log", "w") as out:
        code = subprocess.run([PY, "nav_train.py", "--name", name, "--resume", str(run / "start.zip"), "--obstacles",
                               "--steps", str(NAV_STEPS), "--walker", str(POLICIES / "rocky_walk.npz")],
                              cwd=HERE, stdout=out, stderr=subprocess.STDOUT).returncode
    if code != 0:
        log(f"round {k}: navigator training exited with {code}; see runs/{name}_train.log")
        return
    time.sleep(120)   # let the last background nav_eval finish
    # the walker may have changed since the champion was scored: score it again on the same footing
    best, best_note = nav_score(champ["npz"])
    best_snap = None
    for snap in sorted(run.glob("nav_*M.npz")):
        s, note = nav_score(snap)
        with open(run / "scores.md", "a") as f:
            f.write(f"- {snap.name}: {note} -> {s}\n")
        if s is not None and (best is None or s > best):
            best, best_note, best_snap = s, note, snap
    if best_snap is None:
        log(f"round {k}: no navigator snapshot beat the champion ({best_note})")
    else:
        d = champion_dir()
        shutil.copy(best_snap, d / "nav.npz")
        shutil.copy(best_snap.with_suffix(".zip"), d / "nav.zip")
        shutil.copy(best_snap.with_name(best_snap.stem + "_vecnormalize.pkl"), d / "nav_vecnormalize.pkl")
        state["nav"] = {"zip": str(d / "nav.zip"), "vecnorm": str(d / "nav_vecnormalize.pkl"),
                        "npz": str(d / "nav.npz"), "from": f"{name} {best_snap.stem}", "score": best}
        log(f"round {k}: new navigator champion {best_snap.name} ({best_note})")
        install(d / "nav.npz", POLICIES / "rocky_nav_lidar.npz", "navigator")
    prune(run)


# ---------------------------------------------------------------- self-righting
def recover_score(npz):
    from recover_eval import evaluate
    r = evaluate(npz)
    t = r["time"] if r["time"] == r["time"] else 8.0   # nan: never got up
    # getting up matters most, then doing it smoothly (no flinging, no slams), then how fast, then
    # sparing the lidar
    return (10 * r["success"] - 0.5 * r["spin_max"] - r["impact_max"] / 100 - 0.5 * t
            - 0.5 * r["lidar_s"] - r["lidar_peak"] / 50), r


def best_recover_snapshot(run, champion_npz=None):
    best, best_r, best_snap = (None, None, None)
    if champion_npz:
        best, best_r = recover_score(champion_npz)
    for snap in sorted((run / "progress").glob("policy_*M.npz")):
        s, r = recover_score(snap)
        with open(run / "scores.md", "a") as f:
            f.write(f"- {snap.name}: {r} -> {s:.2f}\n")
        if best is None or s > best:
            best, best_r, best_snap = s, r, snap
    return best_snap, best, best_r


def promote_recover(state, snap, score, r, source):
    d = champion_dir()
    n = re.findall(r"(\d+)M", snap.name)[-1]
    shutil.copy(snap, d / "recover.npz")
    shutil.copy(snap.with_suffix(".zip"), d / "recover.zip")
    shutil.copy(snap.with_name(f"policy_vecnormalize_{n}M.pkl"), d / "recover_vecnormalize.pkl")
    state["recover"] = {"zip": str(d / "recover.zip"), "vecnorm": str(d / "recover_vecnormalize.pkl"),
                        "npz": str(d / "recover.npz"), "from": source, "score": score}
    log(f"new self-righting champion {source}: recovered {r['success']:.0%}, {r['time']:.1f} s, "
        f"lidar {r['lidar_s']:.1f} s/fall, peak {r['lidar_peak']:.0f} N")
    install(d / "recover.npz", POLICIES / "rocky_recover.npz", "self-righting")


def recover_round(state, k):
    champ = state["recover"]
    name = f"round_r{k:04d}"
    run = RUNS / name
    run.mkdir(parents=True, exist_ok=True)
    shutil.copy(champ["zip"], run / "start.zip")
    shutil.copy(champ["vecnorm"], run / "vecnormalize.pkl")
    log(f"round {k}: self-righting training {RECOVER_STEPS / 1e6:.0f}M steps ({name})")
    with open(RUNS / f"{name}_train.log", "w") as out:
        code = subprocess.run([PY, "train.py", "--task", "recover", "--name", name, "--resume", str(run / "start.zip"),
                               "--steps", str(RECOVER_STEPS), "--lr", "1e-4", "--target-kl", "0.01",
                               "--penalty-start", "1.0", "--seed", str(k)],
                              cwd=HERE, stdout=out, stderr=subprocess.STDOUT).returncode
    if code != 0:
        log(f"round {k}: self-righting training exited with {code}; see runs/{name}_train.log")
        return
    snap, score, r = best_recover_snapshot(run, champ["npz"])
    if snap is None:
        log(f"round {k}: no self-righting snapshot beat the champion ({r})")
    else:
        promote_recover(state, snap, score, r, f"{name} {snap.stem}")
    prune(run)


# ---------------------------------------------------------------- setup
def seed_state(state):
    d = champion_dir()
    state.setdefault("recover", None)
    if state["recover"] is None and (RUNS / "recover" / "progress").exists():
        snap, score, r = best_recover_snapshot(RUNS / "recover")
        if snap is not None:
            promote_recover(state, snap, score, r, f"recover {snap.stem}")
    if state["walker"] is None:
        src = RUNS / "walk_turn2"   # the walker installed when this loop was written (79M, its last snapshot)
        shutil.copy(src / "latest.zip", d / "walker.zip")
        shutil.copy(src / "vecnormalize.pkl", d / "walker_vecnormalize.pkl")
        shutil.copy(POLICIES / "rocky_walk.npz", d / "walker.npz")
        state["walker"] = {"zip": str(d / "walker.zip"), "vecnorm": str(d / "walker_vecnormalize.pkl"),
                           "npz": str(d / "walker.npz"), "from": "walk_turn2 79M", "score": None}
    if state["nav"] is None and (RUNS / "nav_lidar" / "latest.zip").exists():
        src = RUNS / "nav_lidar"
        shutil.copy(src / "latest.zip", d / "nav.zip")
        shutil.copy(src / "vecnormalize.pkl", d / "nav_vecnormalize.pkl")
        shutil.copy(src / "policy.npz", d / "nav.npz")
        state["nav"] = {"zip": str(d / "nav.zip"), "vecnorm": str(d / "nav_vecnormalize.pkl"),
                        "npz": str(d / "nav.npz"), "from": "nav_lidar", "score": None}
        if not (POLICIES / "rocky_nav_lidar.npz").exists():
            install(d / "nav.npz", POLICIES / "rocky_nav_lidar.npz", "navigator")


def take_lock():
    """Hold an exclusive lock for as long as this process lives: only one session at a time.
    Returns None if another session has it."""
    f = open(STATE_DIR / "lock", "w")
    try:
        if os.name == "nt":
            import msvcrt
            msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        return None
    return f


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--hours", type=float, default=2.0, help="start rounds for this long, then stop (default 2)")
    p.add_argument("--rounds", type=int, help="run this many rounds instead")
    p.add_argument("--tasks", default="walker,nav,recover",
                   help="what to train, comma-separated: walker (incl. stairs), nav (trained lidar navigator), recover")
    args = p.parse_args()
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    lock = take_lock()
    if lock is None:
        print("training is already running", flush=True)
        return
    STOP.unlink(missing_ok=True)
    keep_awake()
    budget = f"{args.rounds} round(s)" if args.rounds else f"{args.hours:g} hour(s)"
    log(f"training session started: {budget}")
    end = time.time() + args.hours * 3600
    left = args.rounds or 0
    tasks = set(args.tasks.split(","))
    while other_training():
        log("waiting for another training job to finish first")
        time.sleep(30)
    state = load_state()
    seed_state(state)
    save_state(state)
    while not STOP.exists() and (left > 0 if args.rounds else time.time() < end):
        left -= 1 if args.rounds else 0
        wait_for_disk()
        state["round"] += 1
        k = state["round"]
        try:
            if "walker" in tasks:
                walker_round(state, k)
                save_state(state)
            if "nav" in tasks and state["nav"] is not None and k % NAV_EVERY == 0 and not STOP.exists():
                nav_round(state, k)
                save_state(state)
            if "recover" in tasks and state.get("recover") is not None and k % RECOVER_EVERY == 0 and not STOP.exists():
                recover_round(state, k)
                save_state(state)
        except Exception as e:   # keep going: a bad round must not end the session
            log(f"round {k} failed: {e!r}")
            time.sleep(300)
    log("training session finished" + (" (stopped early)" if STOP.exists() else ""))
    STOP.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
