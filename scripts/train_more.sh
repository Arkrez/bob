#!/usr/bin/env bash
# Train Bob some more, then stop (rl/train_rounds.py). Each session carries on from the last.
#
#   scripts/train_more.sh                 2 hours of rounds in the background, then stop
#   scripts/train_more.sh --hours 6       or as long as you like (overnight, say)
#   scripts/train_more.sh --rounds 3      or a number of rounds (walker ~30 min each)
#   scripts/train_more.sh --hours 6 --tasks walker,recover   only some skills (walker, nav, recover)
#   scripts/train_more.sh --status        what it's doing, the champions so far, recent log
#   scripts/train_more.sh --stop          finish the current round, then stop early
#
# The Mac is kept awake only while a session runs (caffeinate). New champions are installed into
# rl/policies/ as they win, and a running sim picks them up by itself.
set -euo pipefail
RL="$(cd "$(dirname "${BASH_SOURCE[0]}")/../rl" && pwd)"
mkdir -p "$RL/runs/rounds"
running() { pgrep -f "python.*train_rounds.py" >/dev/null; }

case "${1:-}" in
    --status)
        if running; then echo "training: running"; else echo "training: not running"; fi
        [[ -f "$RL/runs/rounds/state.json" ]] && cat "$RL/runs/rounds/state.json"
        echo; tail -n 15 "$RL/runs/rounds/log.md" 2>/dev/null || true
        ;;
    --stop)
        touch "$RL/runs/rounds/STOP"
        echo "Will stop after the current round."
        ;;
    ""|--hours|--rounds|--tasks)
        if running; then echo "Already training (--status to see it)"; exit 0; fi
        cd "$RL"
        nohup caffeinate -is .venv/bin/python train_rounds.py "$@" >> runs/rounds/console.log 2>&1 &
        echo "Training started (pid $!), budget: ${*:---hours 2}. Log: rl/runs/rounds/log.md"
        ;;
    *) echo "usage: $0 [--hours H | --rounds N | --status | --stop]" >&2; exit 2 ;;
esac
