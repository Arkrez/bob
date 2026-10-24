#!/usr/bin/env bash
# Webots demos of Bob (rl/demo.py): starts the sim in planner mode if it isn't running, then runs them.
#   scripts/demo.sh             all demos (~5 min): tour, stairs, flip, walls
#   scripts/demo.sh stairs      just one
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! docker ps --format '{{.Names}}' | grep -qx bob-sim || ! docker logs bob-sim 2>&1 | tail -50 | grep -q "navigation mode: planner"; then
    # a running Webots keeps whatever world it has open, so restart it on the obstacle world
    "$REPO/scripts/start_sim.sh" --stop >/dev/null 2>&1 || true
    pkill -x webots 2>/dev/null || true; sleep 3; pkill -9 -x webots 2>/dev/null || true
    "$REPO/scripts/start_sim.sh" --world rocky --detach --nav planner | grep -E "Bridge|Navigation|Lidar"
    sleep 8
fi
exec "$REPO/scripts/sim_ros.sh" python3 /workspace/rl/demo.py "$@"
