#!/usr/bin/env bash
# Start the Bob simulation: Webots on this Mac + ROS 2 (controller and bridge) in Docker.
#
#   scripts/start_sim.sh                 Rocky-Bob world, follow logs (Ctrl-C stops the container)
#   scripts/start_sim.sh --world cad     the older CAD-derived Bob world
#   scripts/start_sim.sh --detach        start everything and return
#   scripts/start_sim.sh --demo          also run the joint-motion smoke test
#   scripts/start_sim.sh --policy rl/policies/rocky_walk.npz   walk with a trained policy (drive it with /cmd_vel)
#   scripts/start_sim.sh --nav planner           walk to the orange target marker (drag it in Webots), finding
#                                                the way round walls with the lidar (rl/nav_spec.PlannerNavigator);
#   scripts/start_sim.sh --nav scripted          the simpler scripted navigator;
#                                                or --nav rl/policies/rocky_nav.npz for the trained navigator
#   scripts/start_sim.sh --nav scripted --follow-training rl/runs/walk_turn2
#                                        walk with that run's best snapshot so far (rl/keep_best.py), swapping
#                                        in each new best live while it trains
#   scripts/start_sim.sh --rebuild       rebuild the Docker image first
# The lidar viewer (rl/scan_viewer.py) always starts too: http://localhost:8765
#   scripts/start_sim.sh --stop          stop the container (quit Webots yourself)
#
# Needs Docker Desktop and Webots R2025a (/Applications/Webots.app, or set WEBOTS_APP).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE=(docker compose -f "$REPO/docker/compose.yaml")
WEBOTS_APP="${WEBOTS_APP:-/Applications/Webots.app}"
WEBOTS_PORT=1234
LOG_DIR="${TMPDIR:-/tmp}/bob-sim"

world=rocky detach=0 demo=0 rebuild=0 policy="" nav=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --world) world="$2"; shift 2 ;;
        --detach) detach=1; shift ;;
        --demo) demo=1; shift ;;
        --policy) policy="$2"; shift 2 ;;
        --follow-training) policy="${2%/}/best.npz"; shift 2 ;;
        --nav) nav="$2"; shift 2 ;;
        --rebuild) rebuild=1; shift ;;
        --stop) "${COMPOSE[@]}" down; echo "Container stopped. Quit Webots from its window."; exit 0 ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
done

case "$world" in
    rocky) world_file="$REPO/cad/rocky_bob/sim/worlds/rocky_bob.wbt" ;;
    cad) world_file="$REPO/webots_quadruped/worlds/bob.wbt" ;;
    *.wbt) world_file="$(cd "$(dirname "$world")" && pwd)/$(basename "$world")" ;;
    *) echo "--world must be rocky, cad or a path to a .wbt file" >&2; exit 2 ;;
esac
[[ -f "$world_file" ]] || { echo "World not found: $world_file" >&2; exit 1; }
[[ -d "$WEBOTS_APP" ]] || { echo "Webots not found at $WEBOTS_APP. Install R2025a from https://cyberbotics.com or set WEBOTS_APP." >&2; exit 1; }
mkdir -p "$LOG_DIR"

# 1. Docker
if ! docker info >/dev/null 2>&1; then
    echo "Starting Docker Desktop..."
    open -a Docker
    for _ in $(seq 1 90); do docker info >/dev/null 2>&1 && break; sleep 2; done
    docker info >/dev/null 2>&1 || { echo "Docker did not start within 3 minutes." >&2; exit 1; }
fi

# 2. Webots
if pgrep -x webots >/dev/null; then
    echo "Webots is already running; make sure it has $(basename "$world_file") open and is not paused."
else
    echo "Opening Webots with $(basename "$world_file")"
    "$WEBOTS_APP/Contents/MacOS/webots" --port="$WEBOTS_PORT" --mode=realtime "$world_file" \
        >"$LOG_DIR/webots.log" 2>&1 &
fi

# 3. ROS 2 container (first run builds the image and the workspace; later runs are quick).
# With --policy the bridge runs the policy itself, in lockstep with sim time (see the bridge's
# docstring); compose recreates the container whenever BOB_POLICY changes.
in_container() {   # repo path -> path inside the container
    local f="$1"
    [[ -d "$f" ]] && f="$f/policy.npz"
    f="$(cd "$(dirname "$f")" && pwd)/$(basename "$f")"
    [[ -f "$f" && "$f" == "$REPO"/* ]] || { echo "Policy not found inside the repo: $1" >&2; exit 1; }
    echo "/workspace/${f#"$REPO"/}"
}
[[ -n "$nav" && -z "$policy" ]] && policy="$REPO/rl/policies/rocky_walk.npz"
export BOB_POLICY="" BOB_NAV=""
[[ -n "$policy" ]] && BOB_POLICY="$(in_container "$policy")"
if [[ "$nav" == "scripted" || "$nav" == "planner" ]]; then BOB_NAV="$nav"; elif [[ -n "$nav" ]]; then BOB_NAV="$(in_container "$nav")"; fi
since="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
build_flag=()
[[ $rebuild == 1 ]] && build_flag=(--build)
"${COMPOSE[@]}" up -d ${build_flag[@]+"${build_flag[@]}"}   # macOS bash 3.2-safe empty array

echo "Waiting for the bridge to connect to Webots..."
connected=0
for _ in $(seq 1 150); do
    if docker logs --since "$since" bob-sim 2>&1 | grep -q "Bob Webots bridge ready"; then connected=1; break; fi
    sleep 2
done
if [[ $connected == 1 ]]; then
    echo "Bridge connected: 12 motors live."
else
    echo "Bridge has not connected yet. Check 'docker logs bob-sim' and that the simulation is running." >&2
fi

if [[ $demo == 1 ]]; then
    "$REPO/scripts/sim_ros.sh" python3 /workspace/webots_quadruped/scripts/joint_motion_demo.py \
        >"$LOG_DIR/demo.log" 2>&1 &
    echo "Joint-motion demo running (log: $LOG_DIR/demo.log)"
fi

docker exec -d bob-sim bash -c 'source /opt/ros/humble/setup.bash && exec python3 /workspace/rl/scan_viewer.py \
    > /tmp/scan_viewer.log 2>&1'
echo "Lidar point cloud: http://localhost:8765 (Webots: View > Optional Rendering > Show Lidar Point Cloud)"

if [[ -n "$BOB_NAV" ]]; then
    echo "Navigation running in the bridge: $BOB_NAV (walker $BOB_POLICY)"
    echo "  drag the orange target post in Webots, or:"
    echo "  scripts/sim_ros.sh ros2 topic pub --once /goal geometry_msgs/msg/PointStamped \"{point: {x: -1.0, y: 0.5}}\""
    echo "  status:    scripts/sim_ros.sh ros2 topic echo /nav/status"
elif [[ -n "$BOB_POLICY" ]]; then
    echo "Walking policy running in the bridge: $BOB_POLICY"
    echo "  drive it:  scripts/sim_ros.sh ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.15}}\""
    echo "  measure:   scripts/sim_ros.sh python3 /workspace/rl/webots_eval.py"
fi

cat <<EOF

Simulation is up. Try:
  scripts/sim_ros.sh ros2 topic echo /joint_states --once
  scripts/sim_ros.sh ros2 topic pub --once /set_angles std_msgs/msg/String "{data: '0,0,10,0'}"
  scripts/sim_ros.sh python3 /workspace/webots_quadruped/scripts/joint_motion_demo.py
Stop with: scripts/start_sim.sh --stop
EOF

if [[ $detach == 0 ]]; then
    trap '"${COMPOSE[@]}" stop >/dev/null; echo; echo "Container stopped. Quit Webots from its window."' EXIT
    "${COMPOSE[@]}" logs -f sim
fi
