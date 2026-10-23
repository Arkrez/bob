#!/usr/bin/env bash
# Run a command inside the running bob-sim container with ROS 2 sourced.
#   scripts/sim_ros.sh ros2 topic list
#   scripts/sim_ros.sh            (interactive shell)
set -euo pipefail

tty_flag=()
[[ -t 0 && -t 1 ]] && tty_flag=(-it)
if [[ $# -eq 0 ]]; then
    exec docker exec -it bob-sim bash -c 'source /opt/ros/humble/setup.bash && source /colcon/install/setup.bash && exec bash'
fi
exec docker exec ${tty_flag[@]+"${tty_flag[@]}"} bob-sim bash -c \
    'source /opt/ros/humble/setup.bash && source /colcon/install/setup.bash && exec "$@"' bash "$@"
