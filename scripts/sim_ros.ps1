# Run a command inside the running bob-sim container with ROS 2 sourced (twin of sim_ros.sh).
#   scripts\sim_ros.ps1 ros2 topic list
#   scripts\sim_ros.ps1 python3 /workspace/rl/stairs_test.py east
if ($args.Count -eq 0) {
    docker exec -it bob-sim bash -c "source /opt/ros/humble/setup.bash && source /colcon/install/setup.bash && exec bash"
} else {
    docker exec bob-sim bash -c 'source /opt/ros/humble/setup.bash && source /colcon/install/setup.bash && exec "$@"' bash @args
}
