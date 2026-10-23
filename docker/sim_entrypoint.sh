#!/usr/bin/env bash
# Runs inside the bob-sim container: build the ROS packages, then start the
# quadruped controller (servo bridge off) and the Webots extern-controller bridge.
set -eo pipefail

source /opt/ros/humble/setup.bash
cd /workspace

echo "[bob-sim] building bob_description + quadruped_control"
colcon --log-base /colcon/log build --symlink-install \
    --packages-select bob_description quadruped_control \
    --build-base /colcon/build --install-base /colcon/install \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source /colcon/install/setup.bash

echo "[bob-sim] starting quadruped controller (enable_servo_bridge:=false)"
ros2 run quadruped_control quadruped --ros-args -p enable_servo_bridge:=false &
controller_pid=$!

# webots-controller exits if Webots isn't reachable yet, so keep retrying.
bridge() {
    while true; do
        echo "[bob-sim] connecting to Webots at ${WEBOTS_HOST}:${WEBOTS_PORT} as robot '${WEBOTS_ROBOT}'"
        webots-controller --protocol=tcp --ip-address="${WEBOTS_HOST}" --port="${WEBOTS_PORT}" \
            --robot-name="${WEBOTS_ROBOT}" /workspace/webots_quadruped/controllers/bob_ros2_external.py || true
        echo "[bob-sim] Webots bridge exited; retrying in 3 s (is the simulation running?)"
        sleep 3
    done
}
bridge &
bridge_pid=$!

trap 'kill "$controller_pid" "$bridge_pid" 2>/dev/null; exit 0' INT TERM
wait -n "$controller_pid" "$bridge_pid"
echo "[bob-sim] a process exited; shutting down"
kill "$controller_pid" "$bridge_pid" 2>/dev/null || true
exit 1
