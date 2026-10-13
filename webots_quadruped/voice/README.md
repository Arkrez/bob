# Bob voice-controlled Webots simulation (experimental)

This runs a **local voice/text command interface** for the existing ROS 2
`/leg_position_controller/commands` topic. The interface uses the browser's
SpeechRecognition implementation (when supported) and a Python `rclpy`
publisher. **It does not connect this ChatGPT conversation to your Mac.**
Commands are rule-based, not LLM-generated. It does not launch Webots or
control the physical Servo2040.

## Start

1. On macOS, open `webots_quadruped/worlds/bob.wbt` in Webots, run the
   simulation, and start the **external Webots ROS 2 controller** according to
   the main README. Confirm that
   `ros2 topic info /leg_position_controller/commands` shows a subscriber.
   Do not start `servo2040_bridge_node` or any hardware servo bridge.
2. In the same ROS 2 Humble Docker container where `rclpy` and
   `std_msgs` are available, run:

   ```bash
   source /opt/ros/humble/setup.bash
   python3 /workspace/webots_quadruped/voice/voice_sim_bridge.py --host 0.0.0.0
   ```

   **Only bind 0.0.0.0 inside a trusted, locally forwarded Docker container.**
   The recommended Docker run configuration publishes **only localhost**:
   `-p 127.0.0.1:8765:8765`. If you use host networking on Linux, keep
   `--host 127.0.0.1`. On Docker Desktop for Mac,
   publish the port when creating the container; Docker port mappings
   cannot be added to an existing container.
3. On the Mac, open `http://localhost:8765` in a compatible Chrome browser,
   allow microphone permission, and speak:
   **“set leg 0 joint 3 to 15 degrees”**. Or type it in the input box.
   Commands `reset` and `stop` are also supported.

Leg numbers are 0–3. Joints are 1, 3, 4, corresponding to the existing
`joint1_legN`, `joint3_legN`, and `joint4_legN` convention.
Targets are limited to ±45° (rather than unverified placeholder joint
limits). `stop` holds the last commanded target; it does **not** stop the
simulator or guarantee a motor is motionless.

**No walking, natural-language IK, body height adjustment, obstacle avoidance
or autonomous AI agent is implemented yet.** The robot model and joint
mapping require validation in Webots before extending the command set.
SpeechRecognition support varies by browser and may rely on a remote
recognition service. The interface uses local HTTP and is not authenticated;
do not expose it on a network or the internet.
