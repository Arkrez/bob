# Bob — ROS 2 / Webots quadruped

Bob is a motorcycle-chain maintenance robot in development, using a Raspberry Pi 5, a Pimoroni Servo 2040, MG995 servos, and ROS 2 Humble. This repository contains the controller software and the **current CAD-derived Bob model**.

## Current robot model

- `webots_quadruped/protos/Bob.proto`: Webots PROTO; the model references `../meshes/*.stl` relative to the PROTO.
- `webots_quadruped/meshes/`: 17 meshes for the current Bob chassis and four legs.
- `webots_quadruped/worlds/bob.wbt`: world that instantiates `Bob` (not an older robot definition).
- `src/bob_description/urdf/robot.xacro`: maintainable ROS 2 description with reusable leg macro.
- `src/bob_description/urdf/robot.urdf`: expanded URDF, usable without Xacro processing.
- `src/bob_description/meshes/`: same mesh blobs packaged for `package://bob_description/meshes/...` lookup.

### Webots

Open `webots_quadruped/worlds/bob.wbt` in Webots. The `Bob.proto` file uses an external controller (`<extern>`), which must be connected for motor control. Webots resolves the STL mesh URLs relative to `webots_quadruped/protos/Bob.proto`.

### ROS 2

From the repository root inside a ROS 2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bob_description
source install/setup.bash
ros2 pkg prefix bob_description
xacro src/bob_description/urdf/robot.xacro > /tmp/bob.urdf
```

The existing locomotion and servo bridge code remains under `src/quadruped_control/` for compatibility with existing commands. **Its older leg ordering/joint nomenclature and IK axes are not yet validated against the new 12-joint CAD model.** Do not command the physical robot using these new model coordinates until they have been checked.

### Important CAD assumptions

Joint anchors are approximated from STEP geometry; hip joint axes are Z and the two downstream axes are Y. Limits are placeholder ±90 degrees; masses, collision geometry, and motor torques are provisional. The CAD rest pose may not be a standing pose. Confirm motion, collisions, inertia and hardware servo mapping before locomotion tests.

## Project goal

Bob will eventually walk around a motorcycle, visually locate its chain and lubricate it using a carried mechanism. The Raspberry Pi handles high-level control, the Servo 2040 drives the MG995 servos, and ROS 2 coordinates perception, locomotion, and hardware communication.

## ROS 2 -> Webots direct motor bridge (Bob CAD)

The existing ROS 2 controller publishes twelve angles (radians) on
`/leg_position_controller/commands`, with joint order `J1,J3,J4` for
each leg index 0–3. The Webots external bridge subscribes to this topic
and applies positions to twelve `RotationalMotor` devices.
Motor positions are published to `/joint_states`.

The Webots and URDF joint names are `joint1_leg0`, `joint3_leg0`,
`joint4_leg0` through leg 3.

### On a Mac with Webots GUI and ROS 2 Humble in Docker

1. Open `webots_quadruped/worlds/bob.wbt` in Webots and run the simulation.
2. In the ROS 2 container, mount the Bob repo to `/workspace`.
   Ensure `rclpy`, `sensor_msgs`, `std_msgs`, Webots Python controller
   runtime, `orocos-kdl`, `cv_bridge` and `xacro` are available.
3. Build:

   ```bash
   source /opt/ros/humble/setup.bash
   cd /workspace
   colcon build --symlink-install --packages-select bob_description quadruped_control --cmake-args -DBUILD_TESTING=OFF
   source install/setup.bash
   ```

4. Launch the ROS controller, explicitly disabling physical servo writes:

   ```bash
   ros2 run quadruped_control quadruped --ros-args -p enable_servo_bridge:=false
   ```

5. In another ROS 2 terminal (inside Docker), start the Webots external
   controller via the installed `webots-controller` launcher:

   ```bash
   webots-controller --protocol=tcp --ip-address=host.docker.internal --port=1234 --robot-name=Bob /workspace/webots_quadruped/controllers/bob_ros2_external.py
   ```

   Check `which webots-controller` first. Webots controller runtime and a
   Python interpreter able to import both `controller` and `rclpy` are
   required inside the container. If necessary, enable remote controller
   connections in Webots Preferences > Network.

6. Confirm connection and move one joint:

   ```bash
   ros2 topic info /leg_position_controller/commands
   ros2 topic echo /joint_states
   ros2 topic pub --once /set_angles std_msgs/msg/String "{data: '0,0,10,0'}"
   ```

   `/leg_position_controller/commands` should show at least one subscriber
   and `/joint_states` should report twelve positions. You may also send
   a 12-radian array directly:

   ```bash
   ros2 topic pub --once /leg_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0.2,0, 0,0,0, 0,0,0, 0,0,0]}"
   ```

7. After verifying individual joints move, run the small-amplitude
   **motion smoke test** (not a stable gait):

   ```bash
   python3 /workspace/webots_quadruped/scripts/joint_motion_demo.py
   ```

### Calibration and safety

The previous API remains: `/set_angles` expects
`leg,J1,J3,J4` in degrees and `/ik_target` uses inches in the body
frame. The IK chain now uses STEP-derived hip/pivot positions and Y-axis
downstream rotation, but its **foot tip length is still estimated**.
Verify signs, end-effector offsets, gait, inertia and collision contact
before any walking tests. The physical servo bridge is off in the
simulation command above; do not run the demo on physical hardware.

The CAD leg geometry positions are not yet independently matched to the
old logical labels (FL/FR/BL/BR), so confirm leg indexing by moving
one joint at a time. The external bridge deliberately bypasses
`controller_manager` / `ros2_control` while keeping the existing
position-command topic.

### Troubleshooting

If the controller reports no subscriber on
`/leg_position_controller/commands`, the Webots external bridge is not
connected. Confirm `--robot-name=Bob`, the Webots TCP port, Docker host
reachability, and that the controller's Python can import `rclpy` and
`controller`.
