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
