# Bob — Quadruped Robot

Bob is a custom quadruped robotics project built to explore legged locomotion, inverse kinematics, gait generation, ROS 2, simulation, and embedded hardware control.

The project is being developed from the ground up, with the same control concepts intended to work in simulation and on the physical robot.

<img width="1215" height="749" alt="Bob quadruped robot" src="https://github.com/user-attachments/assets/0fdad499-c6f1-402a-a0a8-6c3416fa759d" />

## Project Goals

- Build a fully controllable four-legged robot
- Develop custom inverse kinematics and walking gaits
- Use ROS 2 for communication between robot subsystems
- Test movement safely in Webots before running it on hardware
- Bridge high-level ROS 2 commands to the physical servo controller
- Experiment with camera input and voice-controlled behaviors

## Hardware

The physical robot is designed around:

- Raspberry Pi 5 — high-level compute and ROS 2
- Pimoroni Servo 2040 — low-level servo control
- 16 × MG995 servos — four servos per leg
- Custom quadruped frame and leg geometry
- Camera / vision input
- Audio hardware for voice-control experiments

## Software Stack

- C++
- ROS 2 Humble
- Webots
- CMake / ament_cmake
- OpenCV / cv_bridge
- Orocos KDL
- Cyclone DDS
- whisper.cpp for speech-recognition experiments

## Architecture

The main ROS 2 executable starts several nodes inside one process:

- `QuadrupedControllerNode` — robot motion, gait, and IK control
- `Servo2040BridgeNode` — bridge between ROS 2 and the physical Servo 2040 controller
- `CameraDisplayNode` — camera-image handling and display

The control package is located at:

```text
src/quadruped_control/
```

The Webots simulation is located at:

```text
webots_quadruped/
```

## Repository Layout

```text
bob/
├── src/
│   ├── quadruped_control/     # Main ROS 2 C++ control package
│   └── quadruped_sim/         # Gazebo / simulation experiments
├── webots_quadruped/
│   ├── protos/                # Webots robot PROTO files
│   └── worlds/
│       └── quadruped.wbt      # Main Webots world
├── voice_control/             # Voice-control experiments
├── Quadruped.proto            # Generated / exported Webots robot model
├── quadruped_webots.urdf      # Robot description used with Webots
├── cyclonedds.xml             # ROS 2 DDS configuration
└── README.md
```

## Building the ROS 2 Workspace

From the repository root inside a ROS 2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The project uses external dependencies including ROS 2 packages, OpenCV, Orocos KDL, audio messages, and `whisper.cpp`.

## Running the Controller

After building and sourcing the workspace:

```bash
ros2 run quadruped_control quadruped
```

This starts the quadruped controller, Servo 2040 bridge, and camera node.

## Webots Simulation

The current Webots world is:

```text
webots_quadruped/worlds/quadruped.wbt
```

The simulation is used to test joint motion, inverse kinematics, gait logic, and ROS 2 integration before sending equivalent commands to the physical robot.

## Motion Control

Bob's motion stack is being developed around custom leg kinematics and gait generation.

Current work includes:

- Joint-angle control
- Per-leg inverse kinematics
- Coordinate transforms between robot and leg frames
- Servo-direction and angle mapping
- Multi-leg gait sequencing
- Simulation-to-hardware control flow

## Current Status

Bob is an active work-in-progress. The project currently contains working ROS 2 control infrastructure, a Servo 2040 hardware bridge, a Webots robot/world, camera integration, and ongoing locomotion / gait development.

The immediate focus is making simulated walking reliable and then transferring the same motion logic to the physical quadruped.
